/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/*
 * The link's DATA channel (issue #23 U1).  See link_data.h for the wire format, the
 * ownership rules and the detach ordering rule.
 *
 * Register-agnostic and clock-agnostic (clock-safe): pools and queues only.  The UART is
 * never touched here -- app/erpc.c's service thread remains the single owner of it, and
 * this file only hands it frames to write and takes frames it has read.
 *
 * LOCKING.  All shared state (both pools, both queues, the counters) is guarded by a
 * short interrupt-masked region -- the same idiom app/net_shell.c uses for its transmit
 * ring.  A mutex would be wrong here for two reasons: link_data_send() must be callable
 * from a context that already holds a network stack's lock without inverting anything
 * (U3), and it must never block.  Every region below is a handful of instructions with
 * no loop over frame data, so the interrupt latency cost is bounded and tiny next to the
 * UART's own grace (see app/rtl8720.c).  The payload COPY is deliberately outside it.
 */
#include "tx_api.h"       /* tx_event_flags_set via erpc_data_posted() */
#include "stm32h7xx_hal.h"/* __get_PRIMASK / __disable_irq / __set_PRIMASK */
#include "erpc.h"         /* erpc_crc16, erpc_data_posted */
#include "link_data.h"

#include <string.h>

/*
 * One pool entry.  @body holds exactly what goes on the wire after the 6-byte frame
 * header, so a receive fills it straight from the UART ring and a transmit is written
 * out of it without another copy.  aligned(8) makes the payload land at +2, which puts
 * an Ethernet frame's IP header on a 4-byte boundary (see link_data.h).
 */
struct link_data_buf {
	uint8_t  body[LINK_DATA_BODY_MAX];
	uint16_t len;                            /* body length, including the 2-byte hdr */
	uint16_t crc;
} __attribute__((aligned(8)));

static struct link_data_buf ld_rx_pool[LINK_DATA_RX_BUFS];
static struct link_data_buf ld_tx_pool[LINK_DATA_TX_BUFS];

/* Index rings.  Capacity is the pool size + 1 so "full" and "empty" stay distinct. */
static uint8_t  ld_rx_free[LINK_DATA_RX_BUFS + 1u], ld_rx_free_n;
static uint8_t  ld_rx_done[LINK_DATA_RX_BUFS + 1u], ld_rx_done_head, ld_rx_done_tail;
static uint8_t  ld_tx_free[LINK_DATA_TX_BUFS + 1u], ld_tx_free_n;
static uint8_t  ld_tx_q[LINK_DATA_TX_BUFS + 1u], ld_tx_head, ld_tx_tail;

/*
 * Per-buffer state plus an epoch, and neither is bookkeeping for its own sake.
 *
 * link_data_reset() runs from a link teardown, and it can land while somebody holds a
 * buffer OUTSIDE the critical section: the frame reader mid-frame, a sender doing its
 * memcpy, or a consumer that kept a received frame.  Two distinct hazards follow, and
 * they need different answers:
 *
 *   - Returning a buffer twice.  The STATE byte answers that: a release is a no-op
 *     unless the slot is actually held.
 *   - Handing a held slot to somebody ELSE.  A state byte alone CANNOT answer that -- if
 *     reset put a BUSY slot back in the free list, a new claimant would take it, mark it
 *     BUSY again, and the old holder's "is it still BUSY?" test would pass while it wrote
 *     into a buffer that now belongs to someone else (a textbook ABA).  So reset does NOT
 *     reclaim held slots: they come back when their holder returns them, which every
 *     holder does.  What reset DOES do is bump the EPOCH, so that holder's frame is
 *     dropped instead of delivered into a link that no longer exists.
 */
enum { LD_FREE = 0, LD_BUSY = 1, LD_QUEUED = 2 };
static uint8_t  ld_rx_state[LINK_DATA_RX_BUFS];
static uint8_t  ld_tx_state[LINK_DATA_TX_BUFS];
static uint8_t  ld_rx_epoch[LINK_DATA_RX_BUFS];
static uint8_t  ld_tx_epoch[LINK_DATA_TX_BUFS];
static uint8_t  ld_epoch;                        /* bumped by link_data_reset() */
/* 8 bits is enough: a holder lives for the microseconds of a memcpy or the milliseconds
 * of one frame arriving, and 256 link teardowns cannot happen inside that. */

static uint8_t  ld_ready;                        /* pools initialised */
static uint8_t  ld_rx_inuse;                     /* claimed or queued RX buffers */

static int    (*ld_rx_cb)(void *ctx, uint8_t chan, uint8_t *p, uint16_t n);
static void    *ld_rx_ctx;

static struct link_data_stats ld_ctr;

#define LD_RX_Q_CAP  (LINK_DATA_RX_BUFS + 1u)
#define LD_TX_Q_CAP  (LINK_DATA_TX_BUFS + 1u)

/* ------------------------------------------------------------------ *
 *  critical section
 * ------------------------------------------------------------------ */
static uint32_t ld_enter(void)
{
	uint32_t primask = __get_PRIMASK();

	__disable_irq();
	return primask;
}

static void ld_exit(uint32_t primask)
{
	__set_PRIMASK(primask);
}

/* Populate the free lists once.  Idempotent, and safe to reach from any thread: it runs
 * inside the caller's critical section. */
static void ld_init_locked(void)
{
	uint8_t i;

	if (ld_ready)
		return;
	ld_rx_free_n = 0u;
	for (i = 0u; i < LINK_DATA_RX_BUFS; i++) {
		ld_rx_state[i] = LD_FREE;
		ld_rx_free[ld_rx_free_n++] = i;
	}
	ld_tx_free_n = 0u;
	for (i = 0u; i < LINK_DATA_TX_BUFS; i++) {
		ld_tx_state[i] = LD_FREE;
		ld_tx_free[ld_tx_free_n++] = i;
	}
	ld_rx_done_head = ld_rx_done_tail = 0u;
	ld_tx_head      = ld_tx_tail      = 0u;
	ld_rx_inuse     = 0u;
	ld_ready        = 1u;
}

/*
 * Empty both pools of everything NOBODY IS HOLDING, and start a new epoch.
 *
 * Queued buffers are ours to take back; held (BUSY) ones are not -- see the ABA argument
 * at the state/epoch declaration.  Their holders return them on their own, and the epoch
 * bump is what makes their frames stale so they are dropped rather than delivered.
 * Caller is inside the critical section.
 */
static void ld_reclaim_locked(void)
{
	unsigned i;

	ld_epoch++;
	ld_rx_done_head = ld_rx_done_tail = 0u;
	ld_tx_head      = ld_tx_tail      = 0u;
	ld_rx_free_n    = 0u;
	ld_rx_inuse     = 0u;
	for (i = 0u; i < LINK_DATA_RX_BUFS; i++) {
		if (ld_rx_state[i] == LD_QUEUED)
			ld_rx_state[i] = LD_FREE;
		if (ld_rx_state[i] == LD_FREE)
			ld_rx_free[ld_rx_free_n++] = (uint8_t)i;
		else
			ld_rx_inuse++;
	}
	ld_tx_free_n = 0u;
	for (i = 0u; i < LINK_DATA_TX_BUFS; i++) {
		if (ld_tx_state[i] == LD_QUEUED)
			ld_tx_state[i] = LD_FREE;
		if (ld_tx_state[i] == LD_FREE)
			ld_tx_free[ld_tx_free_n++] = (uint8_t)i;
	}
	ld_ready = 1u;
}

/* Return one RX buffer to the pool.  A no-op unless the slot is actually held, which is
 * what makes every "give it back" path (abort, free, a failed CRC, a late commit after a
 * reset) safe to call unconditionally.  Caller is inside the critical section. */
static void ld_rx_release_locked(int idx)
{
	if (idx < 0 || ld_rx_state[idx] == LD_FREE)
		return;
	ld_rx_state[idx] = LD_FREE;
	ld_rx_free[ld_rx_free_n++] = (uint8_t)idx;
	if (ld_rx_inuse != 0u)
		ld_rx_inuse--;
}

/* Map a payload pointer back to its pool slot; -1 if it is not one of ours.  Used by
 * link_data_rx_free(), so a consumer can hand back the pointer it was given rather than
 * having to carry an opaque handle around. */
static int ld_rx_index(const uint8_t *payload)
{
	const uint8_t *base = (const uint8_t *)&ld_rx_pool[0];
	size_t off;
	int idx;

	if (payload == NULL || payload < base + LINK_DATA_HDR)
		return -1;
	off = (size_t)(payload - base) - LINK_DATA_HDR;
	if (off % sizeof(ld_rx_pool[0]) != 0u)
		return -1;
	idx = (int)(off / sizeof(ld_rx_pool[0]));
	return (idx < (int)LINK_DATA_RX_BUFS) ? idx : -1;
}

static int ld_rx_index_body(const uint8_t *body)
{
	return (body == NULL) ? -1 : ld_rx_index(body + LINK_DATA_HDR);
}

/* ------------------------------------------------------------------ *
 *  consumer registration
 * ------------------------------------------------------------------ */
int link_data_attach(int (*rx)(void *ctx, uint8_t chan, uint8_t *p, uint16_t n),
                     void *ctx)
{
	uint32_t p;
	int rc = 0;

	if (rx == NULL)
		return -1;
	p = ld_enter();
	ld_init_locked();
	if (ld_rx_cb != NULL) {
		rc = -1;                         /* the channel has exactly one owner */
	} else {
		ld_rx_cb  = rx;
		ld_rx_ctx = ctx;
	}
	ld_exit(p);
	return rc;
}

void link_data_detach(void)
{
	uint32_t p = ld_enter();

	ld_rx_cb  = NULL;
	ld_rx_ctx = NULL;
	ld_exit(p);
	link_data_reset();                       /* nothing left to deliver it to */
}

int link_data_attached(void)
{
	uint32_t p = ld_enter();
	int a = (ld_rx_cb != NULL);

	ld_exit(p);
	return a;
}

/* ------------------------------------------------------------------ *
 *  transmit
 * ------------------------------------------------------------------ */
int link_data_send(uint8_t chan, const uint8_t *p, uint16_t n)
{
	struct link_data_buf *b;
	uint32_t pm;
	uint8_t idx;

	if (n > LINK_DATA_PAYLOAD_MAX || (n != 0u && p == NULL))
		return -1;

	pm = ld_enter();
	ld_init_locked();
	if (ld_tx_free_n == 0u) {
		ld_ctr.tx_drops++;               /* no buffer: drop, as an Ethernet link may */
		ld_exit(pm);
		return -1;
	}
	idx = ld_tx_free[--ld_tx_free_n];
	ld_tx_state[idx] = LD_BUSY;
	ld_tx_epoch[idx] = ld_epoch;
	ld_exit(pm);

	/* Outside the critical section on purpose: the copy and the CRC are the expensive
	 * part (~11 us for 1500 bytes) and the buffer is exclusively ours now. */
	b = &ld_tx_pool[idx];
	b->body[0] = chan;
	b->body[1] = 0u;                         /* flags: reserved, must be zero */
	if (n != 0u)
		memcpy(b->body + LINK_DATA_HDR, p, n);
	b->len = (uint16_t)(LINK_DATA_HDR + n);
	b->crc = erpc_crc16(b->body, b->len);

	pm = ld_enter();
	if (ld_tx_epoch[idx] != ld_epoch) {
		/* link_data_reset() ran while we were copying: this frame belongs to a link
		 * that no longer exists.  The slot is still OURS to give back (reset never
		 * takes a held one), so release it here. */
		ld_tx_state[idx] = LD_FREE;
		ld_tx_free[ld_tx_free_n++] = (uint8_t)idx;
		ld_ctr.tx_drops++;
		ld_exit(pm);
		return -1;
	}
	ld_tx_state[idx] = LD_QUEUED;
	ld_tx_q[ld_tx_head] = idx;
	ld_tx_head = (uint8_t)((ld_tx_head + 1u) % LD_TX_Q_CAP);
	ld_ctr.tx_frames++;
	ld_ctr.tx_bytes += n;
	ld_exit(pm);

	/* The service thread parks indefinitely when it believes nothing is outstanding,
	 * so a queued frame MUST wake it (app/erpc.c erpc_work_pending). */
	erpc_data_posted();
	return 0;
}

const uint8_t *link_data_tx_peek(uint16_t *body_len, uint16_t *crc)
{
	const struct link_data_buf *b = NULL;
	uint32_t p = ld_enter();

	if (ld_tx_head != ld_tx_tail) {
		b = &ld_tx_pool[ld_tx_q[ld_tx_tail]];
		if (body_len)
			*body_len = b->len;
		if (crc)
			*crc = b->crc;
	}
	ld_exit(p);
	return (b != NULL) ? b->body : NULL;
}

void link_data_tx_pop(void)
{
	uint32_t p = ld_enter();

	if (ld_tx_head != ld_tx_tail) {
		uint8_t idx = ld_tx_q[ld_tx_tail];

		ld_tx_tail = (uint8_t)((ld_tx_tail + 1u) % LD_TX_Q_CAP);
		if (ld_tx_state[idx] != LD_FREE) {
			ld_tx_state[idx] = LD_FREE;
			ld_tx_free[ld_tx_free_n++] = idx;
		}
	}
	ld_exit(p);
}

int link_data_tx_pending(void)
{
	uint32_t p = ld_enter();
	int n = (ld_tx_head != ld_tx_tail);

	ld_exit(p);
	return n;
}

/* ------------------------------------------------------------------ *
 *  receive
 * ------------------------------------------------------------------ */
uint8_t *link_data_rx_claim(uint16_t body_len)
{
	uint8_t *body = NULL;
	uint32_t p;

	if (body_len < LINK_DATA_HDR || body_len > LINK_DATA_BODY_MAX) {
		p = ld_enter();
		ld_ctr.rx_oversize++;
		ld_exit(p);
		return NULL;
	}
	p = ld_enter();
	ld_init_locked();
	if (ld_rx_free_n != 0u) {
		uint8_t idx = ld_rx_free[--ld_rx_free_n];

		ld_rx_state[idx] = LD_BUSY;
		ld_rx_epoch[idx] = ld_epoch;
		ld_rx_inuse++;
		body = ld_rx_pool[idx].body;
	} else {
		ld_ctr.rx_drops++;               /* caller drains the body to stay in sync */
	}
	ld_exit(p);
	return body;
}

void link_data_rx_abort(uint8_t *body)
{
	int idx = ld_rx_index_body(body);
	uint32_t p = ld_enter();

	ld_rx_release_locked(idx);
	ld_exit(p);
}

void link_data_rx_commit(uint8_t *body, uint16_t body_len, uint16_t crc)
{
	int idx = ld_rx_index_body(body);
	uint32_t p;

	if (idx < 0)
		return;
	/* CRC outside the critical section: it is the one expensive step here. */
	if (body_len < LINK_DATA_HDR || erpc_crc16(body, body_len) != crc) {
		p = ld_enter();
		if (ld_rx_state[idx] == LD_BUSY)
			ld_ctr.rx_crc_err++;
		ld_rx_release_locked(idx);
		ld_exit(p);
		return;
	}

	p = ld_enter();
	if (ld_rx_state[idx] != LD_BUSY || ld_rx_epoch[idx] != ld_epoch) {
		/* Reset ran while the body was arriving: the frame belongs to a link that no
		 * longer exists.  Release rather than deliver -- the slot is still ours. */
		ld_rx_release_locked(idx);
		ld_exit(p);
		return;
	}
	ld_rx_pool[idx].len = body_len;
	ld_rx_pool[idx].crc = crc;
	if (ld_rx_cb == NULL) {
		ld_ctr.rx_no_consumer++;
		ld_rx_release_locked(idx);
	} else {
		ld_rx_state[idx] = LD_QUEUED;
		ld_rx_done[ld_rx_done_head] = (uint8_t)idx;
		ld_rx_done_head = (uint8_t)((ld_rx_done_head + 1u) % LD_RX_Q_CAP);
		ld_ctr.rx_frames++;
		ld_ctr.rx_bytes += (uint32_t)(body_len - LINK_DATA_HDR);
	}
	ld_exit(p);
}

int link_data_rx_ready(void)
{
	uint32_t p = ld_enter();
	int n = (ld_rx_done_head != ld_rx_done_tail);

	ld_exit(p);
	return n;
}

void link_data_rx_dispatch(void)
{
	for (;;) {
		int (*cb)(void *, uint8_t, uint8_t *, uint16_t);
		struct link_data_buf *b;
		void *ctx;
		uint32_t p;
		uint8_t idx;
		uint16_t n;
		int kept;

		p = ld_enter();
		if (ld_rx_done_head == ld_rx_done_tail) {
			ld_exit(p);
			return;
		}
		idx = ld_rx_done[ld_rx_done_tail];
		ld_rx_done_tail = (uint8_t)((ld_rx_done_tail + 1u) % LD_RX_Q_CAP);
		if (ld_rx_state[idx] != LD_QUEUED || ld_rx_epoch[idx] != ld_epoch) {
			ld_exit(p);              /* reset reclaimed it: skip, do not deliver */
			continue;
		}
		ld_rx_state[idx] = LD_BUSY;      /* the consumer owns it while cb runs */
		cb  = ld_rx_cb;
		ctx = ld_rx_ctx;
		ld_exit(p);

		b = &ld_rx_pool[idx];
		n = (uint16_t)(b->len - LINK_DATA_HDR);
		/* The callback runs with NO lock held -- that is the whole point of queueing
		 * here rather than calling straight out of the frame reader (app/erpc.c). */
		kept = (cb != NULL) ? cb(ctx, b->body[0], b->body + LINK_DATA_HDR, n) : 0;
		if (kept != 1)
			link_data_rx_abort(b->body);
	}
}

void link_data_rx_free(uint8_t *p)
{
	int idx = ld_rx_index(p);
	uint32_t pm = ld_enter();

	ld_rx_release_locked(idx);
	ld_exit(pm);
}

/* ------------------------------------------------------------------ *
 *  reset / stats
 * ------------------------------------------------------------------ */
void link_data_reset(void)
{
	uint32_t p = ld_enter();

	ld_init_locked();                        /* first call ever: build the free lists */
	ld_reclaim_locked();
	memset(&ld_ctr, 0, sizeof(ld_ctr));
	ld_exit(p);
}

void link_data_stats(struct link_data_stats *out)
{
	uint32_t p;

	if (out == NULL)
		return;
	p = ld_enter();
	*out = ld_ctr;
	out->tx_queued = (uint32_t)((ld_tx_head + LD_TX_Q_CAP - ld_tx_tail) % LD_TX_Q_CAP);
	out->rx_inuse  = ld_rx_inuse;
	ld_exit(p);
}
