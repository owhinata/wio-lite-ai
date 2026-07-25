/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/*
 * Minimal clean-room eRPC client for the onboard RTL8720DN (issue #5).
 * See erpc.h for the wire format and the public contract.
 *
 * Register-agnostic (no RCC / peripheral setup here) -- XIP-safe.  It only uses
 * ThreadX time/sync and the #17 rtl8720 UART driver for the raw bytes.
 *
 * Concurrency model (issue #21 increment 8).  ONE resident service thread owns the
 * link; every caller just posts a request and waits for its own reply:
 *
 *   caller: erpc_begin()   builds [header][seq][params] + CRC into a free slot,
 *                          marks it QUEUED and wakes the service thread
 *   svc:    send step      sends one queued frame at a time, subject to the wire
 *                          budget (see erpc_wire_may_send), and marks it SENT
 *   svc:    receive step   feeds the resumable frame reader from the RX ring and
 *                          routes each finished reply to the slot whose SEQUENCE it
 *                          carries, copying the payload into that caller's @out
 *   caller: erpc_wait()    polls its slot / abort hook / deadline on 1 ms slices
 *
 * Why a dedicated thread: the issue-#20 N3 firmware serves several requests at once
 * and replies out of order, and its accept/recv block for seconds on the module.  A
 * per-call lock would let one blocking accept freeze the whole link, so instead the
 * link has a single owner that multiplexes by sequence number, and callers never
 * touch the UART themselves.  erpc_call()/erpc_call_ex() are begin+wait wrappers, so
 * their behaviour is unchanged from the pre-increment-8 single-owner client.
 *
 * Locking.  erpc_lock guards the slot table, the wire ledger, the frame reader and
 * the UART itself: the service thread holds it across its whole work section and
 * releases it only while parked on the event flag, so "holding erpc_lock" means the
 * service thread is touching neither USART1 nor the RX ring.  app/rtl_link.c relies
 * on exactly that when it opens/closes the UART (erpc_link_lock/_unlock).  Callers
 * take erpc_lock only for short, non-blocking sections.  Lock order elsewhere is
 * rtl_link's coarse mutex first, then this one -- the service thread takes only this
 * one, so no cycle exists.
 */
#include "tx_api.h"       /* tx_time_get / mutex / event flags (1 tick = 1 ms here) */
#include "rtl8720.h"      /* rtl8720_uart_read / _write (USART1 @2 Mbaud, #17) */
#include "erpc.h"

#include <string.h>

/* BasicCodec / message constants (erpc_basic_codec.cpp, erpc_codec.h). */
#define ERPC_CODEC_VERSION   1u
#define ERPC_MSGTYPE_REPLY   2u          /* invocation=0, oneway=1, reply=2, notif=3 */

/* rpc_system interface (rpc_system.h). */
#define ERPC_SYS_SERVICE     1u
#define ERPC_SYS_VERSION_REQ 1u
#define ERPC_SYS_ACK_REQ     2u

/* Receive scratch: every frame body is assembled here, independent of the caller's
 * out_cap, and matches the firmware's MessageBuffer size (erpc_config).  Static
 * BSS in AXI-SRAM; CPU-only (no DMA) so it is D-cache coherent. */
#define ERPC_RX_SCRATCH      4096u

/* Drop a partial frame that has made no progress for this long and resynchronise.
 * Backstop for a module that dies / is reset mid-frame (byte LOSS is caught earlier
 * and precisely by the RX ring's overflow counter).  A full 4 KB frame takes ~20 ms
 * at 2 Mbaud, so 200 ms cannot fire on a healthy transfer. */
#define ERPC_FRAME_STALL_MS  200u

/* Wire ledger size: how many sent-but-unanswered request frames may be tracked.  With
 * ERPC_WIRE_BUDGET_SAFE 127 and the smallest frame 13 B (rpc_system_ack) at most 9 can
 * coexist, so 8 entries is in step with the budget and simply throttles (the caller
 * times out) rather than losing accounting.  On a wio-n4 link the budget stops binding
 * and this becomes the only limit, which is deliberate: ERPC_MAX_INFLIGHT is 4, so the
 * ledger can never actually fill. */
#define ERPC_WIRE_DEBTS      8

/* Reader steps per pass through the service loop.  Bounds how long the loop can hold
 * erpc_lock when bytes keep arriving; anything left is picked up on the next pass. */
#define ERPC_RX_STEPS_PER_PASS 32

/* Service thread: above the shell (16) and background jobs (17) so replies are routed
 * while a command computes, below the USB pump (8) and the IWDG petter (5).  Integer
 * marshalling only (no printf) and frames are assembled in static BSS, so the stack
 * stays shallow: measured on board #2 after 69k scheduling activations covering every
 * path (including a 1992-byte scan reply and the `net echo` data loop) the high-water
 * mark was 268 B of 1536.  The headroom is deliberate but could be reduced to 1 KB if
 * AXI-SRAM ever gets tight. */
#define ERPC_SVC_PRIORITY    10u
#define ERPC_SVC_STACK       1536u

/* Event flags: one "work posted" bit plus one completion bit per slot. */
#define ERPC_F_WORK          0x00000001u
#define ERPC_F_DONE(i)       (0x00000100u << (i))

static uint8_t erpc_scratch[ERPC_RX_SCRATCH];
static uint32_t erpc_seq;                /* monotonically-increasing request sequence */

/* ------------------------------------------------------------------ *
 *  in-flight slots
 * ------------------------------------------------------------------ */
enum {
	ERPC_ST_FREE = 0,
	ERPC_ST_QUEUED,                  /* body built, waiting for the service thread */
	ERPC_ST_SENT,                    /* on the wire, waiting for a reply */
	ERPC_ST_DONE,                    /* reply captured into out */
	ERPC_ST_ABANDONED                /* link torn down; the waiter must release it */
};

struct erpc_slot {
	uint8_t    state;
	uint8_t    service;              /* expected reply service id */
	uint8_t    request;              /* expected reply request id */
	uint32_t   seq;                  /* sequence that identifies this call's reply */
	uint8_t   *out;                  /* caller's reply buffer (may be NULL) */
	uint16_t   out_cap;              /* capacity of @out */
	uint16_t   reply_len;            /* full payload length of the captured reply */
	uint16_t   body_len;             /* 8 + request length */
	uint16_t   crc;                  /* CRC of body[0 .. body_len) */
	TX_THREAD *waiter;               /* the one thread allowed to wait/cancel */
	uint8_t    body[8u + ERPC_REQ_MAX];
};
static struct erpc_slot erpc_slots[ERPC_MAX_INFLIGHT];

/* ------------------------------------------------------------------ *
 *  wire ledger (what the module has not answered yet)
 * ------------------------------------------------------------------ */
/*
 * One entry per request frame written to the UART, kept until the module answers that
 * sequence -- which proves it read those bytes out of its 127-byte input ring.  The
 * ledger deliberately OUTLIVES the caller's slot: a caller that times out or aborts
 * releases its token, but the module may still be reading its frame, so those bytes
 * must keep counting against the budget.  Entries are also dropped wholesale when the
 * UART closes or the link is force-quiesced (both destroy the module's view too).
 */
struct erpc_debt {
	uint32_t seq;
	uint16_t len;                    /* full frame length (4 + body_len) */
	uint8_t  used;
};
static struct erpc_debt erpc_debts[ERPC_WIRE_DEBTS];

/* How many of those bytes may be outstanding.  Module-scoped, owned by app/rtl_link.c --
 * see the erpc_set_wire_budget() contract in erpc.h for why it does NOT follow the link
 * open/close cycle. */
static uint16_t erpc_budget = ERPC_WIRE_BUDGET_SAFE;

static uint16_t erpc_bytes_on_wire(void)
{
	uint16_t sum = 0u;
	int i;

	for (i = 0; i < ERPC_WIRE_DEBTS; i++)
		if (erpc_debts[i].used)
			sum = (uint16_t)(sum + erpc_debts[i].len);
	return sum;
}

static int erpc_debt_add(uint32_t seq, uint16_t len)
{
	int i;

	for (i = 0; i < ERPC_WIRE_DEBTS; i++) {
		if (erpc_debts[i].used)
			continue;
		erpc_debts[i].seq  = seq;
		erpc_debts[i].len  = len;
		erpc_debts[i].used = 1u;
		return 0;
	}
	return -1;                       /* ledger full: do not send */
}

static int erpc_debt_have_room(void)
{
	int i;

	for (i = 0; i < ERPC_WIRE_DEBTS; i++)
		if (!erpc_debts[i].used)
			return 1;
	return 0;
}

static void erpc_debt_clear(uint32_t seq)
{
	int i;

	for (i = 0; i < ERPC_WIRE_DEBTS; i++)
		if (erpc_debts[i].used && erpc_debts[i].seq == seq)
			erpc_debts[i].used = 0u;
}

static void erpc_debt_reset(void)
{
	memset(erpc_debts, 0, sizeof(erpc_debts));
}

/* ------------------------------------------------------------------ *
 *  diagnostics (monotonic; only the service thread writes them)
 * ------------------------------------------------------------------ */
struct erpc_counters {
	uint32_t crc_fail;
	uint32_t oversize;
	uint32_t skipped_reply;
	uint32_t unsupported_invocation;
	uint32_t frame_stall;
};
static struct erpc_counters erpc_ctr;

static uint16_t erpc_delta16(uint32_t now, uint32_t then)
{
	uint32_t d = now - then;                     /* wrap-safe */

	return (d > 0xFFFFu) ? 0xFFFFu : (uint16_t)d;
}

static void erpc_diag_delta(struct erpc_diag *diag, const struct erpc_counters *snap)
{
	if (diag == NULL)
		return;
	diag->crc_fail               = erpc_delta16(erpc_ctr.crc_fail, snap->crc_fail);
	diag->oversize               = erpc_delta16(erpc_ctr.oversize, snap->oversize);
	diag->skipped_reply          = erpc_delta16(erpc_ctr.skipped_reply,
	                                            snap->skipped_reply);
	diag->unsupported_invocation = erpc_delta16(erpc_ctr.unsupported_invocation,
	                                            snap->unsupported_invocation);
	diag->frame_stall            = erpc_delta16(erpc_ctr.frame_stall, snap->frame_stall);
}

/* ------------------------------------------------------------------ *
 *  ThreadX objects
 * ------------------------------------------------------------------ */
static TX_MUTEX             erpc_lock;
static TX_EVENT_FLAGS_GROUP erpc_flags;
static TX_THREAD            erpc_svc_thread;
static UCHAR                erpc_svc_stack[ERPC_SVC_STACK] __attribute__((aligned(8)));
static uint8_t              erpc_ready;   /* service thread + objects exist */

/*
 * Whether the eRPC UART is open (app/rtl_link.c tells us).  Requests are refused while
 * it is down instead of being queued: rtl8720_uart_write() silently discards bytes with
 * no UART open, which would otherwise book wire debt for a frame that never went out and
 * make the caller wait out its whole timeout for a reply that cannot come.  A LOG-UART
 * open (the `wifi log` bridge) does NOT count -- that carries no eRPC.
 */
static uint8_t              erpc_link_up;

static void erpc_lock_get(void)  { tx_mutex_get(&erpc_lock, TX_WAIT_FOREVER); }
static void erpc_lock_put(void)  { tx_mutex_put(&erpc_lock); }

void erpc_link_lock(void)   { if (erpc_ready) erpc_lock_get(); }
void erpc_link_unlock(void) { if (erpc_ready) erpc_lock_put(); }

/* ------------------------------------------------------------------ *
 *  little-endian helpers / CRC
 * ------------------------------------------------------------------ */
static void put_u16le(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put_u32le(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
	p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static uint16_t get_u16le(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t get_u32le(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
	       ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

uint16_t erpc_crc16(const uint8_t *d, uint16_t n)
{
	uint32_t crc = 0xEF4Au;
	uint16_t j;

	for (j = 0u; j < n; j++) {
		int i;
		crc ^= (uint32_t)d[j] << 8;
		for (i = 0; i < 8; i++) {
			uint32_t t = crc << 1;
			if (crc & 0x8000u)
				t ^= 0x1021u;
			crc = t;
		}
	}
	return (uint16_t)crc;
}

/* ------------------------------------------------------------------ *
 *  frame reader (resumable, never blocks) -- service thread only
 * ------------------------------------------------------------------ */
/*
 * The service thread cannot block waiting for bytes (it must also notice newly posted
 * requests), so the reader keeps its partial-frame state across calls instead of
 * reading with a deadline: it consumes whatever the RX ring holds and only acts once a
 * whole frame is in.  Two things resynchronise it, because a lost byte would otherwise
 * make it parse the next frame's header as this frame's body forever:
 *   - the RX ring's overflow counter increasing (the ring drops the NEWEST byte, so
 *     this is the exact detector for a truncated frame), and
 *   - ERPC_FRAME_STALL_MS without progress (module gone / reset mid-frame).
 * Both drop the partial frame, flush the ring and count a frame_stall.  Recovery is
 * best effort from there (the following frame may still mis-parse, bounded by the 4 KB
 * scratch); the deterministic clean point is the flush the send step does whenever
 * nothing is on the wire, i.e. at every command boundary.
 */
static uint8_t  rx_hdr[4];
static uint8_t  rx_hdr_got;
static uint16_t rx_size, rx_crc, rx_body_got;
static uint32_t rx_drain_left;           /* oversize frame: bytes still to discard */
static uint8_t  rx_active;               /* a partial frame is in progress */
static ULONG    rx_progress;             /* tick of the last byte consumed */
static uint32_t rx_ovf_seen;             /* last observed rtl8720_uart_overflows() */

static void erpc_reader_reset(void)
{
	rx_hdr_got    = 0u;
	rx_size       = 0u;
	rx_crc        = 0u;
	rx_body_got   = 0u;
	rx_drain_left = 0u;
	rx_active     = 0u;
}

/* Drop any bytes still buffered so a call starts from a clean frame boundary
 * (discards stale/partial frames or a late reply to a previous, timed-out call).
 * Bounded to one scratch-full so a continuously-streaming peer (wrong baud) cannot
 * spin here. */
static void erpc_rx_flush(void)
{
	uint8_t tmp[64];
	uint32_t budget = ERPC_RX_SCRATCH;
	size_t r;

	while (budget != 0u && (r = rtl8720_uart_read(tmp, sizeof(tmp))) > 0u)
		budget -= (r > budget) ? budget : (uint32_t)r;
}

/* Reset the byte stream: drop what is buffered, forget any partial frame and resync
 * the overflow watch.  Caller holds erpc_lock (so the reader is not running). */
static void erpc_stream_reset_locked(void)
{
	erpc_rx_flush();
	erpc_reader_reset();
	rx_ovf_seen = rtl8720_uart_overflows();
}

/*
 * Route one complete, CRC-verified frame body.  Frames that match no live slot, or
 * that are device->host invocations / malformed, are counted and dropped.
 * Caller holds erpc_lock.
 */
static void erpc_dispatch(const uint8_t *body, uint16_t len, uint16_t crc)
{
	uint32_t hdr, seq;
	int i;

	if (len < 8u || erpc_crc16(body, len) != crc) {
		erpc_ctr.crc_fail++;
		return;
	}
	hdr = get_u32le(body + 0);
	seq = get_u32le(body + 4);
	if (((hdr >> 24) & 0xffu) != ERPC_CODEC_VERSION) {
		erpc_ctr.crc_fail++;             /* wrong version = malformed */
		return;
	}
	if ((hdr & 0xffu) != ERPC_MSGTYPE_REPLY) {
		erpc_ctr.unsupported_invocation++;   /* device->host request (oneway event) */
		return;
	}

	/* The module answered this sequence, so its request bytes have left the module's
	 * input ring -- free them from the wire budget even if the caller is long gone. */
	erpc_debt_clear(seq);

	for (i = 0; i < ERPC_MAX_INFLIGHT; i++) {
		struct erpc_slot *s = &erpc_slots[i];
		uint16_t plen, copy;

		if (s->state != ERPC_ST_SENT || s->seq != seq)
			continue;
		if (((hdr >> 16) & 0xffu) != s->service ||
		    ((hdr >> 8) & 0xffu) != s->request) {
			erpc_ctr.skipped_reply++;    /* seq matched, wrong svc/req = bad */
			return;
		}
		plen = (uint16_t)(len - 8u);
		copy = (plen < s->out_cap) ? plen : s->out_cap;
		if (s->out != NULL && copy != 0u)
			memcpy(s->out, body + 8, copy);
		s->reply_len = plen;
		s->state     = ERPC_ST_DONE;
		tx_event_flags_set(&erpc_flags, ERPC_F_DONE(i), TX_OR);
		return;
	}

	erpc_ctr.skipped_reply++;                /* reply for an unknown / released seq */
}

/* Advance the reader by whatever the ring holds.  Returns 1 if any byte was consumed
 * (or a frame completed), 0 if it made no progress.  Caller holds erpc_lock. */
static int erpc_rx_step(void)
{
	int progress = 0;
	size_t r;

	if (rx_drain_left != 0u) {               /* oversize frame: discard to resync */
		uint8_t tmp[64];

		while (rx_drain_left != 0u) {
			size_t want = (rx_drain_left < sizeof(tmp)) ? (size_t)rx_drain_left
			                                           : sizeof(tmp);
			r = rtl8720_uart_read(tmp, want);
			if (r == 0u)
				return progress;
			rx_drain_left -= (uint32_t)r;
			progress = 1;
		}
		erpc_reader_reset();
		return 1;
	}

	if (rx_hdr_got < 4u) {
		r = rtl8720_uart_read(rx_hdr + rx_hdr_got, (size_t)(4u - rx_hdr_got));
		if (r != 0u) {
			rx_hdr_got = (uint8_t)(rx_hdr_got + r);
			rx_active  = 1u;
			progress   = 1;
		}
		if (rx_hdr_got < 4u)
			return progress;
		rx_size     = get_u16le(rx_hdr + 0);
		rx_crc      = get_u16le(rx_hdr + 2);
		rx_body_got = 0u;
		if (rx_size > ERPC_RX_SCRATCH) {         /* too big: drain to resync */
			erpc_ctr.oversize++;
			rx_drain_left = rx_size;
			return 1;
		}
		if (rx_size < 8u) {                      /* cannot be a valid message */
			erpc_ctr.crc_fail++;
			erpc_reader_reset();
			return 1;
		}
	}

	if (rx_body_got < rx_size) {
		r = rtl8720_uart_read(erpc_scratch + rx_body_got,
		                      (size_t)(rx_size - rx_body_got));
		if (r != 0u) {
			rx_body_got = (uint16_t)(rx_body_got + r);
			progress    = 1;
		}
		if (rx_body_got < rx_size)
			return progress;
	}

	erpc_dispatch(erpc_scratch, rx_size, rx_crc);
	erpc_reader_reset();
	return 1;
}

/* ------------------------------------------------------------------ *
 *  send step -- service thread only
 * ------------------------------------------------------------------ */
/*
 * Send at most one queued request frame.  Returns 1 if a frame went out, 0 if there
 * was nothing to send or the wire budget / ledger says "not yet" (the frame stays
 * QUEUED and the caller's own timeout bounds the wait).  Caller holds erpc_lock.
 *
 * The budget is what keeps the module's 127-byte input ring from silently dropping
 * bytes now that several requests can be in flight (see ERPC_WIRE_BUDGET).  A lone
 * frame is always sent, whatever its size, so behaviour for a single call -- including
 * the deliberately oversized probes `net echo <port> 256` uses -- is exactly as it was.
 */
static int erpc_send_one(void)
{
	static int rr;                           /* round-robin start, for fairness */
	int k;

	if (!erpc_link_up)                       /* UART closed: nothing can go out */
		return 0;
	for (k = 0; k < ERPC_MAX_INFLIGHT; k++) {
		int i = (rr + k) % ERPC_MAX_INFLIGHT;
		struct erpc_slot *s = &erpc_slots[i];
		uint16_t frame_len, on_wire;
		uint8_t frame_hdr[4];

		if (s->state != ERPC_ST_QUEUED)
			continue;
		frame_len = (uint16_t)(s->body_len + 4u);
		on_wire   = erpc_bytes_on_wire();
		if (!erpc_debt_have_room())
			return 0;                /* ledger full: nothing may go out at all */
		if (on_wire != 0u && (uint32_t)on_wire + frame_len > erpc_budget)
			continue;                /* too big for the remaining budget right now;
			                          * a smaller queued frame may still fit */
		if (on_wire == 0u) {
			/* Nothing outstanding: drop stale RX so this call starts at a frame
			 * boundary (what erpc_begin() did before increment 8).  Never done
			 * with a frame on the wire -- its reply may already be buffered. */
			erpc_stream_reset_locked();
		}
		(void)erpc_debt_add(s->seq, frame_len);
		put_u16le(frame_hdr + 0, s->body_len);
		put_u16le(frame_hdr + 2, s->crc);
		rtl8720_uart_write(frame_hdr, 4u);
		rtl8720_uart_write(s->body, s->body_len);
		s->state = ERPC_ST_SENT;
		rr = (i + 1) % ERPC_MAX_INFLIGHT;
		return 1;
	}
	return 0;
}

/* Any slot still needing the service thread's attention? Caller holds erpc_lock. */
static int erpc_work_pending(void)
{
	int i;

	for (i = 0; i < ERPC_MAX_INFLIGHT; i++)
		if (erpc_slots[i].state == ERPC_ST_QUEUED ||
		    erpc_slots[i].state == ERPC_ST_SENT)
			return 1;
	return 0;
}

/* ------------------------------------------------------------------ *
 *  service thread
 * ------------------------------------------------------------------ */
static void erpc_svc_entry(ULONG arg)
{
	(void)arg;

	for (;;) {
		ULONG flags, wait;
		uint32_t ovf;
		int busy, guard;

		erpc_lock_get();

		/* Byte loss (ring overflow) desynchronises the reader: resync at once. */
		ovf = rtl8720_uart_overflows();
		if (ovf != rx_ovf_seen) {
			if (rx_active)
				erpc_ctr.frame_stall++;
			erpc_stream_reset_locked();
		}

		/* Both loops are capped so this thread cannot hold erpc_lock indefinitely if
		 * the module streams continuously (a caller waiting on the mutex would stall,
		 * and rtl_link could not take the UART away).  Whatever is left over is
		 * handled on the next pass -- work is still pending, so the wait below is a
		 * single tick. */
		for (guard = 0; guard < ERPC_MAX_INFLIGHT && erpc_send_one(); guard++)
			;
		for (guard = 0; guard < ERPC_RX_STEPS_PER_PASS && erpc_rx_step(); guard++)
			rx_progress = tx_time_get();

		/* Backstop: a partial frame that stopped advancing (module gone/reset). */
		if (rx_active &&
		    (int32_t)(tx_time_get() - rx_progress) >= (int32_t)ERPC_FRAME_STALL_MS) {
			erpc_ctr.frame_stall++;
			erpc_stream_reset_locked();
		}

		busy = erpc_work_pending();
		erpc_lock_put();

		/* Poll on 1 ms slices while anything is in flight; sleep until a caller
		 * posts work otherwise -- idle costs nothing and, crucially, leaves the
		 * UART/RX ring untouched so `wifi log` / the flash download path can own
		 * them.  ERPC_F_WORK is sticky, so a post that races this cannot be lost. */
		wait = busy ? 1u : TX_WAIT_FOREVER;
		(void)tx_event_flags_get(&erpc_flags, ERPC_F_WORK, TX_OR_CLEAR, &flags, wait);
	}
}

int erpc_service_init(void)
{
	if (erpc_ready)
		return 0;
	if (tx_mutex_create(&erpc_lock, "erpc", TX_INHERIT) != TX_SUCCESS)
		return -1;
	if (tx_event_flags_create(&erpc_flags, "erpc") != TX_SUCCESS) {
		tx_mutex_delete(&erpc_lock);
		return -1;
	}
	if (tx_thread_create(&erpc_svc_thread, "erpc", erpc_svc_entry, 0,
	                     erpc_svc_stack, sizeof(erpc_svc_stack),
	                     ERPC_SVC_PRIORITY, ERPC_SVC_PRIORITY,
	                     TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS) {
		tx_event_flags_delete(&erpc_flags);
		tx_mutex_delete(&erpc_lock);
		return -1;
	}
	erpc_ready = 1u;
	return 0;
}

/* ------------------------------------------------------------------ *
 *  slot lifecycle helpers (caller holds erpc_lock)
 * ------------------------------------------------------------------ */
static void erpc_slot_release(struct erpc_slot *s)
{
	s->out     = NULL;               /* detach first: the reply must not be copied */
	s->out_cap = 0u;
	s->waiter  = NULL;
	s->state   = ERPC_ST_FREE;
}

/*
 * Decide a waiter's fate for @s in ONE locked step, so a reply that lands between two
 * checks can never be thrown away.  Returns
 *   1        the reply is in: *@len is its full payload length, slot released
 *   -2       the link was torn down under us (ABANDONED), slot released
 *   @fail_rc the caller's own verdict (timeout / abort) actually applies: the reply
 *            buffer is detached and the slot released, so a late reply is dropped as
 *            stale rather than written into a buffer the caller is about to abandon
 *   0        nothing decided yet -- only when @fail_rc == 0 (keep waiting)
 */
static int erpc_slot_settle(struct erpc_slot *s, int fail_rc, int *len)
{
	int rc = 0;

	erpc_lock_get();
	if (s->state == ERPC_ST_DONE) {
		*len = (int)s->reply_len;
		erpc_slot_release(s);
		rc = 1;
	} else if (s->state == ERPC_ST_ABANDONED) {
		erpc_slot_release(s);
		rc = -2;
	} else if (fail_rc != 0) {
		erpc_slot_release(s);
		rc = fail_rc;
	}
	erpc_lock_put();
	return rc;
}

void erpc_abandon_all(void)
{
	int i;

	if (!erpc_ready)
		return;
	erpc_lock_get();
	erpc_link_up = 0u;                       /* nothing may be sent until reopened */
	erpc_debt_reset();                       /* the module's view is going away too */
	for (i = 0; i < ERPC_MAX_INFLIGHT; i++) {
		struct erpc_slot *s = &erpc_slots[i];

		if (s->state == ERPC_ST_FREE)
			continue;
		s->out     = NULL;               /* nothing may be written into it again */
		s->out_cap = 0u;
		if (s->waiter == NULL) {
			erpc_slot_release(s);
		} else {
			/* Someone is blocked on this token: hand the slot to THEM to
			 * release.  Freeing it here would let a fresh erpc_begin() reuse
			 * the index and the waiter would pick up that call's completion. */
			s->state = ERPC_ST_ABANDONED;
			tx_event_flags_set(&erpc_flags, ERPC_F_DONE(i), TX_OR);
		}
	}
	erpc_lock_put();
}

void erpc_link_opened(int carries_erpc)
{
	if (!erpc_ready)
		return;
	erpc_lock_get();
	erpc_debt_reset();
	erpc_stream_reset_locked();
	erpc_link_up = carries_erpc ? 1u : 0u;
	erpc_lock_put();
}

uint16_t erpc_wire_budget(void)
{
	uint16_t b;

	if (!erpc_ready)
		return ERPC_WIRE_BUDGET_SAFE;
	erpc_lock_get();
	b = erpc_budget;
	erpc_lock_put();
	return b;
}

void erpc_set_wire_budget(uint16_t bytes)
{
	if (!erpc_ready)
		return;
	if (bytes < ERPC_WIRE_BUDGET_SAFE)
		bytes = ERPC_WIRE_BUDGET_SAFE;   /* never below the always-safe floor */
	erpc_lock_get();
	erpc_budget = bytes;
	erpc_lock_put();
}

void erpc_link_closed(void)
{
	if (!erpc_ready)
		return;
	erpc_abandon_all();                      /* mutex is owner-reentrant; clears link_up */
	erpc_lock_get();
	erpc_stream_reset_locked();
	erpc_lock_put();
}

/* ------------------------------------------------------------------ *
 *  public API
 * ------------------------------------------------------------------ */
int erpc_begin(uint8_t service, uint8_t request,
               const uint8_t *req, uint16_t req_len,
               uint8_t *out, uint16_t out_cap)
{
	struct erpc_slot *s;
	uint32_t msg_hdr, seq;
	int idx = -1, i;

	if (!erpc_ready)
		return -1;
	if (req_len > ERPC_REQ_MAX || (req_len != 0u && req == NULL))
		return -1;

	erpc_lock_get();
	if (!erpc_link_up) {
		erpc_lock_put();
		return -1;                       /* no eRPC UART open: fail fast */
	}
	for (i = 0; i < ERPC_MAX_INFLIGHT; i++) {
		if (erpc_slots[i].state == ERPC_ST_FREE) {
			idx = i;
			break;
		}
	}
	if (idx < 0) {
		erpc_lock_put();
		return -1;                       /* no free slot: too many in flight */
	}
	s = &erpc_slots[idx];

	/* 32-bit monotonic sequence: strictly greater than every currently-live token's
	 * seq, so a fresh call can never collide with one still pending -- which is also
	 * what makes a stale reply for a released slot safe to drop. */
	seq = ++erpc_seq;
	msg_hdr = ((uint32_t)ERPC_CODEC_VERSION << 24) | ((uint32_t)service << 16) |
	          ((uint32_t)request << 8) | 0u /* kInvocationMessage */;
	put_u32le(s->body + 0, msg_hdr);
	put_u32le(s->body + 4, seq);
	if (req_len != 0u)
		memcpy(s->body + 8, req, req_len);
	s->body_len  = (uint16_t)(8u + req_len);
	s->crc       = erpc_crc16(s->body, s->body_len);
	s->service   = service;
	s->request   = request;
	s->seq       = seq;
	s->out       = out;
	s->out_cap   = out_cap;
	s->reply_len = 0u;
	s->waiter    = NULL;
	s->state     = ERPC_ST_QUEUED;
	/* Clear a completion bit left over from an earlier use of this slot. */
	tx_event_flags_set(&erpc_flags, ~ERPC_F_DONE(idx), TX_AND);
	erpc_lock_put();

	tx_event_flags_set(&erpc_flags, ERPC_F_WORK, TX_OR);
	return idx;
}

int erpc_wait(int token, uint32_t timeout_ms, struct erpc_diag *diag,
              int (*should_abort)(void *ctx), void *abort_ctx)
{
	struct erpc_counters snap;
	struct erpc_slot *s;
	ULONG deadline;

	if (diag)
		memset(diag, 0, sizeof(*diag));
	if (!erpc_ready || token < 0 || token >= ERPC_MAX_INFLIGHT)
		return -1;
	s = &erpc_slots[token];

	erpc_lock_get();
	if (s->state == ERPC_ST_FREE ||
	    (s->waiter != NULL && s->waiter != tx_thread_identify())) {
		erpc_lock_put();
		return -1;                       /* stale token, or already being waited on */
	}
	s->waiter = tx_thread_identify();        /* a token has ONE consumer */
	snap = erpc_ctr;
	erpc_lock_put();

	deadline = tx_time_get() + (ULONG)timeout_ms;   /* 1 tick = 1 ms */
	for (;;) {
		ULONG flags;
		int len = -1, rc;

		rc = erpc_slot_settle(s, 0, &len);       /* completed already? */
		if (rc != 0) {
			erpc_diag_delta(diag, &snap);
			return (rc > 0) ? len : rc;
		}

		/* Abort first, then the deadline: same order (and 1 ms granularity) as the
		 * pre-increment-8 receive loop, and always on the CALLER's thread. */
		if (should_abort != NULL && should_abort(abort_ctx)) {
			rc = erpc_slot_settle(s, -4, &len);
			erpc_diag_delta(diag, &snap);
			return (rc > 0) ? len : rc;
		}
		if ((int32_t)(tx_time_get() - deadline) >= 0) {
			rc = erpc_slot_settle(s, -2, &len);
			erpc_diag_delta(diag, &snap);
			if (rc > 0)
				return len;              /* it landed in the same instant */
			if (diag)
				diag->timeout++;
			return rc;
		}
		(void)tx_event_flags_get(&erpc_flags, ERPC_F_DONE(token), TX_OR_CLEAR,
		                         &flags, 1u);
	}
}

void erpc_cancel(int token)
{
	struct erpc_slot *s;

	if (!erpc_ready || token < 0 || token >= ERPC_MAX_INFLIGHT)
		return;
	erpc_lock_get();
	s = &erpc_slots[token];
	/* Idempotent, and only the token's own consumer may release it: freeing a slot
	 * another thread is waiting on would let it observe a reused slot's completion. */
	if (s->state != ERPC_ST_FREE &&
	    (s->waiter == NULL || s->waiter == tx_thread_identify()))
		erpc_slot_release(s);
	erpc_lock_put();
}

int erpc_call_ex(uint8_t service, uint8_t request,
                 const uint8_t *req, uint16_t req_len,
                 uint8_t *out, uint16_t out_cap, uint32_t timeout_ms,
                 struct erpc_diag *diag,
                 int (*should_abort)(void *ctx), void *abort_ctx)
{
	int token;

	/* Synchronous round-trip = begin + wait, so the contract is identical to the
	 * pre-increment-8 client: the same leading rx-flush (done by the service thread
	 * when nothing is on the wire), the same -1/-2/-4, the same truncate-to-out_cap
	 * with the full payload length returned, the same diag.  erpc_wait() invalidates
	 * the token on failure, so there is nothing left to cancel here. */
	if (diag)
		memset(diag, 0, sizeof(*diag));  /* keep the old contract: cleared even on -1 */
	token = erpc_begin(service, request, req, req_len, out, out_cap);
	if (token < 0)
		return -1;
	return erpc_wait(token, timeout_ms, diag, should_abort, abort_ctx);
}

int erpc_call(uint8_t service, uint8_t request,
              const uint8_t *req, uint16_t req_len,
              uint8_t *out, uint16_t out_cap, uint32_t timeout_ms,
              struct erpc_diag *diag)
{
	/* Backwards-compatible wrapper: no abort hook (never cancelled early). */
	return erpc_call_ex(service, request, req, req_len, out, out_cap,
	                    timeout_ms, diag, NULL, NULL);
}

int erpc_system_ack(uint8_t c, uint8_t *echoed, struct erpc_diag *diag)
{
	uint8_t r = 0u;
	int n = erpc_call(ERPC_SYS_SERVICE, ERPC_SYS_ACK_REQ, &c, 1u, &r, 1u, 300u, diag);

	if (n < 0)
		return n;
	if (n < 1)
		return -3;                       /* reply carried no result byte */
	if (echoed)
		*echoed = r;
	return 0;
}

int erpc_system_version(char *out, uint16_t out_cap, struct erpc_diag *diag)
{
	uint8_t r[128];                          /* build-id strings are short; 128 is ample */
	int n = erpc_call(ERPC_SYS_SERVICE, ERPC_SYS_VERSION_REQ, NULL, 0u,
	                  r, (uint16_t)sizeof(r), 300u, diag);
	uint32_t slen;

	if (n < 0)
		return n;                        /* -2 timeout / -4 aborted, forwarded as-is */
	if (n < 4 || n > (int)sizeof(r))         /* need the u32 length; reject truncation */
		return -3;
	slen = get_u32le(r);                     /* BasicCodec writeString: u32 len + bytes */
	if (slen > (uint32_t)(n - 4))            /* length word disagrees with the payload */
		return -3;
	if (out && out_cap) {
		uint16_t copy = (slen < (uint32_t)(out_cap - 1u)) ? (uint16_t)slen
		                                                  : (uint16_t)(out_cap - 1u);
		memcpy(out, r + 4, copy);
		out[copy] = '\0';
	}
	return (int)slen;
}
