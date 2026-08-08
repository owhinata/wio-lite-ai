/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/*
 * Minimal clean-room eRPC client for the onboard RTL8720DN (issue #5).
 * See erpc.h for the wire format and the public contract.
 *
 * Register-agnostic (no RCC / peripheral setup here) -- clock-safe.  It only uses
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
 * Issue #23 U0-3 adds a second frame type on the same wire, the LINK-CTRL channel (see
 * erpc.h): one extra slot, sent and received by the same thread, that carries link-layer
 * business rather than eRPC -- reading the module's UART counters, changing the baud
 * rate, and generating measured traffic.  It is only ever used on a quiescent link.
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
#include "link_data.h"    /* the DATA channel this thread also multiplexes (U1) */

#include <string.h>
#include "mem_sections.h"  /* DTCM_BSS: CPU-only data out of AXI-SRAM (issue #46) */

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
 * mark was 268 B of 1536.
 *
 * Issue #23 U3 raised it to 3072 on the argument that the DATA receive callback had become
 * a NetX driver -- allocating a packet, copying up to 1514 bytes into it and queueing it
 * for the IP thread -- and that a stack overflow on the only surviving board is not how
 * one wants to learn the new figure.  U4 then drove 2.5 MB of TCP and a telnet console
 * through that callback and measured 332 B.  The driver costs 64 B, not 2 kB, because it
 * does not recurse into the stack: the deferred-receive call only queues.
 *
 * So it comes back to 1536, which is 4.6x the measured peak, and the 3072 is retired as
 * what it always was -- headroom held until there was a number. */
#define ERPC_SVC_PRIORITY    10u
#define ERPC_SVC_STACK       1536u

/* Event flags: one "work posted" bit, one CTRL completion bit, one completion bit per
 * eRPC slot.  ERPC_F_CTRL sits next to ERPC_F_WORK, clear of the 0x100.. DONE group. */
#define ERPC_F_WORK          0x00000001u
#define ERPC_F_CTRL          0x00000002u
#define ERPC_F_DONE(i)       (0x00000100u << (i))

/* Leading u16 of a LINK-CTRL frame (issue #23 U0-3).  See the channel description in
 * erpc.h for why this cannot collide with an eRPC frame's message size.  0xFFFE is the
 * DATA channel's marker (app/link_data.h), by the same argument. */
#define ERPC_CTRL_MAGIC      0xFFFFu

/* DATA frames the service thread may write in one pass through the loop.  Bounded so a
 * saturated transmit queue cannot keep it from draining the RX ring: at 6 Mbaud two
 * 1500-byte frames is 5 ms of wire time, which is already far more than the ring holds
 * in reserve, and the loop comes straight back for the rest. */
#define ERPC_DATA_TX_PER_PASS 2

/* CTRL body header: cmd, seq, status, reserved. */
#define ERPC_CTRL_HDR        4u

static uint8_t erpc_scratch[ERPC_RX_SCRATCH];

/* CTRL frames assemble into the same scratch (the reader is single and never has more
 * than one frame in progress), so they must fit it. */
_Static_assert(ERPC_CTRL_MAX <= ERPC_RX_SCRATCH,
               "CTRL frames share the eRPC receive scratch");

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
 *  LINK-CTRL slot (issue #23 U0-3) -- exactly one exchange at a time
 * ------------------------------------------------------------------ */
/*
 * Deliberately NOT part of erpc_slots[]: a CTRL frame is a different frame type with its
 * own header, so merging them would mean teaching the eRPC send/dispatch paths about a
 * second wire format.  It does, however, share every LIFECYCLE rule -- it is sent by the
 * service thread, counts as pending work, and is abandoned when the link is torn down --
 * and each of those places is updated in step (see erpc_work_pending, erpc_abandon_all).
 *
 * One slot is enough because CTRL is only issued on a quiescent link, under the coarse
 * link mutex; a second caller gets -1 rather than queueing.
 */
struct erpc_ctrl_slot {
	uint8_t    state;                /* the ERPC_ST_* states, same meanings */
	uint8_t    cmd;                  /* command this exchange is waiting on */
	uint8_t    seq;                  /* and its sequence byte */
	uint8_t    status;               /* status byte the module answered with */
	uint8_t   *out;                  /* caller's reply buffer (may be NULL) */
	uint16_t   out_cap;
	uint16_t   reply_len;            /* payload length, excluding the 4-byte body header */
	uint16_t   body_len;             /* ERPC_CTRL_HDR + request payload */
	uint16_t   crc;
	TX_THREAD *waiter;
	uint8_t    body[ERPC_CTRL_MAX];
};
static struct erpc_ctrl_slot erpc_ctrl;
static uint8_t               erpc_ctrl_seq;

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

/* How many of those bytes may be outstanding, and the module generation it is derived
 * from.  Module-scoped, owned by app/rtl_link.c -- see the erpc_set_module_gen() contract
 * in erpc.h for why they do NOT follow the link open/close cycle. */
static uint16_t erpc_budget = ERPC_WIRE_BUDGET_SAFE;
static uint8_t  erpc_mod_gen;            /* N of "2.1.3+wio-nN"; 0 = unknown */

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
	uint32_t ctrl_bad;
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
	diag->ctrl_bad               = erpc_delta16(erpc_ctr.ctrl_bad, snap->ctrl_bad);
}

/* ------------------------------------------------------------------ *
 *  ThreadX objects
 * ------------------------------------------------------------------ */
static TX_MUTEX             erpc_lock;
static TX_EVENT_FLAGS_GROUP erpc_flags;
static TX_THREAD            erpc_svc_thread;
static UCHAR                erpc_svc_stack[ERPC_SVC_STACK] DTCM_BSS __attribute__((aligned(8)));
static uint8_t              erpc_ready;   /* service thread + objects exist */

/*
 * Whether the eRPC UART is open (app/rtl_link.c tells us).  Requests are refused while
 * it is down instead of being queued: rtl8720_uart_write() silently discards bytes with
 * no UART open, which would otherwise book wire debt for a frame that never went out and
 * make the caller wait out its whole timeout for a reply that cannot come.  A LOG-UART
 * open (the `wifi log` bridge) does NOT count -- that carries no eRPC.
 */
static uint8_t              erpc_link_up;

/*
 * Set while the service thread is (about to be) parked on the event flag, i.e. while it
 * is NOT looking at the RX ring.  Written under erpc_lock before the park and cleared on
 * wake; read by the RX interrupt to decide whether a wake-up is worth signalling.  See
 * the ordering argument at the park site -- dropping a notification is always safe.
 */
static volatile uint8_t     erpc_parked;

/*
 * RX interrupt hook (issue #23 U0-3).  Without it the service thread only notices bytes
 * on its 1 ms poll, which adds up to 1 ms to every frame -- ~20 % of a 1500-byte frame at
 * 6 Mbaud, enough to hide the 4-versus-6 Mbaud difference the U0-3 measurements exist to
 * find.  Runs in interrupt context at NVIC priority 5; tx_event_flags_set is legal there
 * (this ThreadX port uses PRIMASK critical sections, so an ISR cannot preempt one).
 */
static void erpc_rx_ready(void)
{
	if (erpc_parked)
		tx_event_flags_set(&erpc_flags, ERPC_F_WORK, TX_OR);
}

static void erpc_lock_get(void)  { tx_mutex_get(&erpc_lock, TX_WAIT_FOREVER); }
static void erpc_lock_put(void)  { tx_mutex_put(&erpc_lock); }

void erpc_link_lock(void)   { if (erpc_ready) erpc_lock_get(); }
void erpc_link_unlock(void) { if (erpc_ready) erpc_lock_put(); }

/*
 * CRC-16 (poly 0x1021, MSB-first, no reflection, init 0xEF4A -- erpc_crc16.cpp).
 *
 * TABLE-DRIVEN since issue #23 U1, for the DATA channel: the bit-at-a-time loop this
 * replaces costs ~40 cycles/byte, which is ~110 us for a 1500-byte Ethernet frame, and
 * the receive side of that runs on the link service thread for every frame in both
 * directions.  One byte per step brings it to ~11 us.  The table is built once on first
 * use FROM THE SAME BIT LOOP, so the values cannot drift from the definition -- the
 * standard identity CRC(byte) = table[(crc >> 8) ^ byte] ^ (crc << 8) holds exactly for
 * a non-reflected MSB-first CRC.  Verified byte-for-byte against the old implementation
 * over every length 0..64 and 1500 with pseudo-random data (off-target check, U1).
 *
 * 512 B of BSS.  The lazy init costs 256 * 8 iterations once and is idempotent, so it
 * does not need erpc_service_init() to have run (cmd_wifi_link.c computes CRCs too).
 */
static uint16_t erpc_crc_tab[256];
static uint8_t  erpc_crc_tab_ready;

static void erpc_crc16_build(void)
{
	uint32_t b;

	for (b = 0u; b < 256u; b++) {
		uint32_t crc = b << 8;
		int i;

		for (i = 0; i < 8; i++) {
			uint32_t t = crc << 1;

			if (crc & 0x8000u)
				t ^= 0x1021u;
			crc = t;
		}
		erpc_crc_tab[b] = (uint16_t)crc;
	}
	erpc_crc_tab_ready = 1u;                 /* publish last: the table is idempotent */
}

uint16_t erpc_crc16(const uint8_t *d, uint16_t n)
{
	uint32_t crc = 0xEF4Au;
	uint16_t j;

	if (!erpc_crc_tab_ready)
		erpc_crc16_build();
	for (j = 0u; j < n; j++)
		crc = (uint32_t)erpc_crc_tab[((crc >> 8) ^ d[j]) & 0xFFu] ^ (crc << 8);
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
/* Header staging: 4 bytes for an eRPC frame, 6 for a CTRL or DATA one (magic, length,
 * CRC). */
static uint8_t  rx_hdr[6];
static uint8_t  rx_hdr_got;
static uint8_t  rx_hdr_need = 4u;        /* 4, or 6 once a channel magic is seen */
static uint8_t  rx_is_ctrl;              /* the frame being assembled is a CTRL frame */
static uint8_t  rx_is_data;              /* ... or a DATA frame (issue #23 U1) */
static uint16_t rx_size, rx_crc, rx_body_got;
static uint32_t rx_drain_left;           /* oversize frame: bytes still to discard */
static uint8_t  rx_active;               /* a partial frame is in progress */
static ULONG    rx_progress;             /* tick of the last byte consumed */
static uint32_t rx_ovf_seen;             /* last observed rtl8720_uart_overflows() */

/*
 * Where the body being assembled is going: erpc_scratch for eRPC and CTRL frames, or a
 * DATA pool buffer, which the reader fills DIRECTLY so a 1500-byte Ethernet frame is not
 * copied through a scratch buffer first.
 *
 * That makes the reader a buffer OWNER, which every abandon path has to respect --
 * hence the single reset below rather than a bare state clear: a partial DATA frame is
 * dropped by resync (ring overflow), by the stall backstop, by a stream reset before a
 * send, by link_data_reset() and by the UART closing, and all of them come through here.
 */
static uint8_t *rx_dst;

static void erpc_reader_reset(void)
{
	if (rx_is_data && rx_dst != NULL)
		link_data_rx_abort(rx_dst);      /* no-op if the pool already took it back */
	rx_dst        = NULL;
	rx_hdr_got    = 0u;
	rx_hdr_need   = 4u;
	rx_is_ctrl    = 0u;
	rx_is_data    = 0u;
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

/* Reset the byte stream: drop what is buffered, forget any partial frame (returning its
 * DATA pool buffer, if it had claimed one) and resync the overflow watch.  Caller holds
 * erpc_lock (so the reader is not running). */
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
	hdr = erpc_get_u32le(body + 0);
	seq = erpc_get_u32le(body + 4);
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

/*
 * Route one complete CTRL frame body to the single CTRL slot.  Caller holds erpc_lock.
 *
 * Two different rejections, deliberately handled differently: a CRC failure says the BYTE
 * STREAM is suspect (very likely a desynchronised reader that aligned on 0xFFFF by
 * chance), so it resynchronises; a well-formed frame nobody is waiting for is just a late
 * reply to a timed-out exchange, which leaves the stream perfectly healthy.
 */
static void erpc_ctrl_dispatch(const uint8_t *body, uint16_t len, uint16_t crc)
{
	struct erpc_ctrl_slot *s = &erpc_ctrl;
	uint16_t plen, copy;

	if (len < ERPC_CTRL_HDR || erpc_crc16(body, len) != crc) {
		erpc_ctr.ctrl_bad++;
		erpc_stream_reset_locked();
		return;
	}
	if (s->state != ERPC_ST_SENT || body[0] != s->cmd || body[1] != s->seq) {
		erpc_ctr.ctrl_bad++;             /* stale / unsolicited: drop it, stream is fine */
		return;
	}

	plen = (uint16_t)(len - ERPC_CTRL_HDR);
	copy = (plen < s->out_cap) ? plen : s->out_cap;
	if (s->out != NULL && copy != 0u)
		memcpy(s->out, body + ERPC_CTRL_HDR, copy);
	s->status    = body[2];
	s->reply_len = plen;
	s->state     = ERPC_ST_DONE;
	tx_event_flags_set(&erpc_flags, ERPC_F_CTRL, TX_OR);
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

	if (rx_hdr_got < rx_hdr_need) {
		r = rtl8720_uart_read(rx_hdr + rx_hdr_got,
		                      (size_t)(rx_hdr_need - rx_hdr_got));
		if (r != 0u) {
			rx_hdr_got = (uint8_t)(rx_hdr_got + r);
			rx_active  = 1u;
			progress   = 1;
		}
		/* The channel marker is decided from the first two bytes, so the header
		 * grows from 4 to 6 as soon as they are in -- before the length is
		 * believed. */
		if (rx_hdr_got >= 2u && rx_hdr_need == 4u) {
			uint16_t lead = erpc_get_u16le(rx_hdr + 0);

			if (lead == ERPC_CTRL_MAGIC) {
				rx_is_ctrl  = 1u;
				rx_hdr_need = 6u;
			} else if (lead == LINK_DATA_MAGIC) {
				rx_is_data  = 1u;
				rx_hdr_need = 6u;
			}
		}
		if (rx_hdr_got < rx_hdr_need)
			return progress;

		rx_body_got = 0u;
		if (rx_is_data) {
			rx_size = erpc_get_u16le(rx_hdr + 2);
			rx_crc  = erpc_get_u16le(rx_hdr + 4);
			/* TWO different failures, and they must not be handled the same way:
			 *  - a length outside the channel's bounds is not a frame at all (a
			 *    desynchronised stream that landed on 0xFFFE), so resynchronise --
			 *    draining a number we do not believe would swallow real frames;
			 *  - a valid length with the pool empty IS a frame, just one we cannot
			 *    hold, so drain exactly it.  Losing a frame is an ordinary
			 *    Ethernet event; losing stream synchronisation is not. */
			if (rx_size < LINK_DATA_HDR || rx_size > LINK_DATA_BODY_MAX) {
				rx_is_data = 0u;
				(void)link_data_rx_claim(rx_size);   /* counts rx_oversize */
				erpc_stream_reset_locked();
				return 1;
			}
			rx_dst = link_data_rx_claim(rx_size);
			if (rx_dst == NULL) {
				rx_is_data    = 0u;   /* nothing to give back */
				rx_drain_left = rx_size;
				return 1;
			}
		} else if (rx_is_ctrl) {
			rx_size = erpc_get_u16le(rx_hdr + 2);
			rx_crc  = erpc_get_u16le(rx_hdr + 4);
			/* NEVER drain a CTRL length we do not believe: on a desynchronised
			 * stream it is attacker-free but arbitrary, and draining it would
			 * swallow real frames.  Resynchronise instead. */
			if (rx_size > ERPC_CTRL_MAX || rx_size < ERPC_CTRL_HDR) {
				erpc_ctr.ctrl_bad++;
				erpc_stream_reset_locked();
				return 1;
			}
		} else {
			rx_size = erpc_get_u16le(rx_hdr + 0);
			rx_crc  = erpc_get_u16le(rx_hdr + 2);
			if (rx_size > ERPC_RX_SCRATCH) {  /* too big: drain to resync */
				erpc_ctr.oversize++;
				rx_drain_left = rx_size;
				return 1;
			}
			if (rx_size < 8u) {               /* cannot be a valid message */
				erpc_ctr.crc_fail++;
				erpc_reader_reset();
				return 1;
			}
		}
	}

	/* eRPC and CTRL frames assemble into the shared scratch (the reader is single and
	 * never has more than one frame in progress; ERPC_CTRL_MAX <= ERPC_RX_SCRATCH);
	 * a DATA frame goes straight into the pool buffer claimed above. */
	{
		uint8_t *dst = rx_is_data ? rx_dst : erpc_scratch;

		if (rx_body_got < rx_size) {
			r = rtl8720_uart_read(dst + rx_body_got,
			                      (size_t)(rx_size - rx_body_got));
			if (r != 0u) {
				rx_body_got = (uint16_t)(rx_body_got + r);
				progress    = 1;
			}
			if (rx_body_got < rx_size)
				return progress;
		}

		if (rx_is_data) {
			/* Hands the buffer to link_data, which checks the CRC and queues it
			 * for dispatch OUTSIDE this lock (see the service loop). */
			link_data_rx_commit(dst, rx_size, rx_crc);
			rx_is_data = 0u;         /* no longer ours: do not abort it in reset */
			rx_dst     = NULL;
		} else if (rx_is_ctrl) {
			erpc_ctrl_dispatch(dst, rx_size, rx_crc);
		} else {
			erpc_dispatch(dst, rx_size, rx_crc);
		}
	}
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
		if (on_wire == 0u && !link_data_attached()) {
			/* Nothing outstanding: drop stale RX so this call starts at a frame
			 * boundary (what erpc_begin() did before increment 8).  Never done
			 * with a frame on the wire -- its reply may already be buffered.
			 *
			 * ...and never while the DATA channel is live (issue #23 U1): those
			 * "stale" bytes are then most likely the middle of an Ethernet frame
			 * arriving right now, and throwing them away would desynchronise the
			 * reader instead of cleaning it.  Nothing is lost by skipping it: a
			 * late reply is still dropped by the strict sequence match, and the
			 * reader's own frame-boundary state is what resynchronisation uses.
			 * The rule that makes this safe in the other direction is the detach
			 * ordering contract in app/link_data.h -- the channel is only
			 * un-attached once both ends are known to be quiet. */
			erpc_stream_reset_locked();
		}
		(void)erpc_debt_add(s->seq, frame_len);
		erpc_put_u16le(frame_hdr + 0, s->body_len);
		erpc_put_u16le(frame_hdr + 2, s->crc);
		rtl8720_uart_write(frame_hdr, 4u);
		rtl8720_uart_write(s->body, s->body_len);
		s->state = ERPC_ST_SENT;
		rr = (i + 1) % ERPC_MAX_INFLIGHT;
		return 1;
	}
	return 0;
}

/*
 * Send the queued CTRL frame, if any.  Returns 1 if it went out.  Caller holds erpc_lock.
 *
 * CTRL bypasses the wire ledger and the budget entirely: it is only ever issued on a
 * quiescent link (erpc_link_quiescent() is a precondition of every caller), so there is
 * nothing to account against and nothing to interleave with.
 */
static int erpc_ctrl_send(void)
{
	struct erpc_ctrl_slot *s = &erpc_ctrl;
	uint8_t hdr[6];

	if (!erpc_link_up || s->state != ERPC_ST_QUEUED)
		return 0;
	if (erpc_bytes_on_wire() == 0u && !link_data_attached()) {
		/* Same rule as the eRPC send step: with nothing outstanding, start from a
		 * clean frame boundary so a stale byte cannot be parsed as our reply -- and
		 * the same exception while DATA is live. */
		erpc_stream_reset_locked();
	}
	erpc_put_u16le(hdr + 0, ERPC_CTRL_MAGIC);
	erpc_put_u16le(hdr + 2, s->body_len);
	erpc_put_u16le(hdr + 4, s->crc);
	rtl8720_uart_write(hdr, sizeof(hdr));
	rtl8720_uart_write(s->body, s->body_len);
	s->state = ERPC_ST_SENT;
	return 1;
}

/*
 * Send one queued DATA frame.  Returns 1 if a frame went out.  Caller holds erpc_lock.
 *
 * DATA is the lowest priority of the three channels: CTRL first (it only runs on a
 * quiescent link and is what measures the link), then eRPC (low rate, and its latency is
 * a telnet keystroke), then bulk DATA.  Frames are written WHOLE and never interleaved
 * with another channel's bytes -- the far end has no way to reassemble a fragment -- so
 * a 1500-byte frame at 6 Mbaud does delay whatever is queued behind it by 2.5 ms.  That
 * is the head-of-line cost of not fragmenting, and it is deliberate.
 *
 * No wire ledger and no budget: a DATA frame has no reply, so there is nothing to free
 * it, and the module's 16 kB input ring is what absorbs a burst (issue #23 U1).
 */
static int erpc_data_send_one(void)
{
	const uint8_t *body;
	uint16_t body_len = 0u, crc = 0u;
	uint8_t hdr[6];

	if (!erpc_link_up)
		return 0;
	body = link_data_tx_peek(&body_len, &crc);
	if (body == NULL)
		return 0;
	/*
	 * Only start a frame the UART can swallow WITHOUT waiting.  rtl8720_uart_write()
	 * blocks when its ring is full, and this runs holding erpc_lock -- a wait here would
	 * stall every other user of the link, `wifi reset`'s force-quiesce included.  Bulk
	 * traffic has no deadline of its own, so leaving it queued for the next pass costs
	 * nothing; eRPC and CTRL frames are small and keep their unconditional write.
	 */
	if (rtl8720_uart_tx_space() < (uint32_t)body_len + sizeof(hdr))
		return 0;
	erpc_put_u16le(hdr + 0, LINK_DATA_MAGIC);
	erpc_put_u16le(hdr + 2, body_len);
	erpc_put_u16le(hdr + 4, crc);
	rtl8720_uart_write(hdr, sizeof(hdr));
	rtl8720_uart_write(body, body_len);
	link_data_tx_pop();
	return 1;
}

/* Any slot still needing the service thread's attention? Caller holds erpc_lock.
 *
 * The CTRL slot MUST be counted here: it is what makes the loop below wait one tick
 * instead of parking on TX_WAIT_FOREVER, and a parked thread polls no RX at all -- a
 * CTRL reply would then never be read and every CTRL call would time out.  The DATA
 * queues are counted for the same reason (issue #23 U1): a frame handed to
 * link_data_send() from another thread must not sit until something else happens to
 * wake this one. */
static int erpc_work_pending(void)
{
	int i;

	if (erpc_ctrl.state == ERPC_ST_QUEUED || erpc_ctrl.state == ERPC_ST_SENT)
		return 1;
	if (link_data_tx_pending() || link_data_rx_ready())
		return 1;
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

		/* CTRL first: it only runs on a quiescent link, so there is nothing for it
		 * to be fair to, and it is the path whose latency is being measured. */
		(void)erpc_ctrl_send();

		/* Both loops are capped so this thread cannot hold erpc_lock indefinitely if
		 * the module streams continuously (a caller waiting on the mutex would stall,
		 * and rtl_link could not take the UART away).  Whatever is left over is
		 * handled on the next pass -- work is still pending, so the wait below is a
		 * single tick. */
		for (guard = 0; guard < ERPC_MAX_INFLIGHT && erpc_send_one(); guard++)
			;
		/* DATA after eRPC: bulk traffic must not delay a reply the shell is
		 * waiting on, and the cap keeps a saturated queue from starving RX. */
		for (guard = 0; guard < ERPC_DATA_TX_PER_PASS && erpc_data_send_one(); guard++)
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
		/* Publish "about to sleep" while still holding the lock, so the RX interrupt
		 * (see erpc_rx_ready) sees it no later than the decision it is meant to
		 * shorten.  Ordering argument in full:
		 *   - ERPC_F_WORK is sticky, so a notification raised any time AFTER this
		 *     store either returns the wait immediately or is consumed next pass.
		 *   - A notification raised BEFORE it can be dropped by the guard, and that
		 *     is harmless in both cases: if busy, the wait below is one tick anyway;
		 *     if not busy, nothing is in flight, so those bytes are stale and the
		 *     next send's leading flush discards them -- exactly what happened
		 *     before this optimisation existed.
		 * So no window pairs a dropped notification with an indefinite park. */
		erpc_parked = 1u;
		erpc_lock_put();

		/*
		 * Deliver received DATA frames with NO LOCK HELD (issue #23 U1).  The
		 * consumer is a network stack once U3 lands, and calling it from inside the
		 * lock would (a) hold the link shut for as long as the stack takes to
		 * process a packet and (b) put erpc_lock underneath whatever locks the stack
		 * takes, on a path where the reverse order (stack -> link_data_send) also
		 * exists.  Doing it here, after erpc_parked is published and the lock is
		 * dropped, costs nothing: ERPC_F_WORK is sticky, so anything that arrives
		 * while the callback runs returns the wait below immediately.
		 */
		link_data_rx_dispatch();

		/* Poll on 1 ms slices while anything is in flight; sleep until a caller
		 * posts work otherwise -- idle costs nothing and, crucially, leaves the
		 * UART/RX ring untouched so `wifi log` / the flash download path can own
		 * them.  ERPC_F_WORK is sticky, so a post that races this cannot be lost. */
		wait = busy ? 1u : TX_WAIT_FOREVER;
		(void)tx_event_flags_get(&erpc_flags, ERPC_F_WORK, TX_OR_CLEAR, &flags, wait);
		erpc_parked = 0u;
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

/* Release a CTRL slot nobody can be waiting on any more.  Caller holds erpc_lock. */
static void erpc_ctrl_release(void)
{
	erpc_ctrl.out     = NULL;                /* detach first, as for the eRPC slots */
	erpc_ctrl.out_cap = 0u;
	erpc_ctrl.waiter  = NULL;
	erpc_ctrl.state   = ERPC_ST_FREE;
}

void erpc_abandon_all(void)
{
	int i;

	if (!erpc_ready)
		return;
	erpc_lock_get();
	erpc_link_up = 0u;                       /* nothing may be sent until reopened */
	erpc_debt_reset();                       /* the module's view is going away too */
	/* Order matters: the reader gives its half-filled buffer back FIRST, then the
	 * pools are emptied.  The other way round would hand a buffer to a pool that has
	 * already reclaimed it (link_data.c's state byte catches that, but relying on the
	 * catch rather than the order would be sloppy). */
	erpc_reader_reset();
	link_data_reset();                       /* queued/partial DATA belongs to a link
	                                          * that no longer exists */
	/* The CTRL exchange dies with the link like any token: hand a slot somebody is
	 * blocked on to THEM (freeing it here would let a fresh erpc_ctrl_call() reuse it
	 * and the old waiter would pick up the new call's completion), otherwise free it. */
	if (erpc_ctrl.state != ERPC_ST_FREE) {
		erpc_ctrl.out     = NULL;
		erpc_ctrl.out_cap = 0u;
		if (erpc_ctrl.waiter == NULL) {
			erpc_ctrl_release();
		} else {
			erpc_ctrl.state = ERPC_ST_ABANDONED;
			tx_event_flags_set(&erpc_flags, ERPC_F_CTRL, TX_OR);
		}
	}
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
	erpc_stream_reset_locked();              /* returns any claimed DATA buffer first */
	link_data_reset();
	erpc_link_up = carries_erpc ? 1u : 0u;
	/* Arm the wake-up hook only for the link we actually read.  A LOG-UART open (the
	 * `wifi log` bridge) is consumed by the command thread, not by us, and rtl8720.c
	 * clears the hook on every open/close so this can never outlive the session. */
	rtl8720_uart_set_rx_notify(carries_erpc ? erpc_rx_ready : NULL);
	erpc_lock_put();
}

void erpc_data_posted(void)
{
	if (erpc_ready)
		tx_event_flags_set(&erpc_flags, ERPC_F_WORK, TX_OR);
}

int erpc_data_quiescent(void)
{
	int quiet;

	if (!erpc_ready)
		return 0;
	erpc_lock_get();
	/* Only a DATA frame half-received matters here.  rx_active would also be true for
	 * an eRPC frame arriving, and on a link with a telnet console that is a normal,
	 * frequent state -- asking about it would make this answer "no" for reasons that
	 * have nothing to do with the DATA channel. */
	quiet = !rx_is_data;
	erpc_lock_put();
	return quiet && !link_data_tx_pending() && !link_data_rx_ready();
}

int erpc_link_quiescent(void)
{
	int quiet, i;

	if (!erpc_ready)
		return 0;
	erpc_lock_get();
	quiet = (erpc_ctrl.state == ERPC_ST_FREE) && (erpc_bytes_on_wire() == 0u);
	/* The DATA channel counts too (issue #23 U1): a queued frame is about to be
	 * written, and a reader mid-frame means bytes are still coming.  `wifi link baud`
	 * relies on this -- changing the line rate under either would corrupt it. */
	if (quiet && (link_data_tx_pending() || link_data_rx_ready() || rx_active))
		quiet = 0;
	for (i = 0; quiet && i < ERPC_MAX_INFLIGHT; i++)
		if (erpc_slots[i].state != ERPC_ST_FREE)
			quiet = 0;
	erpc_lock_put();
	return quiet;
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

uint8_t erpc_module_gen(void)
{
	uint8_t g;

	if (!erpc_ready)
		return 0u;
	erpc_lock_get();
	g = erpc_mod_gen;
	erpc_lock_put();
	return g;
}

void erpc_set_module_gen(uint8_t gen)
{
	if (!erpc_ready)
		return;
	erpc_lock_get();
	erpc_mod_gen = gen;
	/* DERIVED, never latched separately: one fact about the module cannot be recorded
	 * in two places without eventually disagreeing with itself. */
	erpc_budget  = (gen >= 4u) ? ERPC_WIRE_BUDGET_FAST : ERPC_WIRE_BUDGET_SAFE;
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
	erpc_put_u32le(s->body + 0, msg_hdr);
	erpc_put_u32le(s->body + 4, seq);
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

/* ------------------------------------------------------------------ *
 *  LINK-CTRL (issue #23 U0-3)
 * ------------------------------------------------------------------ */
/*
 * Decide the CTRL waiter's fate in ONE locked step, exactly as erpc_slot_settle() does
 * for eRPC tokens, so a reply landing between two checks is never thrown away.
 *   1        reply captured: *@len is the payload length, *@status the module's status
 *   -2       the link was torn down under us (ABANDONED)
 *   @fail_rc the caller's own verdict applies; the reply buffer is detached first
 *   0        undecided (only when @fail_rc == 0)
 */
static int erpc_ctrl_settle(int fail_rc, int *len, uint8_t *status)
{
	struct erpc_ctrl_slot *s = &erpc_ctrl;
	int rc = 0;

	erpc_lock_get();
	if (s->state == ERPC_ST_DONE) {
		*len    = (int)s->reply_len;
		*status = s->status;
		erpc_ctrl_release();
		rc = 1;
	} else if (s->state == ERPC_ST_ABANDONED) {
		erpc_ctrl_release();
		rc = -2;
	} else if (fail_rc != 0) {
		erpc_ctrl_release();
		rc = fail_rc;
	}
	erpc_lock_put();
	return rc;
}

/*
 * The module's status byte from the exchange that most recently returned -3.  Written only
 * here, read only by the caller that just got the -3; single-slot, single-owner CTRL is
 * what makes that well defined (see erpc.h).
 */
static uint8_t erpc_ctrl_status_v;

uint8_t erpc_ctrl_last_status(void)
{
	return erpc_ctrl_status_v;
}

int erpc_ctrl_call(uint8_t cmd, const uint8_t *req, uint16_t req_len,
                   uint8_t *out, uint16_t out_cap, uint32_t timeout_ms,
                   struct erpc_diag *diag)
{
	struct erpc_ctrl_slot *s = &erpc_ctrl;
	struct erpc_counters snap;
	ULONG deadline;

	erpc_ctrl_status_v = 0u;
	if (diag)
		memset(diag, 0, sizeof(*diag));
	if (!erpc_ready)
		return -1;
	if (req_len > (uint16_t)(ERPC_CTRL_MAX - ERPC_CTRL_HDR) ||
	    (req_len != 0u && req == NULL))
		return -1;

	erpc_lock_get();
	if (!erpc_link_up || s->state != ERPC_ST_FREE) {
		erpc_lock_put();
		return -1;                       /* no eRPC UART, or one is already in flight */
	}
	s->cmd       = cmd;
	s->seq       = ++erpc_ctrl_seq;
	s->status    = 0u;
	s->body[0]   = cmd;
	s->body[1]   = s->seq;
	s->body[2]   = 0u;                       /* status: host->module always 0 */
	s->body[3]   = 0u;
	if (req_len != 0u)
		memcpy(s->body + ERPC_CTRL_HDR, req, req_len);
	s->body_len  = (uint16_t)(ERPC_CTRL_HDR + req_len);
	s->crc       = erpc_crc16(s->body, s->body_len);
	s->out       = out;
	s->out_cap   = out_cap;
	s->reply_len = 0u;
	s->waiter    = tx_thread_identify();
	s->state     = ERPC_ST_QUEUED;
	snap = erpc_ctr;
	/* Clear a completion bit left over from an earlier exchange. */
	tx_event_flags_set(&erpc_flags, ~ERPC_F_CTRL, TX_AND);
	erpc_lock_put();

	tx_event_flags_set(&erpc_flags, ERPC_F_WORK, TX_OR);

	deadline = tx_time_get() + (ULONG)timeout_ms;   /* 1 tick = 1 ms */
	for (;;) {
		uint8_t status = 0u;
		ULONG flags;
		int len = -1, rc;

		rc = erpc_ctrl_settle(0, &len, &status);
		if (rc != 0) {
			erpc_diag_delta(diag, &snap);
			if (rc < 0)
				return rc;
			erpc_ctrl_status_v = status;
			return (status != 0u) ? -3 : len;
		}
		if ((int32_t)(tx_time_get() - deadline) >= 0) {
			rc = erpc_ctrl_settle(-2, &len, &status);
			erpc_diag_delta(diag, &snap);
			if (rc > 0) {
				erpc_ctrl_status_v = status;
				return (status != 0u) ? -3 : len;  /* landed in the same instant */
			}
			if (diag)
				diag->timeout++;
			return rc;
		}
		(void)tx_event_flags_get(&erpc_flags, ERPC_F_CTRL, TX_OR_CLEAR, &flags, 1u);
	}
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
	slen = erpc_get_u32le(r);                /* BasicCodec writeString: u32 len + bytes */
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
