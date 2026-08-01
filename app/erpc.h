/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/*
 * Wio Lite AI (STM32H725AEI6) -- minimal clean-room eRPC client for the onboard
 * RTL8720DN (issue #5, increment 1: eRPC-over-UART link bring-up).
 *
 * The RTL8720DN ships Seeed's eRPC firmware (seeed-ambd-firmware @Wio-Lite-AI),
 * which exposes an eRPC server over UART @2,000,000 baud on "Serial3" (= the STM32
 * USART1 / BLE_UART, PA10/PB14).  This is a hand-written C client that speaks the
 * exact wire format of that firmware -- no C++ eRPC runtime is pulled in.  It sits
 * on top of the #17 rtl8720 UART driver (app/rtl8720.c) and never touches the RCC
 * clock tree, so it is clock-safe.
 *
 * Wire format (reference: https://github.com/Seeed-Studio/seeed-ambd-firmware,
 * branch Wio-Lite-AI):
 *   - Frame  = {u16 messageSize, u16 crc} (LE, 4 B) + body[messageSize].
 *              CRC is over the body only.  (erpc_framed_transport.cpp)
 *   - CRC16  = init 0xEF4A, poly 0x1021, MSB-first, no reflection. (erpc_crc16.cpp)
 *   - body   = u32 header + u32 sequence (LE) + params.            (erpc_basic_codec.cpp)
 *              header = (1<<24) | (service<<16) | (request<<8) | type.
 *              type: invocation=0, reply=2.
 *   - scalar = raw LE; string = u32 length + bytes (no NUL).
 *   - rpc_system: service=1, rpc_system_version=req1(->string),
 *                 rpc_system_ack(u8)=req2(->u8 echo).             (rpc_system.h)
 *
 * NOTE: device->host callbacks are NOT serviced.  The factory client waits forever
 * for a reply, so we only use APIs that trigger no non-oneway callback; the oneway
 * ones the module pushes at us (scan-done, WiFi events) are drained and counted as
 * unsupported invocations.  A host server stub is a separate issue.
 *
 * ---- Concurrency model (issue #21 increment 8) --------------------------------
 *
 * The link is owned by ONE resident service thread created by erpc_service_init()
 * (see app/erpc.c).  It is the only reader of the USART1 RX ring (which is a strict
 * SPSC pipe) and the only writer of request frames.  Callers -- any thread -- build
 * a request into a slot, hand it to that thread and wait for their own reply:
 *
 *     erpc_call/_ex(...)                     synchronous round-trip (begin+wait)
 *     t = erpc_begin(...); erpc_wait(t, ...) asynchronous, several in flight
 *
 * That is what lets a caller keep a blocking accept/recv outstanding on the module
 * (the issue-#20 N3 firmware serves requests concurrently and may reply OUT OF
 * ORDER) while another thread runs `wifi`/`net` commands on the same link.  Up to
 * ERPC_MAX_INFLIGHT requests may be outstanding.
 *
 * Before increment 8 all of this ran on the caller's thread and was safe only
 * because the shell held cli_console_claim; the public contract below (return
 * codes, truncation, diag, abort semantics) is unchanged by the move.
 *
 * Lock order, when a caller needs both: app/rtl_link.c's coarse link mutex FIRST,
 * then this module's internal mutex (the service thread only ever takes the latter,
 * so the order cannot cycle).
 */
#ifndef APP_ERPC_H
#define APP_ERPC_H

#include <stdint.h>
#include <stddef.h>

/*
 * ---- little-endian wire codec (issue #27) -------------------------------------
 *
 * Every byte this link carries is little-endian at its natural width: the eRPC
 * BasicCodec (app/wifi_rpc.c), the eRPC framing itself (app/erpc.c) and the CTRL /
 * DATA channels below (shell/cmds/cmd_wifi_link.c, app/nx_net.c).  One definition,
 * because they are all the SAME wire -- four private copies is how the four ends of
 * one protocol drift apart.  Byte-at-a-time deliberately: the buffers are packed, so
 * a cast to uint32_t* would be an unaligned access.
 */
static inline void erpc_put_u16le(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
}
static inline void erpc_put_u32le(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
	p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static inline uint16_t erpc_get_u16le(const uint8_t *p)
{
	return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}
static inline uint32_t erpc_get_u32le(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
	       ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* Maximum eRPC requests that may be outstanding at once (erpc_begin without a matching
 * erpc_wait/erpc_cancel yet).  Each live token costs one slot (metadata + a request
 * staging buffer) in AXI-SRAM static BSS. */
#define ERPC_MAX_INFLIGHT 4

/* Largest request parameter block one call may carry.  The service thread sends from a
 * per-slot buffer of 8 + ERPC_REQ_MAX bytes (8 = eRPC message header + sequence), so a
 * caller's own request bytes never have to outlive its erpc_begin().  It is bounded by
 * app/wifi_rpc.c's _Static_assert on 12 + WIFI_RPC_STREAM_MAX = 268 B, which was the
 * size of wifi_rpc_lwip_send() before issue #23 U4 moved TCP off the module and deleted
 * it; the assert is kept as the ceiling on WIFI_RPC_STREAM_MAX itself. */
#define ERPC_REQ_MAX      320u

/*
 * Bytes of request frames the service thread may have on the wire at once.
 *
 * The stock module transport reads the UART one byte at a time behind an Arduino
 * RingBuffer of 127 usable bytes that SILENTLY DROPS a byte when full (see the
 * ASYMMETRY note in app/wifi_rpc.h): a frame whose bytes cannot all fit in that ring
 * is only delivered if the module's reader keeps up, which is load-dependent.  The
 * only structural guarantee available is "everything we have sent and not yet seen
 * answered fits in the ring", so the service thread sends a queued frame only when
 * either nothing is on the wire (a lone frame behaves exactly as it did before this
 * budget existed -- including the deliberately oversized ones `net echo <port> 256`
 * uses to reproduce the measured cliff) or the frame still fits this budget.
 * A reply proves the module consumed that request's bytes, which is what frees them.
 *
 * That ring is gone in the issue #23 U0-2 firmware (`2.1.3+wio-n4`), which owns USI0
 * directly behind an 8 kB ring, so the budget can be lifted -- but ONLY once we have
 * seen that firmware answer on THIS link.  Hence a runtime value that always starts at
 * (and falls back to) the safe number: see erpc_set_wire_budget().
 */
#define ERPC_WIRE_BUDGET_SAFE 127u

/*
 * The budget once the module is known to be wio-n4.  Deliberately far above anything the
 * host can actually reach -- ERPC_MAX_INFLIGHT slots x (12 + ERPC_REQ_MAX) bytes = 1328 B
 * worst case -- so on an n4 link the budget stops throttling entirely and the ledger's
 * slot count is the only limit.  It is still a bound rather than "off" because it is what
 * automatically reappears as the safe 127 if the link is reopened or the module is
 * reflashed with an older image.
 */
#define ERPC_WIRE_BUDGET_FAST 4096u

/* Per-call receive diagnostics (why a link test failed / what was skipped).
 *
 * NOTE: the link is shared, so these count what the SERVICE THREAD saw while this
 * call was waiting -- frames dropped because of another caller's traffic land here
 * too.  Only `timeout` is strictly this call's own. */
struct erpc_diag {
	uint16_t crc_fail;               /* frames with a bad CRC / malformed header */
	uint16_t oversize;               /* frames larger than the RX scratch (drained) */
	uint16_t timeout;                /* ran past the deadline waiting for the reply */
	uint16_t skipped_reply;          /* reply frames for a different sequence (stale) */
	uint16_t unsupported_invocation; /* device->host request frames we cannot serve */
	uint16_t frame_stall;            /* partial frames dropped to resynchronise the
	                                  * stream (RX ring overflow / a module that went
	                                  * quiet mid-frame) */
	uint16_t ctrl_bad;               /* CTRL frames rejected: bad length / CRC / unknown
	                                  * command / nobody waiting.  MUST STAY 0 on a
	                                  * healthy link -- a non-zero count is either a
	                                  * regression or a desynchronised stream that
	                                  * happened to align on the 0xFFFF marker */
};

/* eRPC CRC-16 (init 0xEF4A, poly 0x1021, MSB-first) over @n bytes of @d. */
uint16_t erpc_crc16(const uint8_t *d, uint16_t n);

/*
 * Create the link service thread and its ThreadX objects.  Call ONCE from
 * tx_application_define() (no HAL_GetTick dependency, so it is safe there -- see
 * issue #12); every other entry point here needs it.  Returns 0 on success, -1 if a
 * ThreadX object could not be created -- in which case the module stays unusable and
 * every call below fails with -1 (fail-soft: the rest of the firmware still runs).
 */
int erpc_service_init(void);

/*
 * Tear the link down: detach every live token's reply buffer, drop the record of what
 * is on the wire and reset the frame reader.  A caller waiting on a token wakes up and
 * gets -2; a token nobody waits on is released immediately.
 *
 * This is the "recovery" primitive: it must run (under the coarse link mutex) BEFORE
 * CHIP_EN is power-cycled / the UART is force-closed, so `wifi reset` can always take
 * the link away from whoever holds it -- see rtl_link_force_quiesce().  Not for normal
 * teardown: an ordinary caller cancels its own token.
 */
void erpc_abandon_all(void);

/*
 * ---- UART hand-over (app/rtl8720.c is shared with the console bridge / flasher) ----
 *
 * erpc_link_lock() / _unlock() take the service thread's own mutex.  Because that
 * thread holds it across its whole work section and drops it only while parked,
 * holding it PROVES the service thread is touching neither USART1 nor the RX ring --
 * which is what makes it safe to call rtl8720_uart_open() / _close() (open resets the
 * shared ring, close stops the peripheral).  app/rtl_link.c brackets every open/close
 * with these; nothing else should need them.  Recursive for the same thread.
 *
 * erpc_link_opened() / _closed() then tell the client what happened: both drop the
 * wire-budget record and reset the frame reader so a session starts from a clean
 * stream, and _closed() additionally abandons any token still live (a live token at
 * close time means the link was taken away from its owner).  Call them with the lock
 * held, right after the open / before the close.
 *
 * @carries_erpc says whether the UART just opened is the eRPC one: only then may
 * requests be sent.  Pass 0 for the LOG-UART console bridge, whose bytes are not eRPC --
 * a request queued against it would be silently discarded by the UART driver and the
 * caller would wait out its whole timeout for a reply that cannot exist.
 */
void erpc_link_lock(void);
void erpc_link_unlock(void);
void erpc_link_opened(int carries_erpc);
void erpc_link_closed(void);

/*
 * ---- which firmware is on the module is a runtime latch ----
 *
 * ONE latch answers every "may we do X on this link?" question, because they all depend
 * on the same fact: WHICH FIRMWARE IS ON THE MODULE.  That is module state, not link
 * state.  Deliberately NOT reset by erpc_link_opened/_closed: `wifi ver`, the call
 * that proves the firmware, drops the UART reference before it returns, so anything tied
 * to the UART "open" would revert the moment it was earned and could never take effect
 * (this is exactly the defect the issue #23 U0-2 plan had).  app/rtl_link.c clears it
 * instead at every point the module's identity can change -- every CHIP_EN transition and
 * every flash write -- the same set of points it already uses for rtl_tcpip_set_inited().
 *
 * @gen is the N of the "2.1.3+wio-nN" build id, 0 when unknown (the safe default, and
 * what the factory / pre-N2 firmware leaves it at).  What it currently gates:
 *   >= 4  the wire budget is lifted to ERPC_WIRE_BUDGET_FAST (n4 owns USI0 behind an
 *         8 kB ring, so the 127-byte input ring that shaped every request size is gone)
 *   >= 5  the LINK-CTRL channel below exists at all
 *   >= 6  the DATA channel exists (`wifi link dbench`)
 *   >= 7  the L2 bridge exists (the tap the host stack runs on -- `wifi connect` arms it)
 * The wire budget is DERIVED here rather than latched separately, so the two can never
 * disagree about the same module.
 */
void     erpc_set_module_gen(uint8_t gen);
uint8_t  erpc_module_gen(void);
uint16_t erpc_wire_budget(void);

/*
 * ---- LINK-CTRL channel (issue #23 U0-3) ---------------------------------------
 *
 * A second frame type multiplexed onto the same UART, owned by the link layer at BOTH
 * ends rather than by eRPC.  It carries the things eRPC cannot: reading the module's own
 * UART counters (its LOG UART is unreachable while the eRPC link is held), changing the
 * baud rate of the link itself, and generating measured traffic.  Adding them as eRPC
 * methods would mean hand-editing the generated server shim, which has bitten this
 * project before (the rpc_system_version erpc_free() bug).
 *
 *   CTRL frame = u16 0xFFFF | u16 body_len | u16 crc16(body) | body[body_len]
 *   body       = u8 cmd | u8 seq | u8 status | u8 rsvd | payload[]
 *
 * 0xFFFF cannot collide with an eRPC frame, whose leading u16 is the message size: no
 * SENDER can produce one that large (a host request is at most 8 + ERPC_REQ_MAX, a module
 * reply at most its 4096-byte MessageBuffer), and both receivers reject anything above
 * 4096 independently.  A desynchronised stream can still align on 0xFFFF by chance, so
 * every CTRL frame is bounded (ERPC_CTRL_MAX), CRC-checked, length-checked per command,
 * and the one command with a real side effect carries a magic word as well.
 *
 * U0 SIMPLIFICATION: CTRL is only issued on a QUIESCENT link (erpc_link_quiescent()), so
 * it consumes no wire budget and needs no interleaving rules.  U1, which multiplexes CTRL
 * with Ethernet frames continuously, has to revisit that.
 */
#define ERPC_CTRL_MAX 1600u              /* bound on body_len, both directions */

enum {
	ERPC_CTRL_PING       = 1,        /* no payload; proves the link at this baud */
	ERPC_CTRL_STATS      = 2,        /* -> u32 LE x 12, see cmd_wifi_link.c */
	ERPC_CTRL_SETBAUD    = 3,        /* u32 baud + u32 magic; ACK on the OLD baud */
	ERPC_CTRL_BENCH      = 4,        /* u32 reply_bytes, u8 seed, u8 rsvd[3], data[] */
	/* DATA channel control (issue #23 U1), firmware wio-n6 and later. */
	ERPC_CTRL_DATA_CFG   = 5,        /* u8 mode, u8 seed, u16 bytes, u32 ms, u32 magic */
	ERPC_CTRL_DATA_STATS = 6,        /* -> u32 LE x 12, see cmd_wifi_link.c */
	/* L2 bridge (issue #23 U2), firmware wio-n7 and later. */
	ERPC_CTRL_ETH_INFO   = 7         /* -> u8 mac[6], u8 flags, u8 rsvd, u32 LE x 8 */
};

/* ETH_INFO flags. */
#define ERPC_ETH_F_BRIDGED      0x01u    /* the module's netif is tapped right now */
#define ERPC_ETH_F_RUNNING      0x02u    /* the radio is up (rltk_wlan_running) */

/* ETH_INFO counters: rx frames/bytes/no-buf/oversize, tx frames/bytes/fail/down. */
#define ERPC_ETH_STAT_WORDS     8u

/* Guards the one CTRL command with a side effect against a chance 0xFFFF alignment. */
#define ERPC_CTRL_SETBAUD_MAGIC 0x42415544u   /* 'BAUD' */

/*
 * DATA_CFG modes.  SINK makes the module accept and count DATA frames; SOURCE makes it
 * generate them for the requested number of milliseconds.  Mode 0 is "off", and its
 * acknowledgement carries a CONTRACT the host relies on when it tears the channel down:
 * the module has stopped its source AND drained its own transmit queue before it answers,
 * so no further DATA frame can be on the way.  See the detach ordering rule in
 * app/link_data.h -- a time-based "it has been quiet for a while" test would not do,
 * because there is no defensible number to wait for.
 */
#define ERPC_CTRL_DATA_MAGIC    0x44415441u   /* 'DATA' */
#define ERPC_DATA_MODE_OFF      0x00u
#define ERPC_DATA_MODE_SINK     0x01u
#define ERPC_DATA_MODE_SOURCE   0x02u
/*
 * BRIDGE (issue #23 U2, firmware wio-n7 and later) taps the module's WiFi netif: frames
 * received from the air arrive as DATA on LINK_DATA_CHAN_ETH, and DATA frames sent on that
 * channel go out over the air.  It is a MODE rather than a command of its own so that
 * OFF's contract above covers the bridge's producer too -- the module's WiFi receive
 * thread -- instead of needing a second teardown protocol that could disagree with it.
 * The @ms field doubles as the module's watchdog: it takes the bridge down on its own that
 * long after CFG, so a host that dies mid-session cannot leave the module forwarding into
 * a link nobody reads while its own lwIP starves.
 */
#define ERPC_DATA_MODE_BRIDGE   0x04u

/*
 * Send one CTRL command and wait for its reply.  Only ONE CTRL exchange may be in flight
 * (there is a single slot), and the caller is expected to hold app/rtl_link.c's coarse
 * mutex, which is what makes that true in practice.
 *
 * Returns the reply payload length (>= 0; bytes copied into @out, truncated to @out_cap),
 * or negative:
 *   -1 bad args / no link / a CTRL exchange is already in flight / no service thread
 *   -2 timeout, or the link was torn down under us (erpc_abandon_all)
 *   -3 the module answered with a non-zero status byte
 *
 * -3 collapses every module-side refusal into one number, which is not enough once a
 * command can refuse for more than one reason: erpc_ctrl_last_status() returns the byte
 * itself so the caller can say WHY (issue #23 U2 -- asking the module to bridge before it
 * has an lwIP netif to tap is a different problem from a malformed request, and the first
 * is what a new user hits; see the status decode in cmd_wifi_link.c's link_data_cfg).
 * Valid immediately after a -3 and nowhere else; it is well defined for the same reason
 * the single reply slot is: one CTRL exchange at a time, held by one owner.
 */
int erpc_ctrl_call(uint8_t cmd, const uint8_t *req, uint16_t req_len,
                   uint8_t *out, uint16_t out_cap, uint32_t timeout_ms,
                   struct erpc_diag *diag);

/* The module's status byte from the exchange that just returned -3.  0 otherwise. */
uint8_t erpc_ctrl_last_status(void);

/*
 * ---- DATA channel (issue #23 U1) ------------------------------------------------
 *
 * The third frame type on this wire lives in app/link_data.{c,h}; the service thread
 * here is what actually reads and writes it, exactly as it does for eRPC and CTRL.  The
 * only thing link_data needs from this side is a way to wake the thread: it parks on
 * TX_WAIT_FOREVER whenever it believes nothing is outstanding, so a frame queued from
 * another thread would otherwise sit there.
 *
 * Deliberately NOT the mutex-taking kind of call: link_data_send() may run with a
 * network stack's own lock held (U3), so it must not be able to block on this module.
 */
void erpc_data_posted(void);

/*
 * True when NO DATA frame is anywhere in the host's pipeline: none queued to send, none
 * waiting to be delivered, none half-received.
 *
 * Deliberately NOT erpc_link_quiescent() -- that answers "is the whole link idle?", which
 * additionally requires no eRPC request to be outstanding.  Something normally is: the
 * interface owner refreshes the association over eRPC on its own schedule, and back when
 * TCP still ran on the module the telnet console kept a blocking accept outstanding by
 * design (issue #21).  Asking the stronger question when tearing the DATA channel down
 * would therefore fail for reasons that have nothing to do with DATA.  This is the half of
 * it that matters: the detach ordering rule in app/link_data.h needs the DATA channel
 * quiet, not the link idle.
 */
int erpc_data_quiescent(void);

/*
 * True when nothing at all is outstanding on the link: no eRPC token, no CTRL exchange,
 * no queued or half-received DATA frame, and no request bytes the module has not
 * answered.  `wifi link baud` requires it -- changing the baud rate under an in-flight frame
 * would corrupt it at both ends -- and `wifi link info` uses it to decide whether it may ask
 * the module anything.
 */
int erpc_link_quiescent(void);

/*
 * Perform one eRPC invocation on the currently-open RTL8720 UART and wait for the
 * matching reply.  The service thread sends the request frame, then reads reply frames
 * until one matches (type=reply, same service/request/sequence) or @timeout_ms elapses.
 * Non-matching frames are categorised into @diag (may be NULL) and skipped:
 * stale replies, unsupported device->host invocations, oversize (drained), CRC fails.
 *
 * The reply payload (bytes after the 8-byte reply header) is copied into @out,
 * truncated to @out_cap.  Returns the full payload length (>=0) on a matched reply,
 * or negative on failure:
 *   -1 request too large / bad args / no free slot / service thread missing
 *   -2 timeout waiting for the reply     -4 aborted by @should_abort (erpc_call_ex)
 * A negative return may also come from a send with no open UART.
 *
 * NOTE: abort/timeout only stops the HOST-side wait; the RTL8720DN keeps executing an
 * in-flight call (e.g. connect / DHCP).  A subsequent call resyncs via the leading
 * rx-flush (done when nothing is on the wire) + the strict (type,service,request,seq)
 * match, which drops the module's late reply.
 */
int erpc_call(uint8_t service, uint8_t request,
              const uint8_t *req, uint16_t req_len,
              uint8_t *out, uint16_t out_cap, uint32_t timeout_ms,
              struct erpc_diag *diag);

/*
 * As erpc_call(), plus an optional cancellation hook: @should_abort(@abort_ctx) is
 * polled once per millisecond while waiting; if it returns non-zero the call returns
 * -4 without waiting out the timeout.  Used for the multi-second connect / DHCP calls
 * so Ctrl+C can interrupt them.  @should_abort == NULL behaves exactly like erpc_call.
 * The hook always runs on the CALLING thread (the shell passes a thunk over
 * cli_cancel_requested(), which drains its console), never on the service thread.
 */
int erpc_call_ex(uint8_t service, uint8_t request,
                 const uint8_t *req, uint16_t req_len,
                 uint8_t *out, uint16_t out_cap, uint32_t timeout_ms,
                 struct erpc_diag *diag,
                 int (*should_abort)(void *ctx), void *abort_ctx);

/*
 * ---- Asynchronous, multi-in-flight API (issue #20 N3) --------------------------
 *
 * These let a caller keep several requests outstanding at once -- what the N3
 * worker-dispatch firmware enables.  Since increment 8 they are thread-safe: the slot
 * table is mutex-protected and the shared receive dispatcher runs on the service
 * thread.  A TOKEN, however, is a single-consumer capability: exactly one thread may
 * wait on it (a second erpc_wait() on the same token returns -1), and only that thread
 * may cancel it.
 *
 * Lifecycle:
 *   int t = erpc_begin(svc, req, body, len, out, out_cap);   // send, get a token
 *   ... optionally begin / wait other tokens ...
 *   int n = erpc_wait(t, timeout_ms, diag, abort, ctx);      // collect t's reply
 *
 * @out (given to erpc_begin) must stay valid, and must not alias another live token's
 * @out, until the token is waited or cancelled -- the service thread copies the reply
 * payload straight into it, truncated to @out_cap (erpc_wait returns the FULL payload
 * length).  A failed erpc_wait() detaches @out before it returns, so a caller may let
 * a stack buffer go out of scope right after any erpc_wait() outcome.
 */

/* Send an invocation and return a token >= 0 to wait on later; -1 on bad args (request
 * larger than ERPC_REQ_MAX), no free slot (ERPC_MAX_INFLIGHT already outstanding) or no
 * service thread.  Does not wait.  Stale RX is dropped when nothing is on the wire, so
 * a fresh call resyncs without ever discarding another live token's buffered reply. */
int erpc_begin(uint8_t service, uint8_t request,
               const uint8_t *req, uint16_t req_len,
               uint8_t *out, uint16_t out_cap);

/* Wait for @token's reply.  Returns its payload length (>= 0; the bytes are already in
 * the @out passed to erpc_begin, truncated to @out_cap) and releases the token; or
 *   -1 invalid / stale token, or another thread is already waiting on it
 *   -2 timeout, or the link was torn down under us (erpc_abandon_all)
 *   -4 aborted by @should_abort
 * On ANY of those the token is invalidated and released too (its reply buffer is
 * detached first, so a late reply is dropped as stale rather than written into a dead
 * stack frame).  erpc_cancel() afterwards is a harmless no-op, and the token must not
 * be waited on again.  @diag / @should_abort behave as in erpc_call_ex. */
int erpc_wait(int token, uint32_t timeout_ms, struct erpc_diag *diag,
              int (*should_abort)(void *ctx), void *abort_ctx);

/* Release @token without waiting for its reply: the reply buffer is detached and a
 * reply that arrives afterwards is dropped (counted as a skipped reply).  Idempotent --
 * safe on a token erpc_wait() already invalidated. */
void erpc_cancel(int token);

/* rpc_system_ack(c): round-trips one byte through the firmware (service 1, req 2).
 * On success returns 0 and stores the echoed byte in *echoed; negative on failure. */
int erpc_system_ack(uint8_t c, uint8_t *echoed, struct erpc_diag *diag);

/* rpc_system_version(): asks the module for its firmware build-id string (service 1,
 * req 1).  Returns the string length and writes a NUL-terminated copy to @out
 * (truncated to @out_cap-1); negative on failure (-2 timeout / -3 malformed / -4
 * aborted).
 *
 * WARNING -- only send this to issue-#20 N2+ firmware.  The factory / N1 server shim
 * erpc_free()s a string literal when version is invoked, which corrupts the module's
 * heap regardless of what the host does with the reply; that is why increment 1 never
 * called it.  N2 (patch 0002) makes the shim's target a heap copy ("2.1.3+wio-n2"), so
 * from N2 on this is safe.  Board #2 currently runs n8 ("2.1.3+wio-n8", issue #31). */
int erpc_system_version(char *out, uint16_t out_cap, struct erpc_diag *diag);

#endif /* APP_ERPC_H */
