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
 * clock tree, so it is XIP-safe.
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

/* Maximum eRPC requests that may be outstanding at once (erpc_begin without a matching
 * erpc_wait/erpc_cancel yet).  Each live token costs one slot (metadata + a request
 * staging buffer) in AXI-SRAM static BSS. */
#define ERPC_MAX_INFLIGHT 4

/* Largest request parameter block one call may carry.  The service thread sends from a
 * per-slot buffer of 8 + ERPC_REQ_MAX bytes (8 = eRPC message header + sequence), so a
 * caller's own request bytes never have to outlive its erpc_begin().  The biggest
 * request in the tree is wifi_rpc_lwip_send()'s 12 + WIFI_RPC_STREAM_MAX = 268 B
 * (app/wifi_rpc.c static-asserts against this). */
#define ERPC_REQ_MAX      320u

/*
 * Bytes of request frames the service thread may have on the wire at once.
 *
 * The module's eRPC transport reads the UART one byte at a time behind an Arduino
 * RingBuffer of 127 usable bytes that SILENTLY DROPS a byte when full (see the
 * ASYMMETRY note in app/wifi_rpc.h): a frame whose bytes cannot all fit in that ring
 * is only delivered if the module's reader keeps up, which is load-dependent.  The
 * only structural guarantee available is "everything we have sent and not yet seen
 * answered fits in the ring", so the service thread sends a queued frame only when
 * either nothing is on the wire (a lone frame behaves exactly as it did before this
 * budget existed -- including the deliberately oversized ones `net echo <port> 256`
 * uses to reproduce the measured cliff) or the frame still fits this budget.
 * A reply proves the module consumed that request's bytes, which is what frees them.
 */
#define ERPC_WIRE_BUDGET  127u

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
 * from N2 on this is safe.  The board runs N3 ("2.1.3+wio-n3"). */
int erpc_system_version(char *out, uint16_t out_cap, struct erpc_diag *diag);

#endif /* APP_ERPC_H */
