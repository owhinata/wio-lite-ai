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
 * NOTE (increment 1): only rpc_system_ack is used.  rpc_system_version is NOT called
 * -- the factory firmware's generated server shim erpc_free()s a string literal
 * (rpc_system_server.cpp), which would corrupt its heap.  And device->host callbacks
 * are NOT serviced yet: the factory client waits forever for a reply, so we only use
 * APIs (ack) that trigger no callback.  A host server stub comes in a later increment.
 *
 * NOTE (issue #20 N3): the N3 device firmware replaces the stock single-task serial
 * eRPC server with a receive-task + worker-pool dispatcher, so it can serve several
 * requests at once and replies may come back OUT OF ORDER.  To use that, this client
 * grows a small asynchronous API (erpc_begin / erpc_wait / erpc_cancel, below) that
 * keeps up to ERPC_MAX_INFLIGHT requests outstanding from a single owner thread.  The
 * synchronous erpc_call()/erpc_call_ex() are unchanged in behaviour -- they are now
 * thin begin+wait wrappers over the same shared, sequence-routed receive dispatcher.
 */
#ifndef APP_ERPC_H
#define APP_ERPC_H

#include <stdint.h>
#include <stddef.h>

/* Maximum eRPC requests that may be outstanding at once (erpc_begin without a matching
 * erpc_wait/erpc_cancel yet).  Each live token costs one small slot (metadata + the
 * caller's reply pointer) in AXI-SRAM static BSS. */
#define ERPC_MAX_INFLIGHT 4

/* Per-call receive diagnostics (why a link test failed / what was skipped). */
struct erpc_diag {
	uint16_t crc_fail;               /* frames with a bad CRC / malformed header */
	uint16_t oversize;               /* frames larger than the RX scratch (drained) */
	uint16_t timeout;                /* ran past the deadline waiting for bytes */
	uint16_t skipped_reply;          /* reply frames for a different sequence (stale) */
	uint16_t unsupported_invocation; /* device->host request frames we cannot serve */
};

/* eRPC CRC-16 (init 0xEF4A, poly 0x1021, MSB-first) over @n bytes of @d. */
uint16_t erpc_crc16(const uint8_t *d, uint16_t n);

/*
 * Perform one eRPC invocation on the currently-open RTL8720 UART and wait for the
 * matching reply.  Builds/sends the request frame, then reads reply frames until one
 * matches (type=reply, same service/request/sequence) or @timeout_ms elapses.
 * Non-matching frames are categorised into @diag (may be NULL) and skipped:
 * stale replies, unsupported device->host invocations, oversize (drained), CRC fails.
 *
 * The reply payload (bytes after the 8-byte reply header) is copied into @out,
 * truncated to @out_cap.  Returns the full payload length (>=0) on a matched reply,
 * or negative on failure:
 *   -1 request too large / bad args     -2 timeout waiting for the reply
 *   -4 aborted by @should_abort (erpc_call_ex only)
 * A negative return may also come from a send with no open UART.
 *
 * NOTE: abort/timeout only stops the HOST-side wait; the RTL8720DN keeps executing
 * an in-flight call (e.g. connect / DHCP).  Because its eRPC server is single-
 * threaded it will not service the next request until that finishes -- a subsequent
 * call resyncs via the leading rx-flush + strict (type,service,request,seq) match,
 * which drains the module's late reply.  Callers should serialise calls (one owner
 * at a time; the shell uses cli_console_claim) since the RX ring is strict SPSC.
 */
int erpc_call(uint8_t service, uint8_t request,
              const uint8_t *req, uint16_t req_len,
              uint8_t *out, uint16_t out_cap, uint32_t timeout_ms,
              struct erpc_diag *diag);

/*
 * As erpc_call(), plus an optional cancellation hook: @should_abort(@abort_ctx) is
 * polled on each receive-loop yield; if it returns non-zero the call returns -4
 * without waiting out the timeout.  Used for the multi-second connect / DHCP calls
 * so Ctrl+C can interrupt them.  @should_abort == NULL behaves exactly like erpc_call.
 * erpc.c stays shell-agnostic: the shell passes a thunk over cli_cancel_requested().
 */
int erpc_call_ex(uint8_t service, uint8_t request,
                 const uint8_t *req, uint16_t req_len,
                 uint8_t *out, uint16_t out_cap, uint32_t timeout_ms,
                 struct erpc_diag *diag,
                 int (*should_abort)(void *ctx), void *abort_ctx);

/*
 * ---- Asynchronous, multi-in-flight API (issue #20 N3) --------------------------
 *
 * These let one owner thread keep several requests outstanding at once -- what the N3
 * worker-dispatch firmware enables.  They are NOT thread-safe: exactly like the
 * synchronous calls they assume a single console owner (cli_console_claim), and
 * begin/wait/cancel must all run on that one thread.  Do not mix these with a call on
 * another thread.
 *
 * Lifecycle:
 *   int t = erpc_begin(svc, req, body, len, out, out_cap);   // send, get a token
 *   ... optionally begin / wait other tokens ...
 *   int n = erpc_wait(t, timeout_ms, diag, abort, ctx);      // collect t's reply
 * erpc_wait() drives a shared receive dispatcher: it reads reply frames and routes each
 * to whichever live token's sequence it carries (so waiting on one token still delivers
 * another's reply into that token's @out), until @token's own reply lands or the
 * deadline / abort fires.
 *
 * @out (given to erpc_begin) must stay valid and must not alias another live token's
 * @out until the token is waited or cancelled -- the dispatcher copies the reply payload
 * straight into it, truncated to @out_cap (erpc_wait returns the FULL payload length).
 */

/* Send an invocation and return a token >= 0 to wait on later; -1 on bad args (request
 * larger than the scratch) or no free slot (ERPC_MAX_INFLIGHT already outstanding).
 * Does not wait.  Unlike a fresh erpc_call, begin only flushes stale RX when nothing
 * else is in flight, so it never discards another live token's buffered reply. */
int erpc_begin(uint8_t service, uint8_t request,
               const uint8_t *req, uint16_t req_len,
               uint8_t *out, uint16_t out_cap);

/* Wait for @token's reply.  Returns its payload length (>= 0; the bytes are already in
 * the @out passed to erpc_begin, truncated to @out_cap) and releases the token; or a
 * negative code WITHOUT releasing it: -1 invalid/stale token, -2 timeout, -4 aborted.
 * On a negative return the caller must erpc_cancel(@token) (or erpc_wait again).
 * @diag / @should_abort behave as in erpc_call_ex. */
int erpc_wait(int token, uint32_t timeout_ms, struct erpc_diag *diag,
              int (*should_abort)(void *ctx), void *abort_ctx);

/* Release @token without waiting for its reply.  A reply that arrives for it afterwards
 * is dropped by a later erpc_wait()'s dispatcher (counted as a skipped reply). */
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
 * from N2 on this is safe.  The board runs N2 after issue #20's N2 milestone. */
int erpc_system_version(char *out, uint16_t out_cap, struct erpc_diag *diag);

#endif /* APP_ERPC_H */
