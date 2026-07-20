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
 * Wire format (reference: _ref/seeed-ambd-firmware, git-ignored):
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
 */
#ifndef APP_ERPC_H
#define APP_ERPC_H

#include <stdint.h>
#include <stddef.h>

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
 * or a negative value on timeout / send failure / no open UART.
 */
int erpc_call(uint8_t service, uint8_t request,
              const uint8_t *req, uint16_t req_len,
              uint8_t *out, uint16_t out_cap, uint32_t timeout_ms,
              struct erpc_diag *diag);

/* rpc_system_ack(c): round-trips one byte through the firmware (service 1, req 2).
 * On success returns 0 and stores the echoed byte in *echoed; negative on failure. */
int erpc_system_ack(uint8_t c, uint8_t *echoed, struct erpc_diag *diag);

#endif /* APP_ERPC_H */
