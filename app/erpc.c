/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/*
 * Minimal clean-room eRPC client for the onboard RTL8720DN (issue #5).
 * See erpc.h for the wire format and the increment-1 scope/limitations.
 *
 * Runs in the shell command thread (post-kernel): it uses ThreadX time/sleep for a
 * bounded, yielding receive, and the #17 rtl8720 UART driver for the raw bytes.
 * Register-agnostic (no RCC / peripheral setup here) -- XIP-safe.
 */
#include "tx_api.h"       /* tx_time_get / tx_thread_sleep (1 tick = 1 ms here) */
#include "rtl8720.h"      /* rtl8720_uart_read / _write (USART1 @2 Mbaud, #17) */
#include "erpc.h"

#include <string.h>

/* BasicCodec / message constants (erpc_basic_codec.cpp, erpc_codec.h). */
#define ERPC_CODEC_VERSION   1u
#define ERPC_MSGTYPE_REPLY   2u          /* invocation=0, oneway=1, reply=2, notif=3 */

/* rpc_system interface (rpc_system.h). */
#define ERPC_SYS_SERVICE     1u
#define ERPC_SYS_ACK_REQ     2u

/* Receive scratch: every frame is read here first, independent of the caller's
 * out_cap, and matches the firmware's MessageBuffer size (erpc_config).  Static
 * BSS in AXI-SRAM; CPU-only (no DMA) so it is D-cache coherent. */
#define ERPC_RX_SCRATCH      4096u
static uint8_t erpc_scratch[ERPC_RX_SCRATCH];

static uint32_t erpc_seq;                /* monotonically-increasing request sequence */

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

/* True once the ThreadX tick count has reached/passed @deadline (wrap-safe). */
static int erpc_expired(ULONG deadline)
{
	return (int32_t)(tx_time_get() - deadline) >= 0;
}

/* Read exactly @n bytes into @buf by the @deadline; 0 on success, -1 on timeout. */
static int erpc_recv_exact(uint8_t *buf, uint16_t n, ULONG deadline)
{
	uint16_t got = 0u;

	while (got < n) {
		got += (uint16_t)rtl8720_uart_read(buf + got, (size_t)(n - got));
		if (got >= n)
			break;
		if (erpc_expired(deadline))
			return -1;
		tx_thread_sleep(1);          /* yield ~1 ms while the bytes arrive */
	}
	return 0;
}

/* Discard @count incoming bytes (resync after an oversize frame); 0 / -1 timeout. */
static int erpc_drain(uint32_t count, ULONG deadline)
{
	uint8_t tmp[64];

	while (count != 0u) {
		uint16_t chunk = (count < sizeof(tmp)) ? (uint16_t)count : (uint16_t)sizeof(tmp);
		uint16_t r = (uint16_t)rtl8720_uart_read(tmp, chunk);
		count -= r;
		if (count == 0u)
			break;
		if (erpc_expired(deadline))
			return -1;
		tx_thread_sleep(1);
	}
	return 0;
}

/* Drop any bytes still buffered so a call starts from a clean frame boundary
 * (discards stale/partial frames or a late reply to a previous, timed-out call).
 * Bounded to one scratch-full so a continuously-streaming peer (wrong baud) cannot
 * spin here -- the receive loop's deadline then bounds the rest. */
static void erpc_rx_flush(void)
{
	uint8_t tmp[64];
	uint32_t budget = ERPC_RX_SCRATCH;
	size_t r;

	while (budget != 0u && (r = rtl8720_uart_read(tmp, sizeof(tmp))) > 0u)
		budget -= (r > budget) ? budget : (uint32_t)r;
}

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

int erpc_call(uint8_t service, uint8_t request,
              const uint8_t *req, uint16_t req_len,
              uint8_t *out, uint16_t out_cap, uint32_t timeout_ms,
              struct erpc_diag *diag)
{
	uint8_t frame_hdr[4];
	uint32_t msg_hdr;
	uint32_t seq;
	uint16_t body_len;
	uint16_t crc;
	ULONG deadline;

	if (diag)
		memset(diag, 0, sizeof(*diag));

	if (req_len > (uint16_t)(ERPC_RX_SCRATCH - 8u))   /* guard 8+req_len overflow */
		return -1;
	body_len = (uint16_t)(8u + req_len);

	/* Build the request body in the scratch (reused for RX after the send). */
	seq = ++erpc_seq;
	msg_hdr = ((uint32_t)ERPC_CODEC_VERSION << 24) | ((uint32_t)service << 16) |
	          ((uint32_t)request << 8) | 0u /* kInvocationMessage */;
	put_u32le(erpc_scratch + 0, msg_hdr);
	put_u32le(erpc_scratch + 4, seq);
	if (req_len)
		memcpy(erpc_scratch + 8, req, req_len);
	crc = erpc_crc16(erpc_scratch, body_len);

	/* Start from a clean RX boundary, then send {size,crc} header + body. */
	erpc_rx_flush();
	put_u16le(frame_hdr + 0, body_len);
	put_u16le(frame_hdr + 2, crc);
	rtl8720_uart_write(frame_hdr, 4u);
	rtl8720_uart_write(erpc_scratch, body_len);

	deadline = tx_time_get() + (ULONG)timeout_ms;   /* 1 tick = 1 ms */

	for (;;) {
		uint16_t rsize, rcrc;
		uint32_t rhdr, rseq;

		if (erpc_recv_exact(frame_hdr, 4u, deadline)) {
			if (diag) diag->timeout++;
			return -2;
		}
		rsize = get_u16le(frame_hdr + 0);
		rcrc  = get_u16le(frame_hdr + 2);

		if (rsize > ERPC_RX_SCRATCH) {          /* too big: drain to resync */
			if (diag) diag->oversize++;
			if (erpc_drain(rsize, deadline)) {
				if (diag) diag->timeout++;
				return -2;
			}
			continue;
		}
		if (erpc_recv_exact(erpc_scratch, rsize, deadline)) {
			if (diag) diag->timeout++;
			return -2;
		}
		if (erpc_crc16(erpc_scratch, rsize) != rcrc || rsize < 8u) {
			if (diag) diag->crc_fail++;
			continue;
		}

		rhdr = get_u32le(erpc_scratch + 0);
		rseq = get_u32le(erpc_scratch + 4);
		if (((rhdr >> 24) & 0xffu) != ERPC_CODEC_VERSION) {
			if (diag) diag->crc_fail++;         /* wrong version = malformed */
			continue;
		}
		if ((rhdr & 0xffu) != ERPC_MSGTYPE_REPLY) {
			if (diag) diag->unsupported_invocation++;   /* device->host request */
			continue;
		}
		if (((rhdr >> 16) & 0xffu) != service ||
		    ((rhdr >> 8) & 0xffu) != request || rseq != seq) {
			if (diag) diag->skipped_reply++;    /* reply for something else */
			continue;
		}

		/* Matched reply: copy the payload after the 8-byte header. */
		{
			uint16_t plen = (uint16_t)(rsize - 8u);
			uint16_t copy = (plen < out_cap) ? plen : out_cap;
			if (out && copy)
				memcpy(out, erpc_scratch + 8, copy);
			return (int)plen;
		}
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
