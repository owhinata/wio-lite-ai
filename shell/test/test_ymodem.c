/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 *
 * Host unit test for the clean-room YMODEM-CRC sender (issue #50, svc/ymodem.c).
 * Pure: no HAL/ThreadX/shell.  Drives ymodem_send() against a reactive mock
 * receiver (a response FIFO that scales with the block count) and a mock byte
 * source, then parses the captured TX stream into frames and asserts:
 *   A. CRC-16/CCITT vectors,
 *   B. happy path: block 0 (name + decimal size), one short SOH block padded
 *      with 0x1A, EOT, and the closing null block,
 *   C. exact 1024 -> a single STX block, no short block (the camera-frame path),
 *   D. 1025 -> STX(1024) + SOH(128) short block, 0x1A padded,
 *   E. 153600 (QVGA RGB565 frame) -> 150 STX blocks, no padding,
 *   F. NAK -> the block is resent and the transfer still completes,
 *   G. CAN CAN -> YM_ERR_CANCEL and the CAN teardown is emitted,
 *   H. a 1-byte-at-a-time source still fills full blocks (core read-loop),
 *   I. seq wraps mod 256 past block 255,
 *   J. a source read fault -> YM_ERR_SOURCE and the CAN teardown is emitted,
 *   K. a silent receiver -> YM_ERR_TIMEOUT and the CAN teardown is emitted.
 */
#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ymodem.h"

/* protocol bytes (mirrored from ymodem.c for the test's own assertions) */
#define SOH 0x01
#define STX 0x02
#define EOT 0x04
#define ACK 0x06
#define NAK 0x15
#define CAN 0x18
#define CRC_C 0x43
#define CPMEOF 0x1A

/* ---- mock byte source: dst[j] = (pos + j) & 0xFF -------------------------- */
struct src_ctx {
	uint32_t pos;
	uint32_t size;
	uint32_t chunk;     /* 0 = unlimited; N = at most N bytes per read */
	uint32_t fail_at;   /* 0 = never; else read returns -1 once pos >= fail_at */
};

static int src_read(void *ctx, uint8_t *dst, uint32_t want, uint32_t *got)
{
	struct src_ctx *s = ctx;
	uint32_t rem;
	if (s->fail_at && s->pos >= s->fail_at)
		return -1;                       /* simulate a read fault */
	rem = s->size - s->pos;
	uint32_t n = want;
	if (n > rem)
		n = rem;
	if (s->chunk && n > s->chunk)
		n = s->chunk;
	for (uint32_t i = 0; i < n; i++)
		dst[i] = (uint8_t)((s->pos + i) & 0xFFu);
	s->pos += n;
	*got = n;
	return 0;
}

/* ---- reactive mock receiver ---------------------------------------------- */
struct mock {
	uint8_t  resp[64];        /* response FIFO (>=2 pending is enough) */
	int      rhead, rtail;
	int      phase;           /* 0=expect block0, 1=data/EOT, 2=null block */
	int      put_count;       /* number of put() calls (block/EOT sends) */
	int      nak_at;          /* one-shot NAK at this put index, or -1 */
	int      can_at;          /* CAN CAN at this put index, or -1 */
	int      silent_at;       /* from this put index on, queue no response (-1=off) */
	uint8_t  tx[320000];      /* captured TX stream */
	size_t   txlen;
};

static void resp_push(struct mock *m, uint8_t b)
{
	m->resp[m->rtail] = b;
	m->rtail = (m->rtail + 1) % 64;
	assert(m->rtail != m->rhead);   /* FIFO overflow == test bug */
}

static int mock_getc(void *ctx, unsigned timeout_ms)
{
	struct mock *m = ctx;
	(void)timeout_ms;
	if (m->rhead != m->rtail) {
		int b = m->resp[m->rhead];
		m->rhead = (m->rhead + 1) % 64;
		return b;
	}
	return YM_IO_TIMEOUT;
}

static int mock_put(void *ctx, const uint8_t *buf, size_t len)
{
	struct mock *m = ctx;
	uint8_t kind;

	memcpy(m->tx + m->txlen, buf, len);
	m->txlen += len;

	/* The cancel teardown (leading CAN) draws no response. */
	if (buf[0] == CAN)
		return 0;

	m->put_count++;

	if (m->put_count == m->can_at) {
		resp_push(m, CAN);
		resp_push(m, CAN);
		return 0;
	}
	if (m->put_count == m->nak_at) {
		resp_push(m, NAK);       /* one-shot: the resend gets a normal ACK */
		return 0;
	}
	if (m->silent_at >= 0 && m->put_count >= m->silent_at)
		return 0;                /* queue nothing -> getc times out */

	kind = buf[0];
	if (kind == EOT) {
		resp_push(m, ACK);
		resp_push(m, CRC_C);     /* receiver then requests the null block */
		m->phase = 2;
		return 0;
	}
	/* a data/SOH/STX block */
	if (m->phase == 0) {
		resp_push(m, ACK);
		resp_push(m, CRC_C);     /* ACK block0, then request block 1 */
		m->phase = 1;
	} else {
		resp_push(m, ACK);       /* data block (phase 1) or null block (phase 2) */
	}
	return 0;
}

static void mock_init(struct mock *m)
{
	memset(m, 0, sizeof *m);
	m->nak_at = -1;
	m->can_at = -1;
	m->silent_at = -1;
	resp_push(m, CRC_C);             /* receiver spontaneously requests CRC mode */
}

/* ---- TX-stream parser ----------------------------------------------------- */
struct frame {
	uint8_t  kind;       /* SOH / STX / EOT */
	uint8_t  seq;
	uint16_t datalen;    /* 0 for EOT */
	const uint8_t *data; /* into mock.tx */
};

/* Parse blocks/EOT out of tx until a non-frame byte (e.g. the CAN teardown) or
 * the end.  Verifies ~seq and the CRC of every block.  Returns frame count. */
static int parse_frames(const uint8_t *tx, size_t len, struct frame *out, int max)
{
	size_t i = 0;
	int    n = 0;
	while (i < len && n < max) {
		uint8_t kind = tx[i];
		uint16_t dl;
		uint16_t crc, got;
		if (kind == EOT) {
			out[n].kind = EOT; out[n].seq = 0;
			out[n].datalen = 0; out[n].data = NULL;
			n++; i++;
			continue;
		}
		if (kind == SOH) dl = 128;
		else if (kind == STX) dl = 1024;
		else break;                       /* CAN teardown or garbage */
		assert(i + 3 + dl + 2 <= len);
		assert((uint8_t)~tx[i + 1] == tx[i + 2]);   /* seq / ~seq */
		crc = ymodem_crc16(tx + i + 3, dl);
		got = (uint16_t)((tx[i + 3 + dl] << 8) | tx[i + 3 + dl + 1]);
		assert(crc == got);
		out[n].kind = kind;
		out[n].seq = tx[i + 1];
		out[n].datalen = dl;
		out[n].data = tx + i + 3;
		n++;
		i += 3 + dl + 2;
	}
	return n;
}

/* ---- shared run helper ---------------------------------------------------- */
static enum ym_result run(struct mock *m, struct src_ctx *s, const char *name)
{
	struct ym_io io = { m, mock_getc, mock_put };
	struct ym_source src = { s, name, s->size, src_read };
	return ymodem_send(&io, &src);
}

int main(void)
{
	/* ---- A. CRC vectors ---- */
	assert(ymodem_crc16((const uint8_t *)"123456789", 9) == 0x31C3);
	assert(ymodem_crc16((const uint8_t *)"", 0) == 0x0000);
	{
		uint8_t z[1] = { 0 };
		assert(ymodem_crc16(z, 1) == 0x0000);
		uint8_t a[1] = { 'A' };
		assert(ymodem_crc16(a, 1) == 0x58E5);
	}

	static struct mock m;
	struct frame f[300];
	int nf;

	/* ---- B. happy path, size 100 -> one short SOH block, 0x1A padded ---- */
	{
		struct src_ctx s = { 0, 100, 0, 0 };
		mock_init(&m);
		assert(run(&m, &s, "frame.raw") == YM_OK);
		nf = parse_frames(m.tx, m.txlen, f, 300);
		/* block0, block1(SOH/128), EOT, null block0 */
		assert(nf == 4);
		assert(f[0].kind == SOH && f[0].seq == 0);
		/* block0 payload: "frame.raw\0100\0..." */
		assert(memcmp(f[0].data, "frame.raw", 10) == 0);   /* incl NUL */
		assert(memcmp(f[0].data + 10, "100", 4) == 0);     /* incl NUL */
		assert(f[1].kind == SOH && f[1].seq == 1);
		for (int j = 0; j < 100; j++)
			assert(f[1].data[j] == (uint8_t)(j & 0xFF));
		for (int j = 100; j < 128; j++)
			assert(f[1].data[j] == CPMEOF);                /* padding */
		assert(f[2].kind == EOT);
		assert(f[3].kind == SOH && f[3].seq == 0);
		for (int j = 0; j < 128; j++)
			assert(f[3].data[j] == 0);                     /* null block */
	}

	/* ---- C. size exactly 1024 -> single STX block, no short block ---- */
	{
		struct src_ctx s = { 0, 1024, 0, 0 };
		mock_init(&m);
		assert(run(&m, &s, "a") == YM_OK);
		nf = parse_frames(m.tx, m.txlen, f, 300);
		assert(nf == 4);                                   /* b0, STX, EOT, null */
		assert(f[1].kind == STX && f[1].seq == 1);
		for (int j = 0; j < 1024; j++)
			assert(f[1].data[j] == (uint8_t)(j & 0xFF));
		assert(f[2].kind == EOT);
	}

	/* ---- D. size 1025 -> STX(1024) + SOH(128) short block ---- */
	{
		struct src_ctx s = { 0, 1025, 0, 0 };
		mock_init(&m);
		assert(run(&m, &s, "a") == YM_OK);
		nf = parse_frames(m.tx, m.txlen, f, 300);
		assert(nf == 5);                                   /* b0, STX, SOH, EOT, null */
		assert(f[1].kind == STX && f[1].seq == 1);
		assert(f[2].kind == SOH && f[2].seq == 2);
		assert(f[2].data[0] == (uint8_t)(1024 & 0xFF));    /* the 1025th byte */
		for (int j = 1; j < 128; j++)
			assert(f[2].data[j] == CPMEOF);
		assert(f[3].kind == EOT);
	}

	/* ---- E. 153600 (QVGA RGB565 frame) -> 150 STX blocks, no padding ---- */
	{
		struct src_ctx s = { 0, 153600, 0, 0 };
		mock_init(&m);
		assert(run(&m, &s, "frame.raw") == YM_OK);
		nf = parse_frames(m.tx, m.txlen, f, 300);
		assert(nf == 153 /* b0 + 150 + EOT + null */);
		for (int k = 1; k <= 150; k++) {
			assert(f[k].kind == STX);
			assert(f[k].seq == (uint8_t)(k & 0xFF));
		}
		assert(f[151].kind == EOT);
		assert(f[152].kind == SOH && f[152].seq == 0);
	}

	/* ---- F. NAK on the data block -> resent, transfer still OK ---- */
	{
		struct src_ctx s = { 0, 50, 0, 0 };
		mock_init(&m);
		m.nak_at = 2;                /* put #1 = block0, #2 = data block */
		assert(run(&m, &s, "a") == YM_OK);
		nf = parse_frames(m.tx, m.txlen, f, 300);
		/* block0, data(NAKed), data(resent), EOT, null = 5 */
		assert(nf == 5);
		assert(f[1].kind == SOH && f[1].seq == 1);
		assert(f[2].kind == SOH && f[2].seq == 1);   /* same block resent */
		assert(f[3].kind == EOT);
	}

	/* ---- G. CAN CAN -> cancel, teardown emitted ---- */
	{
		struct src_ctx s = { 0, 50, 0, 0 };
		mock_init(&m);
		m.can_at = 2;                /* abort the data block */
		assert(run(&m, &s, "a") == YM_ERR_CANCEL);
		/* the tail of the TX stream must contain the CAN teardown */
		int cans = 0;
		for (size_t i = 0; i < m.txlen; i++)
			if (m.tx[i] == CAN)
				cans++;
		assert(cans >= 5);
	}

	/* ---- H. 1-byte-at-a-time source still fills full blocks ---- */
	{
		struct src_ctx s = { 0, 300, 1 /* chunk */, 0 };
		mock_init(&m);
		assert(run(&m, &s, "a") == YM_OK);
		nf = parse_frames(m.tx, m.txlen, f, 300);
		assert(nf == 4);                                   /* b0, STX(300<=1024), EOT, null */
		assert(f[1].kind == STX && f[1].seq == 1);
		for (int j = 0; j < 300; j++)
			assert(f[1].data[j] == (uint8_t)(j & 0xFF));
		for (int j = 300; j < 1024; j++)
			assert(f[1].data[j] == CPMEOF);
	}

	/* ---- I. seq wraps mod 256 past block 255 ---- */
	{
		struct src_ctx s = { 0, 260u * 1024u, 0, 0 };
		mock_init(&m);
		assert(run(&m, &s, "big") == YM_OK);
		nf = parse_frames(m.tx, m.txlen, f, 300);
		assert(nf == 263);                                 /* b0 + 260 + EOT + null */
		assert(f[255].seq == 255);                         /* data block 255 */
		assert(f[256].seq == 0);                           /* wrap */
		assert(f[257].seq == 1);
	}

	/* ---- J. source read fault -> YM_ERR_SOURCE + CAN teardown ---- */
	{
		struct src_ctx s = { 0, 4096, 0, 2048 };   /* fault at byte 2048 */
		int cans = 0;
		mock_init(&m);
		assert(run(&m, &s, "a") == YM_ERR_SOURCE);
		for (size_t i = 0; i < m.txlen; i++)
			if (m.tx[i] == CAN)
				cans++;
		assert(cans >= 5);                                 /* teardown emitted */
	}

	/* ---- K. receiver goes silent -> YM_ERR_TIMEOUT + CAN teardown ---- */
	{
		struct src_ctx s = { 0, 50, 0, 0 };
		int cans = 0;
		mock_init(&m);
		m.silent_at = 2;          /* ACK block0 (#1), then never respond */
		assert(run(&m, &s, "a") == YM_ERR_TIMEOUT);
		for (size_t i = 0; i < m.txlen; i++)
			if (m.tx[i] == CAN)
				cans++;
		assert(cans >= 5);                                 /* teardown emitted */
	}

	printf("test_ymodem: all assertions passed\n");
	return 0;
}
