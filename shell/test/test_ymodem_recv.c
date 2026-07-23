/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 *
 * Host unit test for the clean-room YMODEM-CRC receiver (issue #19 M5,
 * svc/ymodem.c :: ymodem_recv).  Pure: no HAL/ThreadX/shell.
 *
 * Two harnesses, because they prove different things:
 *
 *   1. DUPLEX LOOPBACK (pthread).  ymodem_send() runs in one thread and
 *      ymodem_recv() in another, wired together by two blocking byte FIFOs.  A
 *      "capture the sender's TX, then replay it into the receiver" scheme could
 *      NOT do this: the sender only advances when it sees the receiver's 'C' and
 *      per-block ACKs, so a canned replay never exercises the real handshake.
 *      This works because the two directions keep SEPARATE static block buffers
 *      (see the banner in svc/ymodem.c) and share no other mutable state.
 *      Asserts byte-exact delivery and YM_OK (i.e. the batch actually closed).
 *
 *   2. SCRIPTED TRANSCRIPTS (single-threaded).  Error paths are driven from a
 *      canned byte stream so each failure is deterministic: CRC damage -> NAK ->
 *      recovery, a duplicate block, an out-of-order block, a sink that refuses
 *      the file, a batch that never closes, and short-block trimming.
 *      The script mock answers a timeout_ms == 0 poll with YM_IO_TIMEOUT, which
 *      is what the wire really looks like: after a bad block the sender is
 *      waiting for our NAK, so nothing is buffered for recv_drain() to eat.
 */
#include <assert.h>
#include <pthread.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

#include "ymodem.h"

/* protocol bytes (mirrored for the test's own frame building / assertions) */
#define SOH 0x01
#define STX 0x02
#define EOT 0x04
#define ACK 0x06
#define NAK 0x15
#define CAN 0x18
#define CRC_C 0x43
#define CPMEOF 0x1A

/* ======================================================================== *
 *  1. duplex loopback over two blocking FIFOs
 * ======================================================================== */

#define CHAN_CAP 65536u

struct chan {
	uint8_t         buf[CHAN_CAP];
	size_t          head, tail;     /* head = read pos, tail = write pos */
	pthread_mutex_t mu;
	pthread_cond_t  cv;
};

static void chan_init(struct chan *c)
{
	c->head = c->tail = 0;
	pthread_mutex_init(&c->mu, NULL);
	pthread_cond_init(&c->cv, NULL);
}

static size_t chan_len(const struct chan *c) { return c->tail - c->head; }

static int chan_getc(void *ctx, unsigned timeout_ms)
{
	struct chan *c = ctx;
	int          b = YM_IO_TIMEOUT;

	pthread_mutex_lock(&c->mu);
	if (chan_len(c) == 0 && timeout_ms > 0) {
		struct timeval  now;
		struct timespec ts;

		gettimeofday(&now, NULL);
		ts.tv_sec  = now.tv_sec + (time_t)(timeout_ms / 1000u);
		ts.tv_nsec = now.tv_usec * 1000L + (long)(timeout_ms % 1000u) * 1000000L;
		if (ts.tv_nsec >= 1000000000L) { ts.tv_sec++; ts.tv_nsec -= 1000000000L; }
		while (chan_len(c) == 0)
			if (pthread_cond_timedwait(&c->cv, &c->mu, &ts) != 0)
				break;
	}
	if (chan_len(c) > 0) {
		b = c->buf[c->head % CHAN_CAP];
		c->head++;
		pthread_cond_broadcast(&c->cv);
	}
	pthread_mutex_unlock(&c->mu);
	return b;
}

static int chan_put(void *ctx, const uint8_t *src, size_t len)
{
	struct chan *c = ctx;

	pthread_mutex_lock(&c->mu);
	for (size_t i = 0; i < len; i++) {
		while (chan_len(c) >= CHAN_CAP)
			pthread_cond_wait(&c->cv, &c->mu);   /* reader drains */
		c->buf[c->tail % CHAN_CAP] = src[i];
		c->tail++;
	}
	pthread_cond_broadcast(&c->cv);
	pthread_mutex_unlock(&c->mu);
	return 0;
}

/* ---- pattern source / capturing sink -------------------------------------- */
struct pat_src { uint32_t pos, size; };

static int pat_read(void *ctx, uint8_t *dst, uint32_t want, uint32_t *got)
{
	struct pat_src *s = ctx;
	uint32_t        n = s->size - s->pos;

	if (n > want)
		n = want;
	for (uint32_t i = 0; i < n; i++)
		dst[i] = (uint8_t)((s->pos + i) & 0xFFu);
	s->pos += n;
	*got = n;
	return 0;
}

struct cap_sink {
	uint8_t *buf;
	uint32_t cap, len;
	uint32_t declared;
	char     name[64];
	int      begins, rejects;
};

static int cap_begin(void *ctx, const char *name, uint32_t size)
{
	struct cap_sink *s = ctx;

	s->begins++;
	if (s->rejects)
		return -1;
	snprintf(s->name, sizeof s->name, "%s", name);
	s->declared = size;
	if (size > s->cap)
		return -1;
	s->len = 0;
	return 0;
}

static int cap_write(void *ctx, const uint8_t *data, uint32_t len)
{
	struct cap_sink *s = ctx;

	assert(s->len + len <= s->cap);      /* receiver overran the declared size */
	memcpy(s->buf + s->len, data, len);
	s->len += len;
	return 0;
}

/*
 * The ym_io vtable carries a single ctx for both directions, so each endpoint
 * gets a tiny adapter holding its own rx/tx channel pair.
 */
struct endpoint { struct chan *rx, *tx; };

static int ep_getc(void *ctx, unsigned timeout_ms)
{
	return chan_getc(((struct endpoint *)ctx)->rx, timeout_ms);
}

static int ep_put(void *ctx, const uint8_t *buf, size_t len)
{
	return chan_put(((struct endpoint *)ctx)->tx, buf, len);
}

struct send_arg {
	struct endpoint  ep;
	struct pat_src   src;
	const char      *name;
	enum ym_result   rc;
};

static void *run_sender(void *arg)
{
	struct send_arg *a = arg;
	struct ym_io     io = { &a->ep, ep_getc, ep_put };
	struct ym_source src = { &a->src, a->name, a->src.size, pat_read };

	a->rc = ymodem_send(&io, &src);
	return NULL;
}

/* One full send<->recv round trip of @size bytes; asserts byte-exact delivery. */
static void loopback(uint32_t size, const char *name)
{
	static struct chan a2b, b2a;    /* static: 64 KB each, off the thread stack */
	struct send_arg    sa;
	struct endpoint    rep;
	struct cap_sink    sink;
	struct ym_io       rio;
	struct ym_sink     vs = { &sink, cap_begin, cap_write };
	pthread_t          th;
	enum ym_result     rrc;

	chan_init(&a2b);
	chan_init(&b2a);

	memset(&sa, 0, sizeof sa);
	sa.ep.rx = &b2a; sa.ep.tx = &a2b;
	sa.src.pos = 0; sa.src.size = size;
	sa.name = name;

	memset(&sink, 0, sizeof sink);
	sink.cap = size ? size : 1u;
	sink.buf = malloc(sink.cap);
	assert(sink.buf != NULL);

	rep.rx = &a2b; rep.tx = &b2a;
	rio.ctx = &rep; rio.getc = ep_getc; rio.put = ep_put;

	assert(pthread_create(&th, NULL, run_sender, &sa) == 0);
	rrc = ymodem_recv(&rio, &vs);
	pthread_join(th, NULL);

	assert(sa.rc == YM_OK);
	assert(rrc == YM_OK);                          /* batch actually closed */
	assert(sink.begins == 1);
	assert(strcmp(sink.name, name) == 0);
	assert(sink.declared == size);
	assert(sink.len == size);
	for (uint32_t i = 0; i < size; i++)
		assert(sink.buf[i] == (uint8_t)(i & 0xFFu));
	free(sink.buf);
}

/* ======================================================================== *
 *  2. scripted transcripts
 * ======================================================================== */

struct script {
	uint8_t *in;          /* bytes the "sender" will produce, in order */
	size_t   inlen, inpos;
	uint8_t  out[8192];   /* bytes the receiver emitted */
	size_t   outlen;
};

static int script_getc(void *ctx, unsigned timeout_ms)
{
	struct script *s = ctx;

	/* A zero timeout is recv_drain() asking "is anything already buffered?".
	 * On the wire the answer is no -- the sender is waiting on our ACK/NAK. */
	if (timeout_ms == 0)
		return YM_IO_TIMEOUT;
	if (s->inpos >= s->inlen)
		return YM_IO_TIMEOUT;
	return s->in[s->inpos++];
}

static int script_put(void *ctx, const uint8_t *buf, size_t len)
{
	struct script *s = ctx;

	assert(s->outlen + len <= sizeof s->out);
	memcpy(s->out + s->outlen, buf, len);
	s->outlen += len;
	return 0;
}

/* Append one framed block to @p dst. */
static size_t put_block(uint8_t *dst, size_t at, uint8_t kind, uint8_t seq,
                        const uint8_t *data, uint16_t dl, int corrupt_crc)
{
	uint16_t crc;

	dst[at++] = kind;
	dst[at++] = seq;
	dst[at++] = (uint8_t)~seq;
	memcpy(dst + at, data, dl);
	crc = ymodem_crc16(dst + at, dl);
	if (corrupt_crc)
		crc ^= 0xFFFFu;
	at += dl;
	dst[at++] = (uint8_t)(crc >> 8);
	dst[at++] = (uint8_t)(crc & 0xFFu);
	return at;
}

/* Build a 128-byte block-0 payload: "name\0size\0", zero padded. */
static void build_b0(uint8_t *p, const char *name, const char *size)
{
	size_t i = 0;

	memset(p, 0, 128);
	while (*name)
		p[i++] = (uint8_t)*name++;
	i++;
	while (*size)
		p[i++] = (uint8_t)*size++;
}

static int count_byte(const struct script *s, uint8_t b)
{
	int n = 0;

	for (size_t i = 0; i < s->outlen; i++)
		if (s->out[i] == b)
			n++;
	return n;
}

static enum ym_result run_script(struct script *s, struct cap_sink *sink)
{
	struct ym_io   io = { s, script_getc, script_put };
	struct ym_sink vs = { sink, cap_begin, cap_write };

	return ymodem_recv(&io, &vs);
}

int main(void)
{
	/* ---- 1. duplex loopback ---- */
	loopback(1, "one.bin");
	loopback(100, "frame.raw");
	loopback(128, "b128.bin");
	loopback(1024, "b1024.bin");
	loopback(1025, "b1025.bin");
	loopback(260u * 1024u, "wrap.bin");     /* crosses the seq 255 -> 0 wrap */
	loopback(0xDC000u, "stock.bin");        /* the #19 M5 stock-image size */
	printf("  duplex loopback: ok\n");

	static uint8_t in[16384];
	static uint8_t data[1024];
	uint8_t        b0[128], nullb0[128];
	struct script  s;
	struct cap_sink sink;
	static uint8_t sinkbuf[4096];

	for (int i = 0; i < 1024; i++)
		data[i] = (uint8_t)(i & 0xFF);
	memset(nullb0, 0, sizeof nullb0);

#define SCRIPT_BEGIN()  do {                                    \
		memset(&s, 0, sizeof s);                        \
		s.in = in; s.inlen = 0;                         \
		memset(&sink, 0, sizeof sink);                  \
		sink.buf = sinkbuf; sink.cap = sizeof sinkbuf;  \
	} while (0)

	/* ---- L. CRC damage on block 1 -> NAK -> the resend completes ---- */
	{
		size_t n = 0;
		SCRIPT_BEGIN();
		build_b0(b0, "a.bin", "1024");
		n = put_block(in, n, SOH, 0, b0, 128, 0);
		n = put_block(in, n, STX, 1, data, 1024, 1);   /* bad CRC */
		n = put_block(in, n, STX, 1, data, 1024, 0);   /* the resend */
		in[n++] = EOT;
		n = put_block(in, n, SOH, 0, nullb0, 128, 0);
		s.inlen = n;

		assert(run_script(&s, &sink) == YM_OK);
		assert(sink.len == 1024);
		assert(memcmp(sink.buf, data, 1024) == 0);     /* exactly once */
		assert(count_byte(&s, NAK) == 1);              /* the bad block was NAKed */
	}

	/* ---- M. duplicate block (our ACK was lost) -> re-ACKed and dropped ---- */
	{
		size_t n = 0;
		SCRIPT_BEGIN();
		build_b0(b0, "a.bin", "2048");
		n = put_block(in, n, SOH, 0, b0, 128, 0);
		n = put_block(in, n, STX, 1, data, 1024, 0);
		n = put_block(in, n, STX, 1, data, 1024, 0);   /* duplicate seq 1 */
		n = put_block(in, n, STX, 2, data, 1024, 0);
		in[n++] = EOT;
		n = put_block(in, n, SOH, 0, nullb0, 128, 0);
		s.inlen = n;

		assert(run_script(&s, &sink) == YM_OK);
		assert(sink.len == 2048);                      /* not 3072 */
		assert(count_byte(&s, NAK) == 0);
	}

	/* ---- N. out-of-order block -> YM_ERR_PROTO + CAN teardown ---- */
	{
		size_t n = 0;
		SCRIPT_BEGIN();
		build_b0(b0, "a.bin", "4096");
		n = put_block(in, n, SOH, 0, b0, 128, 0);
		n = put_block(in, n, STX, 1, data, 1024, 0);
		n = put_block(in, n, STX, 7, data, 1024, 0);   /* jumped */
		s.inlen = n;

		assert(run_script(&s, &sink) == YM_ERR_PROTO);
		assert(count_byte(&s, CAN) >= 5);
	}

	/* ---- O. sink refuses the file -> YM_ERR_SINK, no data delivered ---- */
	{
		size_t n = 0;
		SCRIPT_BEGIN();
		sink.rejects = 1;
		build_b0(b0, "a.bin", "1024");
		n = put_block(in, n, SOH, 0, b0, 128, 0);
		n = put_block(in, n, STX, 1, data, 1024, 0);
		s.inlen = n;

		assert(run_script(&s, &sink) == YM_ERR_SINK);
		assert(sink.begins == 1);
		assert(sink.len == 0);
		assert(count_byte(&s, CAN) >= 5);
	}

	/* ---- P. batch never closes (stream ends after EOT) -> TIMEOUT, not OK ---- */
	{
		size_t n = 0;
		SCRIPT_BEGIN();
		build_b0(b0, "a.bin", "1024");
		n = put_block(in, n, SOH, 0, b0, 128, 0);
		n = put_block(in, n, STX, 1, data, 1024, 0);
		in[n++] = EOT;
		s.inlen = n;                                   /* no closing block 0 */

		assert(run_script(&s, &sink) == YM_ERR_TIMEOUT);
		assert(sink.len == 1024);                      /* data arrived ... */
		assert(count_byte(&s, CAN) >= 5);              /* ... but it is NOT success */
	}

	/* ---- Q. declared size trims the padded short block ---- */
	{
		size_t  n = 0;
		uint8_t shortblk[128];
		SCRIPT_BEGIN();
		memcpy(shortblk, data, 100);
		memset(shortblk + 100, CPMEOF, 28);
		build_b0(b0, "a.bin", "100");
		n = put_block(in, n, SOH, 0, b0, 128, 0);
		n = put_block(in, n, SOH, 1, shortblk, 128, 0);
		in[n++] = EOT;
		n = put_block(in, n, SOH, 0, nullb0, 128, 0);
		s.inlen = n;

		assert(run_script(&s, &sink) == YM_OK);
		assert(sink.declared == 100);
		assert(sink.len == 100);                       /* padding trimmed */
		assert(memcmp(sink.buf, data, 100) == 0);
	}

	/* ---- R. an immediate null block 0 = the sender offered no file ---- */
	{
		size_t n = 0;
		SCRIPT_BEGIN();
		n = put_block(in, n, SOH, 0, nullb0, 128, 0);
		s.inlen = n;

		assert(run_script(&s, &sink) == YM_ERR_CANCEL);
		assert(sink.begins == 0);
	}

	/* ---- T. EOT before the declared size is delivered -> NOT success ----
	 * A truncated file must not be able to reach the closing block and be reported
	 * as OK; the caller would otherwise stage a short image and program it. */
	{
		size_t n = 0;
		SCRIPT_BEGIN();
		build_b0(b0, "a.bin", "2048");           /* promises 2048 ... */
		n = put_block(in, n, SOH, 0, b0, 128, 0);
		n = put_block(in, n, STX, 1, data, 1024, 0);   /* ... delivers 1024 */
		in[n++] = EOT;
		n = put_block(in, n, SOH, 0, nullb0, 128, 0);  /* and closes the batch */
		s.inlen = n;

		assert(run_script(&s, &sink) == YM_ERR_PROTO);
		assert(count_byte(&s, CAN) >= 5);
	}

	/* ---- U. malformed block 0 -> rejected before any data is accepted ---- */
	{
		size_t  n = 0;
		uint8_t bad[128];

		/* U1: a decimal size that overflows 32 bits (would wrap to a small value
		 * and slip past the sink's capacity check). */
		SCRIPT_BEGIN();
		build_b0(bad, "a.bin", "99999999999");
		n = put_block(in, n, SOH, 0, bad, 128, 0);
		s.inlen = n;
		assert(run_script(&s, &sink) == YM_ERR_PROTO);
		assert(sink.begins == 0);

		/* U2: an unterminated name field (no NUL anywhere in the block). */
		SCRIPT_BEGIN();
		memset(bad, 'x', sizeof bad);
		n = 0;
		n = put_block(in, n, SOH, 0, bad, 128, 0);
		s.inlen = n;
		assert(run_script(&s, &sink) == YM_ERR_PROTO);
		assert(sink.begins == 0);
	}

	/* ---- S. a silent sender -> TIMEOUT after the handshake budget ---- */
	{
		SCRIPT_BEGIN();
		s.inlen = 0;
		assert(run_script(&s, &sink) == YM_ERR_TIMEOUT);
		assert(count_byte(&s, CRC_C) > 1);             /* it kept poking with 'C' */
	}

	printf("test_ymodem_recv: all assertions passed\n");
	return 0;
}
