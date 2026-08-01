/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 *
 * Host unit test for the camera frame pipeline core (svc/frame_pipeline.c,
 * issue #47 design / #46).  Pure: no HAL/ThreadX/shell -- a no-op frame_os and
 * mock sinks exercise the ring/refcount/policy logic on the host.  Asserts:
 *   A. init + acquire returns distinct free slots; acquire excludes the latest
 *      published slot (so a pull reader is not recycled under).
 *   B. publish stamps a monotonic generation.
 *   C. DROP policy: a frame published while the sink is busy is dropped.
 *   D. LATEST policy: coalesces to the newest pending; on completion put()
 *      delivers it; a superseded pending is dropped.
 *   E. get()/put() refcount; acquire() skips a pinned (held) slot.
 *   F. detach() reports in-flight pins and stops further consume().
 *   G. read_latest() returns the latest frame's bytes + generation (tear via
 *      gen across calls).
 *   H. a counting (auto-put) sink keeps an N=4 ring cycling like the producer.
 */
#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "frame.h"
#include "frame_pipeline.h"

/* ---- no-op injected lock (single-threaded host) -------------------------- */
static int lock_depth;
static void host_lock(void *ctx)   { (void)ctx; lock_depth++; }
static void host_unlock(void *ctx) { (void)ctx; lock_depth--; }
static const struct frame_os HOST_OS = { NULL, host_lock, host_unlock };

/* ---- mock sink ----------------------------------------------------------- */
struct mock {
	struct frame_sink        sink;
	struct frame_pipeline   *p;
	int                      auto_put;  /* 1 = put inside consume (counting)   */
	int                      open_rc;   /* open() return                       */
	int                      consume_calls;
	int                      open_calls;
	const struct frame_desc *last;      /* last consumed frame (for manual put) */
};

static int mock_open(void *ctx, enum frame_format fmt, uint16_t w, uint16_t h)
{
	struct mock *m = ctx;
	(void)fmt; (void)w; (void)h;
	m->open_calls++;
	return m->open_rc;
}

static int mock_consume(void *ctx, const struct frame_desc *f)
{
	struct mock *m = ctx;
	m->consume_calls++;
	m->last = f;
	if (m->auto_put)
		frame_pipeline_put(m->p, &m->sink, f);
	return 0;
}

static void mock_init(struct mock *m, struct frame_pipeline *p,
                      uint8_t policy, int auto_put)
{
	memset(m, 0, sizeof *m);
	m->p = p;
	m->auto_put = auto_put;
	m->sink.name    = "mock";
	m->sink.ctx     = m;
	m->sink.policy  = policy;
	m->sink.open    = mock_open;
	m->sink.consume = mock_consume;
	m->sink.close   = NULL;
}

/* ---- shared backing store ------------------------------------------------ */
#define SLOT_SZ 16u
#define NSLOTS  5u
static uint8_t mem[NSLOTS * SLOT_SZ];

static void fresh(struct frame_pipeline *p, uint32_t n)
{
	assert(frame_pipeline_init(p, &HOST_OS, mem, SLOT_SZ, n) == 0);
}

static struct frame_desc *pub(struct frame_pipeline *p)
{
	struct frame_desc *d = frame_pipeline_acquire(p);
	assert(d != NULL);
	frame_pipeline_publish(p, d, SLOT_SZ, FRAME_FMT_RGB565, 4, 2, 8);
	return d;
}

/* A. distinct slots + acquire excludes latest -------------------------------*/
static void test_acquire_latest(void)
{
	struct frame_pipeline p;
	fresh(&p, 4);

	struct frame_desc *a = frame_pipeline_acquire(&p);
	struct frame_desc *b = frame_pipeline_acquire(&p);
	assert(a && b && a != b);             /* distinct free slots */

	/* publish a -> it becomes latest; a later acquire must avoid it */
	frame_pipeline_publish(&p, a, SLOT_SZ, FRAME_FMT_RGB565, 4, 2, 8);
	struct frame_desc *c = frame_pipeline_acquire(&p);
	assert(c && c != a);                  /* not the latest */
}

/* B. monotonic generation ---------------------------------------------------*/
static void test_generation(void)
{
	struct frame_pipeline p;
	fresh(&p, 4);
	struct frame_desc *a = pub(&p);
	struct frame_desc *b = pub(&p);
	assert(a->gen == 1 && b->gen == 2);
	struct frame_stats st;
	frame_pipeline_stats(&p, &st);
	assert(st.published == 2);
}

/* C. DROP policy drops while busy ------------------------------------------ */
static void test_drop(void)
{
	struct frame_pipeline p;
	struct mock m;
	fresh(&p, 4);
	mock_init(&m, &p, FRAME_POLICY_DROP, 0 /* hold */);
	assert(frame_pipeline_attach(&p, &m.sink) == 0);
	assert(m.open_calls == 1);

	struct frame_desc *d1 = frame_pipeline_acquire(&p);
	frame_pipeline_publish(&p, d1, SLOT_SZ, FRAME_FMT_RGB565, 4, 2, 8);
	assert(m.consume_calls == 1);         /* delivered, now busy (held) */

	struct frame_desc *d2 = frame_pipeline_acquire(&p);
	frame_pipeline_publish(&p, d2, SLOT_SZ, FRAME_FMT_RGB565, 4, 2, 8);
	assert(m.consume_calls == 1);         /* dropped while busy */
	assert(m.sink.dropped == 1);
	assert(m.sink.delivered == 1);

	frame_pipeline_put(&p, &m.sink, d1);  /* release -> no pending */
	assert(m.sink._busy == 0);
}

/* D. LATEST coalesce + pending transfer ------------------------------------ */
static void test_latest(void)
{
	struct frame_pipeline p;
	struct mock m;
	fresh(&p, 5);
	mock_init(&m, &p, FRAME_POLICY_LATEST, 0 /* hold */);
	assert(frame_pipeline_attach(&p, &m.sink) == 0);

	struct frame_desc *d1 = frame_pipeline_acquire(&p);
	frame_pipeline_publish(&p, d1, SLOT_SZ, FRAME_FMT_RGB565, 4, 2, 8);
	assert(m.consume_calls == 1);         /* d1 delivered, busy */

	struct frame_desc *d2 = frame_pipeline_acquire(&p);
	frame_pipeline_publish(&p, d2, SLOT_SZ, FRAME_FMT_RGB565, 4, 2, 8);
	assert(m.consume_calls == 1);         /* d2 -> pending (not delivered) */

	struct frame_desc *d3 = frame_pipeline_acquire(&p);
	frame_pipeline_publish(&p, d3, SLOT_SZ, FRAME_FMT_RGB565, 4, 2, 8);
	assert(m.consume_calls == 1);         /* d3 supersedes pending d2 */
	assert(m.sink.dropped == 1);          /* d2 dropped */

	/* completing d1 transfers the pending (d3) into the next delivery */
	frame_pipeline_put(&p, &m.sink, d1);
	assert(m.consume_calls == 2);         /* d3 now delivered */
	assert(m.last == d3);
	frame_pipeline_put(&p, &m.sink, d3);
	assert(m.sink._busy == 0 && m.sink._pins == 0);
}

/* E. get/put refcount + acquire skips a held slot -------------------------- */
static void test_refcount(void)
{
	struct frame_pipeline p;
	struct mock m;
	int i, n = 0;
	fresh(&p, 4);
	mock_init(&m, &p, FRAME_POLICY_DROP, 0 /* hold */);
	assert(frame_pipeline_attach(&p, &m.sink) == 0);

	struct frame_desc *d1 = frame_pipeline_acquire(&p);
	frame_pipeline_publish(&p, d1, SLOT_SZ, FRAME_FMT_RGB565, 4, 2, 8);
	assert(m.sink._pins == 1);            /* d1 held by the sink */

	/* d1 is latest + pinned; every other acquire must avoid it (the 3 other
	 * slots become FILLING, then acquire returns NULL -> overrun). */
	for (i = 0; i < 5; i++) {
		struct frame_desc *d = frame_pipeline_acquire(&p);
		if (d) { assert(d != d1); n++; }
	}
	assert(n == 3);

	frame_pipeline_put(&p, &m.sink, d1);  /* release the pin */
	assert(m.sink._pins == 0);
}

/* F. detach reports in-flight pins, stops consume -------------------------- */
static void test_detach(void)
{
	struct frame_pipeline p;
	struct mock m;
	fresh(&p, 4);
	mock_init(&m, &p, FRAME_POLICY_DROP, 0 /* hold */);
	assert(frame_pipeline_attach(&p, &m.sink) == 0);

	struct frame_desc *d1 = frame_pipeline_acquire(&p);
	frame_pipeline_publish(&p, d1, SLOT_SZ, FRAME_FMT_RGB565, 4, 2, 8);
	assert(m.sink._pins == 1);

	int inflight = frame_pipeline_detach(&p, &m.sink);
	assert(inflight == 1);                /* one pin still held */

	/* after detach, a publish reaches no sink */
	int before = m.consume_calls;
	struct frame_desc *d2 = frame_pipeline_acquire(&p);
	frame_pipeline_publish(&p, d2, SLOT_SZ, FRAME_FMT_RGB565, 4, 2, 8);
	assert(m.consume_calls == before);
}

/* F2. detach drops an undelivered LATEST pending pin ------------------------*/
static void test_detach_pending(void)
{
	struct frame_pipeline p;
	struct mock m;
	int inflight, before;
	fresh(&p, 5);
	mock_init(&m, &p, FRAME_POLICY_LATEST, 0 /* hold */);
	assert(frame_pipeline_attach(&p, &m.sink) == 0);

	struct frame_desc *d1 = frame_pipeline_acquire(&p);
	frame_pipeline_publish(&p, d1, SLOT_SZ, FRAME_FMT_RGB565, 4, 2, 8);
	struct frame_desc *d2 = frame_pipeline_acquire(&p);
	frame_pipeline_publish(&p, d2, SLOT_SZ, FRAME_FMT_RGB565, 4, 2, 8);
	assert(m.sink._pins == 2);          /* active d1 + pending d2 */

	inflight = frame_pipeline_detach(&p, &m.sink);
	assert(inflight == 1);              /* pending dropped; only the active pin */
	assert(m.sink._pending == NULL);
	assert(m.sink._pins == 1);

	/* the sink completes its active delivery; no pending re-consume happens */
	before = m.consume_calls;
	frame_pipeline_put(&p, &m.sink, d1);
	assert(m.consume_calls == before);
	assert(m.sink._pins == 0);
}

/* G. read_latest bytes + generation ---------------------------------------- */
static void test_read_latest(void)
{
	struct frame_pipeline p;
	uint8_t buf[SLOT_SZ];
	uint32_t gen = 0;
	fresh(&p, 4);

	assert(frame_pipeline_read_latest(&p, 0, buf, 4, &gen) < 0); /* no frame yet */

	struct frame_desc *a = frame_pipeline_acquire(&p);
	memset(a->data, 0xAB, SLOT_SZ);
	frame_pipeline_publish(&p, a, SLOT_SZ, FRAME_FMT_RGB565, 4, 2, 8);
	assert(frame_pipeline_read_latest(&p, 0, buf, 4, &gen) == 0);
	assert(gen == 1 && buf[0] == 0xAB && buf[3] == 0xAB);

	struct frame_desc *b = frame_pipeline_acquire(&p);
	memset(b->data, 0xCD, SLOT_SZ);
	frame_pipeline_publish(&p, b, SLOT_SZ, FRAME_FMT_RGB565, 4, 2, 8);
	assert(frame_pipeline_read_latest(&p, 0, buf, 4, &gen) == 0);
	assert(gen == 2 && buf[0] == 0xCD);   /* gen change = frame replaced */

	assert(frame_pipeline_read_latest(&p, SLOT_SZ, buf, 1, &gen) < 0); /* OOB */
}

/* H. counting sink keeps an N=4 ring cycling (producer-like) ---------------- */
static void test_ring_cycle(void)
{
	struct frame_pipeline p;
	struct mock m;
	int i;
	fresh(&p, 4);
	mock_init(&m, &p, FRAME_POLICY_DROP, 1 /* auto-put (counting) */);
	assert(frame_pipeline_attach(&p, &m.sink) == 0);

	/* Mimic the producer: each frame acquired, filled, published; the counting
	 * sink consumes+puts immediately, so the ring never stalls. */
	for (i = 0; i < 100; i++) {
		struct frame_desc *d = frame_pipeline_acquire(&p);
		assert(d != NULL);                /* never NULL with N=4 + auto-put */
		frame_pipeline_publish(&p, d, SLOT_SZ, FRAME_FMT_RGB565, 4, 2, 8);
	}
	assert(m.consume_calls == 100 && m.sink.delivered == 100);
	assert(m.sink.dropped == 0 && m.sink.errors == 0);

	struct frame_stats st;
	frame_pipeline_stats(&p, &st);
	assert(st.published == 100 && st.overruns == 0);
}

int main(void)
{
	test_acquire_latest();
	test_generation();
	test_drop();
	test_latest();
	test_refcount();
	test_detach();
	test_detach_pending();
	test_read_latest();
	test_ring_cycle();
	assert(lock_depth == 0);              /* every lock balanced by unlock */
	printf("test_frame_pipeline: all assertions passed\n");
	return 0;
}
