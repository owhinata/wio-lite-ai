/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    frame_pipeline.c
 * @brief   Camera frame ring + sink dispatch -- freestanding core (svc/ layer,
 *          issues #47 design / #46 implementation).
 *
 * Pure bookkeeping over caller-provided SDRAM slots: it calls no HAL and no
 * ThreadX.  Mutual exclusion is the injected @ref frame_os; all waiting and
 * threading lives in the glue (port/camera).  The core is always entered from
 * thread context under that lock -- never from an ISR.  consume() is the only
 * callback invoked with the lock released, so a non-recursive injected mutex is
 * safe.  Host-unit-tested in shell/test/test_frame_pipeline.c with a no-op lock.
 *
 * Concurrency contract (see docs/{ja,en}/architecture/frame-pipeline.md):
 *   - publish() stamps gen + pre-pins one reference per delivering sink under
 *     the lock, then calls consume() outside it;
 *   - a sink balances each pre-pin / get() with exactly one put();
 *   - a LATEST sink coalesces to one pending frame (pinned); on completion put()
 *     transfers that pin to the next delivery;
 *   - acquire() returns a free, unpinned, non-latest slot (so a pull reader's
 *     latest frame is not recycled under it);
 *   - detach() unlinks and reports the sink's in-flight pin count; the caller
 *     drains the sink's thread before freeing it.
 */
#include "frame_pipeline.h"

#include <string.h>

enum { SLOT_FREE = 0, SLOT_FILLING = 1 };

/* ---- injected mutual exclusion ------------------------------------------- */

static void pl_lock(struct frame_pipeline *p)
{
	if (p->os && p->os->lock)
		p->os->lock(p->os->ctx);
}

static void pl_unlock(struct frame_pipeline *p)
{
	if (p->os && p->os->unlock)
		p->os->unlock(p->os->ctx);
}

static struct frame_slot *slot_of(const struct frame_desc *f)
{
	return (struct frame_slot *)f->_slot;
}

/* Drop one pin a sink holds on f's slot (caller holds the lock). */
static void unpin_locked(struct frame_sink *s, const struct frame_desc *f)
{
	struct frame_slot *slot = slot_of(f);

	if (slot->refcount > 0)
		slot->refcount--;
	if (s && s->_pins > 0)
		s->_pins--;
}

/* ---- producer side ------------------------------------------------------- */

int frame_pipeline_init(struct frame_pipeline *p, const struct frame_os *os,
                        void *slot_mem, uint32_t slot_size, uint32_t nslots)
{
	uint32_t i;

	if (!p || !slot_mem || slot_size == 0u || nslots == 0u ||
	    nslots > FRAME_PIPELINE_MAX_SLOTS)
		return -1;

	memset(p, 0, sizeof *p);
	p->os        = os;
	p->nslots    = nslots;
	p->slot_size = slot_size;
	p->latest    = -1;
	p->fmt       = FRAME_FMT_RGB565;

	for (i = 0; i < nslots; i++) {
		struct frame_slot *slot = &p->slots[i];

		slot->desc.data  = (uint8_t *)slot_mem + (size_t)i * slot_size;
		slot->desc._slot = slot;
		slot->state      = SLOT_FREE;
		slot->refcount   = 0;
	}
	return 0;
}

void frame_pipeline_set_format(struct frame_pipeline *p, enum frame_format fmt,
                               uint16_t w, uint16_t h)
{
	pl_lock(p);
	p->fmt    = fmt;
	p->width  = w;
	p->height = h;
	pl_unlock(p);
}

struct frame_desc *frame_pipeline_acquire(struct frame_pipeline *p)
{
	struct frame_desc *r = NULL;
	uint32_t i;

	pl_lock(p);
	for (i = 0; i < p->nslots; i++) {
		struct frame_slot *slot = &p->slots[i];

		if (slot->state == SLOT_FREE && slot->refcount == 0 &&
		    (int)i != p->latest) {
			slot->state = SLOT_FILLING;
			r = &slot->desc;
			p->stats.captured++;
			break;
		}
	}
	if (!r)
		p->stats.overruns++;
	pl_unlock(p);
	return r;
}

void frame_pipeline_publish(struct frame_pipeline *p, struct frame_desc *f,
                            uint32_t bytes, enum frame_format fmt,
                            uint16_t w, uint16_t h, uint16_t stride)
{
	struct frame_slot *slot = slot_of(f);
	struct frame_sink *deliver[FRAME_PIPELINE_MAX_SINKS];
	unsigned ndeliver = 0;
	struct frame_sink *s;
	unsigned i;

	pl_lock(p);
	p->gen++;
	f->gen    = p->gen;
	f->bytes  = bytes;
	f->format = (uint8_t)fmt;
	f->width  = w;
	f->height = h;
	f->stride = stride;
	slot->state = SLOT_FREE;          /* no longer filling; now published */
	p->latest   = (int)(slot - p->slots);
	p->stats.published = p->gen;

	for (s = p->sinks; s != NULL; s = s->_next) {
		if (s->_busy) {
			if (s->policy == FRAME_POLICY_DROP) {
				s->dropped++;
				continue;
			}
			/* FRAME_POLICY_LATEST: keep only the newest pending. */
			if (s->_pending) {
				unpin_locked(s, s->_pending);
				s->dropped++;
			}
			s->_pending = f;
			slot->refcount++;
			s->_pins++;
			continue;
		}
		s->_busy = 1;
		slot->refcount++;
		s->_pins++;
		if (ndeliver < FRAME_PIPELINE_MAX_SINKS)
			deliver[ndeliver++] = s;
	}
	pl_unlock(p);

	for (i = 0; i < ndeliver; i++) {
		int rc = deliver[i]->consume(deliver[i]->ctx, f);

		pl_lock(p);
		if (rc < 0)
			deliver[i]->errors++;
		deliver[i]->delivered++;
		pl_unlock(p);
	}
}

/* ---- sink registry ------------------------------------------------------- */

int frame_pipeline_attach(struct frame_pipeline *p, struct frame_sink *s)
{
	struct frame_sink *it;
	unsigned n = 0;

	if (!p || !s)
		return -1;

	pl_lock(p);
	for (it = p->sinks; it != NULL; it = it->_next)
		n++;
	pl_unlock(p);
	if (n >= FRAME_PIPELINE_MAX_SINKS)
		return -1;

	if (s->open) {
		int rc = s->open(s->ctx, p->fmt, p->width, p->height);

		if (rc < 0)
			return rc;
	}

	pl_lock(p);
	s->_busy    = 0;
	s->_pending = NULL;
	s->_pins    = 0;
	s->delivered = 0;
	s->dropped   = 0;
	s->errors    = 0;
	s->_next  = p->sinks;
	p->sinks  = s;
	pl_unlock(p);
	return 0;
}

int frame_pipeline_detach(struct frame_pipeline *p, struct frame_sink *s)
{
	struct frame_sink **pp;
	int inflight = 0;

	if (!p || !s)
		return 0;

	pl_lock(p);
	for (pp = &p->sinks; *pp != NULL; pp = &(*pp)->_next) {
		if (*pp == s) {
			*pp = s->_next;
			break;
		}
	}
	s->_next = NULL;
	/* A LATEST pending frame was never delivered to the sink, so the sink can
	   never put() it.  Drop it here (no new consume() after detach), leaving
	   only the pins the sink actually holds for the caller to drain. */
	if (s->_pending) {
		unpin_locked(s, s->_pending);
		s->_pending = NULL;
	}
	inflight = s->_pins;
	pl_unlock(p);

	if (s->close)
		s->close(s->ctx);
	return inflight;
}

/* ---- slot reference counting --------------------------------------------- */

void frame_pipeline_get(struct frame_pipeline *p, struct frame_sink *s,
                        const struct frame_desc *f)
{
	struct frame_slot *slot = slot_of(f);

	pl_lock(p);
	slot->refcount++;
	if (s)
		s->_pins++;
	pl_unlock(p);
}

void frame_pipeline_put(struct frame_pipeline *p, struct frame_sink *s,
                        const struct frame_desc *f)
{
	const struct frame_desc *next = NULL;

	pl_lock(p);
	unpin_locked(s, f);
	if (s) {
		s->_busy = 0;
		if (s->_pending) {
			/* Transfer the pending pin into the next delivery. */
			next = s->_pending;
			s->_pending = NULL;
			s->_busy = 1;
		}
	}
	pl_unlock(p);

	if (next) {
		int rc = s->consume(s->ctx, next);

		pl_lock(p);
		if (rc < 0)
			s->errors++;
		s->delivered++;
		pl_unlock(p);
	}
}

/* ---- pull access --------------------------------------------------------- */

int frame_pipeline_read_latest(struct frame_pipeline *p, uint32_t off,
                               void *dst, uint32_t len, uint32_t *gen)
{
	int rc = -1;

	if (!dst || len == 0u)
		return -1;

	pl_lock(p);
	if (p->latest >= 0) {
		struct frame_slot *slot = &p->slots[p->latest];

		if (off < slot->desc.bytes && len <= slot->desc.bytes - off) {
			memcpy(dst, (const uint8_t *)slot->desc.data + off, len);
			if (gen)
				*gen = slot->desc.gen;
			rc = 0;
		}
	}
	pl_unlock(p);
	return rc;
}

const struct frame_desc *frame_pipeline_pin_latest(struct frame_pipeline *p)
{
	const struct frame_desc *r = NULL;

	pl_lock(p);
	if (p->latest >= 0) {
		struct frame_slot *slot = &p->slots[p->latest];

		slot->refcount++;               /* keep it out of acquire() until put() */
		r = &slot->desc;
	}
	pl_unlock(p);
	return r;                           /* release with frame_pipeline_put(p, NULL, r) */
}

/* ---- statistics ---------------------------------------------------------- */

void frame_pipeline_stats(struct frame_pipeline *p, struct frame_stats *out)
{
	if (!out)
		return;
	pl_lock(p);
	*out = p->stats;
	pl_unlock(p);
}
