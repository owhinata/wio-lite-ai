/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    frame_pipeline.h
 * @brief   Camera frame ring + sink dispatch (svc/ layer, #47 design / #46
 *          implementation; multi-sink cascade live since #100/#101).
 *
 * One producer (DCMI capture) publishes frames into an N-slot SDRAM ring; many
 * sinks consume them.  The base capture attaches an internal stats sink plus up
 * to three external subscribers (GUIX preview / nncam / MJPEG) that attach and
 * detach at runtime -- the "subscriber cascade" (Epic #99).  This is the @ref frame_desc data contract (frame.h) plus
 * the ownership/back-pressure mechanics, applied to frame distribution the same
 * way @ref fs_device (#34) abstracts media and @ref ym_source (#50) abstracts a
 * byte source.
 *
 * Layering (issue #43): this core is **freestanding** -- it depends only on
 * <stdint.h>/<stddef.h> and an injected mutual-exclusion vtable (@ref
 * frame_os).  It calls NO HAL and NO ThreadX, so it sits in svc/ next to
 * ymodem.c/fmt.c and is host-unit-testable (ring/refcount/policy logic with a
 * no-op lock and mock sinks).  All ThreadX -- the producer/sink threads, the
 * ISR -> thread notification, the TX_MUTEX behind @ref frame_os -- lives in the
 * glue (port/camera and each sink); the core is always entered from thread
 * context under the injected lock (the DCMI ISR only posts cam_done, never
 * touches the ring -- the existing discipline).  Unlike timebase.c (which
 * includes HAL), this header includes none.
 *
 * Two access styles coexist:
 *   - push sinks (streaming: LTDC / Ethernet / VCP preview) register via
 *     frame_pipeline_attach() and get consume() called on each publish;
 *   - pull access (snapshot: save / send / stats) reads the latest published
 *     slot via frame_pipeline_read_latest() (the generalised camera_frame_read),
 *     or pins it with frame_pipeline_pin_latest() to copy a whole frame out of the
 *     lock (camera save/send, #102).
 * A single on-demand `camera capture` is just the degenerate case: an N=1 ring,
 * no push sinks, one publish, pulled by read_latest -- so the existing
 * camera capture/frame_read/save/send keep their semantics unchanged.
 *
 * See docs/{ja,en}/architecture/frame-pipeline.md for the architecture, the
 * concurrency/lifetime contract (publish lock discipline, refcount pin/put,
 * LATEST pending transfer, detach quiesce) and the HW rationale.
 */
#ifndef FRAME_PIPELINE_H
#define FRAME_PIPELINE_H

#include <stdint.h>

#include "frame.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Injected mutual exclusion.  The glue wires this to a ThreadX TX_MUTEX
 * (TX_INHERIT); a host unit test wires it to a no-op.  The core never blocks on
 * anything else: there is no wait primitive in svc/, so all waiting/notification
 * (producer DMA completion, sink wakeups, detach drain) is owned by the glue.
 * Held only for short bookkeeping critical sections -- never across consume().
 */
struct frame_os {
	void *ctx;
	void (*lock)(void *ctx);
	void (*unlock)(void *ctx);
};

/**
 * Per-sink back-pressure policy.  Push sinks are non-blocking by construction --
 * the DCMI cannot be stalled mid-frame, so a sink that falls behind drops
 * frames.  A must-complete consumer (a save that must not lose data) is NOT a
 * push policy: it is expressed as pull access in snapshot mode (read_latest),
 * where the producer naturally blocks on a single frame.  Hence there is no
 * BLOCK policy.
 */
enum frame_policy {
	FRAME_POLICY_DROP = 0, /**< if the sink is busy, drop this frame (live default) */
	FRAME_POLICY_LATEST,   /**< coalesce: while busy keep only the newest as pending */
};

/**
 * A push sink: a thin vtable in the @ref fs_device / @ref ym_source idiom.  The
 * producer never knows the concrete sink.  Caller-allocated and caller-owned --
 * see the detach contract below before freeing one.
 *
 * consume() is called by frame_pipeline_publish() **outside** the pipeline lock,
 * with the slot already pre-pinned once on this sink's behalf.  The sink must
 * release that pin with exactly one frame_pipeline_put() -- a synchronous sink
 * does its work and puts before returning; an asynchronous sink hands the
 * descriptor to its own thread/queue (the pre-pin keeps the slot alive) and puts
 * from that thread when done.  This holds even on an error return.  Because
 * put() is called with the pipeline lock released, a non-recursive mutex never
 * self-deadlocks.
 */
struct frame_sink {
	const char *name;
	void       *ctx;
	uint8_t     policy;  /**< enum frame_policy */
	/** Negotiate format/geometry at attach; return <0 to reject (unsupported). */
	int  (*open)(void *ctx, enum frame_format fmt, uint16_t w, uint16_t h);
	/** Consume one frame (read-only).  <0 = sink error (counted; producer keeps
	 *  running).  Must arrange exactly one frame_pipeline_put(p, s, f) of the
	 *  pre-pinned slot (synchronously, or from the sink's own thread). */
	int  (*consume)(void *ctx, const struct frame_desc *f);
	void (*close)(void *ctx);

	/* Statistics -- updated by the core under the lock; read-only to the owner. */
	uint32_t delivered;
	uint32_t dropped;
	uint32_t errors;

	/* Core-internal; the owner must not touch these. */
	struct frame_sink       *_next;
	const struct frame_desc *_pending; /**< LATEST coalesce slot (pinned)         */
	int                      _busy;     /**< a consume() for this sink is in flight */
	int                      _pins;     /**< slots this sink currently holds (detach) */
};

/** Compile-time caps for the inline bookkeeping (issue #46 implementation).
 *  A producer statically allocates one struct frame_pipeline; nslots <= MAX. */
#ifndef FRAME_PIPELINE_MAX_SLOTS
#define FRAME_PIPELINE_MAX_SLOTS 8u
#endif
#ifndef FRAME_PIPELINE_MAX_SINKS
#define FRAME_PIPELINE_MAX_SINKS 4u
#endif

/** One ring slot: a descriptor over caller SDRAM plus its bookkeeping. */
struct frame_slot {
	struct frame_desc desc;     /**< .data fixed at init; geometry/gen at publish */
	int               state;    /**< 0 = free, 1 = filling (acquired, pre-publish) */
	int               refcount; /**< sink pins (0 = reusable when free)            */
};

/** Producer/pipeline counters (read via frame_pipeline_stats). */
struct frame_stats {
	uint32_t captured;  /**< frames the producer acquired/filled  */
	uint32_t published; /**< frames published (== last gen)        */
	uint32_t overruns;  /**< acquire() calls that found no free slot */
};

/**
 * Ring/dispatch engine.  Statically allocated by the producer (port/camera owns
 * one instance, the way it owns cam_frame[] today); pass its address to every
 * call.  Fields are implementation-internal -- callers treat it as opaque and
 * only keep the pointer.
 */
struct frame_pipeline {
	const struct frame_os *os;                          /* injected mutex          */
	struct frame_sink     *sinks;                       /* attached-sink list head */
	struct frame_slot      slots[FRAME_PIPELINE_MAX_SLOTS];
	uint32_t               nslots;
	uint32_t               slot_size;                   /* bytes per slot          */
	int                    latest;                      /* last published idx, -1  */
	uint32_t               gen;                          /* publish counter         */
	enum frame_format      fmt;                          /* current format (open)   */
	uint16_t               width, height;               /* current geometry (open) */
	struct frame_stats     stats;
};

/* ---- producer side ------------------------------------------------------- */

/**
 * Bind the engine to @p nslots ring slots carved from caller-owned SDRAM
 * (slot_mem is nslots * slot_size bytes, .sdram, non-cacheable).  The producer
 * owns the slot memory; the core owns only the bookkeeping (gen / refcount /
 * registry).  @p os supplies mutual exclusion.  Returns 0 or <0.
 */
int frame_pipeline_init(struct frame_pipeline *p, const struct frame_os *os,
                        void *slot_mem, uint32_t slot_size, uint32_t nslots);

/**
 * Claim a free slot to fill (refcount==0 and not the latest published one, so a
 * pull reader's current frame is never recycled under it).  Returns a writable
 * descriptor whose @ref frame_desc.data points into the slot, or NULL when every
 * slot is pinned (the producer then counts an overrun and drops the frame).
 */
struct frame_desc *frame_pipeline_acquire(struct frame_pipeline *p);

/**
 * Publish a filled slot: stamp the generation and the geometry, then fan out to
 * the attached sinks.  Lock discipline (the core of the design): the lock is
 * held only to stamp gen, snapshot the delivery set and pre-pin one reference
 * per delivering sink; consume() is then called with the lock released.  A busy
 * DROP sink is skipped (dropped++); a busy LATEST sink keeps @p f as its pending
 * (old pending, if any, is put first; the new one is pre-pinned).
 */
void frame_pipeline_publish(struct frame_pipeline *p, struct frame_desc *f,
                            uint32_t bytes, enum frame_format fmt,
                            uint16_t w, uint16_t h, uint16_t stride);

/* ---- sink registry ------------------------------------------------------- */

/**
 * Set the current frame format/geometry the producer will publish.  attach()
 * passes these to a sink's open(); a producer calls this before attaching sinks
 * (and, for #45, on a format change -- re-opening sinks is the caller's job).
 */
void frame_pipeline_set_format(struct frame_pipeline *p, enum frame_format fmt,
                               uint16_t w, uint16_t h);

/**
 * Register a push sink.  attach() calls s->open() with the pipeline's current
 * format/geometry to negotiate acceptance (<0 from open rejects the attach).
 * The current format is owned by the producer: fixed QVGA RGB565 today (set from
 * its init config); #45 makes it variable, re-opening sinks (close()+open()) on a
 * format change.  Returns 0 or <0.
 */
int frame_pipeline_attach(struct frame_pipeline *p, struct frame_sink *s);

/**
 * Unlink a sink so no further consume() is issued to it, and report how many
 * pins it still holds in flight (including a LATEST pending pin).  The core does
 * NOT wait: before freeing @p s / its ctx, the caller (which owns the sink's
 * thread and queue) must drain that thread -- let the running consume() finish
 * and every pin be put() -- until the in-flight count reaches zero.  Returns the
 * in-flight pin count at detach time.
 */
int frame_pipeline_detach(struct frame_pipeline *p, struct frame_sink *s);

/* ---- slot reference counting (async sinks) ------------------------------- */

/**
 * Pin / release a slot on behalf of sink @p s.  publish() pre-pins once per
 * delivering sink; the sink balances that with exactly one put() of the same
 * (s, f).  put() also clears that sink's busy state and, for a LATEST sink with a
 * pending frame, transfers the pending pin into the next delivery (see the
 * design doc).  get() takes an ADDITIONAL pin when sink @p s re-queues a frame;
 * such pins also count toward the sink's in-flight total at detach.  The sink
 * argument is required because several sinks share one descriptor, so the slot
 * alone cannot identify whose pin/busy/pending this is.  Both take the lock
 * briefly internally, so they must be called with no pipeline lock held (from
 * consume()/sink-thread context, never nested) -- this keeps a non-recursive
 * mutex deadlock-free.  A slot becomes reusable by the producer when its refcount
 * returns to 0.
 */
void frame_pipeline_get(struct frame_pipeline *p, struct frame_sink *s,
                        const struct frame_desc *f);
void frame_pipeline_put(struct frame_pipeline *p, struct frame_sink *s,
                        const struct frame_desc *f);

/* ---- pull access (snapshot) ---------------------------------------------- */

/**
 * Copy @p len bytes at byte offset @p off out of the latest published frame into
 * @p dst (any alignment), under the lock, and return its generation in @p gen
 * (may be NULL).  The generalised camera_frame_read: a single call cannot tear
 * (the slot is not recycled mid-copy); a multi-call reader (row-by-row save)
 * compares @p gen across calls to detect a frame replaced between reads.  Fails
 * <0 when no frame has been published yet.
 */
int frame_pipeline_read_latest(struct frame_pipeline *p, uint32_t off,
                               void *dst, uint32_t len, uint32_t *gen);

/**
 * Pin the latest published slot (refcount++) and return its descriptor, or NULL
 * when no frame has been published yet.  Unlike frame_pipeline_read_latest(), the
 * whole-frame copy happens OUTSIDE the lock: the returned @ref frame_desc (its
 * data/bytes/gen) stays valid until the caller releases the pin with exactly one
 * frame_pipeline_put(p, NULL, desc) -- while pinned the slot is never re-acquired
 * (refcount != 0) even after a newer publish moves `latest` off it, so the copy is
 * tear-free without holding the pipeline lock across it.  Use for a one-shot
 * snapshot of a live streamed frame (camera save/send) without stalling the
 * producer's publish/DMA-repoint (#102).
 */
const struct frame_desc *frame_pipeline_pin_latest(struct frame_pipeline *p);

/* ---- statistics (struct frame_stats defined above frame_pipeline) -------- */

void frame_pipeline_stats(struct frame_pipeline *p, struct frame_stats *out);

#ifdef __cplusplus
}
#endif

#endif /* FRAME_PIPELINE_H */
