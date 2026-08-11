/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    nn_camera.h
 * @brief   Camera -> BlazeFace inference glue (issue #9 phase 3).
 *
 * Drives a worker thread that repeatedly asks the band stream for one frame,
 * downsamples it straight into the model's input tensor, runs inference and
 * publishes decoded face boxes.
 *
 * WHY THIS IS IN app/ AND NOT port/nn/.  The donor firmware keeps the equivalent
 * glue under port/nn/, and it can: it has a camera subscriber registry, so its glue
 * only ever calls DOWN.  Here the glue claims a band stream, publishes boxes for the
 * display and is driven by a shell command -- that is board integration, and this
 * repository already has the shape for it in app/cam_preview.c, which binds
 * port/camera to port/ltdc from app/.  Nothing under port/ includes an app/ header
 * (checked, not assumed), and keeping it that way is what leaves port/nn exactly
 * what its own header claims to be: a model-agnostic, hardware-free inference API.
 *
 * NO STAGING BUFFER, AND THAT IS THE INTERESTING PART.  The donor stages frames
 * through two 192 KB buffers behind a four-state machine with epoch counters.  Here
 * the ratio does that job instead: inference is ~373 ms and a frame period is ~74 ms,
 * so a fresh frame is always available the moment one is wanted.  What is needed is
 * not buffering but a way to say "fill me one":
 *
 *     worker:   want_frame = 1 -> wait(sem) -> nn_run() -> decode -> publish -> repeat
 *     band cb:  band 0 && want_frame -> latch filling
 *               filling -> downsample this band's rows into nn_input()->data
 *               band 3  -> filling = 0; want_frame = 0; post(sem)
 *
 * While want_frame is 0 the producer does not touch the input tensor at all, and the
 * worker owns it exclusively for the whole inference.  That removes the staging
 * buffers, the state machine, the epoch counters and a 196,608 B memcpy per
 * inference -- but ONLY because of the properties spelled out on nn_camera_stop()
 * and nn_camera_start() below.  The handoff IS the correctness argument here.
 */
#ifndef NN_CAMERA_H
#define NN_CAMERA_H

#include <stdint.h>

#include "blazeface.h"   /* struct bf_det / BF_MAX_DET */

#define NNCAM_OK           0
#define NNCAM_ERR_RUNNING (-1)  /**< a stream is already running                 */
#define NNCAM_ERR_NOTRUN  (-2)  /**< nothing to stop                             */
#define NNCAM_ERR_MODEL   (-3)  /**< no model loaded, or it has no input tensor   */
#define NNCAM_ERR_SESSION (-4)  /**< the NN session is held (bench / model load)  */
#define NNCAM_ERR_PSRAM   (-5)  /**< PSRAM down, or OCTOSPI1 held by a retuner    */
#define NNCAM_ERR_BAND    (-6)  /**< the camera would not start a band stream     */
#define NNCAM_ERR_GEOM    (-7)  /**< the model input does not tile onto the bands */
#define NNCAM_ERR_INIT    (-8)  /**< thread / semaphore / mutex creation failed   */
#define NNCAM_ERR_TEARING (-9)  /**< stop: still tearing down -- see below        */
#define NNCAM_ERR_REARM  (-10)  /**< re-arm after a lost stream failed; stop first */
#define NNCAM_ERR_QUANT  (-11)  /**< int8 input without a per-tensor quant scale   */

struct nn_camera_stats {
	uint8_t  running;
	uint8_t  holds_guards;   /**< the session + OCTOSPI1 guard are still held  */
	uint8_t  stream_lost;    /**< the band stream died under us (latched)      */
	uint8_t  norm_signed;    /**< input range: 1 = [-1,1], 0 = [0,1]           */
	uint8_t  overlay;
	uint32_t infers;
	uint32_t frames;         /**< complete frames ingested into the tensor     */
	uint32_t skipped;        /**< complete frames that passed while busy       */
	uint32_t errors;
	uint32_t raced;          /**< bands that wrote the tensor mid-inference (#54) */
	uint32_t stale_posts;    /**< frame posts discarded by the pre-arm drain (#54) */
	uint32_t ingest_last_cyc;/**< DWT cycles of the last band's downsample     */
	uint32_t ingest_max_cyc; /**< worst band since start -- vs the ~18.5 ms deadline */
	uint32_t infer_last_cyc;
	uint32_t elapsed_ms;
	int      ndet;
};

/**
 * Claim the band stream and start inferring.
 *
 * Holds the NN session AND psram_acquire_shared() for the WHOLE lifetime of the
 * stream, not per inference.  Both are deliberate:
 *
 *  - The session being held is what makes `ai model load` refuse while streaming,
 *    which matters more than it looks: a reload rebuilds the interpreter and
 *    re-plans the arena, so nn_input()->data MOVES.  (The band callback re-reads
 *    that pointer every frame anyway, rather than caching it across a session.)
 *  - 🔴 The OCTOSPI1 guard closes a hole camera_streaming() does not cover.
 *    psram_acquire() consults only the camera and the LTDC, so the instant a DCMI
 *    overrun tears the band stream down while the worker is still inside nn_run()
 *    reading the arena, a `psram clk` retune would become legal underneath it.
 *    Per-inference holding would refuse the same commands AND leave a gap between
 *    inferences for exactly that.
 *
 * The documented consequence: START THE PREVIEW BEFORE `ai stream start`.  The guard
 * coexists with an already-armed LTDC scan-out and band DMA, but it refuses a NEW
 * `lcd on` / `lcd reset` / `camera capture` / `camera preview on` while held -- the
 * same behaviour `ai bench` has, for the same reason.
 */
int nn_camera_start(int colorbar);

/**
 * Stop inferring, drain, and release the guards.
 *
 * 🔴 RETURNS NNCAM_ERR_TEARING WITHOUT RELEASING ANYTHING if either side is still
 * in flight: a band callback that never returned (a producer killed by a DCMI
 * overrun) or a worker still inside nn_run().  Releasing early would hand the model
 * and its arena to `ai bench` while something can still write the input tensor.
 * Holding a session nobody can use is recoverable; that is not.  Calling stop again
 * re-checks and completes the release, which is why it is idempotent.
 */
int nn_camera_stop(void);

int nn_camera_running(void);
void nn_camera_stats_get(struct nn_camera_stats *out);

/**
 * Copy the most recently published detections.  Returns how many were copied, or -1
 * if the decoder does not recognise the loaded model's outputs at all -- the same
 * convention blazeface_decode() uses, because it is the same answer travelling
 * through the worker.  -1 and 0 are different facts and the caller must not merge
 * them: one is "this is not a face model", the other is "no faces" (issue #57).
 */
int nn_camera_dets_get(struct bf_det *out, int max);

/** Input normalization: 1 = [-1,1], 0 = [0,1] (default).  Applies to float32 and
 *  quantized inputs alike -- a quantized input is the normalized value put through
 *  the tensor's own scale/zero_point. */
void nn_camera_set_norm(int signed_range);
int  nn_camera_get_norm(void);

/** Draw the boxes on the LCD preview (app/cam_preview.c does the drawing). */
void nn_camera_set_overlay(int on);
int  nn_camera_get_overlay(void);

#endif /* NN_CAMERA_H */
