/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    blazeface.h
 * @brief   BlazeFace-front 128 face-detection decode (issue #9 phase 3).
 *
 * Model-specific post-processing for ST Model Zoo's BlazeFace Front 128x128 (a
 * MediaPipe/PINTO-derived SSD face detector).  Lives ABOVE the backend-agnostic
 * nn layer, which nn.h deliberately keeps model-agnostic: this turns the 4 raw
 * float32 output tensors (2 anchor scales x {box, score}) into a short list of
 * face bounding boxes via SSD anchor decode + non-max suppression.
 *
 * Anchors (896 total, MediaPipe BlazeFace-front layout):
 *   - 16x16 grid (stride 8), 2 anchors/cell  -> 512  (the 1x512x* tensors)
 *   - 8x8  grid (stride 16), 6 anchors/cell  -> 384  (the 1x384x* tensors)
 * Each anchor is a fixed-size cell-centre point (w=h=1, normalized).
 *
 * This file depends on nn.h alone -- no HAL, no ThreadX, no hardware and no libm --
 * which is why it stays in port/ while the camera glue that drives it lives in
 * app/nn_camera.c, and why shell/test/test_blazeface.c can run the whole decoder
 * on the host against a stubbed nn_output().
 */
#ifndef BLAZEFACE_H
#define BLAZEFACE_H

#include <stdint.h>

struct nn_model;   /* port/nn/nn.h */

/** One detection: normalized [0,1] box (top-left origin) + confidence. */
struct bf_det {
	float x, y, w, h;   /* normalized to the 128x128 input frame */
	float score;        /* sigmoid confidence 0..1                */
};

/** Max detections returned (post-NMS). */
#define BF_MAX_DET 8

/**
 * Default score threshold in milli-probability.
 *
 * The donor firmware tuned this as a raw pre-sigmoid logit of 0.405 and that is the
 * value proven to detect faces, so the default here IS that same logit -- expressed
 * in the friendly unit via the same algebraic sigmoid the reported scores use:
 * bf_sigmoid(0.405) = 0.5 + 0.5*0.405/1.405 = 0.644.  Keeping the unit consistent
 * with what `ai dets` prints matters more than the round number 600 would look:
 * "threshold 644" and "score 71%" are then the same scale, and a box is shown
 * exactly when its printed score clears the printed threshold.
 */
#define BF_DEFAULT_THRESH_MILLI 644u

/**
 * Decode BlazeFace outputs from model @p m into @p out[0..max).  Returns the
 * number of detections (>=0), or -1 if @p m's outputs are NOT BlazeFace-shaped
 * (a safe no-op for other models -- callers may invoke it unconditionally, which
 * is what makes it harmless in a CONFIG_NN_BACKEND=null build).
 */
int blazeface_decode(struct nn_model *m, struct bf_det *out, int max);

/**
 * Score threshold as a milli-probability on the same scale as bf_det.score.
 * Clamped to 1..999 (0 and 1000 are the poles of the inverse sigmoid).
 */
void     blazeface_set_thresh_milli(unsigned milli);
unsigned blazeface_get_thresh_milli(void);

/** The same threshold as the raw pre-sigmoid logit actually compared against. */
float blazeface_get_thresh_logit(void);

/** Diagnostic: highest raw (pre-sigmoid) score seen in the last decode. */
float blazeface_last_max_score(void);

/**
 * Diagnostic: candidates that passed the threshold in the last decode, BEFORE NMS.
 * Separates "the model responded to nothing" from "NMS collapsed everything into
 * one box" -- two states the detection count alone cannot tell apart.
 */
int blazeface_last_ncand(void);

#endif /* BLAZEFACE_H */
