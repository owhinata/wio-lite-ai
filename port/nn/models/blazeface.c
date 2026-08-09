/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    blazeface.c
 * @brief   BlazeFace-front 128 face-detection decode (issue #9 phase 3).  See blazeface.h.
 *
 * SSD anchor decode + NMS, ported from the MediaPipe/PINTO BlazeFace convention
 * (the ST Model Zoo model's provenance).  No libm dependency: score filtering
 * compares the raw pre-sigmoid score against a stored logit, and the reported
 * confidence uses an algebraic fast-sigmoid, so we never call expf().
 *
 * The four output tensors are located by SHAPE, not index (the generated order is
 * not the README order): C==16 => box regressors, C==1 => scores; anchors==512 is
 * the 16x16 layer, anchors==384 is the 8x8 layer.  A model whose outputs do not
 * match returns -1 without touching anything, which is what lets `ai dets` be
 * registered unconditionally -- including in the CONFIG_NN_BACKEND=null build,
 * whose stub publishes 1x256x16 int8 and 1x256 float32 and therefore matches
 * nothing here.
 */
#include "blazeface.h"
#include "nn.h"

#include "mem_sections.h"

#include <stddef.h>
#include <stdint.h>

#define BF_INPUT       128.0f     /* box/score scale (x/y/w/h_scale in MediaPipe) */
#define BF_A512        512
#define BF_A384        384
#define BF_NANCHOR     (BF_A512 + BF_A384)   /* 896 */
#define BF_BOX_STRIDE  16         /* 4 bbox + 12 (6 keypoints x 2) per anchor      */

#define BF_NMS_IOU     0.5f
#define BF_MAX_CAND    64

/*
 * Anchor cell centres (normalized) + pre-NMS candidate scratch, in the cacheable
 * PSRAM carve-out (issue #9 phase 2a).  bf_cx/bf_cy are computed once and read many
 * times, bf_cand is write-before-read scratch, and every access is CPU-only from the
 * single serialized caller -- which is the whole precondition PSRAM_AI states, and
 * the row of the memory-placement table this carve-out exists for.  w=h=1 is fixed
 * for the anchors, so only the centre is stored.
 *
 * The section is NOLOAD, so none of these start at zero.  bf_anchors_ready is
 * deliberately NOT in here: it is the flag that decides whether the (uninitialised)
 * anchor arrays have been filled yet, so it has to live somewhere the startup code
 * actually zeroes.
 */
static float bf_cx[BF_NANCHOR] PSRAM_AI __attribute__((aligned(32)));
static float bf_cy[BF_NANCHOR] PSRAM_AI __attribute__((aligned(32)));
static struct bf_det bf_cand[BF_MAX_CAND] PSRAM_AI __attribute__((aligned(32)));

static int bf_anchors_ready;   /* .bss -- see above */

/* Diagnostics: peak raw model response and pre-NMS candidate count of the last
 * decode.  Between them they separate "the model saw nothing" from "the threshold
 * is too high" from "NMS merged everything", which one detection count cannot. */
static float bf_last_max = -1e9f;
static int   bf_last_ncand;

float blazeface_last_max_score(void) { return bf_last_max; }
int   blazeface_last_ncand(void)     { return bf_last_ncand; }

static float bf_fabsf(float v) { return v < 0.0f ? -v : v; }

/* Algebraic fast sigmoid (no expf); adequate for a displayed confidence. */
static float bf_sigmoid(float x)
{
	return 0.5f + 0.5f * x / (1.0f + bf_fabsf(x));
}

/*
 * Exact inverse of bf_sigmoid, so the threshold the user types and the score the
 * board prints are on one scale.  p = 0.5 + 0.5x/(1+|x|)  =>  u = 2p-1 = x/(1+|x|)
 * =>  x = u/(1-|u|), for either sign of x.  The poles at u = +-1 are why the milli
 * value is clamped to 1..999 before it gets here.
 */
static float bf_logit(float p)
{
	float u = 2.0f * p - 1.0f;

	return u / (1.0f - bf_fabsf(u));
}

/*
 * Default = the donor's tuned raw logit 0.405, which is what actually detects faces.
 * BF_DEFAULT_THRESH_MILLI is that same point in the friendly unit (sigmoid(0.405) =
 * 0.644127 -> 644), so `ai thresh` with no argument reports the default rather than
 * some other number, and re-setting 644 lands within 0.0005 of the logit below --
 * the resolution of an integer milli, not a discrepancy.
 */
static unsigned bf_thresh_milli = BF_DEFAULT_THRESH_MILLI;
static float    bf_thresh_logit = 0.405f;

void blazeface_set_thresh_milli(unsigned milli)
{
	if (milli < 1u)
		milli = 1u;
	if (milli > 999u)
		milli = 999u;
	bf_thresh_milli = milli;
	bf_thresh_logit = bf_logit((float)milli / 1000.0f);
}

unsigned blazeface_get_thresh_milli(void) { return bf_thresh_milli; }
float    blazeface_get_thresh_logit(void) { return bf_thresh_logit; }

static void bf_gen_anchors(void)
{
	int idx = 0;

	/* layer 0: 16x16 grid, 2 anchors/cell -> 512 (matches the 1x512x* tensors) */
	for (int y = 0; y < 16; y++)
		for (int x = 0; x < 16; x++)
			for (int a = 0; a < 2; a++) {
				(void)a;
				bf_cx[idx] = ((float)x + 0.5f) / 16.0f;
				bf_cy[idx] = ((float)y + 0.5f) / 16.0f;
				idx++;
			}
	/* layer 1: 8x8 grid, 6 anchors/cell -> 384 (matches the 1x384x* tensors) */
	for (int y = 0; y < 8; y++)
		for (int x = 0; x < 8; x++)
			for (int a = 0; a < 6; a++) {
				(void)a;
				bf_cx[idx] = ((float)x + 0.5f) / 8.0f;
				bf_cy[idx] = ((float)y + 0.5f) / 8.0f;
				idx++;
			}
	bf_anchors_ready = 1;   /* idx == BF_NANCHOR */
}

/* IoU of two normalized boxes. */
static float bf_iou(const struct bf_det *a, const struct bf_det *b)
{
	float ax2 = a->x + a->w, ay2 = a->y + a->h;
	float bx2 = b->x + b->w, by2 = b->y + b->h;
	float ix1 = a->x > b->x ? a->x : b->x;
	float iy1 = a->y > b->y ? a->y : b->y;
	float ix2 = ax2 < bx2 ? ax2 : bx2;
	float iy2 = ay2 < by2 ? ay2 : by2;
	float iw = ix2 - ix1, ih = iy2 - iy1;
	float inter, uni;

	if (iw <= 0.0f || ih <= 0.0f)
		return 0.0f;
	inter = iw * ih;
	uni = a->w * a->h + b->w * b->h - inter;
	return uni > 0.0f ? inter / uni : 0.0f;
}

static int bf_finite(float x) { return x == x && x < 3.0e38f && x > -3.0e38f; }

/*
 * Find an output tensor by (anchors, channels).  Validates dtype, exact shape, a
 * non-NULL buffer, and that it is big enough for anchors*chan floats -- so a
 * malformed or same-shaped-but-smaller tensor can never cause an OOB read.
 */
static const float *bf_find(struct nn_model *m, int anchors, int chan)
{
	for (int i = 0; i < nn_output_count(m); i++) {
		struct nn_tensor *t = nn_output(m, i);

		if (t && t->dtype == NN_DTYPE_FLOAT32 && t->ndim == 3 && t->dims[0] == 1 &&
		    t->dims[1] == anchors && t->dims[2] == chan && t->data &&
		    t->bytes >= (uint32_t)anchors * (uint32_t)chan * sizeof(float))
			return (const float *)t->data;
	}
	return NULL;
}

/* Decode one anchor group (box[anchors*16], score[anchors]) into bf_cand. */
static int bf_decode_group(const float *box, const float *score, int anchors,
                           int anchor_off, int ncand)
{
	for (int i = 0; i < anchors && ncand < BF_MAX_CAND; i++) {
		const float *r = box + (size_t)i * BF_BOX_STRIDE;
		float cx, cy, w, h;

		if (!bf_finite(score[i]))           /* reject NaN/Inf scores            */
			continue;
		if (score[i] > bf_last_max)         /* diagnostic: peak model response  */
			bf_last_max = score[i];
		if (score[i] <= bf_thresh_logit)    /* pre-sigmoid threshold            */
			continue;

		cx = r[0] / BF_INPUT + bf_cx[anchor_off + i];
		cy = r[1] / BF_INPUT + bf_cy[anchor_off + i];
		w  = r[2] / BF_INPUT;
		h  = r[3] / BF_INPUT;
		if (!bf_finite(cx) || !bf_finite(cy) || !bf_finite(w) || !bf_finite(h))
			continue;                       /* reject NaN/Inf boxes             */
		if (w <= 0.0f || h <= 0.0f)
			continue;

		bf_cand[ncand].x = cx - w * 0.5f;
		bf_cand[ncand].y = cy - h * 0.5f;
		bf_cand[ncand].w = w;
		bf_cand[ncand].h = h;
		bf_cand[ncand].score = bf_sigmoid(score[i]);
		ncand++;
	}
	return ncand;
}

int blazeface_decode(struct nn_model *m, struct bf_det *out, int max)
{
	const float *box512, *scr512, *box384, *scr384;
	int ncand = 0, nout = 0;
	uint8_t used[BF_MAX_CAND] = { 0 };

	if (!m || !out || max <= 0)
		return -1;

	box512 = bf_find(m, BF_A512, BF_BOX_STRIDE);
	scr512 = bf_find(m, BF_A512, 1);
	box384 = bf_find(m, BF_A384, BF_BOX_STRIDE);
	scr384 = bf_find(m, BF_A384, 1);
	if (!box512 || !scr512 || !box384 || !scr384)
		return -1;                       /* not BlazeFace-shaped -> no-op */

	if (!bf_anchors_ready)
		bf_gen_anchors();

	bf_last_max = -1e9f;
	ncand = bf_decode_group(box512, scr512, BF_A512, 0, ncand);
	ncand = bf_decode_group(box384, scr384, BF_A384, BF_A512, ncand);
	bf_last_ncand = ncand;

	/* Hard NMS: repeatedly take the highest-scoring unused box, suppress the
	 * rest that overlap it beyond BF_NMS_IOU. */
	while (nout < max) {
		int best = -1;
		float best_s = 0.0f;

		for (int i = 0; i < ncand; i++)
			if (!used[i] && bf_cand[i].score > best_s) {
				best_s = bf_cand[i].score;
				best = i;
			}
		if (best < 0)
			break;

		out[nout++] = bf_cand[best];
		used[best] = 1;
		for (int i = 0; i < ncand; i++)
			if (!used[i] && bf_iou(&bf_cand[best], &bf_cand[i]) > BF_NMS_IOU)
				used[i] = 1;
	}
	return nout;
}
