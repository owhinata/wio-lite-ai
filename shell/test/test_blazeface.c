/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/*
 * Host unit test for port/nn/models/blazeface.c (issue #9 phase 3).
 *
 * The decoder is the one genuinely new piece of arithmetic this phase adds, and it
 * is the piece whose failure mode is silent: wrong anchor indexing, a wrong scale or
 * an off-by-512 in the second group all produce boxes that are plausible on screen
 * and wrong.  On the board the only evidence would be "the rectangle does not sit on
 * the face", which is also what bad exposure, the wrong normalization and a wrong
 * threshold look like.  So it is pinned here, where the expected box can be computed
 * by hand.
 *
 * blazeface.c reaches into the nn layer through exactly two functions --
 * nn_output_count() and nn_output() -- and nn.h is HAL/ThreadX-free, so the whole
 * decoder runs on the host against the stub at the bottom of this file.  struct
 * nn_model is opaque in nn.h (nn.c defines it), which is precisely what lets the
 * stub define its own.
 */
#include "blazeface.h"
#include "nn.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define CHECK(cond)                                                              \
	do {                                                                         \
		if (!(cond)) {                                                           \
			printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);               \
			exit(1);                                                             \
		}                                                                        \
	} while (0)

#define CLOSE(a, b) (fabs((double)(a) - (double)(b)) < 1e-4)

/* ------------------------------------------------------------------ stub nn ---- */

#define A512 512
#define A384 384
#define BOXC 16

struct nn_model {
	int dummy;
};

static struct nn_model stub_model;

static float box512[A512 * BOXC];
static float scr512[A512];
static float box384[A384 * BOXC];
static float scr384[A384];

/* NN_MAX_IO is 8; BlazeFace publishes 4 outputs. */
static struct nn_tensor stub_out[4];
static int stub_nout;

int nn_output_count(const struct nn_model *m)
{
	(void)m;
	return stub_nout;
}

struct nn_tensor *nn_output(struct nn_model *m, int idx)
{
	(void)m;
	if (idx < 0 || idx >= stub_nout)
		return NULL;
	return &stub_out[idx];
}

static void set_tensor(int i, void *data, uint32_t bytes, uint16_t a, uint16_t c,
                       uint8_t dtype)
{
	memset(&stub_out[i], 0, sizeof(stub_out[i]));
	stub_out[i].data = data;
	stub_out[i].bytes = bytes;
	stub_out[i].ndim = 3;
	stub_out[i].dims[0] = 1;
	stub_out[i].dims[1] = a;
	stub_out[i].dims[2] = c;
	stub_out[i].dims[3] = 1;
	stub_out[i].dtype = dtype;
}

/*
 * Publish BlazeFace-shaped outputs, deliberately NOT in the order the decoder looks
 * them up: bf_find() is supposed to match on shape, so a build that quietly went
 * back to indexing must fail here.
 */
static void publish_blazeface(void)
{
	memset(box512, 0, sizeof(box512));
	memset(box384, 0, sizeof(box384));
	for (int i = 0; i < A512; i++)
		scr512[i] = -10.0f;
	for (int i = 0; i < A384; i++)
		scr384[i] = -10.0f;

	set_tensor(0, scr384, sizeof(scr384), A384, 1, NN_DTYPE_FLOAT32);
	set_tensor(1, box512, sizeof(box512), A512, BOXC, NN_DTYPE_FLOAT32);
	set_tensor(2, scr512, sizeof(scr512), A512, 1, NN_DTYPE_FLOAT32);
	set_tensor(3, box384, sizeof(box384), A384, BOXC, NN_DTYPE_FLOAT32);
	stub_nout = 4;
}

/* Place one detection in the 16x16 layer at grid cell (gx, gy), anchor a (0..1). */
static int put512(int gx, int gy, int a, float dx, float dy, float w, float h,
                  float raw_score)
{
	int idx = ((gy * 16) + gx) * 2 + a;

	box512[idx * BOXC + 0] = dx;
	box512[idx * BOXC + 1] = dy;
	box512[idx * BOXC + 2] = w;
	box512[idx * BOXC + 3] = h;
	scr512[idx] = raw_score;
	return idx;
}

/* Place one detection in the 8x8 layer at grid cell (gx, gy), anchor a (0..5). */
static int put384(int gx, int gy, int a, float dx, float dy, float w, float h,
                  float raw_score)
{
	int idx = ((gy * 8) + gx) * 6 + a;

	box384[idx * BOXC + 0] = dx;
	box384[idx * BOXC + 1] = dy;
	box384[idx * BOXC + 2] = w;
	box384[idx * BOXC + 3] = h;
	scr384[idx] = raw_score;
	return idx;
}

/* ---------------------------------------------------------------- the tests ---- */

/* A model whose outputs are not BlazeFace-shaped must be a no-op, not a crash: the
 * CONFIG_NN_BACKEND=null build publishes exactly this and `ai dets` calls in anyway. */
static void test_not_blazeface(void)
{
	struct bf_det out[BF_MAX_DET];

	/* the null backend's shapes: 1x256x16 int8 + 1x256 float32 */
	set_tensor(0, box512, sizeof(box512), 256, BOXC, NN_DTYPE_INT8);
	set_tensor(1, scr512, sizeof(scr512), 256, 1, NN_DTYPE_FLOAT32);
	stub_nout = 2;
	CHECK(blazeface_decode(&stub_model, out, BF_MAX_DET) == -1);

	/* right shapes, wrong dtype (int8 scores) */
	publish_blazeface();
	stub_out[2].dtype = NN_DTYPE_INT8;
	CHECK(blazeface_decode(&stub_model, out, BF_MAX_DET) == -1);

	/* right shape, buffer too small for anchors*chan floats -- the OOB guard */
	publish_blazeface();
	stub_out[1].bytes = 16u;
	CHECK(blazeface_decode(&stub_model, out, BF_MAX_DET) == -1);

	/* NULL buffer */
	publish_blazeface();
	stub_out[3].data = NULL;
	CHECK(blazeface_decode(&stub_model, out, BF_MAX_DET) == -1);

	/* bad arguments */
	publish_blazeface();
	CHECK(blazeface_decode(NULL, out, BF_MAX_DET) == -1);
	CHECK(blazeface_decode(&stub_model, NULL, BF_MAX_DET) == -1);
	CHECK(blazeface_decode(&stub_model, out, 0) == -1);
}

/*
 * The load-bearing arithmetic: a single anchor in the 16x16 layer must decode to the
 * box computed by hand from the MediaPipe convention.  Cell (8,8) has centre
 * (8.5/16, 8.5/16) = (0.53125, 0.53125); the regressors are divided by 128 and the
 * box is centre-to-corner.
 */
static void test_decode_512(void)
{
	struct bf_det out[BF_MAX_DET];
	int n;

	publish_blazeface();
	blazeface_set_thresh_milli(BF_DEFAULT_THRESH_MILLI);
	put512(8, 8, 0, 0.0f, 0.0f, 25.6f, 25.6f, 2.0f);

	n = blazeface_decode(&stub_model, out, BF_MAX_DET);
	CHECK(n == 1);
	CHECK(CLOSE(out[0].w, 0.2f));                 /* 25.6 / 128            */
	CHECK(CLOSE(out[0].h, 0.2f));
	CHECK(CLOSE(out[0].x, 0.53125f - 0.1f));      /* centre - w/2          */
	CHECK(CLOSE(out[0].y, 0.53125f - 0.1f));
	/* algebraic sigmoid: 0.5 + 0.5*2/(1+2) = 0.8333 */
	CHECK(CLOSE(out[0].score, 0.83333f));
	CHECK(CLOSE(blazeface_last_max_score(), 2.0f));
	CHECK(blazeface_last_ncand() == 1);

	/* a non-zero centre offset shifts the box by dx/128, dy/128 */
	publish_blazeface();
	put512(8, 8, 0, 12.8f, -12.8f, 25.6f, 25.6f, 2.0f);
	n = blazeface_decode(&stub_model, out, BF_MAX_DET);
	CHECK(n == 1);
	CHECK(CLOSE(out[0].x, 0.53125f + 0.1f - 0.1f));
	CHECK(CLOSE(out[0].y, 0.53125f - 0.1f - 0.1f));
}

/*
 * The 8x8 layer is indexed with anchor_off = 512.  An off-by-512 here would still
 * produce boxes -- just centred on the wrong cell -- so it is worth its own case.
 * Cell (2,5) of the 8x8 grid has centre (2.5/8, 5.5/8) = (0.3125, 0.6875).
 */
static void test_decode_384(void)
{
	struct bf_det out[BF_MAX_DET];
	int n;

	publish_blazeface();
	put384(2, 5, 3, 0.0f, 0.0f, 12.8f, 12.8f, 3.0f);

	n = blazeface_decode(&stub_model, out, BF_MAX_DET);
	CHECK(n == 1);
	CHECK(CLOSE(out[0].w, 0.1f));
	CHECK(CLOSE(out[0].x, 0.3125f - 0.05f));
	CHECK(CLOSE(out[0].y, 0.6875f - 0.05f));
}

/* NMS: identical overlapping boxes collapse to one, distant boxes both survive, and
 * the survivor is the higher-scoring of an overlapping pair. */
static void test_nms(void)
{
	struct bf_det out[BF_MAX_DET];
	int n;

	/* two anchors of the SAME cell with the same box -> IoU 1.0 -> one output */
	publish_blazeface();
	put512(8, 8, 0, 0.0f, 0.0f, 25.6f, 25.6f, 2.0f);
	put512(8, 8, 1, 0.0f, 0.0f, 25.6f, 25.6f, 3.0f);
	n = blazeface_decode(&stub_model, out, BF_MAX_DET);
	CHECK(blazeface_last_ncand() == 2);
	CHECK(n == 1);
	CHECK(CLOSE(out[0].score, 0.5f + 0.5f * 3.0f / 4.0f));   /* the higher one */

	/* opposite corners of the frame -> no overlap -> both survive */
	publish_blazeface();
	put512(1, 1, 0, 0.0f, 0.0f, 12.8f, 12.8f, 2.0f);
	put512(14, 14, 0, 0.0f, 0.0f, 12.8f, 12.8f, 2.5f);
	n = blazeface_decode(&stub_model, out, BF_MAX_DET);
	CHECK(n == 2);
	CHECK(out[0].score > out[1].score);   /* emitted highest-score first */

	/* max caps the output even when more survive */
	publish_blazeface();
	for (int i = 0; i < 6; i++)
		put512(i * 2, 1, 0, 0.0f, 0.0f, 6.4f, 6.4f, 2.0f + (float)i);
	CHECK(blazeface_decode(&stub_model, out, 3) == 3);
}

/* The threshold knob has to reach the comparison, and its two units have to agree. */
static void test_threshold(void)
{
	struct bf_det out[BF_MAX_DET];

	/* The default milli IS the donor's tuned logit, to within the resolution of an
	 * integer milli: sigmoid(0.405) = 0.644127, and inverting the rounded 644 gives
	 * 0.40449 back.  That 0.0005 of logit is 0.05% of probability -- worth stating
	 * rather than hiding behind a loose epsilon everywhere else in this test. */
	blazeface_set_thresh_milli(BF_DEFAULT_THRESH_MILLI);
	CHECK(blazeface_get_thresh_milli() == BF_DEFAULT_THRESH_MILLI);
	CHECK(fabs(blazeface_get_thresh_logit() - 0.405) < 1e-3);

	/* p = 0.5 is logit 0 for the algebraic sigmoid too */
	blazeface_set_thresh_milli(500);
	CHECK(CLOSE(blazeface_get_thresh_logit(), 0.0f));

	/* round trip through the inverse: sigmoid(logit(p)) == p */
	for (unsigned milli = 100; milli <= 900; milli += 100) {
		float x, p;

		blazeface_set_thresh_milli(milli);
		x = blazeface_get_thresh_logit();
		p = 0.5f + 0.5f * x / (1.0f + (x < 0.0f ? -x : x));
		CHECK(CLOSE(p, (float)milli / 1000.0f));
	}

	/* the poles are clamped, not divided by zero */
	blazeface_set_thresh_milli(0);
	CHECK(blazeface_get_thresh_milli() == 1);
	CHECK(blazeface_get_thresh_logit() < 0.0f);
	blazeface_set_thresh_milli(100000);
	CHECK(blazeface_get_thresh_milli() == 999);
	CHECK(blazeface_get_thresh_logit() > 0.0f);

	/* and it actually filters: raw 2.0 -> reported 0.833 */
	publish_blazeface();
	put512(8, 8, 0, 0.0f, 0.0f, 25.6f, 25.6f, 2.0f);
	blazeface_set_thresh_milli(800);
	CHECK(blazeface_decode(&stub_model, out, BF_MAX_DET) == 1);
	blazeface_set_thresh_milli(900);
	CHECK(blazeface_decode(&stub_model, out, BF_MAX_DET) == 0);

	/* 🔴 and the peak raw score is still reported when NOTHING passes -- that is
	 * the whole point of the diagnostic: it separates "the model is dead" from
	 * "the threshold is too high", which the detection count cannot. */
	CHECK(CLOSE(blazeface_last_max_score(), 2.0f));
	CHECK(blazeface_last_ncand() == 0);

	blazeface_set_thresh_milli(BF_DEFAULT_THRESH_MILLI);
}

/* NaN/Inf in either the score or the box must be dropped, not propagated. */
static void test_nonfinite(void)
{
	struct bf_det out[BF_MAX_DET];
	float inf = (float)(1.0 / 0.0);
	float nan = (float)(0.0 / 0.0);

	publish_blazeface();
	scr512[0] = nan;
	scr512[2] = inf;
	put512(8, 8, 0, nan, 0.0f, 25.6f, 25.6f, 2.0f);   /* good score, bad box */
	CHECK(blazeface_decode(&stub_model, out, BF_MAX_DET) == 0);

	/* a zero or negative extent is not a box either */
	publish_blazeface();
	put512(8, 8, 0, 0.0f, 0.0f, 0.0f, 25.6f, 2.0f);
	CHECK(blazeface_decode(&stub_model, out, BF_MAX_DET) == 0);
	publish_blazeface();
	put512(8, 8, 0, 0.0f, 0.0f, -25.6f, 25.6f, 2.0f);
	CHECK(blazeface_decode(&stub_model, out, BF_MAX_DET) == 0);
}

int main(void)
{
	test_not_blazeface();
	test_decode_512();
	test_decode_384();
	test_nms();
	test_threshold();
	test_nonfinite();
	printf("test_blazeface: ok\n");
	return 0;
}
