/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    nn_null.c
 * @brief   `null` nn backend -- a synthetic stub, not an inference runtime (#9 P1).
 *
 * The always-buildable backend.  It exists so the whole path -- CMake backend
 * selection, the vtable, the singleton open, the session guard, the cycle
 * measurement and the `ai` command -- can be brought up and proven on the board
 * before TFLM, C++ and a real model arrive.
 *
 * IT DOES NOT INFER ANYTHING, and it says so: the model is named "null-stub" and
 * `ai info` prints a note.  But unlike the donor firmware's version, whose run() is
 * an empty `return 0`, this one does a deterministic pass over the input and writes
 * derived values to both outputs.  An empty run() reports zero cycles, and zero is
 * exactly the reading a BROKEN cycle counter gives -- the one number that cannot
 * distinguish "measured correctly" from "measured nothing".  A real pass makes
 * `ai bench` report a stable non-zero latency, and makes the outputs change when
 * the input changes, so the tensor plumbing is observably end-to-end.
 *
 * SHAPES.  Deliberately not the donor's BlazeFace-shaped 128x128 stub.  That
 * geometry earned its keep there because that firmware's P1 also shipped the camera
 * inference sink; here the sink is a later phase and will run against the real
 * model, so all this stub owes us is one multi-dimensional quantized input, more
 * than one output, and both a quantized and a non-quantized output so `ai info`
 * exercises both print paths.
 *
 * PLACEMENT.  All three buffers live in the cacheable PSRAM carve-out (.psram_ai,
 * issue #9 phase 2a).  They are 17 KB of CPU-only bulk, which is the PSRAM row of
 * the memory policy in include/mem_sections.h: AXI-SRAM is reserved for what a bus
 * master must reach, and DTCM has under 8 KB genuinely free once the main stack is
 * accounted for.  Being NOLOAD they hold garbage at reset, so a caller measuring
 * with them fills the input first (shell/cmds/cmd_ai.c).  Nothing here dereferences
 * them outside run() -- in particular null_open() only fills descriptors, which is
 * what lets `ai info` work with the PSRAM down.
 *
 * Clean-room design; no third-party code reused.
 */
#include "nn.h"
#include "nn_backend.h"

#include <stddef.h>
#include <stdint.h>

#include "mem_sections.h"   /* PSRAM_AI: the cacheable, CPU-only carve-out */

/* Stub geometry.  NHWC input, two outputs shaped like a small detector head. */
#define NULL_IN_H      64
#define NULL_IN_W      64
#define NULL_IN_C      3
#define NULL_IN_BYTES  ((uint32_t)NULL_IN_H * NULL_IN_W * NULL_IN_C)   /* 12288 */
#define NULL_CELLS     256
#define NULL_BOX_LANES 16
#define NULL_BOX_BYTES ((uint32_t)NULL_CELLS * NULL_BOX_LANES)         /*  4096 */
#define NULL_SCR_COUNT ((uint32_t)NULL_CELLS)
#define NULL_SCR_BYTES (NULL_SCR_COUNT * (uint32_t)sizeof(float))      /*  1024 */

/* In the CACHEABLE PSRAM carve-out since issue #9 phase 2a (the rest of that window
 * is non-cacheable).  32-byte aligned so no two of them share a cache line, which
 * matters here in a way it did not while the window was uniform. */
static int8_t null_in_buf[NULL_IN_BYTES]   PSRAM_AI __attribute__((aligned(32)));
static int8_t null_box_buf[NULL_BOX_BYTES] PSRAM_AI __attribute__((aligned(32)));
static float  null_scr_buf[NULL_SCR_COUNT] PSRAM_AI __attribute__((aligned(32)));

/* Defeats dead-store elimination of the pass below.  The firmware is built -Os with
 * LTO, so the optimiser sees the whole program and would otherwise be free to prove
 * that nothing reads these buffers and delete the loop that fills them -- the same
 * trap cmd_membench.c guards with its own sink, and the one that silently removed
 * issue #14's malloc probe. */
static volatile uint32_t null_sink;

struct null_model {
	struct nn_tensor in[1];
	struct nn_tensor out[2];
};

static struct null_model g_null;

static int null_init(void)
{
	return 0;
}

/* Descriptors only -- see the file header.  Idempotent: a second open just rewrites
 * the same values, so the singleton in nn.c can call it without extra state. */
static int null_open(void **impl_out)
{
	struct null_model *m = &g_null;

	m->in[0] = (struct nn_tensor){
		.data = null_in_buf, .bytes = NULL_IN_BYTES,
		.dims = { 1, NULL_IN_H, NULL_IN_W, NULL_IN_C }, .ndim = 4,
		.dtype = NN_DTYPE_INT8, .scale = 1.0f / 128.0f, .zero_point = 0,
	};
	m->out[0] = (struct nn_tensor){
		.data = null_box_buf, .bytes = NULL_BOX_BYTES,
		.dims = { 1, NULL_CELLS, NULL_BOX_LANES, 1 }, .ndim = 3,
		.dtype = NN_DTYPE_INT8, .scale = 1.0f / 256.0f, .zero_point = 0,
	};
	m->out[1] = (struct nn_tensor){
		.data = null_scr_buf, .bytes = NULL_SCR_BYTES,
		.dims = { 1, NULL_CELLS, 1, 1 }, .ndim = 3,
		.dtype = NN_DTYPE_FLOAT32, .scale = 0.0f, .zero_point = 0,
	};

	*impl_out = m;
	return 0;
}

static void null_close(void *impl)
{
	(void)impl;
}

static const char *null_model_name(void *impl)
{
	(void)impl;
	return "null-stub";
}

static int null_in_count(void *impl)
{
	(void)impl;
	return 1;
}

static int null_out_count(void *impl)
{
	(void)impl;
	return 2;
}

static struct nn_tensor *null_input(void *impl, int idx)
{
	struct null_model *m = (struct null_model *)impl;

	return (m && idx == 0) ? &m->in[0] : NULL;
}

static struct nn_tensor *null_output(void *impl, int idx)
{
	struct null_model *m = (struct null_model *)impl;

	return (m && idx >= 0 && idx < 2) ? &m->out[idx] : NULL;
}

/* A stub has no intermediate tensors, so there is no activation arena to report.
 * Reporting the input size instead (as the donor does) would put a number on the
 * `ai info` arena line that means nothing. */
static uint32_t null_activations_bytes(void *impl)
{
	(void)impl;
	return 0u;
}

/*
 * The synthetic workload: FNV-1a over the input, then both outputs written from it.
 * Reads 12,288 B and writes 5,120 B, which is comfortably above the noise floor of
 * the cycle counter and far below the wrap bound nn_run() documents.  It is also the
 * measurement that showed why phase 2a was worth doing: through the non-cacheable
 * window this cost ~9.8 core cycles per access, i.e. one bus transaction each, with
 * the streaming bandwidth of the device nowhere in sight.
 */
static int null_run(void *impl)
{
	struct null_model *m = (struct null_model *)impl;
	const uint8_t *in;
	int8_t *box;
	float *scr;
	uint32_t acc = 2166136261u;   /* FNV-1a offset basis */
	uint32_t i;

	if (!m)
		return -1;

	in  = (const uint8_t *)m->in[0].data;
	box = (int8_t *)m->out[0].data;
	scr = (float *)m->out[1].data;
	if (!in || !box || !scr)
		return -1;

	for (i = 0u; i < NULL_IN_BYTES; i++)
		acc = (acc ^ in[i]) * 16777619u;

	for (i = 0u; i < NULL_BOX_BYTES; i++)
		box[i] = (int8_t)(acc + i);

	for (i = 0u; i < NULL_SCR_COUNT; i++)
		scr[i] = (float)((acc + i) & 0xFFu) * (1.0f / 255.0f);

	null_sink = acc;
	return 0;
}

/*
 * load_region / reload / heap_allocs are left out on purpose, so they are NULL and
 * nn.c answers NN_ERR_NOSUP for them.  A stub could fake a model swap -- accept the
 * bytes, ignore them and report success -- and that would be the worst possible
 * behaviour: `ai model load` would appear to work, and the first wrong inference would
 * be blamed on the model rather than on the backend that never read it.  This backend
 * has no model to swap, and says so.
 */
const struct nn_backend_vt nn_backend_vt_selected = {
	.info = &(const struct nn_backend_info){ .name = "null", .version = "stub" },
	.init              = null_init,
	.open              = null_open,
	.close             = null_close,
	.model_name        = null_model_name,
	.in_count          = null_in_count,
	.out_count         = null_out_count,
	.input             = null_input,
	.output            = null_output,
	.activations_bytes = null_activations_bytes,
	.run               = null_run,
};
