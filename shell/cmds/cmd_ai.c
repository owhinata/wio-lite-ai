/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    cmd_ai.c
 * @brief   `ai` shell command: on-device NN inference (issue #9 phases 1 and 2c).
 *
 *   ai info              backend / model / tensor shapes / quantization / arena
 *   ai bench [n]         run inference n times, report min/avg/max latency
 *   ai out [i] [n]       dequantized values the last run left in output tensor i
 *   ai model load <slot> read a .tflite out of a NOR blob slot and run it
 *   ai model unload      drop it again
 *
 * Phase 1 shipped the backend-agnostic plumbing with the `null` stub behind it, so
 * both original subcommands worked end to end while doing no actual inference.
 * Phase 2c added the TFLM backend and, with it, the two `ai model` subcommands --
 * which the `null` build still accepts and answers honestly, because the "this
 * backend cannot load a model" case is a value nn.c returns rather than a missing
 * command (port/nn/nn.h).  The camera-driven `ai run` / `ai stream` are phase 3.
 * `ai out` is issue #57: once this build could load models other than BlazeFace
 * (issue #55), `ai dets` was the only way to read an output and it reads them as
 * BlazeFace, so any other model ran with its answer invisible.
 *
 * WHERE A MODEL COMES FROM IS DECIDED HERE, not in port/nn.  That layer is
 * model-agnostic on purpose; the fact that this board keeps models in the external
 * NOR's blob region (issue #10) is a property of the board, so the blob_* calls and
 * the CRC check live in this file and the nn API only ever sees "here are len bytes".
 *
 * LATENCY IS MEASURED WITH THE DWT CYCLE COUNTER, captured inside nn_run() (see
 * port/nn/nn.c) and converted here.  The divisor is SystemCoreClock -- 550 MHz, the
 * CPU clock CYCCNT actually counts -- and NOT HAL_RCC_GetHCLKFreq(), which on this
 * part returns the 275 MHz D2/AHB clock and would report every latency at twice its
 * true value while looking perfectly stable.  cmd_membench.c made the same call for
 * the same reason.
 *
 * Numbers reported here include any preemption that landed inside the run, which is
 * why `ai bench` reports min as well as avg: for a like-for-like figure, stop the
 * camera and the display first (`camera stream stop`, `lcd off`).
 *
 * Clean-room design; no third-party code reused.
 */
#include "cli.h"
#include "nn.h"
#include "blazeface.h"       /* issue #9 P3: model-specific decode, above the nn API */
#include "psram.h"

#if BSP_ENABLE_KV
#include "blob.h"            /* issue #10: the NOR asset slots a model lives in */
#include <flashdb.h>         /* fdb_calc_crc32 -- see the note at ai_model_load() */
#endif
#if BSP_ENABLE_LCD
#include "ltdc_display.h"    /* ltdc_errors: the underrun evidence around a bench */
#endif
#if BSP_ENABLE_CAMERA
#include "cam_band.h"        /* who is on the band stream (the preview-first note) */
#include "camera.h"          /* camera_band_streaming(): is `ai run` cold-starting? */
#include "nn_camera.h"       /* issue #9 P3: the camera-driven worker              */
#endif

#include "stm32h7xx_hal.h"   /* SystemCoreClock (the DWT/CPU clock) */

#include <stdint.h>
#include <string.h>

#define AI_BENCH_DEFAULT_ITERS 50u
#define AI_BENCH_MAX_ITERS     100000u

/*
 * `ai out` prints a PREFIX of a tensor, never all of it (issue #57).  16 covers every
 * classifier this board runs -- vww01 has 2 classes, ic0x 10, kws01 12 -- and the cap
 * exists because the other kind of output tensor is BlazeFace's 512x16, which at one
 * line each would be 8192 lines the console cannot be interrupted out of usefully
 * (cli_core drops a cancelled handler's output, issue #16, so a long dump is not
 * something the user can stop and still read).
 */
#define AI_OUT_DEFAULT_ELEMS   16u
#define AI_OUT_MAX_ELEMS       64u

static const char *ai_dtype_name(uint8_t dt)
{
	switch (dt) {
	case NN_DTYPE_INT8:    return "int8";
	case NN_DTYPE_UINT8:   return "uint8";
	case NN_DTYPE_INT16:   return "int16";
	case NN_DTYPE_INT32:   return "int32";
	case NN_DTYPE_FLOAT32: return "f32";
	default:               return "none";
	}
}

/* Print one tensor line: "  in[0]  1x64x64x3 int8  q(s=0.007812 zp=0)  12288B" */
static void ai_print_tensor(struct cli_instance *sh, const char *tag, int idx,
                            const struct nn_tensor *t)
{
	char shape[48];
	int pos = 0;
	int i;

	if (!t)
		return;

	/* "d0xd1xd2..." built by hand: svc/fmt has no width-limited numeric field and
	 * pulling newlib snprintf in for one label is not worth it. */
	for (i = 0; i < (int)t->ndim && i < NN_MAX_DIMS; i++) {
		unsigned v = t->dims[i];
		char tmp[8];
		int tl = 0;

		if (v == 0u)
			tmp[tl++] = '0';
		while (v) { tmp[tl++] = (char)('0' + v % 10u); v /= 10u; }
		if (i && pos < (int)sizeof shape - 1)
			shape[pos++] = 'x';
		while (tl && pos < (int)sizeof shape - 1)
			shape[pos++] = tmp[--tl];
	}
	shape[pos] = '\0';

	if (t->dtype == NN_DTYPE_INT8 || t->dtype == NN_DTYPE_UINT8) {
		/*
		 * Integer part and six fraction digits, printed separately: svc/fmt.c
		 * implements no %f (and no precision flag), so every fractional value in
		 * this firmware is printed as scaled integers -- the same idiom as the fps
		 * and MHz lines elsewhere.
		 *
		 * 🔴 NOT parts-per-million into a fixed "0.%06lu" (issue #51).  That form
		 * silently misprints any scale >= 1 -- 1.5 came out as "0.1500000" -- and
		 * output tensors routinely have one.  A quantization scale is the number
		 * you check when detections look wrong, so it has to survive being read.
		 */
		float s = t->scale;
		uint32_t s_int, s_frac;

		/* 🔴 Clamped BEFORE the cast, not after: float -> uint32_t is undefined for
		 * NaN and for anything outside the destination's range, and `ai info` is the
		 * command reached for when a model is already suspect -- it must not be the
		 * thing that then misbehaves. */
		if (!(s > 0.0f)) {
			/* Zero, negative, or NaN.  Zero is the one that actually happens: a
			 * per-axis quantized tensor reads back as scale 0 through this API
			 * (app/nn_camera.c refuses such an input for exactly that reason). */
			s_int = 0u;
			s_frac = 0u;
		} else if (s >= 1000000.0f) {
			s_int = 999999u;   /* saturated; no real quantization scale is here */
			s_frac = 999999u;
		} else {
			s_int = (uint32_t)s;
			s_frac = (uint32_t)((s - (float)s_int) * 1000000.0f + 0.5f);
			if (s_frac >= 1000000u) {   /* the rounding carried */
				s_int++;
				s_frac -= 1000000u;
			}
		}
		cli_print(sh, "  %s[%d]  %s %s  q(s=%lu.%06lu zp=%ld)  %luB\r\n",
		          tag, idx, shape, ai_dtype_name(t->dtype),
		          (unsigned long)s_int, (unsigned long)s_frac,
		          (long)t->zero_point, (unsigned long)t->bytes);
	} else {
		cli_print(sh, "  %s[%d]  %s %s  %luB\r\n",
		          tag, idx, shape, ai_dtype_name(t->dtype),
		          (unsigned long)t->bytes);
	}
}

/* Reads descriptors only, so it works with the PSRAM down -- see nn_backend.h. */
static int cmd_ai_info(struct cli_instance *sh, int argc, char **argv)
{
	const struct nn_backend_info *bi = nn_backend();
	struct nn_model *m = NULL;
	uint32_t cyc;
	int i, rc;

	(void)argc; (void)argv;

	cli_print(sh, "backend : %s (%s)\r\n",
	          bi->name, (bi->version && bi->version[0]) ? bi->version : "-");

	rc = nn_model_open(&m);
	if (rc != 0) {
		cli_error(sh, "ai: model open failed (%d)\r\n", rc);
		return 1;
	}

	cli_print(sh, "model   : %s\r\n", nn_model_name(m));
	cli_print(sh, "arena   : %lu B (activations)\r\n",
	          (unsigned long)nn_activations_bytes(m));

	for (i = 0; i < nn_input_count(m); i++)
		ai_print_tensor(sh, "in", i, nn_input(m, i));
	for (i = 0; i < nn_output_count(m); i++)
		ai_print_tensor(sh, "out", i, nn_output(m, i));

	cyc = nn_last_cycles(m);
	if (cyc)
		cli_print(sh, "last    : %lu cyc\r\n", (unsigned long)cyc);

	/*
	 * The heap-use counter, when the backend has a language runtime to count.  This
	 * is the one line on this page that is EVIDENCE rather than description: the TFLM
	 * backend is built so its interpreter allocates only from the arena, and a
	 * non-zero number here is the only way that assumption announces itself being
	 * wrong.  It would otherwise surface much later as an unexplained shortage of
	 * AXI-SRAM heap in `free`, with nothing pointing at inference.
	 */
	{
		uint32_t allocs;

		if (nn_heap_allocs(&allocs) == 0)
			cli_print(sh, "cxx new : %lu call(s)%s\r\n", (unsigned long)allocs,
			          allocs ? "  <-- UNEXPECTED: inference reached the heap" : "");
	}

	/* Say plainly that nothing is being inferred, so a latency from `ai bench` is
	 * never mistaken for a model's. */
	if (strcmp(bi->name, "null") == 0)
		cli_warn(sh, "note    : synthetic workload, not inference\r\n");
	else if (nn_input_count(m) == 0)
		cli_warn(sh, "note    : no model loaded -- `blob list`, then "
		             "`ai model load <slot>`\r\n");

	return 0;
}

/* ---- ai model load / unload (issue #9 phase 2c) --------------------------- */

/*
 * Take both guards, in this order, for anything that touches tensors or the arena.
 *
 * THE ORDER IS NOT ARBITRARY.  The NN session is the software claim and the OCTOSPI1
 * guard is the hardware one; taking the hardware first would mean holding a peripheral
 * for as long as it takes to tell the user that a piece of software was busy.  The
 * same order as `ai bench` below, and the same reasoning issue #9 phase 2a recorded.
 *
 * psram_acquire_shared() is the READ/WRITE guard, not a reconfiguration one, so it
 * coexists with an already-armed LTDC scan-out or camera stream.  It is still a single
 * lock: while it is held, a new `lcd on` or `camera capture` is refused.
 */
static int ai_session_take(struct cli_instance *sh)
{
	if (nn_session_try_acquire() != 0) {
		cli_error(sh, "ai: NN busy (another ai command is running)\r\n");
		return -1;
	}
	return 0;
}

static int ai_psram_take(struct cli_instance *sh)
{
	if (!psram_ready()) {
		cli_error(sh, "ai: PSRAM not ready -- the arena and the model live there "
		          "(see `psram info`)\r\n");
		return -1;
	}
	if (!psram_acquire_shared()) {
		cli_error(sh, "ai: OCTOSPI1 busy (a psram/membench/wifi flash command "
		          "holds it, or `ai stream` is running -- `ai stream stop`)\r\n");
		return -1;
	}
	return 0;
}

static int ai_guards_take(struct cli_instance *sh)
{
	if (ai_session_take(sh) != 0)
		return -1;
	if (ai_psram_take(sh) != 0) {
		nn_session_release();
		return -1;
	}
	return 0;
}

static void ai_guards_give(void)
{
	psram_release();
	nn_session_release();
}

#if BSP_ENABLE_KV
static int cmd_ai_model_load(struct cli_instance *sh, int argc, char **argv)
{
	struct blob_info info;
	struct nn_model *m = NULL;
	void     *stage = NULL;
	uint32_t  cap = 0u, slot, crc;
	int rc;

	(void)argc;

	if (cli_parse_u32(argv[1], &slot) != 0 || slot >= BLOB_SLOT_COUNT) {
		cli_error(sh, "ai: slot must be 0 .. %u (see `blob list`)\r\n",
		          (unsigned)BLOB_SLOT_COUNT - 1u);
		return 1;
	}

	rc = nn_model_open(&m);
	if (rc != 0) {
		cli_error(sh, "ai: model open failed (%d)\r\n", rc);
		return 1;
	}

	/*
	 * 🔴 THE SESSION IS TAKEN BEFORE load_region(), NOT AFTER.  Which staging slot is
	 * "the inactive one" is a function of backend state, so a slot number handed out
	 * before the claim can be stale by the time it is used: two consoles both ask,
	 * both are told slot 1, the first wins the session and makes slot 1 ACTIVE, and
	 * the second then writes its download straight over the flatbuffer the live
	 * interpreter is reading.  The double-buffering only means anything if the answer
	 * and the use are inside the same claim -- which is why port/nn/nn.h states the
	 * requirement rather than leaving it to be rediscovered here.
	 *
	 * The OCTOSPI1 guard is NOT taken yet: everything up to the read is either
	 * backend state or NOR traffic, and holding the PSRAM across a header decode buys
	 * nothing while refusing an `lcd on` for the duration.
	 */
	if (ai_session_take(sh) != 0)
		return 1;

	/* Ask the backend for a staging buffer before touching the NOR, so a backend that
	 * cannot swap models at run time (the `null` stub) costs no flash traffic to find
	 * out about.  It is also the only honest way to learn the capacity -- assuming the
	 * blob payload maximum instead would put the same constant in two places and let
	 * them drift apart. */
	rc = nn_model_load_region(&stage, &cap);
	if (rc != 0) {
		nn_session_release();
		cli_error(sh, "ai: %s\r\n", nn_model_strerror(rc));
		return 1;
	}

	/*
	 * Hold the blob mutation lock across the header decode AND the payload read, so
	 * the length and CRC we validate against belong to the same generation of the slot
	 * as the bytes we read.  app/blob.h says the read side needs no exclusion, and for
	 * a hexdump that is true -- but this is a read-decide-read sequence, and a
	 * `blob erase 0` from the other console landing between the two halves would have
	 * us reading an erased slot against a live header.
	 *
	 * The CRC check below would catch that anyway, which is the point of taking it
	 * over the copy in PSRAM.  This turns a mysterious "CRC32 mismatch" into an
	 * accurate "blob busy", and it costs a PRIMASK test-and-set.  blob_busy is not
	 * recursive, but neither blob_stat() nor blob_read() takes it, so this is safe.
	 */
	if (blob_busy_acquire() != BLOB_OK) {
		nn_session_release();
		cli_error(sh, "ai: blob busy (a blob write/erase is running)\r\n");
		return 1;
	}

	if (blob_stat(slot, &info) != BLOB_OK) {
		blob_busy_release();
		nn_session_release();
		cli_error(sh, "ai: cannot read slot %lu's header (see `nor info`)\r\n",
		          (unsigned long)slot);
		return 1;
	}
	if (info.state != BLOB_VALID) {
		blob_busy_release();
		nn_session_release();
		cli_error(sh, "ai: slot %lu holds no valid blob -- `blob list`\r\n",
		          (unsigned long)slot);
		return 1;
	}
	if (info.length == 0u || info.length > cap) {
		blob_busy_release();
		nn_session_release();
		cli_error(sh, "ai: slot %lu is %lu B, staging holds %lu B\r\n",
		          (unsigned long)slot, (unsigned long)info.length,
		          (unsigned long)cap);
		return 1;
	}

	/* From here on the PSRAM is written and then interpreted, so take the hardware
	 * guard too -- software claim first, as ai_guards_take() explains. */
	if (ai_psram_take(sh) != 0) {
		blob_busy_release();
		nn_session_release();
		return 1;
	}

	/* ~17 ms for 189 KB in one indirect read; no yield, by blob_read()'s contract.
	 * Every higher-priority thread -- the IWDG petter included -- still preempts. */
	rc = blob_read(slot, 0u, stage, info.length);
	blob_busy_release();          /* the NOR is done with; the rest is PSRAM only */
	if (rc != BLOB_OK) {
		ai_guards_give();
		cli_error(sh, "ai: NOR read failed (%d) -- the previous model is "
		          "untouched\r\n", rc);
		return 1;
	}

	/*
	 * 🔴 CRC THE COPY IN PSRAM, NOT THE FLASH.  `blob verify` re-reads the NOR and
	 * compares it against the stored value, which says nothing about the bytes that
	 * are about to be interpreted: a fault anywhere between the NOR and this buffer --
	 * the driver, the bus, the PSRAM itself -- would pass that check and still hand a
	 * corrupt flatbuffer to TFLM.  Checking what we are actually going to use is the
	 * only version of this check that means anything.
	 *
	 * fdb_calc_crc32() is used bare, chained from 0.  It inverts at entry and exit
	 * itself, so this IS standard CRC-32/ISO-HDLC and wrapping it would double-invert
	 * (issue #37; shell/test/test_crc32.c pins the property).
	 */
	crc = fdb_calc_crc32(0u, stage, info.length);
	if (crc != info.crc32) {
		ai_guards_give();
		cli_error(sh, "ai: CRC32 mismatch -- stored %08lX, in memory %08lX.  "
		          "The blob is intact on the NOR only if `blob verify %lu` "
		          "passes; the copy is not.\r\n",
		          (unsigned long)info.crc32, (unsigned long)crc,
		          (unsigned long)slot);
		return 1;
	}

	rc = nn_model_reload(stage, info.length, info.name);
	ai_guards_give();

	if (rc != 0) {
		/* Transactional by contract: whatever was loaded before is still loaded. */
		cli_error(sh, "ai: model refused (%d): %s\r\n", rc, nn_model_strerror(rc));
		cli_warn(sh, "ai: the previously loaded model is unchanged\r\n");
		return 1;
	}

	cli_print(sh, "loaded  : %s (slot %lu, %lu B, crc32 %08lX)\r\n",
	          nn_model_name(m), (unsigned long)slot,
	          (unsigned long)info.length, (unsigned long)crc);
	cli_print(sh, "arena   : %lu B used\r\n",
	          (unsigned long)nn_activations_bytes(m));
	{
		int i;

		for (i = 0; i < nn_input_count(m); i++)
			ai_print_tensor(sh, "in", i, nn_input(m, i));
		for (i = 0; i < nn_output_count(m); i++)
			ai_print_tensor(sh, "out", i, nn_output(m, i));
	}
	return 0;
}
#else /* !BSP_ENABLE_KV */
static int cmd_ai_model_load(struct cli_instance *sh, int argc, char **argv)
{
	(void)argc; (void)argv;
	cli_error(sh, "ai: built without the external NOR (BSP_ENABLE_KV=OFF), which is "
	          "where models are kept\r\n");
	return 1;
}
#endif

static int cmd_ai_model_unload(struct cli_instance *sh, int argc, char **argv)
{
	struct nn_model *m = NULL;
	int rc;

	(void)argc; (void)argv;

	rc = nn_model_open(&m);
	if (rc != 0) {
		cli_error(sh, "ai: model open failed (%d)\r\n", rc);
		return 1;
	}
	/* Guarded like a load: tearing the interpreter down walks the arena, which is in
	 * the PSRAM, and no other console may be mid-inference while it happens. */
	if (ai_guards_take(sh) != 0)
		return 1;
	rc = nn_model_reload(NULL, 0u, NULL);
	ai_guards_give();

	if (rc != 0) {
		cli_error(sh, "ai: %s\r\n", nn_model_strerror(rc));
		return 1;
	}
	cli_print(sh, "model   : %s\r\n", nn_model_name(m));
	return 0;
}

/* Bare `ai model` -- report, and show the two verbs.  Same shape as
 * `camera stream` with no argument (shell/cmds/cmd_camera.c). */
static int cmd_ai_model(struct cli_instance *sh, int argc, char **argv)
{
	struct nn_model *m = NULL;

	if (argc > 1) {
		cli_error(sh, "ai: unknown model subcommand '%s'\r\n", argv[1]);
		cli_print(sh, "usage: ai model <load <slot> | unload>\r\n");
		return 1;
	}
	if (nn_model_open(&m) == 0)
		cli_print(sh, "model   : %s\r\n", nn_model_name(m));
	cli_print(sh, "usage: ai model <load <slot> | unload>\r\n");
	return 0;
}

static int cmd_ai_bench(struct cli_instance *sh, int argc, char **argv)
{
	struct nn_model *m = NULL;
	uint32_t iters = AI_BENCH_DEFAULT_ITERS;
	uint32_t cmin = 0xFFFFFFFFu, cmax = 0u, done = 0u;
	uint64_t csum = 0u;
	uint32_t mhz = SystemCoreClock / 1000000u;
	uint32_t i;
	int rc;
#if BSP_ENABLE_LCD
	uint32_t ltdc_pre = 0u, ltdc_err = 0u;
#endif

	if (mhz == 0u)
		mhz = 1u;   /* cannot happen with an inherited 550 MHz; keeps the divide safe */

	if (argc > 1) {
		uint32_t v;

		if (cli_parse_u32(argv[1], &v) != 0 || v == 0u) {
			cli_error(sh, "ai: bad iteration count (1 .. %lu)\r\n",
			          (unsigned long)AI_BENCH_MAX_ITERS);
			return 1;
		}
		iters = (v > AI_BENCH_MAX_ITERS) ? AI_BENCH_MAX_ITERS : v;
	}

	rc = nn_model_open(&m);
	if (rc != 0) {
		cli_error(sh, "ai: model open failed (%d)\r\n", rc);
		return 1;
	}

	/* An interpreting backend opens with NO model (port/nn/tflm/nn_tflm.cc), and
	 * there is nothing to benchmark in that state.  Refusing explicitly matters: a run
	 * over zero tensors would report a latency, and a very small latency is exactly
	 * what a very fast model looks like. */
	if (nn_input_count(m) == 0) {
		cli_error(sh, "ai: no model loaded -- `blob list`, then "
		          "`ai model load <slot>`\r\n");
		return 1;
	}

	/* Unlike `ai info`, benchmarking touches the tensor bodies and the arena, and
	 * those live in the external PSRAM.  Both guards, in the documented order. */
	if (ai_guards_take(sh) != 0)
		return 1;

#if BSP_ENABLE_LCD
	/*
	 * Bracket the measured window with the sticky LTDC error flags, so a run made
	 * while `lcd on` and `camera stream` are up reports the underruns IT caused.
	 *
	 * Reading them here instead of leaving the operator to type `lcd info` afterwards
	 * is the whole point.  The flags are sticky and global, so anything that ran
	 * between the benchmark and a later query is indistinguishable from the benchmark;
	 * only a pair bracketing exactly the measured window attributes an underrun to
	 * inference.  Issue #9 phase 2a checked this co-residency with the synthetic stub,
	 * whose memory traffic is nothing like a real model's, so the question is open
	 * again here.
	 *
	 * Clearing is what makes the attribution work -- a bit that was already set would
	 * otherwise mask an underrun this run caused -- and it is also the one destructive
	 * thing on this page: `lcd info` reads those flags WITHOUT clearing, so they are
	 * the accumulated history since boot and this throws it away.
	 *
	 * So what is being discarded is reported HERE, before the loop, and not with the
	 * results at the end.  A Ctrl+C would never reach the end -- and once cancel
	 * latches, cli_core drops every byte a dispatching handler writes (issue #16), so a
	 * report deferred to the end is not merely late on that path, it is unreachable.
	 * The history would be gone with nothing said about it.
	 */
	ltdc_pre = ltdc_errors(true);
	if (ltdc_pre && ltdc_scanout_active())
		cli_warn(sh, "ltdc    : flags were already set before this run (underrun %s, "
		         "transfer-error %s) -- cleared to measure\r\n",
		         (ltdc_pre & LTDC_ERRFLAG_FIFO_UNDERRUN) ? "yes" : "no",
		         (ltdc_pre & LTDC_ERRFLAG_TRANSFER_ERROR) ? "yes" : "no");
#endif

	/* The carve-out is NOLOAD, so the input holds whatever survived the last reset
	 * until something fills it.  A fixed pattern makes every run comparable. */
	for (i = 0; i < (uint32_t)nn_input_count(m); i++) {
		struct nn_tensor *t = nn_input(m, (int)i);

		if (t && t->data)
			memset(t->data, 0x5A, t->bytes);
	}

	cli_print(sh, "bench %s x%lu ...\r\n", nn_model_name(m), (unsigned long)iters);

	/*
	 * Ctrl+C reports nothing, deliberately.  Once cancel latches, this core drops
	 * every further byte a dispatching handler writes (cli_core.c, issue #16) so a
	 * chatty handler cannot outlive the keystroke -- the "^C" and the prompt that do
	 * appear come from the core's own post-dispatch cleanup.  So the donor
	 * firmware's "^C after N" plus partial statistics could not reach the terminal
	 * here however it were written, and a half-finished benchmark is not a result
	 * worth reporting anyway.  Cancel just unwinds, releasing both guards, which is
	 * the same shape `membench` uses.
	 */
	for (i = 0; i < iters; i++) {
		uint32_t c;

		if (cli_cancel_requested(sh))
			break;
		rc = nn_run(m);
		if (rc != 0) {
			ai_guards_give();
			cli_error(sh, "ai: nn_run failed (%d) at %lu\r\n", rc,
			          (unsigned long)i);
			return 1;
		}
		c = nn_last_cycles(m);
		if (c < cmin) cmin = c;
		if (c > cmax) cmax = c;
		csum += c;
		done++;
	}

#if BSP_ENABLE_LCD
	/* Sampled INSIDE the guards, before anything else can run and add to the sticky
	 * flags -- see the note where they were cleared. */
	ltdc_err = ltdc_errors(false);
#endif
	ai_guards_give();

	/* Only a cancel can leave this short, and a cancelled run prints nothing. */
	if (done < iters)
		return 0;

	{
		uint32_t cavg = (uint32_t)(csum / done);

		cli_print(sh, "cycles  min %lu  avg %lu  max %lu  (%lu runs)\r\n",
		          (unsigned long)cmin, (unsigned long)cavg,
		          (unsigned long)cmax, (unsigned long)done);
		cli_print(sh, "latency min %lu  avg %lu  max %lu us  (@%lu MHz)\r\n",
		          (unsigned long)(cmin / mhz), (unsigned long)(cavg / mhz),
		          (unsigned long)(cmax / mhz), (unsigned long)mhz);
		if (cmin == 0u)
			cli_warn(sh, "note: 0-cycle runs -- the DWT cycle counter is not "
			             "advancing\r\n");
	}
#if BSP_ENABLE_LCD
	/* Printed only when the display was actually scanning out: with the LCD off the
	 * flags cannot be set, and a line reading "underrun no" would be evidence of
	 * nothing while looking like evidence of something.  The pre-existing state was
	 * already reported before the loop -- see the note where the flags were cleared. */
	if (ltdc_scanout_active())
		cli_print(sh, "ltdc    : underrun %s  transfer-error %s  (during this run)\r\n",
		          (ltdc_err & LTDC_ERRFLAG_FIFO_UNDERRUN) ? "YES" : "no",
		          (ltdc_err & LTDC_ERRFLAG_TRANSFER_ERROR) ? "YES" : "no");
#endif
	return 0;
}

/* ---- ai out (issue #57) --------------------------------------------------- */

/*
 * Decompose @p v for printing as "%s%lu.%06lu" -- sign, integer part, six fraction
 * digits.  svc/fmt.c implements no %f and no precision flag, so every fractional value
 * in this firmware is printed as scaled integers; ai_print_tensor() above does the same
 * thing inline for the quantization scale.  It is a separate function here because a
 * dequantized OUTPUT can be negative and a scale cannot, and the sign is not decoration:
 * for a logit it is the whole answer.
 *
 * Returns -1 for NaN, which the caller prints as "nan" rather than as some number.  A
 * NaN in an output tensor is a diagnosis, not a value, and printing "0.000000" for it
 * would hide the one thing worth seeing.
 */
static int ai_f32_parts(float v, const char **sign, uint32_t *ip, uint32_t *frac)
{
	*sign = "";
	if (v != v)                     /* the only value that is not equal to itself */
		return -1;
	if (v < 0.0f) {
		*sign = "-";
		v = -v;
	}
	/* 🔴 Clamped BEFORE the cast, not after: float -> uint32_t is undefined for
	 * anything outside the destination's range, and this command is reached for when
	 * a model is already suspect -- it must not be the thing that then misbehaves.
	 * Same reasoning, same clamp, as the scale in ai_print_tensor(). */
	if (v >= 1000000.0f) {
		*ip = 999999u;
		*frac = 999999u;
		return 0;
	}
	*ip = (uint32_t)v;
	*frac = (uint32_t)((v - (float)*ip) * 1000000.0f + 0.5f);
	if (*frac >= 1000000u) {        /* the rounding carried */
		(*ip)++;
		*frac -= 1000000u;
	}
	return 0;
}

static uint32_t ai_dtype_size(uint8_t dt)
{
	switch (dt) {
	case NN_DTYPE_INT8:
	case NN_DTYPE_UINT8:   return 1u;
	case NN_DTYPE_INT16:   return 2u;
	case NN_DTYPE_INT32:
	case NN_DTYPE_FLOAT32: return 4u;
	default:               return 0u;
	}
}

/* The stored code of element @p i, sign-extended into an int32.  Integer dtypes only --
   the caller has already separated the float case, which has no code to show. */
static int32_t ai_tensor_raw(const struct nn_tensor *t, uint32_t i)
{
	switch (t->dtype) {
	case NN_DTYPE_INT8:  return ((const int8_t  *)t->data)[i];
	case NN_DTYPE_UINT8: return ((const uint8_t *)t->data)[i];
	case NN_DTYPE_INT16: return ((const int16_t *)t->data)[i];
	case NN_DTYPE_INT32: return ((const int32_t *)t->data)[i];
	default:             return 0;   /* unreachable: ai_dtype_size() filtered these */
	}
}

/*
 * Print the values in an output tensor.
 *
 * WHY THIS IS NOT ANOTHER DECODER.  `ai dets` reads the outputs too, but it reads them
 * AS BLAZEFACE: four tensors in a known order, turned into boxes.  Every other model has
 * no decoder here and therefore no way to be read at all, which is what issue #57 hit --
 * vww01 (MLPerf Tiny's MobileNet, issue #55) already runs on camera frames without a
 * line of change, because app/nn_camera.c takes its geometry and its quantization from
 * the input tensor rather than assuming BlazeFace's; its 1x2 answer was simply invisible.
 * Dequantized numbers are model-agnostic, and that is the whole point: a command that
 * knows nothing about the model is the one that can still tell you the model works.
 *
 * IT RUNS NOTHING.  The values are whatever the last inference left in the arena -- the
 * same contract `ai dets` documents, and it pairs with the same two idioms: `ai bench 1`
 * then `ai out` reads a run over the constant pattern, and `ai run` then `ai out` reads
 * one camera frame.
 *
 * 🔴 A ZERO SCALE IS NOT A UNIT SCALE.  A per-axis quantized tensor arrives through this
 * API with params.scale == 0 because its real parameters live in the affine-quantization
 * struct port/nn/nn.h does not expose (issue #51, the same fact app/nn_camera.c refuses
 * an input over).  Dequantizing with it would print 0.000000 for every element of a
 * perfectly good tensor, so the codes are printed alone and the header says why.
 */
static int cmd_ai_out(struct cli_instance *sh, int argc, char **argv)
{
	struct nn_model *m = NULL;
	const struct nn_tensor *t;
	uint32_t idx = 0u, want = AI_OUT_DEFAULT_ELEMS, elems, esz, i;
	int nout, rc, quantized, dequant;

#if BSP_ENABLE_CAMERA
	/*
	 * The worker holds the NN session for the whole stream, so the guards below would
	 * refuse anyway -- but they would refuse with "NN busy", and the useful answer is
	 * which command to run instead.  Unlike `ai dets` there is no snapshot to print:
	 * the worker publishes decoded detections, not raw tensors, and publishing them
	 * would mean copying an output tensor of unknown size out of the arena on every
	 * inference for a command that is usually not watching.
	 */
	if (nn_camera_running()) {
		cli_error(sh, "ai: a stream is running -- the worker owns the arena. "
		          "`ai stream stop`, then `ai run` + `ai out`\r\n");
		return 1;
	}
#endif

	rc = nn_model_open(&m);
	if (rc != 0) {
		cli_error(sh, "ai: model open failed (%d)\r\n", rc);
		return 1;
	}
	nout = nn_output_count(m);
	if (nout <= 0) {
		cli_error(sh, "ai: no model loaded -- `blob list`, then "
		          "`ai model load <slot>`\r\n");
		return 1;
	}

	if (argc > 1) {
		uint32_t v;

		if (cli_parse_u32(argv[1], &v) != 0 || v >= (uint32_t)nout) {
			cli_error(sh, "ai: bad output index (0 .. %d)\r\n", nout - 1);
			return 1;
		}
		idx = v;
	}
	if (argc > 2) {
		uint32_t v;

		if (cli_parse_u32(argv[2], &v) != 0 || v == 0u) {
			cli_error(sh, "ai: bad element count (1 .. %lu)\r\n",
			          (unsigned long)AI_OUT_MAX_ELEMS);
			return 1;
		}
		want = (v > AI_OUT_MAX_ELEMS) ? AI_OUT_MAX_ELEMS : v;
	}

	t = nn_output(m, (int)idx);
	esz = t ? ai_dtype_size(t->dtype) : 0u;
	if (t == NULL || t->data == NULL || esz == 0u) {
		cli_error(sh, "ai: output[%lu] has no readable buffer (dtype %s)\r\n",
		          (unsigned long)idx, t ? ai_dtype_name(t->dtype) : "none");
		return 1;
	}

	/* Element count from the SHAPE, then clamped by the buffer.  The two should agree;
	   taking the smaller means a descriptor that disagrees with itself costs a short
	   listing rather than a read past the end of the arena. */
	elems = 1u;
	for (i = 0; i < t->ndim && i < NN_MAX_DIMS; i++)
		elems *= t->dims[i];
	if (elems > t->bytes / esz)
		elems = t->bytes / esz;
	if (elems == 0u) {
		cli_error(sh, "ai: output[%lu] is empty\r\n", (unsigned long)idx);
		return 1;
	}
	if (want > elems)
		want = elems;

	quantized = (t->dtype != NN_DTYPE_FLOAT32);
	dequant   = !quantized || (t->scale > 0.0f);

	/* Reading the tensor bodies walks the arena, which lives in the PSRAM -- the same
	   reason `ai bench` and `ai dets` are guarded and `ai info` is not. */
	if (ai_guards_take(sh) != 0)
		return 1;

	ai_print_tensor(sh, "out", (int)idx, t);
	if (!dequant)
		cli_warn(sh, "  scale unavailable (per-axis quantization) -- raw codes "
		         "only\r\n");
	for (i = 0; i < want; i++) {
		const char *sign;
		uint32_t ip, frac;
		int32_t raw = 0;
		float v = 0.0f;

		if (quantized) {
			raw = ai_tensor_raw(t, i);
			v = ((float)raw - (float)t->zero_point) * t->scale;
		} else {
			v = ((const float *)t->data)[i];
		}

		if (!dequant)
			cli_print(sh, "  [%lu]  q=%ld\r\n",
			          (unsigned long)i, (long)raw);
		else if (ai_f32_parts(v, &sign, &ip, &frac) != 0)
			/* An integer element cannot itself be NaN, so for a quantized tensor
			   the broken number is the scale -- name it, rather than leaving the
			   element looking like the problem. */
			cli_print(sh, "  [%lu]  nan%s\r\n", (unsigned long)i,
			          quantized ? "  (the SCALE is NaN, not the element)" : "");
		else if (quantized)
			cli_print(sh, "  [%lu]  %s%lu.%06lu  (q=%ld)\r\n", (unsigned long)i,
			          sign, (unsigned long)ip, (unsigned long)frac, (long)raw);
		else
			cli_print(sh, "  [%lu]  %s%lu.%06lu\r\n", (unsigned long)i,
			          sign, (unsigned long)ip, (unsigned long)frac);
	}
	ai_guards_give();

	if (want < elems)
		cli_print(sh, "  ... %lu of %lu shown (`ai out %lu <n>`, max %lu)\r\n",
		          (unsigned long)want, (unsigned long)elems, (unsigned long)idx,
		          (unsigned long)AI_OUT_MAX_ELEMS);
	return 0;
}

/* ---- ai dets / ai thresh (issue #9 phase 3) ------------------------------- */

/*
 * Print a detection list.  Units are spelled out on the header line because they
 * differ on purpose: the geometry is a percentage of the frame (which is how a box
 * is read off the screen), while the score is a milli-probability so it is on the
 * SAME scale as `ai thresh` -- a box is shown exactly when its printed score clears
 * the printed threshold.  svc/fmt.c implements no %f, so both are scaled integers,
 * the same idiom as the quantization scale in ai_print_tensor() above.
 */
static void ai_print_dets(struct cli_instance *sh, const struct bf_det *d, int n)
{
	int i;

	cli_print(sh, "dets    : %d  (x/y/w/h in %% of frame, score in milli; "
	          "thresh %u)\r\n", n, blazeface_get_thresh_milli());
	for (i = 0; i < n; i++)
		cli_print(sh, "  face[%d]  x %ld%% y %ld%% w %ld%% h %ld%%  score %ld\r\n",
		          i, (long)(d[i].x * 100.0f), (long)(d[i].y * 100.0f),
		          (long)(d[i].w * 100.0f), (long)(d[i].h * 100.0f),
		          (long)(d[i].score * 1000.0f));
}

/*
 * Print what the worker's decoder made of the last inference: the detections, or -- for
 * a model the decoder does not recognise at all -- that fact and where to look instead.
 *
 * 🔴 THE TWO CASES MUST NOT PRINT THE SAME THING (issue #57).  A non-BlazeFace model
 * used to arrive here as `dets: 0`, which reads as "no faces in the frame", beside a
 * `maxscore` that blazeface_decode() had returned too early to touch -- so the number
 * still belonged to whichever model ran before.  Both lines look like a measurement of
 * the model that is loaded, and neither is one.  This is the shape of stale diagnostic
 * this file has been bitten by before (the `re-issue ai stream start` hint that could
 * not actually re-arm): a confident wrong answer costs more than no answer.
 */
static void ai_print_decode(struct cli_instance *sh, const struct bf_det *d, int n,
                            const char *suffix)
{
	if (n < 0) {
		cli_print(sh, "dets    : n/a -- this model's outputs are not "
		          "BlazeFace-shaped; read them with `ai out`%s\r\n", suffix);
		return;
	}
	ai_print_dets(sh, d, n);
	cli_print(sh, "maxscore: %ld (raw x1000, pre-sigmoid)  cand: %d (pre-NMS)%s\r\n",
	          (long)(blazeface_last_max_score() * 1000.0f),
	          blazeface_last_ncand(), suffix);
}

/*
 * Decode whatever is currently in the model's output tensors.
 *
 * With no camera in the picture this is exactly what proves the decoder against the
 * board rather than the host: run `ai bench 1` (which fills the input with a constant
 * pattern) and then `ai dets`, and the two diagnostics below say whether the model
 * responded at all.  That separation is the point -- `dets: 0` alone cannot tell a
 * dead model from a threshold set too high, so maxscore and cand are printed beside
 * it.  Returns -1 for a model whose outputs are not BlazeFace-shaped, which is what
 * the CONFIG_NN_BACKEND=null build gets, and it says so rather than printing zeros.
 */
static int cmd_ai_dets(struct cli_instance *sh, int argc, char **argv)
{
	struct bf_det dets[BF_MAX_DET];
	struct nn_model *m = NULL;
	int n, rc;

	(void)argc; (void)argv;

#if BSP_ENABLE_CAMERA
	/* While a stream runs, the worker holds the NN session, so decoding here would
	   simply be refused -- and would be the wrong answer anyway.  Report the boxes
	   the worker published instead. */
	if (nn_camera_running()) {
		n = nn_camera_dets_get(dets, BF_MAX_DET);
		ai_print_decode(sh, dets, n, "  [live stream]");
		return 0;
	}
#endif

	rc = nn_model_open(&m);
	if (rc != 0) {
		cli_error(sh, "ai: model open failed (%d)\r\n", rc);
		return 1;
	}
	/* Reading the output tensors walks the arena, which lives in the PSRAM -- the
	 * same reason `ai bench` is guarded and `ai info` is not. */
	if (ai_guards_take(sh) != 0)
		return 1;
	n = blazeface_decode(m, dets, BF_MAX_DET);
	ai_guards_give();

	if (n < 0) {
		cli_error(sh, "ai: the loaded model's outputs are not BlazeFace-shaped "
		          "(need 1x512x16 / 1x512x1 / 1x384x16 / 1x384x1 float32 -- "
		          "see `ai info`).  `ai out` reads any model's outputs\r\n");
		return 1;
	}
	ai_print_decode(sh, dets, n, "");
	return 0;
}

static int cmd_ai_thresh(struct cli_instance *sh, int argc, char **argv)
{
	if (argc > 1) {
		uint32_t v;

		if (cli_parse_u32(argv[1], &v) != 0 || v < 1u || v > 999u) {
			cli_error(sh, "ai: thresh must be 1 .. 999 (milli-probability)\r\n");
			return 1;
		}
		blazeface_set_thresh_milli((unsigned)v);
	}
	/* The logit is what is actually compared against the raw tensor, so print it
	 * too: it is the number that appears next to a score in any model discussion,
	 * and having only the friendly unit on screen hides the conversion. */
	cli_print(sh, "thresh  : %u milli-probability  (raw logit x1000 = %ld)\r\n",
	          blazeface_get_thresh_milli(),
	          (long)(blazeface_get_thresh_logit() * 1000.0f));
	return 0;
}

/* ---- ai run / ai stream / ai norm / ai overlay (issue #9 phase 3) --------- */

#if BSP_ENABLE_CAMERA

/*
 * How long `ai run` lets the sensor settle when it started the stream itself.
 *
 * 🔴 NOT CAMERA_WARM_FRAMES.  That constant belongs to camera_capture_locked() and
 * neither stream path uses it (port/camera/camera.c says so explicitly) -- a band
 * stream starts delivering immediately, with the AEC/AWB still converging, so the
 * first frames are badly exposed.  ~8 frame periods at ~13.5 fps is enough for them
 * to settle, and saying so on the console is what keeps it from looking hung.
 */
#define AI_RUN_WARM_MS    600u
#define AI_RUN_TIMEOUT_MS 3000u

static const char *ai_nncam_strerror(int rc)
{
	switch (rc) {
	case NNCAM_ERR_RUNNING: return "a stream is already running (`ai stream stats`)";
	case NNCAM_ERR_NOTRUN:  return "not running";
	case NNCAM_ERR_MODEL:   return "no model loaded, or it has no usable input "
	                               "tensor (`blob list`, then `ai model load <slot>`)";
	case NNCAM_ERR_SESSION: return "the NN session is busy (`ai bench` or "
	                               "`ai model load` is running)";
	case NNCAM_ERR_PSRAM:   return "PSRAM not ready, or OCTOSPI1 is held by a "
	                               "psram/membench/devmem/wifi flash command";
	case NNCAM_ERR_BAND:    return "the camera would not start a band stream -- a "
	                               "frame stream may own the DCMI "
	                               "(`camera stream stop`), the other console may be "
	                               "starting or stopping it, or see `dmesg`";
	case NNCAM_ERR_GEOM:    return "the model input does not tile onto the camera's "
	                               "4 bands, or its dtype is neither int8 nor "
	                               "float32 (`ai info`)";
	case NNCAM_ERR_QUANT:   return "the int8 input carries no per-tensor quantization "
	                               "scale (`ai info` shows q(s=0.000000)) -- a "
	                               "per-axis quantized input is not supported";
	case NNCAM_ERR_INIT:    return "the worker thread or its objects could not be "
	                               "created";
	case NNCAM_ERR_TEARING: return "still tearing down (a callback or an inference "
	                               "has not returned) -- run `ai stream stop` again";
	case NNCAM_ERR_REARM:   return "the stream could not be re-armed (the DCMI may "
	                               "be owned elsewhere) -- run `ai stream stop`, "
	                               "then `ai stream start`";
	default:                return "unknown error";
	}
}

static uint32_t ai_cyc_to_us(uint32_t cyc)
{
	uint32_t mhz = SystemCoreClock / 1000000u;

	return mhz ? (cyc / mhz) : 0u;
}

/*
 * The guard is held for the stream's whole lifetime (see app/nn_camera.h), so
 * `camera preview on` is refused while it runs.  Anyone who wanted to watch the
 * boxes has to start the preview first, and finding that out by being refused is
 * worse than being told.
 */
static void ai_stream_order_note(struct cli_instance *sh)
{
#if BSP_ENABLE_LCD
	if (!cam_band_claimed(CAM_BAND_PREVIEW))
		cli_print(sh, "note    : the OCTOSPI1 guard is held for the stream's "
		          "lifetime, so `camera preview on` is refused until "
		          "`ai stream stop` -- start the preview FIRST if you want to "
		          "see the boxes\r\n");
#else
	(void)sh;
#endif
}

static int cmd_ai_stream_start(struct cli_instance *sh, int argc, char **argv)
{
	int colorbar = 0, rc, rearm;

	if (argc > 1) {
		if (strcmp(argv[1], "test") != 0) {
			cli_error(sh, "ai: usage: ai stream start [test]\r\n");
			return 1;
		}
		colorbar = 1;
	}
	/* Sampled before the call, because a successful re-arm clears the latch. */
	rearm = nn_camera_running() && cam_band_stream_lost();

	rc = nn_camera_start(colorbar);
	if (rc != NNCAM_OK) {
		cli_error(sh, "ai: %s\r\n", ai_nncam_strerror(rc));
		return 1;
	}
	if (rearm) {
		/* Say which of the two happened.  "started" over a stream that was only
		   re-armed would hide that an outage occurred at all -- and the counters
		   deliberately keep running across it, so they would not show it either. */
		cli_print(sh, "ai: stream re-armed after a lost stream "
		          "(counters continue; `ai stream stop` to reset them)\r\n");
		return 0;
	}
	cli_print(sh, "ai: inference stream started (worker prio 18%s)\r\n",
	          colorbar ? ", colorbar" : "");
	ai_stream_order_note(sh);
	return 0;
}

static int cmd_ai_stream_stop(struct cli_instance *sh, int argc, char **argv)
{
	int rc;

	(void)argc; (void)argv;

	rc = nn_camera_stop();
	if (rc == NNCAM_ERR_NOTRUN) {
		cli_warn(sh, "ai: not running\r\n");
		return 0;
	}
	if (rc != NNCAM_OK) {
		cli_error(sh, "ai: %s\r\n", ai_nncam_strerror(rc));
		return 1;
	}
	cli_print(sh, "ai: stopped\r\n");
	return 0;
}

/*
 * Deliberately longer than a progress display needs.  This phase deleted the donor's
 * staging machinery on the argument that the 373 ms : 74 ms ratio makes it
 * unnecessary, and these counters are how the board says whether that actually held.
 * `ingest max` against the ~18.5 ms band deadline and `stream` are the two that
 * would otherwise fail silently.
 */
static int cmd_ai_stream_stats(struct cli_instance *sh, int argc, char **argv)
{
	struct nn_camera_stats st;
	struct bf_det dets[BF_MAX_DET];
	uint32_t fps_x100 = 0u;
	int n;

	(void)argc; (void)argv;

	nn_camera_stats_get(&st);
	if (st.elapsed_ms)
		fps_x100 = (uint32_t)((uint64_t)st.infers * 100000u / st.elapsed_ms);

	cli_print(sh, "state   : %s\r\n", st.running ? "running" : "stopped");
	cli_print(sh, "session : %s\r\n",
	          st.holds_guards ? "held (NN + OCTOSPI1)" : "free");
	cli_print(sh, "stream  : %s\r\n",
	          st.stream_lost ? "LOST -- re-issue `ai stream start` to re-arm"
	                         : (st.running ? "ok" : "-"));
	cli_print(sh, "infers  : %lu in %lu ms  (%lu.%02lu inf/s)\r\n",
	          (unsigned long)st.infers, (unsigned long)st.elapsed_ms,
	          (unsigned long)(fps_x100 / 100u), (unsigned long)(fps_x100 % 100u));
	cli_print(sh, "frames  : %lu ingested, %lu skipped (worker busy), %lu error(s)\r\n",
	          (unsigned long)st.frames, (unsigned long)st.skipped,
	          (unsigned long)st.errors);
	/* The ownership invariant, reported rather than assumed (#54).  `raced` must be 0;
	 * anything else means part of the tensor the model saw was activations, not
	 * camera.  `stale` counts the frame posts the pre-arm drain discarded -- each one
	 * is a race that would have started and then never stopped. */
	cli_print(sh, "tensor  : %lu raced (must be 0), %lu stale post(s) dropped\r\n",
	          (unsigned long)st.raced, (unsigned long)st.stale_posts);
	cli_print(sh, "ingest  : last %lu us  max %lu us  (band deadline ~18500 us)\r\n",
	          (unsigned long)ai_cyc_to_us(st.ingest_last_cyc),
	          (unsigned long)ai_cyc_to_us(st.ingest_max_cyc));
	cli_print(sh, "latency : %lu us  (%lu cycles)\r\n",
	          (unsigned long)ai_cyc_to_us(st.infer_last_cyc),
	          (unsigned long)st.infer_last_cyc);
	cli_print(sh, "norm    : %s   overlay: %s\r\n",
	          st.norm_signed ? "[-1,1]" : "[0,1]", st.overlay ? "on" : "off");

	n = nn_camera_dets_get(dets, BF_MAX_DET);
	ai_print_decode(sh, dets, n, "");

	return 0;
}

/* Bare `ai stream` -- report and show the verbs, like `ai model`. */
static int cmd_ai_stream(struct cli_instance *sh, int argc, char **argv)
{
	if (argc > 1) {
		cli_error(sh, "ai: unknown stream subcommand '%s'\r\n", argv[1]);
		cli_print(sh, "usage: ai stream <start [test] | stop | stats>\r\n");
		return 1;
	}
	cli_print(sh, "stream  : %s\r\n", nn_camera_running() ? "running" : "stopped");
	cli_print(sh, "usage: ai stream <start [test] | stop | stats>\r\n");
	return 0;
}

static int cmd_ai_run(struct cli_instance *sh, int argc, char **argv)
{
	struct nn_camera_stats st;
	struct bf_det dets[BF_MAX_DET];
	uint32_t base, waited;
	int cold, rc, n;

	(void)argc; (void)argv;

	if (nn_camera_running()) {
		cli_error(sh, "ai: a stream is already running -- use `ai stream stats` "
		          "or `ai dets`\r\n");
		return 1;
	}

	/* Whether WE start the stream decides whether the sensor needs settling time. */
	cold = !camera_band_streaming();

	rc = nn_camera_start(0);
	if (rc != NNCAM_OK) {
		cli_error(sh, "ai: %s\r\n", ai_nncam_strerror(rc));
		return 1;
	}

	if (cold) {
		cli_print(sh, "ai: starting the camera, %lu ms for the exposure to "
		          "settle...\r\n", (unsigned long)AI_RUN_WARM_MS);
		if (cli_sleep(sh, AI_RUN_WARM_MS) != 0)
			goto cancelled;
	}

	/* Count from HERE, so the reported detection comes from a settled frame rather
	   than from whichever inference happened to finish during the warm-up. */
	nn_camera_stats_get(&st);
	base = st.infers;

	for (waited = 0u; waited < AI_RUN_TIMEOUT_MS; waited += 10u) {
		nn_camera_stats_get(&st);
		if (st.infers > base || st.stream_lost)
			break;
		if (cli_sleep(sh, 10u) != 0)
			goto cancelled;
	}

	n = nn_camera_dets_get(dets, BF_MAX_DET);
	nn_camera_stats_get(&st);
	(void)nn_camera_stop();

	if (st.stream_lost) {
		cli_error(sh, "ai: the band stream died during the run (DCMI overrun?) -- "
		          "see `camera stream stats`\r\n");
		return 1;
	}
	if (st.infers <= base) {
		cli_warn(sh, "ai: no inference completed in %lu ms\r\n",
		         (unsigned long)AI_RUN_TIMEOUT_MS);
		return 0;
	}
	cli_print(sh, "inference: %lu us\r\n",
	          (unsigned long)ai_cyc_to_us(st.infer_last_cyc));
	ai_print_decode(sh, dets, n, "");
	return 0;

cancelled:
	/* Ctrl+C: cli_core drops every byte a dispatching handler writes once cancel
	   latches (issue #16), so there is nothing to report -- just clean up. */
	(void)nn_camera_stop();
	return 1;
}

static int cmd_ai_norm(struct cli_instance *sh, int argc, char **argv)
{
	if (argc > 1) {
		if (strcmp(argv[1], "0") == 0)
			nn_camera_set_norm(0);
		else if (strcmp(argv[1], "1") == 0)
			nn_camera_set_norm(1);
		else {
			cli_error(sh, "ai: usage: ai norm <0|1>  (0=[0,1], 1=[-1,1])\r\n");
			return 1;
		}
	}
	cli_print(sh, "norm    : %s\r\n",
	          nn_camera_get_norm() ? "[-1,1] (signed)" : "[0,1] (unsigned)");
	return 0;
}

#if BSP_ENABLE_LCD
static int cmd_ai_overlay(struct cli_instance *sh, int argc, char **argv)
{
	if (argc > 1) {
		if (strcmp(argv[1], "on") == 0)
			nn_camera_set_overlay(1);
		else if (strcmp(argv[1], "off") == 0)
			nn_camera_set_overlay(0);
		else {
			cli_error(sh, "ai: usage: ai overlay <on|off>\r\n");
			return 1;
		}
	}
	cli_print(sh, "overlay : %s\r\n", nn_camera_get_overlay() ? "on" : "off");
	return 0;
}
#endif /* BSP_ENABLE_LCD */

CLI_SUBCMD_SET_CREATE(ai_stream_subcmds,
	CLI_CMD_ARG(start, NULL, "claim the band stream and infer continuously [test]",
	            cmd_ai_stream_start, 1, 1),
	CLI_CMD(stop,  NULL, "stop inferring and release the stream", cmd_ai_stream_stop),
	CLI_CMD(stats, NULL, "fps / ingest cost / stream health / detections",
	        cmd_ai_stream_stats),
	CLI_SUBCMD_SET_END);

#endif /* BSP_ENABLE_CAMERA */

CLI_SUBCMD_SET_CREATE(ai_model_subcmds,
	CLI_CMD_ARG(load,   NULL, "read a .tflite from NOR blob <slot> and run it",
	            cmd_ai_model_load,   2, 0),
	CLI_CMD(unload, NULL, "drop the loaded model", cmd_ai_model_unload),
	CLI_SUBCMD_SET_END);

CLI_SUBCMD_SET_CREATE(ai_subcmds,
	CLI_CMD_ARG(info,  NULL, "backend / model / tensor shapes / arena",
	            cmd_ai_info,  1, 0),
	CLI_CMD_ARG(model, ai_model_subcmds,
	            "runtime model swap (load <slot> / unload)", cmd_ai_model, 1, 2),
	CLI_CMD_ARG(bench, NULL, "run inference [n] times, report latency",
	            cmd_ai_bench, 1, 1),
	CLI_CMD_ARG(out,   NULL, "dequantized values of output tensor [i] [n]",
	            cmd_ai_out,   1, 2),
	CLI_CMD_ARG(dets,   NULL, "decode the current outputs into face boxes",
	            cmd_ai_dets,   1, 0),
	CLI_CMD_ARG(thresh, NULL, "score threshold [milli-probability 1..999]",
	            cmd_ai_thresh, 1, 1),
#if BSP_ENABLE_CAMERA
	CLI_CMD(run, NULL, "one-shot: grab a camera frame, infer, print the boxes",
	        cmd_ai_run),
	CLI_CMD_ARG(stream, ai_stream_subcmds,
	            "continuous camera inference (start [test] / stop / stats)",
	            cmd_ai_stream, 1, 2),
	CLI_CMD_ARG(norm, NULL, "input normalization [0=[0,1] | 1=[-1,1]]",
	            cmd_ai_norm, 1, 1),
#if BSP_ENABLE_LCD
	CLI_CMD_ARG(overlay, NULL, "draw the boxes on the LCD preview [on|off]",
	            cmd_ai_overlay, 1, 1),
#endif
#endif
	CLI_SUBCMD_SET_END);

CLI_CMD_REGISTER(ai, ai_subcmds,
                 "on-device NN inference (issue #9)", NULL, 1, 0);
