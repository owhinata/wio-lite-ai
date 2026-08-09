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
 *   ai model load <slot> read a .tflite out of a NOR blob slot and run it
 *   ai model unload      drop it again
 *
 * Phase 1 shipped the backend-agnostic plumbing with the `null` stub behind it, so
 * both original subcommands worked end to end while doing no actual inference.
 * Phase 2c added the TFLM backend and, with it, the two `ai model` subcommands --
 * which the `null` build still accepts and answers honestly, because the "this
 * backend cannot load a model" case is a value nn.c returns rather than a missing
 * command (port/nn/nn.h).  The camera-driven `ai run` / `ai stream` are phase 3.
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
#include "psram.h"

#if BSP_ENABLE_KV
#include "blob.h"            /* issue #10: the NOR asset slots a model lives in */
#include <flashdb.h>         /* fdb_calc_crc32 -- see the note at ai_model_load() */
#endif
#if BSP_ENABLE_LCD
#include "ltdc_display.h"    /* ltdc_errors: the underrun evidence around a bench */
#endif

#include "stm32h7xx_hal.h"   /* SystemCoreClock (the DWT/CPU clock) */

#include <stdint.h>
#include <string.h>

#define AI_BENCH_DEFAULT_ITERS 50u
#define AI_BENCH_MAX_ITERS     100000u

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
		/* Scale as parts-per-million: svc/fmt.c implements no %f (and no precision
		 * flag), so every fractional value in this firmware is printed as scaled
		 * integers -- the same idiom as the fps and MHz lines elsewhere. */
		uint32_t s_ppm = (uint32_t)(t->scale * 1000000.0f + 0.5f);

		cli_print(sh, "  %s[%d]  %s %s  q(s=0.%06lu zp=%ld)  %luB\r\n",
		          tag, idx, shape, ai_dtype_name(t->dtype),
		          (unsigned long)s_ppm, (long)t->zero_point,
		          (unsigned long)t->bytes);
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
		          "holds it)\r\n");
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
	CLI_SUBCMD_SET_END);

CLI_CMD_REGISTER(ai, ai_subcmds,
                 "on-device NN inference (issue #9)", NULL, 1, 0);
