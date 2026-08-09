/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    cmd_ai.c
 * @brief   `ai` shell command: on-device NN inference (issue #9 phase 1).
 *
 *   ai info        backend / model / tensor shapes / quantization / arena
 *   ai bench [n]   run inference n times, report min/avg/max latency
 *
 * Phase 1 ships the backend-agnostic plumbing with the `null` stub behind it, so
 * both subcommands work end to end while doing no actual inference; `ai info` says
 * so.  The camera-driven `ai run` / `ai stream` and a real model arrive later.
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

	/* Say plainly that nothing is being inferred, so a latency from `ai bench` is
	 * never mistaken for a model's. */
	if (strcmp(bi->name, "null") == 0)
		cli_warn(sh, "note    : synthetic workload, not inference\r\n");

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

	/* Unlike `ai info`, benchmarking touches the tensor bodies, and those live in
	 * the external PSRAM (port/nn/nn_null.c). */
	if (!psram_ready()) {
		cli_error(sh, "ai: PSRAM not ready -- the tensors live there "
		          "(see `psram info`)\r\n");
		return 1;
	}

	/* The single inference session first: claiming the OCTOSPI1 guard before it
	 * would mean holding a hardware resource while telling the user the software
	 * one was busy. */
	if (nn_session_try_acquire() != 0) {
		cli_error(sh, "ai: NN busy (another bench is running)\r\n");
		return 1;
	}

	/* Read/write access to the PSRAM, not a reconfiguration of it, so this is the
	 * shared guard -- it coexists with an already-running LTDC scan-out or camera
	 * stream.  It is still a single lock, so while it is held another `lcd on` or
	 * `camera capture` is refused as well; the iteration cap and Ctrl+C bound how
	 * long that lasts. */
	if (!psram_acquire_shared()) {
		nn_session_release();
		cli_error(sh, "ai: OCTOSPI1 busy (a psram/membench/wifi flash command "
		          "holds it)\r\n");
		return 1;
	}

	/* .psram_noinit is NOLOAD, so the input holds whatever survived the last reset
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
			psram_release();
			nn_session_release();
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

	psram_release();
	nn_session_release();

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
	return 0;
}

CLI_SUBCMD_SET_CREATE(ai_subcmds,
	CLI_CMD_ARG(info,  NULL, "backend / model / tensor shapes / arena",
	            cmd_ai_info,  1, 0),
	CLI_CMD_ARG(bench, NULL, "run inference [n] times, report latency",
	            cmd_ai_bench, 1, 1),
	CLI_SUBCMD_SET_END);

CLI_CMD_REGISTER(ai, ai_subcmds,
                 "on-device NN inference (issue #9)", NULL, 1, 0);
