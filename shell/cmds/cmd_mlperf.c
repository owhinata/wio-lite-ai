/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    cmd_mlperf.c
 * @brief   `mlperf` -- hand the console to the MLPerf Tiny / EEMBC monitor (issue #55).
 *
 * The harness itself is port/mlperf.  This file is everything AROUND it: which model
 * it may run, which console it may own, which hardware it holds while it owns it, and
 * how the operator gets out.  Splitting it that way keeps port/mlperf free of guards
 * and cli_* calls, and keeps the protocol out of here.
 *
 * HOW A RUN LOOKS
 *
 *     wio> ai model load 0            # a benchmark model from a NOR blob slot
 *     wio> mlperf                     # console goes raw; the host drives from here
 *     ...                             # PC: python main.py --mode=p
 *     ^]                              # back to the shell, with a local summary
 *
 * The host end is lib/mlperf-tiny/benchmark/runner/main.py.  It opens the same
 * /dev/ttyACM0 this shell normally lives on, so the operator must not be holding a
 * terminal open on it -- see the banner.
 */
#include "cli.h"
#include "mlperf_th.h"
#include "nn.h"
#include "psram.h"
#include "net_shell.h"       /* net_shell_guard(): this needs the real serial port */
#include "stm32h7xx_hal.h"   /* SystemCoreClock: the clock the DWT summary counts in */

#include <stdint.h>

/*
 * Ctrl+] -- the telnet escape, and the one control character a terminal will send
 * that the EEMBC protocol has no use for.  It cannot collide with the protocol: every
 * host->DUT byte is printable ASCII (commands and hex digits) terminated by '%'.
 *
 * 🔴 Ctrl+C DELIBERATELY DOES NOT WORK HERE.  In raw mode cli_read_byte() hands 0x03
 * over as an ordinary byte (shell/core/cli_core.c), which is what a binary protocol
 * needs -- and the shell's own cancel path is off for the same reason.  Advertising
 * Ctrl+C and then not honouring it would be worse than picking a key nobody presses
 * by accident.
 */
#define MLPERF_ESCAPE   0x1D

/* How long to block for one byte before looking around.  The host is idle between
 * commands for as long as it takes a human to start it, so this is not a timeout on
 * anything -- it is how often the loop gets to notice a stop request. */
#define MLPERF_POLL_MS  200u

static int mlperf_guards_take(struct cli_instance *sh)
{
	/*
	 * Same order as `ai bench` (shell/cmds/cmd_ai.c): the software session first,
	 * then the hardware.  Taking the PSRAM first would mean holding a peripheral in
	 * order to discover that a second console is already using the model.
	 */
	if (nn_session_try_acquire() != 0) {
		cli_error(sh, "mlperf: NN busy (another ai/mlperf command is running)\r\n");
		return -1;
	}
	if (!psram_ready()) {
		cli_error(sh, "mlperf: PSRAM not ready -- the arena and the model live "
		              "there (see `psram info`)\r\n");
		nn_session_release();
		return -1;
	}
	if (!psram_acquire_shared()) {
		cli_error(sh, "mlperf: OCTOSPI1 busy (a psram/membench/wifi flash command "
		              "holds it, or `ai stream` is running)\r\n");
		nn_session_release();
		return -1;
	}
	return 0;
}

static void mlperf_guards_give(void)
{
	psram_release();
	nn_session_release();
}

/*
 * The monitor loop.  Console already raw, guards already held.
 *
 * Returns 0 on a clean escape, -1 if the shell instance is stopping (cli_read_byte
 * reports that separately from a timeout, and a stopping instance must not be kept
 * waiting for a host that may never speak again).
 */
static int mlperf_monitor(struct cli_instance *sh)
{
	mlperf_monitor_start();

	for (;;) {
		int c = cli_read_byte(sh, MLPERF_POLL_MS);

		if (c == -2)
			return -1;              /* instance stopping */
		if (c == -1)
			continue;               /* idle: the host has not sent anything */
		if (c == MLPERF_ESCAPE)
			return 0;

		/*
		 * Synchronous by design: a whole `infer 20 5` runs inside this call, for
		 * seconds at a time.  Nothing else on this console can proceed meanwhile,
		 * which is exactly the isolation a latency benchmark wants -- and the IWDG
		 * petter (priority 5) and the USB thread (8) both outrank the CLI thread,
		 * so the board stays alive and the CDC keeps draining.
		 */
		mlperf_feed((char)c);
	}
}

static int cmd_mlperf(struct cli_instance *sh, int argc, char **argv)
{
	const struct mlperf_stats *st;
	struct nn_model *m = NULL;
	uint32_t mhz = SystemCoreClock / 1000000u;
	int rc, stopped;

	(void)argc; (void)argv;

	/*
	 * Not over telnet, for the same reason YMODEM is not (shell/cmds/cmd_xfer.c):
	 * the runner is a serial program talking to a real port, and the telnet path
	 * escapes bytes, delivers them in ~250 ms polls and rides an eRPC link whose
	 * round trip dwarfs the protocol's own patience.
	 */
	if (net_shell_guard(sh, "mlperf"))
		return 1;

	if (nn_model_open(&m) != 0) {
		cli_error(sh, "mlperf: model open failed\r\n");
		return 1;
	}
	if (nn_input_count(m) == 0) {
		cli_error(sh, "mlperf: no model loaded -- `blob list`, then "
		              "`ai model load <slot>`\r\n");
		return 1;
	}

	if (mlperf_guards_take(sh) != 0)
		return 1;

	/*
	 * Bind BEFORE going raw, so a model we cannot drive is refused while the
	 * operator can still read why.  Once the console is raw the only listener is a
	 * Python program matching regexes.
	 */
	rc = mlperf_bind(sh, m);
	if (rc != 0) {
		cli_error(sh, "mlperf: %s\r\n", mlperf_strerror(rc));
		cli_print(sh, "  loaded: %s\r\n", nn_model_name(m));
		cli_print(sh, "  the benchmarks are ic01 (1x32x32x3->10), "
		              "kws01 (1x49x10x1->12), vww01 (1x96x96x3->2) and "
		              "ad01 (1x640->640), all int8\r\n");
		mlperf_guards_give();
		return 1;
	}

	/* The banner goes out BEFORE the claim: after it, every byte on this console is
	 * protocol, and a stray human-readable line is at best noise in the host's log. */
	cli_print(sh, "mlperf: EEMBC monitor -- model %s (%s)\r\n",
	          mlperf_model_id(), nn_model_name(m));
	cli_print(sh, "  this console and the PSRAM are held for the whole session; "
	              "`lcd on`, `camera capture` and `psram` will report busy\r\n");
	cli_print(sh, "  close your terminal, then run the runner against this port; "
	              "press Ctrl+] here to exit (Ctrl+C does NOT work)\r\n");

	switch (cli_console_claim(sh)) {
	case 0:
		break;
	case -2:
		cli_error(sh, "mlperf: run it in the foreground, not as a background "
		              "job -- it owns the console's receive path\r\n");
		mlperf_unbind();
		mlperf_guards_give();
		return 1;
	default:
		cli_error(sh, "mlperf: could not take the console\r\n");
		mlperf_unbind();
		mlperf_guards_give();
		return 1;
	}

	cli_rx_flush(sh);              /* drop type-ahead / the command's own newline */
	stopped = mlperf_monitor(sh);
	cli_rx_flush(sh);              /* drop whatever the host was mid-sentence on  */
	cli_console_release(sh);

	/*
	 * The summary is ours, printed after the protocol is over.  It is not a MLPerf
	 * result and must not be read as one: the cycle count is the DWT total across
	 * every th_infer() INCLUDING the warm-up iterations the host excludes from its
	 * own timing, so it answers "what did the core do" and not "what would be
	 * submitted".  The host's own throughput line is the number that matters.
	 */
	st = mlperf_get_stats();
	cli_print(sh, "\r\nmlperf: %s, %lu inference(s)\r\n",
	          stopped ? "stopped" : "exited", (unsigned long)st->inferences);
	if (st->inferences != 0u && mhz != 0u) {
		uint64_t per = st->cycles / st->inferences;

		cli_print(sh, "  local : %lu cycles/inference, %lu us "
		              "(DWT, warm-ups included -- not the reported score)\r\n",
		          (unsigned long)per, (unsigned long)(per / mhz));
	}
	if (st->infer_errors || st->short_loads || st->truncated || st->tx_errors)
		cli_warn(sh, "  faults: %lu infer, %lu short load(s), %lu truncated "
		             "line(s), %lu tx error(s)\r\n",
		         (unsigned long)st->infer_errors, (unsigned long)st->short_loads,
		         (unsigned long)st->truncated, (unsigned long)st->tx_errors);

	mlperf_unbind();               /* nothing may print protocol after this */
	mlperf_guards_give();
	return 0;
}

/* No subcommands: the whole command is "give the console to the host".  Everything
 * that could have been an option is either already state (which model is loaded) or
 * belongs to the host's test script. */
CLI_CMD_REGISTER(mlperf, NULL,
                 "hand the console to the MLPerf Tiny monitor (issue #55)",
                 cmd_mlperf, 1, 0);
