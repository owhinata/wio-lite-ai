/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    cmd_coremark.c
 * @brief   `coremark` built-in shell command: run the EEMBC CoreMark benchmark.
 *
 * The benchmark is built once into the coremark_obj OBJECT library (the
 * lib/coremark sources plus the port in port/coremark, compiled at -O3
 * -funroll-loops with MEM_METHOD=MEM_STATIC) and linked into the shell firmware;
 * this handler just calls into it.
 *
 * core_main.c is compiled with -Dmain=coremark_main (see CMakeLists.txt) so the
 * benchmark entry does not collide with the firmware main() -- hence the local
 * declaration below.  It runs synchronously in the calling shell instance thread
 * (~10-100 s while CoreMark auto-calibrates), so the prompt is blocked until it
 * finishes; the LED heartbeat thread (higher priority) keeps blinking.  The data
 * block is static (MEM_STATIC), so it lives in .bss, not on this thread's stack.
 *
 * Output: CoreMark prints its canonical report via ee_printf -> printf, which the
 * USB CDC backend's strong _write routes to the same console as cli_print (the
 * CDC TX ring is sized to hold the whole report; the timed region itself does no
 * I/O, so TX back-pressure cannot perturb the score).  With the I-cache enabled
 * (app/main.c) the score reflects cached XIP execution rather than raw flash
 * fetch stalls.
 *
 * A singleton guard rejects a second concurrent run (e.g. `coremark &` twice):
 * the EEMBC port keeps global timing/seed state that overlapping runs would
 * corrupt.  Not a dangerous command (read-only CPU benchmark), so it is not gated
 * behind CLI_ENABLE_DANGEROUS_CMDS.
 *
 * Clean-room glue; the CoreMark sources themselves are EEMBC's (Apache-2.0).
 */
#include "cli.h"
#include "stm32h7xx_hal.h"   /* __get_PRIMASK / __disable_irq / __set_PRIMASK */

/* CoreMark entry, renamed from main() by -Dmain=coremark_main on core_main.c. */
int coremark_main(void);

/* Reentrancy guard: only one CoreMark run at a time (fg cli thread + bg workers). */
static volatile uint8_t coremark_busy;

static int coremark_try_acquire(void)
{
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	int ok = !coremark_busy;
	if (ok)
		coremark_busy = 1u;
	__set_PRIMASK(pm);
	return ok;
}

static int cmd_coremark(struct cli_instance *sh, int argc, char **argv)
{
	(void)argc;
	(void)argv;

	if (!coremark_try_acquire()) {
		cli_error(sh, "coremark: already running\r\n");
		return 1;
	}

	/* Not cooperatively cancellable: coremark_main() is a single blocking call
	 * into the read-only EEMBC submodule with no poll point, and it prints via
	 * ee_printf -> printf (not the shell's cli_tx_send_blocking).  Ctrl+C during
	 * the run is ignored; it just queues for the next prompt. */
	cli_info(sh, "Running CoreMark (auto-calibrated, ~10-100s; not interruptible)...\r\n");
	coremark_main();   /* prints the canonical CoreMark report via printf -> CDC */

	coremark_busy = 0u;
	return 0;
}

CLI_CMD_REGISTER(coremark, NULL, "run the EEMBC CoreMark benchmark (~10-100s)",
                 cmd_coremark, 1, 0);
