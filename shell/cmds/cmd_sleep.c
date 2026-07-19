/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    cmd_sleep.c
 * @brief   `sleep` (seconds) and `usleep` (microseconds) delay commands (#21).
 *
 * `sleep N`  -- block N seconds, cancellable with Ctrl+C.  Built on cli_sleep()
 *               (issue #16): it waits on the instance event flags, so a 0x03
 *               wakes it and the dispatcher prints "^C".
 * `usleep N` -- busy-wait N microseconds on the DWT cycle counter (udelay, CPU
 *               clock, SystemCoreClock = 550 MHz).  Short, CPU-bound, NOT
 *               interruptible -- capped small; use `sleep` for long delays.
 *
 * Linked into the threadx executable only (like cmd_system.c / cmd_thread.c).
 * Clean-room design; no third-party code reused.
 */
#include <stdint.h>

#include "cli.h"
#include "timebase.h"   /* udelay (DWT cycle-counter busy-wait) */

/* Parse a plain decimal uint32 (no 0x, no sign).  Empty, non-digit, or 32-bit
 * overflow all fail.  No newlib strtoul (this firmware ships its own parsers). */
static int parse_uint(const char *s, uint32_t *out)
{
	uint32_t val = 0;

	if (s == NULL || *s == '\0')
		return -1;
	for (const char *p = s; *p != '\0'; p++) {
		if (*p < '0' || *p > '9')
			return -1;
		uint32_t d = (uint32_t)(*p - '0');
		if (val > (0xFFFFFFFFu - d) / 10u)      /* overflow */
			return -1;
		val = val * 10u + d;
	}
	*out = val;
	return 0;
}

static int cmd_sleep(struct cli_instance *sh, int argc, char **argv)
{
	uint32_t sec;

	(void)argc;
	if (parse_uint(argv[1], &sec) != 0 || sec > CLI_SLEEP_MAX_SEC) {
		cli_error(sh, "sleep: bad seconds '%s' (0..%u)\r\n",
		          argv[1], (unsigned)CLI_SLEEP_MAX_SEC);
		return 1;
	}
	/* Cancellable (Ctrl+C): cli_sleep returns non-zero on cancel; the dispatcher
	 * then prints "^C" via cancel_req.  1 tick == 1 ms, so sec*1000 ticks. */
	return cli_sleep(sh, sec * 1000u) ? 1 : 0;
}

static int cmd_usleep(struct cli_instance *sh, int argc, char **argv)
{
	uint32_t us;

	(void)argc;
	if (parse_uint(argv[1], &us) != 0 || us > CLI_USLEEP_MAX_US) {
		cli_error(sh, "usleep: bad microseconds '%s' (0..%u)\r\n",
		          argv[1], (unsigned)CLI_USLEEP_MAX_US);
		return 1;
	}
	udelay(us);   /* busy-wait; not interruptible (kept small by the cap) */
	return 0;
}

CLI_CMD_REGISTER(sleep, NULL, "sleep N seconds (Ctrl+C cancels)", cmd_sleep, 2, 0);
CLI_CMD_REGISTER(usleep, NULL, "busy-wait N microseconds (not interruptible)",
                 cmd_usleep, 2, 0);
