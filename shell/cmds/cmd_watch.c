/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    cmd_watch.c
 * @brief   `watch [-n SEC] CMD...` -- re-run CMD every SEC seconds (#21).
 *
 * Clears the screen, runs CMD, waits SEC seconds (default CLI_WATCH_DEFAULT_SEC),
 * repeats until Ctrl+C.  Like `top` over `thread` (`watch -n 1 thread`).
 *
 * Re-dispatch: CMD is the raw tail (CLI_ARG_RAW).  cli_parse() tokenizes in place
 * and writes only into the caller's argv/result, with no global state, so we run
 * CMD by parsing a LOCAL copy into LOCAL scratch and calling the resolved
 * handler directly -- sh->line / sh->argv / sh->pr are never touched.  The
 * recursion/danger denylist is checked against the parser-NORMALISED root token
 * (largv[0], i.e. after quote/escape handling) so `watch "reboot"` etc. cannot
 * slip past it.  Cancellation rides on #16: the inner command's own
 * cli_cancel_requested() polls, the interval cli_sleep(), and an explicit check
 * all observe a Ctrl+C (cancel_req is sticky until the outer dispatch ends).
 *
 * Linked into the threadx executable only.  Clean-room design; no code reused.
 */
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* cli_instance.h (ThreadX-aware) gives the full struct cli_instance for the
 * sh->tx_failed reset below, and transitively cli_internal.h (cli_parse, struct
 * cli_parse_result, CLI_ARGV_CAP) + cli.h.  cmd_watch.c is firmware-only, so the
 * ThreadX dependency it pulls in is fine (like cmd_thread.c). */
#include "cli_instance.h"
#include "cli_vt100.h"      /* CLI_VT100_CLR_SCREEN */

/* Commands that must not be watched: watch (unbounded stack recursion),
 * coremark (~12 s blocking, not interruptible, prints via printf not cli_*),
 * reboot (would reset on the first iteration). */
static int watch_denied(const char *root)
{
	static const char *const deny[] = { "watch", "coremark", "reboot" };

	for (size_t i = 0; i < sizeof deny / sizeof deny[0]; i++)
		if (strcmp(root, deny[i]) == 0)
			return 1;
	return 0;
}

static int cmd_watch(struct cli_instance *sh, int argc, char **argv)
{
	uint32_t interval = CLI_WATCH_DEFAULT_SEC;
	const char *cmd;
	char  buf[CLI_CMD_BUFFER_SIZE];
	char *largv[CLI_ARGV_CAP];
	struct cli_parse_result lpr;
	size_t clen;

	if (argc < 2) {
		cli_error(sh, "watch: usage: watch [-n SEC] CMD...\r\n");
		return 1;
	}
	cmd = argv[1];   /* raw, untokenized tail (leading space already trimmed) */

	/* Optional "-n SEC" prefix (manually split: the tail is not tokenized). */
	if (cmd[0] == '-' && cmd[1] == 'n') {
		const char *p = cmd + 2;
		uint32_t v = 0;

		while (*p == ' ' || *p == '\t')
			p++;
		if (*p < '0' || *p > '9') {
			cli_error(sh, "watch: -n needs a number\r\n");
			return 1;
		}
		for (; *p >= '0' && *p <= '9'; p++) {
			uint32_t d = (uint32_t)(*p - '0');

			/* Bound-check BEFORE the multiply so a huge value cannot wrap past
			 * the cap (e.g. "-n 4294967296" must not fold to 0). */
			if (v > CLI_WATCH_MAX_SEC / 10u ||
			    (v == CLI_WATCH_MAX_SEC / 10u && d > CLI_WATCH_MAX_SEC % 10u)) {
				cli_error(sh, "watch: interval too large (max %u)\r\n",
				          (unsigned)CLI_WATCH_MAX_SEC);
				return 1;
			}
			v = v * 10u + d;
		}
		while (*p == ' ' || *p == '\t')
			p++;
		interval = v;
		cmd = p;
	}

	if (*cmd == '\0') {
		cli_error(sh, "watch: missing command\r\n");
		return 1;
	}
	clen = strlen(cmd);
	if (clen >= sizeof buf) {                 /* defensive: cmd is a suffix of sh->line */
		cli_error(sh, "watch: command too long\r\n");
		return 1;
	}

	/* Pre-resolve once: reject parse errors (no loop) and denylisted roots, using
	 * the parser-normalised first token. */
	memcpy(buf, cmd, clen + 1);
	if (cli_parse(buf, largv, CLI_ARGV_CAP, &lpr) != CLI_PARSE_OK) {
		cli_error(sh, "watch: cannot run '%s'\r\n", cmd);
		return 1;
	}
	if (watch_denied(largv[0])) {
		cli_error(sh, "watch: cannot watch '%s'\r\n", largv[0]);
		return 1;
	}

	for (;;) {
		if (cli_cancel_requested(sh))         /* -n 0 / cancel just after a check */
			break;
		sh->tx_failed = 0;                    /* fresh per refresh (we bypass cli_dispatch_line) */
		cli_write(sh, CLI_VT100_CLR_SCREEN, sizeof(CLI_VT100_CLR_SCREEN) - 1);
		cli_print(sh, "Every %us: %s\r\n\r\n", (unsigned)interval, cmd);

		memcpy(buf, cmd, clen + 1);           /* cli_parse destroys buf, recopy */
		if (cli_parse(buf, largv, CLI_ARGV_CAP, &lpr) != CLI_PARSE_OK)
			break;                        /* defensive: pre-resolved OK already */
		lpr.cmd->handler(sh, lpr.argc, lpr.argv);

		if (cli_cancel_requested(sh))         /* Ctrl+C during the inner command */
			break;
		if (cli_sleep(sh, interval * 1000u))  /* interval wait; non-zero == cancelled */
			break;
	}
	return 0;   /* on cancel the dispatcher prints "^C" via cancel_req */
}

CLI_CMD_REGISTER(watch, NULL, "watch [-n SEC] CMD... (re-run; Ctrl+C stops)",
                 cmd_watch, 1, CLI_ARG_RAW);
