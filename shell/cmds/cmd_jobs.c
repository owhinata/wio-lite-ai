/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    cmd_jobs.c
 * @brief   `jobs` and `kill` background-job control commands (issue #25).
 *
 * `jobs`      -- list the running background jobs ([id] + command); reaps and
 *                announces any that have finished first.
 * `kill %N`   -- request a cooperative stop of the job with id N (a leading '%'
 *                is optional, matching the shell job-spec syntax).  Cancellation
 *                is cooperative (issue #16): a handler that polls
 *                cli_cancel_requested() / uses cli_sleep() stops promptly; a
 *                non-cooperative one (e.g. coremark) ignores it and runs to the
 *                end -- the kill is recorded either way.
 *
 * The job table + worker pool live in cli_job.c; this file is just the command
 * front-end.  Linked into the threadx executable only (like cmd_thread.c).
 * Clean-room design; no third-party code reused.
 */
#include <stdint.h>

#include "cli.h"
#include "cli_instance.h"   /* cli_jobs_print / cli_job_kill (issue #25) */

/* Parse a job id: an optional leading '%' then a plain decimal (no sign/0x).
 * Empty, non-digit, or overflow fail.  (Own parser; this firmware ships no
 * newlib strtoul.) */
static int parse_job_id(const char *s, unsigned long *out)
{
	unsigned long val = 0;

	if (s == NULL)
		return -1;
	if (*s == '%')
		s++;
	if (*s == '\0')
		return -1;
	for (const char *p = s; *p != '\0'; p++) {
		if (*p < '0' || *p > '9')
			return -1;
		unsigned long d = (unsigned long)(*p - '0');
		if (val > (0xFFFFFFFFUL - d) / 10UL)        /* 32-bit overflow */
			return -1;
		val = val * 10UL + d;
	}
	*out = val;
	return 0;
}

static int cmd_jobs(struct cli_instance *sh, int argc, char **argv)
{
	(void)argc;
	(void)argv;
	cli_jobs_print(sh);
	return 0;
}

static int cmd_kill(struct cli_instance *sh, int argc, char **argv)
{
	unsigned long id;

	(void)argc;
	if (parse_job_id(argv[1], &id) != 0) {
		cli_error(sh, "kill: bad job id '%s' (use %%N)\r\n", argv[1]);
		return 1;
	}
	if (cli_job_kill(sh, id) != 0) {
		cli_error(sh, "kill: no such job %lu\r\n", id);
		return 1;
	}
	cli_print(sh, "[%lu] kill requested\r\n", id);
	return 0;
}

CLI_CMD_REGISTER(jobs, NULL, "list background jobs", cmd_jobs, 1, 0);
CLI_CMD_REGISTER(kill, NULL, "kill %N: stop background job N", cmd_kill, 2, 0);
