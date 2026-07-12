/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    cmd_dmesg.c
 * @brief   `dmesg` shell command: replay the RAM log ring.
 *
 *   dmesg                     print every stored record oldest -> newest
 *   dmesg -c                  print, then clear the ring
 *   dmesg -n <err|wrn|inf|dbg>  set the run-time severity threshold (like Linux)
 *
 * Records are read through the snapshot iterator (log_iter_*): each
 * log_iter_next() copies one record out under a short PRIMASK critical section,
 * so a thread/ISR logging concurrently never corrupts the dump.  Output uses the
 * standard buffered cli_print() and polls cli_cancel_requested() between lines so
 * Ctrl+C stops a long replay (same pattern as the hexdump path).
 *
 * Clean-room design; no third-party code reused.
 */
#include "cli.h"
#include "log.h"

#include <string.h>

static const char *lvl_str(uint8_t level)
{
	switch (level) {
	case LOG_LEVEL_ERR: return "ERR";
	case LOG_LEVEL_WRN: return "WRN";
	case LOG_LEVEL_INF: return "INF";
	default:            return "DBG";
	}
}

static int parse_level(const char *s, unsigned *out)
{
	if (!strcmp(s, "err") || !strcmp(s, "0")) { *out = LOG_LEVEL_ERR; return 0; }
	if (!strcmp(s, "wrn") || !strcmp(s, "1")) { *out = LOG_LEVEL_WRN; return 0; }
	if (!strcmp(s, "inf") || !strcmp(s, "2")) { *out = LOG_LEVEL_INF; return 0; }
	if (!strcmp(s, "dbg") || !strcmp(s, "3")) { *out = LOG_LEVEL_DBG; return 0; }
	return -1;
}

static int cmd_dmesg(struct cli_instance *sh, int argc, char **argv)
{
	int clear = 0;

	for (int i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "-c")) {
			clear = 1;
		} else if (!strcmp(argv[i], "-n")) {
			unsigned lvl;
			if (i + 1 >= argc) {
				cli_error(sh, "dmesg: -n needs a level\r\n");
				return 1;
			}
			if (parse_level(argv[++i], &lvl) != 0) {
				cli_error(sh, "dmesg: bad level '%s' (err/wrn/inf/dbg)\r\n",
				          argv[i]);
				return 1;
			}
			log_set_level(lvl);
			cli_print(sh, "dmesg: level = %s\r\n", lvl_str((uint8_t)lvl));
			return 0;
		} else {
			cli_error(sh, "dmesg: bad option '%s'\r\n", argv[i]);
			return 1;
		}
	}

	struct log_iter it;
	struct log_record rec;

	log_iter_start(&it);
	while (log_iter_next(&it, &rec)) {
		if (cli_cancel_requested(sh))
			return 1;
		if (cli_print(sh, "[%6lu.%03lu] %s %s: %s\r\n",
		              (unsigned long)(rec.ts_ms / 1000u),
		              (unsigned long)(rec.ts_ms % 1000u),
		              lvl_str(rec.level), rec.tag, rec.text) < 0)
			return 1;
	}

	if (clear)
		log_clear();
	return 0;
}

CLI_CMD_REGISTER(dmesg, NULL, "show RAM log (-c clear, -n <lvl> set level)",
                 cmd_dmesg, 1, 2);
