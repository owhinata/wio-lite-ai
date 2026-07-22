/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    cmd_xfer.c
 * @brief   Shared YMODEM-over-console send helper (see cmd_xfer.h).
 *
 * The protocol core (svc/ymodem.c) is transport-agnostic; here we wire its IO
 * vtable to the raw-console API (cli_console_claim/cli_read_byte/cli_write/
 * cli_rx_flush).  The source vtable is supplied by the caller -- for issue #19 M4
 * that is the RTL8720DN download-protocol flash reader in cmd_wifi.c.
 *
 * Ported from ../stm32f746g-disco (its issue #50), dropping its FileX-backed
 * `xfer send sd|fs` command: this repo has no filesystem yet, so no command is
 * registered here and only the reusable send helpers live in this file.
 */
#include "cli.h"
#include "cmd_xfer.h"

#include <string.h>

/* ---- YMODEM IO adapter: cli_read_byte / cli_write ------------------------- */

static int io_getc(void *ctx, unsigned timeout_ms)
{
	int r = cli_read_byte((struct cli_instance *)ctx, timeout_ms);
	/* Map a local Ctrl+C to a transfer abort; -1/-2 already match
	 * YM_IO_TIMEOUT/YM_IO_ABORT. */
	return (r == 3) ? YM_IO_ABORT : r;
}

static int io_put(void *ctx, const uint8_t *buf, size_t len)
{
	return cli_write((struct cli_instance *)ctx, buf, len) < 0 ? -1 : 0;
}

int xfer_send_source_locked(struct cli_instance *sh, const struct ym_source *src)
{
	struct ym_io   io = { sh, io_getc, io_put };
	enum ym_result res;

	cli_rx_flush(sh);                  /* drop type-ahead / the command newline */
	res = ymodem_send(&io, src);
	cli_rx_flush(sh);                  /* drop a trailing 'O'/'C'/garbage tail   */

	switch (res) {
	case YM_OK:
		cli_print(sh, "ymodem: sent %lu bytes OK\r\n",
		          (unsigned long)src->size);
		return 0;
	case YM_ERR_CANCEL:
		cli_warn(sh, "ymodem: cancelled\r\n");
		return 1;
	case YM_ERR_TIMEOUT:
		cli_error(sh, "ymodem: timeout -- no receiver? start `rz` on the PC\r\n");
		return 1;
	case YM_ERR_SOURCE:
		cli_error(sh, "ymodem: source read error\r\n");
		return 1;
	case YM_ERR_IO:
	default:
		cli_error(sh, "ymodem: transport error\r\n");
		return 1;
	}
}

int xfer_send_source(struct cli_instance *sh, const struct ym_source *src)
{
	int rc;

	cli_print(sh, "ymodem: sending '%s' (%lu bytes) over the console\r\n",
	          src->name, (unsigned long)src->size);
	cli_print(sh, "ymodem: start the receiver now -- e.g. `rz` (lrzsz YMODEM); "
	              "Ctrl+C aborts\r\n");

	switch (cli_console_claim(sh)) {
	case 0:
		break;
	case -2:
		cli_error(sh, "ymodem: cannot run in the background -- "
		              "drop the trailing '&'\r\n");
		return 1;
	default:
		cli_error(sh, "ymodem: console busy\r\n");
		return 1;
	}
	rc = xfer_send_source_locked(sh, src);
	cli_console_release(sh);
	return rc;
}
