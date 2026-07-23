/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    cmd_xfer.c
 * @brief   Shared YMODEM-over-console send/receive helpers (see cmd_xfer.h).
 *
 * The protocol core (svc/ymodem.c) is transport-agnostic; here we wire its IO
 * vtable to the raw-console API (cli_console_claim/cli_read_byte/cli_write/
 * cli_rx_flush).  The source/sink vtable is supplied by the caller -- for issue
 * #19 M4 that is the RTL8720DN download-protocol flash reader in cmd_wifi.c, and
 * for M5 the PSRAM firmware-image staging buffer in app/rtl8720_img.c.  Both
 * directions share one io_getc()/io_put() pair and therefore one set of rules
 * about who may read the console RX ring (see cmd_xfer.h).
 *
 * Ported from ../stm32f746g-disco (its issue #50), dropping its FileX-backed
 * `xfer send sd|fs` command: this repo has no filesystem yet, so no command is
 * registered here and only the reusable send helpers live in this file.
 */
#include "cli.h"
#include "cmd_xfer.h"
#include "log.h"          /* transfer post-mortem into the dmesg ring -- see below */

#include <string.h>

/* ---- YMODEM IO adapter: cli_read_byte / cli_write ------------------------- */

/*
 * SEND direction only.  Everything the board reads back while SENDING is protocol
 * -- ACK / NAK / 'C' / CAN -- so a 0x03 can only be a human pressing Ctrl+C, and
 * mapping it to an abort is safe.
 *
 * THIS IS WRONG FOR RECEIVING, and using it there was the bug that broke `wifi
 * imgload` on the first try: while receiving, the incoming stream is the FILE, in
 * which 0x03 is just another data byte (~1 in 256 of binary content, and it can
 * equally land in a block's CRC).  Treating those as Ctrl+C aborted the transfer
 * almost immediately.  See io_getc_recv().
 */
static int io_getc(void *ctx, unsigned timeout_ms)
{
	int r = cli_read_byte((struct cli_instance *)ctx, timeout_ms);
	/* -1/-2 already match YM_IO_TIMEOUT/YM_IO_ABORT. */
	return (r == 3) ? YM_IO_ABORT : r;
}

/*
 * RECEIVE direction: pass every byte through untouched, 0x03 included.
 *
 * The cost is that a local Ctrl+C cannot abort a receive -- there is no way to tell
 * it apart from file data, and guessing wrong corrupts the image.  Aborting is done
 * from the PC instead (cancel the sender; lrzsz emits the CAN sequence, which the
 * protocol core honours), and the handshake gives up on its own if no sender ever
 * appears.  cli_read_byte() itself never interprets 0x03, so this is simply the
 * transport's raw byte.
 */
static int io_getc_recv(void *ctx, unsigned timeout_ms)
{
	return cli_read_byte((struct cli_instance *)ctx, timeout_ms);
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

/* ---- receive direction (issue #19 M5) ------------------------------------- */

int xfer_recv_sink_locked(struct cli_instance *sh, const struct ym_sink *sink)
{
	struct ym_io   io = { sh, io_getc_recv, io_put };
	const struct ym_recv_diag *d;
	enum ym_result res;

	cli_rx_flush(sh);                  /* drop type-ahead / the command newline */
	res = ymodem_recv(&io, sink);
	cli_rx_flush(sh);                  /* drop a trailing 'O'/CAN/garbage tail   */

	/*
	 * Mirror the post-mortem into the log ring as well as the console.  While a
	 * transfer is running the PC's terminal has handed the port to `sb`/`rb`, so
	 * anything printed here is swallowed by that program instead of being shown --
	 * `dmesg` after the fact is the only way the operator gets to read it.
	 */
	d = ymodem_recv_diag();
	log_write(res == YM_OK ? LOG_LEVEL_INF : LOG_LEVEL_ERR, "xfer",
	          "recv rc=%d blocks=%lu crc=%lu seq=%lu short=%lu tmo=%lu",
	          (int)res, (unsigned long)d->blocks, (unsigned long)d->bad_crc,
	          (unsigned long)d->bad_seq, (unsigned long)d->short_read,
	          (unsigned long)d->timeouts);
	if (d->first_kind >= 0)
		log_write(LOG_LEVEL_ERR, "xfer",
		          "first bad blk kind=%02X seq=%d nseq=%d body=%lu/%lu crc=%04X/%04X",
		          (unsigned)d->first_kind, d->first_seq, d->first_nseq,
		          (unsigned long)d->first_got, (unsigned long)d->first_want,
		          d->first_crc_want, d->first_crc_got);

	switch (res) {
	case YM_OK:
		cli_print(sh, "ymodem: received OK\r\n");
		return 0;
	case YM_ERR_CANCEL:
		cli_warn(sh, "ymodem: cancelled\r\n");
		return 1;
	case YM_ERR_TIMEOUT:
		cli_error(sh, "ymodem: timeout -- no sender? start `sb <file>` on the "
		              "PC (YMODEM batch send)\r\n");
		return 1;
	case YM_ERR_SINK:
		cli_error(sh, "ymodem: rejected by the destination "
		              "(too large / not ready)\r\n");
		return 1;
	case YM_ERR_PROTO:
		cli_error(sh, "ymodem: protocol error (out-of-order block)\r\n");
		return 1;
	case YM_ERR_IO:
	default:
		cli_error(sh, "ymodem: transport error\r\n");
		return 1;
	}
}

int xfer_recv_sink(struct cli_instance *sh, const struct ym_sink *sink)
{
	int rc;

	cli_print(sh, "ymodem: start the sender now -- e.g. `sb <file>` (lrzsz "
	              "YMODEM batch send); Ctrl+C aborts\r\n");

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
	rc = xfer_recv_sink_locked(sh, sink);
	cli_console_release(sh);
	return rc;
}
