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
#include "net_shell.h"   /* refuse a raw transfer on the telnet console */
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

/* ---- post-failure drain --------------------------------------------------- *
 *
 * After a FAILED transfer, keep discarding until the line goes quiet.
 *
 * ONE cli_rx_flush() IS NOT ENOUGH, and issue #10 caught it on hardware: a peer
 * that has just been sent CAN does not stop instantly.  lrzsz retries the block it
 * was on a couple of times before it accepts the cancel, and those retries arrive
 * AFTER the flush -- so they land in the line editor, which echoes them and RUNS
 * THEM AS A COMMAND.  Rejecting an oversized file with `blob write` put the
 * sender's block 0 (filename, size, mtime, mode) on the prompt and executed the
 * filename.  It printed "command not found" that time; the point is that it is the
 * SENDER choosing what the shell executes, which is not a thing to leave in.
 *
 * Bounded two ways, because the peer might not stop at all: a quiet window that
 * ends as soon as nothing more arrives, and a byte cap so the loop terminates
 * against a peer that never stops.  Both directions drain -- a receiver that never
 * saw our CAN goes on NAKing exactly like a sender does.
 *
 * BE HONEST ABOUT WHAT THE CAP MEANS.  Hitting it is not a clean finish: it is
 * "stopped discarding while bytes were still coming", i.e. the original bug again.
 * So it is set far past anything a real peer emits after a CAN -- lrzsz's retried
 * block 0 is ~133 B, and this is 256 KB, which USB CDC moves in a fraction of a
 * second, because draining bytes is nearly free while returning early is exactly
 * what puts them on the prompt.  If it is ever reached anyway, that is SAID, on the
 * console and in the log, rather than being allowed to read as a clean drain.
 *
 * Nor is the quiet window a proof of quiescence -- nothing available here is.  A
 * peer that goes quiet for a second and then retries can still get bytes to the
 * prompt.  It is a mitigation sized against measured behaviour (on board #2, lrzsz
 * retried twice within well under a second and then quit), not a guarantee.  What
 * keeps that acceptable is that the sender and the console operator are the SAME
 * party: this is robustness, not a privilege boundary being defended.
 *
 * ONLY ON THE FAILURE PATH.  After YM_OK the batch is closed and the peer has
 * stopped by construction, so the proven success path keeps its timing unchanged.
 */
#define XFER_DRAIN_QUIET_MS    1000u
#define XFER_DRAIN_MAX_BYTES   262144u

static void xfer_drain_until_quiet(struct cli_instance *sh)
{
	uint32_t dropped = 0u;

	while (dropped < XFER_DRAIN_MAX_BYTES) {
		/* <0 is either the quiet window expiring or the session going away;
		 * both mean stop.  cli_read_byte() waits on an event flag rather than
		 * spinning, so the IWDG petter keeps running throughout. */
		if (cli_read_byte(sh, XFER_DRAIN_QUIET_MS) < 0)
			break;
		dropped++;
	}
	cli_rx_flush(sh);

	if (dropped >= XFER_DRAIN_MAX_BYTES) {
		cli_warn(sh, "xfer: the peer is STILL sending -- stopped discarding after "
		         "%lu B, so stray bytes may reach the prompt and be run as a "
		         "command. Stop the program on the PC.\r\n",
		         (unsigned long)dropped);
		log_write(LOG_LEVEL_WRN, "xfer",
		          "drain cap hit at %lu B -- peer did not stop",
		          (unsigned long)dropped);
	} else if (dropped != 0u) {
		log_write(LOG_LEVEL_INF, "xfer",
		          "drained %lu late byte(s) from the peer after a failed transfer",
		          (unsigned long)dropped);
	}
}

int xfer_send_source_locked(struct cli_instance *sh, const struct ym_source *src)
{
	struct ym_io   io = { sh, io_getc, io_put };
	enum ym_result res;

	/*
	 * Not over telnet.  YMODEM needs a raw, byte-timed console: the telnet path escapes
	 * 0xFF (IAC) on the way out, hands bytes over in ~250 ms polls, and rides an eRPC link
	 * whose round-trip dwarfs the protocol's own timeouts -- the transfer would corrupt or
	 * stall.  Checked here rather than in each caller so every YMODEM user (`xfer`,
	 * `wifi flash imgload` / `flash backup`) is covered by one rule.
	 */
	if (net_shell_guard(sh, "ymodem"))
		return 1;

	cli_rx_flush(sh);                  /* drop type-ahead / the command newline */
	res = ymodem_send(&io, src);
	cli_rx_flush(sh);                  /* drop a trailing 'O'/'C'/garbage tail   */
	if (res != YM_OK)
		xfer_drain_until_quiet(sh);    /* the receiver may still be NAKing */

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

	/*
	 * Not over telnet.  YMODEM needs a raw, byte-timed console: the telnet path escapes
	 * 0xFF (IAC) on the way out, hands bytes over in ~250 ms polls, and rides an eRPC link
	 * whose round-trip dwarfs the protocol's own timeouts -- the transfer would corrupt or
	 * stall.  Checked here rather than in each caller so every YMODEM user (`xfer`,
	 * `wifi flash imgload` / `flash backup`) is covered by one rule.
	 */
	if (net_shell_guard(sh, "ymodem"))
		return 1;

	cli_rx_flush(sh);                  /* drop type-ahead / the command newline */
	res = ymodem_recv(&io, sink);
	cli_rx_flush(sh);                  /* drop a trailing 'O'/CAN/garbage tail   */
	if (res != YM_OK)
		xfer_drain_until_quiet(sh);    /* the sender may still be retrying */

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

	/* NOT "Ctrl+C aborts", which this line claimed until issue #10 noticed it
	 * contradicts the receive contract three functions up: while receiving, 0x03 is
	 * file data and io_getc_recv() deliberately passes it through, so there is no
	 * local abort at all.  Telling the operator to press a key that does nothing is
	 * worse than telling them nothing. */
	cli_print(sh, "ymodem: start the sender now -- e.g. `sb <file>` (lrzsz "
	              "YMODEM batch send); abort from the PC, not with Ctrl+C\r\n");

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
