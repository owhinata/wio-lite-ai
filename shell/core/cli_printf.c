/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    cli_printf.c
 * @brief   Shell output API: staging buffer + colour + hexdump over svc/fmt.
 *
 * The public cli_print/error/warn/info/write/hexdump are implemented here.  Each
 * call is bracketed by cli_lock/cli_unlock (per-instance TX mutex) so formatting,
 * 32 B staging and flush are atomic, then reaches the transport only through
 * cli_tx_send_blocking() -- the ThreadX-specific lock + flow control live in
 * cli_core.c.  This file calls no tx_* function, so it builds and unit-tests on
 * the host against the tx_api.h shim.
 *
 * The actual number formatting is the clean-room formatter in svc/fmt.c (issue
 * #28); this file only supplies the staging-buffer putc sink (cli_out_putc, ctx
 * = the instance) and the lock/colour brackets.  Behaviour is unchanged from the
 * earlier inline formatter.
 */
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "fmt.h"
#include "cli_instance.h"
#include "cli_internal.h"
#include "cli_vt100.h"

/* ---- staging ----------------------------------------------------------- */

void cli_out_flush(struct cli_instance *sh)
{
	if (sh->out_len == 0)
		return;
	/* Once a send has failed this command, discard the rest (req §11). */
	if (!sh->tx_failed) {
		if (cli_tx_send_blocking(sh, (const uint8_t *)sh->out_buf,
		                         sh->out_len) < 0)
			sh->tx_failed = 1;
	}
	sh->out_len = 0;
}

void cli_out_putc(struct cli_instance *sh, char c)
{
	if (sh->tx_failed)
		return;                         /* drop further output this command */
	sh->out_buf[sh->out_len++] = c;
	if (sh->out_len >= CLI_PRINTF_BUFFER_SIZE)
		cli_out_flush(sh);
}

/* fmt putc sink: ctx is the owning instance, output goes to its staging. */
static void inst_putc(void *ctx, char c)
{
	cli_out_putc((struct cli_instance *)ctx, c);
}

static void out_str(struct cli_instance *sh, const char *s)
{
	while (*s)
		cli_out_putc(sh, *s++);
}

/* ---- output bracket (lock + bg line-break, issues #5/#25) --------------- */

int cli_out_begin(struct cli_instance *sh)
{
	if (cli_lock(sh) != 0) {
		sh->tx_failed = 1;      /* so the command result is forced non-zero */
		return -1;
	}
	/* Background job (issue #25): on the FIRST output since the foreground prompt
	 * was last (re)drawn, break to a fresh line so this output does not splice
	 * into the user's half-typed line, and invalidate the fg's render bookkeeping
	 * (old_rows/draw_row = 0 -> "draw at the cursor, do not erase") so the fg
	 * repaints prompt+line below this output on its next keystroke.  Best-effort:
	 * a dropped break just leaves the staged output to set tx_failed as usual. */
	if (sh->fg != NULL && sh->fg->render_dirty == 0) {
		cli_tx_send_blocking(sh, (const uint8_t *)"\r\n", 2);
		sh->fg->old_rows     = 0;
		sh->fg->draw_row     = 0;
		sh->fg->render_dirty = 1;
	}
	return 0;
}

void cli_out_end(struct cli_instance *sh)
{
	cli_unlock(sh);
}

/* ---- public API -------------------------------------------------------- */

/* Bracket a formatted call with lock + autoflush; @p color is "" for cli_print
 * or a VT100 SGR for the level helpers (and "" when CLI_USE_COLOR=0). */
static int vemit(struct cli_instance *sh, const char *color,
                 const char *fmt, va_list ap)
{
	if (cli_out_begin(sh) != 0)
		return -1;
	if (color[0]) out_str(sh, color);
	fmt_vformat(inst_putc, sh, fmt, ap);
	if (color[0]) out_str(sh, CLI_VT100_RESET);
	cli_out_flush(sh);
	int r = sh->tx_failed ? -1 : 0;
	cli_out_end(sh);
	return r;
}

int cli_print(struct cli_instance *sh, const char *fmt, ...)
{
	va_list ap; va_start(ap, fmt);
	int r = vemit(sh, "", fmt, ap);
	va_end(ap);
	return r;
}

int cli_error(struct cli_instance *sh, const char *fmt, ...)
{
	va_list ap; va_start(ap, fmt);
	int r = vemit(sh, CLI_VT100_RED, fmt, ap);
	va_end(ap);
	return r;
}

int cli_warn(struct cli_instance *sh, const char *fmt, ...)
{
	va_list ap; va_start(ap, fmt);
	int r = vemit(sh, CLI_VT100_YELLOW, fmt, ap);
	va_end(ap);
	return r;
}

int cli_info(struct cli_instance *sh, const char *fmt, ...)
{
	va_list ap; va_start(ap, fmt);
	int r = vemit(sh, CLI_VT100_GREEN, fmt, ap);
	va_end(ap);
	return r;
}

int cli_write(struct cli_instance *sh, const void *data, size_t len)
{
	const uint8_t *p = (const uint8_t *)data;
	if (cli_out_begin(sh) != 0)
		return -1;
	for (size_t i = 0; i < len; i++)
		cli_out_putc(sh, (char)p[i]);
	cli_out_flush(sh);
	int r = sh->tx_failed ? -1 : 0;
	cli_out_end(sh);
	return r;
}

int cli_hexdump_base(struct cli_instance *sh, const void *data, size_t len,
                     unsigned long long base)
{
	const uint8_t *p = (const uint8_t *)data;
	char body[20];

	if (cli_out_begin(sh) != 0)
		return -1;

	for (size_t off = 0; off < len; off += 16) {
		/* Ctrl+C between rows: stop before emitting the next 16-byte line
		 * (issue #16).  Drop the staged tail and release the lock; the
		 * dispatcher detects cancel_req and prints "^C". */
		if (cli_cancel_requested(sh)) {
			cli_out_end(sh);
			return -1;
		}

		int n = fmt_utoa(base + (unsigned long long)off, 16, 0, body);
		fmt_padded(inst_putc, sh, NULL, body, n, 8, 1, 0);   /* %08x offset */
		out_str(sh, "  ");

		for (int j = 0; j < 16; j++) {
			if (off + (size_t)j < len) {
				int h = fmt_utoa(p[off + j], 16, 0, body);
				fmt_padded(inst_putc, sh, NULL, body, h, 2, 1, 0);  /* %02x */
				cli_out_putc(sh, ' ');
			} else {
				out_str(sh, "   ");
			}
		}
		cli_out_putc(sh, ' ');

		for (int j = 0; j < 16 && off + (size_t)j < len; j++) {
			uint8_t b = p[off + j];
			cli_out_putc(sh, (b >= 0x20 && b <= 0x7E) ? (char)b : '.');
		}
		out_str(sh, "\r\n");
	}

	cli_out_flush(sh);
	int r = sh->tx_failed ? -1 : 0;
	cli_out_end(sh);
	return r;
}

int cli_hexdump(struct cli_instance *sh, const void *data, size_t len)
{
	return cli_hexdump_base(sh, data, len, 0);
}
