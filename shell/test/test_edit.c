/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 *
 * Host unit test for the Shell line editor (issue #9): the cursor model (cur
 * split from len), in-line insert / overwrite / delete, the meta keys
 * (Ctrl+a/b/d/e/f/k/u/w, Alt+b/f, Ctrl+l), the VT100 escape parser (arrows /
 * Home / End / Del / Insert / SS3, §13 invalid-escape ignore), the CPR
 * terminal-width probe + guarded reply, and the wrap redraw row arithmetic.
 *
 * It drives the ThreadX-free cli_edit.c directly with cli_input_byte() and
 * asserts the editor *model* (line / len / cur / overwrite / term_width /
 * old_rows / draw_row) plus a few key output fragments; output and the tx_*
 * glue go through the shared dummy backend + host_glue, exactly like
 * test_core.c.  Built with the default CLI_* sizes (room for multi-row cases)
 * and colour OFF so escape bytes compare plainly.
 */
#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "cli_instance.h"
#include "cli_internal.h"
#include "cli_backend_dummy.h"
#include "host_glue.h"

/* ---- test command (for the dispatch / cursor-reset check) -------------- */

static int h_ok(struct cli_instance *sh, int argc, char **argv)
{
	(void)argc; (void)argv;
	cli_write(sh, "ran\r\n", 5);
	return 0;
}
CLI_CMD_REGISTER(ec, NULL, "echo", h_ok, 1, 0);

/* ---- harness ----------------------------------------------------------- */

CLI_BACKEND_DUMMY_DEFINE(tr);
static struct cli_instance sh;

static void reset_sh(void)
{
	memset(&sh, 0, sizeof sh);
	sh.tr = &tr;
	tr.sh = &sh;
	strcpy(sh.prompt, "> ");
	sh.term_width = 80;                    /* deterministic width (no CPR here) */
	cli_dummy_clear_output(&tr);
	cli_dummy_clear_rx(&tr);
	cli_dummy_reset_stats(&tr);
	cli_dummy_set_tx_fail(&tr, 0);
	cli_dummy_set_tx_cap(&tr, 0);          /* unlimited */
	cli_test_set_tx_wait_hook(NULL, NULL);
}

static void feed_byte(uint8_t b) { cli_input_byte(&sh, b); }
static void feed(const char *s) { for (const char *p = s; *p; p++) feed_byte((uint8_t)*p); }

/* ESC[ <s> -- a CSI sequence; s carries any params and the final byte. */
static void csi(const char *s) { feed_byte(0x1B); feed_byte('['); feed(s); }
/* ESC O <f> -- an SS3 (application-mode) sequence. */
static void ss3(char f) { feed_byte(0x1B); feed_byte('O'); feed_byte(f); }
/* ESC <c> -- a meta (Alt) key. */
static void meta(char c) { feed_byte(0x1B); feed_byte((uint8_t)c); }

static const char *cap_str(void) { return cli_dummy_output_str(&tr); }
static int cap_has(const char *needle) { return strstr(cap_str(), needle) != NULL; }

/* ---- tests ------------------------------------------------------------- */

static void test_cursor_moves(void)
{
	reset_sh();
	feed("abc");
	assert(sh.len == 3 && sh.cur == 3 && strcmp(sh.line, "abc") == 0);

	feed_byte(0x01); assert(sh.cur == 0);          /* Ctrl+a: home */
	feed_byte(0x05); assert(sh.cur == 3);          /* Ctrl+e: end */
	feed_byte(0x02); assert(sh.cur == 2);          /* Ctrl+b: left */
	feed_byte(0x06); assert(sh.cur == 3);          /* Ctrl+f: right */
	csi("D");        assert(sh.cur == 2);          /* left arrow */
	csi("C");        assert(sh.cur == 3);          /* right arrow */
	csi("H");        assert(sh.cur == 0);          /* Home */
	csi("F");        assert(sh.cur == 3);          /* End */
	ss3('D');        assert(sh.cur == 2);          /* SS3 left */
	ss3('C');        assert(sh.cur == 3);          /* SS3 right */

	/* Moves never change the buffer; bounds clamp at both ends. */
	assert(sh.len == 3 && strcmp(sh.line, "abc") == 0);
	feed_byte(0x06); assert(sh.cur == 3);          /* right at end: no-op */
	feed_byte(0x01); feed_byte(0x02); assert(sh.cur == 0);  /* left at home: no-op */
}

static void test_insert_and_overwrite(void)
{
	/* Insert in the middle shifts the tail. */
	reset_sh();
	feed("ac");
	feed_byte(0x02);                                /* cursor between a and c */
	assert(sh.cur == 1);
	feed("b");
	assert(sh.len == 3 && sh.cur == 2 && strcmp(sh.line, "abc") == 0);

	/* Insert key (ESC[2~) toggles overwrite; a char then replaces in place. */
	reset_sh();
	feed("abc");
	feed_byte(0x01);                                /* home */
	csi("2~");
	assert(sh.overwrite == 1);
	feed("X");
	assert(sh.len == 3 && sh.cur == 1 && strcmp(sh.line, "Xbc") == 0);
	csi("2~");                                      /* back to insert mode */
	assert(sh.overwrite == 0);
	feed("Y");                                      /* now inserts, not overwrites */
	assert(sh.len == 4 && strcmp(sh.line, "XYbc") == 0);
}

static void test_delete_and_backspace(void)
{
	/* Backspace at end (fast path) and forward delete. */
	reset_sh();
	feed("abc");
	feed_byte(0x7F);                                /* DEL = backspace by default */
	assert(sh.len == 2 && sh.cur == 2 && strcmp(sh.line, "ab") == 0);
	assert(cap_has("\b \b"));                       /* end-of-line fast path kept */

	feed_byte(0x01);                                /* home */
	feed_byte(0x04);                                /* Ctrl+d: delete under cursor */
	assert(sh.len == 1 && sh.cur == 0 && strcmp(sh.line, "b") == 0);

	/* Delete key (ESC[3~). */
	reset_sh();
	feed("abc");
	feed_byte(0x01);
	csi("3~");
	assert(strcmp(sh.line, "bc") == 0 && sh.cur == 0);

	/* Mid-line backspace deletes the char before the cursor. */
	reset_sh();
	feed("abc");
	feed_byte(0x02);                                /* cur = 2 */
	feed_byte(0x08);                                /* backspace -> remove 'b' */
	assert(strcmp(sh.line, "ac") == 0 && sh.cur == 1 && sh.len == 2);
}

static void test_kill_and_word(void)
{
	/* Ctrl+k kills to end of line. */
	reset_sh();
	feed("abcdef");
	feed_byte(0x01); feed_byte(0x06); feed_byte(0x06); feed_byte(0x06); /* cur=3 */
	feed_byte(0x0B);                                /* Ctrl+k */
	assert(strcmp(sh.line, "abc") == 0 && sh.len == 3 && sh.cur == 3);

	/* Ctrl+u kills from start to cursor. */
	reset_sh();
	feed("abcdef");
	feed_byte(0x01); feed_byte(0x06); feed_byte(0x06); feed_byte(0x06); /* cur=3 */
	feed_byte(0x15);                                /* Ctrl+u */
	assert(strcmp(sh.line, "def") == 0 && sh.len == 3 && sh.cur == 0);

	/* Ctrl+w deletes the word (and its trailing spaces) before the cursor. */
	reset_sh();
	feed("foo bar");
	feed_byte(0x17);                                /* Ctrl+w at end */
	assert(strcmp(sh.line, "foo ") == 0 && sh.cur == 4);
	feed_byte(0x17);                                /* again */
	assert(strcmp(sh.line, "") == 0 && sh.cur == 0 && sh.len == 0);

	/* Alt+f / Alt+b move by words. */
	reset_sh();
	feed("foo bar baz");
	feed_byte(0x01);                                /* home, cur=0 */
	meta('f'); assert(sh.cur == 3);                 /* end of "foo" */
	meta('f'); assert(sh.cur == 7);                 /* end of "bar" */
	meta('b'); assert(sh.cur == 4);                 /* start of "bar" */
	meta('b'); assert(sh.cur == 0);                 /* start of "foo" */
}

static void test_invalid_escapes_ignored(void)
{
	/* §13: an unknown CSI final and an unknown ESC <x> are ignored, leave the
	 * line untouched and return to NORMAL so following input is normal. */
	reset_sh();
	feed("ab");
	csi("99z");                                     /* unknown final 'z' */
	assert(sh.rx == CLI_RX_NORMAL && sh.len == 2 && strcmp(sh.line, "ab") == 0);
	meta('q');                                      /* unknown ESC q */
	assert(sh.rx == CLI_RX_NORMAL && sh.len == 2);
	feed("c");
	assert(strcmp(sh.line, "abc") == 0);

	/* Ctrl+C recovers even from a half-read CSI (req §9). */
	reset_sh();
	feed("ab");
	feed_byte(0x1B); feed_byte('[');                /* enter CSI */
	feed_byte(0x03);                                /* Ctrl+C */
	assert(sh.rx == CLI_RX_NORMAL && sh.len == 0 && sh.cur == 0);
	assert(cap_has("^C"));
}

static void test_cpr_width(void)
{
	/* A guarded CPR reply updates term_width. */
	reset_sh();
	sh.probing_cpr = 1;
	csi("24;100R");
	assert(sh.term_width == 100 && sh.probing_cpr == 0);
	assert(sh.len == 0);                            /* never leaked into the line */

	/* Guards reject: only one param, out-of-range column, or no probe pending. */
	reset_sh();
	sh.probing_cpr = 1;
	csi("24R");                                     /* one param */
	assert(sh.term_width == 80 && sh.len == 0);

	reset_sh();
	sh.probing_cpr = 1;
	csi("24;300R");                                 /* col > 255 */
	assert(sh.term_width == 80 && sh.len == 0);

	reset_sh();
	sh.probing_cpr = 0;
	csi("24;100R");                                 /* stray, no probe */
	assert(sh.term_width == 80 && sh.len == 0);

	reset_sh();
	sh.probing_cpr = 1;
	csi("24;;100R");                                /* extra ';' -> malformed */
	assert(sh.term_width == 80 && sh.len == 0);
}

static void test_session_start_probe(void)
{
	reset_sh();
	cli_edit_session_start(&sh);
	assert(sh.probing_cpr == 1);                    /* awaiting the reply */
	assert(cap_has("\x1b[999C"));                   /* jump far right ... */
	assert(cap_has("\x1b[6n"));                     /* ... then request CPR */
	assert(cap_has("> "));                          /* prompt drawn regardless */
}

static void test_clear_screen(void)
{
	reset_sh();
	feed("abc");
	feed_byte(0x0C);                                /* Ctrl+l */
	assert(cap_has("\x1b[2J"));                     /* clear screen emitted */
	assert(sh.len == 3 && sh.cur == 3 && strcmp(sh.line, "abc") == 0);
}

static void test_backspace_mode(void)
{
	/* Default: 0x7F erases backward.  After cli_set_backspace_mode(,1) it
	 * deletes the char under the cursor instead. */
	reset_sh();
	cli_set_backspace_mode(&sh, 1);
	assert(sh.bs_swap == 1);
	feed("abc");
	feed_byte(0x01);                                /* home, cur=0 */
	feed_byte(0x7F);                                /* DEL -> forward delete */
	assert(strcmp(sh.line, "bc") == 0 && sh.cur == 0);
	feed_byte(0x08);                                /* BS still erases backward (no-op at home) */
	assert(strcmp(sh.line, "bc") == 0 && sh.cur == 0);

	cli_set_backspace_mode(&sh, 0);
	assert(sh.bs_swap == 0);
}

static void test_wrap_rows(void)
{
	/* Narrow terminal so the line wraps; check the render row arithmetic.
	 * prompt "> " = 2 cols, width 10.  After 8 chars pend = 10 -> the line
	 * spans 2 rows (old_rows == 2) and the boundary forces a fresh row. */
	reset_sh();
	sh.term_width = 10;
	feed("abcdefgh");                               /* 8 chars */
	assert(sh.len == 8 && sh.cur == 8);
	assert(sh.old_rows == 2);                       /* prompt(2)+8 = 10 -> 2 rows */
	assert(sh.draw_row == 1);                       /* cursor on the 2nd row */

	feed_byte(0x01);                                /* home: cursor to row 0 */
	assert(sh.cur == 0 && sh.draw_row == 0);
	feed_byte(0x05);                                /* end: back to row 1 */
	assert(sh.cur == 8 && sh.draw_row == 1);
}

static void test_dispatch_resets_cursor(void)
{
	reset_sh();
	feed("ec");
	feed_byte(0x02);                                /* move cursor off the end */
	assert(sh.cur == 1);
	feed_byte('\r');                                /* submit */
	assert(cap_has("ran"));
	assert(sh.len == 0 && sh.cur == 0);             /* cursor + line reset */
	assert(sh.old_rows == 1 && sh.draw_row == 0);   /* prompt is the new baseline */
}

static void test_enter_moves_to_end(void)
{
	/* Wrapped line with the cursor pulled up to row 0: Enter must drop the
	 * cursor to the bottom render row before the dispatch newline, or command
	 * output would overwrite the input's lower row(s). */
	reset_sh();
	sh.term_width = 10;
	feed("abcdefgh");                               /* prompt(2)+8 -> 2 rows, cur on row 1 */
	feed_byte(0x01);                                /* home -> cursor to row 0 */
	assert(sh.draw_row == 0);
	cli_dummy_clear_output(&tr);
	feed_byte('\r');                                /* submit */
	assert(cap_has("\x1b[1B"));                     /* moved down to the bottom row first */
	assert(sh.len == 0 && sh.cur == 0);
}

static void test_ctrl_c_baseline(void)
{
	/* Ctrl+C draws a fresh prompt; its render baseline must be that prompt (not
	 * "no render"), so a later shortening refresh clears the stale text. */
	reset_sh();
	feed("xyz");
	feed_byte(0x03);                                /* Ctrl+C */
	assert(sh.old_rows == 1 && sh.draw_row == 0);   /* prompt baseline */
	feed("abc");
	feed_byte(0x01);                                /* home */
	cli_dummy_clear_output(&tr);
	feed_byte(0x0B);                                /* Ctrl+k from home: kill whole line */
	assert(cap_has("\x1b[2K"));                     /* the refresh cleared the row */
	assert(sh.len == 0);
}

int main(void)
{
	test_cursor_moves();
	test_insert_and_overwrite();
	test_delete_and_backspace();
	test_kill_and_word();
	test_invalid_escapes_ignored();
	test_cpr_width();
	test_session_start_probe();
	test_clear_screen();
	test_backspace_mode();
	test_wrap_rows();
	test_dispatch_resets_cursor();
	test_enter_moves_to_end();
	test_ctrl_c_baseline();
	printf("OK: cursor / insert / delete / kill / word / vt100 / cpr / wrap / dispatch pass\n");
	return 0;
}
