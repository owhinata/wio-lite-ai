/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    cli_edit.c
 * @brief   Interactive line editor: RX/escape state machine, edit ops, redraw.
 *
 * Grows out of the minimal escape *swallower* that issue #4 kept in
 * cli_session.c: cli_input_byte() now drives a full VT100 line editor.  The
 * cursor (sh->cur) is split from the length (sh->len) so text can be inserted,
 * overwritten and deleted anywhere in the line; meta keys (Ctrl+a/b/d/e/f/k/u/w,
 * Alt+b/f, Ctrl+l) and the arrow / Home / End / Del / Insert escape sequences
 * move and edit it.  Like cli_session.c this file calls no ThreadX (tx_*) API --
 * it reaches the transport only through the buffered output primitives
 * (cli_lock / cli_out_putc / cli_out_flush, issue #5) -- so it compiles and
 * unit-tests on the host against the tx_api.h shim.
 *
 * Redraw model (req §2 "wrap at terminal width"): a single cli_edit_refresh()
 * repaints the line and repositions the cursor.  It never relies on the
 * terminal's auto-wrap behaviour at the last column -- it forces a newline at an
 * exact width boundary and tracks the render's row count (old_rows) and the
 * cursor's row (draw_row) itself, so wrapped lines redraw deterministically.
 * Two fast paths keep the common cases cheap (and byte-identical to issue #4):
 * appending at end of line echoes the one byte, and Backspace at end of line
 * emits "\b \b".  Terminal width is auto-detected via a CPR probe (see
 * cli_edit_session_start); until/without a reply CLI_TERM_WIDTH is used.
 *
 * Clean-room design: the multi-row redraw is informed by the *concept* of a
 * terminal line editor (as in GNU readline / linenoise / the Zephyr shell) but
 * reuses none of their code, comments or identifiers.
 */
#include <stddef.h>
#include <string.h>

#include "cli_instance.h"
#include "cli_internal.h"
#include "cli_vt100.h"

/* Effective terminal width: the probed value, or the configured default until a
 * CPR reply lands (also the value the host tests run with, since the dummy
 * backend never answers -- term_width stays 0 -> CLI_TERM_WIDTH). */
static unsigned edit_cols(const struct cli_instance *sh)
{
	return sh->term_width ? sh->term_width : (unsigned)CLI_TERM_WIDTH;
}

/* ---- staged-output helpers (caller holds the TX lock) ------------------- */

static void e_putn(struct cli_instance *sh, const char *s, size_t n)
{
	for (size_t i = 0; i < n; i++)
		cli_out_putc(sh, s[i]);
}

static void e_puts(struct cli_instance *sh, const char *s)
{
	while (*s)
		cli_out_putc(sh, *s++);
}

/* Emit "ESC[<n><final>" with n in decimal (n is small: rows/cols <= 255, or a
 * column up to the line length).  Built digit-by-digit -- no snprintf, no large
 * local buffer (keeps the instance thread stack flat, §15 verification). */
static void e_csi_n(struct cli_instance *sh, unsigned n, char final)
{
	char d[3];
	int  k = 0;

	cli_out_putc(sh, 0x1b);
	cli_out_putc(sh, '[');
	if (n == 0) {
		cli_out_putc(sh, '0');
	} else {
		while (n && k < (int)sizeof d) { d[k++] = (char)('0' + n % 10); n /= 10; }
		while (k > 0)
			cli_out_putc(sh, d[--k]);
	}
	cli_out_putc(sh, final);
}

/* ---- redraw ------------------------------------------------------------- *
 *
 * Symbols: cols = terminal width, pend = prompt_len + len (screen offset of the
 * line end), pcur = prompt_len + cur (offset of the cursor).  The render spans
 * rows 0..pend/cols (= pend/cols + 1 rows); the cursor's target row/col are
 * pcur/cols and pcur%cols.  draw_row holds the cursor's current row so the next
 * refresh knows where it is without trusting the terminal's wrap state.
 */

/* Full repaint + reposition.  Single, deterministic algorithm (req §2). */
static void cli_edit_refresh(struct cli_instance *sh)
{
	unsigned cols = edit_cols(sh);
	unsigned plen = (unsigned)strlen(sh->prompt);
	unsigned pend = plen + sh->len;
	unsigned pcur = plen + sh->cur;
	unsigned end_row = pend / cols;
	unsigned tgt_row = pcur / cols;
	unsigned tgt_col = pcur % cols;

	if (cli_lock(sh) != 0) {
		sh->tx_failed = 1;
		return;
	}

	/* 1. Erase the previous render (skip when there is none, e.g. first draw
	 *    or just after dispatch printed a fresh prompt). */
	if (sh->old_rows > 0) {
		unsigned down = (unsigned)(sh->old_rows - 1) - sh->draw_row;
		if (down)
			e_csi_n(sh, down, 'B');          /* go to the bottom row */
		for (unsigned r = sh->old_rows; r > 1; r--) {
			cli_out_putc(sh, '\r');
			e_puts(sh, CLI_VT100_CLR_LINE);
			e_csi_n(sh, 1, 'A');             /* clear this row, move up */
		}
		cli_out_putc(sh, '\r');
		e_puts(sh, CLI_VT100_CLR_LINE);          /* clear the top row */
	} else {
		cli_out_putc(sh, '\r');
	}

	/* 2. Paint prompt + line.  Force a newline at an exact width boundary so
	 *    the physical cursor is unambiguously at (end_row, col 0). */
	e_putn(sh, sh->prompt, plen);
	e_putn(sh, sh->line, sh->len);
	if (pend > 0 && pend % cols == 0)
		e_puts(sh, "\r\n");

	/* 3. Move the cursor from the line end (row end_row) back to the target. */
	if (end_row > tgt_row)
		e_csi_n(sh, end_row - tgt_row, 'A');
	cli_out_putc(sh, '\r');
	if (tgt_col)
		e_csi_n(sh, tgt_col, 'C');

	/* 4. Record the render invariants for the next refresh. */
	sh->old_rows = (uint8_t)(end_row + 1);
	sh->draw_row = (uint8_t)tgt_row;

	cli_out_flush(sh);
	cli_unlock(sh);
}

/* Public thin wrapper over the static refresh: lets cli_history.c (a separate
 * translation unit) repaint after recalling an entry into the line buffer.  The
 * caller sets sh->line / len / cur first; this redraws prompt + line (issue #10). */
void cli_edit_redraw(struct cli_instance *sh)
{
	cli_edit_refresh(sh);
}

/* Cursor-only move: reposition without repainting the line (cheaper, no flicker
 * for plain arrow / Home / End / word moves).  old_rows is unchanged. */
static void edit_reposition(struct cli_instance *sh)
{
	unsigned cols = edit_cols(sh);
	unsigned plen = (unsigned)strlen(sh->prompt);
	unsigned pcur = plen + sh->cur;
	unsigned cur_row = sh->draw_row;
	unsigned tgt_row = pcur / cols;
	unsigned tgt_col = pcur % cols;

	if (cli_lock(sh) != 0) {
		sh->tx_failed = 1;
		return;
	}
	if (cur_row > tgt_row)
		e_csi_n(sh, cur_row - tgt_row, 'A');
	else if (tgt_row > cur_row)
		e_csi_n(sh, tgt_row - cur_row, 'B');
	cli_out_putc(sh, '\r');
	if (tgt_col)
		e_csi_n(sh, tgt_col, 'C');
	sh->draw_row = (uint8_t)tgt_row;

	cli_out_flush(sh);
	cli_unlock(sh);
}

/* ---- fast paths (preserve the issue #4 byte stream for the common cases) - */

/* Append @p c at end of line with a single-byte echo, iff the cursor is at the
 * end and the new char stays within the current physical row (not landing on a
 * wrap boundary).  Returns 1 if handled. */
static int fast_append(struct cli_instance *sh, char c)
{
	unsigned cols = edit_cols(sh);
	unsigned pend = (unsigned)strlen(sh->prompt) + sh->len;

	if (sh->cur != sh->len || (pend + 1) % cols == 0)
		return 0;
	sh->line[sh->len++] = c;
	sh->line[sh->len] = '\0';
	sh->cur = sh->len;
	cli_write(sh, &c, 1);                            /* draw_row/old_rows unchanged */
	return 1;
}

/* Erase the char before the cursor with "\b \b", iff the cursor is at the end
 * and the erase does not cross a wrap boundary.  Returns 1 if handled. */
static int fast_backspace(struct cli_instance *sh)
{
	unsigned cols = edit_cols(sh);
	unsigned pend = (unsigned)strlen(sh->prompt) + sh->len;

	if (sh->cur != sh->len || sh->len == 0 || pend % cols == 0)
		return 0;
	sh->len--;
	sh->line[sh->len] = '\0';
	sh->cur = sh->len;
	cli_write(sh, "\b \b", 3);                       /* draw_row/old_rows unchanged */
	return 1;
}

/* ---- edit operations ---------------------------------------------------- */

static void op_insert(struct cli_instance *sh, char c)
{
	/* Overwrite mode replaces the char under the cursor (mid-line only). */
	if (sh->overwrite && sh->cur < sh->len) {
		sh->line[sh->cur++] = c;
		cli_edit_refresh(sh);
		return;
	}
	if (sh->len >= CLI_CMD_BUFFER_SIZE - 1) {        /* line full (req §8): BEL */
		char bel = 0x07;
		cli_write(sh, &bel, 1);
		return;
	}
	if (sh->cur == sh->len) {                        /* append */
		if (fast_append(sh, c))
			return;
		sh->line[sh->len++] = c;
		sh->line[sh->len] = '\0';
		sh->cur = sh->len;
		cli_edit_refresh(sh);
		return;
	}
	/* Insert in the middle: open a gap, then repaint the tail. */
	memmove(&sh->line[sh->cur + 1], &sh->line[sh->cur], sh->len - sh->cur);
	sh->line[sh->cur] = c;
	sh->len++;
	sh->cur++;
	sh->line[sh->len] = '\0';
	cli_edit_refresh(sh);
}

static void op_backspace(struct cli_instance *sh)
{
	if (sh->cur == 0)
		return;
	if (fast_backspace(sh))
		return;
	memmove(&sh->line[sh->cur - 1], &sh->line[sh->cur], sh->len - sh->cur);
	sh->cur--;
	sh->len--;
	sh->line[sh->len] = '\0';
	cli_edit_refresh(sh);
}

static void op_delete_fwd(struct cli_instance *sh)   /* Ctrl+d / Del */
{
	if (sh->cur >= sh->len)
		return;
	memmove(&sh->line[sh->cur], &sh->line[sh->cur + 1], sh->len - sh->cur - 1);
	sh->len--;
	sh->line[sh->len] = '\0';
	cli_edit_refresh(sh);
}

static void op_kill_eol(struct cli_instance *sh)     /* Ctrl+k */
{
	if (sh->cur >= sh->len)
		return;
	sh->len = sh->cur;
	sh->line[sh->len] = '\0';
	cli_edit_refresh(sh);
}

static void op_kill_bol(struct cli_instance *sh)     /* Ctrl+u: erase [0, cur) */
{
	if (sh->cur == 0)
		return;
	memmove(&sh->line[0], &sh->line[sh->cur], sh->len - sh->cur);
	sh->len = (uint16_t)(sh->len - sh->cur);
	sh->cur = 0;
	sh->line[sh->len] = '\0';
	cli_edit_refresh(sh);
}

/* Walk left over trailing spaces then a run of non-spaces: start of the word
 * that ends at @p from.  Shared by Ctrl+w (delete) and Alt+b (move). */
static unsigned word_start(const struct cli_instance *sh, unsigned from)
{
	unsigned i = from;
	while (i > 0 && sh->line[i - 1] == ' ')
		i--;
	while (i > 0 && sh->line[i - 1] != ' ')
		i--;
	return i;
}

/* Walk right over leading spaces then a run of non-spaces: end of the word at
 * or after @p from.  Used by Alt+f. */
static unsigned word_end(const struct cli_instance *sh, unsigned from)
{
	unsigned i = from;
	while (i < sh->len && sh->line[i] == ' ')
		i++;
	while (i < sh->len && sh->line[i] != ' ')
		i++;
	return i;
}

static void op_del_word(struct cli_instance *sh)     /* Ctrl+w */
{
	unsigned i;

	if (sh->cur == 0)
		return;
	i = word_start(sh, sh->cur);
	memmove(&sh->line[i], &sh->line[sh->cur], sh->len - sh->cur);
	sh->len = (uint16_t)(sh->len - (sh->cur - i));
	sh->cur = (uint16_t)i;
	sh->line[sh->len] = '\0';
	cli_edit_refresh(sh);
}

static void op_left(struct cli_instance *sh)
{
	if (sh->cur > 0) { sh->cur--; edit_reposition(sh); }
}

static void op_right(struct cli_instance *sh)
{
	if (sh->cur < sh->len) { sh->cur++; edit_reposition(sh); }
}

static void op_home(struct cli_instance *sh)
{
	if (sh->cur > 0) { sh->cur = 0; edit_reposition(sh); }
}

static void op_end(struct cli_instance *sh)
{
	if (sh->cur < sh->len) { sh->cur = sh->len; edit_reposition(sh); }
}

static void op_word_left(struct cli_instance *sh)    /* Alt+b */
{
	unsigned i = word_start(sh, sh->cur);
	if (i != sh->cur) { sh->cur = (uint16_t)i; edit_reposition(sh); }
}

static void op_word_right(struct cli_instance *sh)   /* Alt+f */
{
	unsigned i = word_end(sh, sh->cur);
	if (i != sh->cur) { sh->cur = (uint16_t)i; edit_reposition(sh); }
}

static void op_clear_screen(struct cli_instance *sh) /* Ctrl+l */
{
	if (cli_lock(sh) == 0) {
		e_puts(sh, CLI_VT100_CLR_SCREEN);        /* ESC[2J ESC[H */
		cli_out_flush(sh);
		cli_unlock(sh);
	} else {
		sh->tx_failed = 1;
	}
	sh->old_rows = 0;                                /* screen cleared: no old render */
	sh->draw_row = 0;
	cli_edit_refresh(sh);                            /* reprint prompt + line */
}

/* ---- escape-sequence finals -------------------------------------------- */

static void csi_final(struct cli_instance *sh, char f)
{
	switch (f) {
	case 'A': cli_history_prev(sh); break;           /* up    (#10 ring) */
	case 'B': cli_history_next(sh); break;           /* down  (#10 ring) */
	case 'C': op_right(sh); break;                   /* right */
	case 'D': op_left(sh); break;                    /* left  */
	case 'H': op_home(sh); break;                    /* Home  */
	case 'F': op_end(sh); break;                     /* End   */
	case '~':                                        /* ESC[<n>~ keypad/edit */
		switch (sh->esc_np >= 1 ? sh->esc_p[0] : 0) {
		case 1: case 7: op_home(sh); break;          /* Home */
		case 4: case 8: op_end(sh); break;           /* End */
		case 2: sh->overwrite ^= 1u; break;          /* Insert: toggle mode */
		case 3: op_delete_fwd(sh); break;            /* Delete */
		default: break;                              /* PgUp/PgDn/...: ignore */
		}
		break;
	case 'R':                                        /* CPR reply: ESC[<r>;<c>R */
		if (sh->probing_cpr && !sh->esc_bad && sh->esc_np == 2 &&
		    sh->esc_p[1] >= 20 && sh->esc_p[1] <= 255) {
			uint8_t w = (uint8_t)sh->esc_p[1];
			sh->probing_cpr = 0;
			if (w != sh->term_width) {
				sh->term_width = w;
				cli_edit_refresh(sh);            /* re-wrap (empty line: cheap) */
			}
		}
		break;                                       /* stray/invalid R: ignore §13 */
	default: break;                                  /* unknown final: ignore §13 */
	}
}

static void ss3_final(struct cli_instance *sh, char f)
{
	switch (f) {                                     /* ESC O <f>: app-mode keys */
	case 'A': cli_history_prev(sh); break;
	case 'B': cli_history_next(sh); break;
	case 'C': op_right(sh); break;
	case 'D': op_left(sh); break;
	case 'H': op_home(sh); break;
	case 'F': op_end(sh); break;
	default: break;
	}
}

/* Submit the line.  First move the physical cursor to the logical end of the
 * render: cli_dispatch_line() emits "\r\n" from wherever the cursor sits, so on
 * a wrapped line with the cursor above the bottom row that newline would land on
 * (and the command output overwrite) a row of the input.  Putting the cursor at
 * the end places it on the bottom row, so the dispatch newline opens a fresh
 * line below the whole input. */
static void edit_submit(struct cli_instance *sh)
{
	if (sh->cur != sh->len) {
		sh->cur = sh->len;
		edit_reposition(sh);
	}
	cli_dispatch_line(sh);
}

/* ---- byte input state machine ------------------------------------------ */

void cli_input_byte(struct cli_instance *sh, uint8_t b)
{
	/* Background-job output landed since the last keystroke (issue #25): a bg
	 * worker printed under this instance's tx_lock, broke to a fresh line and
	 * reset old_rows/draw_row/render_dirty.  Repaint the prompt + current input
	 * line here -- BEFORE any byte handling, including the fast-append path that
	 * would otherwise echo onto the bg-output line -- so the live line follows the
	 * bg output cleanly.  cli_edit_refresh() with old_rows == 0 just draws at the
	 * cursor (no erase).  A bg job runs below this instance's priority, so it only
	 * emits while this thread is idle: render_dirty/old_rows are settled here. */
	if (sh->render_dirty) {
		sh->render_dirty = 0;
		cli_edit_refresh(sh);
	}

	/* ASCII filter (req §13): non-ASCII bytes never reach the line buffer. */
	if (b & 0x80u)
		return;

	/* Tab completion is bash-style two-stage (issue #11): the candidate list is
	 * only shown on a SECOND consecutive Tab.  Any other key breaks the run, so
	 * disarm here -- 0x09 is the sole byte that preserves the armed state. */
	if (b != 0x09)
		sh->tab_list_armed = 0;

	/* Ctrl+C cancels the input line from ANY state, including a half-read
	 * escape sequence -- so a stuck terminal always recovers (req §9). */
	if (b == 0x03) {
		sh->rx = CLI_RX_NORMAL;
		sh->prev_cr = 0;
		sh->tx_failed = 0;
		cli_write(sh, "^C\r\n", 4);
		sh->len = 0;
		sh->cur = 0;
		sh->line[0] = '\0';
		sh->hist_nav_on = 0;        /* leave history navigation (issue #10) */
		/* The "^C\r\n" left the cursor on a fresh line; cli_prompt draws the
		 * prompt there, so set the render baseline to that prompt (NOT 0, or a
		 * later shortening refresh would skip the clear and leave stale text). */
		sh->old_rows = (uint8_t)((unsigned)strlen(sh->prompt) / edit_cols(sh) + 1);
		sh->draw_row = 0;
		cli_prompt(sh);
		return;
	}

	switch (sh->rx) {
	case CLI_RX_ESC:
		if (b == '[') {
			sh->rx = CLI_RX_CSI;
			sh->esc_np = 0;
			sh->esc_bad = 0;
			sh->esc_p[0] = 0;
			sh->esc_p[1] = 0;
		} else if (b == 'O') {
			sh->rx = CLI_RX_SS3;
		} else {
			sh->rx = CLI_RX_NORMAL;
			if (b == 'b')      op_word_left(sh);     /* Alt+b */
			else if (b == 'f') op_word_right(sh);    /* Alt+f */
			/* any other ESC <x>: ignore (req §13) */
		}
		return;

	case CLI_RX_CSI:
		if (b >= '0' && b <= '9') {
			if (sh->esc_np == 0)
				sh->esc_np = 1;
			sh->esc_p[sh->esc_np - 1] =
				(uint16_t)(sh->esc_p[sh->esc_np - 1] * 10u + (b - '0'));
			return;
		}
		if (b == ';') {
			if (sh->esc_np < 2)
				sh->esc_np++;                    /* start the next param */
			else
				sh->esc_bad = 1;                 /* a 3rd separator: malformed */
			return;
		}
		sh->rx = CLI_RX_NORMAL;
		if (b >= 0x40 && b <= 0x7E)
			csi_final(sh, (char)b);
		/* else: malformed CSI byte -> ignore (req §13) */
		return;

	case CLI_RX_SS3:
		sh->rx = CLI_RX_NORMAL;
		if (b >= 0x40 && b <= 0x7E)
			ss3_final(sh, (char)b);
		return;

	default:
		break;                                       /* CLI_RX_NORMAL */
	}

	/* CLI_RX_NORMAL.  Line endings first so prev_cr coalesces CR-LF into a
	 * single dispatch (dispatch resets the line but leaves prev_cr to us). */
	if (b == '\r') {
		edit_submit(sh);
		sh->prev_cr = 1;
		return;
	}
	if (b == '\n') {
		if (sh->prev_cr) {                           /* the LF half of CR-LF: swallow */
			sh->prev_cr = 0;
			return;
		}
		edit_submit(sh);
		return;
	}
	sh->prev_cr = 0;

	switch (b) {
	case 0x1B: sh->rx = CLI_RX_ESC; return;          /* ESC: begin sequence */
	case 0x01: op_home(sh);  return;                 /* Ctrl+a */
	case 0x02: op_left(sh);  return;                 /* Ctrl+b */
	case 0x04: op_delete_fwd(sh); return;            /* Ctrl+d */
	case 0x05: op_end(sh);   return;                 /* Ctrl+e */
	case 0x06: op_right(sh); return;                 /* Ctrl+f */
	case 0x0B: op_kill_eol(sh);  return;             /* Ctrl+k */
	case 0x0C: op_clear_screen(sh); return;          /* Ctrl+l */
	case 0x0E: cli_history_next(sh); return;         /* Ctrl+n (#10 ring) */
	case 0x10: cli_history_prev(sh); return;         /* Ctrl+p (#10 ring) */
	case 0x15: op_kill_bol(sh);  return;             /* Ctrl+u */
	case 0x17: op_del_word(sh);  return;             /* Ctrl+w */
	case 0x09: cli_tab_complete(sh); return;         /* Tab: completion (#11) */
	case 0x08:                                       /* Backspace */
		op_backspace(sh);
		return;
	case 0x7F:                                       /* DEL: backspace, or forward in swap mode */
		if (sh->bs_swap)
			op_delete_fwd(sh);
		else
			op_backspace(sh);
		return;
	default:
		break;
	}

	if (b >= 0x20 && b <= 0x7E) {                    /* printable: insert / overwrite */
		op_insert(sh, (char)b);
		return;
	}
	/* Any other control byte: ignore (req §13). */
}

/* ---- session start + config -------------------------------------------- */

void cli_edit_session_start(struct cli_instance *sh)
{
	/* Probe the terminal width: jump far right + request a position report.
	 * The reply (ESC[<r>;<c>R) arrives later through the RX path and only
	 * updates term_width -- so we MUST restore the visible cursor now, with a
	 * normal refresh, or an unanswering terminal would be left at the far
	 * right.  The line is empty here, so the refresh just draws the prompt. */
	sh->probing_cpr = 1;
	if (cli_lock(sh) == 0) {
		e_puts(sh, CLI_VT100_CPR_PROBE);
		cli_out_flush(sh);
		cli_unlock(sh);
	} else {
		sh->tx_failed = 1;
	}
	sh->old_rows = 0;
	sh->draw_row = 0;
	sh->cur = 0;
	cli_edit_refresh(sh);
}

void cli_set_backspace_mode(struct cli_instance *sh, int mode)
{
	sh->bs_swap = mode ? 1u : 0u;
}
