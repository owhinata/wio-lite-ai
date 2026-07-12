/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    cli_complete.c
 * @brief   Tab completion for command / subcommand names (issue #11, req §2/§8/§18.4).
 *
 * cli_edit.c routes a Tab (0x09) here.  Completion is read-only over the input
 * line and the registered command tree -- it never mutates sh->line during the
 * search (unlike cli_parse.c, which tokenizes in place), allocates nothing, and
 * scans the tree linearly (req §8).  The word ending at sh->cur is matched
 * against the command set selected by the tokens that precede it: the root set
 * (.shell_root_cmds) when no token precedes, otherwise the subcommand set of the
 * resolved parent.  Arguments are not completable (struct cli_cmd carries no
 * per-argument metadata), so reaching argument territory is a silent BEL.
 *
 * Outcomes (req §18.4):
 *   - 0 candidates  -> BEL.
 *   - 1 candidate   -> insert the remainder + a trailing space.
 *   - >=2 candidates-> bash-style two-stage: the first Tab extends to the longest
 *                      common prefix; once nothing more can be extended a SECOND
 *                      consecutive Tab lists the candidates (sh->tab_list_armed,
 *                      reset by any non-Tab key in cli_input_byte).
 *
 * Like cli_edit.c/cli_session.c this file calls no ThreadX (tx_*) API; it reaches
 * the transport only through the buffered output primitives and cli_edit_redraw(),
 * so it compiles and host-unit-tests against the tx_api.h shim.
 *
 * Concurrency: runs only in the instance's shell thread (cli_core.c).  The
 * candidate-list block is emitted under a single TX lock (cli_lock + cli_out_putc
 * + cli_out_flush) so it does not interleave with another thread's output to the
 * same instance; the follow-up cli_edit_redraw() takes the lock again -- a seam
 * that is acceptable under the shell-thread-driven model (req §10).
 *
 * Known limitation: the read-only token walk does not re-apply quote/backslash
 * processing, so a quoted command path (e.g. "help") parses on submit but is not
 * completable.  Command names are bare C identifiers, so normal usage is fine.
 *
 * Clean-room design; no third-party code reused.
 */
#include <stddef.h>
#include <string.h>

#include "cli_instance.h"
#include "cli_internal.h"

/* A command set to complete against: either the root array [base, end) or a
 * sentinel-terminated (name == NULL) subcommand array starting at base. */
struct cmd_set {
	const struct cli_cmd *base;
	const struct cli_cmd *end;   /* root only; NULL for a subcommand set */
	int                   is_root;
};

/* ---- small helpers ------------------------------------------------------ */

static void bel(struct cli_instance *sh)
{
	char b = 0x07;
	cli_write(sh, &b, 1);
}

/* Effective terminal width (mirrors cli_edit.c's static edit_cols). */
static unsigned complete_cols(const struct cli_instance *sh)
{
	return sh->term_width ? sh->term_width : (unsigned)CLI_TERM_WIDTH;
}

/* "ESC[<n><final>" via the staged-output path (caller holds the TX lock).
 * Local twin of cli_edit.c's static e_csi_n -- that one is not visible here. */
static void c_csi_n(struct cli_instance *sh, unsigned n, char final)
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

/* Start of the non-space run ending at the cursor (the word being completed).
 * Unlike cli_edit.c's word_start it does NOT also skip leading spaces. */
static unsigned complete_word_start(const struct cli_instance *sh)
{
	unsigned i = sh->cur;

	while (i > 0 && sh->line[i - 1] != ' ' && sh->line[i - 1] != '\t')
		i--;
	return i;
}

/* Read-only whitespace splitter over line[*pos .. limit).  On a token, sets the
 * begin/len out-params and advances *pos past it; returns 1.  Returns 0 at end. */
static int next_ro_token(const char *line, unsigned *pos, unsigned limit,
                         unsigned *begin, unsigned *len)
{
	unsigned i = *pos;

	while (i < limit && (line[i] == ' ' || line[i] == '\t'))
		i++;
	if (i >= limit) {
		*pos = i;
		return 0;
	}
	*begin = i;
	while (i < limit && line[i] != ' ' && line[i] != '\t')
		i++;
	*len = i - *begin;
	*pos = i;
	return 1;
}

/* Exact (length-bounded) name match against the root set / a subcommand set. */
static const struct cli_cmd *ro_match_root(const char *p, unsigned n)
{
	CLI_ROOT_CMD_FOREACH(c) {
		if (c->name != NULL && strlen(c->name) == n && memcmp(c->name, p, n) == 0)
			return c;
	}
	return NULL;
}

static const struct cli_cmd *ro_match_sub(const struct cli_cmd *set,
                                          const char *p, unsigned n)
{
	for (; set->name != NULL; ++set) {
		if (strlen(set->name) == n && memcmp(set->name, p, n) == 0)
			return set;
	}
	return NULL;
}

/* Iterate a command set, skipping NULL-name entries.  Returns the next entry at
 * or after *it (advancing *it past it), or NULL at the end. */
static const struct cli_cmd *set_next(const struct cmd_set *s,
                                      const struct cli_cmd **it)
{
	const struct cli_cmd *c = *it;

	for (;;) {
		if (s->is_root) {
			if (c >= s->end)
				return NULL;
		} else if (c->name == NULL) {
			return NULL;
		}
		if (c->name != NULL) {
			*it = c + 1;
			return c;
		}
		c++;
	}
}

/* ---- command-set resolution -------------------------------------------- */

/* Choose the set to complete the trailing word against by walking the tokens in
 * line[0 .. ws) (the command path before the word).  Returns 1 and fills *out, or
 * 0 when completion is impossible (unknown command, or argument territory). */
static int resolve_set(struct cli_instance *sh, unsigned ws, struct cmd_set *out)
{
	unsigned pos = 0, tb, tl;
	const struct cli_cmd *cmd = NULL;        /* NULL = still at the root level */

	while (next_ro_token(sh->line, &pos, ws, &tb, &tl)) {
		const struct cli_cmd *m = (cmd == NULL)
			? ro_match_root(&sh->line[tb], tl)
			: ro_match_sub(cmd->subcmds, &sh->line[tb], tl);
		if (m == NULL || m->subcmds == NULL)
			return 0;                /* unknown, or a leaf -> arguments: no completion */
		cmd = m;                         /* descend into m's subcommands */
	}

	if (cmd == NULL) {                       /* no preceding tokens -> root set */
		out->base    = __cli_root_cmds_start;
		out->end     = __cli_root_cmds_end;
		out->is_root = 1;
	} else {                                 /* complete against the parent's subcommands */
		out->base    = cmd->subcmds;
		out->end     = NULL;
		out->is_root = 0;
	}
	return 1;
}

/* ---- candidate scan ----------------------------------------------------- */

/* One linear pass: count prefix matches, remember the first (used when count==1)
 * and the longest common prefix length *beyond* the typed prefix (used to extend
 * on >=2).  No candidate storage (req §8). */
static unsigned scan_candidates(const struct cmd_set *set,
                                const char *prefix, unsigned plen,
                                const struct cli_cmd **first_out, unsigned *lcp_out)
{
	const struct cli_cmd *it = set->base, *c, *first = NULL;
	unsigned count = 0, lcp = 0;

	while ((c = set_next(set, &it)) != NULL) {
		if (strncmp(c->name, prefix, plen) != 0)
			continue;
		if (++count == 1) {
			first = c;
			lcp = (unsigned)strlen(c->name) - plen;   /* full tail of the first */
		} else {
			unsigned k = 0;
			while (k < lcp &&
			       c->name[plen + k] != '\0' &&
			       c->name[plen + k] == first->name[plen + k])
				k++;
			lcp = k;
		}
	}
	*first_out = first;
	*lcp_out = lcp;
	return count;
}

/* ---- line insertion ----------------------------------------------------- */

/* Insert @p n bytes from @p src at sh->cur, shifting the tail right.  BEL and
 * return 0 on overflow (same cap/contract as op_insert); 1 on success. */
static int insert_at_cursor(struct cli_instance *sh, const char *src, unsigned n)
{
	if (n == 0)
		return 1;
	if ((unsigned)sh->len + n > (unsigned)CLI_CMD_BUFFER_SIZE - 1) {
		bel(sh);
		return 0;
	}
	memmove(&sh->line[sh->cur + n], &sh->line[sh->cur],
	        (size_t)(sh->len - sh->cur));
	memcpy(&sh->line[sh->cur], src, n);
	sh->len = (uint16_t)(sh->len + n);
	sh->cur = (uint16_t)(sh->cur + n);
	sh->line[sh->len] = '\0';
	return 1;
}

/* ---- candidate list display -------------------------------------------- */

/* Print the prefix-matching names in width-aware columns on fresh lines below the
 * current (possibly wrapped) input, then repaint the prompt + line.  The input
 * line stays visible above the list (like a shell); sh->cur is preserved. */
static void show_list(struct cli_instance *sh, const struct cmd_set *set,
                      const char *prefix, unsigned plen)
{
	unsigned cols = complete_cols(sh);
	const struct cli_cmd *it, *c;
	unsigned longest = 0, colw, per_row, n;

	/* Pass 1: widest matching name -> column width. */
	it = set->base;
	while ((c = set_next(set, &it)) != NULL) {
		if (strncmp(c->name, prefix, plen) != 0)
			continue;
		unsigned l = (unsigned)strlen(c->name);
		if (l > longest)
			longest = l;
	}
	if (longest == 0)
		return;                          /* defensive: nothing to show */

	colw = longest + 2;
	if (colw > cols)
		colw = cols;                     /* an over-wide name still prints */
	per_row = cols / colw;
	if (per_row == 0)
		per_row = 1;

	if (cli_lock(sh) != 0) {
		sh->tx_failed = 1;
		return;
	}

	/* Move to the bottom row of the current render, then open a fresh line so the
	 * list lands below the whole (possibly wrapped) input -- like cli_dispatch_line. */
	if (sh->old_rows > 0) {
		unsigned down = (unsigned)(sh->old_rows - 1) - sh->draw_row;
		if (down)
			c_csi_n(sh, down, 'B');
	}
	cli_out_putc(sh, '\r');
	cli_out_putc(sh, '\n');

	/* Pass 2: print names, manually padded to colw (cli_printf has no %-*s). */
	n = 0;
	it = set->base;
	while ((c = set_next(set, &it)) != NULL) {
		unsigned l, k;
		if (strncmp(c->name, prefix, plen) != 0)
			continue;
		l = (unsigned)strlen(c->name);
		for (k = 0; k < l; k++)
			cli_out_putc(sh, c->name[k]);
		if (++n % per_row == 0) {
			cli_out_putc(sh, '\r');
			cli_out_putc(sh, '\n');
		} else {
			for (k = l; k < colw; k++)
				cli_out_putc(sh, ' ');
		}
	}
	if (n % per_row != 0) {                  /* finish a partial last row */
		cli_out_putc(sh, '\r');
		cli_out_putc(sh, '\n');
	}

	cli_out_flush(sh);
	cli_unlock(sh);

	/* The list scrolled the prompt baseline away; the cursor now sits at column 0
	 * of a fresh row, so repaint with no old render to erase (op_clear_screen tail). */
	sh->old_rows = 0;
	sh->draw_row = 0;
	cli_edit_redraw(sh);
}

/* ---- entry point -------------------------------------------------------- */

void cli_tab_complete(struct cli_instance *sh)
{
	unsigned ws = complete_word_start(sh);
	unsigned plen = (unsigned)sh->cur - ws;
	const char *prefix = &sh->line[ws];
	struct cmd_set set;
	const struct cli_cmd *first;
	unsigned lcp, count;

	if (!resolve_set(sh, ws, &set)) {
		sh->tab_list_armed = 0;
		bel(sh);                         /* argument territory / unknown command */
		return;
	}

	count = scan_candidates(&set, prefix, plen, &first, &lcp);

	if (count == 0) {
		sh->tab_list_armed = 0;
		bel(sh);
		return;
	}
	if (count == 1) {
		sh->tab_list_armed = 0;
		if (insert_at_cursor(sh, first->name + plen,
		                     (unsigned)strlen(first->name) - plen) &&
		    sh->cur == sh->len)          /* at end of line: add a separator space */
			insert_at_cursor(sh, " ", 1);
		cli_edit_redraw(sh);
		return;
	}

	/* count >= 2: bash-style two-stage (req §18.4). */
	{
		int extended = 0;

		if (lcp > 0)
			extended = insert_at_cursor(sh, first->name + plen, lcp);
		if (extended) {
			cli_edit_redraw(sh);
			sh->tab_list_armed = 1;  /* next Tab (now lcp==0) lists */
		} else if (sh->tab_list_armed) {
			show_list(sh, &set, prefix, plen);   /* second Tab: list, keep armed */
		} else {
			sh->tab_list_armed = 1;  /* first Tab at the common prefix: signal only */
			if (lcp == 0)            /* avoid a double BEL: a failed insert already BEL'd */
				bel(sh);
		}
	}
}
