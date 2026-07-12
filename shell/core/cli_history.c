/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    cli_history.c
 * @brief   Command history -- fixed byte ring (issue #10, req §8).
 *
 * Entries are the lines the user submits, packed oldest->newest into the
 * per-instance buffer sh->hist[0..hist_used), each '\0'-terminated.  Adding a
 * line evicts the oldest entries FIFO until the new one fits; consecutive
 * duplicates are dropped.  There is no dynamic allocation -- the ring is a fixed
 * CLI_HISTORY_BUFFER_SIZE array inside struct cli_instance, so each instance's
 * history is independent (req §10).
 *
 * Concurrency: cli_history_* run only in the instance's shell thread context.
 * The RX ISR merely sets an event flag; cli_input_byte() (which routes ↑/↓ and
 * Ctrl+p/n here) and cli_dispatch_line() (which calls add) both execute on the
 * instance thread (cli_core.c), so no locking is needed here.  Recall repaints
 * the line through cli_edit_redraw(), which takes the same TX lock the editor
 * already uses.
 *
 * The line editor's call sites (ESC[A/B, ESC O A/B, Ctrl+p/n) and the
 * dispatch-time add are unchanged from #9; navigation state is reset by the
 * dispatcher and the Ctrl+C handler, not here.
 *
 * Clean-room design; no third-party code reused.
 */
#include <string.h>

#include "cli_instance.h"
#include "cli_internal.h"

/*
 * Offset of the entry that immediately follows offset @p off (skip one
 * '\0'-terminated entry).  Bounded by hist_used: a corrupted buffer with a
 * missing terminator can never scan past the end -- it returns hist_used.
 */
static uint16_t hist_next_off(const struct cli_instance *sh, uint16_t off)
{
	uint16_t i = off;

	while (i < sh->hist_used && sh->hist[i] != '\0')
		i++;
	return (uint16_t)(i < sh->hist_used ? i + 1 : sh->hist_used);
}

/* Offset of the newest (last) entry; caller guarantees hist_used > 0. */
static uint16_t hist_newest_off(const struct cli_instance *sh)
{
	uint16_t start = 0, next;

	while ((next = hist_next_off(sh, start)) < sh->hist_used)
		start = next;
	return start;
}

/* Offset of the entry one older than the one at @p off; caller guarantees
 * off > 0 (i.e. @p off is not already the oldest entry). */
static uint16_t hist_prev_off(const struct cli_instance *sh, uint16_t off)
{
	uint16_t start = 0, next;

	while ((next = hist_next_off(sh, start)) < off)
		start = next;
	return start;
}

/* Recall the entry at @p off into the line buffer, cursor at its end.  The entry
 * length is < CLI_CMD_BUFFER_SIZE by construction (add rejects longer lines), so
 * the copy can never overflow sh->line. */
static void hist_load(struct cli_instance *sh, uint16_t off)
{
	size_t l = strlen(&sh->hist[off]);

	memcpy(sh->line, &sh->hist[off], l + 1);
	sh->len = (uint16_t)l;
	sh->cur = (uint16_t)l;
}

void cli_history_prev(struct cli_instance *sh)
{
	uint16_t target;

	if (sh->hist_used == 0)
		return;                          /* nothing recorded: silent no-op */

	if (!sh->hist_nav_on)
		target = hist_newest_off(sh);    /* first ↑: jump to the newest entry */
	else if (sh->hist_nav == 0)
		return;                          /* already at the oldest: stay put */
	else
		target = hist_prev_off(sh, sh->hist_nav);

	sh->hist_nav = target;
	sh->hist_nav_on = 1;
	hist_load(sh, target);
	cli_edit_redraw(sh);
}

void cli_history_next(struct cli_instance *sh)
{
	uint16_t next;

	if (!sh->hist_nav_on)
		return;                          /* already on the live line */

	next = hist_next_off(sh, sh->hist_nav);
	if (next < sh->hist_used) {
		sh->hist_nav = next;
		hist_load(sh, next);             /* recall the newer entry */
	} else {
		/* Past the newest entry: return to an empty live line (MVP -- the draft
		 * typed before the first ↑ is not restored). */
		sh->hist_nav_on = 0;
		sh->len = 0;
		sh->cur = 0;
		sh->line[0] = '\0';
	}
	cli_edit_redraw(sh);
}

void cli_history_add(struct cli_instance *sh, const char *line)
{
	size_t l = strlen(line);

	/* Skip empties and anything that can't be recalled safely: a line longer
	 * than the line buffer (would overflow on load) or larger than the whole
	 * ring (could never fit). */
	if (l == 0 || l >= CLI_CMD_BUFFER_SIZE || l + 1 > CLI_HISTORY_BUFFER_SIZE)
		return;

	/* Consecutive-duplicate suppression (req §8): drop a line identical to the
	 * most recent entry. */
	if (sh->hist_used > 0 &&
	    strcmp(&sh->hist[hist_newest_off(sh)], line) == 0)
		return;

	/* Evict oldest entries FIFO until the new one fits.  hist_next_off(0) is the
	 * byte length of the oldest entry and is bounded, so a corrupted buffer can
	 * only shrink hist_used (never loop forever / scan out of range). */
	while ((size_t)sh->hist_used + l + 1 > CLI_HISTORY_BUFFER_SIZE) {
		uint16_t first = hist_next_off(sh, 0);
		memmove(sh->hist, &sh->hist[first],
			(size_t)(sh->hist_used - first));
		sh->hist_used = (uint16_t)(sh->hist_used - first);
	}

	memcpy(&sh->hist[sh->hist_used], line, l);
	sh->hist[sh->hist_used + l] = '\0';
	sh->hist_used = (uint16_t)(sh->hist_used + l + 1);
}
