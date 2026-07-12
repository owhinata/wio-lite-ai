/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    cli_session.c
 * @brief   Shell line pipeline: prompt + line dispatch.
 *
 * Pure dispatch logic over a `struct cli_instance`: it submits the accumulated
 * line to the parser, prints the outcome and returns to the prompt.  The byte
 * input / line-editing state machine that feeds the buffer lives in cli_edit.c
 * (issue #9); both files call no ThreadX (tx_*) API -- the thread loop, object
 * creation and ISR notify live in cli_core.c -- so they compile and unit-test on
 * the host against a small tx_api.h type shim (shell/test/shim).  All output
 * goes through the buffered output API (cli_write/cli_error, issue #5): echo,
 * prompt and dispatch messages are flow-controlled and (for errors) coloured.
 * Clean-room design inspired by Zephyr shell; no code reused.
 */
#include <stddef.h>
#include <stdio.h>      /* snprintf (usage command-path join, issue #37) */
#include <string.h>

#include "cli_instance.h"
#include "cli_internal.h"

/* Reset the per-session editor state on a transport (re)connect (issue #49 P4).
 * Mirrors the per-session subset of cli_init() (cli_core.c) but touches NO
 * transport/ThreadX/config field and produces NO output -- the caller
 * (cli_thread_entry, on CLI_EVT_CONN) follows it with the backend's session_begin
 * hook and then cli_edit_session_start() to draw the fresh prompt.  Kept here in
 * the ThreadX-free editor/session layer so the host test harness sees it too. */
void cli_session_reset_state(struct cli_instance *sh)
{
	sh->len            = 0;
	sh->cur            = 0;
	sh->line[0]        = '\0';
	sh->overwrite      = 0;
	sh->rx             = CLI_RX_NORMAL;
	sh->prev_cr        = 0;
	sh->esc_np         = 0;
	sh->esc_bad        = 0;
	sh->esc_p[0]       = 0;
	sh->esc_p[1]       = 0;
	sh->hist_used      = 0;        /* no history leak across clients              */
	sh->hist_nav_on    = 0;
	sh->hist_nav       = 0;
	sh->term_width     = CLI_TERM_WIDTH;   /* re-probed by cli_edit_session_start */
	sh->old_rows       = 0;
	sh->draw_row       = 0;
	sh->probing_cpr    = 0;
	sh->tab_list_armed = 0;
	sh->render_dirty   = 0;
	sh->out_len        = 0;
	sh->tx_failed      = 0;
	sh->dispatching    = 0;
	sh->cancel_req     = 0;
	/* Intentionally NOT touched: tr/sh/state/prompt/ThreadX objects/fg (lifecycle
	 * & binding), bs_swap (a setting), last_result/rx_dropped/tx_dropped (stats). */
}

void cli_prompt(struct cli_instance *sh)
{
	cli_write(sh, sh->prompt, strlen(sh->prompt));
}

int cli_cancel_poll(struct cli_instance *sh)
{
	/* A bg-job worker (issue #25) does NOT own the transport RX: that ring is a
	 * strict SPSC pipe whose single consumer is the interactive shell thread.
	 * Draining it here would steal the user's keystrokes and corrupt the ring, so
	 * a job never reads RX -- its cancel is kill-driven (`kill %N` sets cancel_req
	 * directly), and this just reports that sticky flag. */
	if (sh->fg)
		return sh->cancel_req;

	struct cli_transport *tr = sh->tr;
	uint8_t buf[CLI_RX_DRAIN_CHUNK];
	int n;

	/* Drain everything currently buffered; a 0x03 anywhere latches the cancel.
	 * Non-0x03 bytes are discarded -- type-ahead typed during a running command
	 * is dropped by design (issue #16). */
	while ((n = tr->api->read(tr, buf, sizeof buf)) > 0)
		for (int i = 0; i < n; i++)
			if (buf[i] == 0x03)
				sh->cancel_req = 1;

	return sh->cancel_req;
}

bool cli_cancel_requested(struct cli_instance *sh)
{
	return cli_cancel_poll(sh) != 0;
}

/*
 * Parse and run ONE command segment (issue #23): a single ';'-separated piece of
 * the input line.  Mirrors the original single-command body of cli_dispatch_line
 * -- it resets the per-command flow-control flag, parses @p seg in place and
 * dispatches the resolved handler / prints the parse outcome.  Cross-segment
 * concerns (the line's "\r\n" echo, history, the cooperative-cancel feedback and
 * the prompt) stay in cli_dispatch_line; this runs once per segment.  Exposed
 * (issue #25) so a background-job worker can run its single segment through the
 * exact same parse/dispatch path as the interactive shell.
 */
void cli_dispatch_segment(struct cli_instance *sh, char *seg)
{
	sh->tx_failed = 0;                  /* fresh flow-control state per command */

	enum cli_parse_status st =
		cli_parse(seg, sh->argv, CLI_ARGV_CAP, &sh->pr);

	switch (st) {
	case CLI_PARSE_OK: {
		/* Pass the handler-relative argv/argc (argv[0] = leaf name).  Mark the
		 * window in which a Ctrl+C cancels the command (issue #16): cancel_req is
		 * cleared first, dispatching gates the cooperative-cancel paths.  For a bg
		 * job (#25) do NOT clear it: cancel_req was already reset at launch, and a
		 * `kill` may have landed BEFORE the worker reached here (the fg, at higher
		 * priority, can process `kill %N` from type-ahead before the worker runs) --
		 * clearing it would drop that kill. */
		if (!sh->fg)
			sh->cancel_req = 0;
		sh->dispatching = 1;
		int ret = sh->pr.cmd->handler(sh, sh->pr.argc, sh->pr.argv);
		sh->dispatching = 0;
		/* Force a non-zero result when output was dropped (TX timeout),
		 * even if the handler ignored cli_print's return value (req §11). */
		sh->last_result = (ret == 0 && sh->tx_failed) ? CLI_DISPATCH_ERR : ret;
		break;
	}
	case CLI_PARSE_EMPTY:
		break;                          /* blank segment: skip (req §23) */
	case CLI_PARSE_NOT_FOUND:
		cli_error(sh, "%s: command not found\r\n", sh->pr.argv[0]);
		sh->last_result = CLI_DISPATCH_ERR;
		break;
	case CLI_PARSE_NO_HANDLER:
		cli_error(sh, "%s: missing or unknown subcommand\r\n", sh->pr.argv[0]);
		sh->last_result = CLI_DISPATCH_ERR;
		break;
	case CLI_PARSE_WRONG_ARGS:
		cli_error(sh, "%s: invalid number of arguments\r\n", sh->pr.argv[0]);
		/* Issue #37: follow with the command's usage -- its full command path
		 * (sh->argv[0 .. cmd_level-1], populated by cli_parse before WRONG_ARGS)
		 * plus its one-line .help reused as usage.  Built into one buffer and
		 * emitted with a single cli_print so a background-job line cannot splice
		 * into the middle of the usage line (each cli_print is its own TX lock). */
		if (sh->pr.cmd != NULL && sh->pr.cmd->help != NULL) {
			char path[CLI_CMD_BUFFER_SIZE];
			size_t n = 0;

			path[0] = '\0';
			for (int i = 0; i < sh->pr.cmd_level && n < sizeof path; i++)
				n += (size_t)snprintf(path + n, sizeof path - n,
				                      i ? " %s" : "%s", sh->argv[i]);
			cli_print(sh, "usage: %s  (%s)\r\n", path, sh->pr.cmd->help);
		}
		sh->last_result = CLI_DISPATCH_ERR;
		break;
	case CLI_PARSE_TOO_MANY_TOKENS:
		cli_error(sh, "too many arguments\r\n");
		sh->last_result = CLI_DISPATCH_ERR;
		break;
	case CLI_PARSE_NESTING_TOO_DEEP:
		cli_error(sh, "command nesting too deep\r\n");
		sh->last_result = CLI_DISPATCH_ERR;
		break;
	case CLI_PARSE_UNTERMINATED_QUOTE:
		cli_error(sh, "unterminated quote\r\n");
		sh->last_result = CLI_DISPATCH_ERR;
		break;
	default:
		sh->last_result = CLI_DISPATCH_ERR;
		break;
	}
}

void cli_dispatch_line(struct cli_instance *sh)
{
	sh->tx_failed = 0;                  /* clean state for the leading "\r\n" echo */
	cli_write(sh, "\r\n", 2);           /* echo the newline before any output */
	sh->line[sh->len] = '\0';

	/* Reap any background jobs that finished since the last line (issue #25):
	 * delete their completed worker threads and print "[id] Done/Killed cmd" here
	 * on the foreground thread (so the notice serialises with the prompt), on the
	 * fresh line the "\r\n" echo just opened. */
	cli_jobs_reap(sh);

	/* Record the submitted line for history (issue #10).  Done BEFORE the split
	 * loop, which (via cli_next_segment + cli_parse) rewrites sh->line in place:
	 * the WHOLE line -- ';' separators included -- is one history entry. */
	if (sh->len > 0)
		cli_history_add(sh, sh->line);

	/* Run each ';'-separated segment in order (issue #23).  Errors do NOT stop
	 * the sequence (bash ';' semantics: continue on failure), but a cooperative
	 * Ctrl+C aborts the remaining segments -- the running handler latches
	 * sh->cancel_req, which the unchanged cancel block below then reports once. */
	char *cursor = sh->line;
	char *seg;
	while ((seg = cli_next_segment(&cursor)) != NULL) {
		/* Trailing '&' (issue #25): launch this segment as a background job and
		 * return to the prompt at once instead of running it inline.  cancel_req
		 * is the foreground line's, so it never gates a bg launch. */
		if (cli_segment_is_background(seg)) {
			cli_job_launch(sh, seg);
			continue;
		}
		cli_dispatch_segment(sh, seg);
		if (sh->cancel_req)
			break;
	}

	/* Cooperative Ctrl+C (issue #16): if the command observed a cancel, echo the
	 * editor-style "^C" feedback.  Clear cancel_req + tx_failed + the staging
	 * buffer FIRST (mirrors cli_edit.c's 0x03 path) so the feedback is not eaten
	 * by the cancel fast-fail or the tx_failed output-drop; drop any residual RX
	 * with a raw drain (NOT cli_cancel_poll, which would re-latch on the 0x03) so
	 * queued type-ahead does not auto-run. */
	if (sh->cancel_req) {
		uint8_t b;
		sh->cancel_req = 0;
		sh->tx_failed  = 0;
		sh->out_len    = 0;
		while (sh->tr->api->read(sh->tr, &b, 1) > 0)
			;                          /* discard residual RX */
		cli_write(sh, "^C\r\n", 4);
		sh->last_result = CLI_DISPATCH_CANCELLED;
	}

	/* Always return to the prompt; a bad command or a non-zero handler return
	 * never stops the shell, and one instance's error cannot reach another
	 * (fail-safe, req §9).  Reset the editor's cursor + render state so the
	 * fresh prompt is the new render baseline (issue #9); overwrite mode is
	 * intentionally kept across commands (session-wide, like the terminal). */
	sh->len = 0;
	sh->cur = 0;
	sh->line[0] = '\0';
	sh->rx = CLI_RX_NORMAL;
	/* Leave history navigation unconditionally: cli_history_add only runs for a
	 * non-empty line, so a recalled line cleared to empty (or a blank submit)
	 * would otherwise leave hist_nav_on set and the next ↑ would resume from a
	 * stale offset instead of the newest entry (issue #10). */
	sh->hist_nav_on = 0;
	sh->hist_nav = 0;
	{
		unsigned w = sh->term_width ? sh->term_width : (unsigned)CLI_TERM_WIDTH;
		sh->old_rows = (uint8_t)((unsigned)strlen(sh->prompt) / w + 1);
		sh->draw_row = 0;
	}
	cli_prompt(sh);
}
