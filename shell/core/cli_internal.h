/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    cli_internal.h
 * @brief   Shell core-internal API (not part of the public cli.h surface).
 *
 * Currently holds the command-line parser: tokenizer, static subcommand-tree
 * search and argc/argv validation (issue #3).  The parser is a pure function of
 * the input line plus the registered command tree -- it does not touch a
 * `struct cli_instance` and never invokes a handler.  It resolves the command
 * and the handler-relative argc/argv and reports a status; the core (#4) does
 * the actual dispatch and message printing.
 */
#ifndef CLI_INTERNAL_H
#define CLI_INTERNAL_H

#include "cli.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Minimum capacity of the argv array the core (#4) must provide to cli_parse():
 * up to CLI_MAX_ARGC normal tokens, plus one RAW tail argument, plus the NULL
 * sentinel written at argv[argc].
 */
#define CLI_ARGV_CAP (CLI_MAX_ARGC + 2)

/** Outcome of cli_parse(). */
enum cli_parse_status {
	CLI_PARSE_OK = 0,            /**< cmd has a handler and args validate: dispatch it */
	CLI_PARSE_EMPTY,             /**< blank line (no tokens) */
	CLI_PARSE_NOT_FOUND,         /**< argv[0] matched no root command */
	CLI_PARSE_NO_HANDLER,        /**< resolved a parent that has no handler */
	CLI_PARSE_WRONG_ARGS,        /**< argc outside [mandatory, mandatory+optional] */
	CLI_PARSE_TOO_MANY_TOKENS,   /**< more than CLI_MAX_ARGC tokens */
	CLI_PARSE_NESTING_TOO_DEEP,  /**< subcommand descent exceeded CLI_MAX_SUBCMD_DEPTH */
	CLI_PARSE_UNTERMINATED_QUOTE,/**< a quote was opened but never closed */
};

/** Result of cli_parse(); see the per-status contract below. */
struct cli_parse_result {
	const struct cli_cmd *cmd;   /**< resolved command (leaf or parent), or NULL */
	char                **argv;  /**< handler argv (into caller array; argv[0]=leaf name, argv[argc]=NULL) */
	int                   argc;  /**< handler argc */
	int                   cmd_level; /**< number of command-path tokens (root included) */
};

/**
 * Tokenize @p line in place into @p argv (RAW handling is NOT applied; this is
 * the bare splitter, exposed mainly for unit tests).  Quote (`"`/`'`) and
 * backslash escapes are processed; empty quoted tokens ("" / '') are kept as
 * empty-string arguments.  At most @p max_argc tokens are stored and, on
 * success, argv[argc] is set to NULL -- so @p argv must hold max_argc+1 slots.
 *
 * @return the token count (>=0), or a negative value whose magnitude is the
 *         offending enum cli_parse_status (UNTERMINATED_QUOTE / TOO_MANY_TOKENS).
 */
int cli_tokenize(char *line, char **argv, int max_argc);

/**
 * Split @p line on top-level ';' for command sequencing (issue #23).  Returns
 * the next ';'-separated segment starting at *@p cursor, NUL-terminating it in
 * place (the ';' is overwritten with '\0') and advancing *@p cursor past it; a
 * ';' separates only OUTSIDE quotes and not escaped, using the SAME quote/escape
 * rules as the tokenizer (so `echo "a;b"` / `echo a\;b` stay one segment).  The
 * segment bytes are preserved verbatim (no compaction / quote stripping) so each
 * is parseable by cli_parse().  Returns NULL once exhausted (*@p cursor == NULL);
 * a trailing ';' yields one final empty segment first ("a;" -> "a","").  Empty /
 * whitespace-only segments parse as CLI_PARSE_EMPTY (a no-op) at dispatch.
 */
char *cli_next_segment(char **cursor);

/*
 * Detect a trailing background '&' on a single command segment (issue #25) and,
 * if present, strip it (and any whitespace before it) in place so the remaining
 * command re-tokenizes normally; returns 1 when @p seg was backgrounded, 0 when
 * left untouched.  Uses the same quote/escape rules as cli_next_segment(): a '&'
 * backgrounds only as the last non-whitespace char, outside quotes, unescaped,
 * and not as the tail of "&&".  Pure (no instance), so the host tests exercise it.
 */
int cli_segment_is_background(char *seg);

/**
 * Parse @p line: tokenize, walk the static command tree, handle RAW, and
 * validate the argument count.  @p argv is caller-provided scratch of capacity
 * @p argv_cap (must be >= CLI_ARGV_CAP).  @p out is zero-initialised first; on
 * return it is populated per the status (see cli_parse.c).
 */
enum cli_parse_status cli_parse(char *line, char **argv, int argv_cap,
                                struct cli_parse_result *out);

/*
 * Core line pipeline (issue #4), implemented in cli_session.c.  These functions
 * operate on a `struct cli_instance` but never call a ThreadX (tx_*) API, so the
 * translation unit compiles and unit-tests on the host (the ThreadX glue --
 * thread loop, object creation, ISR notify -- lives in cli_core.c instead).
 */

/** last_result value stored for any non-OK dispatch outcome (req §9: non-zero). */
#define CLI_DISPATCH_ERR (-1)

/** last_result when a running command was cancelled by Ctrl+C (issue #16). */
#define CLI_DISPATCH_CANCELLED (-2)

/*
 * Cooperative-cancel detection (issue #16, cli_session.c, ThreadX-free): drain
 * the transport's RX looking for 0x03 (Ctrl+C).  On a 0x03 it latches
 * sh->cancel_req and returns non-zero (sticky for the rest of the command);
 * non-0x03 bytes are discarded (type-ahead during a running command is dropped).
 * Called by cli_cancel_requested() and -- before each blocking wait -- by
 * cli_tx_send_blocking() / cli_sleep() (cli_core.c) so a 0x03 already buffered in
 * the ring (its CLI_EVT_RX flag possibly already consumed) is still seen.
 */
int cli_cancel_poll(struct cli_instance *sh);

/** Feed one received byte through the ASCII filter + RX/edit state machine (cli_edit.c). */
void cli_input_byte(struct cli_instance *sh, uint8_t b);

/**
 * Begin an interactive session on @p sh (cli_edit.c): kick off the terminal-width
 * probe (CPR) and draw the first prompt with the cursor correctly placed even if
 * the terminal never answers.  Called by the instance thread instead of a bare
 * cli_prompt() (issue #9).
 */
void cli_edit_session_start(struct cli_instance *sh);

/* Reset the per-session editor state (line / cursor / history / escape parse /
 * render / output-staging / cancel) WITHOUT producing any output (issue #49 P4).
 * The instance thread calls this on a transport (re)connect before
 * cli_edit_session_start() so the new session starts clean -- no previous line,
 * history or render state leaks across a reconnect.  ThreadX-free.  Excludes the
 * config (bs_swap/prompt), the cumulative stats and the lifecycle/transport
 * fields. */
void cli_session_reset_state(struct cli_instance *sh);

/** Set the backspace mode at run time (issue #9): 0 = DEL erases backward,
 *  1 = DEL (0x7F) deletes forward.  See CLI_BACKSPACE_MODE. */
void cli_set_backspace_mode(struct cli_instance *sh, int mode);

/** Parse and run the accumulated line, print the outcome, return to the prompt. */
void cli_dispatch_line(struct cli_instance *sh);

/** Parse and run ONE command segment in place (issue #23/#25): used per-segment
 *  by cli_dispatch_line and by a background-job worker for its single segment. */
void cli_dispatch_segment(struct cli_instance *sh, char *seg);

/** Emit the instance prompt. */
void cli_prompt(struct cli_instance *sh);

/**
 * Repaint the prompt + current line (cursor at sh->cur).  A thin non-static
 * wrapper over the editor's internal refresh, exported so cli_history.c can
 * redraw after recalling an entry into the line buffer (the refresh itself is
 * static to cli_edit.c).  Issue #10.
 */
void cli_edit_redraw(struct cli_instance *sh);

/**
 * Tab-completion (issue #11, cli_complete.c): complete the word ending at sh->cur against
 * the command set selected by the preceding tokens.  Single match -> insert the remainder
 * + a trailing space; multiple -> extend to the longest common prefix, then (bash-style,
 * on the next Tab) list the candidates; none -> BEL.  No allocation; the command tree is
 * scanned linearly (req §8).  Routed here by cli_edit.c on a 0x09 byte.
 */
void cli_tab_complete(struct cli_instance *sh);

/*
 * Command history hooks (cli_history.c).  The line editor routes up/down
 * (ESC[A/B, Ctrl+p/n) to prev/next and the dispatcher records each submitted
 * line through add; issue #10 implements the fixed ring here.  The call sites in
 * cli_edit.c / cli_session.c are unchanged -- the only addition #10 needs beyond
 * cli_history.c's body is the cli_edit_redraw export above (recall must repaint).
 */
void cli_history_prev(struct cli_instance *sh);  /**< recall older entry (↑ / Ctrl+p) */
void cli_history_next(struct cli_instance *sh);   /**< recall newer entry (↓ / Ctrl+n) */
void cli_history_add(struct cli_instance *sh, const char *line); /**< record a submitted line */

/*
 * Output plumbing (issue #5).  cli_printf.c (ThreadX-free) does the formatting
 * and 32 B staging and reaches the transport only through these three hooks,
 * which are implemented with ThreadX in cli_core.c (and stubbed in host tests):
 *   - cli_lock/cli_unlock: take/release the per-instance TX mutex around a whole
 *     output call so formatting + staging + flush are atomic (req §10).
 *   - cli_tx_send_blocking: push len bytes to the transport, blocking on TX
 *     space with a timeout; called only while the lock is held (req §11).
 * cli_lock returns 0 on success, <0 if the mutex could not be acquired.
 */
int  cli_lock(struct cli_instance *sh);
void cli_unlock(struct cli_instance *sh);
int  cli_tx_send_blocking(struct cli_instance *sh, const uint8_t *data, size_t len);

/*
 * Raw binary transfer in progress (issue #50): set by cli_console_claim() for the
 * whole duration of a YMODEM send and cleared by cli_console_release().  Two
 * effects: (1) cli_tx_send_blocking() stops draining RX / polling for Ctrl+C while
 * blocked on a full TX ring, so the transfer's own ACK/'C'/NAK bytes are never
 * eaten; (2) the _write retarget drops printf output (which would otherwise reach
 * g_uart_console unlocked from a non-shell thread / ISR and corrupt the stream).
 * Single global -- transfers are serialised by the output lock the claim holds.
 */
extern volatile uint8_t cli_xfer_active;

/*
 * Output bracket (issue #5 lock + issue #25 bg line-break).  cli_out_begin takes
 * the output lock (the FOREGROUND's for a bg-job worker, so its output serialises
 * against the fg line editor) and, for a job's FIRST output since the prompt was
 * drawn, emits a "\r\n" break and marks the fg's render dirty so the next
 * keystroke repaints prompt+line below the bg output.  Returns 0 locked, <0 if
 * the lock could not be taken (tx_failed set).  cli_out_end releases the lock.
 * The public output API (cli_print/cli_write/...) and the UART _write retarget
 * bracket every output call with this pair instead of raw cli_lock/cli_unlock.
 */
int  cli_out_begin(struct cli_instance *sh);
void cli_out_end(struct cli_instance *sh);

/* Staging primitives (cli_printf.c): append one byte (autoflush when full) and
 * flush the staging buffer through cli_tx_send_blocking. */
void cli_out_putc(struct cli_instance *sh, char c);
void cli_out_flush(struct cli_instance *sh);

#ifdef __cplusplus
}
#endif

#endif /* CLI_INTERNAL_H */
