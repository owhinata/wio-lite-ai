/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    cli_parse.c
 * @brief   Command-line parser: tokenizer, subcommand-tree search, validation.
 *
 * Pure functions over the input line and the registered command tree -- no
 * shell instance, no handler invocation, no heap.  Clean-room design inspired
 * by Zephyr shell's make_argv / tree descent / argc check; no code reused.
 */
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "cli_internal.h"

static bool is_space(char c)
{
	return c == ' ' || c == '\t';
}

/*
 * Extract the next token from *rdp into the buffer at *wrp, in place.
 *
 * Tokens are separated by spaces/tabs.  "..." processes backslash escapes and
 * keeps spaces literal; '...' is fully literal; outside quotes a backslash
 * escapes the next character (a trailing backslash is a literal backslash).
 * Quote characters are removed; each produced token is NUL-terminated.  The
 * write cursor never advances past the read cursor, so the still-unread tail of
 * the line stays pristine -- which is what lets RAW capture it verbatim.
 *
 * Returns 1 and sets *tok_out when a token is produced (possibly empty, e.g.
 * from ""), 0 at end of line, or a negative -enum cli_parse_status on error.
 */
static int next_token(char **rdp, char **wrp, char **tok_out)
{
	char *rd = *rdp;
	char *wr = *wrp;
	char quote = 0;             /* 0 = none, otherwise '"' or '\'' */

	while (is_space(*rd))
		rd++;
	if (*rd == '\0') {
		*rdp = rd;
		return 0;
	}

	char *tok = wr;
	for (;;) {
		char c = *rd;

		if (quote == 0) {
			if (c == '\0')
				break;
			if (is_space(c)) {
				rd++;       /* consume one separator */
				break;
			}
			if (c == '"' || c == '\'') {
				quote = c;
				rd++;
				continue;
			}
			if (c == '\\') {
				rd++;
				if (*rd == '\0') {
					*wr++ = '\\';   /* trailing backslash: literal */
					break;
				}
				*wr++ = *rd++;
				continue;
			}
			*wr++ = c;
			rd++;
		} else if (quote == '"') {
			if (c == '\0')
				return -(int)CLI_PARSE_UNTERMINATED_QUOTE;
			if (c == '"') {
				quote = 0;
				rd++;
				continue;
			}
			if (c == '\\') {
				rd++;
				if (*rd == '\0')
					return -(int)CLI_PARSE_UNTERMINATED_QUOTE;
				*wr++ = *rd++;
				continue;
			}
			*wr++ = c;
			rd++;
		} else { /* single quote: fully literal */
			if (c == '\0')
				return -(int)CLI_PARSE_UNTERMINATED_QUOTE;
			if (c == '\'') {
				quote = 0;
				rd++;
				continue;
			}
			*wr++ = c;
			rd++;
		}
	}

	*wr++ = '\0';
	*tok_out = tok;
	*rdp = rd;
	*wrp = wr;
	return 1;
}

/*
 * Split a line on top-level ';' (command sequencing, issue #23).
 *
 * Returns the next ';'-separated segment starting at *cursor, NUL-terminating it
 * in place (the delimiting ';' is overwritten with '\0') and advancing *cursor
 * past it.  A ';' counts as a separator only OUTSIDE quotes and not escaped, so
 * `echo "a;b"` / `echo 'a;b'` / `echo a\;b` stay a single segment.  The quote /
 * escape state machine below MUST stay byte-for-byte in lockstep with
 * next_token() above -- it tracks the SAME states but, unlike the tokenizer,
 * preserves every byte (no compaction, no quote stripping) so each returned
 * segment can be handed to cli_parse() and re-tokenized normally.  An unbalanced
 * quote (or trailing '\') just ends the segment at EOL; cli_parse() then reports
 * UNTERMINATED_QUOTE on it, exactly as for a single malformed line.
 *
 * Contract: returns NULL once exhausted (*cursor == NULL).  A trailing ';'
 * yields one final empty segment before exhaustion, so "a;" -> {"a",""}, "a;;b"
 * -> {"a","","b"}, ";a" -> {"","a"}; empty/whitespace-only segments are handled
 * by cli_parse() as CLI_PARSE_EMPTY (a no-op).
 */
char *cli_next_segment(char **cursor)
{
	char *p = *cursor;

	if (p == NULL)
		return NULL;

	char *start = p;
	char quote = 0;             /* 0 = none, otherwise '"' or '\'' */

	for (; *p != '\0'; p++) {
		char c = *p;

		if (quote == 0) {
			if (c == ';') {             /* top-level separator */
				*p = '\0';
				*cursor = p + 1;    /* may point at the final '\0' */
				return start;
			}
			if (c == '"' || c == '\'') {
				quote = c;
			} else if (c == '\\') {
				if (p[1] == '\0')
					break;      /* trailing backslash: literal, EOL */
				p++;                /* skip the escaped char */
			}
		} else if (quote == '"') {
			if (c == '"') {
				quote = 0;
			} else if (c == '\\') {
				if (p[1] == '\0')
					break;      /* unterminated; ends at EOL */
				p++;                /* skip the escaped char */
			}
		} else { /* single quote: fully literal, only '\'' closes */
			if (c == '\'')
				quote = 0;
		}
	}

	*cursor = NULL;                     /* end of line: last segment */
	return start;
}

/*
 * Detect (and strip) a trailing background '&' on a command segment (issue #25).
 *
 * Scans @p seg with the SAME quote/escape rules as cli_next_segment() /
 * next_token(): a '&' is the background operator only when it is the last
 * non-whitespace character of the segment AND lies OUTSIDE quotes, is not
 * backslash-escaped, and is not the tail of a "&&" pair.  On a match the '&' and
 * any whitespace before it are overwritten with '\0' in place (so the remaining
 * command re-tokenizes normally) and 1 is returned; otherwise @p seg is left
 * untouched and 0 is returned.  Only a TRAILING '&' backgrounds -- a '&' inside
 * a token (e.g. "a&b") stays a literal byte, matching the tokenizer.
 *
 *   "cmd &" / "cmd&" / "cmd & "  -> background, leaves "cmd"
 *   'echo "a&"' / "echo a\&"     -> NOT background (quoted / escaped '&')
 *   "cmd && x" / "cmd &&"        -> NOT background ('&&' is not the bg operator)
 *   "&"                          -> background an empty command (a dispatch no-op)
 */
int cli_segment_is_background(char *seg)
{
	char quote = 0;             /* 0 = none, otherwise '"' or '\'' */
	int  amp = -1;              /* index of the candidate trailing '&', or -1 */
	int  amp_after_amp = 0;     /* candidate '&' was immediately preceded by '&' */
	int  prev_sig_amp = 0;      /* previous significant char was a top-level '&' */

	for (int i = 0; seg[i] != '\0'; i++) {
		char c = seg[i];

		if (quote == 0) {
			if (c == ' ' || c == '\t')
				continue;               /* whitespace: not significant */
			if (c == '\\') {
				if (seg[i + 1] == '\0')
					break;          /* trailing backslash: literal, EOL */
				i++;                    /* escaped char: literal, not an operator */
				amp = -1;
				prev_sig_amp = 0;
				continue;
			}
			if (c == '"' || c == '\'') {
				quote = c;
				amp = -1;
				prev_sig_amp = 0;
				continue;
			}
			if (c == '&') {
				amp = i;
				amp_after_amp = prev_sig_amp;
				prev_sig_amp = 1;
			} else {
				amp = -1;
				prev_sig_amp = 0;
			}
		} else if (quote == '"') {
			if (c == '\\' && seg[i + 1] != '\0')
				i++;                    /* escaped char inside "..." */
			else if (c == '"')
				quote = 0;
			amp = -1;                       /* any quoted content resets the trailing '&' */
			prev_sig_amp = 0;
		} else { /* single quote: fully literal, only '\'' closes */
			if (c == '\'')
				quote = 0;
			amp = -1;
			prev_sig_amp = 0;
		}
	}

	if (amp < 0 || amp_after_amp)
		return 0;                           /* no trailing '&' (or it was '&&') */

	seg[amp] = '\0';                            /* drop the '&' ... */
	for (int j = amp - 1; j >= 0 && (seg[j] == ' ' || seg[j] == '\t'); j--)
		seg[j] = '\0';                      /* ... and the whitespace before it */
	return 1;
}

int cli_tokenize(char *line, char **argv, int max_argc)
{
	char *rd = line;
	char *wr = line;
	char *tok;
	int argc = 0;

	for (;;) {
		int t = next_token(&rd, &wr, &tok);
		if (t == 0)
			break;
		if (t < 0)
			return t;           /* negative -enum cli_parse_status */
		if (argc >= max_argc)
			return -(int)CLI_PARSE_TOO_MANY_TOKENS;
		argv[argc++] = tok;
	}
	argv[argc] = NULL;
	return argc;
}

static const struct cli_cmd *match_root(const char *name)
{
	CLI_ROOT_CMD_FOREACH(c) {
		if (c->name != NULL && strcmp(c->name, name) == 0)
			return c;
	}
	return NULL;
}

static const struct cli_cmd *match_sub(const struct cli_cmd *set, const char *name)
{
	for (; set->name != NULL; ++set) {
		if (strcmp(set->name, name) == 0)
			return set;
	}
	return NULL;
}

enum cli_parse_status cli_parse(char *line, char **argv, int argv_cap,
                                struct cli_parse_result *out)
{
	memset(out, 0, sizeof(*out));

	char *rd = line;
	char *wr = line;
	char *tok;
	int argc = 0;

	/* Token 0: must match a root command. */
	int t = next_token(&rd, &wr, &tok);
	if (t == 0)
		return CLI_PARSE_EMPTY;
	if (t < 0)
		return (enum cli_parse_status)(-t);
	argv[argc++] = tok;

	const struct cli_cmd *cmd = match_root(tok);
	if (cmd == NULL) {
		argv[1] = NULL;
		out->argv = argv;
		out->argc = 1;
		return CLI_PARSE_NOT_FOUND;
	}

	int cmd_level = 1;

	/* Descend the static tree while the current node has subcommands. */
	while (cmd->subcmds != NULL) {
		t = next_token(&rd, &wr, &tok);
		if (t == 0)
			break;              /* parent invoked with no further token */
		if (t < 0)
			return (enum cli_parse_status)(-t);
		if (argc >= CLI_MAX_ARGC)
			return CLI_PARSE_TOO_MANY_TOKENS;
		argv[argc++] = tok;

		const struct cli_cmd *sub = match_sub(cmd->subcmds, tok);
		if (sub != NULL) {
			if ((cmd_level - 1) >= CLI_MAX_SUBCMD_DEPTH)
				return CLI_PARSE_NESTING_TOO_DEEP;
			cmd = sub;
			cmd_level++;
			continue;
		}
		break;                      /* argv[argc-1] is the first argument */
	}

	int handler_base = cmd_level - 1;

	/* A parent without a handler cannot run by itself. */
	if (cmd->handler == NULL) {
		argv[argc] = NULL;
		out->cmd = cmd;
		out->cmd_level = cmd_level;
		out->argv = &argv[handler_base];
		out->argc = argc - handler_base;
		return CLI_PARSE_NO_HANDLER;
	}

	bool is_raw = (cmd->optional == CLI_ARG_RAW) && (cmd->subcmds == NULL);

	if (is_raw) {
		/* Read the remaining mandatory (handler) tokens, then grab the tail. */
		int need = (cmd->mandatory > 0) ? (cmd->mandatory - 1) : 0;
		while (need > 0) {
			t = next_token(&rd, &wr, &tok);
			if (t == 0)
				break;      /* not enough -> WRONG_ARGS below */
			if (t < 0)
				return (enum cli_parse_status)(-t);
			if (argc >= CLI_MAX_ARGC)
				return CLI_PARSE_TOO_MANY_TOKENS;
			argv[argc++] = tok;
			need--;
		}
		if ((argc - handler_base) >= cmd->mandatory) {
			while (is_space(*rd))
				rd++;
			if (*rd != '\0') {
				if (argc >= argv_cap - 1)
					return CLI_PARSE_TOO_MANY_TOKENS;
				argv[argc++] = rd;  /* verbatim, NUL-terminated by line end */
			}
		}
	} else {
		/* Normal: every remaining token is an argument. */
		for (;;) {
			t = next_token(&rd, &wr, &tok);
			if (t == 0)
				break;
			if (t < 0)
				return (enum cli_parse_status)(-t);
			if (argc >= CLI_MAX_ARGC)
				return CLI_PARSE_TOO_MANY_TOKENS;
			argv[argc++] = tok;
		}
	}

	int handler_argc = argc - handler_base;
	argv[argc] = NULL;
	out->cmd = cmd;
	out->cmd_level = cmd_level;
	out->argv = &argv[handler_base];
	out->argc = handler_argc;

	if (handler_argc < cmd->mandatory)
		return CLI_PARSE_WRONG_ARGS;
	/* Only a genuine (leaf) RAW command drops the upper bound; CLI_ARG_RAW on a
	 * non-leaf is unsupported and falls back to a normal large optional count. */
	if (!is_raw && handler_argc > cmd->mandatory + cmd->optional)
		return CLI_PARSE_WRONG_ARGS;

	return CLI_PARSE_OK;
}
