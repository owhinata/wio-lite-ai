/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 *
 * Host unit test for the command-line parser (issue #3): tokenizer quote/escape
 * rules, static subcommand-tree search, and argc/argv validation including RAW.
 *
 * Compiled together with cli_parse.c and small CLI_MAX_ARGC / CLI_MAX_SUBCMD_DEPTH
 * overrides (see run_host_tests.sh) so the token-limit and nesting-limit paths
 * can be exercised with a compact command tree.  Uses the #2 host_sections.ld to
 * supply the .shell_root_cmds section.
 */
#include <assert.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "cli.h"
#include "cli_internal.h"

static int h_ok(struct cli_instance *sh, int argc, char **argv)
{
	(void)sh;
	(void)argc;
	(void)argv;
	return 0;
}

/* deep -> a -> b -> c  (a, b are pure parents; c is a leaf) */
CLI_SUBCMD_SET_CREATE(sub_b, CLI_CMD_ARG(c, NULL, "leaf c", h_ok, 1, 0),
	CLI_SUBCMD_SET_END);
CLI_SUBCMD_SET_CREATE(sub_a, CLI_CMD_ARG(b, sub_b, "node b", NULL, 1, 0),
	CLI_SUBCMD_SET_END);
CLI_SUBCMD_SET_CREATE(sub_deep, CLI_CMD_ARG(a, sub_a, "node a", NULL, 1, 0),
	CLI_SUBCMD_SET_END);

/* thread { list, stacks }  (pure parent) */
CLI_SUBCMD_SET_CREATE(sub_thread,
	CLI_CMD_ARG(list,   NULL, "list",   h_ok, 1, 0),
	CLI_CMD_ARG(stacks, NULL, "stacks", h_ok, 1, 0),
	CLI_SUBCMD_SET_END);

/* mem { dump }  (parent that ALSO has its own handler) */
CLI_SUBCMD_SET_CREATE(sub_mem, CLI_CMD_ARG(dump, NULL, "dump", h_ok, 1, 2),
	CLI_SUBCMD_SET_END);

/* say now <raw...>  (RAW leaf under a parent, for handler-relative index test) */
CLI_SUBCMD_SET_CREATE(sub_say, CLI_CMD_ARG(now, NULL, "now raw", h_ok, 1, CLI_ARG_RAW),
	CLI_SUBCMD_SET_END);

CLI_CMD_REGISTER(version, NULL,       "version",  h_ok, 1, 0);
CLI_CMD_REGISTER(thread,  sub_thread, "thread",   NULL, 1, 0);
CLI_CMD_REGISTER(mem,     sub_mem,    "mem",      h_ok, 1, 2);
CLI_CMD_REGISTER(echo,    NULL,       "echo raw", h_ok, 2, CLI_ARG_RAW);
CLI_CMD_REGISTER(deep,    sub_deep,   "deep",     NULL, 1, 0);
CLI_CMD_REGISTER(say,     sub_say,    "say",      NULL, 1, 0);

static char buf[CLI_CMD_BUFFER_SIZE];
static char *argv_store[CLI_ARGV_CAP];
static char *targv[CLI_MAX_ARGC + 1];
static struct cli_parse_result res;

static enum cli_parse_status run(const char *s)
{
	strncpy(buf, s, sizeof(buf) - 1);
	buf[sizeof(buf) - 1] = '\0';
	return cli_parse(buf, argv_store, CLI_ARGV_CAP, &res);
}

static int run_tok(const char *s)
{
	strncpy(buf, s, sizeof(buf) - 1);
	buf[sizeof(buf) - 1] = '\0';
	return cli_tokenize(buf, targv, CLI_MAX_ARGC);
}

/* Collect every ';'-separated segment of a mutable copy of @p s into @p segv
 * (issue #23).  Returns the segment count; segments are NUL-terminated in place
 * and kept verbatim (only the ';' delimiters are overwritten). */
static char seg_buf[CLI_CMD_BUFFER_SIZE];
static int split_all(const char *s, char **segv, int cap)
{
	strncpy(seg_buf, s, sizeof(seg_buf) - 1);
	seg_buf[sizeof(seg_buf) - 1] = '\0';
	char *cur = seg_buf, *seg;
	int n = 0;
	while ((seg = cli_next_segment(&cur)) != NULL && n < cap)
		segv[n++] = seg;
	return n;
}

static void test_tokenizer(void)
{
	assert(run_tok("a b c") == 3);
	assert(strcmp(targv[0], "a") == 0 && strcmp(targv[1], "b") == 0 &&
	       strcmp(targv[2], "c") == 0);
	assert(targv[3] == NULL);                 /* NULL sentinel */

	assert(run_tok("  a   b  ") == 2);        /* leading/trailing/multiple spaces */
	assert(strcmp(targv[0], "a") == 0 && strcmp(targv[1], "b") == 0);

	assert(run_tok("\"a b\" c") == 2);        /* double quote keeps space */
	assert(strcmp(targv[0], "a b") == 0 && strcmp(targv[1], "c") == 0);

	assert(run_tok("'a b' c") == 2);          /* single quote keeps space */
	assert(strcmp(targv[0], "a b") == 0);

	assert(run_tok("a\\ b") == 1);            /* escaped space -> one token */
	assert(strcmp(targv[0], "a b") == 0);

	assert(run_tok("\"a\\\"b\"") == 1);       /* escape inside double quotes */
	assert(strcmp(targv[0], "a\"b") == 0);

	assert(run_tok("\"\" x") == 2);           /* empty quoted token kept */
	assert(targv[0][0] == '\0' && strcmp(targv[1], "x") == 0);
	assert(run_tok("''") == 1 && targv[0][0] == '\0');

	assert(run_tok("a\\") == 1);              /* trailing backslash -> literal */
	assert(strcmp(targv[0], "a\\") == 0);

	assert(run_tok("\"abc") == -(int)CLI_PARSE_UNTERMINATED_QUOTE);
	assert(run_tok("'abc") == -(int)CLI_PARSE_UNTERMINATED_QUOTE);

	/* 9 tokens with CLI_MAX_ARGC == 8 -> too many */
	assert(run_tok("t1 t2 t3 t4 t5 t6 t7 t8 t9") ==
	       -(int)CLI_PARSE_TOO_MANY_TOKENS);
	assert(run_tok("t1 t2 t3 t4 t5 t6 t7 t8") == 8);   /* exactly the max is OK */
}

static void test_search(void)
{
	assert(run("") == CLI_PARSE_EMPTY);

	assert(run("version") == CLI_PARSE_OK);
	assert(strcmp(res.cmd->name, "version") == 0);
	assert(res.argc == 1 && strcmp(res.argv[0], "version") == 0);
	assert(res.cmd_level == 1 && res.argv[res.argc] == NULL);

	assert(run("nope") == CLI_PARSE_NOT_FOUND);
	assert(res.cmd == NULL && res.argc == 1 && strcmp(res.argv[0], "nope") == 0);

	/* multi-level descent strips the parent path: argv[0] is the leaf */
	assert(run("thread list") == CLI_PARSE_OK);
	assert(strcmp(res.cmd->name, "list") == 0);
	assert(res.cmd_level == 2 && res.argc == 1 && strcmp(res.argv[0], "list") == 0);

	/* pure parent invoked alone / with an unknown subcommand -> NO_HANDLER */
	assert(run("thread") == CLI_PARSE_NO_HANDLER);
	assert(strcmp(res.cmd->name, "thread") == 0 && res.argc == 1);
	assert(run("thread bogus") == CLI_PARSE_NO_HANDLER);
	assert(strcmp(res.cmd->name, "thread") == 0);
	assert(res.argc == 2 && strcmp(res.argv[1], "bogus") == 0);

	/* parent WITH a handler: a non-matching token becomes its argument */
	assert(run("mem foo") == CLI_PARSE_OK);
	assert(strcmp(res.cmd->name, "mem") == 0);
	assert(res.argc == 2 && strcmp(res.argv[0], "mem") == 0 &&
	       strcmp(res.argv[1], "foo") == 0);
	assert(run("mem dump") == CLI_PARSE_OK && strcmp(res.cmd->name, "dump") == 0);

	/* nesting: depth limit is 2 (root + 2 subcommand steps) */
	assert(run("deep a b") == CLI_PARSE_NO_HANDLER);   /* b is a pure parent */
	assert(strcmp(res.cmd->name, "b") == 0 && res.cmd_level == 3);
	assert(run("deep a b c") == CLI_PARSE_NESTING_TOO_DEEP);
	assert(res.cmd == NULL);                           /* error -> zeroed out */
}

static void test_validate(void)
{
	assert(run("mem dump x y") == CLI_PARSE_OK);       /* dump: mand 1, opt 2 */
	assert(res.argc == 3 && strcmp(res.argv[0], "dump") == 0);
	assert(run("mem dump x y z") == CLI_PARSE_WRONG_ARGS);
	assert(strcmp(res.cmd->name, "dump") == 0);        /* out populated on error */
	assert(run("version x") == CLI_PARSE_WRONG_ARGS);

	/* token limit (CLI_MAX_ARGC == 8) fires before arg-count validation */
	assert(run("version 1 2 3 4 5 6 7 8 9") == CLI_PARSE_TOO_MANY_TOKENS);
}

static void test_raw(void)
{
	/* echo: mandatory 2 (echo + 1 token), then the rest verbatim */
	assert(run("echo hello world foo") == CLI_PARSE_OK);
	assert(strcmp(res.cmd->name, "echo") == 0 && res.argc == 3);
	assert(strcmp(res.argv[0], "echo") == 0 && strcmp(res.argv[1], "hello") == 0);
	assert(strcmp(res.argv[2], "world foo") == 0);     /* tail keeps the space */

	/* tail is verbatim: quotes and repeated spaces preserved */
	assert(run("echo k \"a b\"  c") == CLI_PARSE_OK);
	assert(strcmp(res.argv[2], "\"a b\"  c") == 0);

	/* empty tail -> no extra arg, just the mandatory tokens */
	assert(run("echo hello") == CLI_PARSE_OK && res.argc == 2);

	/* too few mandatory tokens -> WRONG_ARGS */
	assert(run("echo") == CLI_PARSE_WRONG_ARGS);

	/* RAW leaf under a parent: handler-relative index, argv[0] is the leaf */
	assert(run("say now hello there") == CLI_PARSE_OK);
	assert(strcmp(res.cmd->name, "now") == 0 && res.cmd_level == 2);
	assert(res.argc == 2 && strcmp(res.argv[0], "now") == 0);
	assert(strcmp(res.argv[1], "hello there") == 0);
}

/* issue #23: top-level ';' splitter -- quote/escape-aware, verbatim segments. */
static void test_segment(void)
{
	char *sv[8];

	/* no delimiter -> one verbatim segment */
	assert(split_all("thread", sv, 8) == 1);
	assert(strcmp(sv[0], "thread") == 0);

	/* basic split; surrounding spaces are preserved per segment (cli_parse
	 * later skips them) */
	assert(split_all("a;b", sv, 8) == 2);
	assert(strcmp(sv[0], "a") == 0 && strcmp(sv[1], "b") == 0);
	assert(split_all("a ; b", sv, 8) == 2);
	assert(strcmp(sv[0], "a ") == 0 && strcmp(sv[1], " b") == 0);

	/* a ';' inside quotes is NOT a delimiter (double and single) */
	assert(split_all("echo \"a;b\"", sv, 8) == 1);
	assert(strcmp(sv[0], "echo \"a;b\"") == 0);
	assert(split_all("echo 'a;b'", sv, 8) == 1);
	assert(strcmp(sv[0], "echo 'a;b'") == 0);

	/* an escaped ';' is NOT a delimiter (kept verbatim) */
	assert(split_all("echo a\\;b", sv, 8) == 1);
	assert(strcmp(sv[0], "echo a\\;b") == 0);

	/* leading / trailing / doubled ';' -> empty segments (parsed as EMPTY) */
	assert(split_all(";a", sv, 8) == 2);
	assert(sv[0][0] == '\0' && strcmp(sv[1], "a") == 0);
	assert(split_all("a;", sv, 8) == 2);
	assert(strcmp(sv[0], "a") == 0 && sv[1][0] == '\0');
	assert(split_all("a;;b", sv, 8) == 3);
	assert(strcmp(sv[0], "a") == 0 && sv[1][0] == '\0' &&
	       strcmp(sv[2], "b") == 0);

	/* unterminated quote -> whole remainder stays ONE segment (no inner split);
	 * cli_parse later reports UNTERMINATED_QUOTE on it */
	assert(split_all("echo \"a ; b", sv, 8) == 1);
	assert(strcmp(sv[0], "echo \"a ; b") == 0);
	assert(split_all("x ; 'a ; b", sv, 8) == 2);   /* split only before the open quote */
	assert(strcmp(sv[0], "x ") == 0 && strcmp(sv[1], " 'a ; b") == 0);

	/* trailing backslash at EOL is literal -> one segment */
	assert(split_all("echo a\\", sv, 8) == 1);
	assert(strcmp(sv[0], "echo a\\") == 0);

	/* escape-boundary agreement with next_token (codex NIT): an escaped
	 * backslash pair leaves the following ';' live */
	assert(split_all("a\\\\;b", sv, 8) == 2);
	assert(strcmp(sv[0], "a\\\\") == 0 && strcmp(sv[1], "b") == 0);
	/* a backslash inside single quotes is literal and does NOT escape the
	 * closing quote, so the ';' after the close is live */
	assert(split_all("'a\\';b", sv, 8) == 2);
	assert(strcmp(sv[0], "'a\\'") == 0 && strcmp(sv[1], "b") == 0);

	/* empty input -> a single empty segment */
	assert(split_all("", sv, 8) == 1 && sv[0][0] == '\0');
}

/* Run cli_segment_is_background() on a mutable copy of @p s: returns the bg flag
 * and leaves the (possibly '&'-stripped) segment in bg_buf for the caller to
 * compare (issue #25). */
static char bg_buf[CLI_CMD_BUFFER_SIZE];
static int is_bg(const char *s)
{
	strncpy(bg_buf, s, sizeof(bg_buf) - 1);
	bg_buf[sizeof(bg_buf) - 1] = '\0';
	return cli_segment_is_background(bg_buf);
}

static void test_background(void)
{
	/* trailing '&', with and without surrounding whitespace -> background; the
	 * '&' and the whitespace before it are stripped. */
	assert(is_bg("cmd &") == 1 && strcmp(bg_buf, "cmd") == 0);
	assert(is_bg("cmd&") == 1 && strcmp(bg_buf, "cmd") == 0);
	assert(is_bg("cmd & ") == 1 && strcmp(bg_buf, "cmd") == 0);
	assert(is_bg("sleep 30 &") == 1 && strcmp(bg_buf, "sleep 30") == 0);

	/* a lone '&' backgrounds an empty command (a dispatch no-op) */
	assert(is_bg("&") == 1 && bg_buf[0] == '\0');

	/* quoted / escaped '&' is a literal, NOT the operator */
	assert(is_bg("echo \"x&\"") == 0 && strcmp(bg_buf, "echo \"x&\"") == 0);
	assert(is_bg("echo 'a&'") == 0 && strcmp(bg_buf, "echo 'a&'") == 0);
	assert(is_bg("echo a\\&") == 0 && strcmp(bg_buf, "echo a\\&") == 0);
	assert(is_bg("cmd \\&") == 0);

	/* '&&' is not the background operator (logical-and is a non-goal) */
	assert(is_bg("cmd && x") == 0 && strcmp(bg_buf, "cmd && x") == 0);
	assert(is_bg("cmd &&") == 0);

	/* '&' inside a token (not trailing) stays a literal byte */
	assert(is_bg("a&b") == 0 && strcmp(bg_buf, "a&b") == 0);

	/* a '&' that ends the segment after an earlier '&&' still backgrounds */
	assert(is_bg("a && b &") == 1 && strcmp(bg_buf, "a && b") == 0);

	/* the segment splitter feeds one segment at a time: "cmd & ; next" splits on
	 * ';' first, so the bg check sees "cmd & " (-> bg "cmd") and " next" (-> not). */
	{
		char line[] = "cmd & ; next";
		char *cur = line, *seg;
		int n = 0;
		int bg0 = -1, bg1 = -1;
		char seg0[CLI_CMD_BUFFER_SIZE] = "", seg1[CLI_CMD_BUFFER_SIZE] = "";
		while ((seg = cli_next_segment(&cur)) != NULL) {
			int b = cli_segment_is_background(seg);
			if (n == 0) { bg0 = b; strcpy(seg0, seg); }
			else if (n == 1) { bg1 = b; strcpy(seg1, seg); }
			n++;
		}
		assert(n == 2);
		assert(bg0 == 1 && strcmp(seg0, "cmd") == 0);
		assert(bg1 == 0 && strcmp(seg1, " next") == 0);
	}
}

int main(void)
{
	test_tokenizer();
	test_search();
	test_validate();
	test_raw();
	test_segment();
	test_background();
	printf("OK: tokenizer / tree search / validation / RAW / segment / background all pass\n");
	return 0;
}
