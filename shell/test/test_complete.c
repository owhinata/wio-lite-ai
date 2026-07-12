/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 *
 * Host unit test for Tab completion (issue #11): the word-boundary scan, the
 * read-only command-set resolution (root vs subcommand, leading spaces, argument
 * territory), the prefix scan with longest-common-prefix tracking, the single-
 * candidate complete + trailing space, the bash-style two-stage candidate list,
 * BEL on no match, and the buffer-full guard (including that a too-long LCP extend
 * still reaches the list on the next Tab).
 *
 * It drives the ThreadX-free cli_complete.c through cli_input_byte() (Tab = 0x09)
 * and asserts the editor model (line / len / cur / tab_list_armed) plus captured
 * output fragments via the shared dummy backend + host_glue, exactly like
 * test_edit.c.  Colour OFF so BEL/escape bytes compare plainly.
 *
 * Two compile modes:
 *   - default                : full command tree, the main suite.
 *   - TEST_COMPLETE_SMALL_BUF: CLI_CMD_BUFFER_SIZE=8 + long names, the overflow
 *                              cases (run by run_host_tests.sh as a second build).
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

static int h_ok(struct cli_instance *sh, int argc, char **argv)
{
	(void)sh; (void)argc; (void)argv;
	return 0;
}

#ifdef TEST_COMPLETE_SMALL_BUF
/* Names longer than the 8-byte line buffer (max len 7) so completion overflows. */
CLI_CMD_REGISTER(verbose12, NULL, "vb12", h_ok, 1, 0);   /* shares "verbose1" LCP */
CLI_CMD_REGISTER(verbose13, NULL, "vb13", h_ok, 1, 0);
CLI_CMD_REGISTER(zapzapzap, NULL, "z",    h_ok, 1, 0);   /* unique 'z', 9 chars */
#else
CLI_SUBCMD_SET_CREATE(sub_thread,
	CLI_CMD_ARG(list,   NULL, "list",   h_ok, 1, 0),
	CLI_CMD_ARG(stacks, NULL, "stacks", h_ok, 1, 0),
	CLI_SUBCMD_SET_END);
CLI_CMD_REGISTER(thread,  sub_thread, "thread",  NULL, 1, 0);
CLI_CMD_REGISTER(verbose, NULL,       "verbose", h_ok, 1, 0);  /* shares "ver" with version */
CLI_CMD_REGISTER(version, NULL,       "version", h_ok, 1, 0);
CLI_CMD_REGISTER(zap,     NULL,       "zap",     h_ok, 1, 0);   /* unique 'z' */
#endif

/* ---- harness ----------------------------------------------------------- */

CLI_BACKEND_DUMMY_DEFINE(tr);
static struct cli_instance sh;

static void reset_sh(void)
{
	memset(&sh, 0, sizeof sh);
	sh.tr = &tr;
	tr.sh = &sh;
	strcpy(sh.prompt, "> ");
	sh.term_width = 80;
	cli_dummy_clear_output(&tr);
	cli_dummy_clear_rx(&tr);
	cli_dummy_reset_stats(&tr);
	cli_dummy_set_tx_fail(&tr, 0);
	cli_dummy_set_tx_cap(&tr, 0);
	cli_test_set_tx_wait_hook(NULL, NULL);
}

static void feed_byte(uint8_t b) { cli_input_byte(&sh, b); }
static void feed(const char *s) { for (const char *p = s; *p; p++) feed_byte((uint8_t)*p); }
static void tab(void) { feed_byte(0x09); }

static const char *cap_str(void) { return cli_dummy_output_str(&tr); }
static int cap_has(const char *needle) { return strstr(cap_str(), needle) != NULL; }
static void cap_clear(void) { cli_dummy_clear_output(&tr); }

/* ---- tests ------------------------------------------------------------- */

#ifndef TEST_COMPLETE_SMALL_BUF

static void test_root_unique(void)
{
	reset_sh();
	feed("z");
	tab();
	assert(strcmp(sh.line, "zap ") == 0 && sh.cur == 4 && sh.len == 4);
	assert(sh.tab_list_armed == 0);

	/* Prefix already a complete, unique name: Tab just appends the separator. */
	reset_sh();
	feed("zap");
	tab();
	assert(strcmp(sh.line, "zap ") == 0 && sh.cur == 4 && sh.len == 4);
	assert(sh.tab_list_armed == 0);
}

static void test_root_lcp_extend(void)
{
	reset_sh();
	feed("ve");
	tab();                                   /* verbose + version share "ver" */
	assert(strcmp(sh.line, "ver") == 0 && sh.len == 3);
	assert(sh.tab_list_armed == 1);          /* armed: next Tab lists */
}

static void test_two_stage_list(void)
{
	reset_sh();
	feed("ver");
	cap_clear();
	tab();                                   /* 1st Tab at common prefix: BEL, no list */
	assert(cap_has("\x07"));
	assert(!cap_has("version") && !cap_has("verbose"));
	assert(sh.tab_list_armed == 1);

	cap_clear();
	tab();                                   /* 2nd consecutive Tab: list */
	assert(cap_has("version") && cap_has("verbose"));

	/* A non-Tab key resets the arm, so the two-press flow restarts. */
	reset_sh();
	feed("ver");
	tab();                                   /* armed = 1 */
	feed_byte(0x06);                         /* Ctrl+f at EOL: no move, but resets arm */
	assert(sh.tab_list_armed == 0);
	cap_clear();
	tab();                                   /* arms again, BEL, no list */
	assert(cap_has("\x07") && !cap_has("version"));
	assert(sh.tab_list_armed == 1);
}

static void test_subcommand(void)
{
	reset_sh();
	feed("thread l");
	tab();
	assert(strcmp(sh.line, "thread list ") == 0);

	reset_sh();
	feed("thread ");
	tab();                                   /* 1st: arm + BEL */
	cap_clear();
	tab();                                   /* 2nd: list subcommands */
	assert(cap_has("list") && cap_has("stacks"));
}

static void test_no_match(void)
{
	reset_sh();
	feed("qqq");
	cap_clear();
	tab();
	assert(cap_has("\x07"));
	assert(strcmp(sh.line, "qqq") == 0 && sh.len == 3);
}

static void test_argument_territory(void)
{
	/* version is a leaf: completing past it is argument territory -> BEL, no change. */
	reset_sh();
	feed("version ");
	cap_clear();
	tab();
	assert(cap_has("\x07"));
	assert(strcmp(sh.line, "version ") == 0);
}

static void test_empty_line(void)
{
	reset_sh();
	cap_clear();
	tab();                                   /* 1st: arm + BEL, no list */
	assert(cap_has("\x07") && !cap_has("version"));
	assert(sh.len == 0 && sh.tab_list_armed == 1);

	cap_clear();
	tab();                                   /* 2nd: list all roots */
	assert(cap_has("version") && cap_has("verbose") &&
	       cap_has("zap") && cap_has("thread"));
	assert(sh.len == 0);
}

static void test_midline_insert(void)
{
	reset_sh();
	feed("z x");                             /* line "z x", cur at end */
	feed_byte(0x01);                         /* Ctrl+a: home, cur = 0 */
	feed_byte(0x06);                         /* Ctrl+f: right, cur = 1 (just after 'z') */
	assert(sh.cur == 1);
	tab();                                   /* complete "z" -> "zap", no trailing space mid-line */
	assert(strcmp(sh.line, "zap x") == 0 && sh.cur == 3);
}

static void test_leading_spaces_root(void)
{
	reset_sh();
	feed("  z");                             /* leading spaces: ws != 0 but no preceding token */
	tab();
	assert(strcmp(sh.line, "  zap ") == 0 && sh.cur == 6);
}

int main(void)
{
	test_root_unique();
	test_root_lcp_extend();
	test_two_stage_list();
	test_subcommand();
	test_no_match();
	test_argument_territory();
	test_empty_line();
	test_midline_insert();
	test_leading_spaces_root();
	printf("OK: test_complete (full tree)\n");
	return 0;
}

#else  /* TEST_COMPLETE_SMALL_BUF: CLI_CMD_BUFFER_SIZE == 8 */

static void test_bufferfull_unique(void)
{
	/* "z" -> zapzapzap (9 chars): the completion cannot fit -> BEL, line unchanged. */
	reset_sh();
	feed("z");
	cap_clear();
	tab();
	assert(cap_has("\x07"));
	assert(strcmp(sh.line, "z") == 0 && sh.len == 1);
}

static void test_bufferfull_lcp_reaches_list(void)
{
	/* "v" -> verbose12 / verbose13 share the 8-char LCP "verbose1", which cannot
	 * fit the 7-char line.  The failed extend must NOT trap the two-stage flow:
	 * 1st Tab BELs + arms, 2nd Tab lists despite the line never growing. */
	reset_sh();
	feed("v");
	cap_clear();
	tab();                                   /* extend fails -> BEL, arm */
	assert(cap_has("\x07"));
	assert(strcmp(sh.line, "v") == 0 && sh.len == 1);
	assert(sh.tab_list_armed == 1);
	assert(!cap_has("verbose12"));

	cap_clear();
	tab();                                   /* armed: list despite failed extend */
	assert(cap_has("verbose12") && cap_has("verbose13"));
	assert(strcmp(sh.line, "v") == 0);
}

int main(void)
{
	test_bufferfull_unique();
	test_bufferfull_lcp_reaches_list();
	printf("OK: test_complete (small buffer)\n");
	return 0;
}

#endif
