/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 *
 * Host unit test for the Shell core (issue #4): the ASCII filter, the RX state
 * machine (printable/echo, Backspace, BEL on full, Ctrl+C, ESC/CSI swallow,
 * CR / LF / CR-LF coalescing) and dispatch (cli_parse status -> message, handler
 * invocation with handler-relative argv, fail-safe, instance isolation).
 *
 * Drives the ThreadX-free cli_session.c directly (no thread): bytes are injected
 * at the session level with cli_input_byte(), while output and the tx_* glue go
 * through the shared dummy backend (cli_backend_dummy) + host_glue, so this test
 * and test_integration.c share one transport/flow-control model.  The full RX
 * read() path is exercised end-to-end in test_integration.c.  Compiled with small
 * CLI_* limits (see run_host_tests.sh) so the buffer-full and too-many-tokens
 * paths fit a compact input.
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

/* ---- test commands ----------------------------------------------------- */

static int  ran_argc;
static char ran_a0[32];
static char ran_a1[32];

static int h_echo(struct cli_instance *sh, int argc, char **argv)
{
	ran_argc = argc;
	ran_a0[0] = ran_a1[0] = '\0';
	if (argc > 0) { strncpy(ran_a0, argv[0], sizeof ran_a0 - 1); ran_a0[sizeof ran_a0 - 1] = '\0'; }
	if (argc > 1) { strncpy(ran_a1, argv[1], sizeof ran_a1 - 1); ran_a1[sizeof ran_a1 - 1] = '\0'; }
	cli_write(sh, "echo-ran\r\n", 10);
	return 0;
}
static int h_fail(struct cli_instance *sh, int argc, char **argv)
{
	(void)sh; (void)argc; (void)argv;
	return 7;
}

CLI_SUBCMD_SET_CREATE(sub_thr, CLI_CMD_ARG(list, NULL, "list", h_echo, 1, 0),
	CLI_SUBCMD_SET_END);

CLI_CMD_REGISTER(version, NULL,    "show version", h_echo, 1, 0);
CLI_CMD_REGISTER(fails,   NULL,    "always fails", h_fail, 1, 0);
CLI_CMD_REGISTER(threads, sub_thr, "threads",      NULL,   1, 0);
CLI_CMD_REGISTER(need2,   NULL,    "needs 2 args", h_echo, 2, 0);

/* ---- harness ----------------------------------------------------------- */

CLI_BACKEND_DUMMY_DEFINE(tr);
static struct cli_instance sh;

static void reset_sh(void)
{
	memset(&sh, 0, sizeof sh);
	sh.tr = &tr;
	tr.sh = &sh;
	strcpy(sh.prompt, "> ");
	cli_dummy_clear_output(&tr);
	cli_dummy_clear_rx(&tr);
	cli_dummy_reset_stats(&tr);
	cli_dummy_set_tx_fail(&tr, 0);
	cli_dummy_set_tx_cap(&tr, 0);          /* unlimited */
	cli_test_set_tx_wait_hook(NULL, NULL);
	ran_argc = -1;
	ran_a0[0] = ran_a1[0] = '\0';
}

static void feed(const char *s)
{
	for (const char *p = s; *p; p++)
		cli_input_byte(&sh, (uint8_t)*p);
}

static void feed_byte(uint8_t b) { cli_input_byte(&sh, b); }

static const char *cap_str(void) { return cli_dummy_output_str(&tr); }
static int cap_has(const char *needle) { return strstr(cap_str(), needle) != NULL; }

static int count_occurrences(const char *needle)
{
	int n = 0;
	size_t nl = strlen(needle);
	for (const char *p = cap_str(); (p = strstr(p, needle)) != NULL; p += nl)
		n++;
	return n;
}

/* ---- tests ------------------------------------------------------------- */

static void test_input_editing(void)
{
	reset_sh();
	feed("abc");
	assert(sh.len == 3 && strcmp(sh.line, "abc") == 0);
	assert(cap_has("abc"));                       /* chars are echoed */

	/* ASCII filter: high-bit bytes are dropped, line unchanged. */
	feed_byte(0xC3);
	feed_byte(0x80);
	assert(sh.len == 3 && strcmp(sh.line, "abc") == 0);

	/* Backspace erases one char and emits the destructive sequence. */
	reset_sh();
	feed("ab");
	feed_byte(0x08);
	assert(sh.len == 1 && strcmp(sh.line, "a") == 0);
	assert(cap_has("\b \b"));
	feed_byte(0x7F);                              /* DEL also erases */
	assert(sh.len == 0);

	/* ESC / CSI (arrow-key) sequence is swallowed, never entering the line. */
	reset_sh();
	feed("a");
	feed_byte(0x1B); feed_byte('['); feed_byte('C');   /* right arrow */
	feed("b");
	assert(sh.len == 2 && strcmp(sh.line, "ab") == 0);
}

static void test_bel_on_full(void)
{
	/* Compiled with CLI_CMD_BUFFER_SIZE=16 -> at most 15 chars. */
	reset_sh();
	for (int i = 0; i < 40; i++)
		feed_byte('x');
	assert(sh.len == CLI_CMD_BUFFER_SIZE - 1);
	assert(cap_has("\x07"));                       /* bell rung */
}

static void test_ctrl_c(void)
{
	reset_sh();
	feed("partial");
	feed_byte(0x03);                              /* Ctrl+C */
	assert(sh.len == 0);
	assert(cap_has("^C"));
	assert(cap_has("> "));                        /* fresh prompt */

	/* Ctrl+C recovers even from a half-read escape sequence. */
	reset_sh();
	feed("ab");
	feed_byte(0x1B); feed_byte('[');              /* enter CSI state */
	feed_byte(0x03);                              /* Ctrl+C cancels + resets state */
	assert(sh.rx == CLI_RX_NORMAL && sh.len == 0);
	feed("cd");                                   /* subsequent input is normal */
	assert(sh.len == 2 && strcmp(sh.line, "cd") == 0);
}

static void test_dispatch_ok(void)
{
	reset_sh();
	feed("version\r");
	assert(cap_has("echo-ran"));
	assert(ran_argc == 1 && strcmp(ran_a0, "version") == 0);
	assert(sh.len == 0);                          /* line reset after dispatch */
	assert(sh.last_result == 0);
	/* prompt reappears at the very end */
	size_t n;
	const char *o = cli_dummy_output(&tr, &n);
	assert(n >= 2 && strcmp(o + n - 2, "> ") == 0);

	/* Subcommand: handler gets the leaf-relative argv (argv[0] == "list"). */
	reset_sh();
	feed("threads list\r");
	assert(cap_has("echo-ran"));
	assert(ran_argc == 1 && strcmp(ran_a0, "list") == 0);
}

static void test_newline_coalescing(void)
{
	reset_sh();
	feed("version\r\n");                          /* CR-LF: one dispatch */
	assert(count_occurrences("echo-ran") == 1);

	reset_sh();
	feed("version\n");                            /* lone LF: one dispatch */
	assert(count_occurrences("echo-ran") == 1);

	reset_sh();
	feed("version\r");                            /* lone CR: one dispatch */
	assert(count_occurrences("echo-ran") == 1);
}

static void test_dispatch_errors(void)
{
	reset_sh();
	feed("nope\r");
	assert(cap_has("nope: command not found"));
	assert(sh.last_result == CLI_DISPATCH_ERR);

	reset_sh();
	feed("threads\r");                            /* pure parent, no handler */
	assert(cap_has("threads: missing or unknown subcommand"));
	assert(sh.last_result == CLI_DISPATCH_ERR);

	reset_sh();
	feed("need2\r");                              /* mandatory 2, got 1 */
	assert(cap_has("need2: invalid number of arguments"));

	reset_sh();
	feed("version a b c d\r");                    /* > CLI_MAX_ARGC(4) tokens */
	assert(cap_has("too many arguments"));

	reset_sh();
	feed("version \"ab\r");                       /* unterminated quote */
	assert(cap_has("unterminated quote"));

	reset_sh();
	feed("\r");                                   /* blank line: no error */
	assert(!cap_has("not found") && !cap_has("invalid"));
}

static void test_fail_safe(void)
{
	reset_sh();
	feed("fails\r");
	assert(sh.last_result == 7);                  /* non-zero handler return kept */
	/* shell keeps going: a following command still dispatches */
	feed("version\r");
	assert(cap_has("echo-ran"));
}

/* §11: a TX failure during a command forces a non-zero result even though the
 * handler itself returned 0 (it did not check cli_print's return).  Here the
 * dummy backend's immediate-failure mode stands in for a dead transport; the
 * tx_dropped *count* (timeout path) is asserted in test_integration.c. */
static void test_tx_failure_promotes_result(void)
{
	reset_sh();
	cli_dummy_set_tx_fail(&tr, 1);
	feed("version\r");
	assert(sh.last_result == CLI_DISPATCH_ERR);
	assert(sh.tx_failed == 1);
}

/* Two independent instances must not share state or cross output streams. */
static void test_instance_isolation(void)
{
	struct cli_dummy     d1 = {0}, d2 = {0};
	struct cli_transport t1 = { &cli_dummy_api, NULL, &d1 };
	struct cli_transport t2 = { &cli_dummy_api, NULL, &d2 };
	struct cli_instance  s1, s2;

	memset(&s1, 0, sizeof s1); s1.tr = &t1; t1.sh = &s1; strcpy(s1.prompt, "1> ");
	memset(&s2, 0, sizeof s2); s2.tr = &t2; t2.sh = &s2; strcpy(s2.prompt, "2> ");
	cli_dummy_set_tx_cap(&t1, 0); cli_dummy_set_tx_cap(&t2, 0);   /* unlimited */

	for (const char *p = "version\r"; *p; p++) cli_input_byte(&s1, (uint8_t)*p);
	for (const char *p = "no\r";      *p; p++) cli_input_byte(&s2, (uint8_t)*p);

	const char *o1 = cli_dummy_output_str(&t1);
	const char *o2 = cli_dummy_output_str(&t2);
	assert(strstr(o1, "echo-ran") != NULL);                  /* s1 ran the command */
	assert(strstr(o1, "command not found") == NULL);
	assert(strstr(o2, "no: command not found") != NULL);     /* s2 saw only its own */
	assert(strstr(o2, "echo-ran") == NULL);                  /* no cross-talk */
	assert(s1.last_result == 0 && s2.last_result == CLI_DISPATCH_ERR);
}

int main(void)
{
	test_input_editing();
	test_bel_on_full();
	test_ctrl_c();
	test_dispatch_ok();
	test_newline_coalescing();
	test_dispatch_errors();
	test_fail_safe();
	test_tx_failure_promotes_result();
	test_instance_isolation();
	printf("OK: ascii filter / state machine / dispatch / fail-safe / isolation pass\n");
	return 0;
}
