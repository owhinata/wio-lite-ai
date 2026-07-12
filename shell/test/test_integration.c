/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 *
 * Host integration test for the Shell core (issue #6): input -> execute ->
 * output verified END-TO-END THROUGH THE DUMMY BACKEND.  Unlike test_core.c
 * (which injects at the session level and stubs the transport), this drives the
 * real transport contract: cli_dummy_inject() fills the backend RX FIFO,
 * cli_test_pump() drains it through read() into the line state machine, and
 * every byte of output reaches the dummy capture log through tr->api->write()
 * via the faithful host cli_tx_send_blocking() (host_glue.c).
 *
 * Coverage at this milestone (implemented surface #2-#5):
 *   - basic dispatch + subcommands (handler ran, output captured, prompt back)
 *   - §13 ASCII filter, CR/LF coalesce, ESC/CSI swallow -- via the backend
 *   - §11 flow control: normal backpressure completes, timeout drops, immediate
 *     write() failure, and dispatch result promotion on TX failure
 *   - §9/§18 abnormal: unknown cmd, arg errors, line overflow (BEL), Ctrl+C,
 *     RX-ring overflow drop + stat
 *   - §10 multi-instance: two dummy instances interleaved, no output crosstalk
 *
 * Built with small CLI_* limits (see run_host_tests.sh) and colour OFF so the
 * overflow / too-many-tokens paths fit compact input and output compares plainly.
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

static int ran;

/* issue #16 cooperative-cancel test commands.  inject_at lets h_loop drop a 0x03
 * into its own RX mid-run, modelling a Ctrl+C arriving while the handler runs
 * (single-threaded host: the byte must enter the ring DURING the handler). */
static int inject_at = -1;
static int hook_fired;        /* one-shot guard for inject_ctrl_c_hook (below) */
static unsigned sleep_ticks = 100;   /* delay h_sleep passes to cli_sleep() */

static int h_ok(struct cli_instance *sh, int argc, char **argv)
{
	(void)argc; (void)argv;
	ran++;
	cli_print(sh, "OK\r\n");
	return 0;
}
static int h_args(struct cli_instance *sh, int argc, char **argv)
{
	cli_print(sh, "argc=%d a1=%s\r\n", argc, argc > 1 ? argv[1] : "-");
	return 0;
}

CLI_SUBCMD_SET_CREATE(sub_thing, CLI_CMD_ARG(list, NULL, "list", h_ok, 1, 0),
	CLI_CMD_ARG(put, NULL, "put <key>", h_args, 2, 0),   /* issue #37: sub w/ mandatory>1 */
	CLI_SUBCMD_SET_END);

CLI_CMD_REGISTER(hello, NULL,      "say hi",       h_ok,   1, 0);
CLI_CMD_REGISTER(echo2, NULL,      "echo arg",     h_args, 1, 1);
CLI_CMD_REGISTER(thing, sub_thing, "parent only",  NULL,   1, 0);
CLI_CMD_REGISTER(need2, NULL,      "needs 2 args", h_args, 2, 0);

/* issue #16: a compute loop that polls cli_cancel_requested(); optionally drops a
 * 0x03 into its own RX at iteration inject_at (async-arrival model). */
static int h_loop(struct cli_instance *sh, int argc, char **argv)
{
	(void)argc; (void)argv;
	for (int i = 0; i < 1000; i++) {
		if (i == inject_at)
			cli_dummy_inject(sh->tr, "\x03", 1);
		if (cli_cancel_requested(sh)) {
			cli_print(sh, "stopped\r\n");   /* suppressed (dispatching+cancel) */
			return 1;
		}
	}
	cli_print(sh, "done\r\n");
	return 0;
}

/* issue #16: floods output -- with a tiny TX cap and no wait hook this would
 * TX-timeout, but a buffered 0x03 cancels it via the wait-path poll. */
static int h_txwait(struct cli_instance *sh, int argc, char **argv)
{
	(void)argc; (void)argv;
	for (int i = 0; i < 100; i++)
		cli_print(sh, "line........................\r\n");
	cli_print(sh, "done\r\n");
	return 0;
}

/* issue #16: cancellable delay via cli_sleep() (host mirror in host_glue.c). */
static int h_sleep(struct cli_instance *sh, int argc, char **argv)
{
	(void)argc; (void)argv;
	int r = cli_sleep(sh, sleep_ticks);
	cli_print(sh, r ? "cancelled\r\n" : "elapsed\r\n");
	return r ? 1 : 0;
}

CLI_CMD_REGISTER(loopc,   NULL, "cancel loop test",  h_loop,   1, 0);
CLI_CMD_REGISTER(txwaitc, NULL, "cancel tx test",    h_txwait, 1, 0);
CLI_CMD_REGISTER(sleepc,  NULL, "cancel sleep test", h_sleep,  1, 0);

/* issue #21: exercises the `watch` re-dispatch recipe -- parse redisp_cmd into
 * LOCAL scratch and call the resolved handler, never touching sh->pr/argv/line.
 * Prints the parser-normalised root token so the test can assert quote/escape
 * normalisation (the basis of watch's denylist). */
static char redisp_cmd[64];
static int h_redisp(struct cli_instance *sh, int argc, char **argv)
{
	char  buf[CLI_CMD_BUFFER_SIZE];
	char *largv[CLI_ARGV_CAP];
	struct cli_parse_result lpr;

	(void)argc; (void)argv;
	strncpy(buf, redisp_cmd, sizeof buf - 1);
	buf[sizeof buf - 1] = '\0';
	if (cli_parse(buf, largv, CLI_ARGV_CAP, &lpr) != CLI_PARSE_OK) {
		cli_print(sh, "PARSEFAIL\r\n");
		return 1;
	}
	cli_print(sh, "root=%s\r\n", largv[0]);   /* normalised root token */
	return lpr.cmd->handler(sh, lpr.argc, lpr.argv);
}
CLI_CMD_REGISTER(redisp, NULL, "re-dispatch recipe test", h_redisp, 1, 0);

/* ---- harness ----------------------------------------------------------- */

CLI_BACKEND_DUMMY_DEFINE(tr0);
CLI_BACKEND_DUMMY_DEFINE(tr1);
static struct cli_instance sh0;
static struct cli_instance sh1;

static void reset(struct cli_instance *s, struct cli_transport *t)
{
	memset(s, 0, sizeof *s);
	s->tr = t;
	t->sh = s;
	strcpy(s->prompt, "> ");
	cli_dummy_clear_output(t);
	cli_dummy_clear_rx(t);
	cli_dummy_reset_stats(t);
	cli_dummy_set_tx_fail(t, 0);
	cli_dummy_set_tx_cap(t, 0);          /* unlimited */
	cli_test_set_tx_wait_hook(NULL, NULL);
	cli_test_set_sleep_hook(NULL, NULL);
	inject_at = -1;
	hook_fired = 0;
	sleep_ticks = 100;
	redisp_cmd[0] = '\0';
	ran = 0;
}

/* Inject a line as if received by the backend, then drain it through read(). */
static void run_line(struct cli_instance *s, const char *line)
{
	cli_dummy_inject(s->tr, line, strlen(line));
	cli_test_pump(s);
}

static void inject_bytes(struct cli_instance *s, const uint8_t *b, size_t n)
{
	cli_dummy_inject(s->tr, b, n);
	cli_test_pump(s);
}

static const char *out_of(struct cli_transport *t) { return cli_dummy_output_str(t); }
static int has(struct cli_transport *t, const char *needle)
{
	return strstr(out_of(t), needle) != NULL;
}
static int count(struct cli_transport *t, const char *needle)
{
	int n = 0;
	size_t nl = strlen(needle);
	for (const char *p = out_of(t); (p = strstr(p, needle)) != NULL; p += nl)
		n++;
	return n;
}

/* ---- basic dispatch through the backend -------------------------------- */

static void test_basic_dispatch(void)
{
	reset(&sh0, &tr0);
	run_line(&sh0, "hello\r");
	assert(ran == 1);
	assert(has(&tr0, "OK"));
	assert(has(&tr0, "hello"));            /* input was echoed back */
	assert(sh0.last_result == 0);
	assert(sh0.len == 0);                  /* line reset after dispatch */

	size_t n;
	const char *o = cli_dummy_output(&tr0, &n);
	assert(n >= 2 && strcmp(o + n - 2, "> ") == 0);   /* prompt reappears */

	/* Subcommand: leaf handler runs. */
	reset(&sh0, &tr0);
	run_line(&sh0, "thing list\r");
	assert(ran == 1 && has(&tr0, "OK"));

	/* Optional arg reaches the handler. */
	reset(&sh0, &tr0);
	run_line(&sh0, "echo2 foo\r");
	assert(has(&tr0, "argc=2 a1=foo"));
}

/* ---- §13 ASCII filter / coalesce / swallow, all via the backend -------- */

static void test_filter_and_editing_via_backend(void)
{
	/* §13: high-bit bytes never reach the line. */
	reset(&sh0, &tr0);
	const uint8_t hi[] = { 'a', 'b', 0xC3, 0x80, 'c' };   /* no newline */
	inject_bytes(&sh0, hi, sizeof hi);
	assert(sh0.len == 3 && strcmp(sh0.line, "abc") == 0);

	/* CR-LF coalesces into exactly one dispatch. */
	reset(&sh0, &tr0);
	run_line(&sh0, "hello\r\n");
	assert(count(&tr0, "OK") == 1);

	/* ESC/CSI (right-arrow) is swallowed, never entering the line. */
	reset(&sh0, &tr0);
	const uint8_t esc[] = { 'a', 0x1B, '[', 'C', 'b' };   /* no newline */
	inject_bytes(&sh0, esc, sizeof esc);
	assert(sh0.len == 2 && strcmp(sh0.line, "ab") == 0);
}

/* ---- §11 flow control -------------------------------------------------- */

/* TX-wait hook that frees a little capacity each time TX reports full -- the
 * host analogue of a backend firing cli_transport_notify_tx() as it drains. */
static void free_some_hook(struct cli_instance *s, void *arg)
{
	(void)arg;
	cli_dummy_free_tx(s->tr, 8);
}

static void test_backpressure_completes(void)
{
	reset(&sh0, &tr0);
	cli_dummy_set_tx_cap(&tr0, 4);                 /* full after 4 bytes */
	cli_test_set_tx_wait_hook(free_some_hook, NULL);

	int r = cli_print(&sh0, "0123456789");         /* 10 B > capacity */
	assert(r == 0);                                /* completed, not dropped */
	assert(sh0.tx_failed == 0 && sh0.tx_dropped == 0);

	size_t n;
	const char *o = cli_dummy_output(&tr0, &n);
	assert(n == 10 && memcmp(o, "0123456789", 10) == 0);   /* in order */
}

static void test_tx_timeout_drops(void)
{
	reset(&sh0, &tr0);
	cli_dummy_set_tx_cap(&tr0, 4);                 /* 4 B then full */
	cli_test_set_tx_wait_hook(NULL, NULL);         /* space never freed */

	int r = cli_print(&sh0, "0123456789");         /* 10 B */
	assert(r < 0);
	assert(sh0.tx_failed == 1);
	assert(sh0.tx_dropped == 6);                   /* 10 - 4 accepted */

	size_t n;
	cli_dummy_output(&tr0, &n);
	assert(n == 4);                                /* only the accepted prefix */
}

static void test_tx_immediate_fail(void)
{
	reset(&sh0, &tr0);
	cli_dummy_set_tx_fail(&tr0, 1);

	int r = cli_print(&sh0, "hello");
	assert(r < 0 && sh0.tx_failed == 1);
	assert(sh0.tx_dropped == 0);                   /* write()<0 path: not a drop */

	size_t n;
	cli_dummy_output(&tr0, &n);
	assert(n == 0);                                /* nothing accepted */
}

/* §11: a TX failure during a command forces a non-zero command result even
 * though the handler returned 0 (it ignored cli_print's return). */
static void test_tx_failure_promotes_result(void)
{
	reset(&sh0, &tr0);
	cli_dummy_set_tx_fail(&tr0, 1);
	run_line(&sh0, "hello\r");
	assert(sh0.last_result == CLI_DISPATCH_ERR);
}

/* ---- §9 / §18 abnormal cases ------------------------------------------- */

static void test_unknown_command(void)
{
	reset(&sh0, &tr0);
	run_line(&sh0, "nope\r");
	assert(has(&tr0, "nope: command not found"));
	assert(sh0.last_result == CLI_DISPATCH_ERR);

	/* fail-safe: prompt resumes and the next command still runs. */
	size_t n;
	const char *o = cli_dummy_output(&tr0, &n);
	assert(n >= 2 && strcmp(o + n - 2, "> ") == 0);
	run_line(&sh0, "hello\r");
	assert(has(&tr0, "OK"));
}

static void test_arg_errors(void)
{
	reset(&sh0, &tr0);
	run_line(&sh0, "need2\r");                     /* mandatory 2, got 1 */
	assert(has(&tr0, "need2: invalid number of arguments"));
	/* issue #37: the usage line follows -- root-leaf path + .help as usage. */
	assert(has(&tr0, "usage: need2  (needs 2 args)"));
	assert(sh0.last_result == CLI_DISPATCH_ERR);

	/* issue #37: a subcommand's WRONG_ARGS prints the FULL command path. */
	reset(&sh0, &tr0);
	run_line(&sh0, "thing put\r");                 /* sub mandatory 2, got 1 */
	assert(has(&tr0, "put: invalid number of arguments"));
	assert(has(&tr0, "usage: thing put  (put <key>)"));
	assert(sh0.last_result == CLI_DISPATCH_ERR);

	reset(&sh0, &tr0);
	run_line(&sh0, "hello a b c d\r");             /* > CLI_MAX_ARGC tokens */
	assert(has(&tr0, "too many arguments"));

	reset(&sh0, &tr0);
	run_line(&sh0, "thing\r");                     /* parent, no handler */
	assert(has(&tr0, "thing: missing or unknown subcommand"));

	reset(&sh0, &tr0);
	run_line(&sh0, "hello \"ab\r");                /* unterminated quote */
	assert(has(&tr0, "unterminated quote"));
}

static void test_ctrl_c(void)
{
	reset(&sh0, &tr0);
	const uint8_t seq[] = { 'p', 'a', 'r', 't', 0x03 };   /* "part" + Ctrl+C */
	inject_bytes(&sh0, seq, sizeof seq);
	assert(sh0.len == 0);
	assert(has(&tr0, "^C"));
	size_t n;
	const char *o = cli_dummy_output(&tr0, &n);
	assert(n >= 2 && strcmp(o + n - 2, "> ") == 0);   /* fresh prompt */
}

/* ---- issue #16: cooperative command cancellation ----------------------- */

/* Inject a Ctrl+C when the send blocks / the sleep waits (async-arrival model).
 * One-shot: it models a SINGLE keypress.  Without this guard, a bounded-TX test
 * keeps blocking through the post-cancel cleanup, the hook re-injects 0x03 each
 * time, and every 0x03 redraws a prompt that blocks again -- an endless loop. */
static void inject_ctrl_c_hook(struct cli_instance *sh, void *arg)
{
	(void)arg;
	if (hook_fired)
		return;
	hook_fired = 1;
	cli_dummy_inject(sh->tr, "\x03", 1);
}

/* BLOCKING1 regression: a 0x03 arriving in the SAME inject as the command line
 * must still cancel the running handler -- proves the 1-byte RX pump leaves it in
 * the ring for cli_cancel_poll() instead of prefetching it past the dispatch. */
static void test_cancel_prefetch(void)
{
	reset(&sh0, &tr0);
	run_line(&sh0, "loopc\r\x03");
	assert(has(&tr0, "^C"));
	assert(!has(&tr0, "done"));            /* loop did not finish */
	assert(!has(&tr0, "stopped"));         /* post-cancel print is suppressed */
	assert(sh0.last_result == CLI_DISPATCH_CANCELLED);
}

/* Compute-loop poll: a Ctrl+C arriving mid-run cancels at the next poll. */
static void test_cancel_compute_loop(void)
{
	reset(&sh0, &tr0);
	inject_at = 5;                         /* h_loop drops a 0x03 at iteration 5 */
	run_line(&sh0, "loopc\r");
	assert(has(&tr0, "^C"));
	assert(!has(&tr0, "done"));
	assert(sh0.last_result == CLI_DISPATCH_CANCELLED);
}

/* TX-blocked early-exit via a 0x03 already buffered before the wait (BLOCKING3):
 * the wait-path poll must see the ring byte and abort before the TX timeout.
 * (The bounded TX is exhausted, so the post-cancel "^C" cannot be captured here;
 * last_result == CANCELLED is the definitive signal that the cancel path ran.) */
static void test_cancel_tx_buffered(void)
{
	reset(&sh0, &tr0);
	cli_dummy_set_tx_cap(&tr0, 16);        /* room for echo + "\r\n", then blocks */
	run_line(&sh0, "txwaitc\r\x03");
	assert(sh0.last_result == CLI_DISPATCH_CANCELLED);
	assert(!has(&tr0, "done"));            /* flood did not complete */
}

/* TX-blocked early-exit via an async 0x03 delivered on the TX-wait wake. */
static void test_cancel_tx_async(void)
{
	reset(&sh0, &tr0);
	cli_dummy_set_tx_cap(&tr0, 16);
	cli_test_set_tx_wait_hook(inject_ctrl_c_hook, NULL);
	run_line(&sh0, "txwaitc\r");
	assert(sh0.last_result == CLI_DISPATCH_CANCELLED);
	assert(!has(&tr0, "done"));
}

/* cli_sleep: buffered 0x03 cancels (poll-before-wait); a plain sleep elapses; an
 * async 0x03 delivered during the wait cancels (BLOCKING3 + CONCERN coverage). */
static void test_cancel_sleep(void)
{
	reset(&sh0, &tr0);
	run_line(&sh0, "sleepc\r\x03");        /* buffered -> cancel */
	assert(has(&tr0, "^C"));
	assert(sh0.last_result == CLI_DISPATCH_CANCELLED);

	reset(&sh0, &tr0);
	run_line(&sh0, "sleepc\r");            /* no 0x03 -> elapsed */
	assert(has(&tr0, "elapsed"));
	assert(!has(&tr0, "^C"));
	assert(sh0.last_result == 0);

	reset(&sh0, &tr0);
	cli_test_set_sleep_hook(inject_ctrl_c_hook, NULL);
	run_line(&sh0, "sleepc\r");            /* async -> cancel */
	assert(has(&tr0, "^C"));
	assert(sh0.last_result == CLI_DISPATCH_CANCELLED);

	/* Contract: ticks==0 elapses at once even with a 0x03 already buffered (the
	 * ticks==0 check precedes the cancel poll).  cli_sleep returns 0 -> "elapsed",
	 * never "cancelled" (the leftover 0x03 is later handled as an input-line ^C). */
	reset(&sh0, &tr0);
	sleep_ticks = 0;
	run_line(&sh0, "sleepc\r\x03");
	assert(has(&tr0, "elapsed"));
	assert(!has(&tr0, "cancelled"));
}

/* issue #21: the watch re-dispatch recipe runs an inner command via LOCAL
 * scratch, resolves subcommands, normalises the root token (quotes stripped),
 * and does not corrupt the outer dispatch state. */
static void test_redispatch_recipe(void)
{
	reset(&sh0, &tr0);
	strcpy(redisp_cmd, "hello");
	run_line(&sh0, "redisp\r");
	assert(has(&tr0, "root=hello"));
	assert(has(&tr0, "OK"));               /* inner hello (h_ok) ran */
	assert(ran == 1);
	/* outer dispatch state intact: a normal command still works afterwards */
	run_line(&sh0, "hello\r");
	assert(ran == 2);

	/* subcommand resolves through the recipe */
	reset(&sh0, &tr0);
	strcpy(redisp_cmd, "thing list");
	run_line(&sh0, "redisp\r");
	assert(has(&tr0, "root=thing"));
	assert(has(&tr0, "OK"));

	/* quote/escape normalised by the tokenizer -> denylist-safe root token */
	reset(&sh0, &tr0);
	strcpy(redisp_cmd, "\"hello\"");
	run_line(&sh0, "redisp\r");
	assert(has(&tr0, "root=hello"));       /* quotes removed by cli_parse */
	assert(has(&tr0, "OK"));
}

/* Type-ahead typed during a running command is discarded, not auto-run. */
static void test_cancel_typeahead_discard(void)
{
	reset(&sh0, &tr0);
	/* "loopc" + Ctrl+C + "hello" + Enter in one burst: the 0x03 cancels loopc;
	 * the trailing "hello\r" must be discarded, not executed (issue #16). */
	run_line(&sh0, "loopc\r\x03hello\r");
	assert(has(&tr0, "^C"));
	assert(ran == 0);                      /* hello (h_ok) never ran */
	assert(!has(&tr0, "OK"));
}

/* issue #23: ';'-separated sequential execution end-to-end through the backend.
 * Lines stay <= 15 chars (CLI_CMD_BUFFER_SIZE=16 in this harness). */
static void test_sequence(void)
{
	/* two commands run in order; both handlers fire; ONE prompt at the end */
	reset(&sh0, &tr0);
	run_line(&sh0, "hello;hello\r");
	assert(ran == 2);
	assert(count(&tr0, "OK") == 2);
	{
		size_t n;
		const char *o = cli_dummy_output(&tr0, &n);
		assert(n >= 2 && strcmp(o + n - 2, "> ") == 0);  /* final prompt */
	}
	assert(count(&tr0, "> ") == 1);                          /* prompt only once */

	/* a bad first command does NOT stop the second (continue-on-error) */
	reset(&sh0, &tr0);
	run_line(&sh0, "nope;hello\r");
	assert(has(&tr0, "nope: command not found"));
	assert(ran == 1 && has(&tr0, "OK"));
	assert(sh0.last_result == 0);                            /* last seg (hello) won */

	/* an error in the SECOND command: first runs, last_result = ERR */
	reset(&sh0, &tr0);
	run_line(&sh0, "hello;nope\r");
	assert(ran == 1);
	assert(has(&tr0, "nope: command not found"));
	assert(sh0.last_result == CLI_DISPATCH_ERR);

	/* empty segments (doubled ';') are skipped, no extra output / crash */
	reset(&sh0, &tr0);
	run_line(&sh0, "hello;;hello\r");
	assert(ran == 2 && count(&tr0, "OK") == 2);

	/* a quoted ';' keeps the command whole: echo2 gets the literal arg */
	reset(&sh0, &tr0);
	run_line(&sh0, "echo2 \"a;b\"\r");
	assert(has(&tr0, "argc=2 a1=a;b"));
	assert(!has(&tr0, "command not found"));

	/* Ctrl+C in the first command aborts the rest of the sequence */
	reset(&sh0, &tr0);
	inject_at = 5;                          /* loopc self-injects 0x03 at iter 5 */
	run_line(&sh0, "loopc;hello\r");
	assert(has(&tr0, "^C") && count(&tr0, "^C") == 1);
	assert(ran == 0 && !has(&tr0, "OK"));   /* hello after ';' did NOT run */
	assert(sh0.last_result == CLI_DISPATCH_CANCELLED);
}

static void test_line_overflow_bel(void)
{
	/* CLI_CMD_BUFFER_SIZE=16 -> at most 15 chars; further chars ring the bell. */
	reset(&sh0, &tr0);
	uint8_t blast[40];
	memset(blast, 'x', sizeof blast);
	inject_bytes(&sh0, blast, sizeof blast);
	assert(sh0.len == CLI_CMD_BUFFER_SIZE - 1);
	assert(has(&tr0, "\x07"));                      /* BEL */
}

static void test_rx_overflow_drops(void)
{
	/* A burst larger than the RX FIFO overflows: excess dropped + counted,
	 * both in the backend and in the instance stat (req §9/§18 10e). */
	reset(&sh0, &tr0);
	uint8_t burst[CLI_DUMMY_RX_BUFFER_SIZE + 64];
	memset(burst, 'y', sizeof burst);
	cli_dummy_inject(&tr0, burst, sizeof burst);    /* no pump: let it overflow */
	assert(tr0_ctx.rx_dropped > 0);
	assert(sh0.rx_dropped == tr0_ctx.rx_dropped);
}

/* ---- §10 multi-instance isolation -------------------------------------- */

static void test_multi_instance_isolation(void)
{
	reset(&sh0, &tr0);
	reset(&sh1, &tr1);

	/* Interleave the two sessions byte-group by byte-group; outputs must not
	 * cross and per-instance state stays independent (req §10). */
	run_line(&sh0, "hel");
	run_line(&sh1, "no");
	run_line(&sh0, "lo\r");                         /* sh0: "hello" -> OK */
	run_line(&sh1, "pe\r");                         /* sh1: "nope"  -> error */

	assert(has(&tr0, "OK"));
	assert(!has(&tr0, "command not found"));
	assert(has(&tr1, "nope: command not found"));
	assert(!has(&tr1, "OK"));                        /* no crosstalk */
	assert(sh0.last_result == 0);
	assert(sh1.last_result == CLI_DISPATCH_ERR);
}

int main(void)
{
	test_basic_dispatch();
	test_filter_and_editing_via_backend();
	test_backpressure_completes();
	test_tx_timeout_drops();
	test_tx_immediate_fail();
	test_tx_failure_promotes_result();
	test_unknown_command();
	test_arg_errors();
	test_ctrl_c();
	test_cancel_prefetch();
	test_cancel_compute_loop();
	test_cancel_tx_buffered();
	test_cancel_tx_async();
	test_cancel_sleep();
	test_cancel_typeahead_discard();
	test_redispatch_recipe();
	test_sequence();
	test_line_overflow_bel();
	test_rx_overflow_drops();
	test_multi_instance_isolation();
	printf("OK: dummy backend end-to-end / flow control / abnormal / isolation pass\n");
	return 0;
}
