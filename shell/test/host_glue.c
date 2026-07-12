/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 *
 * Host-side ThreadX-free glue shared by the Shell host tests.  See host_glue.h.
 *
 * cli_tx_send_blocking() mirrors cli_core.c:cli_tx_send_blocking() so the host
 * tests exercise the real §11 flow-control semantics through tr->api->write()
 * rather than a capture stub:
 *   - partial accept (0<n<len)   -> keep sending the remainder until complete
 *   - full (n==0)                -> invoke the TX-wait hook (notify_tx analogue);
 *                                   repeated no-progress waits == timeout: drop
 *                                   the remainder, bump tx_dropped, return <0
 *   - error (n<0)                -> set tx_failed, return <0
 *   - n>remaining                -> defensive clamp
 * The first three behaviours are pinned by test_integration.c so that if this
 * host model drifts from cli_core.c the tests fail; the clamp mirrors cli_core.c
 * defensively and is not exercised (a well-behaved backend never over-accepts).
 */
#include <stddef.h>
#include <stdint.h>

#include "cli_instance.h"
#include "cli_internal.h"
#include "host_glue.h"

/* How many consecutive no-progress TX waits count as a timeout.  With a hook
 * that frees space, progress resets the counter so a send never times out; with
 * no hook the send drops after this many waits (deterministic, no spin). */
#ifndef CLI_TEST_TX_MAX_STALLS
#define CLI_TEST_TX_MAX_STALLS 2
#endif

/* ---- output lock: no-op on the single-threaded host --------------------- */

int  cli_lock(struct cli_instance *sh)   { (void)sh; return 0; }
void cli_unlock(struct cli_instance *sh) { (void)sh; }

/* ---- backend notify: no-op (RX is pumped synchronously) ----------------- *
 * The real notify_tx contract ("backend calls this when TX space frees") is NOT
 * verified on the host -- it is exercised on target by the UART backend (#7). */
void cli_transport_notify_rx(struct cli_instance *sh) { (void)sh; }
void cli_transport_notify_tx(struct cli_instance *sh) { (void)sh; }

/* ---- background jobs: stubbed (ThreadX worker pool is target-only, #25) --- *
 * cli_session.c (ThreadX-free, host-built) calls these from cli_dispatch_line;
 * the real pool lives in cli_job.c (ThreadX), which the host harness does not
 * link.  Launch fails (no worker on the host) and reap is a no-op, so a host
 * test that submits "cmd &" simply runs nothing in the background -- the trailing
 * '&' strip itself is covered by test_parse.c's cli_segment_is_background tests. */
int  cli_job_launch(struct cli_instance *fg, char *seg) { (void)fg; (void)seg; return -1; }
void cli_jobs_reap(struct cli_instance *fg)             { (void)fg; }

/* ---- TX-wait hook ------------------------------------------------------- */

static cli_test_tx_wait_fn g_tx_wait_fn;
static void               *g_tx_wait_arg;

void cli_test_set_tx_wait_hook(cli_test_tx_wait_fn fn, void *arg)
{
	g_tx_wait_fn  = fn;
	g_tx_wait_arg = arg;
}

/* ---- flow-controlled send (mirrors cli_core.c) -------------------------- */

int cli_tx_send_blocking(struct cli_instance *sh, const uint8_t *data, size_t len)
{
	struct cli_transport *tr = sh->tr;
	size_t sent = 0;
	int    stalls = 0;

	while (sent < len) {
		/* Cooperative Ctrl+C fast-fail, mirroring cli_core.c (issue #16). */
		if (sh->dispatching && sh->cancel_req)
			return -1;

		int n = tr->api->write(tr, data + sent, len - sent);
		if (n < 0) {
			sh->tx_failed = 1;
			return -1;
		}
		if (n > 0) {
			if ((size_t)n > len - sent)         /* defensive clamp */
				n = (int)(len - sent);
			sent += (size_t)n;
			stalls = 0;
			continue;
		}

		/* TX full: wait for space (real core suspends on CLI_EVT_TX, and on
		 * CLI_EVT_RX while a command runs).  Mirror cli_core.c's cancel handling:
		 * poll the ring for a buffered 0x03 before "waiting", treat the wait hook
		 * (the host analogue of an RX/TX wake) as a chance for the test to inject
		 * a 0x03, then poll again (issue #16). */
		if (sh->dispatching && cli_cancel_poll(sh))
			return -1;
		if (g_tx_wait_fn)
			g_tx_wait_fn(sh, g_tx_wait_arg);
		if (sh->dispatching && cli_cancel_poll(sh))
			return -1;
		if (++stalls >= CLI_TEST_TX_MAX_STALLS) {
			sh->tx_dropped += (uint32_t)(len - sent);   /* timeout drop */
			return -1;
		}
	}
	return 0;
}

/* ---- cancellable sleep (host analogue of cli_core.c:cli_sleep) ---------- *
 * No ThreadX timer on the host: poll the ring for an already-buffered 0x03, then
 * (if a sleep hook is installed) let the test inject a 0x03 to model an async
 * Ctrl+C arriving during the wait, then poll again.  Returns 1 if cancelled, 0
 * if the "delay" elapsed -- enough to exercise both cli_cancel_poll seams. */
static cli_test_tx_wait_fn g_sleep_fn;
static void               *g_sleep_arg;

void cli_test_set_sleep_hook(cli_test_tx_wait_fn fn, void *arg)
{
	g_sleep_fn  = fn;
	g_sleep_arg = arg;
}

int cli_sleep(struct cli_instance *sh, unsigned ticks)
{
	if (ticks == 0)                    /* contract: ticks==0 elapses at once */
		return 0;
	if (cli_cancel_poll(sh))            /* a 0x03 already buffered in the ring */
		return 1;
	if (g_sleep_fn)                     /* test may inject a 0x03 (async wake) */
		g_sleep_fn(sh, g_sleep_arg);
	if (cli_cancel_poll(sh))
		return 1;
	return 0;                           /* delay elapsed */
}

/* ---- RX pump (mirrors cli_core.c's thread loop) ------------------------- */

void cli_test_pump(struct cli_instance *sh)
{
	struct cli_transport *tr = sh->tr;
	uint8_t b;

	/* One byte at a time, mirroring cli_core.c: a '\r' dispatches synchronously,
	 * so bulk-reading would hide following type-ahead (e.g. Ctrl+C) from
	 * cli_cancel_poll() during the handler (issue #16). */
	while (tr->api->read(tr, &b, 1) > 0)
		cli_input_byte(sh, b);
}
