/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    cmd_wifi_link.c
 * @brief   `wifi link` subcommands (issue #23): the RTL8720DN UART link itself.
 *
 *   wifi link info                     both ends' counters + the current rate
 *   wifi link baud <bps>               change the rate of the link (2M / 3M / 4M / 6M)
 *   wifi link bench [bytes] [secs] [dir]   measured traffic; dir = rx | tx | both | all
 *   wifi link dbench [bytes] [secs] [dir]  the same, free-running on the DATA channel (U1)
 *
 * Where `wifi` is L2 and `net` is L3, this is L1/L2 of the wire between the STM32 and
 * the companion chip: the maintenance and diagnostics face of the link that issue #23
 * built (the numbers here are what proved the "L2 bypass" road the host stack now runs
 * on, and they remain the link's health monitors -- `ore`/drops must stay 0 -- and the
 * DATA channel's regression witness, `dbench`).
 *
 * It talks the LINK-CTRL channel (app/erpc.h), NOT eRPC -- a second frame type that the
 * link layer owns at both ends, so none of it required touching the generated eRPC server
 * shim on the module.  CTRL exists only in firmware 2.1.3+wio-n5 and later, and only the
 * `wifi ver` command teaches the host which firmware is loaded, hence the
 * erpc_module_gen() gate on every subcommand below.
 *
 * PRECONDITION, for everything except the read-only `info`: THE L2 BRIDGE MUST BE DOWN,
 * i.e. this runs after a power-on and before `wifi connect`.  The reason is time and the
 * DATA channel, not ownership -- see link_ctrl_ready_ex() below, which is where issue #30
 * B2c narrowed it down.
 *
 * TIMING.  Per-frame latency is measured with TIM2->CNT (free-running, 32-bit at
 * 2*PCLK1 = 275 MHz, started unconditionally by port/threadx/tx_glue.c for the execution
 * profiler).  NOT DWT->CYCCNT, which stops when the core is in WFI -- this thread sleeps
 * between polls, so DWT would silently under-count.  TIM2 wraps every ~15.6 s, so it is
 * used only for individual round trips; whole-run elapsed time comes from HAL_GetTick.
 *
 * No clock/RCC/register work of its own: it READS TIM2->CNT and computes BRR arithmetic,
 * nothing more (clock-safe).  Clean-room design.
 */
#include "cli.h"
#include "cmd_wifi_priv.h"
#include "erpc.h"
#include "link_data.h"
#include "rtl_link.h"
#include "rtl8720.h"
#include "nx_net.h"          /* the L2 bridge owns the DATA channel while it is up */

#include "stm32h7xx_hal.h"   /* HAL_GetTick, TIM2 */

#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* USART1/UART9 kernel clock, as in app/rtl8720.c: the inherited PCLK2.  Duplicated rather
 * than exported because it is a property of the bootloader's clock tree, and the two
 * copies are checked against each other by the BRR both files compute. */
#define LINK_PCLK2        137500000u

/* TIM2 counts at 2 * PCLK1 = 275 MHz (port/threadx/tx_user.h). */
#define LINK_TIM_HZ       275000000u
#define LINK_TIM_PER_US   (LINK_TIM_HZ / 1000000u)

/* Bench payload bound.  1500 is the number that matters (the Ethernet MTU the L2-bypass
 * road would have to carry); the headroom keeps the CTRL body inside ERPC_CTRL_MAX. */
#define LINK_BENCH_MAX    1536u
#define LINK_BENCH_DEF    1500u
#define LINK_BENCH_SECS   3u
#define LINK_BENCH_MAX_S  30u

/* Per-frame CTRL timeout.  1536 B each way at 2 Mbaud is ~15 ms of wire time; 2 s is far
 * above anything healthy, and a bench that needs it has already failed. */
#define LINK_BENCH_TMO_MS 2000u
#define LINK_CTRL_TMO_MS  500u

/*
 * LINK_DATA_CFG needs its own, much longer timeout.  Every other CTRL command answers as
 * fast as the module can turn a frame around, but CFG(off) DELIBERATELY delays its
 * acknowledgement until the module's DATA writer is idle -- that delay is the promise the
 * teardown depends on -- and the module bounds its own wait at 1 s.  A 500 ms timeout here
 * would abandon the exchange exactly when the module is keeping its side of the bargain,
 * and leave a stale acknowledgement in the stream behind it.
 */
#define LINK_DCFG_TMO_MS  2500u

/*
 * Latency distribution: a fixed histogram rather than stored samples.  A 3-second run at
 * 1500 B produces hundreds of frames, and keeping them all would cost more RAM than the
 * whole channel; 250 us buckets resolve the difference between the baud rates being
 * compared (a 1500 B frame is 7.5 ms at 2 Mbaud and 2.5 ms at 6 Mbaud).
 *
 * THE RANGE MUST COVER THE SLOWEST CASE MEASURED, not the fastest.  An earlier 33-bucket
 * (8.25 ms) version saturated on board #2 at exactly the rows that mattered -- 2 Mbaud
 * bidirectional runs at ~17 ms reported "p99 <= 8.250", i.e. a number BELOW their own
 * minimum, because every sample landed in the overflow bucket.  A percentile that fails
 * silently, and fails low, is worse than no percentile.  128 buckets reach 32 ms, which
 * covers 1536 B in both directions at 2 Mbaud with room to spare, and saturation is now
 * reported as such.  Cost: 512 bytes.
 */
#define LINK_HIST_N       128u
#define LINK_HIST_US      250u

struct link_run {
	uint32_t frames;
	uint32_t tx_bytes, rx_bytes;     /* payload only, both directions */
	uint32_t rtt_min, rtt_max;       /* microseconds */
	uint64_t rtt_sum;
	uint32_t hist[LINK_HIST_N];
	uint32_t elapsed_ms;
};

/*
 * Bench payload staging.  erpc_ctrl_call() copies the request into the CTRL slot, so this
 * only has to live for the duration of the call -- but a 1.5 kB frame would not fit on a
 * CLI instance stack at all (CLI_INSTANCE_STACK_SIZE = 2048).
 */
/* LINK_STATS reply: this many u32 LE, in the order link_print_module() prints them. */
#define LINK_STATS_WORDS  12u

static uint8_t link_req[8u + LINK_BENCH_MAX];
static uint8_t link_reply[64];           /* CTRL replies are small; bench data is dropped */
_Static_assert(sizeof(link_reply) >= LINK_STATS_WORDS * 4u, "LINK_STATS reply must fit");

/*
 * The run accumulator, static for the same reason and with more force: a CLI instance
 * stack is CLI_INSTANCE_STACK_SIZE = 2048 bytes, and the 128-bucket histogram alone is
 * 512 of them.  Every `link` subcommand holds the console claim and the coarse link
 * mutex, so exactly one of them can be running at a time.
 */
static struct link_run link_run;

/* ---- helpers ------------------------------------------------------------- */

/* The same generator the module runs, so both ends produce identical bytes from a seed.
 * Its only job is to put real bit transitions on the wire -- integrity is checked by the
 * frame CRC, which is why neither end compares the bytes. */
static void link_pattern(uint8_t *buf, uint32_t n, uint8_t seed)
{
	uint32_t x = 0x12345678u ^ (uint32_t)seed;
	uint32_t i;

	for (i = 0u; i < n; i++) {
		x = x * 1103515245u + 12345u;
		buf[i] = (uint8_t)(x >> 24);
	}
}

static uint32_t link_now(void)  { return TIM2->CNT; }

/* TIM2 delta in microseconds.  Wrap-safe by unsigned subtraction; a single round trip is
 * milliseconds, far inside the ~15.6 s period. */
static uint32_t link_elapsed_us(uint32_t t0)
{
	return (uint32_t)(link_now() - t0) / LINK_TIM_PER_US;
}

/* Print @us as "N.MMM ms" -- there is no float in this printf. */
static void link_put_ms(struct cli_instance *sh, const char *label, uint32_t us)
{
	cli_print(sh, "%s%lu.%03lu", label, (unsigned long)(us / 1000u),
	          (unsigned long)(us % 1000u));
}

/* ---- the CTRL gate ------------------------------------------------------- */

/*
 * Every subcommand that speaks to the module needs all of this to hold.  Returns 0 when
 * it does; otherwise prints why and returns non-zero.  Caller already holds the session
 * (rtl_link_begin), so its own UART reference is the 1 that is expected.
 */
static int link_ctrl_ready_ex(struct cli_instance *sh, int allow_busy, int allow_bridged)
{
	/*
	 * THE BRIDGE HAS TO BE DOWN, for everything except the read-only `info`.
	 *
	 * Until issue #30 B2c this was nx_net_guard() -- refuse whenever the host stack was
	 * up -- and the reason given was ownership.  The real reason is narrower and it is
	 * about TIME and about the DATA channel: a bench holds the coarse mutex for seconds
	 * at a stretch, and the interface owner needs that mutex every NXN_REFRESH_MS or the
	 * module drops the tap; `dbench` additionally wants the DATA channel, which has
	 * exactly one consumer and it is the NetX driver.  Neither applies to `info`.
	 *
	 * Since the bridge is armed by `wifi connect` (B2b), the window for the diagnostics
	 * is "this power-on, before associating" -- which is a state, not a mode, and the way
	 * back to it is the module's own reset.
	 */
	if (!allow_bridged && nx_net_state() != NX_NET_OFF) {
		cli_error(sh, "link: refused -- the L2 bridge is up (state %s)\r\n",
		          nx_net_state_name(nx_net_state()));
		cli_print(sh, "  the link diagnostics need it down: `wifi reset`, then "
		          "`wifi ver` and run this before `wifi connect`\r\n");
		return 1;
	}
	if (erpc_module_gen() < 5u) {
		cli_error(sh, "link: this needs firmware 2.1.3+wio-n5 or later, and the host "
		          "does not know which is loaded\r\n");
		cli_print(sh, "  run `wifi ver` first (it is what proves the firmware; the "
		          "answer is dropped again by any `wifi reset` or flash session)\r\n");
		return 1;
	}
	/*
	 * `link dbench` deliberately runs with the link BUSY (issue #23 U1), and `link info`
	 * runs with the bridge up (B2c) -- where the owner's reference makes the count 2 and
	 * a DATA frame may well be in flight.  Both are safe for the same reason: the frame
	 * reader demultiplexes by frame type and the CTRL slot is independent of the eRPC
	 * slots, which is also why the owner itself refreshes over CTRL while bridged.
	 * Everything else inherits the U0 rule that CTRL is only issued on a quiescent link.
	 */
	if (allow_busy || allow_bridged)
		return 0;
	if (rtl_link_uart_refs() != 1u) {
		cli_error(sh, "link: something else is holding the eRPC link\r\n");
		cli_print(sh, "  LINK-CTRL is only sent on a quiescent link -- stop the telnet "
		          "console first (`net shell stop`)\r\n");
		return 1;
	}
	if (!erpc_link_quiescent()) {
		cli_error(sh, "link: the link is not idle (a request is still outstanding)\r\n");
		return 1;
	}
	return 0;
}

static int link_ctrl_ready(struct cli_instance *sh)
{
	return link_ctrl_ready_ex(sh, 0, 0);
}

/* ---- CTRL wrappers ------------------------------------------------------- */

/* Module-side counters.  Returns 0 and fills @st, or -1 (message printed). */
static int link_get_stats(struct cli_instance *sh, uint32_t st[LINK_STATS_WORDS])
{
	struct erpc_diag diag = {0};
	int n = erpc_ctrl_call(ERPC_CTRL_STATS, NULL, 0u, link_reply,
	                       (uint16_t)sizeof(link_reply), LINK_CTRL_TMO_MS, &diag);
	unsigned i;

	if (n < 0) {
		cli_error(sh, "link: LINK_STATS failed (rc %d)\r\n", n);
		return -1;
	}
	if (n < (int)(LINK_STATS_WORDS * 4u)) {
		cli_error(sh, "link: LINK_STATS returned %d B, expected %u -- is the module "
		          "running an older wio-n5 build?\r\n", n, (unsigned)(LINK_STATS_WORDS * 4u));
		return -1;
	}
	for (i = 0u; i < LINK_STATS_WORDS; i++)
		st[i] = erpc_get_u32le(link_reply + i * 4u);
	return 0;
}

/* ---- link info ----------------------------------------------------------- */

/* What the divider actually produces, and by how much it misses.  RM0468 sec 53.5.7:
 * with OVER8 = 0 the baud is usart_ker_ck / BRR, and app/rtl8720.c programs exactly the
 * rounded divider computed here. */
static void link_print_rate(struct cli_instance *sh, uint32_t baud)
{
	uint32_t brr = (LINK_PCLK2 + baud / 2u) / baud;
	uint32_t eff = LINK_PCLK2 / brr;
	int32_t  err_ppm;
	int      neg;

	err_ppm = (int32_t)(((int64_t)eff - (int64_t)baud) * 1000000 / (int64_t)baud);
	neg = (err_ppm < 0);
	if (neg)
		err_ppm = -err_ppm;
	cli_print(sh, "  rate: %lu baud, BRR %lu -> %lu actual (%s%ld.%03ld %%)\r\n",
	          (unsigned long)baud, (unsigned long)brr, (unsigned long)eff,
	          neg ? "-" : "+", (long)(err_ppm / 10000), (long)(err_ppm % 10000 / 10));
}

static void link_print_host(struct cli_instance *sh)
{
	struct rtl8720_uart_stats st = {0};
	uint32_t cyc_per_us = SystemCoreClock / 1000000u;

	if (cyc_per_us == 0u)
		cyc_per_us = 1u;
	rtl8720_uart_stats(&st);
	link_print_rate(sh, rtl_link_erpc_baud());
	cli_print(sh, "  host rx: %lu irq, max %lu/%lu B per irq, isr %lu.%lu us\r\n",
	          (unsigned long)st.isr_count, (unsigned long)st.isr_max_bytes,
	          (unsigned long)st.isr_grace,
	          (unsigned long)(st.isr_max_cycles / cyc_per_us),
	          (unsigned long)((st.isr_max_cycles % cyc_per_us) * 10u / cyc_per_us));
	cli_print(sh, "  host err: ore %lu framing %lu ring-drops %lu (ring %lu B)%s\r\n",
	          (unsigned long)st.ore, (unsigned long)st.ferr, (unsigned long)st.drops,
	          (unsigned long)st.ring_size,
	          (st.ore || st.ferr || st.drops) ? "  <-- NOT CLEAN" : "  (clean)");
}

static void link_print_module(struct cli_instance *sh, const uint32_t st[LINK_STATS_WORDS])
{
	/* `entry` is the number that matters: FIFO occupancy when the interrupt STARTED, out
	 * of the (64 - trigger) it may consume before the hardware overruns.  `burst` is the
	 * bytes one interrupt took in total, which also counts arrivals during the drain --
	 * those are free, so a high burst with a low entry is healthy. */
	cli_print(sh, "  mod rx: %lu irq, %lu B, entry %lu/%lu grace, burst %lu/64 per irq\r\n",
	          (unsigned long)st[0], (unsigned long)st[1], (unsigned long)st[10],
	          (unsigned long)(64u - st[11]), (unsigned long)st[2]);
	cli_print(sh, "  mod err: fifo-overrun %lu framing %lu ring-drops %lu "
	          "(ring peak %lu/%lu B)%s\r\n",
	          (unsigned long)st[5], (unsigned long)st[6], (unsigned long)st[4],
	          (unsigned long)st[3], (unsigned long)st[9],
	          (st[4] || st[5] || st[6]) ? "  <-- NOT CLEAN" : "  (clean)");
	cli_print(sh, "  mod link: %lu baud, free heap %lu B\r\n",
	          (unsigned long)st[8], (unsigned long)st[7]);
}

static int cmd_link_info(struct cli_instance *sh, int argc, char **argv)
{
	uint32_t st[LINK_STATS_WORDS];
	int rc;

	(void)argc; (void)argv;

	/* The host half needs no module at all, so it is reported even with the RTL8720
	 * powered off -- that is exactly when you want to see why. */
	rc = rtl_link_begin(sh, false);
	if (rc == RTL_LINK_OFF) {
		cli_print(sh, "link: RTL8720DN is powered off -- host side only\r\n");
		link_print_host(sh);
		return 0;
	}
	if (rc != RTL_LINK_READY)
		return 1;

	cli_print(sh, "link: USART1 <-> RTL8720DN USI0\r\n");
	link_print_host(sh);

	/* The one subcommand that stays available while the bridge is up: `ore`, ring drops
	 * and FIFO overruns are worth most exactly when traffic is flowing, and reading them
	 * is one short CTRL round trip -- the same one the interface owner already makes
	 * every 8 s. */
	if (link_ctrl_ready_ex(sh, 0, 1) == 0 && link_get_stats(sh, st) == 0)
		link_print_module(sh, st);

	rtl_link_end(sh);
	return 0;
}

/* ---- link baud ----------------------------------------------------------- */

/*
 * Change the rate of the link.  The sequence itself -- ping, LINK_SETBAUD, re-open,
 * verify, best-effort fall back -- lives in rtl_link_set_rate() (app/rtl_link.h), because
 * since issue #23 U4-3 `net up` raises the rate too and two implementations of something
 * this delicate would eventually disagree.  What stays here is the part that is genuinely
 * this command's: argument checking, the session, and saying what happened.
 */
static int cmd_link_baud(struct cli_instance *sh, int argc, char **argv)
{
	uint32_t baud, old;
	int rc;

	if (cli_parse_u32(argv[1], &baud) != 0) {
		cli_error(sh, "usage: wifi link baud <2000000|3000000|4000000|6000000>\r\n");
		return 1;
	}
	if (!rtl_link_rate_supported(baud)) {
		cli_error(sh, "link: %lu is not one of the supported rates "
		          "(2000000 / 3000000 / 4000000 / 6000000)\r\n", (unsigned long)baud);
		return 1;
	}

	if (rtl_link_begin(sh, false) != RTL_LINK_READY)
		return 1;
	if (link_ctrl_ready(sh) != 0) {
		rtl_link_end(sh);
		return 1;
	}
	old = rtl_link_erpc_baud();
	if (baud == old) {
		cli_print(sh, "link: already at %lu baud\r\n", (unsigned long)old);
		rtl_link_end(sh);
		return 0;
	}

	cli_print(sh, "link: %lu -> %lu baud...\r\n", (unsigned long)old,
	          (unsigned long)baud);
	rc = rtl_link_set_rate(baud);
	rtl_link_end(sh);

	if (rc == RTL_RATE_OK) {
		cli_print(sh, "link: now at %lu baud (LINK_PING answered)\r\n",
		          (unsigned long)baud);
		link_print_rate(sh, baud);
		return 0;
	}
	if (rc == RTL_RATE_UNCHANGED) {
		cli_warn(sh, "link: the rate was not changed -- still at %lu baud\r\n",
		         (unsigned long)rtl_link_erpc_baud());
		return 1;
	}
	cli_error(sh, "link: the link is down at both rates -- run `wifi reset` to "
	          "power-cycle the module (it always comes back at 2000000)\r\n");
	return 1;
}

/* ---- link bench ---------------------------------------------------------- */

static void link_run_init(struct link_run *r)
{
	memset(r, 0, sizeof(*r));
	r->rtt_min = 0xFFFFFFFFu;
}

static void link_run_add(struct link_run *r, uint32_t us, uint32_t tx, uint32_t rx)
{
	uint32_t b = us / LINK_HIST_US;

	r->frames++;
	r->tx_bytes += tx;
	r->rx_bytes += rx;
	r->rtt_sum  += us;
	if (us < r->rtt_min) r->rtt_min = us;
	if (us > r->rtt_max) r->rtt_max = us;
	r->hist[(b < LINK_HIST_N) ? b : (LINK_HIST_N - 1u)]++;
}

/*
 * Upper edge of the bucket holding the 99th percentile (resolution LINK_HIST_US).
 * Returns 0 and sets *@saturated when it falls in the overflow bucket, i.e. when the
 * histogram cannot express the answer -- the caller must say so rather than print the
 * range's top edge as if it were a measurement.
 */
static uint32_t link_run_p99(const struct link_run *r, int *saturated)
{
	uint32_t target = (r->frames * 99u) / 100u;
	uint32_t seen = 0u, i;

	*saturated = 0;
	if (r->frames == 0u)
		return 0u;
	for (i = 0u; i + 1u < LINK_HIST_N; i++) {
		seen += r->hist[i];
		if (seen >= target)
			return (i + 1u) * LINK_HIST_US;
	}
	*saturated = 1;
	return (LINK_HIST_N - 1u) * LINK_HIST_US;
}

static void link_run_report(struct cli_instance *sh, const char *dir,
                            uint32_t bytes, const struct link_run *r)
{
	uint64_t total = (uint64_t)r->tx_bytes + (uint64_t)r->rx_bytes;
	uint32_t kbs_x10 = 0u;

	if (r->frames == 0u) {
		cli_error(sh, "  %-4s %4lu B: no frames completed\r\n", dir,
		          (unsigned long)bytes);
		return;
	}
	if (r->elapsed_ms != 0u)
		kbs_x10 = (uint32_t)(total * 10000u / r->elapsed_ms / 1024u);

	cli_print(sh, "  %-4s %4lu B: %lu frames, %lu KB in %lu ms = %lu.%lu KB/s",
	          dir, (unsigned long)bytes, (unsigned long)r->frames,
	          (unsigned long)(total / 1024u), (unsigned long)r->elapsed_ms,
	          (unsigned long)(kbs_x10 / 10u), (unsigned long)(kbs_x10 % 10u));
	link_put_ms(sh, " | rtt ms min ", r->rtt_min);
	link_put_ms(sh, " avg ", (uint32_t)(r->rtt_sum / r->frames));
	{
		int saturated;
		uint32_t p99 = link_run_p99(r, &saturated);

		link_put_ms(sh, saturated ? " p99>" : " p99<=", p99);
	}
	link_put_ms(sh, " max ", r->rtt_max);
	cli_print(sh, "\r\n");
}

/*
 * One direction, for @secs seconds.  Each iteration is one CTRL round trip:
 *   dir rx   8-byte request, @bytes of generated pattern back
 *   dir tx   @bytes out, 4-byte acknowledgement back
 *   dir both @bytes each way
 * Returns 0, or -1 once a round trip fails (the partial result is still reported by the
 * caller, which is the point: a failure at 6 Mbaud IS the measurement).
 */
static int link_bench_run(struct cli_instance *sh, uint32_t bytes, uint32_t secs,
                          uint32_t want_tx, uint32_t want_rx, struct link_run *r)
{
	uint32_t t_start = HAL_GetTick();
	uint8_t seed = 1u;
	int rc = 0;

	link_run_init(r);
	for (;;) {
		struct erpc_diag diag = {0};
		uint32_t t0, elapsed;
		uint16_t req_len;
		int n;

		elapsed = (uint32_t)(HAL_GetTick() - t_start);
		if (elapsed >= secs * 1000u)
			break;
		if (cli_cancel_requested(sh))
			break;

		erpc_put_u32le(link_req + 0, want_rx);
		link_req[4] = seed++;
		link_req[5] = 0u;
		link_req[6] = 0u;
		link_req[7] = 0u;
		if (want_tx != 0u)
			link_pattern(link_req + 8, want_tx, seed);
		req_len = (uint16_t)(8u + want_tx);

		t0 = link_now();
		n = erpc_ctrl_call(ERPC_CTRL_BENCH, link_req, req_len, link_reply,
		                   (uint16_t)sizeof(link_reply), LINK_BENCH_TMO_MS, &diag);
		if (n < 0) {
			cli_error(sh, "  bench: round trip failed after %lu frames (rc %d, "
			          "crc %u stall %u ctrl_bad %u)\r\n",
			          (unsigned long)r->frames, n, diag.crc_fail, diag.frame_stall,
			          diag.ctrl_bad);
			rc = -1;
			break;
		}
		/* The module echoes how many bytes it received; a mismatch means the request
		 * lost its tail, which is precisely the failure this link had before U0-2. */
		if (n >= 4 && erpc_get_u32le(link_reply) != want_tx) {
			cli_error(sh, "  bench: module saw %lu of %lu request bytes\r\n",
			          (unsigned long)erpc_get_u32le(link_reply), (unsigned long)want_tx);
			rc = -1;
			break;
		}
		link_run_add(r, link_elapsed_us(t0), want_tx, want_rx);
	}
	r->elapsed_ms = (uint32_t)(HAL_GetTick() - t_start);
	(void)bytes;
	return rc;
}

/* Parse a direction word into the two payload sizes.  Returns 0 on success. */
static int link_dir_sizes(const char *dir, uint32_t bytes, uint32_t *tx, uint32_t *rx)
{
	if (strcmp(dir, "rx") == 0)   { *tx = 0u;    *rx = bytes; return 0; }
	if (strcmp(dir, "tx") == 0)   { *tx = bytes; *rx = 0u;    return 0; }
	if (strcmp(dir, "both") == 0) { *tx = bytes; *rx = bytes; return 0; }
	return -1;
}

static int cmd_link_bench(struct cli_instance *sh, int argc, char **argv)
{
	uint32_t bytes = LINK_BENCH_DEF, secs = LINK_BENCH_SECS, tx = 0u, rx = 0u;
	const char *dir = "both";
	int all;

	if (argc > 1 && (cli_parse_u32(argv[1], &bytes) != 0 || bytes == 0u ||
	                     bytes > LINK_BENCH_MAX)) {
		cli_error(sh, "link: bad size (1..%u)\r\n", (unsigned)LINK_BENCH_MAX);
		return 1;
	}
	if (argc > 2 && (cli_parse_u32(argv[2], &secs) != 0 || secs == 0u ||
	                     secs > LINK_BENCH_MAX_S)) {
		cli_error(sh, "link: bad duration (1..%u s)\r\n", (unsigned)LINK_BENCH_MAX_S);
		return 1;
	}
	if (argc > 3)
		dir = argv[3];
	all = (strcmp(dir, "all") == 0);
	if (!all && link_dir_sizes(dir, bytes, &tx, &rx) != 0) {
		cli_error(sh, "link: direction must be rx, tx, both or all\r\n");
		return 1;
	}

	if (rtl_link_begin(sh, false) != RTL_LINK_READY)
		return 1;
	if (link_ctrl_ready(sh) != 0) {
		rtl_link_end(sh);
		return 1;
	}

	if (all) {
		/*
		 * All three directions back to back at the CURRENT rate (this absorbed the
		 * old `link sweep`).  It deliberately does NOT walk the baud rates: changing
		 * the rate is the one operation whose failure needs a power cycle to undo,
		 * and burying it inside a loop makes it much harder to say afterwards which
		 * rate was being set when something stopped answering.  Run
		 * `wifi link baud <b>` between runs.
		 */
		static const char *const dirs[] = { "rx", "tx", "both" };
		uint32_t st0[LINK_STATS_WORDS], st1[LINK_STATS_WORDS];
		int have_st0;
		unsigned i;

		cli_print(sh, "link: bench all at %lu baud, %lu B payload, %lu s per direction\r\n",
		          (unsigned long)rtl_link_erpc_baud(), (unsigned long)bytes,
		          (unsigned long)secs);
		have_st0 = (link_get_stats(sh, st0) == 0);
		for (i = 0u; i < sizeof(dirs) / sizeof(dirs[0]); i++) {
			if (cli_cancel_requested(sh))
				break;
			(void)link_dir_sizes(dirs[i], bytes, &tx, &rx);
			(void)link_bench_run(sh, bytes, secs, tx, rx, &link_run);
			link_run_report(sh, dirs[i], bytes, &link_run);
		}
		link_print_host(sh);
		if (have_st0 && link_get_stats(sh, st1) == 0) {
			cli_print(sh, "  mod delta: irq %lu, bytes %lu, drops %lu, "
			          "fifo-overrun %lu, framing %lu\r\n",
			          (unsigned long)(st1[0] - st0[0]),
			          (unsigned long)(st1[1] - st0[1]),
			          (unsigned long)(st1[4] - st0[4]),
			          (unsigned long)(st1[5] - st0[5]),
			          (unsigned long)(st1[6] - st0[6]));
			link_print_module(sh, st1);
		}
		rtl_link_end(sh);
		return 0;
	}

	cli_print(sh, "link: bench %s %lu B for %lu s at %lu baud...\r\n", dir,
	          (unsigned long)bytes, (unsigned long)secs,
	          (unsigned long)rtl_link_erpc_baud());
	(void)link_bench_run(sh, bytes, secs, tx, rx, &link_run);
	/* Report as soon as the run ends: after a Ctrl+C the core discards handler output
	 * (cli_core.c), so anything held back to a final summary would simply vanish. */
	link_run_report(sh, dir, bytes, &link_run);

	rtl_link_end(sh);
	return 0;
}

/* ---- link dbench (issue #23 U1: the DATA channel) ------------------------- */

/*
 * `link bench` measures the CTRL channel, which is a REQUEST/REPLY exchange: the host
 * sends one frame and waits.  That is not how Ethernet will use this link, so it cannot
 * answer the question U1 exists to answer -- can DATA, eRPC and CTRL interleave
 * continuously without losing anything?  `link dbench` runs the DATA channel free: both
 * ends push frames as fast as their queues accept, nobody waits for anybody, and loss is
 * detected by a sequence number inside the payload rather than by a missing reply.
 *
 * A gap in that sequence is the ground truth for "the multiplexer dropped something".
 * The link has no retransmission, and one lost BYTE costs a resynchronisation -- i.e.
 * several frames -- so gaps are also how a byte-level failure becomes visible.
 */
#define LINK_DBENCH_MIN   64u            /* below this the framing dominates */

/* Payload: u32 sequence, then filler.  The frame CRC already proves integrity, so the
 * filler is only there to put real transitions on the wire. */
#define LINK_DBENCH_HDR   4u

/* DATA_STATS reply: u32 LE, in the order link_print_dstats() prints them. */
#define LINK_DSTAT_WORDS  12u

struct link_dbench {
	volatile uint32_t rx_frames;
	volatile uint32_t rx_bytes;
	volatile uint32_t rx_gaps;       /* sequence discontinuities = frames lost */
	volatile uint32_t rx_badlen;
	volatile uint32_t next_seq;
	volatile uint8_t  started;
	uint32_t tx_frames;
	uint32_t tx_bytes;
	uint32_t tx_drops;
	uint32_t elapsed_ms;
};
static struct link_dbench link_db;

/* Transmit staging, static for the same reason as link_req: 1.5 kB will not fit on a CLI
 * instance stack (CLI_INSTANCE_STACK_SIZE = 2048). */
static uint8_t link_dtx[LINK_DATA_PAYLOAD_MAX];

/*
 * Runs ON THE LINK SERVICE THREAD (app/erpc.c), once per received DATA frame, with no
 * link lock held.  It must be short -- while it runs nothing is draining the UART ring --
 * so it only counts.  Returns 0: the buffer is handed straight back to the pool.
 */
static int link_dbench_rx(void *ctx, uint8_t chan, uint8_t *p, uint16_t n)
{
	struct link_dbench *d = (struct link_dbench *)ctx;
	uint32_t seq;

	(void)chan;
	if (n < LINK_DBENCH_HDR) {
		d->rx_badlen++;
		return 0;
	}
	seq = erpc_get_u32le(p);
	if (!d->started) {
		d->started = 1u;                 /* first frame defines the origin */
	} else if (seq != d->next_seq) {
		/* Wrap-safe forward difference; a repeat or a reorder cannot happen on a
		 * single ordered wire, so anything but "the next one" is loss. */
		d->rx_gaps += (uint32_t)(seq - d->next_seq);
	}
	d->next_seq = seq + 1u;
	d->rx_frames++;
	d->rx_bytes += n;
	return 0;
}

/* Tell the module what to do with the DATA channel.  @mode 0 stops it, and its
 * acknowledgement means the module's own transmit queue is drained (see erpc.h). */
static int link_data_cfg(struct cli_instance *sh, uint8_t mode, uint16_t bytes,
                           uint32_t ms, uint8_t seed)
{
	struct erpc_diag diag = {0};
	uint8_t req[12];
	int n;

	req[0] = mode;
	req[1] = seed;
	req[2] = (uint8_t)bytes;
	req[3] = (uint8_t)(bytes >> 8);
	erpc_put_u32le(req + 4, ms);
	erpc_put_u32le(req + 8, ERPC_CTRL_DATA_MAGIC);
	n = erpc_ctrl_call(ERPC_CTRL_DATA_CFG, req, (uint16_t)sizeof(req), link_reply,
	                   (uint16_t)sizeof(link_reply), LINK_DCFG_TMO_MS, &diag);
	if (n < 0) {
		cli_error(sh, "link: LINK_DATA_CFG(%u) failed (rc %d)\r\n", mode, n);
		/*
		 * -3 is "the module refused", which on its own says nothing about WHY -- and a
		 * bridge can refuse for several reasons, one of which is the first thing a new
		 * user does wrong.  Decode it (module statuses are in fw/rtl8720 wio_link_eth.h).
		 */
		if (n == -3) {
			switch (erpc_ctrl_last_status()) {
			case 3u:
				cli_print(sh, "  the module has no TCP/IP stack yet -- run "
				          "`wifi connect <ssid> ...` first (it is what brings "
				          "lwIP up, and the bridge taps its interface)\r\n");
				break;
			case 2u:
				cli_print(sh, "  the module's bridge never initialised (out of "
				          "memory at boot) -- `wifi reset`\r\n");
				break;
			case 4u:
			case 5u:
				cli_print(sh, "  the module's TCP/IP thread did not answer; nothing "
				          "was changed -- retry, then `wifi reset`\r\n");
				break;
			default:
				cli_print(sh, "  the module rejected the request itself "
				          "(status %u)\r\n", erpc_ctrl_last_status());
				break;
			}
		}
		return -1;
	}
	return 0;
}

static int link_get_dstats(struct cli_instance *sh, uint32_t st[LINK_DSTAT_WORDS])
{
	struct erpc_diag diag = {0};
	int n = erpc_ctrl_call(ERPC_CTRL_DATA_STATS, NULL, 0u, link_reply,
	                       (uint16_t)sizeof(link_reply), LINK_CTRL_TMO_MS, &diag);
	unsigned i;

	if (n < (int)(LINK_DSTAT_WORDS * 4u)) {
		cli_error(sh, "link: LINK_DATA_STATS failed (rc %d)\r\n", n);
		return -1;
	}
	for (i = 0u; i < LINK_DSTAT_WORDS; i++)
		st[i] = erpc_get_u32le(link_reply + i * 4u);
	return 0;
}

static void link_print_dstats(struct cli_instance *sh, const uint32_t st[LINK_DSTAT_WORDS])
{
	cli_print(sh, "  mod data rx: %lu frames, %lu B, drops %lu, crc %lu, oversize %lu, "
	          "gaps %lu%s\r\n",
	          (unsigned long)st[0], (unsigned long)st[1], (unsigned long)st[2],
	          (unsigned long)st[3], (unsigned long)st[4], (unsigned long)st[5],
	          (st[2] || st[3] || st[4] || st[5]) ? "  <-- LOSS" : "  (clean)");
	cli_print(sh, "  mod data tx: %lu frames, %lu B, drops %lu (queued %lu, in-use %lu)\r\n",
	          (unsigned long)st[6], (unsigned long)st[7], (unsigned long)st[8],
	          (unsigned long)st[9], (unsigned long)st[10]);
}

static void link_print_dhost(struct cli_instance *sh)
{
	struct link_data_stats ds;

	link_data_stats(&ds);
	cli_print(sh, "  host data rx: %lu frames, %lu B, drops %lu, crc %lu, oversize %lu, "
	          "gaps %lu%s\r\n",
	          (unsigned long)ds.rx_frames, (unsigned long)ds.rx_bytes,
	          (unsigned long)ds.rx_drops, (unsigned long)ds.rx_crc_err,
	          (unsigned long)ds.rx_oversize, (unsigned long)link_db.rx_gaps,
	          (ds.rx_drops || ds.rx_crc_err || ds.rx_oversize || link_db.rx_gaps)
	                  ? "  <-- LOSS" : "  (clean)");
	/* no-buf is NOT loss here: the bench offers frames as fast as it can and a full pool
	 * is the link telling it to wait, which it does.  A frame counted here was never
	 * queued, so it never entered the sequence the far end checks. */
	cli_print(sh, "  host data tx: %lu frames, %lu B, no-buf %lu (queued %lu, rx-in-use %lu)\r\n",
	          (unsigned long)ds.tx_frames, (unsigned long)ds.tx_bytes,
	          (unsigned long)ds.tx_drops, (unsigned long)ds.tx_queued,
	          (unsigned long)ds.rx_inuse);
}

static void link_dbench_report(struct cli_instance *sh, const char *dir, uint32_t bytes)
{
	uint64_t total = (uint64_t)link_db.tx_bytes + (uint64_t)link_db.rx_bytes;
	uint32_t kbs_x10 = 0u;

	if (link_db.elapsed_ms != 0u)
		kbs_x10 = (uint32_t)(total * 10000u / link_db.elapsed_ms / 1024u);
	cli_print(sh, "  %-4s %4lu B: tx %lu / rx %lu frames, %lu KB in %lu ms = %lu.%lu KB/s"
	          " (waited %lu)\r\n",
	          dir, (unsigned long)bytes, (unsigned long)link_db.tx_frames,
	          (unsigned long)link_db.rx_frames, (unsigned long)(total / 1024u),
	          (unsigned long)link_db.elapsed_ms,
	          (unsigned long)(kbs_x10 / 10u), (unsigned long)(kbs_x10 % 10u),
	          (unsigned long)link_db.tx_drops);
}

/*
 * Wait for BOTH ends to be quiet before the channel is detached.  This is the ordering
 * rule from app/link_data.h, and the reason it is a state test rather than a delay: the
 * flush suppression that keeps DATA frames intact is lifted the instant the consumer
 * detaches, so a frame still on the wire at that moment would be thrown away mid-body
 * and desynchronise the reader.  The module's side is proved by the CFG(off)
 * acknowledgement plus its own queue/pool counters; the host's by erpc_data_quiescent().
 *
 * It asks about the DATA CHANNEL, not about the link.  The first version of this used
 * erpc_link_quiescent(), which additionally requires that no eRPC request be outstanding
 * -- and the telnet console keeps a blocking accept outstanding for as long as it is
 * armed (issue #21), so with a console up this could never succeed no matter how quiet
 * the DATA channel was.  Measured on board #2: the module reported queue 0 / in-use 0
 * (its promise kept) while this still timed out for a full second.
 */
static int link_data_settle(struct cli_instance *sh)
{
	uint32_t st[LINK_DSTAT_WORDS];
	int tries;

	for (tries = 0; tries < 50; tries++) {          /* 50 x 20 ms = 1 s */
		if (erpc_data_quiescent() && link_get_dstats(sh, st) == 0 &&
		    st[9] == 0u && st[10] == 0u)
			return 0;
		cli_sleep(sh, 20u);
	}
	cli_error(sh, "link: the DATA channel did not go quiet -- `wifi reset` before using "
	          "the link again\r\n");
	return -1;
}

static int cmd_link_dbench(struct cli_instance *sh, int argc, char **argv)
{
	uint32_t bytes = LINK_BENCH_DEF, secs = LINK_BENCH_SECS, tx, rx;
	uint32_t st[LINK_DSTAT_WORDS], t_start;
	const char *dir = "both";
	uint8_t mode;
	int rc = 0;

	if (argc > 1 && (cli_parse_u32(argv[1], &bytes) != 0 || bytes < LINK_DBENCH_MIN ||
	                     bytes > LINK_DATA_PAYLOAD_MAX)) {
		cli_error(sh, "link: bad size (%u..%u)\r\n", (unsigned)LINK_DBENCH_MIN,
		          (unsigned)LINK_DATA_PAYLOAD_MAX);
		return 1;
	}
	if (argc > 2 && (cli_parse_u32(argv[2], &secs) != 0 || secs == 0u ||
	                     secs > LINK_BENCH_MAX_S)) {
		cli_error(sh, "link: bad duration (1..%u s)\r\n", (unsigned)LINK_BENCH_MAX_S);
		return 1;
	}
	if (argc > 3)
		dir = argv[3];
	if (link_dir_sizes(dir, bytes, &tx, &rx) != 0) {
		cli_error(sh, "link: direction must be rx, tx or both\r\n");
		return 1;
	}

	if (rtl_link_begin(sh, false) != RTL_LINK_READY)
		return 1;
	if (link_ctrl_ready_ex(sh, 1, 0) != 0) {    /* runs with the link busy -- on purpose */
		rtl_link_end(sh);
		return 1;
	}
	if (erpc_module_gen() < 6u) {
		cli_error(sh, "link: the DATA channel needs firmware 2.1.3+wio-n6 or later\r\n");
		rtl_link_end(sh);
		return 1;
	}

	memset(&link_db, 0, sizeof(link_db));
	memset(link_dtx, 0x5A, sizeof(link_dtx));
	if (link_data_attach(link_dbench_rx, &link_db) != 0) {
		cli_error(sh, "link: the DATA channel already has a consumer\r\n");
		rtl_link_end(sh);
		return 1;
	}

	/* The module sinks what we send and sources what we want back.  Its source stops
	 * on its own after the requested time as well as on CFG(off), so a host that dies
	 * mid-run cannot leave it transmitting forever. */
	mode = (uint8_t)((tx != 0u ? ERPC_DATA_MODE_SINK : 0u) |
	                 (rx != 0u ? ERPC_DATA_MODE_SOURCE : 0u));
	cli_print(sh, "link: dbench %s %lu B for %lu s at %lu baud...\r\n", dir,
	          (unsigned long)bytes, (unsigned long)secs,
	          (unsigned long)rtl_link_erpc_baud());
	if (link_data_cfg(sh, mode, (uint16_t)rx, secs * 1000u + 500u, 0x5Au) != 0) {
		rtl_link_end(sh);                /* closes the UART: nothing can arrive now */
		link_data_detach();
		return 1;
	}

	t_start = HAL_GetTick();
	for (;;) {
		uint32_t elapsed = (uint32_t)(HAL_GetTick() - t_start);

		if (elapsed >= secs * 1000u || cli_cancel_requested(sh))
			break;
		if (tx == 0u) {
			cli_sleep(sh, 10u);      /* receive-only: just let the run elapse */
			continue;
		}
		erpc_put_u32le(link_dtx, link_db.tx_frames);
		if (link_data_send(LINK_DATA_CHAN_BENCH, link_dtx, (uint16_t)tx) == 0) {
			link_db.tx_frames++;
			link_db.tx_bytes += tx;
		} else {
			/* Pool full: the service thread is still writing.  This is
			 * BACKPRESSURE, not loss -- the frame was never given a sequence
			 * number, so the far end cannot miss it.  Yielding here is what keeps
			 * this loop (priority 16) from starving the thread that drains it. */
			link_db.tx_drops++;
			cli_sleep(sh, 1u);
		}
	}
	link_db.elapsed_ms = (uint32_t)(HAL_GetTick() - t_start);

	/* Report before anything else can fail: after a Ctrl+C the core discards handler
	 * output, so a summary held back to the end would simply vanish (issue #16). */
	link_dbench_report(sh, dir, bytes);

	/* Stop the far end and prove both ends are quiet BEFORE reading the counters, so
	 * what is printed is a finished run rather than a moving one. */
	if (link_data_cfg(sh, ERPC_DATA_MODE_OFF, 0u, 0u, 0u) != 0)
		rc = 1;
	else if (link_data_settle(sh) != 0)
		rc = 1;

	if (link_get_dstats(sh, st) == 0)
		link_print_dstats(sh, st);
	link_print_dhost(sh);
	link_print_host(sh);

	/*
	 * Order matters.  Detaching restores the pre-send RX flush, so it must not happen
	 * while a DATA frame could still arrive.  TWO things make that true, and which one
	 * applies depends on whether anything else is holding the link:
	 *   - the settle above, which proves the module has stopped (its CFG(off)
	 *     acknowledgement is a promise it has drained its own writer); and
	 *   - rtl_link_end(), which drops OUR reference -- and if it was the last one the
	 *     UART closes outright, so nothing can arrive at all.
	 * With a telnet console resident the reference count does not reach zero and only
	 * the first applies, which is exactly why settle is a state test and not a delay.
	 * If it timed out we detach anyway and say so: a broken promise costs a
	 * resynchronisation (the reader's CRC and overflow detectors handle it), and
	 * refusing to detach would strand the channel with no owner.
	 */
	rtl_link_end(sh);
	link_data_detach();
	return rc;
}

/*
 * `wifi link arp` lived here until issue #30 B2c.  It opened a transient L2 bridge and
 * put an ARP request on the air once a second; an `is-at` reply proved the whole path in
 * both directions at once (host builds a frame -> module transmits it with its own MAC ->
 * a real machine answers -> the driver accepts it -> the tap catches it before lwIP -> it
 * survives the link), which passively watching broadcasts never could.
 *
 * It goes because the bridge is permanent now: `net dhcp` proves exactly the same round
 * trip (DISCOVER out, OFFER back) and `net ping` adds NetX resolving the gateway by ARP
 * itself.  The one property it had that they do not -- proving the module's L2 path with
 * NetX out of the picture -- has no user left, because there is no configuration without
 * the host stack any more.  Its transient-bridge session also cannot coexist with the
 * resident one: the DATA channel has exactly one consumer.  History: 3413170~1.
 */

/* ---- registration -------------------------------------------------------- */

/*
 * Registered UNDER `wifi` (cmd_wifi.c holds the root table; cmd_wifi_priv.h is the
 * seam), so this cannot be CLI_SUBCMD_SET_CREATE -- that macro makes the array static.
 */
const struct cli_cmd wifi_link_subcmds[] = {
	CLI_CMD_ARG(info,  NULL, "both ends' UART counters + the current rate",
	            cmd_link_info,  1, 0),
	CLI_CMD_ARG(baud,  NULL, "change the link rate <2000000|3000000|4000000|6000000>",
	            cmd_link_baud,  2, 0),
	CLI_CMD_ARG(bench, NULL, "measured traffic [bytes] [secs] [rx|tx|both|all]",
	            cmd_link_bench, 1, 3),
	CLI_CMD_ARG(dbench, NULL, "free-running DATA channel [bytes] [secs] [rx|tx|both]",
	            cmd_link_dbench, 1, 3),
	CLI_SUBCMD_SET_END
};
