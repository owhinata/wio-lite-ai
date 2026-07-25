/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    cmd_link.c
 * @brief   `link` shell command (issue #23 U0-3): the RTL8720DN UART link itself.
 *
 *   link info                     both ends' counters + the current rate and its error
 *   link baud <bps>               change the rate of the link (2M / 3M / 4M / 6M)
 *   link bench [bytes] [secs] [dir]   measured traffic; dir = rx | tx | both
 *   link sweep                    bench all three directions at the current rate
 *
 * Where `wifi` is L2 and `net` is L3, this is L1/L2 of the wire between the STM32 and the
 * companion chip.  It exists because issue #23 needs a number before it can commit: can a
 * UART link carry 1500-byte Ethernet frames well enough to put a real TCP/IP stack on the
 * host (the "L2 bypass" road)?  Everything here feeds that go/no-go.
 *
 * It talks the LINK-CTRL channel (app/erpc.h), NOT eRPC -- a second frame type that the
 * link layer owns at both ends, so none of it required touching the generated eRPC server
 * shim on the module.  CTRL exists only in firmware 2.1.3+wio-n5 and later, and only the
 * `wifi rpc ver` command teaches the host which firmware is loaded, hence the
 * erpc_module_gen() gate on every subcommand below.
 *
 * PRECONDITION, uniformly: CTRL is only issued on a QUIESCENT link (the U0 simplification
 * -- see erpc.h).  So every subcommand that talks to the module requires that nothing
 * else holds the eRPC UART, which in practice means the telnet console must be stopped:
 * app/net_shell.c keeps a resident reference AND issues receives without the coarse mutex,
 * so its frames would interleave with ours.
 *
 * TIMING.  Per-frame latency is measured with TIM2->CNT (free-running, 32-bit at
 * 2*PCLK1 = 275 MHz, started unconditionally by port/threadx/tx_glue.c for the execution
 * profiler).  NOT DWT->CYCCNT, which stops when the core is in WFI -- this thread sleeps
 * between polls, so DWT would silently under-count.  TIM2 wraps every ~15.6 s, so it is
 * used only for individual round trips; whole-run elapsed time comes from HAL_GetTick.
 *
 * No clock/RCC/register work of its own: it READS TIM2->CNT and computes BRR arithmetic,
 * nothing more (XIP-safe).  Clean-room design.
 */
#include "cli.h"
#include "erpc.h"
#include "rtl_link.h"
#include "rtl8720.h"

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

/* The rates worth measuring.  Also the allow-list `link baud` accepts, which is what
 * stops a stray CTRL frame from parking the module at some rate we cannot reach:
 * 2 M is where every module boots, and 6 M is its documented ceiling
 * (rtl8721d_usi_uart.h "BaudRate: 110~6000000"). */
static const uint32_t link_bauds[] = { 2000000u, 3000000u, 4000000u, 6000000u };

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

/* Parse a 32-bit unsigned: 0x-hex or decimal.  Returns 0 on success. */
static int parse_u32(const char *s, uint32_t *out)
{
	uint32_t base = 10, val = 0;
	const char *p = s;

	if (p == NULL || *p == '\0')
		return -1;
	if (p[0] == '0' && (p[1] == 'x' || p[1] == 'X')) { base = 16; p += 2; }
	if (*p == '\0')
		return -1;
	for (; *p != '\0'; p++) {
		uint32_t d;
		char c = *p;

		if (c >= '0' && c <= '9')                    d = (uint32_t)(c - '0');
		else if (base == 16 && c >= 'a' && c <= 'f') d = (uint32_t)(c - 'a' + 10);
		else if (base == 16 && c >= 'A' && c <= 'F') d = (uint32_t)(c - 'A' + 10);
		else return -1;
		if (val > (0xFFFFFFFFu - d) / base)
			return -1;
		val = val * base + d;
	}
	*out = val;
	return 0;
}

static void put_u32le(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
	p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

static uint32_t get_u32le(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
	       ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

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
static int link_ctrl_ready(struct cli_instance *sh)
{
	if (erpc_module_gen() < 5u) {
		cli_error(sh, "link: this needs firmware 2.1.3+wio-n5 or later, and the host "
		          "does not know which is loaded\r\n");
		cli_print(sh, "  run `wifi rpc ver` first (it is what proves the firmware; the "
		          "answer is dropped again by any `wifi reset` or flash session)\r\n");
		return 1;
	}
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

/* ---- CTRL wrappers ------------------------------------------------------- */

static int link_ping(struct erpc_diag *diag)
{
	return erpc_ctrl_call(ERPC_CTRL_PING, NULL, 0u, link_reply,
	                      (uint16_t)sizeof(link_reply), LINK_CTRL_TMO_MS, diag);
}

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
		st[i] = get_u32le(link_reply + i * 4u);
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

	if (link_ctrl_ready(sh) == 0 && link_get_stats(sh, st) == 0)
		link_print_module(sh, st);

	rtl_link_end(sh);
	return 0;
}

/* ---- link baud ----------------------------------------------------------- */

/*
 * Change the rate of the link.  This is the one command here with a side effect that can
 * cost the link, so it is deliberately exclusive and deliberately manual:
 *
 *   - the module ACKs on the OLD rate and only then switches, so a lost ACK leaves both
 *     ends where they were and the command aborts having changed nothing;
 *   - after switching the host verifies with LINK_PING and, if that fails, tries to put
 *     both ends back at 2 Mbaud.  That is BEST EFFORT and nothing more: if the new rate
 *     fails because of signal quality, the LINK_SETBAUD(2M) telling the module to come
 *     back is sent at that same bad rate and will not arrive either.
 *   - the guaranteed recovery is `wifi reset` -- a CHIP_EN power cycle, after which the
 *     module is at 2 Mbaud and rtl_link_forget_module() has put the host there too.
 */
static int link_setbaud_module(struct cli_instance *sh, uint32_t baud)
{
	struct erpc_diag diag = {0};
	uint8_t req[8];
	int n;

	/* The module additionally requires that LINK_SETBAUD carry the sequence byte after
	 * the last CTRL frame it accepted -- one more thing a stream that aligned on 0xFFFF
	 * by chance would have to get right.  Pinging first makes that true by construction
	 * (and proves the link is alive at the current rate, immediately before we change
	 * it), instead of relying on nothing having been lost since the last CTRL call. */
	if (link_ping(&diag) < 0) {
		cli_error(sh, "link: no answer at the current rate -- not changing it\r\n");
		return -1;
	}

	put_u32le(req + 0, baud);
	put_u32le(req + 4, ERPC_CTRL_SETBAUD_MAGIC);
	n = erpc_ctrl_call(ERPC_CTRL_SETBAUD, req, (uint16_t)sizeof(req), link_reply,
	                   (uint16_t)sizeof(link_reply), LINK_CTRL_TMO_MS, &diag);
	if (n < 0) {
		cli_error(sh, "link: the module did not acknowledge %lu baud (rc %d) -- "
		          "nothing changed\r\n", (unsigned long)baud, n);
		return -1;
	}
	return 0;
}

/* Re-open the host UART at @baud and prove the link with up to 3 pings.  0 = alive. */
static int link_switch_host(struct cli_instance *sh, uint32_t baud)
{
	struct erpc_diag diag = {0};
	int try;

	/* The module needs a moment after its ACK to drain its TX FIFO and reprogram. */
	if (cli_sleep(sh, 100u))
		return -1;
	if (rtl_link_uart_rebaud(baud) != 0) {
		cli_error(sh, "link: USART1 would not re-open at %lu baud\r\n",
		          (unsigned long)baud);
		return -1;
	}
	for (try = 0; try < 3; try++) {
		if (link_ping(&diag) >= 0)
			return 0;
		if (cli_sleep(sh, 50u))
			break;
	}
	return -1;
}

static int cmd_link_baud(struct cli_instance *sh, int argc, char **argv)
{
	uint32_t baud, old;
	unsigned i;
	int ok = 0;

	if (argc < 2 || parse_u32(argv[1], &baud) != 0) {
		cli_error(sh, "usage: link baud <2000000|3000000|4000000|6000000>\r\n");
		return 1;
	}
	for (i = 0u; i < sizeof(link_bauds) / sizeof(link_bauds[0]); i++)
		if (link_bauds[i] == baud)
			ok = 1;
	if (!ok) {
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
	if (link_setbaud_module(sh, baud) != 0) {
		rtl_link_end(sh);                /* module never switched: link still healthy */
		return 1;
	}
	if (link_switch_host(sh, baud) == 0) {
		cli_print(sh, "link: now at %lu baud (LINK_PING answered)\r\n",
		          (unsigned long)baud);
		link_print_rate(sh, baud);
		rtl_link_end(sh);
		return 0;
	}

	/* Best effort only -- see the block comment above. */
	cli_warn(sh, "link: no answer at %lu baud, trying to fall back to %lu...\r\n",
	         (unsigned long)baud, (unsigned long)old);
	if (link_setbaud_module(sh, old) == 0 && link_switch_host(sh, old) == 0) {
		cli_print(sh, "link: back at %lu baud\r\n", (unsigned long)old);
		rtl_link_end(sh);
		return 1;
	}
	cli_error(sh, "link: the link is down at both rates -- run `wifi reset` to "
	          "power-cycle the module (it always comes back at 2000000)\r\n");
	rtl_link_end(sh);
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

		put_u32le(link_req + 0, want_rx);
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
		if (n >= 4 && get_u32le(link_reply) != want_tx) {
			cli_error(sh, "  bench: module saw %lu of %lu request bytes\r\n",
			          (unsigned long)get_u32le(link_reply), (unsigned long)want_tx);
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
	uint32_t bytes = LINK_BENCH_DEF, secs = LINK_BENCH_SECS, tx, rx;
	const char *dir = "both";

	if (argc > 1 && (parse_u32(argv[1], &bytes) != 0 || bytes == 0u ||
	                 bytes > LINK_BENCH_MAX)) {
		cli_error(sh, "link: bad size (1..%u)\r\n", (unsigned)LINK_BENCH_MAX);
		return 1;
	}
	if (argc > 2 && (parse_u32(argv[2], &secs) != 0 || secs == 0u ||
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
	if (link_ctrl_ready(sh) != 0) {
		rtl_link_end(sh);
		return 1;
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

/* ---- link sweep ---------------------------------------------------------- */

/*
 * All three directions at the CURRENT rate.  It deliberately does NOT walk the baud
 * rates: changing the rate is the one operation whose failure needs a power cycle to undo,
 * and burying it inside a loop makes it much harder to say afterwards which rate was
 * being set when something stopped answering.  Run `link baud <b>` between sweeps.
 */
static int cmd_link_sweep(struct cli_instance *sh, int argc, char **argv)
{
	static const char *const dirs[] = { "rx", "tx", "both" };
	uint32_t st0[LINK_STATS_WORDS], st1[LINK_STATS_WORDS];
	int have_st0 = 0;
	unsigned i;

	(void)argc; (void)argv;

	if (rtl_link_begin(sh, false) != RTL_LINK_READY)
		return 1;
	if (link_ctrl_ready(sh) != 0) {
		rtl_link_end(sh);
		return 1;
	}

	cli_print(sh, "link: sweep at %lu baud, %u B payload, %u s per direction\r\n",
	          (unsigned long)rtl_link_erpc_baud(), (unsigned)LINK_BENCH_DEF,
	          (unsigned)LINK_BENCH_SECS);
	have_st0 = (link_get_stats(sh, st0) == 0);

	for (i = 0u; i < sizeof(dirs) / sizeof(dirs[0]); i++) {
		uint32_t tx, rx;

		if (cli_cancel_requested(sh))
			break;
		(void)link_dir_sizes(dirs[i], LINK_BENCH_DEF, &tx, &rx);
		(void)link_bench_run(sh, LINK_BENCH_DEF, LINK_BENCH_SECS, tx, rx, &link_run);
		link_run_report(sh, dirs[i], LINK_BENCH_DEF, &link_run);
	}

	link_print_host(sh);
	if (have_st0 && link_get_stats(sh, st1) == 0) {
		cli_print(sh, "  mod delta: irq %lu, bytes %lu, drops %lu, fifo-overrun %lu, "
		          "framing %lu\r\n",
		          (unsigned long)(st1[0] - st0[0]), (unsigned long)(st1[1] - st0[1]),
		          (unsigned long)(st1[4] - st0[4]), (unsigned long)(st1[5] - st0[5]),
		          (unsigned long)(st1[6] - st0[6]));
		link_print_module(sh, st1);
	}

	rtl_link_end(sh);
	return 0;
}

/* ---- registration -------------------------------------------------------- */

CLI_SUBCMD_SET_CREATE(link_subcmds,
	CLI_CMD_ARG(info,  NULL, "both ends' UART counters + the current rate",
	            cmd_link_info,  1, 0),
	CLI_CMD_ARG(baud,  NULL, "change the link rate <2000000|3000000|4000000|6000000>",
	            cmd_link_baud,  2, 0),
	CLI_CMD_ARG(bench, NULL, "measured traffic [bytes] [secs] [rx|tx|both]",
	            cmd_link_bench, 1, 3),
	CLI_CMD_ARG(sweep, NULL, "bench all three directions at the current rate",
	            cmd_link_sweep, 1, 0),
	CLI_SUBCMD_SET_END);

CLI_CMD_REGISTER(link, link_subcmds,
                 "the RTL8720 UART link itself (info / baud / bench / sweep)",
                 NULL, 1, 0);
