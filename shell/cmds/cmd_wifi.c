/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    cmd_wifi.c
 * @brief   `wifi` shell command: the onboard RTL8720DN (power / L2 / the eRPC side).
 *
 *   wifi info                     show wiring + CHIP_EN state
 *   wifi on | off                 drive CHIP_EN high (power on) / low (power off)
 *                                 (`on` is a no-op when it is already high -- issue #47)
 *   wifi reset                    power-cycle CHIP_EN (Low 80 ms -> High)
 *   wifi log [reset]              bridge the LOG UART (UART9 @115200) <-> console;
 *                                 `reset` power-cycles first, capturing boot from t=0
 *   wifi ver                      prove the eRPC link (rpc_system_ack) + read the FW
 *                                 build id = the capability gate the L2 bridge requires
 *   wifi connect <ssid> [pw] [sec]  associate (STA) + arm the L2 bridge (#5 inc 3)
 *   wifi disconnect               drop the current association
 *   wifi autoreconnect [on|off]   re-associate by ourselves when the AP goes away (#32)
 *   wifi status                   connected? + RSSI + IP/mask/gw + MAC
 *   wifi scan                     list visible APs: ch/band/rssi/security/bssid/ssid
 *   wifi link <sub>               the UART link itself -- cmd_wifi_link.c (issue #23)
 *   wifi flash <sub>              the module's own SPI flash -- cmd_wifi_flash.c (#19)
 *
 * The log bridge takes over the console (f746g-disco issue #50 raw API): RTL8720DN RX
 * bytes stream to the CDC console and console keystrokes go to the module's TX, so
 * the operator reads the boot banner.  Ctrl+C exits.  Foreground only.  (The AT/HS
 * UART = USART1 carries binary eRPC, not ASCII, so it is driven via `wifi ver` /
 * `wifi connect` etc. over the app/erpc.c + app/wifi_rpc.c client, not a bridge.)
 *
 * Every subcommand that touches the module goes through app/rtl_link.h first, which
 * claims the console (rejecting a background worker: the console RX is a strict SPSC
 * pipe owned by the foreground) and takes the coarse link mutex, so whole flows cannot
 * interleave.  The eRPC subcommands (ver / connect / disconnect / status / scan) use
 * rtl_link_begin(), which also references the eRPC UART; the power / bridge / flash
 * subcommands use rtl_link_hw_claim(), which opens no UART -- they drive it themselves.
 * The eRPC frames are multiplexed by the resident service thread in app/erpc.c, so a
 * command no longer owns the link exclusively for a whole RPC round-trip.
 *
 * connect / DHCP block on the module for seconds, so those calls carry a long
 * timeout and an abort hook wired to Ctrl+C; aborting only stops the host-side wait
 * (the module keeps going -- `wifi reset` power-cycles it if it seems stuck; it takes
 * the link away from whoever holds it via rtl_link_force_quiesce()).
 *
 * Clean-room design; no third-party code reused.
 */
#include "cli.h"
#include "cmd_wifi_priv.h"
#include "rtl8720.h"
#include "erpc.h"
#include "wifi_rpc.h"
#include "rtl_link.h"
#include "wifi_auto.h" /* issue #32: the host's own re-association policy */
#include "wifi_connect.h" /* the association sequence, shared with boot (#37) */
#include "net_shell.h"
#include "nx_net.h"    /* the host stack owns the module while it is up (issue #23 U3) */
#include "stm32h7xx_hal.h"   /* #5 inc 6: HAL_GetTick (1 ms SysTick, fed via tx_glue.c)
                              * for the scan-wait deadline -- read-only, no HAL init. */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>           /* snprintf: naming a scan result's security mode.
                              * Already linked in by cmd_thread.c / cmd_membench.c /
                              * cmd_builtin.c, so this adds no footprint. */

/*
 * The RTL8720DN eRPC session helpers (rtl_link_begin/end), the Ctrl+C abort thunk
 * (rtl_abort_cb) and the module's host-tracked lwIP / IP-mode lifecycle state live in
 * rtl_link.{c,h} so the `net` (L3) command shares one console/RX-ring owner and one
 * copy of that state with `wifi` (L2).  This file keeps only the L2 (power /
 * association / probe) commands.
 */

/*
 * Open one RTL8720DN UART and run a bidirectional bridge to the console until the
 * user presses Ctrl+C.  The console + link are claimed FIRST -- before any hardware
 * action -- so a background job (`wifi ... &`) is rejected without powering/opening
 * anything, and the bridge is refused outright while an eRPC session references the
 * UART (re-opening the peripheral under the service thread would desynchronise it).
 * The bridge is then the sole reader of the RX ring.  Draining the module RX takes
 * priority: when a
 * chunk was moved we only poll for a keypress (timeout 0) and loop again
 * immediately; when the module was idle we block up to 1 ms for a key, which yields
 * the CPU (no busy-spin) without starving anything (IWDG prio 5 / USB prio 8 /
 * SysTick prio 14 preempt the shell thread anyway).
 */
static int wifi_bridge_run(struct cli_instance *sh, enum rtl8720_uart which,
                           uint32_t baud, bool do_reset)
{
	const char *name = (which == RTL8720_UART_LOG) ? "LOG UART (UART9)"
	                                               : "AT UART (USART1)";
	uint8_t buf[256];
	uint32_t drops;

	if (rtl_link_hw_claim(sh, false) != 0)         /* bg/busy-reject: no HW touched */
		return 1;
	if (rtl_link_uart_ref(which, baud) != 0) {
		rtl_link_hw_release(sh);
		cli_error(sh, "wifi: %s did not come ready\r\n", name);
		return 1;
	}

	cli_print(sh, "wifi: bridging RTL8720DN %s @%lu%s. Ctrl+C to exit.\r\n",
	          name, (unsigned long)baud, do_reset ? " (reset first)" : "");
	if (do_reset) {
		rtl8720_reset();                        /* Low->High edge AFTER we listen */
		rtl_link_forget_module();               /* power-cycled: the module we come back
		                                         * to is at 2 Mbaud with unknown firmware
		                                         * and no lwIP */
	}
	cli_rx_flush(sh);                               /* drop console type-ahead */

	for (;;) {
		size_t n = rtl8720_uart_read(buf, sizeof(buf));
		int c;

		if (n)
			cli_write(sh, buf, n);
		c = cli_read_byte(sh, n ? 0u : 1u);   /* poll if busy, else 1 ms yield */
		if (c == 3 || c == -2)                 /* Ctrl+C, or instance stopping */
			break;
		if (c >= 0) {
			uint8_t k = (uint8_t)c;
			rtl8720_uart_write(&k, 1u);         /* forward keystroke to the module */
		}
	}

	cli_rx_flush(sh);
	rtl_link_uart_unref();
	rtl_link_hw_release(sh);

	drops = rtl8720_uart_overflows();
	cli_print(sh, "\r\nwifi: bridge ended\r\n");
	if (drops)
		cli_print(sh, "  note: %lu RX bytes dropped (ring overflow)\r\n",
		          (unsigned long)drops);
	return 0;
}

static int cmd_wifi_info(struct cli_instance *sh, int argc, char **argv)
{
	(void)argc; (void)argv;
	cli_print(sh, "RTL8720DN (onboard WiFi/BLE companion):\r\n");
	cli_print(sh, "  CHIP_EN : PC3 = %s\r\n",
	          rtl8720_powered() ? "high (on)" : "low (off)");
	cli_print(sh, "  LOG UART: UART9  PD14/PD15 @115200   (`wifi log [reset]`)\r\n");
	cli_print(sh, "  eRPC UART: USART1 PA10/PB14 @%lu  (`wifi ver` / `connect` / `status`)\r\n",
	          (unsigned long)rtl_link_erpc_baud());
	return 0;
}

/*
 * Claim the console + link just long enough to drive CHIP_EN: this rejects a background
 * worker (`wifi off &`) and, since an eRPC flow holds the coarse mutex for its whole
 * duration, prevents a power/reset from cutting the module mid-connect.
 *
 * These are the RECOVERY commands, so unlike the bridge/flash claims they are allowed
 * even while the eRPC UART is referenced (allow_busy = true) -- "run `wifi reset`" is
 * the answer to a wedged module, and refusing it exactly when the link is stuck in use
 * would leave no way out.  They therefore take the link away first (see
 * wifi_power_take_link() below): in-flight tokens are abandoned, the reference count is
 * forced to zero and the UART is closed BEFORE CHIP_EN moves, which is also what stops
 * the service thread from writing into a module that is being power-cycled.
 */
static int wifi_power_claim(struct cli_instance *sh, const char *what)
{
	/* Not on the telnet console: cutting CHIP_EN takes the WiFi link -- and therefore the
	 * very session running this command -- away.  Run it from the USB CDC console. */
	if (net_shell_guard(sh, what))
		return 1;
	/*
	 * BEFORE the claim, not after (issue #32).  An automatic re-association holds the
	 * coarse mutex for up to 22 s, and these are the recovery commands -- the ones that
	 * must work exactly when something is stuck.  Disarming is what aborts an attempt in
	 * flight, so doing it here makes the claim below succeed in milliseconds instead of
	 * timing out at RTL_LINK_CLAIM_WAIT_MS.  The credentials go too, which is right: all
	 * three of these move CHIP_EN, so they change the module the credentials were for.
	 */
	wifi_auto_disarm(what);
	return rtl_link_hw_claim(sh, true) != 0 ? 1 : 0;
}

/*
 * Take the link away, and TELL THE HOST STACK (issue #41).
 *
 * The two belong together, so they are one function -- rtl_link.h keeps force-quiesce and
 * its siblings in single functions for exactly this reason, "precisely so that set cannot
 * drift apart".  Dropping the second half is invisible in testing: the owner notices the
 * revoked generation on its own, just up to NXN_REFRESH_MS (8 s) later, so the interface
 * does stand down -- eventually.  Everything typed inside that window meanwhile finds a
 * stale NX_NET_UP, and `wifi connect` refuses because it cannot renew the L2 bridge on a
 * module that was power-cycled a second ago.  That was #41, hit about half the time.
 *
 * All three commands here need it, not just off/reset: what strands the host stack is the
 * force-quiesce, and `wifi on` calls it too -- on the path where it really does raise
 * CHIP_EN.  Since issue #47 that is the only path it has; see cmd_wifi_on().
 */
static void wifi_power_take_link(struct cli_instance *sh)
{
	rtl_link_force_quiesce();                  /* recovery path: take the link away */
	if (nx_net_link_taken() != 0)              /* ... and let the owner give it back */
		cli_print(sh, "wifi: the host stack has not stood down yet -- `net info`, "
		          "and retry if the next command is refused\r\n");
}

/*
 * `wifi on` is IDEMPOTENT (issue #47).
 *
 * The other two recovery commands earn their destruction: they move CHIP_EN, so the module
 * really does become a different module and everything the host believed about it -- the
 * eRPC session, the link rate, the firmware generation, the association, the credentials --
 * is genuinely worthless.  `wifi on` against a module that is ALREADY on moves nothing.
 * rtl8720_powered() reads PC3's ODR -- the GPIO output-data latch that rtl8720_power()
 * itself writes (RM0468 §11.4.6), not a cached belief kept beside it and not the pad's
 * measured level (that would be IDR, which is the wrong question: what matters here is
 * whether the write below would change what we drive, and it would not).  Running the recovery
 * preamble anyway threw all of that state away for a write that changes nothing -- the
 * module kept running, associated, while the host forgot how to talk to it, and the next
 * `wifi scan` / `net info` failed with rc -2 until a `wifi reset` made the two agree again.
 *
 * So: check FIRST, before wifi_power_claim(), because that call's own preamble
 * (wifi_auto_disarm) is already destructive -- it drops the stored credentials, which is
 * right when CHIP_EN moves and wrong when it does not.  Reading the ODR outside the coarse
 * mutex is fine: everything that moves CHIP_EN holds the console, and the only outcomes of
 * losing the race are "say already-on about a module powered off a microsecond ago" (the
 * user types it again) and "take the full path" (exactly today's behaviour).
 *
 * Nothing is lost by refusing to act: the way to re-power a module that is on but wedged is
 * `wifi reset`, which is what the message points at.
 */
static int cmd_wifi_on(struct cli_instance *sh, int argc, char **argv)
{
	(void)argc; (void)argv;
	if (rtl8720_powered()) {
		cli_print(sh, "wifi: already on (CHIP_EN high) -- nothing to do; "
		          "`wifi reset` power-cycles it\r\n");
		return 0;
	}
	if (wifi_power_claim(sh, "wifi on"))
		return 1;
	wifi_power_take_link(sh);
	rtl8720_power(true);
	rtl_link_hw_release(sh);
	cli_print(sh, "wifi: CHIP_EN high (RTL8720DN powered on)\r\n");
	return 0;
}

static int cmd_wifi_off(struct cli_instance *sh, int argc, char **argv)
{
	(void)argc; (void)argv;
	if (wifi_power_claim(sh, "wifi off"))
		return 1;
	wifi_power_take_link(sh);
	rtl8720_power(false);                      /* force_quiesce above already forgot the
	                                            * module: rate, firmware, lwIP, address */
	rtl_link_hw_release(sh);
	cli_print(sh, "wifi: CHIP_EN low (RTL8720DN powered off)\r\n");
	return 0;
}

static int cmd_wifi_reset(struct cli_instance *sh, int argc, char **argv)
{
	(void)argc; (void)argv;
	if (wifi_power_claim(sh, "wifi reset"))
		return 1;
	wifi_power_take_link(sh);
	rtl8720_reset();                           /* force_quiesce above already forgot the
	                                            * module: rate, firmware, lwIP, address */
	rtl_link_hw_release(sh);
	cli_print(sh, "wifi: reset (CHIP_EN low 80 ms -> high)\r\n");
	return 0;
}

static int cmd_wifi_log(struct cli_instance *sh, int argc, char **argv)
{
	bool do_reset = (argc >= 2 && strcmp(argv[1], "reset") == 0);

	if (argc >= 2 && !do_reset) {
		cli_error(sh, "usage: wifi log [reset]\r\n");
		return 1;
	}
	/* The reset happens INSIDE wifi_bridge_run, AFTER the UART is listening, so the
	 * boot banner is captured from the Low->High edge (t=0) -- a separate
	 * `wifi reset; wifi log` cannot do that: the banner is gone before the second
	 * command starts. */
	return wifi_bridge_run(sh, RTL8720_UART_LOG, 115200u, do_reset);
}

/* wifi ver: prove the eRPC link and read the firmware build id (issues #5/#20/#23).
 * Power on the RTL8720DN if needed, open the eRPC UART at the link's current rate and
 * round-trip a byte through rpc_system_ack -- a valid CRC-framed echo proves the link
 * (transport + framing + codec + the Serial3<->USART1 mapping) end to end -- then read
 * rpc_system_version.  That query is the ONLY place the host learns which firmware it
 * is talking to, so it is where every firmware-gated capability is earned; the L2
 * bridge refuses until it has run.  CAUTION: on pre-N2 / stock firmware the version query
 * corrupts the module heap (the factory shim erpc_free()s a string literal) -- RAM
 * only, `wifi reset` recovers, flash is untouched (issue #20). */
/*
 * Report the RX interrupt-latency budget and the three DIFFERENT losses (issue #23
 * U0-1).  Snapshot @st with rtl8720_uart_stats() while the session still holds the
 * UART: the counters survive a close but are cleared by the next open.
 *
 * `max N/G B per irq` is the point of the whole thing: N is the most bytes one
 * interrupt pulled out of the RXFIFO and G is how many more it could have taken
 * before the hardware lost one.  N == threshold means the ISR always arrived on time;
 * N creeping towards G means it is running out of margin, which is what decides
 * whether the threshold survives a higher baud (see app/rtl8720.c).
 *
 * `isr` is the per-interrupt drain cost in tenths of a microsecond -- it is NOT part of
 * the grace budget (the drain loop absorbs bytes that land while it runs), it just has
 * to stay well under one byte time on the wire.  NOTE the ISR path is ITCM-resident,
 * so the FIRST invocation after a cold I-cache pays an external-flash fetch: on board
 * #2 the maximum was 8.7 us over 7 interrupts (`wifi ver`) but 3.3 us over 133
 * (`wifi scan`), i.e. the short-exchange figure is the cold entry, not the real cost.
 */
static void print_rx_budget(struct cli_instance *sh,
                            const struct rtl8720_uart_stats *st)
{
	uint32_t cyc_per_us = SystemCoreClock / 1000000u;

	if (cyc_per_us == 0u)
		cyc_per_us = 1u;                    /* never divide by zero */
	cli_print(sh, "  rx: %lu irq, max %lu/%lu B per irq, isr %lu.%lu us\r\n",
	          (unsigned long)st->isr_count,
	          (unsigned long)st->isr_max_bytes, (unsigned long)st->isr_grace,
	          (unsigned long)(st->isr_max_cycles / cyc_per_us),
	          (unsigned long)((st->isr_max_cycles % cyc_per_us) * 10u / cyc_per_us));
	cli_print(sh, "  rx err: ore %lu framing %lu ring-drops %lu (ring %lu B)%s\r\n",
	          (unsigned long)st->ore, (unsigned long)st->ferr,
	          (unsigned long)st->drops, (unsigned long)st->ring_size,
	          (st->ore || st->ferr || st->drops) ? "  <-- NOT CLEAN" : "  (clean)");
}


static int cmd_wifi_ver(struct cli_instance *sh, int argc, char **argv)
{
	uint32_t baud = rtl_link_erpc_baud();       /* whatever the link runs at now */
	uint8_t echoed = 0u;
	struct erpc_diag diag = {0}, total = {0};
	struct rtl8720_uart_stats st = {0};
	int rc = -1, tries;
	char ver[64];
	int have_ver = 0;

	(void)argc; (void)argv;
	/* Its own claim rather than rtl_link_begin(): the probe must not join a session
	 * somebody else is driving.  allow_busy = false for the same reason. */
	if (rtl_link_hw_claim(sh, false) != 0)
		return 1;
	if (!rtl8720_powered()) {
		cli_print(sh, "wifi: powering on RTL8720DN, waiting ~1.5s for boot...\r\n");
		rtl8720_power(true);
		rtl_link_forget_module();          /* fresh boot: 2 Mbaud, fw unknown, no lwIP */
		if (cli_sleep(sh, 1500u)) {         /* cancellable boot wait */
			rtl_link_hw_release(sh);
			return 1;
		}
	}
	if (rtl_link_uart_ref(RTL8720_UART_AT, baud) != 0) {
		rtl_link_hw_release(sh);
		cli_error(sh, "wifi: USART1 @%lu did not come ready\r\n", (unsigned long)baud);
		return 1;
	}
	cli_print(sh, "wifi: eRPC link test (USART1 @%lu, rpc_system_ack 0x5A)...\r\n",
	          (unsigned long)baud);

	for (tries = 0; tries < 5; tries++) {
		rc = erpc_system_ack(0x5Au, &echoed, &diag);
		total.crc_fail               += diag.crc_fail;
		total.oversize               += diag.oversize;
		total.timeout                += diag.timeout;
		total.skipped_reply          += diag.skipped_reply;
		total.unsupported_invocation += diag.unsupported_invocation;
		total.frame_stall            += diag.frame_stall;
		total.ctrl_bad               += diag.ctrl_bad;
		if (rc == 0 && echoed == 0x5Au)
			break;
		if (cli_sleep(sh, 50u))            /* brief gap; Ctrl+C aborts */
			break;
	}
	if (rc == 0 && echoed == 0x5Au) {
		/* Read the build id while the link is up (pre-N2 caveat in the header above). */
		struct erpc_diag vdiag = {0};
		if (erpc_system_version(ver, (uint16_t)sizeof(ver), &vdiag) >= 0)
			have_ver = 1;
		/* This is the only place the host learns which firmware it is talking to, so it
		 * is where every firmware-dependent capability is earned: the issue #23 U0-2
		 * wire budget (n4 owns USI0 behind an 8 kB ring, so the 127-byte limit that
		 * shaped every request size is gone) and the U0-3 LINK-CTRL channel (n5).
		 * Anything else -- including a version query that failed -- leaves the
		 * generation at 0, i.e. every conservative default.  It is dropped again by any
		 * flash session or CHIP_EN move (rtl_dl_enter / rtl_link_forget_module), so it
		 * can only ever describe the module in front of us.
		 *
		 * Parsed as a NUMBER rather than matched against one string: the build id moves
		 * with every firmware increment, and "== wio-n4" silently withdrew the
		 * capability the moment n5 was flashed. */
		if (have_ver)
			erpc_set_module_gen(wifi_fw_gen_of(ver));
	}
	/* Snapshot before the unref: the counters survive the close (they are only reset
	 * by rtl8720_uart_open) but reading them here keeps them tied to this session. */
	rtl8720_uart_stats(&st);
	rtl_link_uart_unref();
	rtl_link_hw_release(sh);

	if (rc == 0 && echoed == 0x5Au) {
		cli_print(sh, "wifi: eRPC OK -- ack 0x5A -> 0x%02X, link up @%lu (%d tries)\r\n",
		          echoed, (unsigned long)baud, tries + 1);
		if (have_ver)
			cli_print(sh, "  fw version: %s\r\n", ver);
		else
			cli_print(sh, "  fw version: unavailable (pre-N2 firmware, or query failed)\r\n");
		/* What the host currently believes it may put on the wire -- the one piece of
		 * state a reader cannot infer: it outlives this command (the UART reference
		 * does not) but not a reset or a flash session. */
		cli_print(sh, "  wire: budget %u B, send chunk %u B (fw gen n%u%s)\r\n",
		          (unsigned)erpc_wire_budget(), (unsigned)wifi_rpc_send_chunk(),
		          (unsigned)erpc_module_gen(),
		          (erpc_module_gen() >= 5u) ? ", link ctrl"
		                                    : ((erpc_module_gen() == 0u) ? " = unknown"
		                                                                 : ""));
	} else {
		cli_error(sh, "wifi: eRPC FAILED -- ack 0x5A -> 0x%02X (rc %d)\r\n", echoed, rc);
	}
	cli_print(sh, "  diag: crc_fail %u oversize %u timeout %u skipped %u unsupported %u "
	          "stall %u ctrl_bad %u\r\n",
	          total.crc_fail, total.oversize, total.timeout, total.skipped_reply,
	          total.unsupported_invocation, total.frame_stall, total.ctrl_bad);
	/* RX interrupt-latency budget (issue #23 U0-1).  `bytes/grace` is how much of the
	 * RXFIFO's post-threshold headroom the worst interrupt actually used -- it is the
	 * number that says whether the threshold scheme holds at a higher baud.  Times are
	 * in tenths of a microsecond: a single interrupt is only a few microseconds at
	 * 550 MHz, so whole-microsecond resolution would round most of them to zero. */
	print_rx_budget(sh, &st);
	if (!(rc == 0 && echoed == 0x5Au))
		cli_print(sh, "  hint: `wifi reset` power-cycles the module (it always comes "
		          "back at 2 Mbaud); `wifi log reset` shows it boot\r\n");
	return (rc == 0 && echoed == 0x5Au) ? 0 : 1;
}

/*
 * wifi chplan [set <hex> confirm] -- read or install the module's channel plan (#40).
 *
 * ⚠️ WRITING IS PROVISIONING, NOT CONFIGURATION.  The plan survives a full board
 * power-down (measured), so it is written once per module and never on a boot path.  It
 * is also the module's regulatory domain: the value decides which channels the radio may
 * use and actively probe, so installing a foreign one makes the board scan where it is
 * not allowed to transmit.  Board #2 ships as 0x7f (RT_CHANNEL_DOMAIN_REALTEK_DEFINE);
 * the domain indices are in the module SDK's rtw_mlme_ext.h (0x25 = FCC1_FCC1 is the
 * United States, 0x27 = MKK1_MKK1 is Japan).
 *
 * Hence the `confirm` token, the same shape `wifi flash write` uses for the other
 * irreversible-ish thing in this subtree.  A bare `wifi chplan` reads and is safe; a
 * write has to be spelled out in full, because a mistyped one cannot be undone by a
 * power cycle and may not be undoable at all (the storage may be write-once, and the
 * shipped 0x7f has more bits set than any real domain index -- so getting back to it
 * could require clearing bits that a fuse array cannot clear).
 *
 * This is deliberately kept rather than deleted with the rest of the issue #40 bring-up:
 * a non-volatile setting on a device we can otherwise only power-cycle needs a way to be
 * inspected and put back.  Issue #40 itself did NOT turn out to need it -- installing a
 * plan that covers 5 GHz did not stop the association failures.
 */
static int cmd_wifi_chplan(struct cli_instance *sh, int argc, char **argv)
{
	struct wifi_rpc_opts o;
	struct erpc_diag diag;
	const char *val = NULL;
	int32_t result = -1;
	uint32_t want = 0u;
	uint8_t plan = 0u;
	int rc;

	if (argc >= 2) {
		if (strcmp(argv[1], "set") != 0 || argc != 4 ||
		    strcmp(argv[3], "confirm") != 0) {
			cli_error(sh, "usage: wifi chplan [set <hex> confirm]\r\n");
			cli_print(sh, "  the plan is NON-VOLATILE and is the module's "
			          "regulatory domain -- it survives a power cycle and decides "
			          "which channels the radio may transmit on\r\n");
			return 1;
		}
		val = argv[2];
		if (cli_parse_u32(val, &want) != 0 || want > 0xffu) {
			cli_error(sh, "wifi: bad channel plan (one byte, e.g. 0x7f)\r\n");
			return 1;
		}
	}
	if (rtl_link_begin(sh, true) != RTL_LINK_READY)
		return 1;
	if (wifi_hold_bridge(sh, "wifi chplan")) {
		rtl_link_end(sh);
		return 1;
	}
	wifi_opts(&o, sh, &diag);

	if (val != NULL) {
		cli_print(sh, "wifi: installing channel plan 0x%02x -- NON-VOLATILE "
		          "(survives a power cycle) and regulatory\r\n", (unsigned)want);
		o.timeout_ms = 5000u;
		rc = wifi_rpc_set_channel_plan(&o, (uint8_t)want, &result);
		if (rc || result != WIFI_RPC_OK) {
			cli_error(sh, "wifi: channel plan not accepted (rc %d, result %ld)\r\n",
			          rc, (long)result);
			rtl_link_end(sh);
			wifi_fail_diag(sh, &diag);
			return 1;
		}
	}

	/* Always read back, including right after a write: the module accepting the call
	 * and the driver actually holding the value are different claims. */
	o.timeout_ms = 3000u;
	rc = wifi_rpc_get_channel_plan(&o, &plan, &result);
	rtl_link_end(sh);
	if (rc || result != WIFI_RPC_OK) {
		cli_error(sh, "wifi: could not read the channel plan (rc %d, result %ld)\r\n",
		          rc, (long)result);
		return 1;
	}
	cli_print(sh, "wifi: channel plan 0x%02x\r\n", (unsigned)plan);
	if (val != NULL && plan != (uint8_t)want)
		cli_print(sh, "  NOTE: asked for 0x%02x -- the driver kept its own value\r\n",
		          (unsigned)want);
	return 0;
}

/* wifi connect <ssid> [password] [security_hex] (issue #5 inc 3).  The sequence
 * itself moved to app/wifi_connect.c when the boot configuration grew a second
 * caller (issue #37 step 4); what is left here is the argument handling.  Default
 * security is WPA2-AES with a password / OPEN without; a 3rd hex arg overrides it.
 *
 * earn_gen is false: on this path there IS an operator, so an unknown firmware
 * generation is reported and `wifi ver` stays the deliberate act issue #20 made it.
 */
static int cmd_wifi_connect(struct cli_instance *sh, int argc, char **argv)
{
	const char *ssid = argv[1];
	const char *pass = (argc >= 3) ? argv[2] : NULL;
	bool security_explicit = (argc >= 4);
	uint32_t security;

	if (security_explicit) {
		if (cli_parse_u32(argv[3], &security) != 0) {
			cli_error(sh, "wifi: bad security hex (e.g. 0x00400004)\r\n");
			return 1;
		}
	} else {
		security = pass ? WIFI_RPC_SEC_WPA2_AES_PSK : WIFI_RPC_SEC_OPEN;
	}
	return wifi_connect_run(sh, ssid, pass, security, security_explicit, false);
}

/* wifi disconnect: drop the current association (no power-on -- pure query). */
static int cmd_wifi_disconnect(struct cli_instance *sh, int argc, char **argv)
{
	struct wifi_rpc_opts o;
	struct erpc_diag diag;
	int32_t result = -1;
	int rc, link;

	(void)argc; (void)argv;
	/* Dropping the association is a link-down event, not a teardown: the bridge stays
	 * (issue #30 B2b treats an operator disconnect exactly like the AP going away). */
	if (net_shell_guard(sh, "wifi disconnect"))
		return 1;
	/*
	 * Disarm BEFORE the claim (issue #32).  An operator disconnect has to win: leaving
	 * autoreconnect armed would have the owner thread re-join within 8 s, and holding the
	 * coarse mutex in an attempt would make the claim below time out first.
	 */
	wifi_auto_disarm("wifi disconnect");
	link = rtl_link_begin(sh, false);
	if (link == RTL_LINK_OFF) {
		cli_print(sh, "wifi: powered off (nothing to disconnect)\r\n");
		return 0;
	}
	if (link != RTL_LINK_READY)
		return 1;

	wifi_opts(&o, sh, &diag);
	/*
	 * 25 s, not 5 (issue #32).  The disarm above aborts a re-association on the HOST
	 * side only -- rpc_wifi_connect is outside the module firmware's concurrent
	 * allow-list, so the module can still be inside wifi_connect() holding its serial
	 * mutex for the rest of its own 20 s timeout, and this request queues behind it.
	 * Giving up at 5 s would report a failed disconnect and then let the module finish
	 * associating: the operator would be left connected having asked not to be.  The
	 * margin over 22 s covers the disconnect's own work and the queueing on top.
	 * Ctrl+C still cuts the wait short (wifi_opts wires rtl_abort_cb).
	 */
	o.timeout_ms   = 25000u;
	rc = wifi_rpc_disconnect(&o, &result);
	rtl_link_end(sh);

	if (rc || result != WIFI_RPC_OK) {
		cli_error(sh, "wifi: disconnect failed (rc %d, result %ld)\r\n",
		          rc, (long)result);
		wifi_fail_diag(sh, &diag);
		return 1;
	}
	cli_print(sh, "wifi: disconnected\r\n");
	return 0;
}

/*
 * wifi autoreconnect [on|off] (issue #32): whether the HOST re-associates by itself when
 * the AP goes away.  No argument reports the state and the tally.
 *
 * Turning it on does not go near the module -- it only says that the next successful
 * `wifi connect` may keep its credentials, which is what makes holding a passphrase in RAM
 * a decision rather than a default.  Turning it off wipes them and aborts any attempt
 * already running.
 */
static int cmd_wifi_autoreconnect(struct cli_instance *sh, int argc, char **argv)
{
	bool in_flight;

	if (argc < 2) {
		wifi_auto_print(sh);
		return 0;
	}
	if (strcmp(argv[1], "on") == 0) {
		wifi_auto_set_enabled(true);
		cli_print(sh, "wifi: autoreconnect on\r\n");
		if (!wifi_auto_armed())
			cli_print(sh, "  arms at the next successful `wifi connect` (the "
			          "credentials come from there)\r\n");
		return 0;
	}
	if (strcmp(argv[1], "off") == 0) {
		/* Sampled BEFORE disarming, which is what ends the attempt. */
		in_flight = wifi_auto_attempt_in_flight();
		wifi_auto_set_enabled(false);
		cli_print(sh, "wifi: autoreconnect off (credentials cleared)\r\n");
		if (in_flight)
			/* The host stops waiting; the module does not stop trying.  Its
			 * wifi_connect() runs to its own 20 s timeout, so it may still come
			 * back associated -- the next refresh will simply report that as link
			 * up, and nothing will re-join after it. */
			cli_print(sh, "  an attempt was in flight and was cut short -- the "
			          "module may still finish associating (`wifi status`)\r\n");
		return 0;
	}
	cli_error(sh, "usage: wifi autoreconnect [on|off]\r\n");
	return 1;
}

/* wifi status: report association state, RSSI, IP config and MAC (pure query). */
static int cmd_wifi_status(struct cli_instance *sh, int argc, char **argv)
{
	struct wifi_rpc_opts o;
	struct erpc_diag diag;
	char mac[18];
	int32_t connected = -1, rssi = 0, result = -1;
	int rc, link;

	(void)argc; (void)argv;
	link = rtl_link_begin(sh, false);
	if (link == RTL_LINK_OFF) {
		cli_print(sh, "wifi: powered off (`wifi connect <ssid> ...` to bring up)\r\n");
		return 0;
	}
	if (link != RTL_LINK_READY)
		return 1;

	wifi_opts(&o, sh, &diag);
	o.timeout_ms   = 3000u;

	rc = wifi_rpc_is_connected(&o, &connected);
	if (rc) {
		cli_error(sh, "wifi: query failed (rc %d)\r\n", rc);
		rtl_link_end(sh);
		return 1;
	}
	cli_print(sh, "wifi: %s\r\n",
	          connected == WIFI_RPC_OK ? "connected" : "not connected");

	if (wifi_rpc_get_mac(&o, mac, &result) == 0 && result == WIFI_RPC_OK)
		cli_print(sh, "  mac  %s\r\n", mac);

	if (connected == WIFI_RPC_OK) {
		if (wifi_rpc_get_rssi(&o, &rssi, &result) == 0 && result == WIFI_RPC_OK)
			cli_print(sh, "  rssi %ld dBm\r\n", (long)rssi);
	}
	/*
	 * The channel plan (issue #40).  It decides which channels the driver scans and
	 * whether it may only listen passively on them, so it is the first thing to check
	 * when a join reports "target AP not found" for an access point `wifi scan` lists.
	 * Reported raw: the plan-index-to-channel-set table lives in the driver, not here,
	 * and inventing names for the indices would be guessing.
	 */
	{
		uint8_t plan = 0u;
		int32_t plan_rc = -1;

		o.timeout_ms = 3000u;
		if (wifi_rpc_get_channel_plan(&o, &plan, &plan_rc) == 0 &&
		    plan_rc == WIFI_RPC_OK)
			cli_print(sh, "  channel plan: 0x%02x\r\n", (unsigned)plan);
	}
	rtl_link_end(sh);
	/* No address here: L3 belongs to `net` (issue #30 B1).  The module's own lwIP no
	 * longer takes one, so printing it would only ever show 0.0.0.0. */
	cli_print(sh, "  (`net info` for the address)\r\n");
	return 0;
}

/*
 * ---- wifi scan (issue #5, increment 6) ------------------------------------------
 *
 * How many records to pull back in the single get_ap_records reply.  Three ceilings
 * meet here: the module serialises the whole array into ONE eRPC frame and both sides
 * cap a message at 4096 B (app/erpc.c ERPC_RX_SCRATCH / the firmware's 4 KB
 * MessageBufferFactory), the module erpc_malloc()s 62*n bytes out of its FreeRTOS heap,
 * and the reply payload has to land in host BSS because the shell thread stack is 4 KB.
 * 32 records = 1992 B and covers any realistic environment; the firmware can only copy
 * a PREFIX of its array (there is no offset argument), so a busier band is reported as
 * "the first 32 of N" rather than paged.
 */
#define WIFI_SCAN_MAX_APS   32u
_Static_assert(WIFI_SCAN_MAX_APS <= WIFI_RPC_SCAN_MAX_RECORDS,
               "scan fetch cap must fit the module's record array");

/* Raw reply payload for wifi_rpc_scan_record().  Static: far too big for the 4 KB
 * shell stack (same reason as s_bak_chunk below).  Single-owner, like every other eRPC
 * path here -- rtl_link_begin() holds the console for the whole scan. */
static uint8_t s_scan_buf[WIFI_RPC_SCAN_BUF_SIZE(WIFI_SCAN_MAX_APS)];

/* rtw_security_t bits (Realtek wifi_constants.h). */
#define SEC_WEP        0x00000001u
#define SEC_TKIP       0x00000002u
#define SEC_AES        0x00000004u
#define SEC_AES_CMAC   0x00000010u
#define SEC_SHARED     0x00008000u
#define SEC_WPA        0x00200000u
#define SEC_WPA2       0x00400000u
#define SEC_WPA3       0x00800000u
#define SEC_WPS        0x10000000u

/*
 * Band from the CHANNEL, not from the record's `band` field.
 *
 * Measured on board #2: rtw_scan_result_t.band came back 0 (= RTW_802_11_BAND_5GHZ) for
 * EVERY result, including 2.4 GHz channels 5 and 11, so on this firmware 0 evidently
 * means "not filled in" rather than "5 GHz".  (The neighbouring fields are fine: rssi /
 * security / channel all decoded to sensible values in the same records, which is what
 * pins the 62-byte packed stride.)  This is an observation, not something provable from
 * source: the field is written -- or not -- inside Realtek's prebuilt libameba scan path
 * behind wifi_scan_networks(), and the reachable firmware source only memcpy()s the
 * driver's record wholesale (seeed-ambd-firmware wifi_main.c, wifi_scan_result_handler).
 * The channel number is unambiguous, so classify from that and ignore the wire `band`.
 */
static const char *scan_band_name(uint32_t channel)
{
	if (channel >= 1u && channel <= 14u)     /* 2.4 GHz: 1..13 (+14, JP 802.11b) */
		return "2.4G";
	if (channel >= 32u && channel <= 177u)   /* 5 GHz: UNII-1..UNII-4 */
		return "5G";
	return "?";
}

/* Longest name scan_security_name() can build: "WEP-SHARED" + "-MIXED" + NUL. */
#define SEC_NAME_CAP   20u

/*
 * Name a security bitmask into @out (>= SEC_NAME_CAP bytes).  Decoded by BITS rather than matched
 * against the RTW_SECURITY_* combinations, because a real AP's beacon yields values the
 * enum does not enumerate (e.g. WPA|WPA2 with both ciphers).  Anything with a bit we do
 * not know falls back to hex so the operator sees the truth instead of a wrong label.
 */
static void scan_security_name(uint32_t sec, char *out, size_t cap)
{
	static const uint32_t known = SEC_WEP | SEC_TKIP | SEC_AES | SEC_AES_CMAC |
	                              SEC_SHARED | SEC_WPA | SEC_WPA2 | SEC_WPA3 | SEC_WPS;
	const char *base, *ciph;

	if (sec == 0u) { (void)snprintf(out, cap, "OPEN"); return; }
	if (sec == 0xFFFFFFFFu) { (void)snprintf(out, cap, "UNKNOWN"); return; }
	if (sec & ~known) { (void)snprintf(out, cap, "0x%08lX", (unsigned long)sec); return; }

	if (sec & SEC_WPA3)                             base = "WPA3";
	else if ((sec & (SEC_WPA | SEC_WPA2)) == (SEC_WPA | SEC_WPA2)) base = "WPA/WPA2";
	else if (sec & SEC_WPA2)                        base = "WPA2";
	else if (sec & SEC_WPA)                         base = "WPA";
	else if (sec & SEC_WEP)                         base = (sec & SEC_SHARED) ? "WEP-SHARED"
	                                                                          : "WEP";
	else if (sec & SEC_WPS)                         base = "WPS";
	else { (void)snprintf(out, cap, "0x%08lX", (unsigned long)sec); return; }

	if ((sec & (SEC_AES | SEC_TKIP)) == (SEC_AES | SEC_TKIP)) ciph = "-MIXED";
	else if (sec & SEC_AES)                                   ciph = "-AES";
	else if (sec & SEC_TKIP)                                  ciph = "-TKIP";
	else if (sec & SEC_AES_CMAC)                              ciph = "-CMAC";
	else                                                      ciph = "";
	(void)snprintf(out, cap, "%s%s", base, ciph);
}

/*
 * Copy @rec's SSID into @out (>= 33 bytes) as PRINTABLE ASCII ONLY, substituting '?'
 * for every other byte; returns non-zero if anything was substituted.
 *
 * An SSID is 32 bytes chosen by whoever is broadcasting -- unauthenticated, attacker-
 * controlled input that we are about to write to a terminal.  Allow-listing 0x20..0x7E
 * makes that structurally safe: no ESC/CSI, no C0, no DEL, and no 8-bit C1 controls
 * (0x9B is CSI too), and no UTF-8 decoding needed -- note that even valid UTF-8 is not
 * automatically safe, since 0xC2 0x80..0x9F decodes back to C1 controls.  A non-ASCII
 * SSID therefore prints as '?'s; the caller compensates by dumping the raw bytes as hex
 * so no information is lost.
 */
static int scan_ssid_ascii(const struct wifi_ap_record *rec, char *out)
{
	uint8_t i;
	int subst = 0;

	for (i = 0; i < rec->ssid_len; i++) {
		unsigned char c = (unsigned char)rec->ssid[i];
		if (c >= 0x20u && c <= 0x7Eu) {
			out[i] = (char)c;
		} else {
			out[i] = '?';
			subst = 1;
		}
	}
	out[rec->ssid_len] = '\0';
	if (rec->ssid_len == 0u)                /* hidden / zero-length SSID */
		subst = 0;
	return subst;
}

/*
 * Wait until the module reports the scan finished.  Polls rpc_wifi_is_scaning every
 * @POLL_MS with a cancellable sleep in between, giving up @budget_ms after entry.
 * Returns 0 = finished, -1 = transport/decode error (message printed), -2 = budget
 * expired, -4 = Ctrl+C.  The module keeps scanning after -2/-4; it clears the flag on
 * its own, which is why the caller can just wait for it again on the next `wifi scan`.
 *
 * The budget is real elapsed time (HAL_GetTick, wrap-safe by subtraction), NOT a count
 * of sleeps: each poll is itself an eRPC round-trip that may take up to its own timeout,
 * so summing only the sleeps would let a slow-but-responsive module stretch a "15 s"
 * wait several times over.  @budget_ms == 0 therefore means "query once and report".
 */
static int scan_wait_done(struct cli_instance *sh, struct wifi_rpc_opts *o,
                          uint32_t budget_ms)
{
	const uint32_t POLL_MS = 300u;
	uint32_t t0 = HAL_GetTick();
	int scanning, rc;

	for (;;) {
		o->timeout_ms = 3000u;
		rc = wifi_rpc_is_scanning(o, &scanning);
		if (rc == -4)
			return -4;
		if (rc) {
			cli_error(sh, "wifi: scan-state query failed (rc %d)\r\n", rc);
			return -1;
		}
		if (!scanning)
			return 0;
		if ((uint32_t)(HAL_GetTick() - t0) >= budget_ms)
			return -2;
		if (cli_sleep(sh, POLL_MS))            /* cancellable (ticks == ms) */
			return -4;
	}
}

/*
 * wifi scan (issue #5 inc 6): list the APs the module can see.
 *
 * The module's scan is asynchronous, so this is start -> poll -> get_ap_num ->
 * get_ap_records, all inside one console-claimed USART1 session.  It needs an STA
 * interface (boot leaves the radio in RTW_MODE_NONE), so when we are NOT associated it
 * first cycles off -> on(STA) exactly as `wifi connect` step 1 does.  When we ARE
 * associated the radio is already in STA mode, so that cycle is skipped -- it would
 * drop a working link.  lwIP is not involved: scanning is pure L2, and `wifi connect`
 * re-runs tcpip_init before its own off->on(STA), so the netif-before-driver ordering
 * invariant is preserved without doing it here.
 */
static int cmd_wifi_scan(struct cli_instance *sh, int argc, char **argv)
{
	struct wifi_rpc_opts o;
	struct erpc_diag diag;
	struct wifi_ap_record rec;
	struct rtl8720_uart_stats st = {0};
	char ssid[33], sec[SEC_NAME_CAP];
	int32_t connected = -1, result = -1;
	uint16_t ap_num = 0, want, got = 0, i;
	int rc;

	(void)argc; (void)argv;
	/* A scan retunes the radio, which is the one the bridge is relaying through -- so
	 * traffic pauses for its duration, but the tap itself is untouched. */
	if (rtl_link_begin(sh, true) != RTL_LINK_READY)
		return 1;
	if (wifi_hold_bridge(sh, "wifi scan")) {
		rtl_link_end(sh);
		return 1;
	}

	wifi_opts(&o, sh, &diag);

	/* 1) Are we associated?  If so the radio is in STA mode already and we must not
	 * touch it; if not, land in STA the same way `wifi connect` does. */
	o.timeout_ms = 3000u;
	rc = wifi_rpc_is_connected(&o, &connected);
	if (rc == -4) { cli_print(sh, "wifi: aborted\r\n"); goto fail; }
	if (rc) {
		cli_error(sh, "wifi: link query failed (rc %d)\r\n", rc);
		goto fail;
	}
	if (connected != WIFI_RPC_OK) {
		if (wifi_enter_sta(sh, &o) != 0)
			goto fail;
	}

	/* 2) A scan left running by an earlier Ctrl+C must finish before a new start()
	 * would be accepted, and its results would otherwise be half-collected. */
	rc = scan_wait_done(sh, &o, 0u);           /* budget 0: just test the flag once */
	if (rc == -2) {
		cli_print(sh, "wifi: a scan is already running, waiting for it...\r\n");
		rc = scan_wait_done(sh, &o, 15000u);
	}
	if (rc == -4) { cli_print(sh, "wifi: aborted\r\n"); goto fail; }
	if (rc == -1)
		goto fail;
	if (rc == -2) {
		cli_error(sh, "wifi: the previous scan did not finish within 15s\r\n");
		goto fail;
	}

	/* 3) Start, then wait for completion. */
	cli_print(sh, "wifi: scanning (up to ~15s, Ctrl+C aborts)...\r\n");
	o.timeout_ms = 5000u;
	rc = wifi_rpc_scan_start(&o, &result);
	if (rc == -4) { cli_print(sh, "wifi: aborted\r\n"); goto fail; }
	if (rc || result != WIFI_RPC_OK) {
		cli_error(sh, "wifi: scan start failed (rc %d, result %ld)\r\n",
		          rc, (long)result);
		goto fail;
	}
	rc = scan_wait_done(sh, &o, 15000u);
	if (rc == -4) { cli_print(sh, "wifi: aborted\r\n"); goto fail; }
	if (rc == -1)
		goto fail;
	if (rc == -2) {
		cli_error(sh, "wifi: scan did not finish within 15s\r\n");
		goto fail;
	}

	/* 4) How many, then fetch exactly that many -- the firmware's get_ap_records()
	 * copies `number` records regardless of how many were actually found. */
	o.timeout_ms = 3000u;
	rc = wifi_rpc_scan_get_ap_num(&o, &ap_num);
	if (rc == -4) { cli_print(sh, "wifi: aborted\r\n"); goto fail; }
	if (rc) {
		cli_error(sh, "wifi: AP count query failed (rc %d)\r\n", rc);
		goto fail;
	}
	if (ap_num == 0u) {
		rtl_link_end(sh);
		cli_print(sh, "wifi: no networks found\r\n");
		return 0;
	}

	want = (ap_num > WIFI_SCAN_MAX_APS) ? (uint16_t)WIFI_SCAN_MAX_APS : ap_num;
	o.timeout_ms = 5000u;
	rc = wifi_rpc_scan_get_ap_records(&o, want, s_scan_buf, (uint16_t)sizeof(s_scan_buf),
	                                  &got, &result);
	if (rc == -4) { cli_print(sh, "wifi: aborted\r\n"); goto fail; }
	if (rc || result != WIFI_RPC_OK) {
		cli_error(sh, "wifi: fetching scan records failed (rc %d, result %ld)\r\n",
		          rc, (long)result);
		goto fail;
	}

	/* The AP-record reply is the biggest receive in the tree (~2 KB), so this session
	 * is the one that shows the STEADY-STATE interrupt cost rather than a cold-cache
	 * first entry -- which is what the issue-#23 threshold choice rests on. */
	rtl8720_uart_stats(&st);
	rtl_link_end(sh);

	/* 5) Report.  SSID is last so a hex fallback cannot break the column alignment. */
	cli_print(sh, "wifi: %u network%s\r\n", (unsigned)ap_num, ap_num == 1u ? "" : "s");
	if (want < ap_num)
		cli_print(sh, "  (showing the first %u)\r\n", (unsigned)want);
	/* security column is 14 wide so the longest name this can build ("WPA/WPA2-MIXED")
	 * still fits -- a wider one only shifts the two columns after it, never truncates. */
	cli_print(sh, "  ch  band  rssi  security        bssid              ssid\r\n");
	for (i = 0; i < got; i++) {
		if (wifi_rpc_scan_record(s_scan_buf, got, i, &rec) != 0)
			break;
		scan_security_name(rec.security, sec, sizeof(sec));
		rc = scan_ssid_ascii(&rec, ssid);
		cli_print(sh, "  %2lu  %4s  %4d  %-14s  %02x:%02x:%02x:%02x:%02x:%02x  %s\r\n",
		          (unsigned long)rec.channel,
		          scan_band_name(rec.channel),
		          (int)rec.rssi, sec,
		          rec.bssid[0], rec.bssid[1], rec.bssid[2],
		          rec.bssid[3], rec.bssid[4], rec.bssid[5],
		          rec.ssid_len ? ssid : "(hidden)");
		if (rc) {                       /* non-ASCII bytes were replaced by '?' */
			uint8_t k;
			cli_print(sh, "        raw ssid:");
			for (k = 0; k < rec.ssid_len; k++)
				cli_print(sh, " %02x", (unsigned char)rec.ssid[k]);
			cli_print(sh, "\r\n");
		}
	}
	print_rx_budget(sh, &st);
	return 0;

fail:
	rtl_link_end(sh);
	wifi_fail_diag(sh, &diag);
	return 1;
}


CLI_SUBCMD_SET_CREATE(wifi_subcmds,
	CLI_CMD_ARG(info,  NULL, "show RTL8720 wiring + CHIP_EN state",       cmd_wifi_info,  1, 0),
	CLI_CMD_ARG(on,    NULL, "CHIP_EN high (power on RTL8720); no-op if already on",
	            cmd_wifi_on,    1, 0),
	CLI_CMD_ARG(off,   NULL, "CHIP_EN low (power off)",                   cmd_wifi_off,   1, 0),
	CLI_CMD_ARG(reset, NULL, "power-cycle CHIP_EN (low 80ms -> high)",    cmd_wifi_reset, 1, 0),
	CLI_CMD_ARG(log,   NULL, "bridge LOG UART (UART9 @115200); `log reset` captures boot from t=0",
	            cmd_wifi_log, 1, 1),
	CLI_CMD_ARG(ver,   NULL, "prove the eRPC link + read the FW build id (required before `wifi connect`)",
	            cmd_wifi_ver, 1, 0),
	CLI_CMD_ARG(connect,    NULL, "associate with an AP: connect <ssid> [pw] [sec_hex]", cmd_wifi_connect,    2, 2),
	CLI_CMD_ARG(disconnect, NULL, "drop the current WiFi association",     cmd_wifi_disconnect, 1, 0),
	CLI_CMD_ARG(autoreconnect, NULL, "re-associate by ourselves when the AP goes away: autoreconnect [on|off]",
	            cmd_wifi_autoreconnect, 1, 1),
	CLI_CMD_ARG(status,     NULL, "show connection state / RSSI / IP / MAC", cmd_wifi_status, 1, 0),
	CLI_CMD_ARG(scan,       NULL, "list visible APs (ch/band/rssi/security/bssid/ssid)", cmd_wifi_scan, 1, 0),
	CLI_CMD_ARG(chplan,     NULL, "regulatory domain: chplan [set <hex> confirm] (writes PERSIST)",
	            cmd_wifi_chplan, 1, 3),
	CLI_CMD_ARG(link, wifi_link_subcmds,
	            "the RTL8720 UART link itself (info / baud / bench / dbench)", NULL, 1, 0),
	CLI_CMD_ARG(flash, wifi_flash_subcmds,
	            "the module's own SPI flash: survey / backup / reprogram", NULL, 1, 0),
	CLI_SUBCMD_SET_END);

CLI_CMD_REGISTER(wifi, wifi_subcmds,
                 "onboard RTL8720DN: power / WiFi (L2) / UART link / firmware flashing",
                 NULL, 1, 0);
