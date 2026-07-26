/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    cmd_wifi.c
 * @brief   `wifi` shell command (issue #17): RTL8720DN factory-firmware probe.
 *
 *   wifi info                     show wiring + CHIP_EN state
 *   wifi on | off                 drive CHIP_EN high (power on) / low (power off)
 *   wifi reset                    power-cycle CHIP_EN (Low 80 ms -> High)
 *   wifi log                      bridge the LOG UART (UART9 @115200) <-> console
 *   wifi probe                    reset the module and capture its boot log from t=0
 *   wifi rpc [ver] [baud]         eRPC link test: rpc_system_ack (default 2 Mbaud, #5);
 *                                 `ver` also reads the FW build id (N2+ only, #20)
 *   wifi connect <ssid> [pw] [sec]  associate (STA) + DHCP, print the IP (#5 inc 3)
 *   wifi disconnect               drop the current association
 *   wifi status                   connected? + RSSI + IP/mask/gw + MAC
 *   wifi scan                     list visible APs: ch/band/rssi/security/bssid/ssid
 *
 * RTL8720DN firmware-download subcommands (issue #19; see app/rtl8720_flash.c):
 *   wifi flashprobe [hold_us]     M1: prove UART download-mode entry (read-only)
 *   wifi flashload [hold] [baud]  M2: upload the flashloader + read sector 0 (read-only)
 *   wifi flashread <off> [n]      M3: survey sectors, erased-vs-data (read-only)
 *   wifi flashtest <off> confirm  M3: DESTRUCTIVE erase/write/verify on one unused sector
 *   wifi flashinfo                M4: capacity (address wrap) / status regs / checksum
 *   wifi flashbackup [off] [len]  M4: back the flash up to the PC over YMODEM (read-only)
 *   wifi imgload                  M5: receive an image from the PC into PSRAM (read-only)
 *   wifi imginfo                  M5: show + re-verify the staged image (read-only)
 *   wifi imgsend                  M5: send the staged image back to the PC (read-only)
 *   wifi flashwrite <off> confirm M5: DESTRUCTIVE -- program the staged image
 *
 * The log/probe bridge takes over the console (issue #50 raw API): RTL8720DN RX
 * bytes stream to the CDC console and console keystrokes go to the module's TX, so
 * the operator reads the boot banner.  Ctrl+C exits.  Foreground only.  (The AT/HS
 * UART = USART1 carries binary eRPC, not ASCII, so it is driven via `wifi rpc` /
 * `wifi connect` etc. over the app/erpc.c + app/wifi_rpc.c client, not a bridge.)
 *
 * Every subcommand that touches the module goes through app/rtl_link.h first, which
 * claims the console (rejecting a background worker: the console RX is a strict SPSC
 * pipe owned by the foreground) and takes the coarse link mutex, so whole flows cannot
 * interleave.  The eRPC subcommands (rpc / connect / disconnect / status / scan) use
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
/* cli_instance.h (ThreadX-aware) gives the full struct cli_instance, which
 * `wifi imgload` needs for sh->rx_dropped: the console backend silently drops --
 * and counts -- a byte when its RX ring overruns, and a bulk receive is the first
 * thing in this firmware that can provoke that, so the count has to be reported
 * rather than assumed to be zero.  cmd_wifi.c is firmware-only, so the ThreadX
 * dependency it pulls in is fine (like cmd_watch.c / cmd_thread.c). */
#include "cli_instance.h"
#include "cmd_xfer.h"
#include "rtl8720.h"
#include "rtl8720_flash.h"
#include "rtl8720_img.h"     /* #19 M5: PSRAM staging for a host-supplied image */
#include "psram.h"           /* #19 M5: PSRAM_BASE_ADDR + the OCTOSPI1 guard */
#include "log.h"             /* #19 M5: transfer post-mortem into the dmesg ring */
#include "erpc.h"
#include "wifi_rpc.h"
#include "rtl_link.h"
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
		if (c >= '0' && c <= '9')            d = (uint32_t)(c - '0');
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
	cli_print(sh, "RTL8720DN (onboard WiFi/BLE companion, issue #17):\r\n");
	cli_print(sh, "  CHIP_EN : PC3 = %s\r\n",
	          rtl8720_powered() ? "high (on)" : "low (off)");
	cli_print(sh, "  LOG UART: UART9  PD14/PD15 @115200   (`wifi log` / `wifi probe`)\r\n");
	cli_print(sh, "  eRPC UART: USART1 PA10/PB14 @2Mbaud  (`wifi rpc` / `connect` / `status`)\r\n");
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
 * would leave no way out.  They therefore take the link away first (see the callers'
 * rtl_link_force_quiesce()): in-flight tokens are abandoned, the reference count is
 * forced to zero and the UART is closed BEFORE CHIP_EN moves, which is also what stops
 * the service thread from writing into a module that is being power-cycled.
 */
static int wifi_power_claim(struct cli_instance *sh, const char *what)
{
	/* Not on the telnet console: cutting CHIP_EN takes the WiFi link -- and therefore the
	 * very session running this command -- away.  Run it from the USB CDC console. */
	if (net_shell_guard(sh, what))
		return 1;
	return rtl_link_hw_claim(sh, true) != 0 ? 1 : 0;
}

static int cmd_wifi_on(struct cli_instance *sh, int argc, char **argv)
{
	(void)argc; (void)argv;
	if (wifi_power_claim(sh, "wifi on"))
		return 1;
	rtl_link_force_quiesce();                      /* also forgets rate/firmware/lwIP */                      /* nobody may hold the link across
	                                                * a CHIP_EN change */
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
	rtl_link_force_quiesce();                  /* recovery path: take the link away */
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
	rtl_link_force_quiesce();                  /* recovery path: take the link away */
	rtl8720_reset();                           /* force_quiesce above already forgot the
	                                            * module: rate, firmware, lwIP, address */
	rtl_link_hw_release(sh);
	cli_print(sh, "wifi: reset (CHIP_EN low 80 ms -> high)\r\n");
	return 0;
}

static int cmd_wifi_log(struct cli_instance *sh, int argc, char **argv)
{
	(void)argc; (void)argv;
	return wifi_bridge_run(sh, RTL8720_UART_LOG, 115200u, false);
}

static int cmd_wifi_probe(struct cli_instance *sh, int argc, char **argv)
{
	(void)argc; (void)argv;
	/* Reset happens INSIDE wifi_bridge_run, AFTER the UART is listening, so the
	 * boot banner is captured from the Low->High edge (t=0, plan BLOCKING-1). */
	return wifi_bridge_run(sh, RTL8720_UART_LOG, 115200u, true);
}

/* wifi rpc [baud]: eRPC link bring-up (issue #5).  Power on the RTL8720DN, open the
 * eRPC UART (USART1, default 2 Mbaud = the factory firmware's Serial3) and round-trip
 * a byte through rpc_system_ack -- a valid CRC-framed echo proves the eRPC link
 * (transport + framing + codec + the Serial3<->USART1 mapping) end to end. */
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
 * to stay well under one byte time on the wire.  NOTE the ISR runs from OCTOSPI2 XIP,
 * so the FIRST invocation after a cold I-cache pays an external-flash fetch: on board
 * #2 the maximum was 8.7 us over 7 interrupts (`wifi rpc`) but 3.3 us over 133
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

/*
 * Extract N from a "2.1.3+wio-nN" build id; 0 if the string is not one of ours.
 *
 * The generation is what gates every firmware-dependent capability (see
 * erpc_set_module_gen), so it has to be read as an ordered number: a string compare
 * against the newest id withdraws capabilities as soon as the next one is flashed, and a
 * compare against an old one keeps claiming them after a downgrade.
 */
static uint8_t fw_gen_of(const char *ver)
{
	const char *p = strstr(ver, "wio-n");
	unsigned n = 0u;

	if (p == NULL)
		return 0u;
	p += 5;                                     /* past "wio-n" */
	if (*p < '0' || *p > '9')
		return 0u;
	while (*p >= '0' && *p <= '9' && n < 255u)
		n = n * 10u + (unsigned)(*p++ - '0');
	return (uint8_t)((n > 255u) ? 255u : n);
}

static int cmd_wifi_rpc(struct cli_instance *sh, int argc, char **argv)
{
	uint32_t baud = rtl_link_erpc_baud();       /* whatever the link runs at now */
	uint8_t echoed = 0u;
	struct erpc_diag diag = {0}, total = {0};
	struct rtl8720_uart_stats st = {0};
	int rc = -1, tries;
	char ver[64];
	int have_ver = 0;
	int want_version = 0;                       /* `wifi rpc ver`: opt-in build-id query */
	int ai = 1;                                 /* index of the optional baud argument */

	/* `wifi rpc ver [baud]` additionally reads the firmware build id.  It is opt-in
	 * because the query is only safe against issue-#20 N2+ firmware: the pre-N2 shim
	 * erpc_free()s a string literal and corrupts the module heap (recoverable with
	 * `wifi reset`).  Plain `wifi rpc` never sends it, so it stays safe on any FW. */
	if (argc >= 2 && strcmp(argv[1], "ver") == 0) {
		want_version = 1;
		ai = 2;
	}
	/* Upper bound is the module's own UART ceiling (rtl8721d_usi_uart.h: 110..6000000),
	 * raised from 2 M when issue #23 U0-3 made the link rate changeable -- this is a
	 * one-shot probe at a given rate, though; `link baud` is what CHANGES the link. */
	if (argc > ai && (parse_u32(argv[ai], &baud) != 0 ||
	    baud < 2400u || baud > 6000000u)) {
		cli_error(sh, "wifi: bad baud (2400..6000000)\r\n");
		return 1;
	}
	/* Its own claim rather than rtl_link_begin(): the baud is caller-chosen here, and a
	 * non-default baud must not join an existing 2 Mbaud session (rtl_link_uart_ref
	 * refuses that anyway).  allow_busy = false for the same reason. */
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
	if (want_version && rc == 0 && echoed == 0x5Au) {
		/* Opt-in only (see the `ver` note above): read the build id while the link is
		 * up.  Safe here because the caller asserted N2+ firmware by typing `ver`. */
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
			erpc_set_module_gen(fw_gen_of(ver));
	}
	/* Snapshot before the unref: the counters survive the close (they are only reset
	 * by rtl8720_uart_open) but reading them here keeps them tied to this session. */
	rtl8720_uart_stats(&st);
	rtl_link_uart_unref();
	rtl_link_hw_release(sh);

	if (rc == 0 && echoed == 0x5Au) {
		cli_print(sh, "wifi: eRPC OK -- ack 0x5A -> 0x%02X, link up @%lu (%d tries)\r\n",
		          echoed, (unsigned long)baud, tries + 1);
		if (want_version) {
			if (have_ver)
				cli_print(sh, "  fw version: %s\r\n", ver);
			else
				cli_print(sh, "  fw version: unavailable (pre-N2 firmware, or query failed)\r\n");
		}
		/* What the host currently believes it may put on the wire.  Printed on every
		 * successful link test, not only for `ver`, because it is the one piece of state
		 * a reader cannot infer: it outlives this command (the UART reference does not)
		 * but not a reset or a flash session. */
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
		cli_print(sh, "  hint: try `wifi rpc 614400`, or `wifi probe` to confirm boot\r\n");
	return (rc == 0 && echoed == 0x5Au) ? 0 : 1;
}

/* wifi connect <ssid> [password] [security_hex] (issue #5 inc 3): put the module in
 * STA mode, associate with the AP, run the DHCP client and print the leased address.
 * The steps are synchronous eRPC calls on the 2 Mbaud USART1 link; connect blocks on
 * the module up to ~15 s and DHCP up to ~30 s (both abortable with Ctrl+C).  Default
 * security is WPA2-AES with a password / OPEN without; a 3rd hex arg overrides it. */
static int cmd_wifi_connect(struct cli_instance *sh, int argc, char **argv)
{
	const char *ssid = argv[1];
	const char *pass = (argc >= 3) ? argv[2] : NULL;
	struct wifi_rpc_opts o;
	struct erpc_diag diag;
	struct wifi_ip_info ip;
	uint32_t security;
	int32_t result = -1;
	int rc;

	if (argc >= 4) {
		if (parse_u32(argv[3], &security) != 0) {
			cli_error(sh, "wifi: bad security hex (e.g. 0x00400004)\r\n");
			return 1;
		}
	} else {
		security = pass ? WIFI_RPC_SEC_WPA2_AES_PSK : WIFI_RPC_SEC_OPEN;
	}

	/* Re-associating underneath a live host stack would take the module's lwIP,
	 * DHCP client and netif address out from under the bridge, and this flow holds
	 * the coarse mutex for longer than the bridge's watchdog (issue #23 U3). */
	if (nx_net_guard(sh, "wifi connect"))
		return 1;
	if (rtl_link_begin(sh, true) != RTL_LINK_READY)
		return 1;

	o.should_abort = rtl_abort_cb;
	o.abort_ctx    = sh;
	o.diag         = &diag;

	/* 0) Bring up the module's lwIP stack (tcpip_adapter_init = LwIP_Init) BEFORE the
	 * WiFi driver (re)binds in step 1: the factory firmware never inits lwIP at boot
	 * (setup() only calls wifi_init()), so without this the STA netif never exists and
	 * LwIP_DHCP(DHCP_START) blocks forever.  Once per power-on (state survives off/on). */
	if (!rtl_tcpip_inited()) {
		o.timeout_ms = 5000u;
		rc = wifi_rpc_tcpip_init(&o, &result);
		if (rc || result != WIFI_RPC_OK) {
			cli_error(sh, "wifi: tcpip/lwIP init failed (rc %d, result %ld)\r\n",
			          rc, (long)result);
			goto fail;
		}
		rtl_tcpip_set_inited(true);
	}

	/* 1) STA mode.  Boot leaves WiFi running in RTW_MODE_NONE, and wifi_on() then
	 * early-returns "already running" (1) WITHOUT switching mode.  So cycle off then
	 * on(STA) -- the rpcWiFi mode() sequence -- to land cleanly in STA.  wifi_on
	 * returns 0 (fresh) or 1 (already on); only RTW_ERROR (<0) is a real failure. */
	o.timeout_ms = 5000u;
	(void)wifi_rpc_off(&o, &result);            /* best effort: ignore off's result */
	if (cli_sleep(sh, 50u))                     /* let the driver settle (Ctrl+C ok) */
		goto fail;
	o.timeout_ms = 5000u;
	rc = wifi_rpc_on(&o, WIFI_RPC_MODE_STA, &result);
	if (rc == -4) { cli_print(sh, "wifi: aborted\r\n"); goto fail; }
	if (rc || result < 0) {
		cli_error(sh, "wifi: set STA mode failed (rc %d, result %ld)\r\n",
		          rc, (long)result);
		goto fail;
	}

	/* 2) associate (module blocks until connected or it gives up). */
	cli_print(sh, "wifi: connecting to \"%s\"%s...\r\n", ssid, pass ? "" : " (open)");
	o.timeout_ms = 15000u;
	rc = wifi_rpc_connect(&o, ssid, pass, security, &result);
	if (rc == -4) { cli_print(sh, "wifi: aborted\r\n"); goto fail; }
	if (rc || result != WIFI_RPC_OK) {
		cli_error(sh, "wifi: connect failed (rc %d, result %ld)\r\n", rc, (long)result);
		if (argc < 4)
			cli_print(sh, "  hint: try an explicit security, e.g. "
			          "`wifi connect \"%s\" <pw> 0x00600006`\r\n", ssid);
		goto fail;
	}

	/* 3) DHCP.  The module's LwIP_DHCP(DHCP_START) blocks until a lease is assigned,
	 * retrying internally -- that can run past 15 s on a slow network, so give the
	 * host wait generous headroom (Ctrl+C still aborts; IWDG is a separate thread). */
	cli_print(sh, "wifi: associated; requesting DHCP lease (up to ~30s)...\r\n");
	o.timeout_ms = 30000u;
	rc = wifi_rpc_dhcpc_start(&o, 0u, &result);
	if (rc == -4) { cli_print(sh, "wifi: aborted\r\n"); goto fail; }
	if (rc || result != WIFI_RPC_OK) {
		cli_error(sh, "wifi: DHCP failed (rc %d, result %ld)\r\n", rc, (long)result);
		goto fail;
	}
	rtl_set_ip_mode(RTL_IP_DHCP);               /* address obtained via DHCP */

	/* 4) read back the assigned address. */
	o.timeout_ms = 3000u;
	rc = wifi_rpc_get_ip(&o, 0u, &ip, &result);
	if (rc || result != WIFI_RPC_OK) {
		cli_error(sh, "wifi: get IP failed (rc %d, result %ld)\r\n", rc, (long)result);
		goto fail;
	}

	rtl_link_end(sh);
	/* No net_shell_autoarm() here any more: this address belongs to the MODULE's lwIP,
	 * and since issue #23 U4-2 the telnet console is a NetX socket on the host stack.
	 * `net up` + `net dhcp` is what arms it now. */
	cli_print(sh, "wifi: connected\r\n");
	cli_print(sh, "  ip   %u.%u.%u.%u\r\n", ip.ip[0], ip.ip[1], ip.ip[2], ip.ip[3]);
	cli_print(sh, "  mask %u.%u.%u.%u\r\n",
	          ip.netmask[0], ip.netmask[1], ip.netmask[2], ip.netmask[3]);
	cli_print(sh, "  gw   %u.%u.%u.%u\r\n", ip.gw[0], ip.gw[1], ip.gw[2], ip.gw[3]);
	return 0;

fail:
	rtl_link_end(sh);
	cli_print(sh, "  diag: crc_fail %u oversize %u timeout %u skipped %u unsupported %u\r\n",
	          diag.crc_fail, diag.oversize, diag.timeout, diag.skipped_reply,
	          diag.unsupported_invocation);
	cli_print(sh, "  note: if the module seems stuck, `wifi reset` power-cycles it\r\n");
	return 1;
}

/* wifi disconnect: drop the current association (no power-on -- pure query). */
static int cmd_wifi_disconnect(struct cli_instance *sh, int argc, char **argv)
{
	struct wifi_rpc_opts o;
	struct erpc_diag diag;
	int32_t result = -1;
	int rc, link;

	(void)argc; (void)argv;
	if (nx_net_guard(sh, "wifi disconnect"))
		return 1;
	link = rtl_link_begin(sh, false);
	if (link == RTL_LINK_OFF) {
		cli_print(sh, "wifi: powered off (nothing to disconnect)\r\n");
		return 0;
	}
	if (link != RTL_LINK_READY)
		return 1;

	o.should_abort = rtl_abort_cb;
	o.abort_ctx    = sh;
	o.diag         = &diag;
	o.timeout_ms   = 5000u;
	rc = wifi_rpc_disconnect(&o, &result);
	rtl_link_end(sh);

	if (rc || result != WIFI_RPC_OK) {
		cli_error(sh, "wifi: disconnect failed (rc %d, result %ld)\r\n",
		          rc, (long)result);
		cli_print(sh, "  diag: crc_fail %u oversize %u timeout %u skipped %u unsupported %u\r\n",
		          diag.crc_fail, diag.oversize, diag.timeout, diag.skipped_reply,
		          diag.unsupported_invocation);
		return 1;
	}
	rtl_set_ip_mode(RTL_IP_UNKNOWN);            /* association dropped: no address */
	cli_print(sh, "wifi: disconnected\r\n");
	return 0;
}

/* wifi status: report association state, RSSI, IP config and MAC (pure query). */
static int cmd_wifi_status(struct cli_instance *sh, int argc, char **argv)
{
	struct wifi_rpc_opts o;
	struct erpc_diag diag;
	struct wifi_ip_info ip;
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

	o.should_abort = rtl_abort_cb;
	o.abort_ctx    = sh;
	o.diag         = &diag;
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
		if (wifi_rpc_get_ip(&o, 0u, &ip, &result) == 0 && result == WIFI_RPC_OK) {
			cli_print(sh, "  ip   %u.%u.%u.%u\r\n",
			          ip.ip[0], ip.ip[1], ip.ip[2], ip.ip[3]);
			cli_print(sh, "  mask %u.%u.%u.%u\r\n",
			          ip.netmask[0], ip.netmask[1], ip.netmask[2], ip.netmask[3]);
			cli_print(sh, "  gw   %u.%u.%u.%u\r\n",
			          ip.gw[0], ip.gw[1], ip.gw[2], ip.gw[3]);
		}
	}
	rtl_link_end(sh);
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
	/* A scan retunes the radio, which is the one the bridge is relaying through. */
	if (nx_net_guard(sh, "wifi scan"))
		return 1;
	if (rtl_link_begin(sh, true) != RTL_LINK_READY)
		return 1;

	o.should_abort = rtl_abort_cb;
	o.abort_ctx    = sh;
	o.diag         = &diag;

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
		o.timeout_ms = 5000u;
		(void)wifi_rpc_off(&o, &result);       /* best effort, as in connect */
		if (cli_sleep(sh, 50u))
			goto fail;
		o.timeout_ms = 5000u;
		rc = wifi_rpc_on(&o, WIFI_RPC_MODE_STA, &result);
		if (rc == -4) { cli_print(sh, "wifi: aborted\r\n"); goto fail; }
		if (rc || result < 0) {
			cli_error(sh, "wifi: set STA mode failed (rc %d, result %ld)\r\n",
			          rc, (long)result);
			goto fail;
		}
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
	cli_print(sh, "  diag: crc_fail %u oversize %u timeout %u skipped %u unsupported %u\r\n",
	          diag.crc_fail, diag.oversize, diag.timeout, diag.skipped_reply,
	          diag.unsupported_invocation);
	cli_print(sh, "  note: if the module seems stuck, `wifi reset` power-cycles it\r\n");
	return 1;
}

/* wifi flashprobe [hold_us] (issue #19, M1): prove RTL8720DN UART download-mode ENTRY
 * without touching flash.  Drives the strap (PD14) + reset (PC3) to enter download mode,
 * then issues the read-only download read-word command and checks for the framed reply.
 * Tries raw framing first (matches the pvvx reference tool), then SLIP as an exploratory
 * probe.  ALWAYS power-cycles the module back to its normal eRPC firmware on exit.  No
 * erase / no write -- fully reversible (the mask-ROM download mode is re-enterable). */
static int cmd_wifi_flashprobe(struct cli_instance *sh, int argc, char **argv)
{
	uint32_t hold_us = 30000u;                 /* board #2: 2ms too short, 20ms latches (M1) */
	struct rtl_dl_result res_raw, res_slip;
	const struct rtl_dl_result *hit = NULL;
	int rc, slip_tried = 0, i;

	if (argc >= 2 && (parse_u32(argv[1], &hold_us) != 0 || hold_us > 50000u)) {
		cli_error(sh, "wifi: bad hold_us (0..50000)\r\n");
		return 1;
	}
	/* bg-reject / busy-reject (the download path re-opens the UART itself), and the
	 * coarse link mutex for the whole session: no HW is touched until both are held. */
	if (rtl_link_hw_claim(sh, false) != 0)
		return 1;

	cli_print(sh, "wifi: entering RTL8720 UART download mode "
	          "(strap PD14 low / reset PC3, hold %luus)...\r\n", (unsigned long)hold_us);
	rc = rtl_dl_enter(hold_us, rtl_abort_cb, sh);
	if (rc == -4) { cli_print(sh, "wifi: aborted\r\n"); goto recover; }
	if (rc != 0)  { cli_error(sh, "wifi: UART9 did not come ready (rc %d)\r\n", rc); goto recover; }

	rc = rtl_dl_probe(0, 500u, rtl_abort_cb, sh, &res_raw);   /* raw first (pvvx-style) */
	if (rc == -4) { cli_print(sh, "wifi: aborted\r\n"); goto recover; }
	if (res_raw.entered) {
		hit = &res_raw;
	} else {
		slip_tried = 1;                    /* exploratory: is the framing SLIP? */
		rc = rtl_dl_probe(1, 500u, rtl_abort_cb, sh, &res_slip);
		if (rc == -4) { cli_print(sh, "wifi: aborted\r\n"); goto recover; }
		if (res_slip.entered)
			hit = &res_slip;
	}

	if (hit) {
		cli_print(sh, "wifi: DOWNLOAD MODE ENTERED (%s framing)\r\n",
		          hit->slip ? "SLIP" : "raw");
		cli_print(sh, "  read-word @0x00082000 = 0x%08lX%s\r\n", (unsigned long)hit->word,
		          hit->word == 0x00082021u ? "  (flashloader stub already resident)" : "");
	} else {
		cli_print(sh, "wifi: download mode NOT entered (no framed reply)\r\n");
		cli_print(sh, "  hint: retry with a longer strap hold, e.g. `wifi flashprobe 50000`\r\n");
	}
	cli_print(sh, "  raw rx (%d B):", res_raw.raw_len);
	for (i = 0; i < res_raw.raw_len; i++)
		cli_print(sh, " %02X", res_raw.raw[i]);
	cli_print(sh, "\r\n");
	if (slip_tried) {
		cli_print(sh, "  slip rx (%d B):", res_slip.raw_len);
		for (i = 0; i < res_slip.raw_len; i++)
			cli_print(sh, " %02X", res_slip.raw[i]);
		cli_print(sh, "\r\n");
	}
	if (res_raw.overflows)
		cli_print(sh, "  uart9 overflows: %lu\r\n", (unsigned long)res_raw.overflows);

recover:
	/* Always: close UART9, power-cycle the module back to its normal eRPC firmware, and
	 * invalidate the host-tracked lwIP / IP state (the module rebooted). */
	rtl8720_uart_close();
	rtl8720_reset();
	rtl_link_forget_module();       /* it rebooted: rate, firmware, lwIP all unknown */
	rtl_link_hw_release(sh);
	cli_print(sh, "wifi: RTL8720 reset to normal firmware\r\n");
	return hit ? 0 : 1;
}

/* wifi flashload [hold_us] [baud] (issue #19, M2): NON-DESTRUCTIVE proof of the next
 * download-protocol layer.  Enters download mode (M1), uploads the AmebaD flashloader
 * stub into the module SRAM + raises the link baud, then READS flash sector 0 and checks
 * the km0_boot magic.  It writes NO flash (SRAM stub + flash read only), so it stays
 * fully reversible.  ALWAYS power-cycles the module back to its normal eRPC firmware. */
static int cmd_wifi_flashload(struct cli_instance *sh, int argc, char **argv)
{
	static const uint8_t km0_magic[8] = { 0x99, 0x99, 0x96, 0x96, 0x3f, 0xcc, 0x66, 0xfc };
	uint32_t hold_us = 30000u, baud = 1500000u;
	uint8_t sect0[128];
	int rc, ok = 0;

	if (argc >= 2 && (parse_u32(argv[1], &hold_us) != 0 || hold_us > 50000u)) {
		cli_error(sh, "wifi: bad hold_us (0..50000)\r\n");
		return 1;
	}
	if (argc >= 3 && (parse_u32(argv[2], &baud) != 0 ||
	    (baud != 115200u && baud != 1500000u))) {
		cli_error(sh, "wifi: bad baud (115200 or 1500000)\r\n");
		return 1;
	}
	/* bg-reject / busy-reject (the download path re-opens the UART itself), and the
	 * coarse link mutex for the whole session: no HW is touched until both are held. */
	if (rtl_link_hw_claim(sh, false) != 0)
		return 1;

	cli_print(sh, "wifi: download + flashloader (hold %luus, baud %lu, NON-DESTRUCTIVE)...\r\n",
	          (unsigned long)hold_us, (unsigned long)baud);
	rc = rtl_dl_enter(hold_us, rtl_abort_cb, sh);
	if (rc == -4) { cli_print(sh, "wifi: aborted\r\n"); goto recover; }
	if (rc != 0)  { cli_error(sh, "wifi: UART9 did not come ready (rc %d)\r\n", rc); goto recover; }

	rc = rtl_dl_load_flashloader(baud, rtl_abort_cb, sh);
	if (rc != 0) {
		cli_error(sh, "wifi: flashloader load failed (rc %d) -- block xfer / baud issue\r\n", rc);
		goto recover;
	}
	cli_print(sh, "wifi: flashloader resident @0x00082000 (read-word 0x00082021), link @%lu\r\n",
	          (unsigned long)baud);

	rc = rtl_dl_read_flash(0u, 1u, sect0, sizeof(sect0), rtl_abort_cb, sh);
	if (rc < 0) {
		cli_error(sh, "wifi: flash read failed (rc %d)\r\n", rc);
		goto recover;
	}
	ok = (rc >= 8 && memcmp(sect0, km0_magic, 8) == 0);
	cli_print(sh, "wifi: read %d B of flash @0x0; km0_boot magic %s\r\n",
	          rc, ok ? "OK (99 99 96 96 3f cc 66 fc)" : "MISMATCH");
	cli_hexdump(sh, sect0, (rc < 64) ? (size_t)rc : 64u);

recover:
	/* Always: close UART9, power-cycle back to the normal eRPC firmware, invalidate state. */
	rtl8720_uart_close();
	rtl8720_reset();
	rtl_link_forget_module();       /* it rebooted: rate, firmware, lwIP all unknown */
	rtl_link_hw_release(sh);
	cli_print(sh, "wifi: RTL8720 reset to normal firmware\r\n");
	return ok ? 0 : 1;
}

/* wifi flashread <offset> [nsectors] (issue #19, M3): NON-DESTRUCTIVE flash survey -- read
 * sectors and show whether each looks erased (helps pick an unused sector for flashtest). */
static int cmd_wifi_flashread(struct cli_instance *sh, int argc, char **argv)
{
	uint32_t offset, nsectors = 1u, s;
	uint8_t buf[128];
	int rc, ok = 0;

	if (parse_u32(argv[1], &offset) != 0 || (offset & 0xFFFu) != 0u) {
		cli_error(sh, "wifi: bad offset (4KB-aligned hex, e.g. 0x180000)\r\n");
		return 1;
	}
	if (argc >= 3 && (parse_u32(argv[2], &nsectors) != 0 || nsectors == 0u || nsectors > 64u)) {
		cli_error(sh, "wifi: bad nsectors (1..64)\r\n");
		return 1;
	}
	/* bg-reject / busy-reject (the download path re-opens the UART itself), and the
	 * coarse link mutex for the whole session: no HW is touched until both are held. */
	if (rtl_link_hw_claim(sh, false) != 0)
		return 1;

	cli_print(sh, "wifi: reading flash (NON-DESTRUCTIVE)...\r\n");
	rc = rtl_dl_enter(30000u, rtl_abort_cb, sh);
	if (rc == -4) { cli_print(sh, "wifi: aborted\r\n"); goto recover; }
	if (rc != 0)  { cli_error(sh, "wifi: UART9 did not come ready (rc %d)\r\n", rc); goto recover; }
	rc = rtl_dl_load_flashloader(1500000u, rtl_abort_cb, sh);
	if (rc != 0) { cli_error(sh, "wifi: flashloader load failed (rc %d)\r\n", rc); goto recover; }

	ok = 1;
	for (s = 0u; s < nsectors; s++) {
		uint32_t off = offset + s * 4096u;
		int i, allff = 1;

		rc = rtl_dl_read_flash(off, 1u, buf, sizeof(buf), rtl_abort_cb, sh);
		if (rc < 0) {
			cli_error(sh, "wifi: read @0x%lX failed (rc %d)\r\n", (unsigned long)off, rc);
			ok = 0;
			break;
		}
		for (i = 0; i < rc; i++)
			if (buf[i] != 0xFFu) { allff = 0; break; }
		cli_print(sh, "0x%06lX: first %d B %s\r\n", (unsigned long)off, rc,
		          allff ? "all 0xFF (looks erased)" : "has data");
		cli_hexdump_base(sh, buf, (rc < 64) ? (size_t)rc : 64u, off);
	}

recover:
	rtl8720_uart_close();
	rtl8720_reset();
	rtl_link_forget_module();       /* it rebooted: rate, firmware, lwIP all unknown */
	rtl_link_hw_release(sh);
	cli_print(sh, "wifi: RTL8720 reset to normal firmware\r\n");
	return ok ? 0 : 1;
}

/* wifi flashtest <offset> confirm (issue #19, M3): DESTRUCTIVE erase/write/verify self-test
 * on ONE unused (all-0xFF) 4 KB sector, restored to 0xFF afterwards.  Hard-gated in
 * rtl_dl_flash_selftest (range [0x100000,0x200000) + erasable-content check), and requires
 * the literal `confirm` token.  Never touches boot/app.  Always resets the module back. */
static int cmd_wifi_flashtest(struct cli_instance *sh, int argc, char **argv)
{
	struct rtl_dl_selftest r;
	uint32_t offset;
	int rc, i, ok = 0;

	if (parse_u32(argv[1], &offset) != 0) {
		cli_error(sh, "wifi: bad offset (hex, e.g. 0x180000)\r\n");
		return 1;
	}
	if (argc < 3 || strcmp(argv[2], "confirm") != 0) {
		cli_error(sh, "wifi: DESTRUCTIVE (erases+writes a flash sector). "
		          "Re-run `wifi flashtest 0x%lX confirm` to proceed.\r\n", (unsigned long)offset);
		return 1;
	}
	/* bg-reject / busy-reject (the download path re-opens the UART itself), and the
	 * coarse link mutex for the whole session: no HW is touched until both are held. */
	if (rtl_link_hw_claim(sh, false) != 0)
		return 1;

	cli_print(sh, "wifi: flash erase/write/verify self-test @0x%lX "
	          "(DESTRUCTIVE; power-cycles the module to verify)...\r\n", (unsigned long)offset);
	rc = rtl_dl_flash_selftest(offset, 30000u, &r, rtl_abort_cb, sh);
	if (rc == -1) {
		cli_error(sh, "wifi: offset outside the safe test range "
		          "[0x100000, 0x200000), 4KB-aligned\r\n");
		goto recover;
	}
	if (rc == -2) {
		cli_error(sh, "wifi: download / flashloader setup failed (rc %d)\r\n", rc);
		goto recover;
	}
	if (rc == -3) {
		cli_error(sh, "wifi: refusing -- sector 0x%lX is not erased/unused (foreign data)\r\n",
		          (unsigned long)offset);
		cli_print(sh, "  first 16 B:");
		for (i = 0; i < 16; i++)
			cli_print(sh, " %02X", r.found[i]);
		cli_print(sh, "\r\n  pick an all-0xFF sector (use `wifi flashread`)\r\n");
		goto recover;
	}
	cli_print(sh, "  gate:    %s\r\n", r.gate_ok ?
	          (r.gate_was_ff ? "erasable (all 0xFF)" : "erasable (our leftover pattern)") : "FAIL");
	cli_print(sh, "  erase:   %s\r\n", r.erase_ok ? "OK (all 0xFF)" : "FAIL");
	cli_print(sh, "  write:   %s\r\n",
	          r.write_ok ? "OK (pattern verified after re-enter)" :
	          (rc == -5) ? "FAIL (block send)" :
	          (rc == -7) ? "FAIL (verify read after re-enter)" :
	          (rc == -8) ? "FAIL (verify mismatch)" : "-");
	if (rc == -7 || rc == -8)
		cli_print(sh, "           (read rc %d)\r\n", r.rc_detail);
	cli_print(sh, "  restore: %s\r\n", r.restore_ok ? "OK (re-erased to 0xFF)" :
	          (r.dirty ? "FAIL -- sector DIRTY" : "-"));
	if (rc == 0) {
		cli_print(sh, "wifi: PASSED -- erase/write/verify OK, sector restored to 0xFF\r\n");
		ok = 1;
	} else {
		cli_error(sh, "wifi: FAILED (rc %d)\r\n", rc);
		if (r.dirty)
			cli_print(sh, "  note: sector 0x%lX left with data; re-run "
			          "`wifi flashtest 0x%lX confirm` to heal, or pick another offset\r\n",
			          (unsigned long)offset, (unsigned long)offset);
	}

recover:
	rtl8720_uart_close();
	rtl8720_reset();
	rtl_link_forget_module();       /* it rebooted: rate, firmware, lwIP all unknown */
	rtl_link_hw_release(sh);
	cli_print(sh, "wifi: RTL8720 reset to normal firmware\r\n");
	return ok ? 0 : 1;
}

/* ------------------------------------------------------------------ *
 *  issue #19 M4: capacity detection + full-chip backup (READ-ONLY)
 * ------------------------------------------------------------------ */

/* Open a download session: enter download mode + load the flashloader at 1.5 Mbaud.
 * Returns 0 on success (caller owns the session and must reach its `recover:` label). */
static int flash_session_open(struct cli_instance *sh)
{
	int rc = rtl_dl_enter(30000u, rtl_abort_cb, sh);

	if (rc == -4) { cli_print(sh, "wifi: aborted\r\n"); return -1; }
	if (rc != 0)  { cli_error(sh, "wifi: UART9 did not come ready (rc %d)\r\n", rc); return -1; }
	rc = rtl_dl_load_flashloader(1500000u, rtl_abort_cb, sh);
	if (rc != 0)  { cli_error(sh, "wifi: flashloader load failed (rc %d)\r\n", rc); return -1; }
	return 0;
}

/* Print the detected capacity, or explain why it could not be determined.  Returns the
 * size to use (0 = unknown). */
static uint32_t flash_report_size(struct cli_instance *sh, const struct rtl_dl_size *sz, int rc)
{
	if (rc == -3) {
		cli_warn(sh, "  capacity: UNKNOWN -- the first 8 KB reads all 0xFF, so there is "
		          "no data to detect the address wrap against\r\n");
		return 0u;
	}
	if (rc != 0) {
		cli_error(sh, "  capacity: probe failed (rc %d)\r\n", rc);
		return 0u;
	}
	if (sz->size == 0u) {
		cli_warn(sh, "  capacity: UNKNOWN -- no address wrap up to 8 MB "
		          "(chip is >= 16 MB, or does not wrap)\r\n");
		return 0u;
	}
	cli_print(sh, "  capacity: %lu MB (0x%lX) -- address wrap at that offset, "
	          "8 KB compared byte-for-byte\r\n",
	          (unsigned long)(sz->size >> 20), (unsigned long)sz->size);
	return sz->size;
}

/* wifi flashinfo (issue #19, M4): NON-DESTRUCTIVE flash identification.
 *
 * Runs in TWO download sessions on purpose.  Two operations each want to be last:
 * the 0x27 checksum (a timeout may still be answered later and would desynchronise
 * whatever follows) and the experimental RDID probe (its command shape is not
 * established by the reference tool, so a mis-framed reply may desynchronise too).
 * Session A therefore ends with the checksum, and the RDID probe gets a fresh session
 * of its own -- the same "power-cycle and re-enter" pattern rtl_dl_flash_selftest uses.
 * Only session A's wrap detection is authoritative; everything else is a diagnostic. */
static int cmd_wifi_flashinfo(struct cli_instance *sh, int argc, char **argv)
{
	struct rtl_dl_size sz;
	struct rtl_dl_jedec jd;
	uint8_t sr[3];
	uint32_t size, sum = 0u;
	int rc, ok = 0;

	(void)argc; (void)argv;

	/* bg-reject / busy-reject (the download path re-opens the UART itself), and the
	 * coarse link mutex for the whole session: no HW is touched until both are held. */
	if (rtl_link_hw_claim(sh, false) != 0)
		return 1;

	cli_print(sh, "wifi: identifying RTL8720 flash (NON-DESTRUCTIVE)...\r\n");
	if (flash_session_open(sh) != 0)
		goto recover;

	/* --- session A, step 1: capacity by address wrap (proven read path only) --- */
	rc = rtl_dl_detect_size(&sz, rtl_abort_cb, sh);
	size = flash_report_size(sh, &sz, rc);
	if (rc == -2)
		goto recover;                       /* the read path itself failed */
	ok = (size != 0u);                          /* success == the capacity is known */

	/* --- session A, step 2: status registers (reference-tool command shape only) --- */
	rc = rtl_dl_flash_status(sr, rtl_abort_cb, sh);
	if (rc != 0) {
		cli_warn(sh, "  status:   read failed (rc %d)\r\n", rc);
		goto recover;                       /* link is unhappy; do not push further */
	}
	cli_print(sh, "  status:   SR1 0x%02X  SR2 0x%02X  SR3 0x%02X\r\n",
	          sr[0], sr[1], sr[2]);

	/* --- session A, step 3 (LAST in this session): device-side checksum --- */
	rc = rtl_dl_flash_chksum(0u, 0x10000u, 5000u, &sum, rtl_abort_cb, sh);
	if (rc == 0)
		cli_print(sh, "  chksum:   0x%08lX over the first 64 KB (device-side, 0x27)\r\n",
		          (unsigned long)sum);
	else
		cli_warn(sh, "  chksum:   n/a (rc %d) -- ending the session, a late reply "
		          "would desynchronise it\r\n", rc);

	/* --- session B: the experimental RDID probe, alone in a fresh session --- */
	rtl8720_uart_close();
	cli_print(sh, "  (re-entering download mode for the experimental JEDEC probe)\r\n");
	if (flash_session_open(sh) != 0)
		goto recover;
	rc = rtl_dl_flash_jedec(&jd, rtl_abort_cb, sh);
	if (rc == 0 && jd.ok) {
		cli_print(sh, "  jedec:    %02X %02X %02X", jd.id[0], jd.id[1], jd.id[2]);
		if (jd.size != 0u)
			cli_print(sh, " -> %lu MB", (unsigned long)(jd.size >> 20));
		cli_print(sh, "  (experimental; cross-check only)\r\n");
		if (jd.size != 0u && size != 0u && jd.size != size)
			cli_warn(sh, "  NOTE: JEDEC disagrees with the wrap probe -- trusting the "
			          "wrap probe (%lu MB)\r\n", (unsigned long)(size >> 20));
	} else {
		cli_print(sh, "  jedec:    not available (the 0x21 0x9F command shape is a "
		          "hypothesis, not confirmed by the reference tool)\r\n");
	}

recover:
	rtl8720_uart_close();
	rtl8720_reset();
	rtl_link_forget_module();       /* it rebooted: rate, firmware, lwIP all unknown */
	rtl_link_hw_release(sh);
	cli_print(sh, "wifi: RTL8720 reset to normal firmware\r\n");
	return ok ? 0 : 1;
}

/*
 * YMODEM byte source over the RTL8720 flash (issue #19 M4).
 *
 * ymodem_send() pulls; rtl_dl_read_flash() streams whole sectors, so we refill a
 * chunk-sized staging buffer and serve slices out of it.  A 32 KB chunk = 8 sectors =
 * one read command per 32 blocks, which keeps the per-command overhead off the wire;
 * drop RTL_BACKUP_CHUNK to 4096 to fall back to the single-sector reads proven in M2.
 *
 * NOTE the NULL abort hook in bak_src_read(): while ymodem_send() runs, its io_getc()
 * is the ONLY permitted reader of the console RX ring.  rtl_abort_cb() would call
 * cli_cancel_poll(), which drains that ring and discards every non-0x03 byte -- it
 * would eat the receiver's ACK/'C'/CAN and break the transfer.  Ctrl+C during the
 * transfer is handled by io_getc() instead (cmd_xfer.c).
 */
#define RTL_BACKUP_CHUNK  (8u * 4096u)

static uint8_t s_bak_chunk[RTL_BACKUP_CHUNK];   /* static: off the 4 KB shell stack */

struct rtl_bak_src {
	uint32_t base;        /* flash offset of stream byte 0 (4 KB-aligned) */
	uint32_t total;       /* bytes to send */
	uint32_t pos;         /* bytes served so far */
	uint32_t chunk_pos;   /* stream position of s_bak_chunk[0] */
	uint32_t chunk_len;   /* valid bytes in s_bak_chunk (0 = empty) */
	/* Running digest of everything served, in the module's own 0x27 algorithm, so the
	 * backup can be verified against the device without re-reading the flash.  The
	 * algorithm lives in one place (app/rtl8720_flash.c) and is shared with the M5
	 * staged-image checker -- see struct rtl_dl_digest. */
	struct rtl_dl_digest dg;
	int      failed;      /* sticky: a flash read failed */
};

static int bak_src_read(void *ctx, uint8_t *dst, uint32_t want, uint32_t *got)
{
	struct rtl_bak_src *s = (struct rtl_bak_src *)ctx;
	uint32_t avail, n;

	*got = 0u;
	if (s->pos >= s->total)
		return 0;                                   /* EOF */

	if (s->chunk_len == 0u || s->pos >= s->chunk_pos + s->chunk_len) {
		uint32_t remain = s->total - s->pos;
		uint32_t take   = (remain > RTL_BACKUP_CHUNK) ? RTL_BACKUP_CHUNK : remain;
		uint32_t secs   = (take + 4095u) / 4096u;   /* reads are whole sectors */
		int rc = rtl_dl_read_flash(s->base + s->pos, secs, s_bak_chunk,
		                           sizeof(s_bak_chunk), NULL, NULL);

		if (rc < (int)take) {                       /* short/failed read */
			s->failed = 1;
			return -1;
		}
		s->chunk_pos = s->pos;
		s->chunk_len = take;
	}

	avail = s->chunk_len - (s->pos - s->chunk_pos);
	n = (want < avail) ? want : avail;
	if (n > s->total - s->pos)
		n = s->total - s->pos;
	memcpy(dst, s_bak_chunk + (s->pos - s->chunk_pos), n);
	rtl_dl_digest_add(&s->dg, dst, n);
	s->pos += n;
	*got = n;
	return 0;
}

/* Append @v as @digits uppercase hex digits at @p; returns the new write position. */
static char *bak_put_hex(char *p, uint32_t v, int digits)
{
	static const char hex[] = "0123456789ABCDEF";

	for (int i = digits - 1; i >= 0; i--)
		*p++ = hex[(v >> (4 * i)) & 0xFu];
	return p;
}

/* Build the deterministic YMODEM block-0 filename "rtl8720_<off6>_<len6>.bin" into
 * @buf (needs >= 28 bytes).  No printf: the shell has no snprintf. */
static void bak_build_name(char *buf, uint32_t off, uint32_t len)
{
	const char *pre = "rtl8720_", *suf = ".bin";
	char *p = buf;

	while (*pre)
		*p++ = *pre++;
	p = bak_put_hex(p, off, 6);
	*p++ = '_';
	p = bak_put_hex(p, len, 6);
	while (*suf)
		*p++ = *suf++;
	*p = '\0';
}

/* wifi flashbackup [offset] [len] (issue #19, M4): NON-DESTRUCTIVE full-chip backup.
 * Streams the flash to the PC over the console with YMODEM (receive with `rz`).
 * Defaults to the whole chip as detected by the address-wrap probe. */
static int cmd_wifi_flashbackup(struct cli_instance *sh, int argc, char **argv)
{
	struct rtl_dl_size sz;
	struct rtl_bak_src src_ctx;
	struct ym_source   src;
	char name[32];
	uint32_t offset = 0u, len = 0u, size, devsum = 0u;
	int rc, ok = 0;

	if (argc >= 2 && (parse_u32(argv[1], &offset) != 0 || (offset & 0xFFFu) != 0u)) {
		cli_error(sh, "wifi: bad offset (4KB-aligned, e.g. 0x0)\r\n");
		return 1;
	}
	if (argc >= 3 && (parse_u32(argv[2], &len) != 0 || len == 0u || (len & 0xFFFu) != 0u)) {
		cli_error(sh, "wifi: bad length (4KB-aligned, non-zero, e.g. 0x1000)\r\n");
		return 1;
	}
	/* Reject a range the protocol cannot express (24-bit offsets) BEFORE powering
	 * anything up: otherwise an explicit oversized range on a chip whose capacity we
	 * failed to detect would start the YMODEM transfer and then die part-way through,
	 * leaving the receiver holding a truncated file. */
	if (offset >= RTL_DL_FLASH_LIMIT ||
	    (len != 0u && len > RTL_DL_FLASH_LIMIT - offset)) {
		cli_error(sh, "wifi: range past the protocol's 16 MB (24-bit offset) limit\r\n");
		return 1;
	}
	/* bg-reject / busy-reject (the download path re-opens the UART itself), and the
	 * coarse link mutex for the whole session: no HW is touched until both are held. */
	if (rtl_link_hw_claim(sh, false) != 0)
		return 1;

	cli_print(sh, "wifi: flash backup (NON-DESTRUCTIVE)...\r\n");
	if (flash_session_open(sh) != 0)
		goto recover;

	rc = rtl_dl_detect_size(&sz, rtl_abort_cb, sh);
	size = flash_report_size(sh, &sz, rc);
	if (rc == -2)
		goto recover;
	if (len == 0u) {
		if (size == 0u) {
			cli_error(sh, "wifi: capacity unknown -- pass an explicit length, "
			          "e.g. `wifi flashbackup 0x0 0x200000`\r\n");
			goto recover;
		}
		len = size - offset;
	}
	if (size != 0u && (offset >= size || len > size - offset)) {
		cli_error(sh, "wifi: range 0x%lX+0x%lX is past the detected 0x%lX capacity\r\n",
		          (unsigned long)offset, (unsigned long)len, (unsigned long)size);
		goto recover;
	}

	memset(&src_ctx, 0, sizeof(src_ctx));
	src_ctx.base = offset;
	src_ctx.total = len;
	rtl_dl_digest_init(&src_ctx.dg);

	bak_build_name(name, offset, len);
	src.ctx = &src_ctx; src.name = name; src.size = len; src.read = bak_src_read;

	cli_print(sh, "wifi: sending '%s' (%lu bytes) -- start the receiver now "
	          "(`rz`, or Ctrl+A Ctrl+R in picocom)\r\n", name, (unsigned long)len);
	/* From here until the transfer ends, io_getc() owns the console RX (see above). */
	rc = xfer_send_source_locked(sh, &src);
	if (rc != 0) {
		if (src_ctx.failed)
			cli_error(sh, "wifi: flash read failed %lu bytes in\r\n",
			          (unsigned long)src_ctx.pos);
		goto recover;
	}
	cli_print(sh, "  host digest:   0x%08lX (u32-LE word sum of the bytes sent)\r\n",
	          (unsigned long)rtl_dl_digest_value(&src_ctx.dg));

	/* LAST operation of the session: the module's own digest over the same range, in
	 * the same algorithm -- so this is an END-TO-END VERIFY of the backup that does not
	 * depend on re-reading the flash.  A timeout poisons the session, so we only ever
	 * fall through to recover from here. */
	rc = rtl_dl_flash_chksum(offset, len, 30000u, &devsum, rtl_abort_cb, sh);
	if (rc != 0) {
		cli_warn(sh, "  device digest: n/a (rc %d) -- backup UNVERIFIED\r\n", rc);
	} else if (devsum == rtl_dl_digest_value(&src_ctx.dg)) {
		cli_print(sh, "  device digest: 0x%08lX -- VERIFIED (matches)\r\n",
		          (unsigned long)devsum);
		ok = 1;
	} else {
		cli_error(sh, "  device digest: 0x%08lX -- MISMATCH, the backup is NOT trustworthy\r\n",
		          (unsigned long)devsum);
	}

recover:
	rtl8720_uart_close();
	rtl8720_reset();
	rtl_link_forget_module();       /* it rebooted: rate, firmware, lwIP all unknown */
	rtl_link_hw_release(sh);
	cli_print(sh, "wifi: RTL8720 reset to normal firmware\r\n");
	return ok ? 0 : 1;
}

/* ------------------------------------------------------------------ *
 *  issue #19 M5: image staging (host -> PSRAM) + programming the module
 * ------------------------------------------------------------------ */

/* Print the staged-image record, re-reading PSRAM to catch a clobber.  Returns 1 when a
 * valid image is present AND still intact, 0 otherwise (message already emitted). */
static int img_report(struct cli_instance *sh)
{
	const struct rtl_img *im = rtl_img_get();
	uint32_t              now;
	int                   i;

	if (!im->valid) {
		cli_warn(sh, "wifi: no image staged -- run `wifi imgload` first\r\n");
		return 0;
	}
	cli_print(sh, "  name:    '%s'\r\n", im->name);
	cli_print(sh, "  size:    %lu bytes (padded to %lu = %lu sectors with 0xFF)\r\n",
	          (unsigned long)im->len, (unsigned long)im->padded_len,
	          (unsigned long)(im->padded_len / 4096u));
	cli_print(sh, "  digest:  0x%08lX (device 0x27 algorithm, over the padded range)\r\n",
	          (unsigned long)im->digest);
	cli_print(sh, "  first16:");
	for (i = 0; i < 16; i++)
		cli_print(sh, " %02X", rtl_img_data()[i]);
	cli_print(sh, "%s\r\n",
	          memcmp(rtl_img_data(), rtl_dl_km0_magic, RTL_DL_KM0_MAGIC_LEN) == 0
	                  ? "   (AmebaD km0_boot magic)" : "");

	now = rtl_img_verify();
	if (now != im->digest) {
		cli_error(sh, "  RECHECK: 0x%08lX -- PSRAM was CLOBBERED since the load "
		          "(another command used it); re-run `wifi imgload`\r\n",
		          (unsigned long)now);
		return 0;
	}
	cli_print(sh, "  recheck: 0x%08lX -- intact\r\n", (unsigned long)now);
	return 1;
}

/*
 * wifi imgload (issue #19, M5): receive a firmware image from the PC over YMODEM into
 * the PSRAM staging buffer.  Touches NO RTL8720 hardware at all -- this is purely the
 * host-to-board transfer, and it is what makes the stock backup restorable.
 *
 * Console RX ownership: from cli_console_claim() until xfer_recv_sink_locked() returns,
 * the ONLY reader of the console RX ring is that helper's io_getc().  The sink must not
 * poll cli_cancel_requested() (see cmd_xfer.h) -- here it physically cannot, since
 * rtl_img_sink() has no abort hook to pass.
 */
static int cmd_wifi_imgload(struct cli_instance *sh, int argc, char **argv)
{
	const struct rtl_img *im;
	uint32_t              drops0;
	int                   rc, ok = 0;

	(void)argc; (void)argv;

	if (!psram_ready()) {
		cli_error(sh, "wifi: PSRAM is not available -- nowhere to stage the image\r\n");
		return 1;
	}
	if (cli_console_claim(sh) != 0) {
		cli_error(sh, "wifi: run in the foreground (not `wifi ... &`)\r\n");
		return 1;
	}
	/* Hold the OCTOSPI1 guard for the WHOLE transfer so a backgrounded psram/membench
	 * job cannot overwrite the staging area while it fills. */
	if (!psram_acquire()) {
		cli_console_release(sh);
		cli_error(sh, "wifi: PSRAM is busy (another command holds it)\r\n");
		return 1;
	}

	rc = rtl_img_probe();               /* invalidates, then proves PSRAM stores data */
	if (rc != 0) {
		cli_error(sh, "wifi: PSRAM staging self-check failed (rc %d)\r\n", rc);
		goto out;
	}

	drops0 = sh->rx_dropped;
	cli_print(sh, "wifi: staging an RTL8720 image in PSRAM @0x%08lX (max %lu bytes). "
	          "NOTHING is written to the module.\r\n",
	          (unsigned long)PSRAM_BASE_ADDR, (unsigned long)RTL_IMG_MAX);
	/* xfer_recv_sink_locked() does not print this (its caller owns the console), but
	 * the handshake budget is only long enough if the operator starts now. */
	cli_print(sh, "wifi: start the sender now -- `sb <file>` (lrzsz YMODEM batch send; "
	          "`sz` will NOT work), or Ctrl+A Ctrl+S in picocom; Ctrl+C aborts\r\n");
	rc = xfer_recv_sink_locked(sh, rtl_img_sink());

	/* Print the transfer post-mortem BEFORE branching, so a failure is as
	 * informative as a success -- the counters say which layer broke (see
	 * struct ym_recv_diag). */
	{
		const struct ym_recv_diag *d = ymodem_recv_diag();

		cli_print(sh, "  ymodem: %lu blocks ok, %lu bad-crc, %lu bad-seq, "
		          "%lu short-read, %lu header-timeouts\r\n",
		          (unsigned long)d->blocks, (unsigned long)d->bad_crc,
		          (unsigned long)d->bad_seq, (unsigned long)d->short_read,
		          (unsigned long)d->timeouts);
		if (d->first_kind >= 0)
			cli_print(sh, "  first bad block: kind 0x%02X seq %d ~seq %d, "
			          "body %lu/%lu B, crc want %04X got %04X\r\n",
			          (unsigned)d->first_kind, d->first_seq, d->first_nseq,
			          (unsigned long)d->first_got, (unsigned long)d->first_want,
			          d->first_crc_want, d->first_crc_got);
		/* The console backend drops -- and counts -- a byte when its RX ring
		 * overruns.  A non-zero count here means the loss is below YMODEM. */
		cli_print(sh, "  rx drops during the transfer: %lu%s\r\n",
		          (unsigned long)(sh->rx_dropped - drops0),
		          (sh->rx_dropped - drops0) ? "  <-- NOT CLEAN" : "  (clean)");
		/* Also to the log ring: the PC's terminal is still attached to `sb` when
		 * these lines go out, so `dmesg` is where they can actually be read. */
		log_write((sh->rx_dropped - drops0) ? LOG_LEVEL_WRN : LOG_LEVEL_INF, "wifi",
		          "imgload rc=%d rx_drops=%lu", rc,
		          (unsigned long)(sh->rx_dropped - drops0));
	}

	if (rc != 0) {
		/* Includes "all the data arrived but the batch never closed" -- not a
		 * complete image, so it must not be left staged. */
		rtl_img_invalidate();
		goto out;
	}
	if (rtl_img_finish() != 0) {
		cli_error(sh, "wifi: empty transfer -- nothing staged\r\n");
		goto out;
	}

	im = rtl_img_get();
	cli_print(sh, "wifi: staged '%s', %lu bytes\r\n",
	          im->name, (unsigned long)im->len);
	cli_print(sh, "  padded: %lu bytes (%lu sectors, 0xFF filled)\r\n",
	          (unsigned long)im->padded_len, (unsigned long)(im->padded_len / 4096u));
	cli_print(sh, "  digest: 0x%08lX -- compare with the host-side u32-LE word sum\r\n",
	          (unsigned long)im->digest);
	ok = 1;

out:
	psram_release();
	cli_console_release(sh);
	return ok ? 0 : 1;
}

/* wifi imginfo (issue #19, M5): show the staged image and re-verify it against PSRAM. */
static int cmd_wifi_imginfo(struct cli_instance *sh, int argc, char **argv)
{
	int ok;

	(void)argc; (void)argv;

	if (!psram_acquire()) {
		cli_error(sh, "wifi: PSRAM is busy (another command holds it)\r\n");
		return 1;
	}
	ok = img_report(sh);
	psram_release();
	return ok ? 0 : 1;
}

/* YMODEM source over the staged PSRAM image (issue #19, M5): sends it straight back to
 * the PC so the round trip can be checked with `cmp`, which proves BYTE equality --
 * something the 32-bit device digest alone cannot. */
struct rtl_img_src { uint32_t pos, total; };

static int img_src_read(void *ctx, uint8_t *dst, uint32_t want, uint32_t *got)
{
	struct rtl_img_src *s = (struct rtl_img_src *)ctx;
	uint32_t            n = s->total - s->pos;

	if (n > want)
		n = want;
	memcpy(dst, rtl_img_data() + s->pos, n);
	s->pos += n;
	*got = n;
	return 0;
}

/* wifi imgsend (issue #19, M5): stream the staged image back to the PC over YMODEM. */
static int cmd_wifi_imgsend(struct cli_instance *sh, int argc, char **argv)
{
	const struct rtl_img *im = rtl_img_get();
	struct rtl_img_src    src_ctx = { 0u, 0u };
	struct ym_source      src;
	int                   rc, ok = 0;

	(void)argc; (void)argv;

	if (!im->valid) {
		cli_error(sh, "wifi: no image staged -- run `wifi imgload` first\r\n");
		return 1;
	}
	if (cli_console_claim(sh) != 0) {
		cli_error(sh, "wifi: run in the foreground (not `wifi ... &`)\r\n");
		return 1;
	}
	if (!psram_acquire()) {
		cli_console_release(sh);
		cli_error(sh, "wifi: PSRAM is busy (another command holds it)\r\n");
		return 1;
	}

	/* This command exists to prove byte equality with the host's file, so sending a
	 * silently clobbered buffer would defeat its whole purpose. */
	if (rtl_img_verify() != im->digest) {
		cli_error(sh, "wifi: staged image no longer matches its digest (PSRAM was "
		          "clobbered) -- re-run `wifi imgload`\r\n");
		goto out;
	}

	src_ctx.total = im->padded_len;
	src.ctx = &src_ctx; src.name = im->name;
	src.size = im->padded_len; src.read = img_src_read;

	cli_print(sh, "wifi: sending the staged image '%s' (%lu bytes, padded) -- start the "
	          "receiver now (`rb`, or Ctrl+A Ctrl+R in picocom)\r\n",
	          im->name, (unsigned long)im->padded_len);
	rc = xfer_send_source_locked(sh, &src);
	ok = (rc == 0);

out:
	psram_release();
	cli_console_release(sh);
	return ok ? 0 : 1;
}

/*
 * wifi flashwrite <offset> confirm (issue #19, M5): DESTRUCTIVE.  Erase and program the
 * staged image into the RTL8720DN's flash at <offset>, then verify it with the module's
 * own digest.  THIS IS THE COMMAND THAT CAN REWRITE THE MODULE'S BOOT SECTORS.
 *
 * The gates are layered on purpose (see rtl_dl_flash_program): here we require the
 * literal `confirm` token and re-verify the staged image against PSRAM, and the protocol
 * layer enforces alignment, the 2 MB destructive cap, the AmebaD boot magic at offset 0,
 * and the detected chip capacity.  Recovery if this ever goes wrong: re-enter download
 * mode (mask ROM -- always possible) and re-run with the full 2 MB stock backup staged.
 */
static int cmd_wifi_flashwrite(struct cli_instance *sh, int argc, char **argv)
{
	const struct rtl_img *im = rtl_img_get();
	struct rtl_dl_program pr;
	uint32_t              offset, len;
	int                   rc, ok = 0;

	if (parse_u32(argv[1], &offset) != 0) {
		cli_error(sh, "wifi: bad offset (hex, e.g. 0x0)\r\n");
		return 1;
	}
	if (!im->valid) {
		cli_error(sh, "wifi: no image staged -- run `wifi imgload` first\r\n");
		return 1;
	}
	len = im->padded_len;
	if (argc < 3 || strcmp(argv[2], "confirm") != 0) {
		cli_error(sh, "wifi: DESTRUCTIVE -- erases and rewrites %lu bytes of RTL8720 "
		          "flash at 0x%lX.\r\n", (unsigned long)len, (unsigned long)offset);
		cli_print(sh, "  re-run `wifi flashwrite 0x%lX confirm` to proceed\r\n",
		          (unsigned long)offset);
		return 1;
	}
	/* The factory WiFi-settings sector holds the SSID and a plaintext PSK; erasing it
	 * is legitimate for a full-chip restore but must never be a surprise. */
	if (offset <= 0x105000u && 0x105000u < offset + len)
		cli_warn(sh, "wifi: NOTE this range covers 0x105000, the factory WiFi settings "
		         "sector -- the stored SSID/password will be replaced\r\n");

	/* bg-reject / busy-reject (the download path re-opens the UART itself), and the
	 * coarse link mutex for the whole session: no HW is touched until both are held. */
	if (rtl_link_hw_claim(sh, false) != 0)
		return 1;
	if (!psram_acquire()) {                 /* the image is read straight out of PSRAM */
		rtl_link_hw_release(sh);
		cli_error(sh, "wifi: PSRAM is busy (another command holds it)\r\n");
		return 1;
	}
	/* Last gate before any hardware moves: the staged bytes must still be the ones we
	 * digested at load time. */
	if (rtl_img_verify() != im->digest) {
		cli_error(sh, "wifi: staged image no longer matches its digest (PSRAM was "
		          "clobbered) -- re-run `wifi imgload`\r\n");
		goto out;
	}

	cli_print(sh, "wifi: programming '%s' -> flash 0x%lX..0x%lX (%lu sectors), "
	          "DESTRUCTIVE; power-cycles the module to verify...\r\n",
	          im->name, (unsigned long)offset, (unsigned long)(offset + len - 1u),
	          (unsigned long)(len / 4096u));
	/* erase + transfer + re-enter + digest; there is no progress output in between,
	 * so say so rather than let a long silence look like a hang. */
	cli_print(sh, "  (silent for roughly %lu s: erase, block transfer, power-cycle, "
	          "digest. Ctrl+C aborts.)\r\n",
	          (unsigned long)(10u + (len / 4096u) / 8u + (len >> 17)));
	rc = rtl_dl_flash_program(offset, rtl_img_data(), len, 30000u, &pr,
	                          rtl_abort_cb, sh);

	switch (rc) {
	case -1:
		cli_error(sh, "wifi: bad range -- 4KB-aligned and within 0x%lX required\r\n",
		          (unsigned long)0x200000u);
		goto recover;
	case -2:
		cli_error(sh, "wifi: download / flashloader setup failed\r\n");
		goto recover;
	case -3:
		cli_error(sh, "wifi: refusing -- writing offset 0 requires an AmebaD km0_boot "
		          "image (magic 99 99 96 96 3F CC 66 FC)\r\n");
		goto recover;
	case -4:
		cli_error(sh, "wifi: range past the detected 0x%lX capacity -- nothing "
		          "erased\r\n", (unsigned long)pr.cap);
		goto recover;
	case -10:
		cli_error(sh, "wifi: could not read the flash to size it -- nothing erased. "
		          "Retry; if it persists, check the link with `wifi flashinfo`\r\n");
		goto recover;
	default:
		break;
	}

	if (pr.cap_known)
		cli_print(sh, "  capacity: %lu MB (address wrap)\r\n",
		          (unsigned long)(pr.cap >> 20));
	cli_print(sh, "  erase:  %s (%lu/%lu sectors)\r\n", pr.erase_ok ? "OK" : "FAIL",
	          (unsigned long)pr.erased, (unsigned long)pr.sectors);
	cli_print(sh, "  write:  %s (%lu bytes)\r\n", pr.write_ok ? "OK" : "FAIL",
	          (unsigned long)pr.written);
	cli_print(sh, "  host digest:   0x%08lX\r\n", (unsigned long)pr.host_sum);
	if (rc == -8) {
		cli_error(sh, "  device digest: n/a -- UNVERIFIED\r\n");
	} else if (rc == -9) {
		cli_error(sh, "  device digest: 0x%08lX -- MISMATCH\r\n",
		          (unsigned long)pr.dev_sum);
	} else if (rc == 0) {
		cli_print(sh, "  device digest: 0x%08lX -- VERIFIED (matches)\r\n",
		          (unsigned long)pr.dev_sum);
	}

	if (rc == 0) {
		cli_print(sh, "wifi: PROGRAMMED and verified\r\n");
		ok = 1;
	} else {
		cli_error(sh, "wifi: FAILED (rc %d)\r\n", rc);
		cli_print(sh, "  the range is now INDETERMINATE. Re-run "
		          "`wifi flashwrite 0x%lX confirm` (erase+write is idempotent); if it "
		          "keeps failing, stage the full 2 MB backup and write it at 0x0.\r\n",
		          (unsigned long)offset);
	}

recover:
	rtl8720_uart_close();
	rtl8720_reset();
	rtl_link_forget_module();       /* it rebooted: rate, firmware, lwIP all unknown */
	cli_print(sh, "wifi: RTL8720 reset to normal firmware\r\n");
out:
	psram_release();
	rtl_link_hw_release(sh);
	return ok ? 0 : 1;
}

CLI_SUBCMD_SET_CREATE(wifi_subcmds,
	CLI_CMD_ARG(info,  NULL, "show RTL8720 wiring + CHIP_EN state",       cmd_wifi_info,  1, 0),
	CLI_CMD_ARG(on,    NULL, "CHIP_EN high (power on RTL8720)",           cmd_wifi_on,    1, 0),
	CLI_CMD_ARG(off,   NULL, "CHIP_EN low (power off)",                   cmd_wifi_off,   1, 0),
	CLI_CMD_ARG(reset, NULL, "power-cycle CHIP_EN (low 80ms -> high)",    cmd_wifi_reset, 1, 0),
	CLI_CMD_ARG(log,   NULL, "bridge LOG UART (UART9 @115200)",           cmd_wifi_log,   1, 0),
	CLI_CMD_ARG(probe, NULL, "reset + capture boot log from t=0",         cmd_wifi_probe, 1, 0),
	CLI_CMD_ARG(rpc,   NULL, "eRPC link test (rpc_system_ack) [ver] [baud, default 2M]", cmd_wifi_rpc, 1, 2),
	CLI_CMD_ARG(connect,    NULL, "associate + DHCP: connect <ssid> [pw] [sec_hex]", cmd_wifi_connect,    2, 2),
	CLI_CMD_ARG(disconnect, NULL, "drop the current WiFi association",     cmd_wifi_disconnect, 1, 0),
	CLI_CMD_ARG(status,     NULL, "show connection state / RSSI / IP / MAC", cmd_wifi_status, 1, 0),
	CLI_CMD_ARG(scan,       NULL, "list visible APs (ch/band/rssi/security/bssid/ssid)", cmd_wifi_scan, 1, 0),
	CLI_CMD_ARG(flashprobe, NULL, "probe RTL8720 UART download-mode entry [hold_us]", cmd_wifi_flashprobe, 1, 1),
	CLI_CMD_ARG(flashload,  NULL, "load flashloader + read flash sector0 (non-destructive) [hold] [baud]", cmd_wifi_flashload, 1, 2),
	CLI_CMD_ARG(flashread,  NULL, "read flash <offset> [nsectors] (non-destructive survey)", cmd_wifi_flashread, 2, 1),
	CLI_CMD_ARG(flashtest,  NULL, "DESTRUCTIVE erase/write/verify test <offset> confirm", cmd_wifi_flashtest, 2, 1),
	CLI_CMD_ARG(flashinfo,  NULL, "identify flash: capacity / status regs / checksum", cmd_wifi_flashinfo, 1, 0),
	CLI_CMD_ARG(flashbackup, NULL, "back up flash to the PC over YMODEM [offset] [len]", cmd_wifi_flashbackup, 1, 2),
	CLI_CMD_ARG(imgload,    NULL, "receive a firmware image from the PC into PSRAM (YMODEM `sb`)", cmd_wifi_imgload, 1, 0),
	CLI_CMD_ARG(imginfo,    NULL, "show + re-verify the staged firmware image",        cmd_wifi_imginfo, 1, 0),
	CLI_CMD_ARG(imgsend,    NULL, "send the staged image back to the PC over YMODEM",  cmd_wifi_imgsend, 1, 0),
	CLI_CMD_ARG(flashwrite, NULL, "DESTRUCTIVE program staged image: flashwrite <offset> confirm", cmd_wifi_flashwrite, 2, 1),
	CLI_SUBCMD_SET_END);

CLI_CMD_REGISTER(wifi, wifi_subcmds,
                 "onboard RTL8720DN WiFi/BLE firmware probe", NULL, 1, 0);
