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
 * The eRPC subcommands (rpc / connect / disconnect / status) each claim the console
 * (cli_console_claim) before touching hardware: that rejects a background worker and
 * guarantees a single owner of the SPSC RX ring and one eRPC transaction in flight.
 * connect / DHCP block on the module for seconds, so those calls carry a long
 * timeout and an abort hook wired to Ctrl+C; aborting only stops the host-side wait
 * (the module keeps going -- `wifi reset` power-cycles it if it seems stuck).
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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

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
 * user presses Ctrl+C.  The console is claimed FIRST -- before any hardware action
 * -- so a background job (`wifi ... &`) is rejected without powering/opening
 * anything (cli_console_claim refuses a bg worker; the RX ring is a strict SPSC
 * pipe owned by the foreground).  Draining the module RX takes priority: when a
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

	if (cli_console_claim(sh) != 0) {              /* bg-reject / lock: no HW touched */
		cli_error(sh, "wifi: run in the foreground (not `wifi ... &`)\r\n");
		return 1;
	}
	if (rtl8720_uart_open(which, baud) != 0) {
		cli_console_release(sh);
		cli_error(sh, "wifi: %s did not come ready\r\n", name);
		return 1;
	}

	cli_print(sh, "wifi: bridging RTL8720DN %s @%lu%s. Ctrl+C to exit.\r\n",
	          name, (unsigned long)baud, do_reset ? " (reset first)" : "");
	if (do_reset) {
		rtl8720_reset();                        /* Low->High edge AFTER we listen */
		rtl_tcpip_set_inited(false);            /* power-cycled: lwIP must be re-inited */
		rtl_set_ip_mode(RTL_IP_UNKNOWN);        /* and any address is gone */
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
	rtl8720_uart_close();
	cli_console_release(sh);

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

/* Claim the console just long enough to drive CHIP_EN: this rejects a background
 * worker (`wifi off &`) and, since an eRPC flow holds the claim for its whole
 * duration, prevents a power/reset from cutting the module mid-connect. */
static int wifi_power_claim(struct cli_instance *sh)
{
	if (cli_console_claim(sh) != 0) {
		cli_error(sh, "wifi: run in the foreground (not `wifi ... &`)\r\n");
		return 1;
	}
	return 0;
}

static int cmd_wifi_on(struct cli_instance *sh, int argc, char **argv)
{
	(void)argc; (void)argv;
	if (wifi_power_claim(sh))
		return 1;
	if (!rtl8720_powered()) {
		rtl_tcpip_set_inited(false);           /* fresh power-on: lwIP not yet up */
		rtl_set_ip_mode(RTL_IP_UNKNOWN);
	}
	rtl8720_power(true);
	cli_console_release(sh);
	cli_print(sh, "wifi: CHIP_EN high (RTL8720DN powered on)\r\n");
	return 0;
}

static int cmd_wifi_off(struct cli_instance *sh, int argc, char **argv)
{
	(void)argc; (void)argv;
	if (wifi_power_claim(sh))
		return 1;
	rtl8720_power(false);
	rtl_tcpip_set_inited(false);               /* module state lost on power-off */
	rtl_set_ip_mode(RTL_IP_UNKNOWN);
	cli_console_release(sh);
	cli_print(sh, "wifi: CHIP_EN low (RTL8720DN powered off)\r\n");
	return 0;
}

static int cmd_wifi_reset(struct cli_instance *sh, int argc, char **argv)
{
	(void)argc; (void)argv;
	if (wifi_power_claim(sh))
		return 1;
	rtl8720_reset();
	rtl_tcpip_set_inited(false);               /* power-cycled: lwIP must be re-inited */
	rtl_set_ip_mode(RTL_IP_UNKNOWN);
	cli_console_release(sh);
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
static int cmd_wifi_rpc(struct cli_instance *sh, int argc, char **argv)
{
	uint32_t baud = 2000000u;
	uint8_t echoed = 0u;
	struct erpc_diag diag = {0}, total = {0};
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
	if (argc > ai && (parse_u32(argv[ai], &baud) != 0 ||
	    baud < 2400u || baud > 2000000u)) {
		cli_error(sh, "wifi: bad baud (2400..2000000)\r\n");
		return 1;
	}
	if (cli_console_claim(sh) != 0) {          /* bg-reject / single owner */
		cli_error(sh, "wifi: run in the foreground (not `wifi ... &`)\r\n");
		return 1;
	}
	if (!rtl8720_powered()) {
		cli_print(sh, "wifi: powering on RTL8720DN, waiting ~1.5s for boot...\r\n");
		rtl8720_power(true);
		rtl_tcpip_set_inited(false);       /* fresh boot: lwIP not yet up */
		rtl_set_ip_mode(RTL_IP_UNKNOWN);
		if (cli_sleep(sh, 1500u)) {         /* cancellable boot wait */
			cli_console_release(sh);
			return 1;
		}
	}
	if (rtl8720_uart_open(RTL8720_UART_AT, baud) != 0) {
		cli_console_release(sh);
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
	}
	rtl8720_uart_close();
	cli_console_release(sh);

	if (rc == 0 && echoed == 0x5Au) {
		cli_print(sh, "wifi: eRPC OK -- ack 0x5A -> 0x%02X, link up @%lu (%d tries)\r\n",
		          echoed, (unsigned long)baud, tries + 1);
		if (want_version) {
			if (have_ver)
				cli_print(sh, "  fw version: %s\r\n", ver);
			else
				cli_print(sh, "  fw version: unavailable (pre-N2 firmware, or query failed)\r\n");
		}
	} else {
		cli_error(sh, "wifi: eRPC FAILED -- ack 0x5A -> 0x%02X (rc %d)\r\n", echoed, rc);
	}
	cli_print(sh, "  diag: crc_fail %u oversize %u timeout %u skipped %u unsupported %u\r\n",
	          total.crc_fail, total.oversize, total.timeout, total.skipped_reply,
	          total.unsupported_invocation);
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
	if (cli_console_claim(sh) != 0) {          /* bg-reject / single owner, HW untouched */
		cli_error(sh, "wifi: run in the foreground (not `wifi ... &`)\r\n");
		return 1;
	}

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
	rtl_tcpip_set_inited(false);
	rtl_set_ip_mode(RTL_IP_UNKNOWN);
	cli_console_release(sh);
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
	if (cli_console_claim(sh) != 0) {
		cli_error(sh, "wifi: run in the foreground (not `wifi ... &`)\r\n");
		return 1;
	}

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
	rtl_tcpip_set_inited(false);
	rtl_set_ip_mode(RTL_IP_UNKNOWN);
	cli_console_release(sh);
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
	if (cli_console_claim(sh) != 0) {
		cli_error(sh, "wifi: run in the foreground (not `wifi ... &`)\r\n");
		return 1;
	}

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
	rtl_tcpip_set_inited(false);
	rtl_set_ip_mode(RTL_IP_UNKNOWN);
	cli_console_release(sh);
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
	if (cli_console_claim(sh) != 0) {
		cli_error(sh, "wifi: run in the foreground (not `wifi ... &`)\r\n");
		return 1;
	}

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
	rtl_tcpip_set_inited(false);
	rtl_set_ip_mode(RTL_IP_UNKNOWN);
	cli_console_release(sh);
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

	if (cli_console_claim(sh) != 0) {
		cli_error(sh, "wifi: run in the foreground (not `wifi ... &`)\r\n");
		return 1;
	}

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
	rtl_tcpip_set_inited(false);
	rtl_set_ip_mode(RTL_IP_UNKNOWN);
	cli_console_release(sh);
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
	if (cli_console_claim(sh) != 0) {
		cli_error(sh, "wifi: run in the foreground (not `wifi ... &`)\r\n");
		return 1;
	}

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
	rtl_tcpip_set_inited(false);
	rtl_set_ip_mode(RTL_IP_UNKNOWN);
	cli_console_release(sh);
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

	if (cli_console_claim(sh) != 0) {
		cli_error(sh, "wifi: run in the foreground (not `wifi ... &`)\r\n");
		return 1;
	}
	if (!psram_acquire()) {                 /* the image is read straight out of PSRAM */
		cli_console_release(sh);
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
	rtl_tcpip_set_inited(false);
	rtl_set_ip_mode(RTL_IP_UNKNOWN);
	cli_print(sh, "wifi: RTL8720 reset to normal firmware\r\n");
out:
	psram_release();
	cli_console_release(sh);
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
