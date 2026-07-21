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
 *   wifi rpc [baud]               eRPC link test: rpc_system_ack (default 2 Mbaud, #5)
 *   wifi connect <ssid> [pw] [sec]  associate (STA) + DHCP, print the IP (#5 inc 3)
 *   wifi disconnect               drop the current association
 *   wifi status                   connected? + RSSI + IP/mask/gw + MAC
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
#include "rtl8720.h"
#include "rtl8720_flash.h"
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

	if (argc >= 2 && (parse_u32(argv[1], &baud) != 0 ||
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
	rtl8720_uart_close();
	cli_console_release(sh);

	if (rc == 0 && echoed == 0x5Au) {
		cli_print(sh, "wifi: eRPC OK -- ack 0x5A -> 0x%02X, link up @%lu (%d tries)\r\n",
		          echoed, (unsigned long)baud, tries + 1);
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

CLI_SUBCMD_SET_CREATE(wifi_subcmds,
	CLI_CMD_ARG(info,  NULL, "show RTL8720 wiring + CHIP_EN state",       cmd_wifi_info,  1, 0),
	CLI_CMD_ARG(on,    NULL, "CHIP_EN high (power on RTL8720)",           cmd_wifi_on,    1, 0),
	CLI_CMD_ARG(off,   NULL, "CHIP_EN low (power off)",                   cmd_wifi_off,   1, 0),
	CLI_CMD_ARG(reset, NULL, "power-cycle CHIP_EN (low 80ms -> high)",    cmd_wifi_reset, 1, 0),
	CLI_CMD_ARG(log,   NULL, "bridge LOG UART (UART9 @115200)",           cmd_wifi_log,   1, 0),
	CLI_CMD_ARG(probe, NULL, "reset + capture boot log from t=0",         cmd_wifi_probe, 1, 0),
	CLI_CMD_ARG(rpc,   NULL, "eRPC link test (rpc_system_ack) [baud, default 2M]", cmd_wifi_rpc, 1, 1),
	CLI_CMD_ARG(connect,    NULL, "associate + DHCP: connect <ssid> [pw] [sec_hex]", cmd_wifi_connect,    2, 2),
	CLI_CMD_ARG(disconnect, NULL, "drop the current WiFi association",     cmd_wifi_disconnect, 1, 0),
	CLI_CMD_ARG(status,     NULL, "show connection state / RSSI / IP / MAC", cmd_wifi_status, 1, 0),
	CLI_CMD_ARG(flashprobe, NULL, "probe RTL8720 UART download-mode entry [hold_us]", cmd_wifi_flashprobe, 1, 1),
	CLI_CMD_ARG(flashload,  NULL, "load flashloader + read flash sector0 (non-destructive) [hold] [baud]", cmd_wifi_flashload, 1, 2),
	CLI_SUBCMD_SET_END);

CLI_CMD_REGISTER(wifi, wifi_subcmds,
                 "onboard RTL8720DN WiFi/BLE firmware probe", NULL, 1, 0);
