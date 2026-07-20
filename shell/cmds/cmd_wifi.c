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
#include "erpc.h"
#include "wifi_rpc.h"

#include <stdint.h>
#include <stdbool.h>

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

/* Abort thunk handed to erpc_call_ex(): non-zero once the user pressed Ctrl+C. */
static int wifi_abort_cb(void *ctx)
{
	return cli_cancel_requested((struct cli_instance *)ctx) ? 1 : 0;
}

/* wifi_link_begin() result. */
enum { WLINK_READY = 0, WLINK_ERR = 1, WLINK_OFF = 2 };

/* Whether the module's lwIP stack (tcpip_adapter_init = LwIP_Init) has been brought
 * up since its last power-on.  The factory firmware does NOT init lwIP at boot, so
 * the host must -- once per CHIP_EN power cycle (lwIP state survives wifi off/on but
 * not a power cycle).  Reset whenever the module is powered off / reset / freshly
 * powered on, set after a successful wifi_rpc_tcpip_init(). */
static bool g_wifi_tcpip_inited;

/*
 * Claim the console (single owner of the SPSC RX ring / one eRPC call in flight),
 * ensure the module is powered, and open the eRPC UART (USART1 @2 Mbaud).
 *   @power_on true : power the module and wait for boot if it is off (for `connect`).
 *   @power_on false: if the module is off, release and return WLINK_OFF so the
 *                    caller can report "powered off" without powering it (status /
 *                    disconnect are pure queries).
 * Returns WLINK_READY with the console claimed + UART open (caller must
 * wifi_link_end()), WLINK_OFF (nothing claimed), or WLINK_ERR (nothing claimed,
 * message already printed).
 */
static int wifi_link_begin(struct cli_instance *sh, bool power_on)
{
	if (cli_console_claim(sh) != 0) {              /* bg-reject / single owner */
		cli_error(sh, "wifi: run in the foreground (not `wifi ... &`)\r\n");
		return WLINK_ERR;
	}
	if (!rtl8720_powered()) {
		if (!power_on) {
			cli_console_release(sh);
			return WLINK_OFF;
		}
		cli_print(sh, "wifi: powering on RTL8720DN, waiting ~1.5s for boot...\r\n");
		rtl8720_power(true);
		g_wifi_tcpip_inited = false;           /* fresh boot: lwIP not yet up */
		if (cli_sleep(sh, 1500u)) {            /* cancellable boot wait */
			cli_console_release(sh);
			return WLINK_ERR;
		}
	}
	if (rtl8720_uart_open(RTL8720_UART_AT, 2000000u) != 0) {
		cli_console_release(sh);
		cli_error(sh, "wifi: USART1 @2000000 did not come ready\r\n");
		return WLINK_ERR;
	}
	return WLINK_READY;
}

/* Tear down a wifi_link_begin() session (close the UART, release the console). */
static void wifi_link_end(struct cli_instance *sh)
{
	rtl8720_uart_close();
	cli_console_release(sh);
}

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
		g_wifi_tcpip_inited = false;            /* power-cycled: lwIP must be re-inited */
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
	if (!rtl8720_powered())
		g_wifi_tcpip_inited = false;           /* fresh power-on: lwIP not yet up */
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
	g_wifi_tcpip_inited = false;               /* module state lost on power-off */
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
	g_wifi_tcpip_inited = false;               /* power-cycled: lwIP must be re-inited */
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
		g_wifi_tcpip_inited = false;       /* fresh boot: lwIP not yet up */
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

	if (wifi_link_begin(sh, true) != WLINK_READY)
		return 1;

	o.should_abort = wifi_abort_cb;
	o.abort_ctx    = sh;
	o.diag         = &diag;

	/* 0) Bring up the module's lwIP stack (tcpip_adapter_init = LwIP_Init) BEFORE the
	 * WiFi driver (re)binds in step 1: the factory firmware never inits lwIP at boot
	 * (setup() only calls wifi_init()), so without this the STA netif never exists and
	 * LwIP_DHCP(DHCP_START) blocks forever.  Once per power-on (state survives off/on). */
	if (!g_wifi_tcpip_inited) {
		o.timeout_ms = 5000u;
		rc = wifi_rpc_tcpip_init(&o, &result);
		if (rc || result != WIFI_RPC_OK) {
			cli_error(sh, "wifi: tcpip/lwIP init failed (rc %d, result %ld)\r\n",
			          rc, (long)result);
			goto fail;
		}
		g_wifi_tcpip_inited = true;
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

	/* 4) read back the assigned address. */
	o.timeout_ms = 3000u;
	rc = wifi_rpc_get_ip(&o, 0u, &ip, &result);
	if (rc || result != WIFI_RPC_OK) {
		cli_error(sh, "wifi: get IP failed (rc %d, result %ld)\r\n", rc, (long)result);
		goto fail;
	}

	wifi_link_end(sh);
	cli_print(sh, "wifi: connected\r\n");
	cli_print(sh, "  ip   %u.%u.%u.%u\r\n", ip.ip[0], ip.ip[1], ip.ip[2], ip.ip[3]);
	cli_print(sh, "  mask %u.%u.%u.%u\r\n",
	          ip.netmask[0], ip.netmask[1], ip.netmask[2], ip.netmask[3]);
	cli_print(sh, "  gw   %u.%u.%u.%u\r\n", ip.gw[0], ip.gw[1], ip.gw[2], ip.gw[3]);
	return 0;

fail:
	wifi_link_end(sh);
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
	link = wifi_link_begin(sh, false);
	if (link == WLINK_OFF) {
		cli_print(sh, "wifi: powered off (nothing to disconnect)\r\n");
		return 0;
	}
	if (link != WLINK_READY)
		return 1;

	o.should_abort = wifi_abort_cb;
	o.abort_ctx    = sh;
	o.diag         = &diag;
	o.timeout_ms   = 5000u;
	rc = wifi_rpc_disconnect(&o, &result);
	wifi_link_end(sh);

	if (rc || result != WIFI_RPC_OK) {
		cli_error(sh, "wifi: disconnect failed (rc %d, result %ld)\r\n",
		          rc, (long)result);
		cli_print(sh, "  diag: crc_fail %u oversize %u timeout %u skipped %u unsupported %u\r\n",
		          diag.crc_fail, diag.oversize, diag.timeout, diag.skipped_reply,
		          diag.unsupported_invocation);
		return 1;
	}
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
	link = wifi_link_begin(sh, false);
	if (link == WLINK_OFF) {
		cli_print(sh, "wifi: powered off (`wifi connect <ssid> ...` to bring up)\r\n");
		return 0;
	}
	if (link != WLINK_READY)
		return 1;

	o.should_abort = wifi_abort_cb;
	o.abort_ctx    = sh;
	o.diag         = &diag;
	o.timeout_ms   = 3000u;

	rc = wifi_rpc_is_connected(&o, &connected);
	if (rc) {
		cli_error(sh, "wifi: query failed (rc %d)\r\n", rc);
		wifi_link_end(sh);
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
	wifi_link_end(sh);
	return 0;
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
	CLI_SUBCMD_SET_END);

CLI_CMD_REGISTER(wifi, wifi_subcmds,
                 "onboard RTL8720DN WiFi/BLE firmware probe", NULL, 1, 0);
