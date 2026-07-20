/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    cmd_wifi.c
 * @brief   `wifi` shell command (issue #17): RTL8720DN factory-firmware probe.
 *
 *   wifi info          show wiring + CHIP_EN state
 *   wifi on | off      drive CHIP_EN high (power on) / low (power off)
 *   wifi reset         power-cycle CHIP_EN (Low 80 ms -> High)
 *   wifi log           bridge the LOG UART (UART9 @115200) <-> console
 *   wifi probe         reset the module and capture its boot log from t=0
 *   wifi at [baud]     bridge the AT/HS UART (USART1) <-> console (AT default 38400)
 *
 * The bridge takes over the console (issue #50 raw API): RTL8720DN RX bytes stream
 * to the CDC console and console keystrokes go to the module's TX, so the operator
 * reads the boot banner (identifying eRPC / AT / raw Realtek firmware) and can hand-
 * type AT commands.  Ctrl+C exits.  Foreground only.
 *
 * Clean-room design; no third-party code reused.
 */
#include "cli.h"
#include "rtl8720.h"

#include <stdint.h>

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
	if (do_reset)
		rtl8720_reset();                        /* Low->High edge AFTER we listen */
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
	cli_print(sh, "  AT  UART: USART1 PA10/PB14           (`wifi at [baud]`, AT default 38400)\r\n");
	return 0;
}

static int cmd_wifi_on(struct cli_instance *sh, int argc, char **argv)
{
	(void)argc; (void)argv;
	rtl8720_power(true);
	cli_print(sh, "wifi: CHIP_EN high (RTL8720DN powered on)\r\n");
	return 0;
}

static int cmd_wifi_off(struct cli_instance *sh, int argc, char **argv)
{
	(void)argc; (void)argv;
	rtl8720_power(false);
	cli_print(sh, "wifi: CHIP_EN low (RTL8720DN powered off)\r\n");
	return 0;
}

static int cmd_wifi_reset(struct cli_instance *sh, int argc, char **argv)
{
	(void)argc; (void)argv;
	rtl8720_reset();
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

static int cmd_wifi_at(struct cli_instance *sh, int argc, char **argv)
{
	uint32_t baud = 38400u;             /* RTL8720DN AT-firmware default */

	if (argc >= 2 && (parse_u32(argv[1], &baud) != 0 ||
	    baud < 2400u || baud > 2000000u)) {
		cli_error(sh, "wifi: bad baud (2400..2000000)\r\n");
		return 1;
	}
	return wifi_bridge_run(sh, RTL8720_UART_AT, baud, false);
}

CLI_SUBCMD_SET_CREATE(wifi_subcmds,
	CLI_CMD_ARG(info,  NULL, "show RTL8720 wiring + CHIP_EN state",       cmd_wifi_info,  1, 0),
	CLI_CMD_ARG(on,    NULL, "CHIP_EN high (power on RTL8720)",           cmd_wifi_on,    1, 0),
	CLI_CMD_ARG(off,   NULL, "CHIP_EN low (power off)",                   cmd_wifi_off,   1, 0),
	CLI_CMD_ARG(reset, NULL, "power-cycle CHIP_EN (low 80ms -> high)",    cmd_wifi_reset, 1, 0),
	CLI_CMD_ARG(log,   NULL, "bridge LOG UART (UART9 @115200)",           cmd_wifi_log,   1, 0),
	CLI_CMD_ARG(probe, NULL, "reset + capture boot log from t=0",         cmd_wifi_probe, 1, 0),
	CLI_CMD_ARG(at,    NULL, "bridge AT UART (USART1) [baud, default 38400]", cmd_wifi_at, 1, 1),
	CLI_SUBCMD_SET_END);

CLI_CMD_REGISTER(wifi, wifi_subcmds,
                 "onboard RTL8720DN WiFi/BLE firmware probe", NULL, 1, 0);
