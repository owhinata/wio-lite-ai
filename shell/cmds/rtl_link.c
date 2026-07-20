/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/*
 * Shared RTL8720DN eRPC session helper for `wifi` (L2) and `net` (L3) -- issue #5.
 * See rtl_link.h.  Extracted verbatim from cmd_wifi.c so both command files share one
 * owner of the console / SPSC RX ring and one copy of the module's host-tracked lwIP
 * lifecycle state.  No clock/RCC/register access (XIP-safe).
 */
#include "rtl_link.h"

#include "cli.h"
#include "rtl8720.h"

/* Whether the module's lwIP stack has been brought up since its last power-on.  See
 * rtl_link.h; reset on every power off / reset / fresh power-on, set after tcpip init. */
static bool g_tcpip_inited;

/* Host-side memo of DHCP-vs-static for `net info` (see rtl_link.h). */
static enum rtl_ipmode g_ip_mode;

int rtl_abort_cb(void *ctx)
{
	return cli_cancel_requested((struct cli_instance *)ctx) ? 1 : 0;
}

bool rtl_tcpip_inited(void)          { return g_tcpip_inited; }
void rtl_tcpip_set_inited(bool v)    { g_tcpip_inited = v; }

enum rtl_ipmode rtl_ip_mode(void)         { return g_ip_mode; }
void            rtl_set_ip_mode(enum rtl_ipmode m) { g_ip_mode = m; }

int rtl_link_begin(struct cli_instance *sh, bool power_on)
{
	if (cli_console_claim(sh) != 0) {              /* bg-reject / single owner */
		cli_error(sh, "wifi: run in the foreground (not `... &`)\r\n");
		return RTL_LINK_ERR;
	}
	if (!rtl8720_powered()) {
		if (!power_on) {
			cli_console_release(sh);
			return RTL_LINK_OFF;
		}
		cli_print(sh, "wifi: powering on RTL8720DN, waiting ~1.5s for boot...\r\n");
		rtl8720_power(true);
		g_tcpip_inited = false;                /* fresh boot: lwIP not yet up */
		g_ip_mode      = RTL_IP_UNKNOWN;       /* and no address obtained yet */
		if (cli_sleep(sh, 1500u)) {            /* cancellable boot wait */
			cli_console_release(sh);
			return RTL_LINK_ERR;
		}
	}
	if (rtl8720_uart_open(RTL8720_UART_AT, 2000000u) != 0) {
		cli_console_release(sh);
		cli_error(sh, "wifi: USART1 @2000000 did not come ready\r\n");
		return RTL_LINK_ERR;
	}
	return RTL_LINK_READY;
}

void rtl_link_end(struct cli_instance *sh)
{
	rtl8720_uart_close();
	cli_console_release(sh);
}
