/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    rtl_link.h
 * @brief   Shared RTL8720DN eRPC session helper for the `wifi` (L2) and `net` (L3)
 *          shell commands (issue #5).
 *
 * The onboard RTL8720DN speaks eRPC over USART1 @2 Mbaud; that link is a strict
 * single-owner resource -- one eRPC transaction in flight at a time over a SPSC RX
 * ring (see app/erpc.h).  Both `wifi` (association / power) and `net` (IP config /
 * ping) drive it, so the console-claim + UART-open/close session and the module's
 * host-tracked lifecycle state live here, shared by both command files.
 *
 * No clock/RCC/register work -- it only claims the console, powers the module via
 * app/rtl8720.c and opens the eRPC UART (XIP-safe).  Clean-room design.
 */
#ifndef SHELL_RTL_LINK_H
#define SHELL_RTL_LINK_H

#include <stdbool.h>

struct cli_instance;

/* rtl_link_begin() result. */
enum { RTL_LINK_READY = 0, RTL_LINK_ERR = 1, RTL_LINK_OFF = 2 };

/*
 * Claim the console (single owner of the SPSC RX ring / one eRPC call in flight),
 * ensure the module is powered and open the eRPC UART (USART1 @2 Mbaud).
 *   @power_on true : power the module and wait for boot if it is off (for `connect`).
 *   @power_on false: if the module is off, release and return RTL_LINK_OFF so the
 *                    caller can report "powered off" without powering it (status /
 *                    net queries are pure reads).
 * Returns RTL_LINK_READY with the console claimed + UART open (caller must
 * rtl_link_end()), RTL_LINK_OFF (nothing claimed), or RTL_LINK_ERR (nothing claimed,
 * message already printed).  A fresh power-on here resets the host-tracked lwIP /
 * IP-mode state (see below).
 */
int  rtl_link_begin(struct cli_instance *sh, bool power_on);

/* Tear down an rtl_link_begin() session (close the UART, release the console). */
void rtl_link_end(struct cli_instance *sh);

/* Abort thunk for wifi_rpc_opts.should_abort: non-zero once Ctrl+C was pressed. */
int  rtl_abort_cb(void *ctx);

/*
 * Whether the module's lwIP stack (tcpip_adapter_init = LwIP_Init) has been brought
 * up since its last power-on.  The factory firmware does NOT init lwIP at boot, so
 * the host must -- once per CHIP_EN power cycle (lwIP state survives wifi off/on but
 * not a power cycle).  Reset on every power off / reset / fresh power-on; set after a
 * successful tcpip init.
 */
bool rtl_tcpip_inited(void);
void rtl_tcpip_set_inited(bool v);

/*
 * Host-side memo of how the current IPv4 address was obtained, for `net info`
 * (mirrors f746 nx_net_info.dhcp_mode).  It is only what the host last did -- it is
 * set to DHCP / STATIC on a *successful* `wifi connect`/`net dhcp` / `net ip`, and
 * back to UNKNOWN whenever the module is powered off / reset / freshly powered on or
 * disconnected.  It is not a query of the module.
 */
enum rtl_ipmode { RTL_IP_UNKNOWN = 0, RTL_IP_DHCP, RTL_IP_STATIC };
enum rtl_ipmode rtl_ip_mode(void);
void            rtl_set_ip_mode(enum rtl_ipmode m);

#endif /* SHELL_RTL_LINK_H */
