/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/*
 * Wio Lite AI (STM32H725AEI6) -- typed eRPC wrappers for the onboard RTL8720DN
 * WiFi/tcpip API (issue #5, increment 3: WiFi association + DHCP).
 *
 * Thin, synchronous C wrappers over app/erpc.c's erpc_call_ex().  Each function
 * performs ONE eRPC round-trip on the currently-open RTL8720 UART (open it once at
 * 2 Mbaud with rtl8720_uart_open(RTL8720_UART_AT, 2000000) before calling), so the
 * caller drives the whole connect flow inside a single UART session.  No clock/RCC
 * work here -- XIP-safe.  Layering: HAL/CMSIS <- erpc.c <- wifi_rpc.c <- cmd_wifi.c.
 *
 * Service / method IDs and wire layout come from the factory firmware's generated
 * shims (seeed-ambd-firmware @Wio-Lite-AI, src/erpc_shim/rpc_wifi_api_server.cpp):
 *   rpc_wifi_drv   = service 14 (connect=1, disconnect=3, is_connected_to_ap=4,
 *                    get_mac_address=8, get_rssi=19, wifi_on=27)
 *   rpc_wifi_tcpip = service 15 (get_ip_info=7, dhcpc_start=13)
 * BasicCodec: scalars raw LE at natural width; string/binary = u32 len + bytes;
 * @nullable arg preceded by a 1-byte null flag (0 = present, 1 = null); reply body
 * = out params in declared order then the return value last.
 *
 * The device runs its WiFi/TCP-IP stack internally; rpc_wifi_connect blocks on the
 * module until associated (it ignores the `semaphore` arg and calls wifi_connect(...,
 * NULL)), and dhcpc_start blocks until the lease is assigned -- hence the long
 * timeouts and the abort hook forwarded to erpc_call_ex().
 */
#ifndef APP_WIFI_RPC_H
#define APP_WIFI_RPC_H

#include <stdint.h>
#include "erpc.h"        /* struct erpc_diag, erpc_call_ex return codes */

/* RTW mode (Realtek Ameba wifi_constants.h).  Boot leaves the module in MODE_NONE
 * (wifi_main.c: wifi_on(RTW_MODE_NONE)), so STA must be set before connecting. */
#define WIFI_RPC_MODE_NONE       0u
#define WIFI_RPC_MODE_STA        1u
#define WIFI_RPC_MODE_AP         2u
#define WIFI_RPC_MODE_STA_AP     3u

/* RTW security_type (Realtek Ameba wifi_constants.h).  A wrong value only makes the
 * module's connect return an error -- no host-side hazard. */
#define WIFI_RPC_SEC_OPEN            0x00000000u
#define WIFI_RPC_SEC_WPA_TKIP_PSK    0x00200002u
#define WIFI_RPC_SEC_WPA_AES_PSK     0x00200004u
#define WIFI_RPC_SEC_WPA2_TKIP_PSK   0x00400002u
#define WIFI_RPC_SEC_WPA2_AES_PSK    0x00400004u
#define WIFI_RPC_SEC_WPA2_MIXED_PSK  0x00400006u
#define WIFI_RPC_SEC_WPA_WPA2_MIXED  0x00600006u

/* Module's own success code for the int32 `result` out-parameter (RTW_SUCCESS). */
#define WIFI_RPC_OK              0

/* wifi_rpc-level failure: the round-trip returned but the reply was too short /
 * malformed to decode (distinct from erpc_call_ex's -1/-2/-4 transport codes). */
#define WIFI_RPC_EDECODE         (-10)

/* Decoded IPv4 config from rpc_tcpip_adapter_get_ip_info.  Each field is stored in
 * network byte order, so the four bytes print directly as a.b.c.d. */
struct wifi_ip_info {
	uint8_t ip[4];
	uint8_t netmask[4];
	uint8_t gw[4];
};

/*
 * Per-call options common to every wrapper.  @should_abort / @abort_ctx are
 * forwarded to erpc_call_ex() (poll a cancel flag; NULL = never abort); @diag
 * receives per-call receive diagnostics (may be NULL).
 */
struct wifi_rpc_opts {
	uint32_t timeout_ms;
	int    (*should_abort)(void *ctx);
	void    *abort_ctx;
	struct erpc_diag *diag;
};

/*
 * Every wrapper returns 0 when the eRPC round-trip completed -- in that case the
 * module's own int32 result is stored in *@result (0 = RTW_SUCCESS, negative =
 * module error).  On a transport failure it returns the negative erpc_call_ex code
 * (-1 bad args / -2 timeout / -4 aborted) or WIFI_RPC_EDECODE for a malformed reply.
 */
int wifi_rpc_on(const struct wifi_rpc_opts *o, uint32_t mode, int32_t *result);
int wifi_rpc_off(const struct wifi_rpc_opts *o, int32_t *result);
int wifi_rpc_connect(const struct wifi_rpc_opts *o, const char *ssid,
                     const char *password /* NULL = open network */,
                     uint32_t security, int32_t *result);
int wifi_rpc_disconnect(const struct wifi_rpc_opts *o, int32_t *result);
int wifi_rpc_is_connected(const struct wifi_rpc_opts *o, int32_t *result);
int wifi_rpc_get_rssi(const struct wifi_rpc_opts *o, int32_t *rssi, int32_t *result);
int wifi_rpc_get_mac(const struct wifi_rpc_opts *o, char mac[18], int32_t *result);
int wifi_rpc_tcpip_init(const struct wifi_rpc_opts *o, int32_t *result);
int wifi_rpc_dhcpc_start(const struct wifi_rpc_opts *o, uint32_t itf, int32_t *result);
int wifi_rpc_get_ip(const struct wifi_rpc_opts *o, uint32_t itf,
                    struct wifi_ip_info *ip, int32_t *result);

#endif /* APP_WIFI_RPC_H */
