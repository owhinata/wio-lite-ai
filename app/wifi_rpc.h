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
int wifi_rpc_dhcpc_stop(const struct wifi_rpc_opts *o, uint32_t itf, int32_t *result);
int wifi_rpc_get_ip(const struct wifi_rpc_opts *o, uint32_t itf,
                    struct wifi_ip_info *ip, int32_t *result);
/* Set a static address (rpc_tcpip_adapter_set_ip_info); @ip fields are network byte
 * order (as get_ip returns them).  Stop DHCP first so it will not overwrite it. */
int wifi_rpc_set_ip_info(const struct wifi_rpc_opts *o, uint32_t itf,
                         const struct wifi_ip_info *ip, int32_t *result);

/*
 * ---- raw BSD-socket offload (rpc_wifi_lwip, service 16), for `net ping` ----
 *
 * DIFFERENT return convention from the calls above: these mirror lwIP's own syscall
 * return, NOT the module int32 `result`.  The @fd / @ret out-parameter holds the
 * syscall's value directly -- fd >= 0 or a byte count on success, < 0 on error (call
 * wifi_rpc_lwip_errno() for the reason).  The function's int return is still the
 * transport code: 0 (round-trip ok, @fd/@ret valid), a negative erpc_call_ex code
 * (-1/-2/-4) or WIFI_RPC_EDECODE (malformed reply).
 */
int wifi_rpc_lwip_socket(const struct wifi_rpc_opts *o, int32_t domain, int32_t type,
                         int32_t protocol, int32_t *fd);
/* Set a socket option (rpc_lwip_setsockopt).  @level / @optname are passed straight to
 * the module's lwip_setsockopt(), so use lwIP's numeric constants (e.g. SOL_SOCKET
 * 0xfff, SO_RCVTIMEO 0x1006 with a 4-byte int millisecond @optval).  NOTE: the factory
 * rpc_lwip_recv/recvfrom IGNORE their timeout argument and block, so a caller that
 * needs a bounded receive MUST set SO_RCVTIMEO here first (else a no-reply recv wedges
 * the module's single-threaded eRPC server until `wifi reset`). */
int wifi_rpc_lwip_setsockopt(const struct wifi_rpc_opts *o, int32_t s, int32_t level,
                             int32_t optname, const uint8_t *optval, uint16_t optlen,
                             int32_t *ret);
int wifi_rpc_lwip_sendto(const struct wifi_rpc_opts *o, int32_t s,
                         const uint8_t *data, uint16_t dlen, int32_t flags,
                         const uint8_t *sa, uint16_t salen, int32_t *ret);
/* Blocking receive up to @timeout_ms on the module side.  On a round-trip the received
 * datagram (mem, incl. the IPv4 header for a raw socket) is copied into @buf (fails
 * WIFI_RPC_EDECODE rather than truncating if it exceeds @buf_cap), its length in @got,
 * and the raw recvfrom() return in @ret.  The source address is decoded and discarded. */
int wifi_rpc_lwip_recvfrom(const struct wifi_rpc_opts *o, int32_t s,
                           uint8_t *buf, uint16_t buf_cap, int32_t flags,
                           uint32_t timeout_ms, uint16_t *got, int32_t *ret);
int wifi_rpc_lwip_close(const struct wifi_rpc_opts *o, int32_t s, int32_t *ret);
int wifi_rpc_lwip_errno(const struct wifi_rpc_opts *o, int32_t *err);

#endif /* APP_WIFI_RPC_H */
