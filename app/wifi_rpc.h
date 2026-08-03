/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/*
 * Wio Lite AI (STM32H725AEI6) -- typed eRPC wrappers for the onboard RTL8720DN
 * WiFi/tcpip API (issue #5, increment 3: WiFi association).
 *
 * Thin, synchronous C wrappers over app/erpc.c's erpc_call_ex().  Each function
 * performs ONE eRPC round-trip on the currently-open RTL8720 UART; a caller holds that
 * UART for its whole flow by referencing it once (rtl_link_begin() for a shell command,
 * or rtl_link_uart_ref(RTL8720_UART_AT, rtl_link_erpc_baud()) directly -- never a
 * hard-coded rate, the link is re-based at runtime; see app/rtl_link.h).  The
 * frames themselves are sent and routed by the eRPC service thread, so these are safe
 * to call from any thread.  No clock/RCC work here -- clock-safe.
 * Layering: HAL/CMSIS <- erpc.c <- wifi_rpc.c <- cmd_wifi.c.
 *
 * Service / method IDs and wire layout come from the factory firmware's generated
 * shims (seeed-ambd-firmware @Wio-Lite-AI, src/erpc_shim/rpc_wifi_api_server.cpp):
 *   rpc_wifi_drv   = service 14 (connect=1, disconnect=3, is_connected_to_ap=4,
 *                    get_mac_address=8, get_rssi=19, wifi_on=27, scan_start=64,
 *                    is_scaning=65, scan_get_ap_records=66, scan_get_ap_num=67)
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
#include <stdbool.h>
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

/*
 * rtw_connect_error_flag_t (Realtek Ameba wifi_constants.h): what the module's
 * wifi_get_last_error() reports about the LAST association attempt (issue #40).
 *
 * This is the only way to tell apart the several reasons an association fails: the SDK's
 * wifi_connect() collapses all of them into one RTW_ERROR (-1) return.  It reaches that
 * -1 through exactly one path -- the join semaphore WAS posted (so the driver ran the
 * attempt and reported back) and wifi_is_connected_to_ap() then said the module is not
 * associated -- so a -1 always means "the attempt was made and it failed", never "the call
 * was rejected".  The flag is set by the driver's own event handlers along the way, and
 * every wifi_connect() clears it on entry, so it has to be read after a failure and
 * before anything else tries to associate.
 */
#define WIFI_RPC_ERR_NONE            0   /* RTW_NO_ERROR */
#define WIFI_RPC_ERR_NO_NETWORK      1   /* RTW_NONE_NETWORK: target AP not found */
#define WIFI_RPC_ERR_CONNECT_FAIL    2   /* RTW_CONNECT_FAIL: association refused */
#define WIFI_RPC_ERR_WRONG_PASSWORD  3   /* RTW_WRONG_PASSWORD */
#define WIFI_RPC_ERR_HANDSHAKE       4   /* RTW_4WAY_HANDSHAKE_TIMEOUT */
#define WIFI_RPC_ERR_DHCP_FAIL       5   /* RTW_DHCP_FAIL */
#define WIFI_RPC_ERR_UNKNOWN         6   /* RTW_UNKNOWN: nothing reported a reason */

/* wifi_rpc-level failure: the round-trip returned but the reply was too short /
 * malformed to decode (distinct from erpc_call_ex's -1/-2/-4 transport codes). */
#define WIFI_RPC_EDECODE         (-10)

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
/* The module's channel plan index: which channels it scans, and how (issue #40). */
int wifi_rpc_get_channel_plan(const struct wifi_rpc_opts *o, uint8_t *plan,
                              int32_t *result);
/* Install one.  ⚠️ NON-VOLATILE -- survives a full power-down, so this is provisioning
 * and must not run on every boot.  Read it back afterwards; see the definition. */
int wifi_rpc_set_channel_plan(const struct wifi_rpc_opts *o, uint8_t plan,
                              int32_t *result);
int wifi_rpc_is_connected(const struct wifi_rpc_opts *o, int32_t *result);
/* Why the last association attempt failed: *@err is one of WIFI_RPC_ERR_* above.  Unlike
 * the other wrappers there is no separate module `result` -- the flag IS the return
 * value, so a decode failure is the only way this reports nothing (issue #40). */
int wifi_rpc_get_last_error(const struct wifi_rpc_opts *o, int32_t *err);
/* Name for a WIFI_RPC_ERR_* value; never NULL (unrecognised values read as "unknown"). */
const char *wifi_rpc_err_name(int32_t err);
int wifi_rpc_get_rssi(const struct wifi_rpc_opts *o, int32_t *rssi, int32_t *result);
int wifi_rpc_get_mac(const struct wifi_rpc_opts *o, char mac[18], int32_t *result);
int wifi_rpc_tcpip_init(const struct wifi_rpc_opts *o, int32_t *result);
/*
 * get_ip_info / set_ip_info / dhcpc_start were here until issue #30 B1.  They drove the
 * MODULE's L3 -- an address the host stack throws away the moment the bridge goes in
 * (arming the bridge stops the DHCP client and the module zeroes its netif, because the
 * WLAN driver filters received IP against it).  L3 is the host's alone now; what remains of
 * service 15 is the two calls the BRIDGE itself needs.
 */
int wifi_rpc_dhcpc_stop(const struct wifi_rpc_opts *o, uint32_t itf, int32_t *result);

/*
 * ---- AP scan (rpc_wifi_drv 64..67), for `wifi scan` --------------------------------
 *
 * The module's scan is ASYNCHRONOUS: scan_start() only kicks it off (the firmware's
 * wifi_scan_start() calls wifi_scan_networks(handler, NULL) and returns), results
 * accumulate in a firmware-side array, and the scan-done callback clears the "scanning"
 * flag.  So the sequence is: scan_start -> poll is_scanning until false -> get_ap_num ->
 * get_ap_records(num).  Completion also pushes a oneway rpc_wifi_event_callback
 * (SYSTEM_EVENT_SCAN_DONE) at us, which erpc.c drains as an unsupported invocation.
 *
 * CAUTION: the firmware's wifi_scan_get_ap_records() memcpy()s `number` records out of
 * its static array bounded only by WL_NETWORKS_LIST_MAXNUM (60) -- NOT by how many were
 * actually found.  Ask get_ap_num() first and request exactly that many, or the tail of
 * the reply is the previous scan's leftovers / zeros.
 */

/* One packed rtw_scan_result_t on the wire (Realtek wifi_structures.h, inside a
 * #pragma pack(1) region): SSID.len(1) + SSID.val[33] + BSSID[6] + signal_strength(i16)
 * + bss_type(u32) + security(u32) + wps_type(u32) + channel(u32) + band(u32). */
#define WIFI_RPC_SCAN_REC_SIZE     62u

/* Reply-payload bytes wifi_rpc_scan_get_ap_records() needs for @n records:
 * u32 binary length + n*62 record bytes + i32 result. */
#define WIFI_RPC_SCAN_BUF_SIZE(n)  (8u + (n) * WIFI_RPC_SCAN_REC_SIZE)

/* The firmware's own cap on its record array (WL_NETWORKS_LIST_MAXNUM, wifi_main.c);
 * asking for more makes its shim fail the call outright. */
#define WIFI_RPC_SCAN_MAX_RECORDS  60u

/*
 * rtw_802_11_band_t values (Realtek wifi_constants.h) -- note 0 is 5 GHz, not 2.4.
 *
 * DO NOT TRUST the `band` field of a scan record: measured on board #2 it is 0 for every
 * result, 2.4 GHz APs included (channels 5 and 11 reported band 0), so on this firmware
 * 0 evidently means "unset" rather than "5 GHz".  That is an observation, not a claim
 * about the source -- the field is populated (or not) inside Realtek's prebuilt libameba
 * scan path, which we do not have sources for.  Classify from `channel` instead --
 * cmd_wifi.c's scan_band_name() does.  The field is kept in struct wifi_ap_record
 * because it is what the wire carries, not because it is useful.
 */
#define WIFI_RPC_BAND_5GHZ         0u
#define WIFI_RPC_BAND_2_4GHZ       1u

/* One decoded scan result. */
struct wifi_ap_record {
	char     ssid[33];    /* NUL-terminated RAW bytes -- see the note below */
	uint8_t  ssid_len;    /* 0..32 */
	uint8_t  bssid[6];
	int16_t  rssi;        /* dBm (signed) */
	uint32_t security;    /* rtw_security_t bitmask */
	uint32_t channel;
	uint32_t band;        /* WIFI_RPC_BAND_* */
	uint32_t bss_type;
	uint32_t wps_type;
};

/*
 * NOTE on @ssid: those are RAW bytes broadcast over the air, i.e. attacker-controlled,
 * and may contain terminal escape / control sequences.  This layer only marshals, so
 * whatever PRINTS an ssid must sanitise it first (cmd_wifi.c restricts it to printable
 * ASCII and dumps the raw bytes as hex whenever it had to substitute).
 */

/* Kick off a scan.  Returns 0 when the round-trip completed, with the module's RTW
 * result in *@result (WIFI_RPC_OK = started, negative = e.g. a scan already running). */
int wifi_rpc_scan_start(const struct wifi_rpc_opts *o, int32_t *result);

/* Whether a scan is still running (*@scanning = 0/1).  No module `result` here -- the
 * firmware method returns the flag itself. */
int wifi_rpc_is_scanning(const struct wifi_rpc_opts *o, int *scanning);

/* How many APs the last completed scan found (*@num; the firmware caps it at 60). */
int wifi_rpc_scan_get_ap_num(const struct wifi_rpc_opts *o, uint16_t *num);

/*
 * Fetch the first @number records into @buf, which must be at least
 * WIFI_RPC_SCAN_BUF_SIZE(@number) bytes and is left holding the RAW reply payload for
 * wifi_rpc_scan_record() to decode from.  @number must be 1..WIFI_RPC_SCAN_MAX_RECORDS
 * and should equal wifi_rpc_scan_get_ap_num()'s answer (see the CAUTION above).
 * Returns 0 with *@got = @number and the module's result in *@result, or
 * WIFI_RPC_EDECODE if the reply is not exactly @number records long (which is also how
 * the module reports failure -- its shim then returns a 1-byte dummy blob).
 */
int wifi_rpc_scan_get_ap_records(const struct wifi_rpc_opts *o, uint16_t number,
                                 uint8_t *buf, uint16_t buf_cap,
                                 uint16_t *got, int32_t *result);

/* Decode record @idx (0-based, < @got) out of a buffer filled by
 * wifi_rpc_scan_get_ap_records().  Pure function; returns 0, or -1 if @idx >= @got. */
int wifi_rpc_scan_record(const uint8_t *buf, uint16_t got, uint16_t idx,
                         struct wifi_ap_record *rec);

/*
 * ---- what used to be here (rpc_wifi_lwip, service 16) ----------------------------
 *
 * The socket-offload wrappers lived here in two waves, and both are gone.  bind /
 * listen / accept / recv / send / shutdown / getpeername served issue #21's
 * TCP-on-the-module era; issue #23 U4 moved `net echo` and the telnet console onto the
 * host's own NetX Duo stack and they were deleted with it (history: 41d1ff0~1).  The
 * datagram side -- socket / setsockopt / sendto / recvfrom (+ its async _begin) /
 * close / errno, which carried `net ping`'s raw ICMP and the `net conc` diagnostic --
 * followed in issue #28 when ping became host-stack-only and conc was retired.  The
 * firmware quirks they encoded (accept never filling in the peer address, the recvfrom
 * timeout honoured only on N2+ firmware, the SO_RCVTIMEO handling from issue #20 N2)
 * are recorded with the code in this file's history, and the service-16 method IDs
 * remain listed in wifi_rpc.c as the record of the module's wire contract -- a future
 * module-side socket need would start from them again (issue #5's BSD sockets are the
 * host stack's job and do not).
 */

/* Maximum payload of one streamed eRPC request the link contract allows; today only
 * wifi_rpc_send_chunk() below derives from it. */
#define WIFI_RPC_STREAM_MAX      256u

/*
 * ASYMMETRY -- a big REQUEST is unsafe, a big REPLY is fine.  Measured on board #2.
 *
 * The module's eRPC transport reads the UART ONE BYTE AT A TIME and sleeps a whole
 * FreeRTOS tick whenever it finds the buffer empty
 * (seeed-ambd-firmware src/erpc/erpc_arduino_uart_transport.cpp:
 * `while (!available()) vTaskDelay(1);` then a single `read()`), and behind that sits an
 * Arduino RingBuffer of SERIAL_BUFFER_SIZE = 128 bytes -- 127 usable, and
 * RingBufferN::store_char SILENTLY DROPS a byte when it is full
 * (ArduinoCore-ambd cores/arduino/RingBuffer.h).  configTICK_RATE_HZ is 1000, so at
 * 2 Mbaud one tick is 200 bytes.  Our side writes a frame as a gap-free polled burst
 * (app/rtl8720.c rtl8720_uart_write), and the module stalls mid-frame to allocate its
 * 4 KB message buffer between reading the 4-byte header and the body -- so any frame
 * bigger than that ring loses its tail, the CRC fails, no reply is ever sent and the
 * caller times out (which then leaves the link "dirty": see cmd_net.c).
 *
 * Measured directly: a 264-byte REPLY (recv of 256 B) arrives intact -- our RX is an
 * interrupt-driven ring -- while a 280-byte REQUEST (send of 256 B) never gets answered.
 *
 * A send request frame is 24 + payload bytes (4 framing + 8 header/seq + 4 fd + 4 binary
 * length + 4 flags).  Swept on board #2 with `net echo 2323 <txchunk>`, echoing 1000 B:
 *
 *     payload  64  96 128 160        256
 *     frame    88 120 152 184        280
 *     result   ok  ok  ok  ok        NO REPLY
 *
 * So the cliff is somewhere between 184 and 280 bytes -- NOT at 127, because the module's
 * reader is draining while we write, so what really has to stay under the ring is the
 * backlog that piles up during its longest mid-frame stall.  That makes anything above
 * 127 a LOAD-DEPENDENT race: the same 160 that passes on an idle module can drop bytes
 * once its WiFi/lwIP threads delay the receive task further.  A frame that fits ENTIRELY
 * in the ring cannot be lost no matter how long the reader stalls, and that is the only
 * structural guarantee available here -- it bounds the payload at 127 - 24 = 103.
 *
 * Hence WIFI_RPC_SEND_SAFE is 96: inside the ring with a little headroom, and measured at
 * 6 ms per 256 B echoed versus 8 ms for 64 B chunks.  (The guarantee assumes one request
 * frame in flight at a time, which is how the shell drives the link; two back-to-back
 * erpc_begin() frames could still exceed the ring together.)  Anything that wants to
 * move bytes reliably over eRPC against an unproven module must chunk at
 * WIFI_RPC_SEND_SAFE; receiving may use the full WIFI_RPC_STREAM_MAX.
 *
 * THE FIRMWARE FIX IS NOW IN (issue #23 U0-2, fw/rtl8720/patches/0004): the module owns
 * USI0 directly behind an 8 kB ring and reads it in bulk, so none of the three causes
 * above survives and a full WIFI_RPC_STREAM_MAX payload is safe.  But the host must keep
 * working against an older module -- the two are flashed separately and the recovery path
 * (`wifi flash imgload` / `flash write`) exists precisely to put an older image back -- so the
 * chunk size is a RUNTIME question, not a compile-time one: see wifi_rpc_send_chunk().
 * WIFI_RPC_SEND_SAFE remains the answer for a link we have no proof about.
 */
#define WIFI_RPC_SEND_SAFE        96u

/*
 * Largest request payload that is safe to send RIGHT NOW: WIFI_RPC_STREAM_MAX on a link
 * proved to be wio-n4 or later (`wifi ver`), WIFI_RPC_SEND_SAFE otherwise.  Derived from
 * erpc_wire_budget(), so it follows the same generation-scoped latch and drops back to
 * the conservative answer the moment the link is reopened, reset or reflashed -- callers
 * must therefore read it per burst rather than caching it.  (No caller streams today;
 * `wifi ver` reports it as part of the link's capability summary.)
 */
uint16_t wifi_rpc_send_chunk(void);


#endif /* APP_WIFI_RPC_H */
