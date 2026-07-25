/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/*
 * Wio Lite AI (STM32H725AEI6) -- typed eRPC wrappers for the onboard RTL8720DN
 * WiFi/tcpip API (issue #5, increment 3: WiFi association + DHCP).
 *
 * Thin, synchronous C wrappers over app/erpc.c's erpc_call_ex().  Each function
 * performs ONE eRPC round-trip on the currently-open RTL8720 UART; a caller holds that
 * UART for its whole flow by referencing it once (rtl_link_begin() for a shell command,
 * or rtl_link_uart_ref(RTL8720_UART_AT, 2000000) directly -- see app/rtl_link.h).  The
 * frames themselves are sent and routed by the eRPC service thread, so these are safe
 * to call from any thread.  No clock/RCC work here -- XIP-safe.
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
/*
 * Asynchronous recvfrom for the `net conc` diagnostic (issue #20 N3): encode a recvfrom
 * request and send it with erpc_begin(), returning an eRPC token (>= 0) to collect later
 * with erpc_wait()/erpc_cancel(), or a negative erpc_begin() code (-1).  It lets the
 * caller keep a (bounded) blocking receive outstanding on the module while it issues
 * another request, to show the N3 server serves the second one concurrently instead of
 * queueing behind the receive.  @out receives the raw, UNDECODED reply payload (the
 * diagnostic only cares that the round-trips overlap) and must stay valid until the
 * token is waited/cancelled.  @len is bounded so the module's reply fits one frame.
 */
int wifi_rpc_lwip_recvfrom_begin(int32_t s, uint32_t len, int32_t flags,
                                 uint32_t timeout_ms, uint8_t *out, uint16_t out_cap);
int wifi_rpc_lwip_close(const struct wifi_rpc_opts *o, int32_t s, int32_t *ret);
int wifi_rpc_lwip_errno(const struct wifi_rpc_opts *o, int32_t *err);

/*
 * ---- TCP stream sockets (rpc_wifi_lwip, service 16), for `net echo` -- issue #21 ----
 *
 * Same return convention as the raw-socket calls above: the function's int return is the
 * transport code (0 = round-trip ok, -1 bad args, -2 timeout, -4 aborted, or
 * WIFI_RPC_EDECODE for a malformed reply), and the lwIP syscall's own value lands in
 * @ret / @fd (>= 0 on success, < 0 with the reason in wifi_rpc_lwip_errno()).
 *
 * These are what a TCP server needs on top of the existing socket/setsockopt/close:
 * bind -> listen -> accept -> recv/send.  Three firmware quirks are baked into the
 * wrappers (all verified against seeed-ambd-firmware's generated shim + wifi_api.c):
 *
 *   1. accept's reply carries TWO words (u32 addrlen + i32 result), unlike every other
 *      call here, and the firmware never fills the peer address in: it passes
 *      &addr->dataLength (not the addrlen argument) to lwip_accept and then drops the
 *      buffer.  So accept yields the fd only -- use wifi_rpc_lwip_getpeername() if the
 *      caller wants to know who connected.
 *   2. recv reports failure ONLY through @ret: the firmware hands back a one-byte dummy
 *      blob whenever lwip_recv() returns <= 0, so the payload length must never be used
 *      to decide whether data arrived.
 *   3. recv's @timeout_ms drives the module's SO_RCVTIMEO (issue #20 N2), and a zero
 *      there means "leave the socket as it is" == a BLOCKING receive.  A blocking recv
 *      with no data pins that fd forever, and the N3 firmware's per-fd lifecycle rule
 *      then forbids closing it (a close racing an in-flight recv on the same fd is
 *      unsafe) -- the socket is effectively lost until `wifi reset`.  So the wrapper
 *      REJECTS timeout_ms == 0 outright rather than let a caller ask for that.
 *
 * NOTE on cancellation: do NOT pass a should_abort hook for accept / recv.  Aborting only
 * ends the HOST's wait -- the module keeps running the call, so an aborted accept can
 * complete into a socket whose fd the host never learns (unclosable, and it consumes one
 * of the module's few netconns), and an aborted recv leaves the fd busy so it must not be
 * closed either.  Poll the cancel flag between calls instead (see shell/cmds/cmd_net.c).
 */

/* Maximum payload carried by one recv/send round-trip.  Bounds the wrappers' on-stack
 * request/reply scratch (<= 268 B, comfortable on the 4 KB shell thread stack).  Raising
 * this means moving that scratch out of the locals into static storage. */
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
 * erpc_begin() frames could still exceed the ring together.)  The wrappers still ACCEPT
 * up to WIFI_RPC_STREAM_MAX so the sweep above stays reproducible, but anything that just
 * wants to move bytes reliably -- the issue #21 telnet console's output path included --
 * must chunk at WIFI_RPC_SEND_SAFE.  Receiving may keep using the full
 * WIFI_RPC_STREAM_MAX.
 *
 * The proper fix is on the firmware side (a bulk `readBytes()` in the transport, or a
 * larger SERIAL_BUFFER_SIZE), i.e. another patch in fw/rtl8720/patches -- deliberately
 * out of scope here, where the point is to establish the constraint.
 */
#define WIFI_RPC_SEND_SAFE        96u

/* @sa / @salen are a raw lwIP sockaddr (16-byte sockaddr_in here), as built by the
 * caller in network byte order. */
int wifi_rpc_lwip_bind(const struct wifi_rpc_opts *o, int32_t s,
                       const uint8_t *sa, uint16_t salen, int32_t *ret);
int wifi_rpc_lwip_listen(const struct wifi_rpc_opts *o, int32_t s, int32_t backlog,
                         int32_t *ret);
/* Accept one connection; *@fd is the accepted socket (>= 0) or the lwIP error (< 0).
 * The call BLOCKS on the module until a client arrives or its internal cap expires
 * (10 s, issue #20 N2 -- it then returns -1/ETIMEDOUT), so give @o a timeout that
 * outlasts that cap or the host abandons a call the module is still running. */
int wifi_rpc_lwip_accept(const struct wifi_rpc_opts *o, int32_t s, int32_t *fd);
/* Receive up to @buf_cap (<= WIFI_RPC_STREAM_MAX) bytes.  @timeout_ms must be non-zero
 * (see quirk 3) and @o's timeout should exceed it.  On a completed round-trip *@ret is
 * lwip_recv()'s return: > 0 = bytes (copied into @buf, *@got = count), 0 = peer closed,
 * < 0 = error (EAGAIN when the receive timed out with no data). */
int wifi_rpc_lwip_recv(const struct wifi_rpc_opts *o, int32_t s,
                       uint8_t *buf, uint16_t buf_cap, int32_t flags,
                       uint32_t timeout_ms, uint16_t *got, int32_t *ret);
/* Send 1..WIFI_RPC_STREAM_MAX bytes; *@ret is lwip_send()'s return, which may be SHORT --
 * the caller must loop until everything is out.  Pass no more than WIFI_RPC_SEND_SAFE
 * unless you are deliberately probing the request-size limit (see the ASYMMETRY note). */
int wifi_rpc_lwip_send(const struct wifi_rpc_opts *o, int32_t s,
                       const uint8_t *data, uint16_t dlen, int32_t flags, int32_t *ret);
int wifi_rpc_lwip_shutdown(const struct wifi_rpc_opts *o, int32_t s, int32_t how,
                           int32_t *ret);
/*
 * Peer address of a connected socket, into the 16-byte @sa (*@got = bytes the module
 * actually returned, zero-padded to 16).
 *
 * BEST EFFORT ONLY -- the firmware's shim passes an UNINITIALISED socklen to
 * lwip_getpeername (it erpc_malloc()s the binary_t and never sets dataLength before the
 * call), so lwIP may copy fewer than 16 bytes and leave the rest of the address unset.
 * It cannot overrun anything (the destination is a fixed 16-byte sockaddr), but the
 * VALUE may be partial: validate it (sin_len == 16 && sin_family == AF_INET) before
 * believing it, and treat a failed check as "unknown peer", never as an error.
 */
int wifi_rpc_lwip_getpeername(const struct wifi_rpc_opts *o, int32_t s,
                              uint8_t sa[16], uint16_t *got, int32_t *ret);

#endif /* APP_WIFI_RPC_H */
