/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/*
 * Typed eRPC wrappers for the onboard RTL8720DN WiFi/tcpip API (issue #5, inc 3).
 * See wifi_rpc.h for the service/method IDs, the wire layout and the return
 * convention.  Each function encodes the request body per BasicCodec, calls
 * erpc_call_ex() once, then decodes the reply (out params in order, return value
 * last).  Pure data marshalling -- no register/RCC access, XIP-safe.
 */
#include "wifi_rpc.h"

#include <string.h>

/*
 * The eRPC service thread stages a request in a per-slot buffer (issue #21 increment 8),
 * so the largest request built here has to fit ERPC_REQ_MAX.
 *
 * That used to be wifi_rpc_lwip_send(); issue #23 U4 moved TCP onto the host's own stack
 * and deleted it, leaving wifi_rpc_lwip_recvfrom()'s reply as what WIFI_RPC_STREAM_MAX
 * still sizes.  The bound is kept as-is anyway: it is the ceiling on that constant, and
 * relaxing it to whatever the current largest REQUEST happens to be would mean revisiting
 * it every time a caller comes or goes. */
_Static_assert(12u + WIFI_RPC_STREAM_MAX <= ERPC_REQ_MAX,
               "WIFI_RPC_STREAM_MAX exceeds the eRPC request staging buffer");

uint16_t wifi_rpc_send_chunk(void)
{
	/* One latch decides both questions the old firmware forced on us: how many bytes may
	 * be outstanding (the budget itself) and how big a single frame may be (this).  They
	 * had the same cause -- the module's 127-byte Arduino ring -- so they are answered by
	 * the same generation-scoped proof rather than by two flags that could disagree. */
	return (erpc_wire_budget() > ERPC_WIRE_BUDGET_SAFE) ? WIFI_RPC_STREAM_MAX
	                                                    : WIFI_RPC_SEND_SAFE;
}

/* rpc_wifi_drv (service 14) method IDs. */
#define SVC_WIFI_DRV        14u
#define M_WIFI_CONNECT       1u
#define M_WIFI_DISCONNECT    3u
#define M_WIFI_IS_CONNECTED  4u
#define M_WIFI_GET_MAC       8u
#define M_WIFI_GET_RSSI     19u
#define M_WIFI_ON           27u
#define M_WIFI_OFF          28u
#define M_WIFI_SCAN_START   64u
#define M_WIFI_IS_SCANING   65u
#define M_WIFI_SCAN_RECORDS 66u
#define M_WIFI_SCAN_AP_NUM  67u

/* rpc_wifi_tcpip (service 15) method IDs. */
#define SVC_WIFI_TCPIP      15u
#define M_TCPIP_INIT         1u
#define M_TCPIP_GET_IP_INFO  7u
#define M_TCPIP_SET_IP_INFO  8u
#define M_TCPIP_DHCPC_START 13u
#define M_TCPIP_DHCPC_STOP  14u

/*
 * rpc_wifi_lwip (service 16) method IDs.  Values from the firmware's rpc_wifi_api.h.
 *
 * Only the datagram side is still called: `net ping`'s raw ICMP socket and `net conc`.
 * Issue #23 U4 moved everything that wanted a TCP connection onto the host's own stack,
 * so the wrappers for accept/bind/listen/recv/send/shutdown/getpeername were deleted --
 * dead code that claimed to be a supported path.  The IDs stay because they are the
 * record of the module's wire contract, not a cost, and #5's BSD-socket/TLS work would
 * start from them again.  The deleted wrappers are in this file's history at 41d1ff0~1.
 */
#define SVC_WIFI_LWIP       16u
#define M_LWIP_ACCEPT        1u
#define M_LWIP_BIND          2u
#define M_LWIP_SHUTDOWN      3u
#define M_LWIP_GETPEERNAME   4u
#define M_LWIP_SETSOCKOPT    7u
#define M_LWIP_CLOSE         8u
#define M_LWIP_LISTEN       10u
#define M_LWIP_RECV         12u
#define M_LWIP_RECVFROM     14u
#define M_LWIP_SEND         15u
#define M_LWIP_SENDTO       17u
#define M_LWIP_SOCKET       18u
#define M_LWIP_ERRNO        24u

/* A lwIP sockaddr_in on the wire is 16 bytes (sin_len, sin_family, sin_port,
 * sin_addr, sin_zero[8]) -- the only address size these wrappers deal in. */
#define SOCKADDR_LEN        16u

/* tcpip_adapter_ip_info_t on the wire: ip(4) + netmask(4) + gw(4). */
#define IP_INFO_LEN         12u

/* ---- little-endian codec helpers (BasicCodec is a packed LE byte stream) ---- */
static void put_u32le(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
	p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static uint32_t get_u32le(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
	       ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static uint16_t get_u16le(const uint8_t *p)
{
	return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/* Append a BasicCodec string (u32 length + raw bytes, no NUL) at @p; returns the
 * new cursor.  The caller pre-validates lengths so the request buffer cannot overflow
 * (wifi_rpc_connect bounds ssid<=32 / password<=64 before building the frame). */
static uint8_t *put_string(uint8_t *p, const char *s)
{
	uint32_t n = (uint32_t)strlen(s);
	put_u32le(p, n);
	p += 4;
	memcpy(p, s, n);
	return p + n;
}

/* Append a BasicCodec binary (u32 length + raw bytes) at @p; returns the new cursor.
 * Callers pre-bound @n so the request buffer cannot overflow. */
static uint8_t *put_binary(uint8_t *p, const uint8_t *b, uint32_t n)
{
	put_u32le(p, n);
	p += 4;
	memcpy(p, b, n);
	return p + n;
}

/* Run one round-trip; on transport failure return that (<0), else 0 with the reply
 * length in *@plen.  Centralises the erpc_call_ex option plumbing. */
static int do_call(const struct wifi_rpc_opts *o, uint8_t service, uint8_t request,
                   const uint8_t *req, uint16_t req_len,
                   uint8_t *out, uint16_t out_cap, int *plen)
{
	int n = erpc_call_ex(service, request, req, req_len, out, out_cap,
	                     o->timeout_ms, o->diag, o->should_abort, o->abort_ctx);
	if (n < 0)
		return n;                    /* -1 bad args / -2 timeout / -4 aborted */
	*plen = n;
	return 0;
}

/* Decode a reply that carries only a trailing int32 return value. */
static int decode_result(const uint8_t *rep, int plen, int32_t *result)
{
	if (plen < 4)
		return WIFI_RPC_EDECODE;
	*result = (int32_t)get_u32le(rep);
	return 0;
}

/* ------------------------------------------------------------------ */

int wifi_rpc_on(const struct wifi_rpc_opts *o, uint32_t mode, int32_t *result)
{
	uint8_t req[4], rep[16];
	int plen, rc;

	put_u32le(req, mode);
	rc = do_call(o, SVC_WIFI_DRV, M_WIFI_ON, req, 4u, rep, sizeof(rep), &plen);
	if (rc)
		return rc;
	return decode_result(rep, plen, result);
}

int wifi_rpc_off(const struct wifi_rpc_opts *o, int32_t *result)
{
	uint8_t rep[16];
	int plen, rc;

	rc = do_call(o, SVC_WIFI_DRV, M_WIFI_OFF, NULL, 0u, rep, sizeof(rep), &plen);
	if (rc)
		return rc;
	return decode_result(rep, plen, result);
}

int wifi_rpc_connect(const struct wifi_rpc_opts *o, const char *ssid,
                     const char *password, uint32_t security, int32_t *result)
{
	/* worst case: 4+32(ssid) +1(nullflag) +4+64(pass) +4(sec) +4(key) +4(sem). */
	uint8_t req[160], rep[16];
	uint8_t *p = req;
	int plen, rc;

	if (ssid == NULL || strlen(ssid) > 32u ||
	    (password != NULL && strlen(password) > 64u))
		return WIFI_RPC_EDECODE;         /* caller passed an out-of-range arg */

	p = put_string(p, ssid);
	if (password != NULL) {
		*p++ = 0u;                       /* null flag: kNotNull (value present) */
		p = put_string(p, password);
	} else {
		*p++ = 1u;                       /* null flag: kIsNull */
	}
	put_u32le(p, security);   p += 4;
	put_u32le(p, 0u);         p += 4;    /* key_id = 0 (unused for WPA-PSK) */
	put_u32le(p, 0u);         p += 4;    /* semaphore = 0 (ignored by the module) */

	rc = do_call(o, SVC_WIFI_DRV, M_WIFI_CONNECT, req,
	             (uint16_t)(p - req), rep, sizeof(rep), &plen);
	if (rc)
		return rc;
	return decode_result(rep, plen, result);
}

int wifi_rpc_disconnect(const struct wifi_rpc_opts *o, int32_t *result)
{
	uint8_t rep[16];
	int plen, rc;

	rc = do_call(o, SVC_WIFI_DRV, M_WIFI_DISCONNECT, NULL, 0u, rep, sizeof(rep), &plen);
	if (rc)
		return rc;
	return decode_result(rep, plen, result);
}

int wifi_rpc_is_connected(const struct wifi_rpc_opts *o, int32_t *result)
{
	uint8_t rep[16];
	int plen, rc;

	rc = do_call(o, SVC_WIFI_DRV, M_WIFI_IS_CONNECTED, NULL, 0u, rep, sizeof(rep), &plen);
	if (rc)
		return rc;
	return decode_result(rep, plen, result);
}

int wifi_rpc_get_rssi(const struct wifi_rpc_opts *o, int32_t *rssi, int32_t *result)
{
	uint8_t rep[16];
	int plen, rc;

	rc = do_call(o, SVC_WIFI_DRV, M_WIFI_GET_RSSI, NULL, 0u, rep, sizeof(rep), &plen);
	if (rc)
		return rc;
	if (plen < 8)                        /* pRSSI(i32) + result(i32) */
		return WIFI_RPC_EDECODE;
	*rssi   = (int32_t)get_u32le(rep + 0);
	*result = (int32_t)get_u32le(rep + 4);
	return 0;
}

int wifi_rpc_get_mac(const struct wifi_rpc_opts *o, char mac[18], int32_t *result)
{
	uint8_t rep[32];
	int plen, rc;

	rc = do_call(o, SVC_WIFI_DRV, M_WIFI_GET_MAC, NULL, 0u, rep, sizeof(rep), &plen);
	if (rc)
		return rc;
	if (plen < 18 + 4)                   /* uint8[18] mac + result(i32) */
		return WIFI_RPC_EDECODE;
	memcpy(mac, rep, 18);
	mac[17] = '\0';                      /* firmware already NUL-terminates within 18 */
	*result = (int32_t)get_u32le(rep + 18);
	return 0;
}

int wifi_rpc_tcpip_init(const struct wifi_rpc_opts *o, int32_t *result)
{
	uint8_t rep[16];
	int plen, rc;

	rc = do_call(o, SVC_WIFI_TCPIP, M_TCPIP_INIT, NULL, 0u, rep, sizeof(rep), &plen);
	if (rc)
		return rc;
	return decode_result(rep, plen, result);
}

int wifi_rpc_dhcpc_start(const struct wifi_rpc_opts *o, uint32_t itf, int32_t *result)
{
	uint8_t req[4], rep[16];
	int plen, rc;

	put_u32le(req, itf);
	rc = do_call(o, SVC_WIFI_TCPIP, M_TCPIP_DHCPC_START, req, 4u, rep, sizeof(rep), &plen);
	if (rc)
		return rc;
	return decode_result(rep, plen, result);
}

int wifi_rpc_get_ip(const struct wifi_rpc_opts *o, uint32_t itf,
                    struct wifi_ip_info *ip, int32_t *result)
{
	uint8_t req[4], rep[32];
	uint32_t len;
	int plen, rc;

	put_u32le(req, itf);
	rc = do_call(o, SVC_WIFI_TCPIP, M_TCPIP_GET_IP_INFO, req, 4u, rep, sizeof(rep), &plen);
	if (rc)
		return rc;
	/* reply = binary{u32 len + len bytes} + result(i32).  Bound len BEFORE any
	 * "4 + len + 4" arithmetic so a corrupted huge len cannot wrap the u32 and slip
	 * past the checks (which would OOB-read at rep + 4 + len below). */
	if (plen < 4)
		return WIFI_RPC_EDECODE;
	len = get_u32le(rep);
	if (len < IP_INFO_LEN || len > (uint32_t)sizeof(rep) - 8u)
		return WIFI_RPC_EDECODE;                 /* need ip+mask+gw, and len+8 must fit rep */
	if ((uint32_t)plen < 8u + len)               /* frame shorter than len bytes + result */
		return WIFI_RPC_EDECODE;
	memcpy(ip->ip,      rep + 4,  4);
	memcpy(ip->netmask, rep + 8,  4);
	memcpy(ip->gw,      rep + 12, 4);
	*result = (int32_t)get_u32le(rep + 4 + len);
	return 0;
}

int wifi_rpc_dhcpc_stop(const struct wifi_rpc_opts *o, uint32_t itf, int32_t *result)
{
	uint8_t req[4], rep[16];
	int plen, rc;

	put_u32le(req, itf);
	rc = do_call(o, SVC_WIFI_TCPIP, M_TCPIP_DHCPC_STOP, req, 4u, rep, sizeof(rep), &plen);
	if (rc)
		return rc;
	return decode_result(rep, plen, result);
}

int wifi_rpc_set_ip_info(const struct wifi_rpc_opts *o, uint32_t itf,
                         const struct wifi_ip_info *ip, int32_t *result)
{
	uint8_t req[4 + 4 + IP_INFO_LEN], rep[16];
	uint8_t ipbuf[IP_INFO_LEN];
	uint8_t *p = req;
	int plen, rc;

	memcpy(ipbuf + 0, ip->ip,      4);   /* network byte order, as get_ip returns */
	memcpy(ipbuf + 4, ip->netmask, 4);
	memcpy(ipbuf + 8, ip->gw,      4);
	put_u32le(p, itf); p += 4;
	p = put_binary(p, ipbuf, IP_INFO_LEN);

	rc = do_call(o, SVC_WIFI_TCPIP, M_TCPIP_SET_IP_INFO, req,
	             (uint16_t)(p - req), rep, sizeof(rep), &plen);
	if (rc)
		return rc;
	return decode_result(rep, plen, result);
}

/* ---- AP scan (rpc_wifi_drv 64..67) -- see the header for the async sequence ---- */

int wifi_rpc_scan_start(const struct wifi_rpc_opts *o, int32_t *result)
{
	uint8_t rep[16];
	int plen, rc;

	rc = do_call(o, SVC_WIFI_DRV, M_WIFI_SCAN_START, NULL, 0u, rep, sizeof(rep), &plen);
	if (rc)
		return rc;
	return decode_result(rep, plen, result);
}

int wifi_rpc_is_scanning(const struct wifi_rpc_opts *o, int *scanning)
{
	uint8_t rep[16];
	int plen, rc;

	rc = do_call(o, SVC_WIFI_DRV, M_WIFI_IS_SCANING, NULL, 0u, rep, sizeof(rep), &plen);
	if (rc)
		return rc;
	if (plen < 1)                        /* reply = bool, written as a single byte */
		return WIFI_RPC_EDECODE;
	*scanning = rep[0] ? 1 : 0;
	return 0;
}

int wifi_rpc_scan_get_ap_num(const struct wifi_rpc_opts *o, uint16_t *num)
{
	uint8_t rep[16];
	int plen, rc;

	rc = do_call(o, SVC_WIFI_DRV, M_WIFI_SCAN_AP_NUM, NULL, 0u, rep, sizeof(rep), &plen);
	if (rc)
		return rc;
	if (plen < 2)                        /* reply = uint16 count */
		return WIFI_RPC_EDECODE;
	*num = get_u16le(rep);
	return 0;
}

int wifi_rpc_scan_get_ap_records(const struct wifi_rpc_opts *o, uint16_t number,
                                 uint8_t *buf, uint16_t buf_cap,
                                 uint16_t *got, int32_t *result)
{
	uint8_t req[2];
	uint32_t want, mlen;
	int plen, rc;

	if (number == 0u || number > WIFI_RPC_SCAN_MAX_RECORDS)
		return -1;
	want = (uint32_t)number * WIFI_RPC_SCAN_REC_SIZE;
	if ((uint32_t)buf_cap < WIFI_RPC_SCAN_BUF_SIZE((uint32_t)number))
		return -1;

	req[0] = (uint8_t)number;
	req[1] = (uint8_t)(number >> 8);
	rc = do_call(o, SVC_WIFI_DRV, M_WIFI_SCAN_RECORDS, req, 2u, buf, buf_cap, &plen);
	if (rc)
		return rc;

	/* reply = _scanResult(binary) + result(i32).  erpc truncates the copy to buf_cap
	 * but returns the FULL payload length, so reject an oversized reply before trusting
	 * any offset (same guard as wifi_rpc_lwip_recvfrom).  A failed scan_get_ap_records
	 * on the module returns a 1-byte dummy blob, which the length check below catches. */
	if (plen > (int)buf_cap || plen < 4)
		return WIFI_RPC_EDECODE;
	mlen = get_u32le(buf);
	if (mlen != want)                    /* not exactly `number` whole records */
		return WIFI_RPC_EDECODE;
	if ((uint32_t)plen < 4u + mlen + 4u) /* need the trailing i32 result too */
		return WIFI_RPC_EDECODE;

	*result = (int32_t)get_u32le(buf + 4u + mlen);
	*got    = number;
	return 0;
}

int wifi_rpc_scan_record(const uint8_t *buf, uint16_t got, uint16_t idx,
                         struct wifi_ap_record *rec)
{
	const uint8_t *r;
	uint8_t len;

	if (idx >= got)
		return -1;
	r = buf + 4u + (uint32_t)idx * WIFI_RPC_SCAN_REC_SIZE;

	/* SSID.len then SSID.val[33].  The firmware NUL-terminates val[len], but do not
	 * rely on that: clamp to 32 so ssid[] always gets its own terminator. */
	len = r[0];
	if (len > 32u)
		len = 32u;
	memcpy(rec->ssid, r + 1, len);
	rec->ssid[len] = '\0';
	rec->ssid_len  = len;

	memcpy(rec->bssid, r + 34, 6);
	rec->rssi     = (int16_t)get_u16le(r + 40);   /* signed dBm, LE on both sides */
	rec->bss_type = get_u32le(r + 42);
	rec->security = get_u32le(r + 46);
	rec->wps_type = get_u32le(r + 50);
	rec->channel  = get_u32le(r + 54);
	rec->band     = get_u32le(r + 58);
	return 0;
}

const char *wifi_rpc_errno_name(int32_t e)
{
	switch (e) {
	case WIFI_LWIP_EAGAIN:        return "EAGAIN/EWOULDBLOCK";
	case WIFI_LWIP_EPIPE:         return "EPIPE";
	case WIFI_LWIP_ENOPROTOOPT:   return "ENOPROTOOPT";
	case WIFI_LWIP_ECONNABORTED:  return "ECONNABORTED";
	case WIFI_LWIP_ECONNRESET:    return "ECONNRESET";
	case WIFI_LWIP_ENOTCONN:      return "ENOTCONN";
	case WIFI_LWIP_ETIMEDOUT:     return "ETIMEDOUT";
	default:                      return "?";
	}
}

int wifi_rpc_lwip_socket(const struct wifi_rpc_opts *o, int32_t domain, int32_t type,
                         int32_t protocol, int32_t *fd)
{
	uint8_t req[12], rep[16];
	int plen, rc;

	put_u32le(req + 0, (uint32_t)domain);
	put_u32le(req + 4, (uint32_t)type);
	put_u32le(req + 8, (uint32_t)protocol);
	rc = do_call(o, SVC_WIFI_LWIP, M_LWIP_SOCKET, req, 12u, rep, sizeof(rep), &plen);
	if (rc)
		return rc;
	if (plen < 4)
		return WIFI_RPC_EDECODE;
	*fd = (int32_t)get_u32le(rep);       /* raw socket fd (>=0) or lwIP error (<0) */
	return 0;
}

int wifi_rpc_lwip_setsockopt(const struct wifi_rpc_opts *o, int32_t s, int32_t level,
                             int32_t optname, const uint8_t *optval, uint16_t optlen,
                             int32_t *ret)
{
	/* 4(s)+4(level)+4(optname) + 4+16(optval) + 4(optlen). */
	uint8_t req[40], rep[16];
	uint8_t *p = req;
	int plen, rc;

	if (optlen > 16u)
		return WIFI_RPC_EDECODE;
	put_u32le(p, (uint32_t)s);       p += 4;
	put_u32le(p, (uint32_t)level);   p += 4;
	put_u32le(p, (uint32_t)optname); p += 4;
	p = put_binary(p, optval, optlen);
	put_u32le(p, optlen);            p += 4;

	rc = do_call(o, SVC_WIFI_LWIP, M_LWIP_SETSOCKOPT, req,
	             (uint16_t)(p - req), rep, sizeof(rep), &plen);
	if (rc)
		return rc;
	if (plen < 4)
		return WIFI_RPC_EDECODE;
	*ret = (int32_t)get_u32le(rep);
	return 0;
}

int wifi_rpc_lwip_sendto(const struct wifi_rpc_opts *o, int32_t s,
                         const uint8_t *data, uint16_t dlen, int32_t flags,
                         const uint8_t *sa, uint16_t salen, int32_t *ret)
{
	/* worst case: 4(s) + 4+64(data) + 4(flags) + 4+32(sa) + 4(tolen). */
	uint8_t req[128], rep[16];
	uint8_t *p = req;
	int plen, rc;

	if (dlen > 64u || salen > 32u)
		return WIFI_RPC_EDECODE;         /* caller passed an out-of-range arg */

	put_u32le(p, (uint32_t)s);     p += 4;
	p = put_binary(p, data, dlen);
	put_u32le(p, (uint32_t)flags); p += 4;
	p = put_binary(p, sa, salen);
	put_u32le(p, salen);           p += 4;   /* tolen (== the sockaddr length) */

	rc = do_call(o, SVC_WIFI_LWIP, M_LWIP_SENDTO, req,
	             (uint16_t)(p - req), rep, sizeof(rep), &plen);
	if (rc)
		return rc;
	if (plen < 4)
		return WIFI_RPC_EDECODE;
	*ret = (int32_t)get_u32le(rep);      /* bytes sent (>=0) or lwIP error (<0) */
	return 0;
}

int wifi_rpc_lwip_recvfrom(const struct wifi_rpc_opts *o, int32_t s,
                           uint8_t *buf, uint16_t buf_cap, int32_t flags,
                           uint32_t timeout_ms, uint16_t *got, int32_t *ret)
{
	uint8_t req[20], rep[160];
	uint32_t mlen, flen, off;
	int plen, rc;

	put_u32le(req + 0,  (uint32_t)s);
	put_u32le(req + 4,  (uint32_t)buf_cap);   /* len: max bytes to receive */
	put_u32le(req + 8,  (uint32_t)flags);
	put_u32le(req + 12, 16u);                 /* fromlen: our from-buffer size */
	put_u32le(req + 16, timeout_ms);
	rc = do_call(o, SVC_WIFI_LWIP, M_LWIP_RECVFROM, req, 20u, rep, sizeof(rep), &plen);
	if (rc)
		return rc;

	/* reply = mem(binary) + from(binary) + fromlen(u32) + ret(i32).  Reject a reply
	 * larger than our scratch up-front so every subsequent offset (validated against
	 * plen) also stays within rep[] -- erpc truncates to out_cap but returns the full
	 * length, so a plen > sizeof(rep) would otherwise read uninitialised bytes. */
	if (plen > (int)sizeof(rep) || plen < 4)
		return WIFI_RPC_EDECODE;
	mlen = get_u32le(rep);
	if (mlen > buf_cap)                        /* would truncate -> fail, don't guess */
		return WIFI_RPC_EDECODE;
	off = 4u + mlen;
	if ((uint32_t)plen < off + 4u)            /* need the from-length word */
		return WIFI_RPC_EDECODE;
	flen = get_u32le(rep + off);
	if (flen > (uint32_t)sizeof(rep))         /* bound before the add so it cannot wrap */
		return WIFI_RPC_EDECODE;
	off += 4u + flen;
	if ((uint32_t)plen < off + 8u)            /* need fromlen(u32) + ret(i32) */
		return WIFI_RPC_EDECODE;

	memcpy(buf, rep + 4, mlen);
	*got = (uint16_t)mlen;
	*ret = (int32_t)get_u32le(rep + off + 4u);   /* bytes received (>=0) or error (<0) */
	return 0;
}

int wifi_rpc_lwip_recvfrom_begin(int32_t s, uint32_t len, int32_t flags,
                                 uint32_t timeout_ms, uint8_t *out, uint16_t out_cap)
{
	uint8_t req[20];

	if (len > 1024u)                          /* keep the module's reply within a frame */
		return -1;
	put_u32le(req + 0,  (uint32_t)s);
	put_u32le(req + 4,  len);                  /* len: max bytes to receive */
	put_u32le(req + 8,  (uint32_t)flags);
	put_u32le(req + 12, 16u);                  /* fromlen: our from-buffer size */
	put_u32le(req + 16, timeout_ms);
	/* Fire and reserve a token; the caller drains the reply with erpc_wait()/cancel().
	 * The reply (mem + from + fromlen + ret) is left undecoded on purpose. */
	return erpc_begin(SVC_WIFI_LWIP, M_LWIP_RECVFROM, req, 20u, out, out_cap);
}

int wifi_rpc_lwip_close(const struct wifi_rpc_opts *o, int32_t s, int32_t *ret)
{
	uint8_t req[4], rep[16];
	int plen, rc;

	put_u32le(req, (uint32_t)s);
	rc = do_call(o, SVC_WIFI_LWIP, M_LWIP_CLOSE, req, 4u, rep, sizeof(rep), &plen);
	if (rc)
		return rc;
	if (plen < 4)
		return WIFI_RPC_EDECODE;
	*ret = (int32_t)get_u32le(rep);
	return 0;
}

int wifi_rpc_lwip_errno(const struct wifi_rpc_opts *o, int32_t *err)
{
	uint8_t rep[16];
	int plen, rc;

	rc = do_call(o, SVC_WIFI_LWIP, M_LWIP_ERRNO, NULL, 0u, rep, sizeof(rep), &plen);
	if (rc)
		return rc;
	if (plen < 4)
		return WIFI_RPC_EDECODE;
	*err = (int32_t)get_u32le(rep);
	return 0;
}

/* ---- TCP stream sockets (issue #21) -------------------------------------- */
