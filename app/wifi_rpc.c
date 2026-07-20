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

/* rpc_wifi_drv (service 14) method IDs. */
#define SVC_WIFI_DRV        14u
#define M_WIFI_CONNECT       1u
#define M_WIFI_DISCONNECT    3u
#define M_WIFI_IS_CONNECTED  4u
#define M_WIFI_GET_MAC       8u
#define M_WIFI_GET_RSSI     19u
#define M_WIFI_ON           27u
#define M_WIFI_OFF          28u

/* rpc_wifi_tcpip (service 15) method IDs. */
#define SVC_WIFI_TCPIP      15u
#define M_TCPIP_INIT         1u
#define M_TCPIP_GET_IP_INFO  7u
#define M_TCPIP_SET_IP_INFO  8u
#define M_TCPIP_DHCPC_START 13u
#define M_TCPIP_DHCPC_STOP  14u

/* rpc_wifi_lwip (service 16) method IDs (raw BSD sockets for `net ping`). */
#define SVC_WIFI_LWIP       16u
#define M_LWIP_SETSOCKOPT    7u
#define M_LWIP_CLOSE         8u
#define M_LWIP_RECVFROM     14u
#define M_LWIP_SENDTO       17u
#define M_LWIP_SOCKET       18u
#define M_LWIP_ERRNO        24u

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
