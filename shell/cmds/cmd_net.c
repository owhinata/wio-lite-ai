/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    cmd_net.c
 * @brief   `net` shell command (issue #5): IPv4 (L3) over the onboard RTL8720DN.
 *
 *   net info                    connection state + MAC + IP / mask / gateway
 *   net ip <a.b.c.d/mask> [gw]   set a static address (stops DHCP)
 *   net dhcp                     (re)acquire an address via DHCP
 *   net ping <a.b.c.d> [count]   raw-ICMP echo (default 4) with host-observed RTT
 *   net conc [ms]                concurrency probe: is the module's eRPC server serial
 *                                or worker-dispatched (issue #20 N3)?
 *   net echo [port] [txchunk]    TCP echo server (issue #21): the bring-up rehearsal for
 *                                the telnet shell console -- see the block above
 *                                cmd_net_echo() for what it measures and why
 *
 * This is the Wio port of ../stm32f746g-disco's `net` command.  There the backend is
 * NetX Duo over the on-chip Ethernet MAC; here it is the RTL8720DN's eRPC socket-
 * offload (app/wifi_rpc.c), so `net` is L3 only -- the L2 side (power + WiFi
 * association) stays in `wifi` (wifi connect / status / disconnect).  Association is
 * therefore a prerequisite: ip/dhcp/ping require an active `wifi connect` (they never
 * power the module), and `net info` reports "not connected" otherwise.
 *
 * Every subcommand runs its whole transaction inside one rtl_link_begin() ..
 * rtl_link_end() session (shared with `wifi`, see app/rtl_link.h): that claims the
 * console, takes the coarse link mutex -- so whole flows cannot interleave -- and
 * references the eRPC UART (USART1 @2 Mbaud).  The eRPC frames themselves are owned by
 * the resident service thread in app/erpc.c, which multiplexes several requests by
 * sequence number; `net conc` below is the diagnostic that shows that working.
 * Long / blocking calls carry the Ctrl+C abort hook (accept/recv deliberately do NOT --
 * see the `net echo` block comment).
 *
 * `net ping` opens a raw ICMP socket (rpc_lwip_socket(SOCK_RAW, IPPROTO_ICMP)) and
 * builds/parses the ICMP echo itself.  The reported RTT is host-observed: it includes
 * the two eRPC UART round-trips (sendto + recvfrom), not just the network path.
 *
 * No clock/RCC/register work -- pure marshalling + orchestration (XIP-safe).
 * Clean-room design; no third-party code reused.
 */
#include "cli.h"
#include "wifi_rpc.h"
#include "rtl_link.h"

#include "stm32h7xx_hal.h"   /* HAL_GetTick (1 ms SysTick, fed via tx_glue.c) */

#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* lwIP socket constants (Ameba-D uses the standard values). */
#define NET_AF_INET        2
#define NET_SOCK_STREAM    1
#define NET_SOCK_RAW       3
#define NET_IPPROTO_ICMP   1
#define NET_IPPROTO_TCP    6
#define NET_SOL_SOCKET     0xFFF   /* lwIP SOL_SOCKET */
#define NET_SO_REUSEADDR   0x0004
#define NET_SO_RCVTIMEO    0x1006  /* lwIP SO_RCVTIMEO (an int of milliseconds here) */
#define NET_TCP_NODELAY    0x01    /* at level IPPROTO_TCP */
#define NET_MSG_PEEK       0x01
#define NET_MSG_DONTWAIT   0x08

/* lwIP errno values we name in messages (lwip/errno.h; everything else prints raw). */
#define NET_EAGAIN         11      /* == EWOULDBLOCK */
#define NET_EPIPE          32
#define NET_ENOPROTOOPT    92
#define NET_ECONNABORTED  103
#define NET_ECONNRESET    104
#define NET_ENOTCONN      107
#define NET_ETIMEDOUT     110

#define ICMP_ECHO_REQUEST  8u
#define ICMP_ECHO_REPLY    0u
#define PING_ID            0xAB01u   /* our echo identifier (any fixed 16-bit value) */
#define PING_PAYLOAD       32u
#define PING_TIMEOUT_MS    1000u     /* per-probe receive wait (via SO_RCVTIMEO) */

/* ---- number / address parsing (ported from f746 cmd_net.c) --------------- */

static int parse_uint(const char *s, uint32_t *out)
{
	uint32_t v = 0;

	if (s == NULL || *s == '\0')
		return -1;
	for (const char *p = s; *p != '\0'; p++) {
		if (*p < '0' || *p > '9')
			return -1;
		v = v * 10u + (uint32_t)(*p - '0');
	}
	*out = v;
	return 0;
}

/* Parse "a.b.c.d" into a host-order u32 ((a<<24)|(b<<16)|(c<<8)|d). */
static int parse_ipv4(const char *s, uint32_t *out)
{
	uint32_t b[4];
	int i = 0;
	const char *p = s;

	for (;;) {
		uint32_t v = 0;
		int digits = 0;

		while (*p >= '0' && *p <= '9') {
			v = v * 10u + (uint32_t)(*p - '0');
			if (v > 255u)
				return -1;
			p++;
			digits++;
		}
		if (digits == 0 || i > 3)
			return -1;
		b[i++] = v;
		if (*p == '\0')
			break;
		if (*p != '.')
			return -1;
		p++;
	}
	if (i != 4)
		return -1;
	*out = (b[0] << 24) | (b[1] << 16) | (b[2] << 8) | b[3];
	return 0;
}

/* Parse "a.b.c.d/mask" into address + netmask (host order). */
static int parse_ipv4_cidr(const char *s, uint32_t *ip, uint32_t *mask)
{
	char buf[20];
	const char *slash = NULL;
	uint32_t bits;
	size_t n = 0;

	for (const char *p = s; *p != '\0'; p++) {
		if (*p == '/') { slash = p + 1; break; }
		if (n >= sizeof buf - 1)
			return -1;
		buf[n++] = *p;
	}
	buf[n] = '\0';
	if (slash == NULL || parse_ipv4(buf, ip) != 0)
		return -1;
	if (parse_uint(slash, &bits) != 0 || bits > 32u)
		return -1;
	*mask = (bits == 0) ? 0u : (0xFFFFFFFFu << (32u - bits));
	return 0;
}

static unsigned mask_bits(uint32_t mask)
{
	unsigned n = 0;

	while (mask & 0x80000000u) { n++; mask <<= 1; }
	return n;
}

/* host-order u32 -> network-order octets (a.b.c.d -> [a,b,c,d]). */
static void u32_to_octets(uint32_t v, uint8_t out[4])
{
	out[0] = (uint8_t)(v >> 24); out[1] = (uint8_t)(v >> 16);
	out[2] = (uint8_t)(v >> 8);  out[3] = (uint8_t)v;
}

/* ---- eRPC option / session helpers --------------------------------------- */

static void net_opts(struct wifi_rpc_opts *o, struct cli_instance *sh,
                     struct erpc_diag *diag, uint32_t timeout_ms)
{
	o->timeout_ms   = timeout_ms;
	o->should_abort = rtl_abort_cb;
	o->abort_ctx    = sh;
	o->diag         = diag;
}

/*
 * Open a net (L3) session that needs an active association.  Claims the console and
 * opens the eRPC UART (never powers the module -- that is `wifi connect`'s job), then
 * checks the module reports connected.  On RTL_LINK_READY the session is open and the
 * caller must rtl_link_end(); otherwise it has already released + printed the reason.
 */
static int net_session_connected(struct cli_instance *sh, struct wifi_rpc_opts *o,
                                 struct erpc_diag *diag)
{
	int32_t connected = -1;
	int rc, link;

	link = rtl_link_begin(sh, false);
	if (link == RTL_LINK_OFF) {
		cli_error(sh, "net: RTL8720 powered off (`wifi connect <ssid> ...` first)\r\n");
		return RTL_LINK_ERR;
	}
	if (link != RTL_LINK_READY)
		return RTL_LINK_ERR;

	net_opts(o, sh, diag, 3000u);
	rc = wifi_rpc_is_connected(o, &connected);
	if (rc || connected != WIFI_RPC_OK) {
		cli_error(sh, "net: not connected (`wifi connect <ssid> ...` first)\r\n");
		rtl_link_end(sh);
		return RTL_LINK_ERR;
	}
	return RTL_LINK_READY;
}

static void net_print_ip(struct cli_instance *sh, const struct wifi_ip_info *ip)
{
	uint32_t mask = ((uint32_t)ip->netmask[0] << 24) | ((uint32_t)ip->netmask[1] << 16) |
	                ((uint32_t)ip->netmask[2] << 8)  |  (uint32_t)ip->netmask[3];
	const char *mode = (rtl_ip_mode() == RTL_IP_DHCP)   ? "dhcp"   :
	                   (rtl_ip_mode() == RTL_IP_STATIC) ? "static" : "?";

	cli_print(sh, "ip:    %u.%u.%u.%u/%u\r\n",
	          ip->ip[0], ip->ip[1], ip->ip[2], ip->ip[3], mask_bits(mask));
	cli_print(sh, "gw:    %u.%u.%u.%u (%s)\r\n",
	          ip->gw[0], ip->gw[1], ip->gw[2], ip->gw[3], mode);
}

/* ---- subcommands --------------------------------------------------------- */

static int cmd_net_info(struct cli_instance *sh, int argc, char **argv)
{
	struct wifi_rpc_opts o;
	struct erpc_diag diag;
	struct wifi_ip_info ip;
	char mac[18];
	int32_t connected = -1, result = -1;
	int rc, link;

	(void)argc; (void)argv;
	link = rtl_link_begin(sh, false);
	if (link == RTL_LINK_OFF) {
		cli_print(sh, "net: RTL8720 powered off (`wifi connect <ssid> ...` to bring up)\r\n");
		return 0;
	}
	if (link != RTL_LINK_READY)
		return 1;

	net_opts(&o, sh, &diag, 3000u);
	rc = wifi_rpc_is_connected(&o, &connected);
	if (rc) {
		cli_error(sh, "net: query failed (rc %d)\r\n", rc);
		rtl_link_end(sh);
		return 1;
	}
	cli_print(sh, "link:  %s\r\n",
	          connected == WIFI_RPC_OK ? "up (associated)" : "down (not connected)");
	if (wifi_rpc_get_mac(&o, mac, &result) == 0 && result == WIFI_RPC_OK)
		cli_print(sh, "mac:   %s\r\n", mac);
	if (connected == WIFI_RPC_OK) {
		if (wifi_rpc_get_ip(&o, 0u, &ip, &result) == 0 && result == WIFI_RPC_OK)
			net_print_ip(sh, &ip);
		else
			cli_print(sh, "ip:    none\r\n");
	}
	rtl_link_end(sh);
	return 0;
}

static int cmd_net_ip(struct cli_instance *sh, int argc, char **argv)
{
	struct wifi_rpc_opts o;
	struct erpc_diag diag;
	struct wifi_ip_info ip;
	uint32_t a, mask, gw = 0;
	int32_t result = -1;
	int rc;

	if (parse_ipv4_cidr(argv[1], &a, &mask) != 0) {
		cli_error(sh, "net: bad address '%s' (use a.b.c.d/mask)\r\n", argv[1]);
		return 1;
	}
	if (argc >= 3 && parse_ipv4(argv[2], &gw) != 0) {
		cli_error(sh, "net: bad gateway '%s'\r\n", argv[2]);
		return 1;
	}
	if (net_session_connected(sh, &o, &diag) != RTL_LINK_READY)
		return 1;

	u32_to_octets(a,    ip.ip);
	u32_to_octets(mask, ip.netmask);
	u32_to_octets(gw,   ip.gw);

	/* Stop DHCP first so it will not overwrite the static address (best effort). */
	o.timeout_ms = 5000u;
	(void)wifi_rpc_dhcpc_stop(&o, 0u, &result);
	o.timeout_ms = 5000u;
	rc = wifi_rpc_set_ip_info(&o, 0u, &ip, &result);
	rtl_link_end(sh);

	if (rc || result != WIFI_RPC_OK) {
		cli_error(sh, "net: set static failed (rc %d, result %ld)\r\n", rc, (long)result);
		return 1;
	}
	rtl_set_ip_mode(RTL_IP_STATIC);
	cli_print(sh, "net: static address set\r\n");
	cli_print(sh, "ip:    %u.%u.%u.%u/%u\r\n",
	          ip.ip[0], ip.ip[1], ip.ip[2], ip.ip[3], mask_bits(mask));
	cli_print(sh, "gw:    %u.%u.%u.%u (static)\r\n", ip.gw[0], ip.gw[1], ip.gw[2], ip.gw[3]);
	return 0;
}

static int cmd_net_dhcp(struct cli_instance *sh, int argc, char **argv)
{
	struct wifi_rpc_opts o;
	struct erpc_diag diag;
	struct wifi_ip_info ip;
	int32_t result = -1;
	int rc;

	(void)argc; (void)argv;
	if (net_session_connected(sh, &o, &diag) != RTL_LINK_READY)
		return 1;

	cli_print(sh, "net: requesting DHCP lease (up to ~30s, Ctrl+C to stop)...\r\n");
	o.timeout_ms = 30000u;
	rc = wifi_rpc_dhcpc_start(&o, 0u, &result);
	if (rc == -4) { cli_print(sh, "net: aborted\r\n"); rtl_link_end(sh); return 1; }
	if (rc || result != WIFI_RPC_OK) {
		cli_error(sh, "net: DHCP failed (rc %d, result %ld)\r\n", rc, (long)result);
		rtl_link_end(sh);
		return 1;
	}
	rtl_set_ip_mode(RTL_IP_DHCP);

	o.timeout_ms = 3000u;
	rc = wifi_rpc_get_ip(&o, 0u, &ip, &result);
	rtl_link_end(sh);
	if (rc || result != WIFI_RPC_OK) {
		cli_error(sh, "net: get IP failed (rc %d, result %ld)\r\n", rc, (long)result);
		return 1;
	}
	net_print_ip(sh, &ip);
	return 0;
}

/* ---- raw ICMP ping ------------------------------------------------------- */

/* One's-complement (internet) checksum over @n bytes, treated as big-endian 16-bit
 * words; returns the value to store big-endian in the checksum field. */
static uint16_t inet_csum(const uint8_t *d, size_t n)
{
	uint32_t sum = 0;

	while (n > 1) { sum += (uint32_t)((d[0] << 8) | d[1]); d += 2; n -= 2; }
	if (n)        sum += (uint32_t)(d[0] << 8);
	while (sum >> 16)
		sum = (sum & 0xFFFFu) + (sum >> 16);
	return (uint16_t)~sum;
}

/* Build an ICMP echo request (8-byte header + PING_PAYLOAD) into @pkt. */
static void build_icmp_echo(uint8_t *pkt, uint16_t id, uint16_t seq)
{
	uint16_t c;
	unsigned i;

	pkt[0] = (uint8_t)ICMP_ECHO_REQUEST;
	pkt[1] = 0u;                         /* code */
	pkt[2] = 0u; pkt[3] = 0u;            /* checksum (zero while computing) */
	pkt[4] = (uint8_t)(id >> 8);  pkt[5] = (uint8_t)id;
	pkt[6] = (uint8_t)(seq >> 8); pkt[7] = (uint8_t)seq;
	for (i = 0u; i < PING_PAYLOAD; i++)
		pkt[8u + i] = (uint8_t)(0x40u + (i & 0x3Fu));
	c = inet_csum(pkt, 8u + PING_PAYLOAD);
	pkt[2] = (uint8_t)(c >> 8); pkt[3] = (uint8_t)c;
}

/* Build a 16-byte lwIP sockaddr_in for @ip_host (host order) and @port (host order;
 * 0 where the protocol has no port, e.g. the raw ICMP socket). */
static void build_sockaddr_in(uint8_t sa[16], uint32_t ip_host, uint16_t port)
{
	memset(sa, 0, 16);
	sa[0] = 16u;                         /* sin_len */
	sa[1] = (uint8_t)NET_AF_INET;        /* sin_family */
	sa[2] = (uint8_t)(port >> 8);        /* sin_port, network byte order */
	sa[3] = (uint8_t)port;
	u32_to_octets(ip_host, sa + 4);      /* sin_addr, network byte order */
	/* sa[8..15] sin_zero = 0 */
}

static int cmd_net_ping(struct cli_instance *sh, int argc, char **argv)
{
	struct wifi_rpc_opts o;
	struct erpc_diag diag;
	uint32_t dst, c;
	int count = 4, ok = 0, i;
	unsigned rmin = 0xFFFFFFFFu, rmax = 0, rsum = 0;
	int32_t fd = -1, ret = -1, cerr = 0;
	uint16_t seq = 0;
	int rc;

	if (parse_ipv4(argv[1], &dst) != 0) {
		cli_error(sh, "net: bad address '%s'\r\n", argv[1]);
		return 1;
	}
	if (argc >= 3) {                            /* explicit count must be valid */
		if (parse_uint(argv[2], &c) != 0 || c == 0 || c > 100) {
			cli_error(sh, "net: bad count '%s' (1..100)\r\n", argv[2]);
			return 1;
		}
		count = (int)c;
	}

	if (net_session_connected(sh, &o, &diag) != RTL_LINK_READY)
		return 1;

	o.timeout_ms = 5000u;
	rc = wifi_rpc_lwip_socket(&o, NET_AF_INET, NET_SOCK_RAW, NET_IPPROTO_ICMP, &fd);
	if (rc || fd < 0) {
		if (rc == 0 && fd < 0 && wifi_rpc_lwip_errno(&o, &cerr) == 0)
			cli_error(sh, "net: raw ICMP socket unavailable (errno %ld) -- the RTL8720 "
			          "firmware may lack SOCK_RAW\r\n", (long)cerr);
		else
			cli_error(sh, "net: raw ICMP socket failed (rc %d, fd %ld)\r\n", rc, (long)fd);
		rtl_link_end(sh);
		return 1;
	}

	/* The factory rpc_lwip_recvfrom IGNORES its timeout argument and blocks in
	 * lwip_recvfrom; a no-reply probe would then hang the module's single-threaded
	 * eRPC server (so a later close is never serviced -> wedged until `wifi reset`).
	 * Set SO_RCVTIMEO so lwip_recvfrom instead returns after PING_TIMEOUT_MS.  The
	 * firmware's own rpc_lwip_available uses SO_RCVTIMEO as a 4-byte int of ms. */
	{
		uint8_t ms[4];
		int32_t sret = -1;

		ms[0] = (uint8_t)PING_TIMEOUT_MS;       ms[1] = (uint8_t)(PING_TIMEOUT_MS >> 8);
		ms[2] = (uint8_t)(PING_TIMEOUT_MS >> 16); ms[3] = (uint8_t)(PING_TIMEOUT_MS >> 24);
		o.timeout_ms = 5000u;
		rc = wifi_rpc_lwip_setsockopt(&o, fd, NET_SOL_SOCKET, NET_SO_RCVTIMEO,
		                              ms, 4u, &sret);
		if (rc || sret < 0) {
			cli_error(sh, "net: SO_RCVTIMEO setup failed (rc %d, ret %ld) -- aborting "
			          "ping so a blocking recv cannot wedge the module\r\n",
			          rc, (long)sret);
			o.timeout_ms = 3000u;
			(void)wifi_rpc_lwip_close(&o, fd, &sret);
			rtl_link_end(sh);
			return 1;
		}
	}

	cli_print(sh, "PING %s, %d probes (raw ICMP over eRPC; RTT is host-observed):\r\n",
	          argv[1], count);
	for (i = 0; i < count; i++) {
		uint8_t pkt[8u + PING_PAYLOAD];
		uint8_t sa[16];
		uint8_t rbuf[96];
		uint16_t got = 0;
		uint32_t t0;

		if (cli_cancel_requested(sh))
			break;
		seq++;
		build_icmp_echo(pkt, PING_ID, seq);
		build_sockaddr_in(sa, dst, 0u);

		t0 = HAL_GetTick();
		o.timeout_ms = 3000u;
		rc = wifi_rpc_lwip_sendto(&o, fd, pkt, (uint16_t)sizeof(pkt), 0,
		                          sa, (uint16_t)sizeof(sa), &ret);
		if (rc == -4)
			break;
		if (rc || ret < 0) {
			cli_error(sh, "  probe %d: send error (rc %d, ret %ld)\r\n",
			          i + 1, rc, (long)ret);
		} else {
			int matched = 0;

			/* lwip_recvfrom returns within SO_RCVTIMEO (PING_TIMEOUT_MS, set above);
			 * give the host eRPC wait headroom past that so it does not fire first.
			 * The recvfrom `timeout` argument itself is ignored by the firmware. */
			o.timeout_ms = PING_TIMEOUT_MS + 2000u;
			rc = wifi_rpc_lwip_recvfrom(&o, fd, rbuf, (uint16_t)sizeof(rbuf), 0,
			                            PING_TIMEOUT_MS, &got, &ret);
			if (rc == -4)
				break;
			if (rc == 0 && ret > 0 && got >= 28u && (rbuf[0] >> 4) == 4u) {
				unsigned ihl = (unsigned)(rbuf[0] & 0x0Fu) * 4u;

				if (ihl >= 20u && got >= ihl + 8u) {
					const uint8_t *icmp = rbuf + ihl;
					uint16_t rid  = (uint16_t)((icmp[4] << 8) | icmp[5]);
					uint16_t rseq = (uint16_t)((icmp[6] << 8) | icmp[7]);

					if (icmp[0] == ICMP_ECHO_REPLY && rid == PING_ID && rseq == seq) {
						unsigned rtt = (unsigned)(HAL_GetTick() - t0);

						cli_print(sh, "  reply %d: %u ms\r\n", i + 1, rtt);
						ok++;
						rsum += rtt;
						if (rtt < rmin) rmin = rtt;
						if (rtt > rmax) rmax = rtt;
						matched = 1;
					}
				}
			}
			if (!matched)
				cli_print(sh, "  probe %d: timeout\r\n", i + 1);
		}
		if (i + 1 < count && cli_sleep(sh, 1000u))
			break;                          /* Ctrl+C between probes */
	}

	o.timeout_ms = 3000u;
	(void)wifi_rpc_lwip_close(&o, fd, &ret);
	rtl_link_end(sh);

	cli_print(sh, "%d/%d received", ok, count);
	if (ok > 0)
		cli_print(sh, ", rtt min/avg/max %u/%u/%u ms",
		          rmin, rsum / (unsigned)ok, rmax);
	cli_print(sh, "\r\n");
	return 0;
}

/* ---- concurrency probe (issue #20 N3) ------------------------------------ */

/*
 * Decide whether the module's eRPC server is the stock single-task serial one or the N3
 * worker-dispatch one, using only existing wrappers.  Open a raw ICMP socket like `net
 * ping` but never send an echo request, then issue recvfrom asynchronously: with no data
 * arriving it blocks on the module for up to `ms`.  While it is outstanding, round-trip a
 * foreground system-ack.  A serial server cannot answer the ack until recvfrom returns
 * (so it times out); a worker server answers it in a few ms.
 *
 * WARNING: recvfrom is only bounded on N2+ firmware (which honours the timeout).  On
 * stock / N1 firmware recvfrom blocks forever and wedges the single-task server -- the
 * ack times out AND the module needs `wifi reset` afterwards.
 */
static int cmd_net_conc(struct cli_instance *sh, int argc, char **argv)
{
	struct wifi_rpc_opts o;
	struct erpc_diag diag, adiag;
	uint32_t block_ms = 3000u;
	uint8_t rcvbuf[96];
	uint8_t echoed = 0u;
	int32_t fd = -1, ret = -1;
	int token, rc, an, ack_dt;
	uint32_t t0;

	if (argc >= 2) {
		if (parse_uint(argv[1], &block_ms) != 0 || block_ms < 500u || block_ms > 20000u) {
			cli_error(sh, "net: bad ms '%s' (500..20000)\r\n", argv[1]);
			return 1;
		}
	}
	if (net_session_connected(sh, &o, &diag) != RTL_LINK_READY)
		return 1;

	o.timeout_ms = 5000u;
	rc = wifi_rpc_lwip_socket(&o, NET_AF_INET, NET_SOCK_RAW, NET_IPPROTO_ICMP, &fd);
	if (rc || fd < 0) {
		cli_error(sh, "net: raw ICMP socket failed (rc %d, fd %ld)\r\n", rc, (long)fd);
		rtl_link_end(sh);
		return 1;
	}

	cli_print(sh, "net: concurrency probe -- recvfrom(%lu ms, no data) held open, "
	          "then a foreground ack\r\n", (unsigned long)block_ms);
	cli_print(sh, "     (stock/N1 FW would wedge here -- `wifi reset` if the ack never "
	          "returns)\r\n");

	/* Fire the blocking recvfrom (it will not reply for ~block_ms), then immediately
	 * round-trip a system ack.  erpc_begin does NOT flush the RX while this token is in
	 * flight: the link service never flushes the RX while a frame is on the wire, so the
	 * ack's reply and the recvfrom's reply are routed to their tokens by sequence. */
	token = wifi_rpc_lwip_recvfrom_begin(fd, 64u, 0, block_ms,
	                                     rcvbuf, (uint16_t)sizeof(rcvbuf));
	if (token < 0) {
		cli_error(sh, "net: recvfrom_begin failed (%d)\r\n", token);
		o.timeout_ms = 3000u;
		(void)wifi_rpc_lwip_close(&o, fd, &ret);
		rtl_link_end(sh);
		return 1;
	}

	t0 = HAL_GetTick();
	an = erpc_system_ack(0x5Au, &echoed, &adiag);
	ack_dt = (int)(HAL_GetTick() - t0);

	if (an == 0 && echoed == 0x5Au)
		cli_print(sh, "  ack: echoed 0x5A in %d ms  =>  server is CONCURRENT "
		          "(N3 worker dispatch)\r\n", ack_dt);
	else
		cli_print(sh, "  ack: no reply within %d ms (rc %d)  =>  server is SERIAL "
		          "(stock/N2: ack waits behind recvfrom)\r\n", ack_dt, an);

	/* Collect the recvfrom reply so the link is left clean (it returns after ~block_ms
	 * on N2+; give the wait headroom).  The datagram itself is not decoded. */
	rc = erpc_wait(token, block_ms + 3000u, &diag, rtl_abort_cb, sh);
	if (rc < 0) {
		erpc_cancel(token);
		cli_print(sh, "  recvfrom: no reply (rc %d) -- module may be wedged; "
		          "run `wifi reset`\r\n", rc);
	} else {
		cli_print(sh, "  recvfrom: returned after the ack (payload %d B)\r\n", rc);
	}

	o.timeout_ms = 3000u;
	(void)wifi_rpc_lwip_close(&o, fd, &ret);
	rtl_link_end(sh);
	return 0;
}

/* ---- `net echo`: TCP echo server (issue #21, increment 7) ----------------- */
/*
 * The bring-up rehearsal for the telnet shell console (issue #21).  That console will
 * run a resident service thread holding a BLOCKING accept / recv on the module while the
 * shell instance thread runs commands; this command exercises exactly that mechanism --
 * bind/listen/accept/recv/send with the same firmware timeouts -- but from a single
 * shell command, so it adds no concurrency of its own (increment 8 then moved the link
 * under a service thread that multiple clients can share).  Its output is the
 * measurement the console design needs:
 * accept latency, the firmware's accept cap, whether MSG_DONTWAIT works, echo RTT,
 * whether getpeername is usable, and that the link is left clean.
 *
 * Two rules here are load-bearing and carry over to issue #21's service thread:
 *
 *   - NO abort hook on any of these calls.  Aborting only ends the host's wait; the
 *     module keeps running the call.  An aborted accept can complete into a socket whose
 *     fd the host never learns (unclosable, and it eats one of the module's netconns),
 *     and an aborted recv leaves that fd busy, so it must not be closed either.  Ctrl+C
 *     is therefore honoured between calls only -- which is why it can take one accept
 *     window (~12 s) to act.
 *   - A host-side TIMEOUT (rc -2) means the same thing as an abort, so it is treated as
 *     "link dirty": the sockets are deliberately NOT closed (a close racing an in-flight
 *     accept/recv on the same fd is unsafe on this firmware) and the user is told to run
 *     `wifi reset`, which power-cycles the module and reclaims everything.  The host
 *     timeouts below are set well above the firmware's own caps so this should never
 *     happen -- if it does, the headroom assumption is wrong and must be revisited.
 */

#define ECHO_DEF_PORT       2323u    /* not 23: the telnet console will want that later */
#define ECHO_ACCEPT_MS     12000u    /* > the firmware's internal 10 s accept cap */
#define ECHO_RECV_MS        2000u    /* SO_RCVTIMEO the firmware applies for us */
#define ECHO_RECV_HOST_MS   5000u    /* > ECHO_RECV_MS */
#define ECHO_SHORT_MS       5000u    /* everything else returns immediately */
#define ECHO_CHUNK          WIFI_RPC_STREAM_MAX

/* A host-side timeout / abort left the module still owning the call (see above). */
#define ECHO_DIRTY(rc)      ((rc) == -2 || (rc) == -4)

struct echo_run {
	struct cli_instance  *sh;
	struct wifi_rpc_opts  o;
	struct erpc_diag      diag;      /* per-call (each call clears it) */
	struct erpc_diag      tot;       /* accumulated over the whole command */
	unsigned long         rx, tx;    /* bytes received / echoed back */
	uint32_t              emin, emax, esum, en;   /* echo send RTT, ms */
	uint16_t              txchunk;   /* bytes per send RPC (see WIFI_RPC_SEND_SAFE) */
	unsigned              sessions;
	int                   dirty;
};

static const char *errno_name(int32_t e)
{
	switch (e) {
	case NET_EAGAIN:        return "EAGAIN/EWOULDBLOCK";
	case NET_EPIPE:         return "EPIPE";
	case NET_ENOPROTOOPT:   return "ENOPROTOOPT";
	case NET_ECONNABORTED:  return "ECONNABORTED";
	case NET_ECONNRESET:    return "ECONNRESET";
	case NET_ENOTCONN:      return "ENOTCONN";
	case NET_ETIMEDOUT:     return "ETIMEDOUT";
	default:                return "?";
	}
}

static void diag_acc(struct erpc_diag *tot, const struct erpc_diag *d)
{
	tot->crc_fail               += d->crc_fail;
	tot->oversize               += d->oversize;
	tot->timeout                += d->timeout;
	tot->skipped_reply          += d->skipped_reply;
	tot->unsupported_invocation += d->unsupported_invocation;
	tot->frame_stall            += d->frame_stall;
}

/*
 * Report the accumulated eRPC diagnostics, but only when something actually went wrong --
 * a clean link stays silent.  Called from points reached BEFORE a Ctrl+C can be latched,
 * because the run summary at the end of the command does not survive one.
 */
static void echo_diag_if_dirty(struct echo_run *r)
{
	const struct erpc_diag *d = &r->tot;

	if (d->crc_fail || d->oversize || d->timeout || d->skipped_reply ||
	    d->unsupported_invocation || d->frame_stall)
		cli_warn(r->sh, "  erpc diag so far: crc %u oversize %u timeout %u stale %u "
		         "unsupported %u stall %u\r\n", d->crc_fail, d->oversize, d->timeout,
		         d->skipped_reply, d->unsupported_invocation, d->frame_stall);
}

/* Per-call options for every echo RPC: deliberately WITHOUT the Ctrl+C abort hook. */
static void echo_opts(struct echo_run *r, uint32_t timeout_ms)
{
	r->o.timeout_ms   = timeout_ms;
	r->o.should_abort = NULL;
	r->o.abort_ctx    = NULL;
	r->o.diag         = &r->diag;
}

/* Ask the module why the last call failed.  Returns 0 if even that did not get through
 * (the caller then prints the raw return value instead). */
static int32_t echo_errno(struct echo_run *r)
{
	int32_t e = 0;

	echo_opts(r, ECHO_SHORT_MS);
	if (wifi_rpc_lwip_errno(&r->o, &e) != 0)
		e = 0;
	diag_acc(&r->tot, &r->diag);
	return e;
}

/*
 * Close @fd.  Returns 0 when the round-trip completed -- the fd is no longer ours -- or
 * -1 when the link went dirty (r->dirty is set: the module may still be running a call
 * on that fd, so the caller must neither re-close nor forget it silently).  A close that
 * round-trips but fails is only warned about: retrying cannot help, and `wifi reset`
 * reclaims the socket.
 */
static int echo_close(struct echo_run *r, int32_t fd, const char *what)
{
	int32_t ret = -1;
	int rc;

	echo_opts(r, ECHO_SHORT_MS);
	rc = wifi_rpc_lwip_close(&r->o, fd, &ret);
	diag_acc(&r->tot, &r->diag);
	if (ECHO_DIRTY(rc)) {
		r->dirty = 1;
		return -1;
	}
	if (rc || ret < 0)
		cli_warn(r->sh, "  close(%s, fd %ld) failed (rc %d, ret %ld) -- the module may "
		         "still hold it; `wifi reset` clears it\r\n",
		         what, (long)fd, rc, (long)ret);
	return 0;
}

/* One int-valued setsockopt.  Returns the wrapper's transport code; *@sret is lwIP's. */
static int echo_setopt(struct echo_run *r, int32_t fd, int32_t level, int32_t optname,
                       int32_t val, int32_t *sret)
{
	uint8_t v[4];

	v[0] = (uint8_t)val;         v[1] = (uint8_t)((uint32_t)val >> 8);
	v[2] = (uint8_t)((uint32_t)val >> 16); v[3] = (uint8_t)((uint32_t)val >> 24);
	echo_opts(r, ECHO_SHORT_MS);
	*sret = -1;
	return wifi_rpc_lwip_setsockopt(&r->o, fd, level, optname, v, 4u, sret);
}

/*
 * Serve one accepted connection.  Returns 0 when the session ended and further clients
 * may be served, 1 when the user cancelled, -1 when the link went dirty.
 */
static int echo_session(struct echo_run *r, int32_t cfd)
{
	struct cli_instance *sh = r->sh;
	uint8_t buf[ECHO_CHUNK];
	int32_t ret = -1, sret = -1;
	/* Per-session counters, reported when the session ends.  They exist because the
	 * run-wide summary printed at the end of this command is DROPPED whenever Ctrl+C is
	 * latched -- the core deliberately stops emitting handler output from that moment
	 * (cli_core.c, issue #16) -- and Ctrl+C is the normal way to leave `net echo`.  So
	 * everything worth measuring is printed as it happens, not only at the end. */
	unsigned long srx = 0u, stx = 0u;
	uint32_t smin = 0xFFFFFFFFu, smax = 0u, ssum = 0u, sn = 0u;
	int rc, result = 0;

	/* Without TCP_NODELAY, Nagle holds a single-byte echo until the previous segment is
	 * ACKed -- precisely the latency an interactive console cannot afford. */
	rc = echo_setopt(r, cfd, NET_IPPROTO_TCP, NET_TCP_NODELAY, 1, &sret);
	diag_acc(&r->tot, &r->diag);
	if (ECHO_DIRTY(rc))
		return -1;
	if (rc || sret < 0)
		cli_warn(sh, "  TCP_NODELAY failed (rc %d, ret %ld) -- echo may lag\r\n",
		         rc, (long)sret);

	/* Peer address: best effort only, see the warning in wifi_rpc.h. */
	{
		uint8_t psa[16];
		uint16_t pn = 0;
		int32_t pret = -1;

		memset(psa, 0, sizeof(psa));
		echo_opts(r, ECHO_SHORT_MS);
		rc = wifi_rpc_lwip_getpeername(&r->o, cfd, psa, &pn, &pret);
		diag_acc(&r->tot, &r->diag);
		if (ECHO_DIRTY(rc))
			return -1;
		if (rc != 0 || pret < 0 || pn != 16u ||
		    psa[0] != 16u || psa[1] != (uint8_t)NET_AF_INET) {
			cli_print(sh, "  peer: unknown (rc %d ret %ld len %u sin_len %u family %u)\r\n",
			          rc, (long)pret, (unsigned)pn, (unsigned)psa[0], (unsigned)psa[1]);
		} else if (psa[4] == 0u && psa[5] == 0u && psa[6] == 0u && psa[7] == 0u) {
			/* Measured on board #2: the firmware's uninitialised socklen came out as 4,
			 * so lwIP copied sin_len/sin_family/sin_port and stopped -- the address is
			 * simply not there.  Report the port (which IS real) and say so. */
			cli_print(sh, "  peer: port %u, address truncated by the firmware "
			          "(uninitialised socklen -- see wifi_rpc.h)\r\n",
			          (unsigned)(((unsigned)psa[2] << 8) | psa[3]));
		} else {
			cli_print(sh, "  peer: %u.%u.%u.%u:%u\r\n", psa[4], psa[5], psa[6], psa[7],
			          (unsigned)(((unsigned)psa[2] << 8) | psa[3]));
		}
	}

	/* Does a non-blocking receive actually work on this firmware?  MSG_PEEK keeps it
	 * non-destructive, so the byte (if any) still reaches the echo loop below. */
	{
		uint8_t pb[1];
		uint16_t pgot = 0;
		int32_t pret = -1;
		uint32_t t0 = HAL_GetTick(), dt;

		echo_opts(r, ECHO_SHORT_MS);
		rc = wifi_rpc_lwip_recv(&r->o, cfd, pb, 1u,
		                        NET_MSG_DONTWAIT | NET_MSG_PEEK, 1u, &pgot, &pret);
		diag_acc(&r->tot, &r->diag);
		dt = HAL_GetTick() - t0;
		if (ECHO_DIRTY(rc))
			return -1;
		if (rc)
			cli_print(sh, "  MSG_DONTWAIT probe: transport rc %d\r\n", rc);
		else if (pret > 0)
			cli_print(sh, "  MSG_DONTWAIT probe: %ld byte(s) pending in %lu ms "
			          "(peeked, not consumed)\r\n", (long)pret, (unsigned long)dt);
		else {
			int32_t e = (pret < 0) ? echo_errno(r) : 0;

			cli_print(sh, "  MSG_DONTWAIT probe: ret %ld errno %ld %s in %lu ms\r\n",
			          (long)pret, (long)e, errno_name(e), (unsigned long)dt);
		}
	}

	for (;;) {
		uint16_t got = 0, sent = 0;
		uint32_t t0, dt;

		if (cli_cancel_requested(sh)) {
			result = 1;
			goto done;
		}

		echo_opts(r, ECHO_RECV_HOST_MS);
		rc = wifi_rpc_lwip_recv(&r->o, cfd, buf, (uint16_t)sizeof(buf), 0,
		                        ECHO_RECV_MS, &got, &ret);
		diag_acc(&r->tot, &r->diag);
		if (ECHO_DIRTY(rc)) {
			result = -1;
			goto done;
		}
		if (rc) {
			cli_error(sh, "  recv transport error (rc %d)\r\n", rc);
			goto done;
		}
		if (ret == 0) {
			cli_print(sh, "  peer closed (recv 0)\r\n");
			goto done;
		}
		if (ret < 0) {
			int32_t e = echo_errno(r);

			if (e == NET_EAGAIN)
				continue;         /* idle: the module's receive timed out */
			cli_print(sh, "  session ended: recv errno %ld %s\r\n",
			          (long)e, errno_name(e));
			goto done;
		}
		r->rx += got;
		srx    += got;

		t0 = HAL_GetTick();
		while (sent < got) {
			uint16_t want = (uint16_t)(got - sent);

			/* Chunked because the REQUEST direction is the constrained one: a frame
			 * larger than the module's 127-byte UART ring loses its tail (see the
			 * ASYMMETRY note in wifi_rpc.h).  Receiving above used the full buffer. */
			if (want > r->txchunk)
				want = r->txchunk;
			echo_opts(r, ECHO_SHORT_MS);
			rc = wifi_rpc_lwip_send(&r->o, cfd, buf + sent, want, 0, &ret);
			diag_acc(&r->tot, &r->diag);
			if (ECHO_DIRTY(rc)) {
				cli_error(sh, "  send of %u B (a %u B request frame) got no reply -- "
				          "the module's 127 B UART ring probably dropped it; retry "
				          "with a smaller txchunk\r\n",
				          (unsigned)want, (unsigned)(want + 24u));
				result = -1;
				goto done;
			}
			if (rc || ret <= 0) {
				int32_t e = (rc == 0) ? echo_errno(r) : 0;

				cli_print(sh, "  session ended: send rc %d ret %ld errno %ld %s\r\n",
				          rc, (long)ret, (long)e, errno_name(e));
				goto done;
			}
			if ((uint32_t)ret > (uint32_t)want) {
				cli_error(sh, "  send claimed %ld of %u bytes -- aborting session\r\n",
				          (long)ret, (unsigned)want);
				goto done;
			}
			sent   += (uint16_t)ret;
			r->tx  += (unsigned long)ret;
			stx    += (unsigned long)ret;
		}
		dt = HAL_GetTick() - t0;
		if (dt < r->emin) r->emin = dt;
		if (dt > r->emax) r->emax = dt;
		r->esum += dt;
		r->en++;
		if (dt < smin) smin = dt;
		if (dt > smax) smax = dt;
		ssum += dt;
		sn++;

		/* Show the data path working as it happens (first few chunks, then sparsely so
		 * a bulk transfer does not flood the console). */
		if (sn <= 8u || (sn % 32u) == 0u)
			cli_print(sh, "  echo #%lu: %u B in %lu ms\r\n",
			          (unsigned long)sn, (unsigned)got, (unsigned long)dt);
	}

done:
	if (sn)
		cli_print(sh, "  session %u: rx %lu B, tx %lu B, echo RTT min/avg/max "
		          "%lu/%lu/%lu ms over %lu echoes\r\n",
		          r->sessions, srx, stx, (unsigned long)smin,
		          (unsigned long)(ssum / sn), (unsigned long)smax, (unsigned long)sn);
	else
		cli_print(sh, "  session %u: no data exchanged (rx %lu B)\r\n",
		          r->sessions, srx);
	echo_diag_if_dirty(r);
	return result;
}

static int cmd_net_echo(struct cli_instance *sh, int argc, char **argv)
{
	struct echo_run r;
	uint8_t sa[16];
	uint32_t port = ECHO_DEF_PORT, chunk = WIFI_RPC_SEND_SAFE, v;
	int32_t lfd = -1, cfd = -1, ret = -1, sret = -1;
	int rc, done = 0;

	if (argc >= 2) {
		if (parse_uint(argv[1], &v) != 0 || v == 0u || v > 65535u) {
			cli_error(sh, "net: bad port '%s' (1..65535)\r\n", argv[1]);
			return 1;
		}
		port = v;
	}
	/* Optional txchunk: bytes per send RPC.  Default WIFI_RPC_SEND_SAFE; it is settable
	 * so the request-frame limit can be swept on hardware (see wifi_rpc.h). */
	if (argc >= 3) {
		if (parse_uint(argv[2], &v) != 0 || v == 0u || v > WIFI_RPC_STREAM_MAX) {
			cli_error(sh, "net: bad txchunk '%s' (1..%u)\r\n",
			          argv[2], (unsigned)WIFI_RPC_STREAM_MAX);
			return 1;
		}
		chunk = v;
	}

	memset(&r, 0, sizeof(r));
	r.sh      = sh;
	r.emin    = 0xFFFFFFFFu;
	r.txchunk = (uint16_t)chunk;

	if (net_session_connected(sh, &r.o, &r.diag) != RTL_LINK_READY)
		return 1;

	echo_opts(&r, ECHO_SHORT_MS);
	rc = wifi_rpc_lwip_socket(&r.o, NET_AF_INET, NET_SOCK_STREAM, 0, &lfd);
	diag_acc(&r.tot, &r.diag);
	if (ECHO_DIRTY(rc)) { r.dirty = 1; goto out; }
	if (rc || lfd < 0) {
		cli_error(sh, "net: socket(SOCK_STREAM) failed (rc %d, fd %ld)\r\n",
		          rc, (long)lfd);
		lfd = -1;
		goto out;
	}

	/* Best effort: without SO_REUSEADDR a rebind during TIME_WAIT can fail. */
	rc = echo_setopt(&r, lfd, NET_SOL_SOCKET, NET_SO_REUSEADDR, 1, &sret);
	diag_acc(&r.tot, &r.diag);
	if (ECHO_DIRTY(rc)) { r.dirty = 1; goto out; }
	if (rc || sret < 0)
		cli_warn(sh, "net: SO_REUSEADDR failed (rc %d, ret %ld) -- an immediate "
		         "re-run may not be able to rebind the port\r\n", rc, (long)sret);

	build_sockaddr_in(sa, 0u, (uint16_t)port);     /* 0.0.0.0:port */
	echo_opts(&r, ECHO_SHORT_MS);
	rc = wifi_rpc_lwip_bind(&r.o, lfd, sa, (uint16_t)sizeof(sa), &ret);
	diag_acc(&r.tot, &r.diag);
	if (ECHO_DIRTY(rc)) { r.dirty = 1; goto out; }
	if (rc || ret < 0) {
		int32_t e = (rc == 0) ? echo_errno(&r) : 0;

		cli_error(sh, "net: bind :%lu failed (rc %d, ret %ld, errno %ld %s)\r\n",
		          (unsigned long)port, rc, (long)ret, (long)e, errno_name(e));
		goto out;
	}

	echo_opts(&r, ECHO_SHORT_MS);
	rc = wifi_rpc_lwip_listen(&r.o, lfd, 1, &ret);
	diag_acc(&r.tot, &r.diag);
	if (ECHO_DIRTY(rc)) { r.dirty = 1; goto out; }
	if (rc || ret < 0) {
		int32_t e = (rc == 0) ? echo_errno(&r) : 0;

		cli_error(sh, "net: listen failed (rc %d, ret %ld, errno %ld %s)\r\n",
		          rc, (long)ret, (long)e, errno_name(e));
		goto out;
	}

	{
		struct wifi_ip_info ip;
		int32_t result = -1;

		echo_opts(&r, ECHO_SHORT_MS);
		rc = wifi_rpc_get_ip(&r.o, 0u, &ip, &result);
		diag_acc(&r.tot, &r.diag);
		if (rc == 0 && result == WIFI_RPC_OK)
			cli_print(sh, "net: TCP echo server on %u.%u.%u.%u:%lu\r\n",
			          ip.ip[0], ip.ip[1], ip.ip[2], ip.ip[3], (unsigned long)port);
		else
			cli_print(sh, "net: TCP echo server on port %lu\r\n", (unsigned long)port);
	}
	cli_print(sh, "     connect with `nc <ip> %lu`; Ctrl+C stops (up to %lu s to act)\r\n",
	          (unsigned long)port, (unsigned long)(ECHO_ACCEPT_MS / 1000u));
	cli_print(sh, "     rx up to %u B per RPC, tx %u B per RPC (%u B request frame)\r\n",
	          (unsigned)WIFI_RPC_STREAM_MAX, (unsigned)r.txchunk,
	          (unsigned)(r.txchunk + 24u));

	while (!done) {
		uint32_t t0, dt;

		if (cli_cancel_requested(sh))
			break;

		t0 = HAL_GetTick();
		echo_opts(&r, ECHO_ACCEPT_MS);
		rc = wifi_rpc_lwip_accept(&r.o, lfd, &cfd);
		diag_acc(&r.tot, &r.diag);
		dt = HAL_GetTick() - t0;
		if (ECHO_DIRTY(rc)) { r.dirty = 1; goto out; }
		if (rc) {
			cli_error(sh, "net: accept transport error (rc %d)\r\n", rc);
			goto out;
		}
		if (cfd < 0) {
			int32_t e = echo_errno(&r);

			cfd = -1;
			if (e == NET_ETIMEDOUT) {
				cli_print(sh, "  no client within %lu ms (firmware accept cap) -- "
				          "listening again\r\n", (unsigned long)dt);
				echo_diag_if_dirty(&r);
				continue;
			}
			cli_error(sh, "net: accept failed (errno %ld %s)\r\n",
			          (long)e, errno_name(e));
			goto out;
		}

		r.sessions++;
		cli_print(sh, "  client %u accepted in %lu ms (fd %ld)\r\n",
		          r.sessions, (unsigned long)dt, (long)cfd);

		rc = echo_session(&r, cfd);
		if (rc < 0) { r.dirty = 1; goto out; }
		if (rc > 0) done = 1;             /* Ctrl+C: close this fd, then stop */

		if (echo_close(&r, cfd, "client socket") < 0)
			goto out;                 /* dirty: the module still owns cfd */
		cfd = -1;
	}

out:
	/* Close what is still open -- but NOT once the link is dirty: the module may still be
	 * running an accept/recv on those fds and a close racing that is unsafe on this
	 * firmware.  Then leak them deliberately; `wifi reset` power-cycles the module and
	 * reclaims everything.  echo_close() sets r.dirty itself if it times out, which is
	 * what skips the second close below. */
	if (!r.dirty && cfd >= 0)
		(void)echo_close(&r, cfd, "client socket");
	if (!r.dirty && lfd >= 0)
		(void)echo_close(&r, lfd, "listening socket");
	if (r.dirty)
		cli_error(sh, "net: link dirty -- the host stopped waiting while the module was "
		          "still running a call; its sockets are left open. Run `wifi reset`.\r\n");

	cli_print(sh, "net echo: %u session(s), rx %lu B, tx %lu B\r\n",
	          r.sessions, r.rx, r.tx);
	if (r.en)
		cli_print(sh, "  echo send RTT: min %lu / avg %lu / max %lu ms over %lu sends\r\n",
		          (unsigned long)r.emin, (unsigned long)(r.esum / r.en),
		          (unsigned long)r.emax, (unsigned long)r.en);
	cli_print(sh, "  erpc diag: crc %u oversize %u timeout %u stale %u unsupported %u "
	          "stall %u\r\n", r.tot.crc_fail, r.tot.oversize, r.tot.timeout,
	          r.tot.skipped_reply, r.tot.unsupported_invocation, r.tot.frame_stall);
	rtl_link_end(sh);
	return r.dirty ? 1 : 0;
}

/* ---- registration -------------------------------------------------------- */

CLI_SUBCMD_SET_CREATE(net_subcmds,
	CLI_CMD_ARG(info, NULL, "connection + IP / mask / gateway",   cmd_net_info, 1, 0),
	CLI_CMD_ARG(ip,   NULL, "set static <a.b.c.d/mask> [gw]",     cmd_net_ip,   2, 1),
	CLI_CMD_ARG(dhcp, NULL, "(re)acquire an address via DHCP",    cmd_net_dhcp, 1, 0),
	CLI_CMD_ARG(ping, NULL, "raw ICMP echo <a.b.c.d> [count]",    cmd_net_ping, 2, 1),
	CLI_CMD_ARG(conc, NULL, "eRPC server concurrency probe [ms]", cmd_net_conc, 1, 1),
	CLI_CMD_ARG(echo, NULL, "TCP echo server [port] [txchunk] (Ctrl+C stops)",
	            cmd_net_echo, 1, 2),
	CLI_SUBCMD_SET_END);

CLI_CMD_REGISTER(net, net_subcmds,
                 "IPv4 (L3) over the onboard RTL8720 (info / ip / dhcp / ping / echo)",
                 NULL, 1, 0);
