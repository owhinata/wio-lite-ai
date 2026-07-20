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
 *
 * This is the Wio port of ../stm32f746g-disco's `net` command.  There the backend is
 * NetX Duo over the on-chip Ethernet MAC; here it is the RTL8720DN's eRPC socket-
 * offload (app/wifi_rpc.c), so `net` is L3 only -- the L2 side (power + WiFi
 * association) stays in `wifi` (wifi connect / status / disconnect).  Association is
 * therefore a prerequisite: ip/dhcp/ping require an active `wifi connect` (they never
 * power the module), and `net info` reports "not connected" otherwise.
 *
 * The RTL8720 eRPC link is a single-owner resource (one call in flight over a SPSC RX
 * ring), so every subcommand runs its whole transaction inside one rtl_link_begin() ..
 * rtl_link_end() session (shared with `wifi`, see rtl_link.h), which claims the console
 * and opens USART1 @2 Mbaud.  Long / blocking calls carry the Ctrl+C abort hook.
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
#define NET_SOCK_RAW       3
#define NET_IPPROTO_ICMP   1
#define NET_SOL_SOCKET     0xFFF   /* lwIP SOL_SOCKET */
#define NET_SO_RCVTIMEO    0x1006  /* lwIP SO_RCVTIMEO (an int of milliseconds here) */

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

/* Build a 16-byte lwIP sockaddr_in for @ip_host (host order). */
static void build_sockaddr_in(uint8_t sa[16], uint32_t ip_host)
{
	memset(sa, 0, 16);
	sa[0] = 16u;                         /* sin_len */
	sa[1] = (uint8_t)NET_AF_INET;        /* sin_family */
	/* sa[2..3] sin_port = 0 */
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
		build_sockaddr_in(sa, dst);

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

/* ---- registration -------------------------------------------------------- */

CLI_SUBCMD_SET_CREATE(net_subcmds,
	CLI_CMD_ARG(info, NULL, "connection + IP / mask / gateway",   cmd_net_info, 1, 0),
	CLI_CMD_ARG(ip,   NULL, "set static <a.b.c.d/mask> [gw]",     cmd_net_ip,   2, 1),
	CLI_CMD_ARG(dhcp, NULL, "(re)acquire an address via DHCP",    cmd_net_dhcp, 1, 0),
	CLI_CMD_ARG(ping, NULL, "raw ICMP echo <a.b.c.d> [count]",    cmd_net_ping, 2, 1),
	CLI_SUBCMD_SET_END);

CLI_CMD_REGISTER(net, net_subcmds,
                 "IPv4 (L3) over the onboard RTL8720 (info / ip / dhcp / ping)",
                 NULL, 1, 0);
