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
 *   net echo [port]              TCP echo server on the HOST stack (issue #23 U4-1):
 *                                the bring-up rehearsal for the telnet console -- see the
 *                                block above cmd_net_echo() and app/nx_echo.c
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
#include "net_shell.h"
#include "nx_echo.h"  /* `net echo` on the host stack (issue #23 U4-1) */
#include "nx_net.h"   /* the host stack: `net up` switches the backend (issue #23 U3) */

#include "stm32h7xx_hal.h"   /* HAL_GetTick (1 ms SysTick, fed via tx_glue.c) */

#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* The lwIP socket ABI constants live in app/wifi_rpc.h (WIFI_LWIP_*): they are part of
 * the module wire contract, shared with the issue-#21 telnet console backend. */

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

/* ---- host stack (issue #23 U3) ------------------------------------------- */

/*
 * `net` now has two possible backends and picks between them with ONE predicate,
 * nx_net_is_up(), so no subcommand can disagree with its neighbour about which stack
 * answered:
 *
 *   down (the default)  the module's lwIP, reached over eRPC -- everything below the
 *                       U3 section, unchanged
 *   up   (`net up`)     NetX Duo on this MCU, over the L2 bridge (app/nx_net.c)
 *
 * They are mutually exclusive by construction, not by policy: while the bridge is in,
 * the module's WiFi netif is tapped and its own lwIP receives nothing at all, so
 * `net ping`/`conc`/`echo` and the telnet console -- all of which run on that lwIP --
 * are refused rather than left to time out mysteriously.
 *
 * Note what does NOT change: association, MAC and RSSI come from the module's WiFi
 * DRIVER (eRPC service 14), which the tap does not touch, so `net info` reports them the
 * same way either way.  Only the address changes hands.
 */

#define NET_NX_POLL_MS      100u
#define NET_NX_ARM_WAIT_MS  20000u
#define NET_NX_DHCP_WAIT_MS 30000u

static void net_print_nx_ip(struct cli_instance *sh, const struct nx_net_info *ni)
{
	cli_print(sh, "ip:    %u.%u.%u.%u/%u\r\n",
	          (unsigned)((ni->ip >> 24) & 0xFFu), (unsigned)((ni->ip >> 16) & 0xFFu),
	          (unsigned)((ni->ip >> 8) & 0xFFu),  (unsigned)(ni->ip & 0xFFu),
	          mask_bits(ni->mask));
	cli_print(sh, "gw:    %u.%u.%u.%u (%s)\r\n",
	          (unsigned)((ni->gw >> 24) & 0xFFu), (unsigned)((ni->gw >> 16) & 0xFFu),
	          (unsigned)((ni->gw >> 8) & 0xFFu),  (unsigned)(ni->gw & 0xFFu),
	          ni->dhcp_mode ? "dhcp" : "static");
}

/*
 * ARMING and STOPPING are transient.  Wait for them to resolve rather than guessing a
 * backend -- picking one mid-transition is how a command ends up talking to the stack
 * that is on its way out.
 */
static int net_nx_settled(struct cli_instance *sh)
{
	unsigned waited = 0u;

	while (nx_net_state() == NX_NET_ARMING || nx_net_state() == NX_NET_STOPPING) {
		if (cli_cancel_requested(sh) || waited >= NET_NX_ARM_WAIT_MS) {
			cli_error(sh, "net: the host stack is still %s\r\n",
			          nx_net_state() == NX_NET_ARMING ? "coming up" : "going down");
			return -1;
		}
		cli_sleep(sh, NET_NX_POLL_MS);
		waited += NET_NX_POLL_MS;
	}
	if (nx_net_state() == NX_NET_FAILED) {
		cli_error(sh, "net: the host stack could not be shut down cleanly -- "
		          "run `wifi reset`\r\n");
		return -1;
	}
	return 0;
}

static int cmd_net_up(struct cli_instance *sh, int argc, char **argv)
{
	const char *why = "";
	unsigned waited = 0u;

	(void)argc; (void)argv;

	if (nx_net_is_up()) {
		cli_print(sh, "net: the host stack is already up\r\n");
		nx_net_print_status(sh);
		return 0;
	}
	if (net_nx_settled(sh) != 0)
		return 1;
	if (nx_net_up(&why) != 0) {
		cli_error(sh, "net: %s\r\n", why);
		return 1;
	}

	cli_print(sh, "net: bringing the host stack up (bridging the module)...\r\n");
	while (nx_net_state() == NX_NET_ARMING) {
		if (waited >= NET_NX_ARM_WAIT_MS)
			break;
		/* Ctrl+C stops WAITING, not the owner: aborting a half-installed bridge from
		 * a second thread is exactly the unwind app/nx_net.h refuses to improvise. */
		if (cli_cancel_requested(sh)) {
			cli_print(sh, "net: still arming in the background; `net info` to "
			          "check\r\n");
			return 1;
		}
		cli_sleep(sh, NET_NX_POLL_MS);
		waited += NET_NX_POLL_MS;
	}

	nx_net_print_status(sh);
	if (!nx_net_is_up())
		return 1;
	cli_print(sh, "net: run `net dhcp` to take a lease with the host stack\r\n");
	return 0;
}

static int cmd_net_down(struct cli_instance *sh, int argc, char **argv)
{
	unsigned waited = 0u;

	(void)argc; (void)argv;

	/* Since U4-2 the telnet console IS a socket on this interface, so taking the
	 * interface down from that console cuts the branch it is sitting on -- and the
	 * teardown deliberately refuses to proceed until the console is gone, so it would
	 * deadlock on itself rather than merely disconnect. */
	if (net_shell_guard(sh, "net down"))
		return 1;
	if (nx_net_state() == NX_NET_OFF) {
		cli_print(sh, "net: the host stack is already down\r\n");
		return 0;
	}
	nx_net_down();
	cli_print(sh, "net: taking the host stack down...\r\n");
	while (nx_net_state() != NX_NET_OFF && nx_net_state() != NX_NET_FAILED) {
		if (waited >= NET_NX_ARM_WAIT_MS)
			break;
		cli_sleep(sh, NET_NX_POLL_MS);
		waited += NET_NX_POLL_MS;
	}
	nx_net_print_status(sh);
	if (nx_net_state() != NX_NET_OFF)
		return 1;
	cli_print(sh, "net: the module owns the network again -- `net dhcp` for a lease\r\n");
	return 0;
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

	/* Which stack owns the address decides who is asked for it.  Say so on its own
	 * line: an A/B between the two backends is worthless if the output is ambiguous. */
	cli_print(sh, "stack: %s\r\n",
	          nx_net_is_up() ? "host (NetX Duo, module bridged)"
	                         : "module (lwIP offload)");
	if (nx_net_is_up()) {
		struct nx_net_info ni;

		if (nx_net_info_get(&ni) == NXN_OK && ni.ip_valid)
			net_print_nx_ip(sh, &ni);
		else
			cli_print(sh, "ip:    none (run `net dhcp`)\r\n");
	} else if (connected == WIFI_RPC_OK) {
		if (wifi_rpc_get_ip(&o, 0u, &ip, &result) == 0 && result == WIFI_RPC_OK)
			net_print_ip(sh, &ip);
		else
			cli_print(sh, "ip:    none\r\n");
	}
	rtl_link_end(sh);

	/*
	 * Unconditionally, even when the host stack is OFF.  Its one-line summary carries
	 * WHY it is off -- refused, or taken down by a `wifi reset` -- and until issue #23
	 * U4 that reason only reached `dmesg`, so the question "the console vanished, what
	 * happened?" had no answer on the command that is meant to answer it.
	 * nx_net_print_status() prints the summary first and returns early when OFF.
	 */
	nx_net_print_status(sh);
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
	if (net_nx_settled(sh) != 0)
		return 1;
	if (nx_net_is_up()) {
		struct nx_net_info ni;

		if (nx_net_set_static(a, mask, gw) != NXN_OK) {
			cli_error(sh, "net: NetX refused the static address\r\n");
			return 1;
		}
		cli_print(sh, "net: static address set (host stack)\r\n");
		net_shell_autoarm();
		if (nx_net_info_get(&ni) == NXN_OK)
			net_print_nx_ip(sh, &ni);
		return 0;
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
	if (net_nx_settled(sh) != 0)
		return 1;
	if (nx_net_is_up()) {
		struct nx_net_info ni;
		unsigned waited = 0u;

		if (nx_net_dhcp_renew() != NXN_OK) {
			cli_error(sh, "net: NetX DHCP client would not start\r\n");
			return 1;
		}
		cli_print(sh, "net: requesting DHCP lease with the host stack "
		          "(up to ~30s, Ctrl+C to stop)...\r\n");
		for (;;) {
			if (nx_net_info_get(&ni) == NXN_OK && ni.ip_valid)
				break;
			if (cli_cancel_requested(sh) || waited >= NET_NX_DHCP_WAIT_MS) {
				cli_error(sh, "net: no lease yet\r\n");
				return 1;
			}
			cli_sleep(sh, NET_NX_POLL_MS);
			waited += NET_NX_POLL_MS;
		}
		/* The host stack has an address: this is where the telnet console belongs
		 * since U4-2, and where it is armed from. */
		net_shell_autoarm();
		net_print_nx_ip(sh, &ni);
		return 0;
	}
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
	/* No net_shell_autoarm() on this path: since issue #23 U4-2 the telnet console is a
	 * NetX socket on the HOST stack, so an address the MODULE's lwIP just took is not
	 * one it can listen on. */
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
	sa[1] = (uint8_t)WIFI_LWIP_AF_INET;        /* sin_family */
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

	/* Each probe parks a blocking recvfrom on the module for up to PING_TIMEOUT_MS.  With
	 * the telnet console armed that is the second of the firmware's two workers, so a
	 * console send would have to queue behind BOTH -- long enough for the shell's own TX
	 * deadline (CLI_TX_TIMEOUT) to expire and drop characters.  Refuse instead of
	 * corrupting the other console's output; `net shell stop` frees the worker.
	 * (A device firmware built with more N3_WORKERS would lift this -- see fw/rtl8720/.) */
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

	if (net_nx_settled(sh) != 0)
		return 1;
	if (nx_net_is_up()) {
		/*
		 * A real ICMP echo: built by NetX, its destination resolved by our own ARP
		 * cache, put on the wire by app/nx_link_driver.c and relayed by the module.
		 * The RTT is measured across nx_icmp_ping() alone, so unlike the offload path
		 * below it does not include any eRPC round trip.
		 */
		for (i = 0; i < count; i++) {
			unsigned rtt = 0;
			int nrc = nx_net_ping(dst, PING_TIMEOUT_MS, &rtt);

			if (nrc == NXN_OK) {
				ok++;
				if (rtt < rmin) rmin = rtt;
				if (rtt > rmax) rmax = rtt;
				rsum += rtt;
				cli_print(sh, "  reply %d: %u ms\r\n", i + 1, rtt);
			} else if (nrc == NXN_TIMEOUT) {
				cli_print(sh, "  probe %d: timeout\r\n", i + 1);
			} else {
				cli_error(sh, "  probe %d: NetX error\r\n", i + 1);
			}
			if (i + 1 < count) {
				if (cli_cancel_requested(sh))
					break;
				cli_sleep(sh, 1000u);
			}
		}
		cli_print(sh, "net: %d/%d received", ok, count);
		if (ok)
			cli_print(sh, ", rtt min/avg/max = %u/%u/%u ms", rmin,
			          (unsigned)(rsum / (unsigned)ok), rmax);
		cli_print(sh, "\r\n");
		return ok ? 0 : 1;
	}

	/* Each probe parks a blocking recvfrom on the module for up to PING_TIMEOUT_MS.  With
	 * the telnet console armed that is the second of the firmware's two workers (see the
	 * comment above). */

	if (net_session_connected(sh, &o, &diag) != RTL_LINK_READY)
		return 1;

	o.timeout_ms = 5000u;
	rc = wifi_rpc_lwip_socket(&o, WIFI_LWIP_AF_INET, WIFI_LWIP_SOCK_RAW, WIFI_LWIP_IPPROTO_ICMP, &fd);
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
		rc = wifi_rpc_lwip_setsockopt(&o, fd, WIFI_LWIP_SOL_SOCKET, WIFI_LWIP_SO_RCVTIMEO,
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

	/* Holds a blocking recvfrom open for `block_ms`.  With the telnet console armed that
	 * would be the SECOND long-blocking call on a firmware that has two workers -- both
	 * would be parked and every other RPC (including the console's own output) would queue
	 * behind them. */
	if (nx_net_guard(sh, "net conc"))
		return 1;
	if (argc >= 2) {
		if (parse_uint(argv[1], &block_ms) != 0 || block_ms < 500u || block_ms > 20000u) {
			cli_error(sh, "net: bad ms '%s' (500..20000)\r\n", argv[1]);
			return 1;
		}
	}
	if (net_session_connected(sh, &o, &diag) != RTL_LINK_READY)
		return 1;

	o.timeout_ms = 5000u;
	rc = wifi_rpc_lwip_socket(&o, WIFI_LWIP_AF_INET, WIFI_LWIP_SOCK_RAW, WIFI_LWIP_IPPROTO_ICMP, &fd);
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

/* ---- `net echo`: TCP echo server on the host stack (issue #23 U4-1) ------- */
/*
 * The bring-up rehearsal for the telnet console, as its eRPC ancestor was for issue #21 --
 * one command, one thread, no concurrency of its own, exercising listen / accept / receive
 * / send / disconnect / relisten before a console rides on them.
 *
 * What changed in U4 is what is underneath.  The old version reached the module's lwIP
 * through an eRPC round trip per chunk, so what it measured was dominated by that
 * turnaround and it had to defend against a far end still running a call the host had
 * given up on (the "link dirty" latch, the leaked fds, the ban on aborting accept/recv).
 * None of that exists here: the stack is on this MCU, every wait is bounded and
 * cancellable, and a Ctrl+C stops the server rather than being deferred to the end of a
 * 12-second accept window.
 *
 * The server itself is app/nx_echo.c -- nothing above app/nx_net.h includes nx_api.h.
 */
static int cmd_net_echo(struct cli_instance *sh, int argc, char **argv)
{
	uint32_t port = NX_ECHO_PORT_DEFAULT, v;

	if (net_nx_settled(sh) != 0)
		return 1;
	if (!nx_net_is_up()) {
		cli_error(sh, "net echo: the host stack is not up -- run `net up` first "
		          "(issue #23 U4 moved the echo server onto NetX Duo)\r\n");
		return 1;
	}
	if (argc >= 2) {
		if (parse_uint(argv[1], &v) != 0 || v == 0u || v > 65535u) {
			cli_error(sh, "net: bad port '%s' (1..65535)\r\n", argv[1]);
			return 1;
		}
		port = v;
	}
	return nx_echo_run(sh, (uint16_t)port);
}

/* ---- `net shell`: the telnet console (issue #23 U4-2) --------------------- */
/*
 * The console itself lives in app/net_shell.c; these are just its controls.  Since U4-2 it
 * is a NetX socket on the HOST stack, so it REQUIRES `net up`, and it arms itself when the
 * host stack takes an address (`net dhcp` / `net ip` above call net_shell_autoarm()).
 * `start` is therefore for a different port or after a `stop`.
 *
 * `stop` is no longer about freeing the eRPC UART -- the console does not touch it any
 * more.  It exists because `net down` refuses to unwind the interface while a socket on it
 * is alive, and because a resident console is not always wanted.
 */
#define NET_SHELL_WAIT_MS   10000u
#define NET_SHELL_POLL_MS     100u

static int cmd_net_shell_start(struct cli_instance *sh, int argc, char **argv)
{
	const char *why = NULL;
	uint32_t waited, v;
	uint16_t port = (uint16_t)NET_SHELL_PORT_DEFAULT;

	if (argc >= 2) {
		if (parse_uint(argv[1], &v) != 0 || v == 0u || v > 65535u) {
			cli_error(sh, "net: bad port '%s' (1..65535)\r\n", argv[1]);
			return 1;
		}
		port = (uint16_t)v;
	}
	if (net_shell_start(port, &why) != 0) {
		cli_error(sh, "net shell: cannot start -- %s\r\n", why != NULL ? why : "?");
		net_shell_print_status(sh);
		return 1;
	}
	cli_print(sh, "net shell: arming on port %u...\r\n", (unsigned)port);
	for (waited = 0u; waited < NET_SHELL_WAIT_MS; waited += NET_SHELL_POLL_MS) {
		if (net_shell_state() != NET_SHELL_ARMING)
			break;
		if (cli_sleep(sh, NET_SHELL_POLL_MS))
			break;              /* Ctrl+C: the service carries on regardless */
	}
	net_shell_print_status(sh);
	return net_shell_armed() ? 0 : 1;
}

static int cmd_net_shell_stop(struct cli_instance *sh, int argc, char **argv)
{
	uint32_t waited;

	(void)argc; (void)argv;
	/* Stopping the console we are talking on would cut the wire under our own feet. */
	if (net_shell_guard(sh, "net shell stop"))
		return 1;
	if (net_shell_state() == NET_SHELL_OFF) {
		net_shell_stop();           /* still disables auto-arm */
		cli_print(sh, "net shell: already stopped (auto-arm off)\r\n");
		return 0;
	}
	net_shell_stop();
	cli_print(sh, "net shell: stopping...\r\n");
	for (waited = 0u; waited < NET_SHELL_WAIT_MS; waited += NET_SHELL_POLL_MS) {
		if (net_shell_state() == NET_SHELL_OFF)
			break;
		if (cli_sleep(sh, NET_SHELL_POLL_MS))
			break;
	}
	net_shell_print_status(sh);
	return net_shell_armed() ? 1 : 0;
}

static int cmd_net_shell_status(struct cli_instance *sh, int argc, char **argv)
{
	(void)argc; (void)argv;
	net_shell_print_status(sh);
	return 0;
}

/* ---- registration -------------------------------------------------------- */

CLI_SUBCMD_SET_CREATE(net_shell_subcmds,
	CLI_CMD_ARG(start,  NULL, "arm the telnet console [port] (default 23)",
	            cmd_net_shell_start,  1, 1),
	CLI_CMD_ARG(stop,   NULL, "close it and release the eRPC UART",
	            cmd_net_shell_stop,   1, 0),
	CLI_CMD_ARG(status, NULL, "state / address / counters",
	            cmd_net_shell_status, 1, 0),
	CLI_SUBCMD_SET_END);

CLI_SUBCMD_SET_CREATE(net_subcmds,
	CLI_CMD_ARG(up,   NULL, "bring the host TCP/IP stack up (bridge the module)",
	            cmd_net_up,   1, 0),
	CLI_CMD_ARG(down, NULL, "give the network back to the module",  cmd_net_down, 1, 0),
	CLI_CMD_ARG(info, NULL, "connection + IP / mask / gateway",   cmd_net_info, 1, 0),
	CLI_CMD_ARG(ip,   NULL, "set static <a.b.c.d/mask> [gw]",     cmd_net_ip,   2, 1),
	CLI_CMD_ARG(dhcp, NULL, "(re)acquire an address via DHCP",    cmd_net_dhcp, 1, 0),
	CLI_CMD_ARG(ping, NULL, "raw ICMP echo <a.b.c.d> [count]",    cmd_net_ping, 2, 1),
	CLI_CMD_ARG(conc, NULL, "eRPC server concurrency probe [ms]", cmd_net_conc, 1, 1),
	CLI_CMD_ARG(echo, NULL, "TCP echo server [port] on the host stack (Ctrl+C stops)",
	            cmd_net_echo, 1, 1),
	CLI_CMD_ARG(shell, net_shell_subcmds,
	            "telnet shell console (start [port] / stop / status)", NULL, 1, 0),
	CLI_SUBCMD_SET_END);

CLI_CMD_REGISTER(net, net_subcmds,
                 "IPv4 (L3) over the onboard RTL8720 (info / ip / dhcp / ping / echo / shell)",
                 NULL, 1, 0);
