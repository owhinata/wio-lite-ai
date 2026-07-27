/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    cmd_net.c
 * @brief   `net` shell command (issues #5/#23): IPv4 (L3) over the onboard RTL8720DN.
 *
 *   net up | down                bring the HOST TCP/IP stack (NetX Duo) up over the
 *                                L2 bridge / give the network back to the module
 *   net info                     association + MAC + IP/mask/gw
 *   net ip <a.b.c.d/mask> [gw]   set a static address
 *   net dhcp                     (re)acquire an address via DHCP
 *   net ping <a.b.c.d> [count]   ICMP echo from the host stack
 *   net echo [port]              TCP echo server on the host stack (issue #23 U4-1)
 *   net shell start|stop|status  the telnet console (a NetX socket, issue #23 U4-2)
 *
 * This is the Wio counterpart of ../stm32f746g-disco's `net` command.  There the backend
 * is NetX Duo over the on-chip Ethernet MAC; here it is NetX Duo over the issue-#23 L2
 * bridge, which is the SINGLE L3 backend since issue #30 B1 -- everything below `net up`
 * is the host's own stack.  The module's lwIP still exists (the bridge is a tap on its
 * netif, so it has to), but nothing here drives it any more.  The L2 side (power +
 * association) stays in `wifi`, and association is a prerequisite for everything here.
 *
 * Only `net info` still speaks eRPC, and only to the WiFi DRIVER (service 14) for
 * association and MAC -- the tap does not touch those.  It runs inside one
 * rtl_link_begin() .. rtl_link_end() session (shared with `wifi`, see app/rtl_link.h):
 * that claims the console, takes the coarse link mutex and references the eRPC UART.
 * Every other subcommand talks to NetX only.
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
#include "nx_link_driver.h" /* link state for the `net ping` precondition (issue #30 B2a) */

#include "stm32h7xx_hal.h"   /* HAL_GetTick (1 ms SysTick, fed via tx_glue.c) */

#include <stddef.h>
#include <stdint.h>
#include <string.h>


#define PING_TIMEOUT_MS    1000u     /* per-probe reply wait */

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

/* ---- eRPC option helper -------------------------------------------------- */

/*
 * `net info` still asks the module's WiFi DRIVER for association / MAC (service 14,
 * which the bridge does not touch).  Everything L3 moved to the host stack in issue
 * #30 B1, so this is the only eRPC left in this file.
 */
static void net_opts(struct wifi_rpc_opts *o, struct cli_instance *sh,
                     struct erpc_diag *diag, uint32_t timeout_ms)
{
	o->timeout_ms   = timeout_ms;
	o->should_abort = rtl_abort_cb;
	o->abort_ctx    = sh;
	o->diag         = diag;
}

/* ---- host stack (issue #23 U3) ------------------------------------------- */

/*
 * `net up` installs the L2 bridge and brings NetX Duo up on it (app/nx_net.c); `net down`
 * gives the module its network back.  Since issue #30 B1 there is no second backend to
 * switch to, so everything with an address in it simply requires the stack to be up and
 * says so -- one predicate, nx_net_is_up(), and no subcommand can disagree with its
 * neighbour about who answered.
 *
 * (Issue #30 B2 will make the arm implicit in `wifi connect`, at which point these two
 * disappear as user commands.)
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
	cli_print(sh, "gw:    %u.%u.%u.%u (%s)%s\r\n",
	          (unsigned)((ni->gw >> 24) & 0xFFu), (unsigned)((ni->gw >> 16) & 0xFFu),
	          (unsigned)((ni->gw >> 8) & 0xFFu),  (unsigned)(ni->gw & 0xFFu),
	          ni->dhcp_mode ? "dhcp" : "static",
	          /* The link dropped while this address was live and nothing renews it
	           * automatically (issue #30 B2a) -- say so where the address is shown,
	           * because everything above it still LOOKS configured. */
	          ni->lease_stale ? "  <-- STALE (link went down; run `net dhcp`)" : "");
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

	/* Association, MAC and RSSI come from the module's WiFi DRIVER (service 14), which
	 * the bridge does not touch -- so they read the same whether the host stack is up or
	 * not.  The ADDRESS is the host stack's alone since issue #30 B1. */
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
	rtl_link_end(sh);

	if (nx_net_is_up()) {
		struct nx_net_info ni;

		if (nx_net_info_get(&ni) == NXN_OK && ni.ip_valid)
			net_print_nx_ip(sh, &ni);
		else
			cli_print(sh, "ip:    none (run `net dhcp`)\r\n");
	} else {
		cli_print(sh, "ip:    none (the host stack is down -- run `net up`)\r\n");
	}

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
	struct nx_net_info ni;
	uint32_t a, mask, gw = 0;

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
	if (!nx_net_is_up()) {
		cli_error(sh, "net: the host stack is down -- run `net up` first\r\n");
		return 1;
	}
	if (nx_net_set_static(a, mask, gw) != NXN_OK) {
		cli_error(sh, "net: NetX refused the static address\r\n");
		return 1;
	}
	cli_print(sh, "net: static address set\r\n");
	net_shell_autoarm();
	if (nx_net_info_get(&ni) == NXN_OK)
		net_print_nx_ip(sh, &ni);
	return 0;
}

static int cmd_net_dhcp(struct cli_instance *sh, int argc, char **argv)
{
	struct nx_net_info ni;
	unsigned waited = 0u;

	(void)argc; (void)argv;
	if (net_nx_settled(sh) != 0)
		return 1;
	if (!nx_net_is_up()) {
		cli_error(sh, "net: the host stack is down -- run `net up` first\r\n");
		return 1;
	}
	if (nx_net_dhcp_renew() != NXN_OK) {
		cli_error(sh, "net: NetX DHCP client would not start\r\n");
		return 1;
	}
	cli_print(sh, "net: requesting DHCP lease (up to ~30s, Ctrl+C to stop)...\r\n");
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
	/* The host stack has an address: this is where the telnet console belongs since
	 * U4-2, and where it is armed from. */
	net_shell_autoarm();
	net_print_nx_ip(sh, &ni);
	return 0;
}

static int cmd_net_ping(struct cli_instance *sh, int argc, char **argv)
{
	uint32_t dst, c;
	int count = 4, ok = 0, i;
	unsigned rmin = 0xFFFFFFFFu, rmax = 0, rsum = 0;

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
		/* Say WHY rather than letting NetX refuse every probe in turn: with the link
		 * down (issue #30 B2a) the interface still has an address and looks configured,
		 * so four "NetX error" lines are the least informative possible answer. */
		if (!nx_link_driver_link_up()) {
			cli_error(sh, "net: the link is down (not associated) -- "
			          "`wifi connect` first\r\n");
			return 1;
		}
		/*
		 * A real ICMP echo: built by NetX, its destination resolved by our own ARP
		 * cache, put on the wire by app/nx_link_driver.c and relayed by the module.
		 * The RTT is measured across nx_icmp_ping() alone -- no eRPC round trip.
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

	/* The module-side raw-ICMP path was removed by issue #28: the surviving module
	 * backend is info/ip/dhcp only, and ping belongs to the host stack. */
	cli_error(sh, "net: the host stack is down -- run `net up` first\r\n");
	return 1;
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
	CLI_CMD_ARG(stop,   NULL, "close the console + disable auto-arm (frees `net down`)",
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
	CLI_CMD_ARG(ping, NULL, "ICMP echo <a.b.c.d> [count] (host stack -- `net up` first)",
	            cmd_net_ping, 2, 1),
	CLI_CMD_ARG(echo, NULL, "TCP echo server [port] on the host stack (Ctrl+C stops)",
	            cmd_net_echo, 1, 1),
	CLI_CMD_ARG(shell, net_shell_subcmds,
	            "telnet shell console (start [port] / stop / status)", NULL, 1, 0),
	CLI_SUBCMD_SET_END);

CLI_CMD_REGISTER(net, net_subcmds,
                 "IPv4 (L3) over the onboard RTL8720 (up / down / info / ip / dhcp / "
                 "ping / echo / shell)",
                 NULL, 1, 0);
