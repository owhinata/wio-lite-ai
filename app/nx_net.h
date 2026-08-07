/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/*
 * Wio Lite AI (STM32H725AEI6) -- the host's own IP interface (issue #23 U3).
 *
 * Two things live here, because they are the same thing seen from two sides:
 *
 *   1. The control API `net` calls -- info / static address / DHCP / ping.  It is
 *      deliberately the same shape as ../stm32f746g-disco's port/netxduo/nx_glue.h so
 *      that shell/cmds/cmd_net.c reads the same on both boards, and so that nothing
 *      above this file has to include nx_api.h.
 *   2. The resident owner of the bridge session: the thread that holds the RTL8720 link
 *      reference, turns the module into an L2 relay, keeps its watchdog fed, and takes
 *      the whole thing down again provably.
 *
 * ---- what "up" costs, and why there is an owner at all -------------------------
 *
 * The module's bridge is armed with a deadline (issue #23 U2, firmware wio-n7): it takes
 * its own tap out `ms` after the last DATA_CFG, so a host that stops asking cannot leave
 * it forwarding into a link nobody reads.  The cap is 60 s, so a RESIDENT interface has
 * to keep asking -- that is the refresh loop.  And every re-arm resets the module's DATA
 * counters, so the refresh reads them first and accumulates: "was anything lost, and on
 * which side" is the question this whole issue has been decided by, and a session that
 * zeroed its own evidence every 8 s could not answer it.
 *
 * While the tap is in, the module's own lwIP receives nothing, so anything that runs on
 * that lwIP is refused while the host stack is up -- see nx_net_guard().  Issue #23 U4
 * moved the two things that wanted a TCP connection off it entirely: `net echo`
 * (app/nx_echo.c) and the telnet console now open NetX sockets on THIS interface, and
 * therefore REQUIRE the host stack rather than being refused by it.  What is left on the
 * module's lwIP -- `net info`, `net ip`, `net dhcp` -- keeps its
 * eRPC backend, which is also what keeps a regression test against an untouched module
 * firmware alive.
 *
 * ---- the failure rule ----------------------------------------------------------
 *
 * ONE rule governs every unwind, and it comes from app/link_data.h's detach ordering:
 * detaching re-arms the link service thread's stale-byte flush, and if the module can
 * still emit a DATA frame, those "stale" bytes are the middle of one.  So:
 *
 *      once DATA_CFG(BRIDGE) has been issued, only nxn_stop() may end the session,
 *      whatever the answer was.
 *
 * A success means the tap is in.  Module status 5 means "the tcpip thread did not answer;
 * the tap may have gone in after all and the watchdog will take it out" -- which is not
 * "nothing happened".  A -2 timeout means we know nothing at all.  Statuses 1..4 really
 * do prove no callback was installed, but branching on them buys one CTRL exchange and
 * costs a claim that stops being true the next time the firmware changes.
 *
 * If the teardown cannot prove silence, the state becomes FAILED: force-quiesce (which
 * app/link_data.h names as one of the teardowns that take the module down with them, so
 * detaching after it is safe), then say `wifi reset`.
 */
#ifndef APP_NX_NET_H
#define APP_NX_NET_H

#include <stdbool.h>
#include <stdint.h>

struct cli_instance;

/*
 * ---- the transmit budget (issue #23 U4) ---------------------------------------
 *
 * Every frame this stack transmits ends up in the link's DATA transmit pool
 * (LINK_DATA_TX_BUFS, app/link_data.h), and a full pool there is a silent drop -- correct
 * for Ethernet, but a TCP retransmit timeout for anything running a stream on top.  So the
 * pool has to be at least as deep as the most NetX can hand the driver before the link
 * service thread has drained any of it:
 *
 *   LINK_DATA_TX_BUFS >= NXN_TCP_SOCKETS_MAX * NXN_TCP_TX_DEPTH
 *                        + NX_ARP_MAX_QUEUE_DEPTH
 *                        + NXN_TX_SPARE
 *
 * The ARP term is the one that is easy to miss: NetX queues packets per ARP entry while an
 * address is being resolved and flushes the whole list to the driver at once when the reply
 * arrives (_nx_arp_queue_send()), so it is a BURST, not a steady-state occupancy.
 * NXN_TX_SPARE covers the single frames -- an ICMP echo, a DHCP renewal, an ARP request or
 * response -- that can coincide with such a flush.
 *
 * This is an upper-bound ESTIMATE, not a proof: how many packets NetX can pass down in one
 * go depends on the configuration and the path.  It is checked on hardware instead --
 * `net info` reports `nx tx no-buf`, which must stay at zero.  It is deliberately NOT
 * claimed to survive a hostile ICMP flood or arbitrary IP fragmentation; those are
 * legitimate Ethernet drops.
 *
 * The _Static_assert that ties these to LINK_DATA_TX_BUFS lives in app/nx_echo.c, which
 * already includes nx_api.h for NX_ARP_MAX_QUEUE_DEPTH -- putting it in app/link_data.h
 * would make the DATA channel depend on NetX headers.
 */
#define NXN_TCP_TX_DEPTH      4u   /* nx_tcp_socket_transmit_configure max_queue_depth  */
#define NXN_TCP_SOCKETS_MAX   2u   /* telnet console + `net echo` can be live together  */
#define NXN_TX_SPARE          4u   /* room for single ICMP / DHCP / ARP frames          */

/*
 * Initial TCP retransmit timeout, in ThreadX ticks (= ms, NX_IP_PERIODIC_RATE is 1000),
 * and the same value the f746 port uses.
 *
 * A shorter one is tempting -- this link's round trip is 1-5 ms, so 2 s to notice a lost
 * segment is a long time to stare at a console.  U4-1 tried 200 ms and it is NOT a free
 * choice: nx_tcp_socket_transmit_configure() writes nx_tcp_socket_timeout_rate, which NetX
 * also uses as the SYN+ACK retransmit timer during a passive open
 * (nx_tcp_packet_process.c:765-770), so it changes how a handshake behaves and not just
 * how a stall recovers.  Tuning it is therefore an experiment of its own with its own
 * evidence -- issue #23 U4-3, against nx_tcp_socket_info_get()'s retransmit counter --
 * and not something to carry into a first bring-up.
 */
#define NXN_TCP_RTO_MS        (2u * 1000u)
#define NXN_TCP_RTO_RETRIES   10u
#define NXN_TCP_RTO_SHIFT     1u

/* Return codes, mirroring the f746 nx_glue.h. */
#define NXN_OK          0
#define NXN_ERR_STATE  (-1)      /* the host stack is not up                       */
#define NXN_ERR        (-2)      /* NetX refused                                   */
#define NXN_TIMEOUT    (-3)      /* no answer (ping)                               */

enum nx_net_state {
	NX_NET_OFF = 0,          /* the module's own lwIP owns the network          */
	NX_NET_ARMING,           /* bringing the bridge in                          */
	NX_NET_UP,               /* the host stack owns the network                 */
	NX_NET_STOPPING,
	NX_NET_FAILED            /* teardown could not prove silence -- `wifi reset` */
};

struct nx_net_info {
	bool     ip_valid;
	bool     dhcp_mode;
	bool     lease_stale;    /* the link went down while this address was live */
	uint32_t ip;             /* host byte order */
	uint32_t mask;
	uint32_t gw;
};

/* Module-side counters accumulated across bridge re-arms (each re-arm zeroes the
 * module's own), in the order LINK_DATA_STATS reports them. */
struct nx_net_modstats {
	uint32_t rx_frames, rx_bytes, rx_drops, rx_crc, rx_oversize, rx_gaps;
	uint32_t tx_frames, tx_bytes, tx_drops;
};

/*
 * Create every ThreadX/NetX object and the owner thread.  Object creation only -- no
 * blocking and no HAL_GetTick dependency -- so it is safe from tx_application_define()
 * (issue #12).  The IP instance exists from boot with address 0.0.0.0, no MAC and the
 * link down; nothing touches the RTL8720 until nx_net_up().
 */
int  nx_net_init(void);

/* Is the host stack the one that owns the network right now? */
bool nx_net_is_up(void);
enum nx_net_state nx_net_state(void);

/* "off" / "arming" / "up" / "stopping" / "FAILED" -- so a refusal can name the state it
 * was refused in rather than just saying no. */
const char *nx_net_state_name(enum nx_net_state s);

/* Ask the owner thread to bring the interface up / take it down.  nx_net_up() returns 0
 * once the request is accepted (state becomes ARMING before it returns, so guards close
 * immediately); *why explains a refusal.  Poll nx_net_state() for the outcome. */
int  nx_net_up(const char **why);
void nx_net_down(void);

/*
 * "The link has been taken away and CHIP_EN is about to move" -- the epilogue of
 * rtl_link_force_quiesce() for `wifi on/off/reset` (issue #41).
 *
 * The owner detects a revoked link by itself (the uart generation moved), but only when
 * it next wakes, which is up to NXN_REFRESH_MS later.  This hands it the news now and
 * waits, briefly, for it to leave NX_NET_UP -- so the very next command does not find a
 * stale UP and try to renew a bridge on a module that has just been power-cycled.
 *
 * NOT nx_net_down(): that is the graceful stop, which needs the coarse mutex and speaks
 * eRPC to the module, neither of which a recovery command can afford.  See the body.
 *
 * Call AFTER rtl_link_force_quiesce() and BEFORE moving CHIP_EN.  Returns 0 once the
 * interface is no longer up (already-down counts), -1 if it has not stood down in time --
 * which the caller reports and then carries on with, rather than refusing.
 */
int  nx_net_link_taken(void);

int  nx_net_info_get(struct nx_net_info *out);
int  nx_net_set_static(uint32_t ip, uint32_t mask, uint32_t gw);
int  nx_net_dhcp_renew(void);
int  nx_net_ping(uint32_t ip, unsigned timeout_ms, unsigned *rtt_ms);

/* The MAC the interface transmits with (the module's), valid once up. */
void nx_net_mac_get(uint8_t mac[6]);
void nx_net_modstats_get(struct nx_net_modstats *out);

/*
 * The NetX objects, for the things that put a socket on this interface (app/nx_echo.c,
 * app/net_shell.c).  Typed as void * so that a caller which only needs to pass them along
 * does not have to include nx_api.h; cast to NX_IP * / NX_PACKET_POOL *.
 *
 * NULL until nx_net_init() has succeeded.  Both are valid from then on regardless of the
 * session state -- the IP instance exists from boot with address 0.0.0.0 and the link
 * down -- so a caller still has to ask nx_net_is_up() before expecting traffic to move.
 *
 * (The f746 nx_glue.h has the same pair.  U3 deliberately did not port them because
 * nothing on this board created a socket yet; U4 does.)
 */
void *nx_net_ip(void);
void *nx_net_pool(void);

/* Print the host-stack section of `net info` / the result of arming the interface. */
void nx_net_print_status(struct cli_instance *sh);

/*
 * Refuse a command that would reconfigure the module underneath the interface.
 * Returns 0 when it may proceed, non-zero after printing why.
 *
 * It refuses in EVERY state except OFF, not just UP.  ARM releases the coarse mutex
 * before it declares itself up, and rtl_link_uart_gen() does not advance for ordinary
 * eRPC flows (only a 0->1 open and a force-quiesce), so the owner's generation watch
 * cannot see a `wifi connect` land in that window.  nx_net_up() therefore publishes
 * ARMING on the caller's thread, before the owner is even signalled.
 *
 * `wifi on` / `off` / `reset` must NOT use this: they are the recovery path, they
 * force-quiesce, and the generation watch turns the interface down cleanly afterwards.
 * Refusing them would take away the only way out of FAILED.
 */
int nx_net_guard(struct cli_instance *sh, const char *what);

/*
 * Keep the module's bridge alive across a long eRPC flow (issue #30 B2b).  Caller must
 * hold the rtl_link coarse mutex.  Returns 0 (nothing to hold, or held) / -1 (the module
 * did not answer -- do NOT start the flow).  See the definition for why.
 */
int nx_net_hold_extend(void);

#endif /* APP_NX_NET_H */
