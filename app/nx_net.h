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
 * While the tap is in, the module's own lwIP receives nothing.  `net ping`, `net echo`
 * and the telnet console all run on that lwIP, so they are refused while the host stack
 * is up -- see nx_net_guard().
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

/* Ask the owner thread to bring the interface up / take it down.  nx_net_up() returns 0
 * once the request is accepted (state becomes ARMING before it returns, so guards close
 * immediately); *why explains a refusal.  Poll nx_net_state() for the outcome. */
int  nx_net_up(const char **why);
void nx_net_down(void);

int  nx_net_info_get(struct nx_net_info *out);
int  nx_net_set_static(uint32_t ip, uint32_t mask, uint32_t gw);
int  nx_net_dhcp_renew(void);
int  nx_net_ping(uint32_t ip, unsigned timeout_ms, unsigned *rtt_ms);

/* The MAC the interface transmits with (the module's), valid once up. */
void nx_net_mac_get(uint8_t mac[6]);
void nx_net_modstats_get(struct nx_net_modstats *out);

/* Print the host-stack section of `net info` / the result of `net up`. */
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

#endif /* APP_NX_NET_H */
