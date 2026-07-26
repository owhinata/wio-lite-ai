/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/*
 * The host's own IP interface over the RTL8720 L2 bridge (issue #23 U3).
 * See app/nx_net.h for the design, the failure rule and the threading.
 */
#include "nx_net.h"

#include <string.h>

#include "tx_api.h"
#include "stm32h7xx_hal.h"   /* __get_PRIMASK / __disable_irq / __set_PRIMASK */
#include "nx_api.h"
#include "nxd_dhcp_client.h"

#include "cli.h"
#include "cli_instance.h"
#include "erpc.h"
#include "link_data.h"
#define LOG_TAG "nx"
#include "log.h"
#include "net_shell.h"
#include "nx_link_driver.h"
#include "rtl_link.h"
#include "rtl8720.h"
#include "wifi_rpc.h"

/* ---- tunables --------------------------------------------------------------- */

/*
 * Packet pool.  1600 leaves room above a 1514-byte frame for the 2-byte receive pad and
 * the NX_PHYSICAL_HEADER/TRAILER reservation.
 *
 * 48 kB is about 29 packets (U3 ran 32 kB / 19, which was sized for ARP, DHCP and ICMP
 * alone).  U4 puts TCP sockets on this interface, and a socket holds packets that the pool
 * cannot reclaim until they are acknowledged -- up to NXN_TCP_TX_DEPTH per socket, for as
 * many as NXN_TCP_SOCKETS_MAX of them -- on top of whatever is in flight on the receive
 * side.  Running the pool dry does not merely slow things down: nx_link_driver_rx()
 * allocates with NX_NO_WAIT, so an empty pool is a dropped frame and, for a stream, a
 * retransmit timeout.  `net info` reports the low-water mark; issue #23 U4-3 settles the
 * final number from it rather than from this estimate.
 */
#define NXN_PAYLOAD        1600u
#define NXN_POOL_BYTES     (48u * 1024u)

#define NXN_IP_PRIORITY    11u        /* just below the link service thread (10)     */
#define NXN_IP_STACK       2048u
#define NXN_ARP_CACHE      1040u      /* ~20 entries                                 */

#define NXN_OWNER_PRIORITY 13u
#define NXN_OWNER_STACK    2048u

/*
 * The module takes its own tap out this long after the last DATA_CFG (cap 60 s,
 * fw/rtl8720 WIO_ETH_HOLD_MAX_MS), so the interface re-arms well inside it.  A refresh
 * that cannot get the coarse mutex is skipped, not failed, and 30 s absorbs about three
 * of those.
 */
#define NXN_HOLD_MS        30000u
#define NXN_REFRESH_MS     8000u

#define NXN_CTRL_TMO_MS    500u
#define NXN_DCFG_TMO_MS    2500u      /* CFG(off) delays its ack until the module's
                                       * DATA writer is idle -- as `link` does        */
#define NXN_RPC_TMO_MS     5000u
#define NXN_CLAIM_MS       RTL_LINK_CLAIM_WAIT_MS
#define NXN_ARM_CLAIM_MS   20000u     /* how long ARM waits for the coarse mutex      */

#define NXN_OFF_RETRIES    3
#define NXN_SETTLE_TRIES   50         /* x 20 ms = 1 s, as link_data_settle()         */
#define NXN_REFRESH_FAILS  2          /* consecutive CFG failures before STOP         */

#define NXN_DSTAT_WORDS    12u        /* LINK_DATA_STATS reply                        */
#define NXN_ETH_REPLY_MIN  (8u + ERPC_ETH_STAT_WORDS * 4u)

#define NXN_EVT_CMD        0x1u

/* Firmware generation that first has the L2 bridge (2.1.3+wio-n7). */
#define NXN_MIN_MODULE_GEN 7u

/* ---- state ------------------------------------------------------------------ */

/*
 * The pool lives in DTCM (ldscript .nx_pool).  The only bus master that ever touches a
 * payload is the CPU -- frames are memcpy'd to and from the link's DATA pool, there is no
 * DMA in this firmware at all -- so DTCM is both safe and the better choice: zero wait
 * states, no D-cache lines spent on 1.5 kB frames, and none of the AXI-SRAM budget.
 */
static UCHAR nxn_pool_mem[NXN_POOL_BYTES]
	__attribute__((aligned(32), section(".nx_pool")));

static NX_PACKET_POOL nxn_pool;
static NX_IP          nxn_ip;
static NX_DHCP        nxn_dhcp;
static ULONG          nxn_ip_stack[NXN_IP_STACK / sizeof(ULONG)];
static ULONG          nxn_arp_cache[NXN_ARP_CACHE / sizeof(ULONG)];

static bool nxn_ready;                /* the NetX objects exist                      */
static bool nxn_dhcp_created;

/*
 * Address-acquisition state, and the mutex that owns it.
 *
 * These are touched from two directions: a CLI thread running `net dhcp` / `net ip`, and
 * the owner thread halting the client as it takes the interface down.  Without
 * serialisation `net dhcp &` (a background job) racing a foreground `net down` can leave
 * them lying: the owner reads nxn_dhcp_started == false, skips the stop, and the job then
 * sets it true after the halt point -- a client left running under an interface that is
 * on its way out.
 *
 * LOCK ORDER: this mutex is NEVER taken while the rtl_link coarse mutex is held.
 * nxn_dhcp_halt() runs before nxn_stop() claims the link, and the CLI entry points below
 * take no link at all.  Below it sit the NetX DHCP mutex and nx_ip_protection, which is
 * the same direction everything else in this file goes.
 */
static TX_MUTEX nxn_addr_lock;
static bool nxn_dhcp_started;
static bool nxn_static_mode;

static void nxn_addr_take(void)
{
	if (nxn_ready)
		(void)tx_mutex_get(&nxn_addr_lock, TX_WAIT_FOREVER);
}

static void nxn_addr_give(void)
{
	if (nxn_ready)
		(void)tx_mutex_put(&nxn_addr_lock);
}

/* Owner thread. */
static TX_THREAD             nxn_thread;
static TX_EVENT_FLAGS_GROUP  nxn_evt;
static UCHAR                 nxn_stack[NXN_OWNER_STACK] __attribute__((aligned(8)));

/*
 * State is written by the owner thread and, for the ARMING/STOPPING transitions, by the
 * requesting CLI thread -- deliberately, so that nx_net_guard() closes the moment the
 * command is typed rather than when the owner gets around to it.
 */
static volatile enum nx_net_state nxn_state = NX_NET_OFF;
static volatile uint8_t  nxn_req_stop;

/* Owner-thread-private. */
static uint32_t nxn_uart_gen;
static uint8_t  nxn_mac[6];
static uint8_t  nxn_radio_up;
static uint8_t  nxn_reply[ERPC_CTRL_MAX];
static struct nx_net_modstats nxn_mod;
static struct wifi_ip_info    nxn_saved_ip;   /* the module's address before we took it */

/* Why the interface last went down or refused, for `net info`. */
static const char *nxn_why = "";

/* The dirty latch behind FAILED: cleared when the module is actually power-cycled. */
static uint32_t nxn_failed_at;

/*
 * Consecutive failed bridge re-arms, SESSION state and not a static local inside the
 * refresh: a failure carried over from a previous session would make the next one give
 * up after a single bad refresh instead of two consecutive ones.  Reset wherever a
 * session begins or ends.
 */
static int nxn_refresh_fails;

/* ---- small helpers ---------------------------------------------------------- */

static uint32_t nxn_u32le(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
	       ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void nxn_put_u32le(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)v;
	p[1] = (uint8_t)(v >> 8);
	p[2] = (uint8_t)(v >> 16);
	p[3] = (uint8_t)(v >> 24);
}

static void nxn_opts(struct wifi_rpc_opts *o, uint32_t ms)
{
	memset(o, 0, sizeof *o);
	o->timeout_ms = ms;
	/* No abort hook: this is not running on a console thread, so there is nobody to
	 * press Ctrl+C, and an aborted request would leave the link dirty. */
}

/* ---- CTRL exchanges (owner thread, coarse mutex held) ----------------------- */

static int nxn_data_cfg(uint8_t mode, uint32_t ms)
{
	struct erpc_diag diag = {0};
	uint8_t req[12];
	int n;

	req[0] = mode;
	req[1] = 0u;
	req[2] = 0u;
	req[3] = 0u;
	nxn_put_u32le(req + 4, ms);
	nxn_put_u32le(req + 8, ERPC_CTRL_DATA_MAGIC);
	n = erpc_ctrl_call(ERPC_CTRL_DATA_CFG, req, (uint16_t)sizeof req, nxn_reply,
	                   (uint16_t)sizeof nxn_reply, NXN_DCFG_TMO_MS, &diag);
	if (n < 0) {
		LOG_ERR("DATA_CFG(%u) rc %d status %u", (unsigned)mode, n,
		        (unsigned)((n == -3) ? erpc_ctrl_last_status() : 0u));
		return -1;
	}
	return 0;
}

/* Read the module's DATA counters into @st.  Returns 0 on success. */
static int nxn_data_stats(uint32_t st[NXN_DSTAT_WORDS])
{
	struct erpc_diag diag = {0};
	unsigned i;
	int n = erpc_ctrl_call(ERPC_CTRL_DATA_STATS, NULL, 0u, nxn_reply,
	                       (uint16_t)sizeof nxn_reply, NXN_CTRL_TMO_MS, &diag);

	if (n < (int)(NXN_DSTAT_WORDS * 4u))
		return -1;
	for (i = 0u; i < NXN_DSTAT_WORDS; i++)
		st[i] = nxn_u32le(nxn_reply + i * 4u);
	return 0;
}

/*
 * Fold the module's DATA counters into our running totals.  Every non-OFF DATA_CFG zeroes
 * them on the module, so this must happen BEFORE each re-arm or a long-lived interface
 * would keep destroying its own loss ledger.  Words 9/10 are queue depth and pool
 * in-use -- instantaneous, not cumulative, so they are not accumulated here.
 */
static void nxn_mod_accumulate(const uint32_t st[NXN_DSTAT_WORDS])
{
	/*
	 * Interrupts masked for the whole update, and likewise for the read in
	 * nx_net_modstats_get(), so a reader can never catch the ledger half-folded.
	 * Individual aligned words could not tear on this core anyway -- what this buys is
	 * that the NINE of them are consistent WITH EACH OTHER, which is the only reason
	 * anyone reads them: "frames vs drops" is a ratio, and a snapshot spliced from two
	 * different moments quietly lies about it.  A few dozen cycles, once per refresh.
	 */
	uint32_t pm = __get_PRIMASK();

	__disable_irq();
	nxn_mod.rx_frames   += st[0];
	nxn_mod.rx_bytes    += st[1];
	nxn_mod.rx_drops    += st[2];
	nxn_mod.rx_crc      += st[3];
	nxn_mod.rx_oversize += st[4];
	nxn_mod.rx_gaps     += st[5];
	nxn_mod.tx_frames   += st[6];
	nxn_mod.tx_bytes    += st[7];
	nxn_mod.tx_drops    += st[8];
	__set_PRIMASK(pm);
}

static int nxn_eth_info(uint8_t mac[6], uint8_t *flags)
{
	struct erpc_diag diag = {0};
	int n = erpc_ctrl_call(ERPC_CTRL_ETH_INFO, NULL, 0u, nxn_reply,
	                       (uint16_t)sizeof nxn_reply, NXN_CTRL_TMO_MS, &diag);

	if (n < (int)NXN_ETH_REPLY_MIN)
		return -1;
	memcpy(mac, nxn_reply, 6);
	*flags = nxn_reply[6];
	return 0;
}

/* ---- NetX callbacks ---------------------------------------------------------- */

static void nxn_dhcp_state_cb(NX_DHCP *dhcp, UCHAR new_state)
{
	(void)dhcp;
	/* 2=INIT 3=SELECTING(discover sent) 4=REQUESTING(offer seen) 5=BOUND.  A stall at
	 * SELECTING is "no OFFER came back", which on this board means the bridge. */
	LOG_INF("dhcp state -> %u", (unsigned)new_state);
}

/*
 * IP helper thread, under nx_ip_protection.  NetX only calls this because
 * nx_ip_link_status_change_notify_set() registered it -- without a callback,
 * _nx_ip_deferred_link_status_process() returns without even asking the driver, so the
 * interface flag would never move.
 *
 * It updates one field and nothing else.  The f746 port starts the DHCP client from here
 * because DHCP is its boot default; doing that on this board would be wrong twice over:
 * NetX runs this callback WITH nx_ip_protection held (nx_ip_thread_entry.c), while the
 * DHCP client takes its own mutex and then sends packets that take nx_ip_protection --
 * so an IP->DHCP edge here plus the DHCP->IP edge that already exists is a lock-order
 * cycle.  And `net dhcp` can start the client from a CLI thread at the same moment.
 * There is therefore exactly ONE owner of the DHCP client: nx_net_dhcp_renew(), i.e. the
 * `net dhcp` command, which is also what `net up` tells the user to run.
 */
static void nxn_link_status_cb(NX_IP *ip, UINT iface_index, UINT link_up)
{
	ip->nx_ip_interface[iface_index].nx_interface_link_up = (UCHAR)link_up;
}

/* ---- the owner thread -------------------------------------------------------- */

static void nxn_dhcp_halt(void)
{
	nxn_addr_take();
	if (nxn_dhcp_started) {
		nx_dhcp_stop(&nxn_dhcp);
		nxn_dhcp_started = false;
	}
	nxn_addr_give();
}

/*
 * Wait until BOTH ends are quiet.  This is app/link_data.h's detach ordering rule, and it
 * is a state test rather than a delay because there is no defensible number of
 * milliseconds to wait: the module's side is proved by the CFG(off) acknowledgement plus
 * its own queue and pool counters, the host's by erpc_data_quiescent().
 *
 * erpc_data_quiescent(), NOT erpc_link_quiescent(): the stronger question additionally
 * requires that no eRPC request be outstanding, and a telnet console keeps a blocking
 * accept outstanding by design, so it could never succeed with one armed (issue #23 U1).
 */
static int nxn_settle(void)
{
	uint32_t st[NXN_DSTAT_WORDS];
	int tries;

	for (tries = 0; tries < NXN_SETTLE_TRIES; tries++) {
		if (erpc_data_quiescent() && nxn_data_stats(st) == 0 &&
		    st[9] == 0u && st[10] == 0u) {
			nxn_mod_accumulate(st);
			return 0;
		}
		tx_thread_sleep(20u);
	}
	return -1;
}

/*
 * The teardown could not prove the module is silent.  Do NOT detach yet: instead take the
 * host's byte stream away entirely.  app/link_data.h names force-quiesce as one of the
 * teardowns "that takes the module down with it, where nothing survives to be
 * desynchronised" -- it drops the reference count to zero, runs erpc_link_closed() (which
 * reclaims every DATA buffer) and closes the UART, so after it there is no stream left to
 * lose synchronisation with.
 *
 * It does NOT power-cycle the module, so the tap may stay in until the module's own
 * watchdog fires (up to NXN_HOLD_MS).  `wifi reset` ends it immediately, which is why
 * that is what the message says and why `wifi on/off/reset` are never guarded.
 *
 * Called with the coarse mutex HELD; force-quiesce requires that and does not release it.
 */
static void nxn_enter_failed(const char *why)
{
	rtl_link_force_quiesce();
	link_data_detach();
	nxn_uart_gen = 0u;
	nxn_failed_at = rtl_link_quiesce_gen();
	nxn_why = why;
	nxn_state = NX_NET_FAILED;
	LOG_ERR("%s -- the host stack could not be shut down cleanly; run `wifi reset`",
	        why);
}

/*
 * End the session.  Every exit passes through exactly one rtl_link_unclaim(), tracked by
 * @claimed rather than by remembering to write it on each branch.
 */
static void nxn_stop(const char *why)
{
	int claimed = 0;
	int tries;

	nxn_state = NX_NET_STOPPING;
	nxn_refresh_fails = 0;

	/* Stop producing on our side first. */
	nx_link_driver_set_link(0);
	nxn_dhcp_halt();

	if (rtl_link_claim(NXN_ARM_CLAIM_MS) != 0) {
		/* Nobody gave up the mutex in twenty seconds.  We cannot even ask the module
		 * to stop, let alone prove it did.  There is no claim-free way to reach
		 * force-quiesce safely, so say so and leave the tap to the watchdog. */
		nxn_why = "the link stayed busy";
		nxn_failed_at = rtl_link_quiesce_gen();
		nxn_state = NX_NET_FAILED;
		LOG_ERR("could not take the link to shut down; run `wifi reset`");
		return;
	}
	claimed = 1;

	for (tries = 0; tries < NXN_OFF_RETRIES; tries++) {
		if (nxn_data_cfg(ERPC_DATA_MODE_OFF, 0u) == 0)
			break;
	}
	if (tries == NXN_OFF_RETRIES) {
		nxn_enter_failed("the module never acknowledged DATA_CFG(off)");
		goto out;
	}

	if (nxn_settle() != 0) {
		nxn_enter_failed("the DATA channel did not go quiet");
		goto out;
	}

	link_data_detach();
	rtl_link_uart_unref_gen(nxn_uart_gen);
	nxn_uart_gen = 0u;
	nxn_why = why;
	nxn_state = NX_NET_OFF;
	LOG_INF("host stack down (%s)", why);

out:
	if (claimed)
		rtl_link_unclaim();
}

/*
 * Bring the interface up.  The prologue order is `link eth`'s (shell/cmds/cmd_link.c) and
 * every step is in that position for a reason recorded there.
 */
static void nxn_arm(void)
{
	struct wifi_rpc_opts o;
	uint32_t waited = 0u;
	int32_t result = 0;
	uint8_t flags = 0u;
	int claimed = 0;
	int armed = 0;                   /* DATA_CFG(BRIDGE) has been ISSUED */
	int attached = 0;

	nxn_refresh_fails = 0;

	/* While the tap is in, the module's lwIP receives nothing -- and an armed telnet
	 * console is sitting on an accept that would then never complete. */
	if (net_shell_state() != NET_SHELL_OFF) {
		nxn_why = "the telnet console is armed (run `net shell stop` first)";
		goto refuse;
	}
	if (!rtl8720_powered()) {
		nxn_why = "the RTL8720 is powered off";
		goto refuse;
	}

	while (rtl_link_claim(NXN_CLAIM_MS) != 0) {
		waited += NXN_CLAIM_MS;
		if (nxn_req_stop || waited >= NXN_ARM_CLAIM_MS) {
			nxn_why = "the link stayed busy";
			goto refuse;
		}
	}
	claimed = 1;

	if (rtl_link_uart_ref(RTL8720_UART_AT, rtl_link_erpc_baud()) != 0) {
		nxn_why = "the link UART is held at a different rate";
		goto refuse;
	}
	nxn_uart_gen = rtl_link_uart_gen();

	if (erpc_module_gen() < NXN_MIN_MODULE_GEN) {
		nxn_why = "the module firmware has no L2 bridge (needs 2.1.3+wio-n7; "
		          "run `wifi rpc ver`)";
		goto unref;
	}

	nxn_opts(&o, NXN_RPC_TMO_MS);
	/* The module answers WIFI_RPC_OK (which is 0) for "associated" -- a result code, not
	 * a boolean.  Testing it for truth gets the answer exactly backwards, which is what
	 * the first board run did: `wifi status` said connected and this refused. */
	if (wifi_rpc_is_connected(&o, &result) != 0 || result != WIFI_RPC_OK) {
		nxn_why = "the module is not associated (run `wifi connect <ssid> ...`)";
		goto unref;
	}

	/* Last point at which giving up is free: nothing has been told to the module and
	 * the DATA channel has no consumer yet. */
	if (nxn_req_stop) {
		nxn_req_stop = 0u;
		nxn_why = "cancelled";
		goto unref;
	}
	/*
	 * Read the module's address BEFORE stopping DHCP and before the bridge zeroes the
	 * netif: it is the state we restore the user's expectations to afterwards, and once
	 * the tap is in it is gone.
	 */
	memset(&nxn_saved_ip, 0, sizeof nxn_saved_ip);
	(void)wifi_rpc_get_ip(&o, 0u, &nxn_saved_ip, &result);

	/*
	 * Stop the module's DHCP client.  A TRANSPORT failure refuses the whole session --
	 * "we do not know whether it stopped" is not good enough when the bridge is about
	 * to zero the netif address underneath it.  A non-zero module RESULT is different:
	 * it was executed, and "it was not running" is the state we want.
	 */
	if (wifi_rpc_dhcpc_stop(&o, 0u, &result) != 0) {
		nxn_why = "could not stop the module's DHCP client";
		goto unref;
	}

	/* Attach BEFORE the bridge is enabled, so no frame can arrive with no consumer. */
	nx_link_driver_reset_stats();
	memset(&nxn_mod, 0, sizeof nxn_mod);
	if (link_data_attach(nx_link_driver_rx, NULL) != 0) {
		nxn_why = "the link's DATA channel already has an owner "
		          "(a `link` command is running)";
		goto unref;
	}
	attached = 1;

	if (nxn_req_stop) {
		/* Still free: attached, but the module has not been asked to bridge, so the
		 * detach in `unref:` cannot desynchronise anything. */
		nxn_req_stop = 0u;
		nxn_why = "cancelled";
		goto unref;
	}

	armed = 1;                       /* from here on, only nxn_stop() may end this */
	if (nxn_data_cfg(ERPC_DATA_MODE_BRIDGE, NXN_HOLD_MS) != 0) {
		nxn_why = "the module refused to bridge (see the log; `wifi connect` first?)";
		goto stop;
	}

	if (nxn_eth_info(nxn_mac, &flags) != 0) {
		nxn_why = "the module did not report its MAC";
		goto stop;
	}
	nxn_radio_up = (flags & ERPC_ETH_F_RUNNING) ? 1u : 0u;
	if (!nxn_radio_up) {
		nxn_why = "the module's radio is down";
		goto stop;
	}

	rtl_link_unclaim();
	claimed = 0;

	/*
	 * The NetX calls go here, AFTER the coarse mutex is released: they take
	 * nx_ip_protection, and the owner must never block on a NetX mutex while holding
	 * the lock every shell command needs.
	 */
	if (nx_link_driver_set_mac(nxn_mac) != 0) {
		nxn_why = "NetX refused the module's MAC";
		goto stop;
	}
	nx_link_driver_set_speed(rtl_link_erpc_baud());
	nx_link_driver_set_link(1);

	nxn_why = "";
	nxn_state = NX_NET_UP;
	/* A stop that arrived while the bridge was going in is honoured by the next pass of
	 * the owner loop, now that the state it has to unwind is fully established -- that
	 * is the whole point of only letting nxn_stop() end an armed session. */
	LOG_INF("host stack up, mac %02x:%02x:%02x:%02x:%02x:%02x",
	        nxn_mac[0], nxn_mac[1], nxn_mac[2], nxn_mac[3], nxn_mac[4], nxn_mac[5]);
	return;

stop:
	/* The module has been told to bridge (whatever it answered), so the only honest
	 * way out is the full teardown. */
	if (claimed) {
		rtl_link_unclaim();
		claimed = 0;
	}
	nxn_stop(nxn_why);
	return;

unref:
	if (attached && !armed)
		link_data_detach();          /* safe: the module was never told to send */
	rtl_link_uart_unref_gen(nxn_uart_gen);
	nxn_uart_gen = 0u;
refuse:
	if (claimed)
		rtl_link_unclaim();
	nxn_state = NX_NET_OFF;
	LOG_WRN("cannot bring the host stack up -- %s", nxn_why);
}

/*
 * Feed the module's watchdog, and take its loss ledger before doing so: every non-OFF
 * DATA_CFG zeroes the module's DATA counters.
 */
static void nxn_refresh(void)
{
	uint32_t st[NXN_DSTAT_WORDS];

	if (rtl_link_claim(NXN_CLAIM_MS) != 0)
		return;                      /* skipped, not failed -- 30 s absorbs a few */

	if (nxn_data_stats(st) == 0)
		nxn_mod_accumulate(st);

	if (nxn_data_cfg(ERPC_DATA_MODE_BRIDGE, NXN_HOLD_MS) != 0) {
		nxn_refresh_fails++;
	} else {
		uint8_t mac[6], flags = 0u;

		nxn_refresh_fails = 0;
		if (nxn_eth_info(mac, &flags) == 0) {
			memcpy(nxn_mac, mac, sizeof mac);
			nxn_radio_up = (flags & ERPC_ETH_F_RUNNING) ? 1u : 0u;
		}
	}
	rtl_link_unclaim();

	if (nxn_refresh_fails >= NXN_REFRESH_FAILS) {
		nxn_refresh_fails = 0;
		/* Not a bare "go down": the module may still be bridged, so the same proof
		 * obligation applies and only nxn_stop() discharges it. */
		nxn_stop("the module stopped acknowledging the bridge");
	}
}

/*
 * `wifi on/off/reset` force-quiesced the link and revoked our reference.  That is the
 * module-down exception in person, so it does not go through nxn_stop(): the UART is
 * already closed and erpc_link_closed() has already reclaimed every DATA buffer, so
 * detaching cannot desynchronise anything.  rtl_link_uart_unref_gen() is a no-op once the
 * generation has moved, which is what stops us decrementing somebody else's reference.
 */
static void nxn_link_revoked(void)
{
	nx_link_driver_set_link(0);
	nxn_dhcp_halt();
	link_data_detach();
	rtl_link_uart_unref_gen(nxn_uart_gen);
	nxn_uart_gen = 0u;
	nxn_why = "the RTL8720 link was taken away (wifi on/off/reset)";
	nxn_state = NX_NET_OFF;
	LOG_WRN("%s", nxn_why);
}

static void nxn_entry(ULONG arg)
{
	ULONG flags;

	(void)arg;

	for (;;) {
		/*
		 * Consume the stop request only in a state that can act on it.  Clearing it
		 * unconditionally used to lose a `net down` typed while ARMING: the owner
		 * wiped the flag and then carried on bringing the interface up, so the
		 * request evaporated.  During ARMING the flag is left standing -- nxn_arm()
		 * checks it at its own safe points, and whatever it does not catch is acted
		 * on by the next pass through here.
		 */
		if (nxn_req_stop && (nxn_state == NX_NET_UP || nxn_state == NX_NET_OFF ||
		                     nxn_state == NX_NET_FAILED)) {
			int was_up = (nxn_state == NX_NET_UP);

			nxn_req_stop = 0u;
			if (was_up)
				nxn_stop("requested");
		}

		switch (nxn_state) {
		case NX_NET_ARMING:
			nxn_arm();
			break;

		case NX_NET_UP:
			/*
			 * A moved generation means wifi on/off/reset force-quiesced the link
			 * and revoked our reference.  Check it before the refresh: there is
			 * no point asking a module that has been power-cycled.
			 */
			if (rtl_link_uart_gen() != nxn_uart_gen) {
				nxn_link_revoked();
				break;
			}
			nxn_refresh();
			if (nxn_state == NX_NET_UP)
				(void)tx_event_flags_get(&nxn_evt, NXN_EVT_CMD, TX_OR_CLEAR,
				                         &flags, NXN_REFRESH_MS);
			break;

		case NX_NET_FAILED:
			/*
			 * FAILED still holds things -- possibly the DATA consumer and a UART
			 * reference -- because it could not prove they were safe to give
			 * back.  `wifi reset` is what makes them safe, and it shows up here
			 * as the quiesce generation moving.  Without this the latch would
			 * never clear and `link eth` would be refused forever.
			 */
			if (rtl_link_quiesce_gen() != nxn_failed_at) {
				link_data_detach();
				rtl_link_uart_unref_gen(nxn_uart_gen);
				nxn_uart_gen = 0u;
				nxn_why = "recovered by a module power cycle";
				nxn_state = NX_NET_OFF;
				LOG_INF("%s", nxn_why);
				break;
			}
			(void)tx_event_flags_get(&nxn_evt, NXN_EVT_CMD, TX_OR_CLEAR, &flags,
			                         500u);
			break;

		default:
			(void)tx_event_flags_get(&nxn_evt, NXN_EVT_CMD, TX_OR_CLEAR, &flags,
			                         TX_WAIT_FOREVER);
			break;
		}
	}
}

/* ---- init -------------------------------------------------------------------- */

int nx_net_init(void)
{
	UINT s;

	nx_system_initialize();

	s = nx_packet_pool_create(&nxn_pool, "nx-link", NXN_PAYLOAD, nxn_pool_mem,
	                          sizeof nxn_pool_mem);
	if (s != NX_SUCCESS) {
		LOG_ERR("packet pool create failed (0x%02x)", (unsigned)s);
		return NXN_ERR;
	}

	/* Before nx_ip_create(), which runs NX_LINK_INITIALIZE. */
	nx_link_driver_set_pool(&nxn_pool);

	s = nx_ip_create(&nxn_ip, "nx-link", 0, 0xFFFFFF00UL, &nxn_pool, nx_link_driver,
	                 (VOID *)nxn_ip_stack, sizeof nxn_ip_stack, NXN_IP_PRIORITY);
	if (s != NX_SUCCESS) {
		LOG_ERR("ip create failed (0x%02x)", (unsigned)s);
		return NXN_ERR;
	}

	nx_arp_enable(&nxn_ip, (VOID *)nxn_arp_cache, sizeof nxn_arp_cache);
	nx_icmp_enable(&nxn_ip);
	nx_udp_enable(&nxn_ip);                  /* DHCP                                */
	nx_tcp_enable(&nxn_ip);                  /* compiled in for U4; no sockets yet  */

	nx_ip_link_status_change_notify_set(&nxn_ip, nxn_link_status_cb);

	if (nx_dhcp_create(&nxn_dhcp, &nxn_ip, "nx-link") == NX_SUCCESS) {
		nx_dhcp_packet_pool_set(&nxn_dhcp, &nxn_pool);
		nx_dhcp_state_change_notify(&nxn_dhcp, nxn_dhcp_state_cb);
		nxn_dhcp_created = true;
	} else {
		LOG_WRN("dhcp create failed; use `net ip` for a static address");
	}

	if (tx_mutex_create(&nxn_addr_lock, "nx-addr", TX_INHERIT) != TX_SUCCESS)
		return NXN_ERR;
	if (tx_event_flags_create(&nxn_evt, "nx-net") != TX_SUCCESS)
		return NXN_ERR;
	if (tx_thread_create(&nxn_thread, "nx-net", nxn_entry, 0, nxn_stack,
	                     sizeof nxn_stack, NXN_OWNER_PRIORITY, NXN_OWNER_PRIORITY,
	                     TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS)
		return NXN_ERR;

	nxn_ready = true;
	LOG_INF("NetX Duo ready (pool %u B in DTCM, IP prio %u)",
	        (unsigned)NXN_POOL_BYTES, (unsigned)NXN_IP_PRIORITY);
	return NXN_OK;
}

/* ---- control API -------------------------------------------------------------- */

bool nx_net_is_up(void)
{
	return nxn_state == NX_NET_UP;
}

enum nx_net_state nx_net_state(void)
{
	return nxn_state;
}

int nx_net_up(const char **why)
{
	if (!nxn_ready) {
		if (why)
			*why = "the host stack did not initialise";
		return -1;
	}
	if (nxn_state == NX_NET_FAILED) {
		/* The owner thread clears this within half a second of a `wifi reset`; it
		 * has to do the clearing, because it is the one holding what must be given
		 * back.  Until then, say what fixes it. */
		if (why)
			*why = "the host stack could not be shut down cleanly -- run "
			       "`wifi reset`";
		return -1;
	}
	if (nxn_state != NX_NET_OFF) {
		if (why)
			*why = "the host stack is already up or in transition";
		return -1;
	}

	/*
	 * Publish ARMING on THIS thread, before the owner is signalled: nx_net_guard() has
	 * to be closed from the moment the command is typed, not from whenever the owner
	 * next runs.  Same lost-wakeup ordering as net_shell_start().
	 */
	nxn_why = "";
	nxn_state = NX_NET_ARMING;
	tx_event_flags_set(&nxn_evt, NXN_EVT_CMD, TX_OR);
	return 0;
}

void nx_net_down(void)
{
	if (!nxn_ready)
		return;
	nxn_req_stop = 1u;
	tx_event_flags_set(&nxn_evt, NXN_EVT_CMD, TX_OR);
}

int nx_net_info_get(struct nx_net_info *out)
{
	ULONG ip = 0, mask = 0, gw = 0;

	if (!nxn_ready)
		return NXN_ERR_STATE;

	nx_ip_address_get(&nxn_ip, &ip, &mask);
	nx_ip_gateway_address_get(&nxn_ip, &gw);
	out->ip        = (uint32_t)ip;
	out->mask      = (uint32_t)mask;
	out->gw        = (uint32_t)gw;
	out->ip_valid  = (ip != 0);
	/* Under the lock like every other reader of it: the cost is a mutex on a reporting
	 * path, and the alternative is a rule with an exception, which is the kind of thing
	 * that is true right up until somebody adds the next caller. */
	nxn_addr_take();
	out->dhcp_mode = nxn_dhcp_created && !nxn_static_mode;
	nxn_addr_give();
	return NXN_OK;
}

int nx_net_set_static(uint32_t ip, uint32_t mask, uint32_t gw)
{
	int rc = NXN_OK;

	if (!nxn_ready)
		return NXN_ERR_STATE;

	nxn_addr_take();
	/* Re-checked HERE, under the lock, not by the caller: between a CLI thread's
	 * nx_net_is_up() and this call the owner may have started taking the interface
	 * down, and the loser of that race must be this one. */
	if (nxn_state != NX_NET_UP) {
		nxn_addr_give();
		return NXN_ERR_STATE;
	}
	if (nxn_dhcp_started) {
		nx_dhcp_stop(&nxn_dhcp);
		nxn_dhcp_started = false;
	}
	nxn_static_mode = true;
	if (nx_ip_address_set(&nxn_ip, ip, mask) != NX_SUCCESS)
		rc = NXN_ERR;
	else
		nx_ip_gateway_address_set(&nxn_ip, gw);
	nxn_addr_give();
	return rc;
}

int nx_net_dhcp_renew(void)
{
	UINT s;

	if (!nxn_ready)
		return NXN_ERR_STATE;
	if (!nxn_dhcp_created)
		return NXN_ERR;

	nxn_addr_take();
	/* The client only makes sense while the host stack owns the network: with the link
	 * down every DISCOVER goes nowhere and the state machine just retries.  Checked
	 * under the lock so a concurrent `net down` either finishes first (and this
	 * refuses) or waits (and then stops what this started). */
	if (nxn_state != NX_NET_UP) {
		nxn_addr_give();
		return NXN_ERR_STATE;
	}

	nxn_static_mode = false;
	if (nxn_dhcp_started) {
		nx_dhcp_stop(&nxn_dhcp);
		nx_dhcp_reinitialize(&nxn_dhcp);
		nxn_dhcp_started = false;
	}
	s = nx_dhcp_start(&nxn_dhcp);
	/* ALREADY_STARTED means the state we wanted is the state we have -- that is a
	 * success, not a failure, and treating it as one is how a second `net dhcp` ends up
	 * reporting an error while a lease is on its way. */
	if (s != NX_SUCCESS && s != NX_DHCP_ALREADY_STARTED) {
		nxn_addr_give();
		return NXN_ERR;
	}
	nxn_dhcp_started = true;
	nxn_addr_give();
	return NXN_OK;
}

int nx_net_ping(uint32_t ip, unsigned timeout_ms, unsigned *rtt_ms)
{
	NX_PACKET *resp = NX_NULL;
	ULONG t0, t1;
	UINT s;

	if (!nxn_ready)
		return NXN_ERR_STATE;

	t0 = tx_time_get();
	/* NX_IP_PERIODIC_RATE is 1000 here (port/netxduo/nx_user.h), so the wait option
	 * really is a millisecond count. */
	s  = nx_icmp_ping(&nxn_ip, (ULONG)ip, "wio-nx-ping", 11, &resp, (ULONG)timeout_ms);
	t1 = tx_time_get();

	if (resp != NX_NULL)
		nx_packet_release(resp);
	if (s == NX_NO_RESPONSE)
		return NXN_TIMEOUT;
	if (s != NX_SUCCESS)
		return NXN_ERR;
	if (rtt_ms)
		*rtt_ms = (unsigned)(t1 - t0);
	return NXN_OK;
}

void nx_net_mac_get(uint8_t mac[6])
{
	memcpy(mac, nxn_mac, 6);
}

void nx_net_modstats_get(struct nx_net_modstats *out)
{
	/* Paired with nxn_mod_accumulate(): a consistent snapshot, not nine independent
	 * reads (`net info` and `net echo` both quote drops as a fraction of frames). */
	uint32_t pm = __get_PRIMASK();

	__disable_irq();
	*out = nxn_mod;
	__set_PRIMASK(pm);
}

/*
 * The NetX objects, for whoever puts a socket on this interface.  Handing them out only
 * once nxn_ready is set is the whole safety argument: before that the control blocks are
 * uninitialised memory, and nx_tcp_socket_create() would happily scribble on an IP
 * instance that does not exist yet.
 */
void *nx_net_ip(void)
{
	return nxn_ready ? (void *)&nxn_ip : NULL;
}

void *nx_net_pool(void)
{
	return nxn_ready ? (void *)&nxn_pool : NULL;
}

/* ---- reporting / guard --------------------------------------------------------- */

static const char *nxn_state_name(enum nx_net_state s)
{
	switch (s) {
	case NX_NET_OFF:      return "off";
	case NX_NET_ARMING:   return "arming";
	case NX_NET_UP:       return "up";
	case NX_NET_STOPPING: return "stopping";
	default:              return "FAILED";
	}
}

void nx_net_print_status(struct cli_instance *sh)
{
	struct nx_link_stats st;
	struct nx_net_modstats ms;
	ULONG avail = 0, total = 0, empty = 0, waits = 0, invalid = 0;

	cli_print(sh, "  host stack: %s%s%s\r\n", nxn_state_name(nxn_state),
	          (nxn_why && nxn_why[0]) ? " -- " : "", (nxn_why) ? nxn_why : "");
	if (nxn_state == NX_NET_OFF)
		return;

	cli_print(sh, "  mac: %02x:%02x:%02x:%02x:%02x:%02x, radio %s, link %s\r\n",
	          nxn_mac[0], nxn_mac[1], nxn_mac[2], nxn_mac[3], nxn_mac[4], nxn_mac[5],
	          nxn_radio_up ? "up" : "DOWN",
	          nx_link_driver_link_up() ? "up" : "down");

	nx_link_driver_get_stats(&st);
	cli_print(sh, "  nx rx: %lu frames, %lu B, no-buf %lu, oversize %lu, undersize %lu, "
	          "unknown %lu, down %lu\r\n",
	          (unsigned long)st.rx_frames, (unsigned long)st.rx_bytes,
	          (unsigned long)st.rx_no_buf, (unsigned long)st.rx_oversize,
	          (unsigned long)st.rx_undersize, (unsigned long)st.rx_unknown_type,
	          (unsigned long)st.rx_link_down);
	cli_print(sh, "  nx tx: %lu frames, %lu B, no-buf %lu, oversize %lu, coalesced %lu, "
	          "down %lu\r\n",
	          (unsigned long)st.tx_frames, (unsigned long)st.tx_bytes,
	          (unsigned long)st.tx_no_buf, (unsigned long)st.tx_oversize,
	          (unsigned long)st.tx_coalesced, (unsigned long)st.tx_link_down);

	nx_packet_pool_info_get(&nxn_pool, &total, &avail, &empty, &waits, &invalid);
	cli_print(sh, "  nx pool: %lu/%lu free, %lu empty requests\r\n",
	          (unsigned long)avail, (unsigned long)total, (unsigned long)empty);

	/* Accumulated across bridge re-arms -- each one zeroes the module's own copy. */
	nx_net_modstats_get(&ms);
	cli_print(sh, "  mod data rx: %lu frames, %lu B, drops %lu, crc %lu, oversize %lu, "
	          "gaps %lu%s\r\n",
	          (unsigned long)ms.rx_frames, (unsigned long)ms.rx_bytes,
	          (unsigned long)ms.rx_drops, (unsigned long)ms.rx_crc,
	          (unsigned long)ms.rx_oversize, (unsigned long)ms.rx_gaps,
	          (ms.rx_drops || ms.rx_crc || ms.rx_oversize || ms.rx_gaps)
	                  ? "  <-- LOSS" : "  (clean)");
	cli_print(sh, "  mod data tx: %lu frames, %lu B, drops %lu\r\n",
	          (unsigned long)ms.tx_frames, (unsigned long)ms.tx_bytes,
	          (unsigned long)ms.tx_drops);
}

int nx_net_guard(struct cli_instance *sh, const char *what)
{
	if (nxn_state == NX_NET_OFF)
		return 0;

	if (nxn_state == NX_NET_FAILED)
		cli_error(sh, "%s: refused -- the host stack could not be shut down "
		          "cleanly; run `wifi reset`\r\n", what);
	else
		cli_error(sh, "%s: refused -- the host stack owns the module's network "
		          "(state %s); run `net down` first\r\n", what,
		          nxn_state_name(nxn_state));
	return 1;
}
