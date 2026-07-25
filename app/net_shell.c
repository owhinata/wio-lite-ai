/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    net_shell.c
 * @brief   telnet shell console over the RTL8720DN socket offload.  See net_shell.h.
 *
 * Structure (ported from ../stm32f746g-disco port/netxduo/nx_shell.c): a transport vtable
 * that only moves bytes between two rings and a `connected` write gate, plus a server
 * thread that owns the sockets.  What differs is that there is no local TCP stack -- every
 * socket operation is an eRPC round-trip to the companion chip -- so the "callbacks push,
 * the server blocks in accept" model of the NetX version becomes a single thread that polls
 * the link with blocking calls.  Three properties of that link shape everything below:
 *
 *  1. ONE blocking module call at a time.  The issue-#20 N3 firmware runs a receive task
 *     plus TWO workers, and only the blocking socket receives are allowed to run in
 *     parallel (fw/rtl8720/README.md).  Our accept/recv permanently occupies one worker; if
 *     we started a second blocking call the shell's own RPCs -- from either console -- would
 *     have no worker left.  Hence one service thread, and no accept while a session runs
 *     (a second client waits in the listen backlog until the first leaves).
 *  2. Never abort accept/recv, and never close an fd with a call outstanding on it.
 *     Aborting only ends the HOST's wait; the module keeps running the call, so an aborted
 *     accept can complete into a socket whose fd we never learn and an aborted recv leaves
 *     the fd busy (N3 constraint 2).  So no should_abort hook is passed, and a host-side
 *     timeout (rc -2) is treated as "the module still owns these fds": they are LEAKED, not
 *     closed, and the console latches dirty until a `wifi reset` power-cycles the module.
 *  3. Requests must stay small on a stock module.  A frame bigger than its 127-byte UART
 *     ring loses its tail (the ASYMMETRY note in wifi_rpc.h), so output is sent in
 *     wifi_rpc_send_chunk()-byte chunks -- 96 there, the full 256 on a wio-n4 module.
 *
 * Ring ownership.  The RX ring is strict SPSC: this thread is the only producer, the CLI
 * instance thread the only consumer.  The TX ring has SEVERAL producers -- the instance
 * thread, a background-job worker writing through sh->fg, and session_begin() (which the
 * core calls WITHOUT the output lock) -- so producers serialise with a short PRIMASK
 * critical section, exactly like the USB CDC backend.  This thread is the sole consumer and
 * touches only `tail`, never `head`, so it never contends with them.
 *
 * No clock/RCC/register work of its own (XIP-safe).  Clean-room design.
 */
#include "net_shell.h"

#include <stddef.h>
#include <stdint.h>

#include "stm32h7xx_hal.h"    /* HAL_GetTick + CMSIS __get_PRIMASK/__disable_irq */
#include "tx_api.h"

#include "cli.h"
#include "cli_instance.h"
#include "cli_uart_ring.h"

#include "erpc.h"
#include "rtl8720.h"
#include "rtl_link.h"
#include "wifi_rpc.h"

#define LOG_TAG "nshell"
#include "log.h"

/* ---- tunables ------------------------------------------------------------ */

/* Above the CLI instances (16) so a reply is routed while a command computes, below the
 * eRPC service (10) that it depends on, and below usb (8) / iwdg (5). */
#define NSH_PRIORITY        12u
/* wifi_rpc_lwip_send() stages a 268 B request and _recv() a 264 B reply on the caller's
 * stack, on top of this thread's own frame and the lazy-VFP context.  Sized generously for
 * the first hardware validation; shrink only against a measured `thread` high-water mark. */
#define NSH_STACK           3072u

#define NSH_RX_RING         512u
#define NSH_TX_RING         1024u

/* Bytes per send RPC comes from wifi_rpc_send_chunk(), read fresh on every burst: 96 on a
 * stock module (24 + 96 = 120 fits entirely in its 127-byte UART ring, the only structural
 * guarantee that a request cannot lose its tail no matter how long its reader stalls) and
 * the full 256 once the link is proved to be wio-n4, whose ring is 8 kB.  Reading it per
 * burst is what lets a `wifi reset` mid-session drop us back to the safe size. */
/* Send at most this many chunks before polling RX again, so a long output burst can still
 * see a Ctrl+C (~4 x 2 ms of send + one 2 ms poll). */
#define NSH_TX_BURST        4u

/* Module-side receive window when there is nothing to send.  This is also the worst-case
 * latency for output that is NOT a reply to input (a `watch` refresh, a background job), so
 * it trades link occupancy (4 RPC/s ~ 0.8% of the link) against that lag. */
#define NSH_RECV_IDLE_MS    250u
/* Module-side receive window for the "is there a keystroke?" poll between output chunks. */
#define NSH_RECV_POLL_MS    1u
/* After handing received bytes to the shell, wait this long for its answer before going
 * back to a blocking receive.  Without it the echo would wait out NSH_RECV_IDLE_MS: this
 * thread runs ABOVE the CLI instance, so it would re-arm the receive before the shell ever
 * ran.  Only costs anything when input produced no output at all. */
#define NSH_TX_GRACE_MS     20u

/* The firmware's own accept cap (issue #20 N2 patch: a hard 10 s lwip_select, SO_RCVTIMEO
 * is NOT honoured there). */
#define NSH_ACCEPT_MS       10000u

/*
 * Host-side headroom added to every call's module-side duration.
 *
 * It has to be BIG.  Everything except the blocking receives takes the module's single
 * serial mutex, so while another console runs `wifi connect` / `net dhcp` our send can be
 * queued behind it for its whole 20-30 s -- and the eRPC wire budget (127 B) additionally
 * holds our frame back while their 128 B request is outstanding.  A host timeout firing
 * there would be a FALSE "link dirty" verdict, costing the session and leaking two fds.
 * The cost of being generous is only that a genuinely dead module takes this long to
 * notice; `wifi reset` remains instant either way, because force-quiesce wakes every
 * waiting token with -2 immediately.
 */
#define NSH_RPC_SLACK_MS    45000u

/* How long ARM waits for the coarse link mutex before giving up (a `wifi connect` holds it
 * for its whole flow). */
#define NSH_ARM_CLAIM_MS    60000u

/* Service thread event flags. */
#define NSH_EVT_TX          0x1u    /* the TX ring gained bytes         */
#define NSH_EVT_CMD         0x2u    /* a start/stop request was posted  */

/* A host-side timeout or abort: the module is still running the call (see the header). */
#define NSH_DIRTY(rc)       ((rc) == -2 || (rc) == -4)

/* ---- state --------------------------------------------------------------- */

/* PRIMASK critical section, as in shell/backend/cli_backend_usbcdc.c.  Nests safely inside
 * ThreadX's own PRIMASK sections. */
#define NSH_CRIT_ENTER()  do { uint32_t _pm = __get_PRIMASK(); __disable_irq()
#define NSH_CRIT_EXIT()   __set_PRIMASK(_pm); } while (0)

struct nsh_ctx {
	struct cli_instance *sh;             /* set by nsh_init() from tr->sh          */
	struct cli_uart_ring rx;             /* service thread -> CLI thread (SPSC)    */
	struct cli_uart_ring tx;             /* CLI/bg threads -> service thread (MPSC)*/
	uint8_t              rx_buf[NSH_RX_RING];
	uint8_t              tx_buf[NSH_TX_RING];
	volatile uint8_t     connected;      /* write gate; owned by the CLI thread    */
};

static struct nsh_ctx       g_ctx;
static TX_THREAD            g_thread;
static TX_EVENT_FLAGS_GROUP g_evt;
static UCHAR                g_stack[NSH_STACK] __attribute__((aligned(8)));
static uint8_t              g_ready;

/* Owned by the service thread (others only read). */
static volatile enum net_shell_state g_state;
static int32_t   g_lfd = -1, g_cfd = -1;
static uint16_t  g_port = NET_SHELL_PORT_DEFAULT;
static uint32_t  g_uart_gen;             /* the UART "open" our reference belongs to */
static int       g_iac;                  /* telnet IAC receive state                 */
static uint8_t   g_rxbuf[WIFI_RPC_STREAM_MAX];   /* static: keeps the stack shallow  */
static uint8_t   g_ip[4];                /* address we are listening on (for status) */

/* A connection is currently accepted.  Set here before CLI_EVT_CONN is posted and cleared
 * on disconnect, so a session_begin() the CLI thread only reaches AFTER the client vanished
 * does not resurrect the write gate on a dead socket. */
static volatile uint8_t g_link_live;

/*
 * Requests from command threads.  Arming is posted by moving the state to NET_SHELL_ARMING
 * (only ever done from NET_SHELL_OFF, which no client can be using) and THEN raising
 * NSH_EVT_CMD -- that order is what makes the service thread's "test the state, else park on
 * the flag" loop free of lost wakeups without a lock.
 */
static volatile uint8_t  g_req_stop;
static volatile uint8_t  g_autoarm = 1u;
static volatile uint16_t g_req_port = NET_SHELL_PORT_DEFAULT;

/* Dirty latch: module-side sockets were leaked and only a power cycle reclaims them.  Held
 * against the quiesce generation at the time, so any later `wifi on/off/reset` clears it. */
static uint8_t  g_dirty;
static uint32_t g_dirty_at;

static const char *g_last;               /* last state-changing reason, for `status` */
static uint32_t g_sessions, g_rx_bytes, g_tx_bytes, g_rx_drops;
static struct erpc_diag g_diag, g_diag_tot;

/* ---- TX ring producer side (PRIMASK; may run on any shell thread) --------- */

/* Store one byte verbatim.  Returns 1 if stored, 0 if the ring was full. */
static int nsh_tx_put_raw(uint8_t b)
{
	int ok;

	NSH_CRIT_ENTER();
	ok = cli_uart_ring_put(&g_ctx.tx, b);
	NSH_CRIT_EXIT();
	return ok;
}

/*
 * Store one shell byte, telnet-encoded: a literal 0xFF must go out as IAC IAC or the client
 * reads it as the start of a command (f746's nx_shell strips IAC on receive but never
 * escaped its output -- which corrupts any binary the shell prints).  The pair is stored
 * inside ONE critical section so another producer can never be interleaved between them.
 */
static int nsh_tx_put(uint8_t b)
{
	int ok;

	NSH_CRIT_ENTER();
	if (b == 0xFFu)
		ok = (cli_uart_ring_free(&g_ctx.tx) >= 2u) &&
		     cli_uart_ring_put(&g_ctx.tx, 0xFFu) &&
		     cli_uart_ring_put(&g_ctx.tx, 0xFFu);
	else
		ok = cli_uart_ring_put(&g_ctx.tx, b);
	NSH_CRIT_EXIT();
	return ok;
}

/* Drop everything queued.  CONSUMER side (service thread): it only advances `tail`, so it
 * never races a producer's `head`.  Always followed by the TX notify below, so a writer
 * blocked on CLI_EVT_TX wakes up, re-enters write() and returns at once on the closed gate
 * instead of waiting out CLI_TX_TIMEOUT. */
static void nsh_tx_discard(void)
{
	size_t n = cli_uart_ring_count(&g_ctx.tx);

	if (n)
		cli_uart_ring_advance_tail(&g_ctx.tx, n);
	if (g_ctx.sh != NULL)
		cli_transport_notify_tx(g_ctx.sh);
}

/* ---- transport vtable ---------------------------------------------------- */

static int nsh_init(struct cli_transport *tr)
{
	cli_uart_ring_init(&g_ctx.rx, g_ctx.rx_buf, sizeof g_ctx.rx_buf);
	cli_uart_ring_init(&g_ctx.tx, g_ctx.tx_buf, sizeof g_ctx.tx_buf);
	g_ctx.connected = 0;
	g_ctx.sh        = tr->sh;      /* cli_init() sets tr->sh before calling init */
	return 0;
}

static int nsh_enable(struct cli_transport *tr)
{
	(void)tr;                      /* the sockets are opened by the service thread */
	return 0;
}

static int nsh_write(struct cli_transport *tr, const uint8_t *data, size_t len)
{
	size_t i;

	(void)tr;
	/* Not connected: swallow it (req §11).  Returning `len` is what keeps the shell from
	 * ever wedging on a console nobody is attached to. */
	if (!g_ctx.connected)
		return (int)len;

	for (i = 0; i < len; i++) {
		if (!nsh_tx_put(data[i]))
			break;                 /* full: the core waits for CLI_EVT_TX */
	}
	if (i != 0u)
		(void)tx_event_flags_set(&g_evt, NSH_EVT_TX, TX_OR);
	return (int)i;
}

static int nsh_read(struct cli_transport *tr, uint8_t *data, size_t cap)
{
	(void)tr;
	/* SPSC: the service thread is the only producer, this (the CLI thread) the only
	 * consumer -- lock-free, like the UART/CDC backends. */
	return (int)cli_uart_ring_get_buf(&g_ctx.rx, data, cap);
}

/* Ask a telnet client for character-at-a-time mode (no local echo, no line buffering),
 * which is what the interactive line editor needs.  A raw `nc` shows these six bytes as
 * harmless garbage before the prompt -- the session is telnet-first. */
static const uint8_t nsh_charmode[] = {
	0xFFu, 0xFBu, 0x01u,   /* IAC WILL ECHO              */
	0xFFu, 0xFBu, 0x03u,   /* IAC WILL SUPPRESS_GO_AHEAD */
};

static void nsh_session_begin(struct cli_transport *tr)
{
	uint8_t junk;
	unsigned i;

	(void)tr;
	/* The CLI thread calls this on CLI_EVT_CONN, after the editor state was reset and
	 * before the prompt is drawn.  Drain the previous session's leftovers HERE, on the
	 * consumer side, so the RX ring stays strict SPSC. */
	while (cli_uart_ring_get(&g_ctx.rx, &junk))
		;
	/* Enable output only if the connection is still up: a client that vanished before the
	 * CLI thread got here leaves g_link_live == 0, so the prompt is dropped rather than
	 * written to a socket that is gone. */
	g_ctx.connected = g_link_live;
	if (!g_ctx.connected)
		return;
	for (i = 0u; i < sizeof nsh_charmode; i++)
		(void)nsh_tx_put_raw(nsh_charmode[i]);   /* IAC bytes: NOT escaped */
	(void)tx_event_flags_set(&g_evt, NSH_EVT_TX, TX_OR);
}

static const struct cli_transport_api nsh_api = {
	nsh_init, nsh_enable, nsh_write, nsh_read, NULL, NULL, nsh_session_begin,
};

struct cli_transport net_shell_transport = {
	.api = &nsh_api,
	.sh  = NULL,                   /* set by cli_init() */
	.ctx = &g_ctx,
};

/* ---- telnet receive ------------------------------------------------------ */

/* Drop 0xFF + command [+ option]; 0xFF 0xFF becomes one literal 0xFF.  Returns 1 when @b is
 * consumed (negotiation), 0 when it is shell data. */
static int nsh_iac_consume(uint8_t b)
{
	switch (g_iac) {
	case 0:
		if (b == 0xFFu) { g_iac = 1; return 1; }
		return 0;
	case 1:
		if (b == 0xFFu) { g_iac = 0; return 0; }              /* IAC IAC -> 0xFF   */
		if (b >= 0xFBu && b <= 0xFEu) { g_iac = 2; return 1; } /* WILL/WONT/DO/DONT */
		g_iac = 0; return 1;                                   /* other 2-byte cmd  */
	default:
		g_iac = 0; return 1;                                   /* option byte       */
	}
}

/* Hand received bytes to the shell instance. */
static void nsh_feed(const uint8_t *d, uint16_t n)
{
	uint16_t i;
	int woke = 0;

	g_rx_bytes += n;
	for (i = 0u; i < n; i++) {
		if (nsh_iac_consume(d[i]))
			continue;
		if (!cli_uart_ring_put(&g_ctx.rx, d[i])) {
			g_rx_drops++;          /* ring full: byte lost, keep the stream in sync */
			continue;
		}
		woke = 1;
	}
	if (woke && g_ctx.sh != NULL)
		cli_transport_notify_rx(g_ctx.sh);
}

/* ---- eRPC helpers -------------------------------------------------------- */

/* Per-call options.  Deliberately WITHOUT an abort hook -- see the header. */
static void nsh_opts(struct wifi_rpc_opts *o, uint32_t module_ms)
{
	o->timeout_ms   = module_ms + NSH_RPC_SLACK_MS;
	o->should_abort = NULL;
	o->abort_ctx    = NULL;
	o->diag         = &g_diag;
}

static void nsh_diag_acc(void)
{
	g_diag_tot.crc_fail               += g_diag.crc_fail;
	g_diag_tot.oversize               += g_diag.oversize;
	g_diag_tot.timeout                += g_diag.timeout;
	g_diag_tot.skipped_reply          += g_diag.skipped_reply;
	g_diag_tot.unsupported_invocation += g_diag.unsupported_invocation;
	g_diag_tot.frame_stall            += g_diag.frame_stall;
	g_diag_tot.ctrl_bad               += g_diag.ctrl_bad;
}

/* Ask the module why the last socket call failed (0 when even that did not get through). */
static int32_t nsh_errno(void)
{
	struct wifi_rpc_opts o;
	int32_t e = 0;

	nsh_opts(&o, 0u);
	if (wifi_rpc_lwip_errno(&o, &e) != 0)
		e = 0;
	nsh_diag_acc();
	return e;
}

/*
 * Give up the link without closing anything.  Called when a call did not come back: the
 * module may still be running it, and closing an fd underneath an in-flight receive is
 * exactly what N3 constraint 2 forbids -- so the fds are LEAKED and only a power cycle
 * reclaims them.  If our UART reference was revoked instead (force-quiesce from `wifi
 * on/off/reset`), the module is being power-cycled anyway: the sockets die with it, so that
 * case does NOT latch dirty and a re-arm after the reset is free.
 */
static void nsh_fail(const char *why)
{
	int revoked = (rtl_link_uart_gen() != g_uart_gen);

	g_ctx.connected = 0;
	g_link_live     = 0;
	nsh_tx_discard();
	g_cfd = -1;
	g_lfd = -1;
	if (!revoked) {
		g_dirty    = 1u;
		g_dirty_at = rtl_link_quiesce_gen();
		LOG_ERR("%s -- module sockets leaked, run `wifi reset`", why);
	} else {
		LOG_WRN("%s", why);
	}
	rtl_link_uart_unref_gen(g_uart_gen);   /* no-op once the generation moved */
	g_last  = why;
	g_state = NET_SHELL_OFF;
}

/* Close @*fd.  Safe by construction: this thread issues every blocking call on these fds
 * and is here, so none is outstanding (N3 constraint 2).  Returns 0, or -1 when the link
 * went dirty (nsh_fail() has already run). */
static int nsh_close(int32_t *fd, const char *what)
{
	struct wifi_rpc_opts o;
	int32_t ret = -1;
	int rc;

	if (*fd < 0)
		return 0;
	nsh_opts(&o, 0u);
	rc = wifi_rpc_lwip_close(&o, *fd, &ret);
	nsh_diag_acc();
	if (NSH_DIRTY(rc)) {
		nsh_fail("close got no reply from the module");
		return -1;
	}
	if (rc || ret < 0)
		LOG_WRN("close(%s fd %ld) failed (rc %d ret %ld)", what, (long)*fd, rc, (long)ret);
	*fd = -1;
	return 0;
}

/*
 * One int-valued setsockopt.  The OPTION is best effort -- a module that refuses it only
 * costs us Nagle or a keepalive -- but the CALL is not: no reply means the module is still
 * running it, so the link is dirty and nothing further may be issued.  Returns 0 to carry
 * on, -1 when the link went dirty (nsh_fail() has already run).
 */
static int nsh_setopt(int32_t fd, int32_t level, int32_t opt, int32_t val, const char *what)
{
	struct wifi_rpc_opts o;
	uint8_t v[4];
	int32_t ret = -1;
	int rc;

	v[0] = (uint8_t)val;                     v[1] = (uint8_t)((uint32_t)val >> 8);
	v[2] = (uint8_t)((uint32_t)val >> 16);   v[3] = (uint8_t)((uint32_t)val >> 24);
	nsh_opts(&o, 0u);
	rc = wifi_rpc_lwip_setsockopt(&o, fd, level, opt, v, 4u, &ret);
	nsh_diag_acc();
	if (NSH_DIRTY(rc)) {
		nsh_fail("setsockopt got no reply from the module");
		return -1;
	}
	if (rc || ret < 0)
		LOG_WRN("%s failed (rc %d ret %ld)", what, rc, (long)ret);
	return 0;
}

/* ---- arm / teardown ------------------------------------------------------ */

/* Build a 16-byte lwIP sockaddr_in for 0.0.0.0:@port. */
static void nsh_sockaddr(uint8_t sa[16], uint16_t port)
{
	unsigned i;

	for (i = 0u; i < 16u; i++)
		sa[i] = 0u;
	sa[0] = 16u;                             /* sin_len                       */
	sa[1] = (uint8_t)WIFI_LWIP_AF_INET;      /* sin_family                    */
	sa[2] = (uint8_t)(port >> 8);            /* sin_port, network byte order  */
	sa[3] = (uint8_t)port;
}

/* Take the link and open the listening socket.  Returns 0 with the state left in
 * NET_SHELL_LISTENING, or -1 with g_last set (state OFF). */
static int nsh_arm(uint16_t port)
{
	struct wifi_rpc_opts o;
	struct wifi_ip_info ip;
	uint8_t sa[16];
	uint32_t t0 = HAL_GetTick();
	int32_t fd = -1, ret = -1, res = -1, conn = -1;
	int rc;

	if (!rtl8720_powered()) {
		g_last = "the RTL8720 is powered off";
		return -1;
	}
	/* The coarse mutex serialises us against whole command flows; a `wifi connect` can
	 * hold it for its full 30 s, so wait properly rather than give up on the first miss. */
	while (rtl_link_claim(RTL_LINK_CLAIM_WAIT_MS) != 0) {
		if (g_req_stop || (uint32_t)(HAL_GetTick() - t0) > NSH_ARM_CLAIM_MS) {
			g_last = "the RTL8720 link stayed busy";
			return -1;
		}
	}
	/* Whatever rate the link is running at (issue #23 U0-3 made it changeable): a
	 * mismatched literal here would refuse the reference rather than re-open, because
	 * rtl_link_uart_ref() rejects a configuration that disagrees with the open one. */
	if (rtl_link_uart_ref(RTL8720_UART_AT, rtl_link_erpc_baud()) != 0) {
		rtl_link_unclaim();
		g_last = "USART1 did not come ready";
		return -1;
	}
	/* Record which "open" our reference belongs to: a force-quiesce (`wifi on/off/reset`)
	 * revokes it, and releasing it afterwards must NOT decrement the next user's. */
	g_uart_gen = rtl_link_uart_gen();

	nsh_opts(&o, 3000u);
	rc = wifi_rpc_is_connected(&o, &conn);
	nsh_diag_acc();
	if (rc || conn != WIFI_RPC_OK) {
		g_last = "the RTL8720 is not associated";
		goto fail;
	}

	nsh_opts(&o, 0u);
	rc = wifi_rpc_lwip_socket(&o, WIFI_LWIP_AF_INET, WIFI_LWIP_SOCK_STREAM, 0, &fd);
	nsh_diag_acc();
	if (NSH_DIRTY(rc))
		goto dirty;
	if (rc || fd < 0) {
		g_last = "socket(SOCK_STREAM) failed";
		goto fail;
	}
	/* Own it from here on, so every path below either closes it (fail) or deliberately
	 * leaks it (dirty) rather than losing the number. */
	g_lfd = fd;
	/* Best effort: without SO_REUSEADDR a re-arm while a previous connection is in
	 * TIME_WAIT cannot rebind the port (lwIP scans the TIME_WAIT list unless it is set). */
	if (nsh_setopt(fd, WIFI_LWIP_SOL_SOCKET, WIFI_LWIP_SO_REUSEADDR, 1, "SO_REUSEADDR") != 0) {
		rtl_link_unclaim();        /* nsh_fail() already ran */
		return -1;
	}

	nsh_sockaddr(sa, port);
	nsh_opts(&o, 0u);
	rc = wifi_rpc_lwip_bind(&o, fd, sa, (uint16_t)sizeof sa, &ret);
	nsh_diag_acc();
	if (NSH_DIRTY(rc))
		goto dirty;
	if (rc || ret < 0) {
		int32_t e = (rc == 0) ? nsh_errno() : 0;

		LOG_ERR("bind :%u failed (rc %d ret %ld errno %ld %s)", (unsigned)port, rc,
		        (long)ret, (long)e, wifi_rpc_errno_name(e));
		g_last = "bind failed (port in use? retry in ~1 min)";
		goto fail;
	}

	nsh_opts(&o, 0u);
	rc = wifi_rpc_lwip_listen(&o, fd, 1, &ret);
	nsh_diag_acc();
	if (NSH_DIRTY(rc))
		goto dirty;
	if (rc || ret < 0) {
		g_last = "listen failed";
		goto fail;
	}

	/* Address, for the banner and `net shell status` (best effort). */
	nsh_opts(&o, 3000u);
	if (wifi_rpc_get_ip(&o, 0u, &ip, &res) == 0 && res == WIFI_RPC_OK) {
		g_ip[0] = ip.ip[0]; g_ip[1] = ip.ip[1];
		g_ip[2] = ip.ip[2]; g_ip[3] = ip.ip[3];
	}
	nsh_diag_acc();

	rtl_link_unclaim();
	g_port  = port;
	g_iac   = 0;
	g_last  = "listening";
	g_state = NET_SHELL_LISTENING;
	LOG_INF("listening on %u.%u.%u.%u:%u (telnet)", g_ip[0], g_ip[1], g_ip[2], g_ip[3],
	        (unsigned)port);
	return 0;

dirty:
	rtl_link_unclaim();
	nsh_fail("the module stopped answering while arming");
	return -1;
fail:
	if (g_lfd >= 0)
		(void)nsh_close(&g_lfd, "listening");
	rtl_link_uart_unref_gen(g_uart_gen);
	rtl_link_unclaim();
	g_state = NET_SHELL_OFF;
	return -1;
}

/* Close the sockets and give the UART reference back. */
static void nsh_teardown(const char *why)
{
	g_ctx.connected = 0;
	g_link_live     = 0;
	nsh_tx_discard();
	if (nsh_close(&g_cfd, "client") != 0)
		return;                        /* dirty: nsh_fail() already finished up */
	if (nsh_close(&g_lfd, "listening") != 0)
		return;
	/* The coarse mutex is the right thing to hold while dropping the reference (a
	 * bridge/flash session drives the same UART without referencing it).  If another
	 * console is sitting on it we release anyway: keeping the reference would leave those
	 * commands refused forever, and the count itself is protected by the eRPC link lock. */
	if (rtl_link_claim(RTL_LINK_CLAIM_WAIT_MS) == 0) {
		rtl_link_uart_unref_gen(g_uart_gen);
		rtl_link_unclaim();
	} else {
		rtl_link_uart_unref_gen(g_uart_gen);
		LOG_WRN("dropped the UART reference without the coarse mutex (it stayed busy)");
	}
	g_last  = why;
	g_state = NET_SHELL_OFF;
	LOG_INF("stopped (%s)", why);
}

/* End the current session and go back to listening. */
static void nsh_end_session(const char *why)
{
	g_ctx.connected = 0;
	g_link_live     = 0;
	nsh_tx_discard();
	/* A command may still be running for the client that just left.  0x03 is exactly what
	 * Ctrl+C delivers, so pushing one cancels it and leaves the instance idle for the next
	 * client instead of holding the link until it finishes on its own. */
	if (cli_uart_ring_put(&g_ctx.rx, 0x03u) && g_ctx.sh != NULL)
		cli_transport_notify_rx(g_ctx.sh);
	if (nsh_close(&g_cfd, "client") != 0)
		return;                        /* dirty */
	g_iac   = 0;
	g_last  = why;
	g_state = NET_SHELL_LISTENING;
	LOG_INF("client disconnected (%s)", why);
}

/* ---- service rounds ------------------------------------------------------ */

/* Wait briefly for the shell to produce output (see NSH_TX_GRACE_MS).  The ring, never this
 * flag, is the source of truth: the caller re-checks it, so a stale or cleared flag can
 * only cost one extra loop. */
static void nsh_tx_grace(void)
{
	ULONG f;

	if (cli_uart_ring_count(&g_ctx.tx) != 0u)
		return;
	(void)tx_event_flags_get(&g_evt, NSH_EVT_TX, TX_OR_CLEAR, &f, NSH_TX_GRACE_MS);
}

/* Push queued output.  Returns 0 to carry on, -1 when the session or the link ended. */
static int nsh_tx_drain(void)
{
	uint16_t chunk = wifi_rpc_send_chunk();
	unsigned n;

	for (n = 0u; n < NSH_TX_BURST; n++) {
		struct wifi_rpc_opts o;
		const uint8_t *p;
		size_t run = cli_uart_ring_contig(&g_ctx.tx, &p);
		int32_t ret = -1;
		int rc;

		if (run == 0u)
			break;
		if (run > chunk)
			run = chunk;

		nsh_opts(&o, 0u);
		rc = wifi_rpc_lwip_send(&o, g_cfd, p, (uint16_t)run, 0, &ret);
		nsh_diag_acc();
		if (NSH_DIRTY(rc)) {
			nsh_fail("send got no reply from the module");
			return -1;
		}
		if (rc) {
			nsh_end_session("send transport error");
			return -1;
		}
		if (ret <= 0 || (uint32_t)ret > (uint32_t)run) {
			nsh_end_session("send refused by the peer");
			return -1;
		}
		cli_uart_ring_advance_tail(&g_ctx.tx, (size_t)ret);
		g_tx_bytes += (uint32_t)ret;
		/* Space freed: the core may be blocked on CLI_EVT_TX waiting for exactly this
		 * (the vtable contract in cli_instance.h). */
		if (g_ctx.sh != NULL)
			cli_transport_notify_tx(g_ctx.sh);
	}
	return 0;
}

static void nsh_listen_round(void)
{
	struct wifi_rpc_opts o;
	int32_t cfd = -1;
	int rc;

	nsh_opts(&o, NSH_ACCEPT_MS);
	rc = wifi_rpc_lwip_accept(&o, g_lfd, &cfd);
	nsh_diag_acc();
	if (NSH_DIRTY(rc)) {
		nsh_fail("accept got no reply from the module");
		return;
	}
	if (rc) {
		g_state = NET_SHELL_STOPPING;
		nsh_teardown("accept transport error");
		return;
	}
	if (cfd < 0) {
		int32_t e = nsh_errno();

		if (e == WIFI_LWIP_ETIMEDOUT)
			return;                /* nobody within the firmware's cap: listen again */
		LOG_ERR("accept failed (errno %ld %s)", (long)e, wifi_rpc_errno_name(e));
		g_state = NET_SHELL_STOPPING;
		nsh_teardown("accept failed");
		return;
	}

	g_cfd = cfd;
	/* Without TCP_NODELAY, Nagle holds a one-byte echo until the previous segment is
	 * ACKed -- exactly the latency an interactive console cannot afford. */
	if (nsh_setopt(cfd, WIFI_LWIP_IPPROTO_TCP, WIFI_LWIP_TCP_NODELAY, 1, "TCP_NODELAY") != 0)
		return;
	if (nsh_setopt(cfd, WIFI_LWIP_SOL_SOCKET, WIFI_LWIP_SO_KEEPALIVE, 1, "SO_KEEPALIVE") != 0)
		return;

	/* Drop whatever the previous session left queued BEFORE opening the gate (consumer
	 * side, so no producer is involved), then let the CLI thread start a clean session. */
	nsh_tx_discard();
	g_iac       = 0;
	g_link_live = 1u;
	g_sessions++;
	g_state = NET_SHELL_SESSION;
	g_last  = "client connected";
	if (g_ctx.sh != NULL)
		cli_transport_notify_conn(g_ctx.sh);
	LOG_INF("client connected (fd %ld, session %lu)", (long)cfd, (unsigned long)g_sessions);

	/* Let the instance draw its prompt before we go back to a blocking receive. */
	nsh_tx_grace();
	(void)nsh_tx_drain();
}

static void nsh_session_round(void)
{
	struct wifi_rpc_opts o;
	uint16_t got = 0u;
	int32_t ret = -1;
	uint32_t win;
	int32_t flags;
	int rc;

	if (cli_uart_ring_count(&g_ctx.tx) != 0u) {
		if (nsh_tx_drain() != 0)
			return;
		/* Output is flowing: poll RX instead of parking on it, so a Ctrl+C interrupts a
		 * long report within about one burst. */
		win   = NSH_RECV_POLL_MS;
		flags = WIFI_LWIP_MSG_DONTWAIT;
	} else {
		win   = NSH_RECV_IDLE_MS;
		flags = 0;
	}

	nsh_opts(&o, win);
	rc = wifi_rpc_lwip_recv(&o, g_cfd, g_rxbuf, (uint16_t)sizeof g_rxbuf, flags, win,
	                        &got, &ret);
	nsh_diag_acc();
	if (NSH_DIRTY(rc)) {
		nsh_fail("recv got no reply from the module");
		return;
	}
	if (rc) {
		nsh_end_session("recv transport error");
		return;
	}
	if (ret == 0) {
		nsh_end_session("peer closed");
		return;
	}
	if (ret < 0) {
		int32_t e = nsh_errno();

		if (e == WIFI_LWIP_EAGAIN)
			return;                /* idle: the module's receive window expired */
		LOG_INF("recv ended (errno %ld %s)", (long)e, wifi_rpc_errno_name(e));
		nsh_end_session("peer gone");
		return;
	}

	nsh_feed(g_rxbuf, got);
	nsh_tx_grace();                    /* give the shell a chance to answer before recv */
}

/* ---- service thread ------------------------------------------------------ */

static void nsh_entry(ULONG arg)
{
	(void)arg;

	for (;;) {
		ULONG f;

		if (g_req_stop) {
			g_req_stop = 0u;
			g_autoarm  = 0u;           /* an explicit stop stays stopped */
			if (g_state != NET_SHELL_OFF) {
				g_state = NET_SHELL_STOPPING;
				nsh_teardown("requested");
			}
			continue;
		}

		switch (g_state) {
		case NET_SHELL_OFF:
			/* Nothing to do.  Sticky flags: a request that lands between the switch
			 * above and this wait still wakes us, because the requester moves the
			 * state FIRST and raises NSH_EVT_CMD second. */
			(void)tx_event_flags_get(&g_evt, NSH_EVT_CMD, TX_OR_CLEAR, &f,
			                         TX_WAIT_FOREVER);
			break;

		case NET_SHELL_ARMING:
			if (nsh_arm(g_req_port) != 0) {
				LOG_ERR("arm failed: %s", g_last ? g_last : "?");
				/* Belt and braces: every failure path must leave ARMING, or this
				 * loop would spin retrying forever. */
				if (g_state == NET_SHELL_ARMING)
					g_state = NET_SHELL_OFF;
			}
			break;

		case NET_SHELL_LISTENING:
			nsh_listen_round();
			break;

		case NET_SHELL_SESSION:
			nsh_session_round();
			break;

		default:
			g_state = NET_SHELL_OFF;
			break;
		}

		/* Did somebody take the link away underneath us (`wifi on/off/reset`)?  The RPCs
		 * above usually notice first (-2 from an abandoned token), but a revocation
		 * between two calls would otherwise go unseen until the next one. */
		if ((g_state == NET_SHELL_LISTENING || g_state == NET_SHELL_SESSION) &&
		    rtl_link_uart_gen() != g_uart_gen)
			nsh_fail("the RTL8720 link was taken away (wifi on/off/reset)");
	}
}

/* ---- public API ---------------------------------------------------------- */

int net_shell_init(void)
{
	if (tx_event_flags_create(&g_evt, "nshell") != TX_SUCCESS)
		return -1;
	if (tx_thread_create(&g_thread, "net-shell", nsh_entry, 0,
	                     g_stack, sizeof g_stack, NSH_PRIORITY, NSH_PRIORITY,
	                     TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS)
		return -1;
	g_ready = 1u;
	return 0;
}

/* True while the module still holds sockets we leaked. */
static int nsh_blocked_dirty(void)
{
	return g_dirty && rtl_link_quiesce_gen() == g_dirty_at;
}

int net_shell_start(uint16_t port, const char **why)
{
	const char *reason = NULL;   /* string literals: valid after we return */

	if (!g_ready) {
		reason = "the telnet console service is not available";
	} else if (g_state != NET_SHELL_OFF) {
		reason = "it is already running";
	} else if (nsh_blocked_dirty()) {
		reason = "the module still holds sockets this console leaked -- "
		         "run `wifi reset` first";
	}
	if (reason != NULL) {
		if (why != NULL)
			*why = reason;
		return -1;
	}
	g_dirty    = 0u;
	g_autoarm  = 1u;
	g_req_port = port;
	g_state    = NET_SHELL_ARMING;   /* state first, then the wake-up (see the note above) */
	(void)tx_event_flags_set(&g_evt, NSH_EVT_CMD, TX_OR);
	return 0;
}

void net_shell_stop(void)
{
	if (!g_ready)
		return;
	g_autoarm  = 0u;      /* an explicit stop must not be undone by the next `net dhcp` */
	g_req_stop = 1u;
	(void)tx_event_flags_set(&g_evt, NSH_EVT_CMD, TX_OR);
}

void net_shell_autoarm(void)
{
	if (!g_ready || !g_autoarm || g_state != NET_SHELL_OFF || nsh_blocked_dirty())
		return;
	g_dirty    = 0u;
	g_req_port = (g_port != 0u) ? g_port : (uint16_t)NET_SHELL_PORT_DEFAULT;
	g_state    = NET_SHELL_ARMING;
	(void)tx_event_flags_set(&g_evt, NSH_EVT_CMD, TX_OR);
}

enum net_shell_state net_shell_state(void)
{
	return g_state;
}

bool net_shell_armed(void)
{
	return g_state != NET_SHELL_OFF;
}

bool net_shell_is_console(const struct cli_instance *sh)
{
	const struct cli_instance *owner;

	if (sh == NULL || g_ctx.sh == NULL)
		return false;
	owner = (sh->fg != NULL) ? sh->fg : sh;      /* a bg job shares its launcher's console */
	return owner == g_ctx.sh;
}

int net_shell_guard(struct cli_instance *sh, const char *what)
{
	if (!net_shell_is_console(sh))
		return 0;
	cli_error(sh, "%s: refused on the telnet console -- it would destroy this session. "
	          "Run it from the USB CDC console.\r\n", what);
	return 1;
}

int net_shell_guard_link(struct cli_instance *sh, const char *what)
{
	/* On the telnet console itself the advice below ("stop it first") is useless -- and
	 * `net shell stop` is refused there anyway -- so give that console its own message. */
	if (net_shell_guard(sh, what))
		return 1;
	if (!net_shell_armed())
		return 0;
	cli_error(sh, "%s: the telnet console already owns the module's one blocking socket "
	          "call (its firmware has two workers) -- run `net shell stop` first\r\n", what);
	return 1;
}

void net_shell_print_status(struct cli_instance *sh)
{
	const char *st;

	switch (g_state) {
	case NET_SHELL_ARMING:    st = "arming";    break;
	case NET_SHELL_LISTENING: st = "listening"; break;
	case NET_SHELL_SESSION:   st = "session";   break;
	case NET_SHELL_STOPPING:  st = "stopping";  break;
	default:                  st = "stopped";   break;
	}
	cli_print(sh, "net shell: %s", st);
	if (g_state == NET_SHELL_LISTENING || g_state == NET_SHELL_SESSION)
		cli_print(sh, " on %u.%u.%u.%u:%u",
		          g_ip[0], g_ip[1], g_ip[2], g_ip[3], (unsigned)g_port);
	cli_print(sh, "\r\n");
	cli_print(sh, "  auto-arm  %s   port %u\r\n",
	          g_autoarm ? "on" : "off", (unsigned)g_port);
	cli_print(sh, "  sessions  %lu   rx %lu B   tx %lu B   rx-drops %lu\r\n",
	          (unsigned long)g_sessions, (unsigned long)g_rx_bytes,
	          (unsigned long)g_tx_bytes, (unsigned long)g_rx_drops);
	cli_print(sh, "  erpc diag crc %u oversize %u timeout %u stale %u unsupported %u "
	          "stall %u ctrl_bad %u\r\n", g_diag_tot.crc_fail, g_diag_tot.oversize,
	          g_diag_tot.timeout, g_diag_tot.skipped_reply,
	          g_diag_tot.unsupported_invocation, g_diag_tot.frame_stall,
	          g_diag_tot.ctrl_bad);
	if (g_last != NULL)
		cli_print(sh, "  last      %s\r\n", g_last);
	if (nsh_blocked_dirty())
		cli_warn(sh, "  the module still holds leaked sockets -- `wifi reset` before "
		         "starting again\r\n");
}
