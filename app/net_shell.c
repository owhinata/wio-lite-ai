/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    net_shell.c
 * @brief   telnet shell console on the host's NetX Duo stack.  See net_shell.h.
 *
 * Three threads touch what is here, and the split between them is the whole design:
 *
 *   server thread (14)   owns the socket.  It is the ONLY caller of create / listen /
 *                        accept / relisten / disconnect / unaccept / unlisten / delete,
 *                        which is why the teardown can prove what it proves.
 *   NetX IP thread (11)  runs the receive / disconnect / window callbacks.  They only
 *                        move bytes into the RX ring and set flags -- never a socket call.
 *   CLI threads (16)     the shell instance and any background job, inside write().
 *
 * The RX ring is strict SPSC: the IP thread is the only producer, the CLI instance thread
 * the only consumer.  There is no TX ring at all -- output goes straight into an NX_PACKET
 * and out through nx_tcp_socket_send(), and back-pressure is TCP's own: a refused send
 * makes write() return short, the core waits on CLI_EVT_TX, and NetX's window-update /
 * queue-depth callbacks wake it.  That is the mechanism the packet pool and the link's
 * DATA transmit pool were sized around (see the transmit budget note in app/nx_net.h).
 *
 * No clock/RCC/register work of its own (XIP-safe).  Clean-room design.
 */
#include "net_shell.h"

#include <stddef.h>
#include <stdint.h>

#include "tx_api.h"
#include "stm32h7xx_hal.h"   /* __get_PRIMASK / __disable_irq / __set_PRIMASK */
#include "nx_api.h"

#include "cli.h"
#include "cli_instance.h"
#include "cli_uart_ring.h"

#include "nx_net.h"

#define LOG_TAG "nshell"
#include "log.h"

/* ---- tunables ------------------------------------------------------------ */

/* Below the nx-net owner (13) and the NetX IP thread (11) it depends on, above the CLI
 * instances (16) so a connection is accepted while a command computes.  Same slot the
 * f746 port uses. */
#define NSH_PRIORITY        14u
/* Sized for first hardware validation, not measured: the accept/dispatch work is shallow
 * (command execution happens in the bound CLI instance, not here), but issue #23 U4-3
 * shrinks it against a `thread` high-water mark rather than a guess. */
#define NSH_STACK           3072u

#define NSH_RX_RING         512u
#define NSH_WINDOW          2048u    /* advertised TCP receive window                */
#define NSH_MSS             1400u    /* bytes per transmitted packet                 */
#define NSH_EXTRACT         1500u    /* per-packet receive extraction buffer         */

/*
 * How long the server sleeps between housekeeping passes when nothing is happening.  It is
 * NOT how quickly a request is noticed -- every request raises a flag this wait is armed
 * on -- it only bounds how long an unannounced change (the host stack going away, a
 * handshake NetX abandoned) can go unseen.
 *
 * There is no polling here on purpose.  nx_tcp_socket_state_wait() would have been the
 * obvious way to wait for a connection and it is what issue #23 U4-1's `net echo` uses,
 * but it is a tx_thread_sleep(1) loop -- 1000 wake-ups a second, for ever, on a console
 * that is idle almost all of its life.  A command can afford that; a resident service
 * cannot.  nx_tcp_socket_establish_notify() gives the same information as an event.
 */
#define NSH_IDLE_MS         1000u
#define NSH_DISC_MS         1000u    /* bounded FIN handshake                        */

/*
 * Packets of the shared pool this console will not take.  Its output bursts (a `dmesg`, a
 * `membench` table) must not be able to starve the receive path, which allocates with
 * NX_NO_WAIT and drops the frame when it fails.  Advisory -- read without a lock -- which
 * is all it needs to be.
 */
#define NSH_POOL_RESERVE    8u

/* Service thread event flags. */
#define NSH_EVT_CMD         0x1u    /* a start/stop request was posted   */
#define NSH_EVT_DISC        0x2u    /* the peer disconnected             */
#define NSH_EVT_DONE        0x4u    /* teardown finished AND was proved  */
#define NSH_EVT_CONN        0x8u    /* the socket reached ESTABLISHED    */

/* ---- state --------------------------------------------------------------- */

struct nsh_ctx {
	struct cli_instance *sh;             /* set by nsh_init() from tr->sh          */
	struct cli_uart_ring rx;             /* IP thread -> CLI thread (SPSC)         */
	uint8_t              rx_buf[NSH_RX_RING];
	volatile uint8_t     connected;      /* write gate; owned by the CLI thread    */
};

static struct nsh_ctx       g_ctx;
static TX_THREAD            g_thread;
static TX_EVENT_FLAGS_GROUP g_evt;
static UCHAR                g_stack[NSH_STACK] __attribute__((aligned(8)));
static uint8_t              g_ready;

static NX_TCP_SOCKET        g_sock;
static uint8_t              g_sock_created;   /* server thread only */

/*
 * Serialises "the socket is usable" between the CLI threads inside write() and the server
 * thread's teardown.  Held across short socket calls, with ONE exception: the bounded FIN
 * handshake in nsh_unwind(), which must not race a delete.  nsh_write() pays for that
 * exception by re-checking `connected` when its TX_NO_WAIT acquisition fails.
 */
static TX_MUTEX             g_sock_lock;

/* Owned by the server thread (others only read). */
static volatile enum net_shell_state g_state;
static uint16_t  g_port = NET_SHELL_PORT_DEFAULT;
static int       g_iac;                          /* telnet IAC receive state          */
static uint8_t   g_extract[NSH_EXTRACT];         /* IP thread only                    */
static uint8_t   g_stage[2u * NSH_MSS];          /* write() escaping; under g_sock_lock */
static uint8_t   g_ip[4];                        /* address we listen on, for status  */

/* Set by the disconnect callback (IP thread). */
static volatile uint8_t g_peer_gone;

/*
 * A connection is currently accepted.  Set before CLI_EVT_CONN is posted and cleared on
 * disconnect, so a session_begin() the CLI thread only reaches AFTER the client vanished
 * does not resurrect the write gate on a dead socket.
 */
static volatile uint8_t g_link_live;

/* Requests from command threads. */
static volatile uint8_t  g_req_stop;
static volatile uint8_t  g_autoarm = 1u;
static volatile uint16_t g_req_port = NET_SHELL_PORT_DEFAULT;

static const char *g_last = "never started";     /* last state-changing reason        */
static uint32_t g_sessions, g_rx_bytes, g_tx_bytes, g_rx_drops;
/*
 * Why a write() was not accepted, kept apart because the three answers mean opposite
 * things and a single "refused" counter says nothing:
 *
 *   g_tx_wait   the TCP window or the socket's transmit queue is full.  This is the
 *               DESIGNED back-pressure -- the core waits on CLI_EVT_TX and NetX's
 *               window/queue notify resumes it -- so a large number here is health, not
 *               trouble.  A 64 kB `dmesg` over telnet produces thousands.
 *   g_tx_nobuf  the packet pool was empty or below this console's reserve.  THIS is the
 *               sizing signal: it means output was competing with the receive path for
 *               packets, which is what NXN_TCP_TX_DEPTH and the pool were sized to avoid.
 *   g_tx_busy   the socket mutex was held -- only the teardown does that.
 */
static uint32_t g_tx_wait, g_tx_nobuf, g_tx_busy;

/* ---- transport vtable ---------------------------------------------------- */

static int nsh_init(struct cli_transport *tr)
{
	cli_uart_ring_init(&g_ctx.rx, g_ctx.rx_buf, sizeof g_ctx.rx_buf);
	g_ctx.connected = 0;
	g_ctx.sh        = tr->sh;      /* cli_init() sets tr->sh before calling init */
	return 0;
}

static int nsh_enable(struct cli_transport *tr)
{
	(void)tr;                      /* the socket is opened by the server thread */
	return 0;
}

/*
 * Put @n bytes on the wire verbatim.  MUST be called with g_sock_lock held.  Returns the
 * number of bytes accepted (0 when the pool or the socket refused).
 */
static int nsh_emit_locked(const uint8_t *p, ULONG n)
{
	NX_PACKET_POOL *pool = (NX_PACKET_POOL *)nx_net_pool();
	NX_PACKET *pkt = NX_NULL;
	UINT s;

	if (pool == NX_NULL || !g_sock_created)
		return 0;
	/* Leave the receive path some pool.  Advisory: an unlocked read of a counter NetX
	 * maintains, which is the right weight for a policy whose only job is to keep an
	 * output burst from being the reason a frame was dropped. */
	if (pool->nx_packet_pool_available <= NSH_POOL_RESERVE) {
		g_tx_nobuf++;
		return 0;
	}
	if (nx_packet_allocate(pool, &pkt, NX_TCP_PACKET, NX_NO_WAIT) != NX_SUCCESS) {
		g_tx_nobuf++;
		return 0;
	}
	if (nx_packet_data_append(pkt, (VOID *)p, n, pool, NX_NO_WAIT) != NX_SUCCESS) {
		nx_packet_release(pkt);
		g_tx_nobuf++;
		return 0;
	}
	s = nx_tcp_socket_send(&g_sock, pkt, NX_NO_WAIT);
	if (s != NX_SUCCESS) {
		/* Release it -- NetX did not take it -- and let the core wait for the notify.
		 * The window/queue answers are the healthy path and are counted apart from
		 * anything else (NX_NOT_CONNECTED on a disconnect race lands in g_tx_nobuf,
		 * which is harmless: the session is ending anyway). */
		nx_packet_release(pkt);
		if (s == NX_WINDOW_OVERFLOW || s == NX_TX_QUEUE_DEPTH)
			g_tx_wait++;
		else
			g_tx_nobuf++;
		return 0;
	}
	return (int)n;
}

/*
 * ---- the write path, and the one rule it must not break ------------------------
 *
 * cli_transport_api.write() is a NON-BLOCKING contract (cli_instance.h): it returns how
 * much it took and the core waits on CLI_EVT_TX for the rest.  So g_sock_lock is acquired
 * with TX_NO_WAIT and a failure returns 0 rather than waiting -- during a teardown the
 * server holds it only across short socket calls, and the teardown clears `connected` and
 * fires the TX notify, so the core re-enters write() and gets `len` back on the closed
 * gate instead of waiting out CLI_TX_TIMEOUT.
 *
 * A literal 0xFF must go out as IAC IAC or a telnet client reads it as the start of a
 * command (f746's nx_shell strips IAC on receive but never escaped its output, which
 * corrupts any binary the shell prints).  The escape happens into a static staging buffer,
 * which is safe precisely because g_sock_lock is held: write() has three possible callers
 * (the instance thread, a background job through sh->fg, and session_begin(), which the
 * core calls WITHOUT its output lock).
 */
static int nsh_write(struct cli_transport *tr, const uint8_t *data, size_t len)
{
	size_t i, out = 0, taken = 0;
	int sent;

	(void)tr;
	/* Not connected: swallow it.  Returning `len` is what keeps the shell from ever
	 * wedging on a console nobody is attached to. */
	if (!g_ctx.connected)
		return (int)len;
	if (len == 0u)
		return 0;

	if (tx_mutex_get(&g_sock_lock, TX_NO_WAIT) != TX_SUCCESS) {
		/*
		 * The holder is the teardown, which DOES wait under this lock (the bounded
		 * FIN handshake in nsh_unwind()).  It clears `connected` first, but we may
		 * have read it just before that -- so re-read it here.  Without this the core
		 * would wait out CLI_TX_TIMEOUT for space that is never coming, on a session
		 * that has already ended.
		 */
		g_tx_busy++;
		return g_ctx.connected ? 0 : (int)len;
	}

	for (i = 0; i < len; i++) {
		size_t need = (data[i] == 0xFFu) ? 2u : 1u;

		if (out + need > NSH_MSS)
			break;                       /* one packet's worth per call */
		g_stage[out++] = data[i];
		if (need == 2u)
			g_stage[out++] = 0xFFu;
		taken++;
	}
	sent = nsh_emit_locked(g_stage, (ULONG)out);
	(void)tx_mutex_put(&g_sock_lock);

	if (sent <= 0)
		return 0;                            /* core waits for CLI_EVT_TX */
	g_tx_bytes += (uint32_t)out;
	return (int)taken;                           /* INPUT bytes consumed */
}

static int nsh_read(struct cli_transport *tr, uint8_t *data, size_t cap)
{
	(void)tr;
	/* SPSC: the IP-thread callback is the only producer, this (the CLI thread) the
	 * only consumer -- lock-free, like the UART/CDC backends. */
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

	(void)tr;
	/* The CLI thread calls this on CLI_EVT_CONN, after the editor state was reset and
	 * before the prompt is drawn.  Drain the previous session's leftovers HERE, on the
	 * consumer side, so the RX ring stays strict SPSC. */
	while (cli_uart_ring_get(&g_ctx.rx, &junk))
		;
	/* Enable output only if the connection is still up: a client that vanished before
	 * the CLI thread got here leaves g_link_live == 0, so the prompt is dropped rather
	 * than written to a socket that is gone. */
	g_ctx.connected = g_link_live;
	if (!g_ctx.connected)
		return;
	/* Raw: these ARE IAC sequences and must not be escaped. */
	if (tx_mutex_get(&g_sock_lock, TX_NO_WAIT) == TX_SUCCESS) {
		(void)nsh_emit_locked(nsh_charmode, (ULONG)sizeof nsh_charmode);
		(void)tx_mutex_put(&g_sock_lock);
	}
}

static const struct cli_transport_api nsh_api = {
	nsh_init, nsh_enable, nsh_write, nsh_read, NULL, NULL, nsh_session_begin,
};

struct cli_transport net_shell_transport = {
	.api = &nsh_api,
	.sh  = NULL,                   /* set by cli_init() */
	.ctx = &g_ctx,
};

/* ---- NetX callbacks (IP-thread context: ring + flags only) --------------- */

/*
 * Put one byte into the RX ring.
 *
 * The ring is lock-free and leaves concurrency to the backend (cli_uart_ring.h), and this
 * backend has TWO producers: the receive callback on the NetX IP thread, and the server
 * thread injecting a Ctrl+C when a client vanishes.  Only the consumer side is naturally
 * exclusive (one CLI instance thread, touching only `tail`), so the producers serialise
 * with the same short interrupt-masked region the USB CDC backend uses.  Returns 1 if
 * stored.
 */
static int nsh_rx_put(uint8_t b)
{
	uint32_t pm = __get_PRIMASK();
	int ok;

	__disable_irq();
	ok = cli_uart_ring_put(&g_ctx.rx, b);
	__set_PRIMASK(pm);
	return ok;
}

/* Drop 0xFF + command [+ option]; 0xFF 0xFF becomes one literal 0xFF.  Returns 1 when @b
 * is consumed (negotiation), 0 when it is shell data. */
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

static void nsh_rx_notify(NX_TCP_SOCKET *s)
{
	NX_PACKET *pkt;
	int woke = 0;

	while (nx_tcp_socket_receive(s, &pkt, NX_NO_WAIT) == NX_SUCCESS) {
		ULONG copied = 0, off = 0;

		while (off < pkt->nx_packet_length) {
			ULONG got = 0, i;

			if (nx_packet_data_extract_offset(pkt, off, g_extract,
			                                  sizeof g_extract, &got) != NX_SUCCESS ||
			    got == 0)
				break;
			off += got;
			copied += got;
			for (i = 0; i < got; i++) {
				if (nsh_iac_consume(g_extract[i]))
					continue;
				if (!nsh_rx_put(g_extract[i])) {
					g_rx_drops++;   /* ring full: byte lost, stream in sync */
					continue;
				}
				woke = 1;
			}
		}
		g_rx_bytes += (uint32_t)copied;
		nx_packet_release(pkt);
	}
	if (woke && g_ctx.sh != NULL)
		cli_transport_notify_rx(g_ctx.sh);
}

static void nsh_disconnect_cb(NX_TCP_SOCKET *s)
{
	(void)s;
	/* Peer FIN/RST.  Stop output, reset the telnet IAC state (same IP-thread domain as
	 * nsh_iac_consume) and wake the server thread, which completes the close and
	 * relistens. */
	g_peer_gone     = 1u;
	g_link_live     = 0u;
	g_ctx.connected = 0;
	g_iac           = 0;
	(void)tx_event_flags_set(&g_evt, NSH_EVT_DISC, TX_OR);
}

/* The handshake completed.  IP-thread context (_nx_tcp_socket_state_syn_received calls it
 * the instant the state becomes ESTABLISHED): flag only. */
static void nsh_establish_cb(NX_TCP_SOCKET *s)
{
	(void)s;
	(void)tx_event_flags_set(&g_evt, NSH_EVT_CONN, TX_OR);
}

static void nsh_tx_notify(NX_TCP_SOCKET *s)
{
	(void)s;                            /* window / queue space freed */
	if (g_ctx.sh != NULL)
		cli_transport_notify_tx(g_ctx.sh);
}

/* ---- socket lifecycle (server thread only) ------------------------------- */

/*
 * Arm the passive open.  CALL EXACTLY ONCE PER CONNECTION.
 *
 * Re-entering nx_tcp_server_socket_accept() is NOT a legal way to stay cancellable: every
 * entry from LISTEN regenerates the ISN and moves the socket to SYN_RECEIVED
 * (nx_tcp_server_socket_accept.c:102-124), and the timeout path restores LISTEN WITHOUT
 * unbinding, leaving a bound-but-LISTEN socket whose completing ACK
 * _nx_tcp_socket_packet_process() drops -- it has no LISTEN case
 * (nx_tcp_socket_packet_process.c:334-449).  Issue #23 U4-1 found that on hardware: the
 * client connects and receives nothing, for ever.  So arm once with NX_NO_WAIT and wait
 * with nx_tcp_socket_state_wait(), which touches nothing and may be re-entered freely.
 */
static int nsh_arm_accept(void)
{
	UINT s = nx_tcp_server_socket_accept(&g_sock, NX_NO_WAIT);

	if (s == NX_SUCCESS || s == NX_IN_PROGRESS)
		return 0;
	LOG_ERR("accept arm failed (0x%02x)", (unsigned)s);
	return -1;
}

/* Create the socket and start listening.  Returns 0 with the state left LISTENING. */
static int nsh_arm(uint16_t port)
{
	NX_IP *ip = (NX_IP *)nx_net_ip();
	struct nx_net_info ni;
	UINT s;

	if (ip == NX_NULL || !nx_net_is_up()) {
		g_last = "the host stack is not up (`net up` first)";
		goto fail;
	}
	/*
	 * A socket NetX still has on its created list must never be created over: the
	 * control block IS the list node, so a second create would corrupt it.  That only
	 * happens if a previous delete was refused, which is already reported as an error --
	 * this is the latch that stops it becoming silent memory corruption later.
	 */
	if (g_sock_created) {
		g_last = "the previous socket was never released (reset to clear)";
		LOG_ERR("refusing to re-create: the socket is still on the IP instance");
		g_state = NET_SHELL_OFF;
		return -1;
	}

	s = nx_tcp_socket_create(ip, &g_sock, "net-shell", NX_IP_NORMAL, NX_FRAGMENT_OKAY,
	                         NX_IP_TIME_TO_LIVE, NSH_WINDOW, NX_NULL, nsh_disconnect_cb);
	if (s != NX_SUCCESS) {
		LOG_ERR("socket create failed (0x%02x)", (unsigned)s);
		g_last = "socket create failed";
		goto fail;
	}
	g_sock_created = 1u;

	(void)nx_tcp_socket_receive_notify(&g_sock, nsh_rx_notify);
	/* Available because NX_DISABLE_EXTENDED_NOTIFY_SUPPORT is not defined; it is what
	 * lets this thread wait on an event instead of polling for ESTABLISHED. */
	(void)nx_tcp_socket_establish_notify(&g_sock, nsh_establish_cb);
	(void)nx_tcp_socket_window_update_notify_set(&g_sock, nsh_tx_notify);
	(void)nx_tcp_socket_queue_depth_notify_set(&g_sock, nsh_tx_notify);
	/* Bound what this socket can have outstanding towards the link's DATA transmit
	 * pool -- the transmit budget note in app/nx_net.h is what makes that number safe. */
	(void)nx_tcp_socket_transmit_configure(&g_sock, NXN_TCP_TX_DEPTH, NXN_TCP_RTO_MS,
	                                       NXN_TCP_RTO_RETRIES, NXN_TCP_RTO_SHIFT);

	s = nx_tcp_server_socket_listen(ip, port, &g_sock, 1, NX_NULL);
	if (s != NX_SUCCESS) {
		LOG_ERR("listen :%u failed (0x%02x)", (unsigned)port, (unsigned)s);
		g_last = "listen failed (port in use?)";
		goto fail;
	}
	g_peer_gone = 0u;
	if (nsh_arm_accept() != 0) {
		(void)nx_tcp_server_socket_unlisten(ip, port);
		g_last = "could not arm accept";
		goto fail;
	}

	if (nx_net_info_get(&ni) == NXN_OK && ni.ip_valid) {
		g_ip[0] = (uint8_t)(ni.ip >> 24); g_ip[1] = (uint8_t)(ni.ip >> 16);
		g_ip[2] = (uint8_t)(ni.ip >> 8);  g_ip[3] = (uint8_t)ni.ip;
	}
	g_port  = port;
	g_iac   = 0;
	g_last  = "listening";
	g_state = NET_SHELL_LISTENING;
	LOG_INF("listening on %u.%u.%u.%u:%u (telnet, host stack)",
	        g_ip[0], g_ip[1], g_ip[2], g_ip[3], (unsigned)port);
	return 0;

fail:
	if (g_sock_created) {
		(void)tx_mutex_get(&g_sock_lock, TX_WAIT_FOREVER);
		if (nx_tcp_socket_delete(&g_sock) == NX_SUCCESS)
			g_sock_created = 0u;
		(void)tx_mutex_put(&g_sock_lock);
	}
	LOG_WRN("arm failed: %s", g_last);
	g_state = NET_SHELL_OFF;
	return -1;
}

/*
 * Unwind the socket and PROVE it.  Returns 0 only when nx_tcp_socket_delete() succeeded,
 * which is what net_shell_stop_sync()'s contract rests on.
 *
 * The order is forced by nx_tcp_socket_delete(), which refuses anything still bound or not
 * in NX_TCP_CLOSED (nx_tcp_socket_delete.c:93).  disconnect + unaccept are issued
 * UNCONDITIONALLY, not only when a session was accepted, because an armed passive open can
 * be bound too: nx_tcp_server_socket_relisten() binds the socket when a SYN was already
 * queued.  Between them the pair covers every state -- ESTABLISHED (disconnect leaves
 * >= CLOSE_WAIT or LISTEN, unaccept forces LISTEN then unbinds), bound LISTEN (unaccept
 * unbinds via bound_next), unbound LISTEN (unaccept clears the listen request).  unaccept
 * MUST precede unlisten: it clears the request's socket pointer, unlisten removes the
 * request.
 */
static int nsh_unwind(void)
{
	NX_IP *ip = (NX_IP *)nx_net_ip();
	UINT s;

	/* Stop producers first, and wake anyone parked in the core's TX wait so it
	 * re-enters write() and returns on the closed gate. */
	g_ctx.connected = 0;
	g_link_live     = 0u;
	if (g_ctx.sh != NULL)
		cli_transport_notify_tx(g_ctx.sh);

	if (!g_sock_created)
		return 0;

	/*
	 * The lock keeps a CLI thread from being inside nx_tcp_socket_send() while the
	 * socket is deleted.  It IS held across the bounded FIN handshake below -- the one
	 * place this file waits under it -- which is why nsh_write() re-checks `connected`
	 * when its TX_NO_WAIT acquisition fails: a writer must not be told "no space" for a
	 * second while the session it belongs to is being dismantled.
	 */
	(void)tx_mutex_get(&g_sock_lock, TX_WAIT_FOREVER);
	(void)nx_tcp_socket_disconnect(&g_sock, NSH_DISC_MS);
	(void)nx_tcp_server_socket_unaccept(&g_sock);
	if (ip != NX_NULL)
		(void)nx_tcp_server_socket_unlisten(ip, g_port);
	s = nx_tcp_socket_delete(&g_sock);
	if (s == NX_SUCCESS)
		g_sock_created = 0u;
	(void)tx_mutex_put(&g_sock_lock);

	if (s != NX_SUCCESS) {
		LOG_ERR("socket delete refused (0x%02x) -- the console may still transmit",
		        (unsigned)s);
		return -1;
	}
	return 0;
}

/* End the current session and go back to listening.  Returns 0, or -1 if the socket could
 * not be re-armed (the caller then stops). */
static int nsh_end_session(const char *why)
{
	NX_IP *ip = (NX_IP *)nx_net_ip();
	UINT s;

	g_ctx.connected = 0;
	g_link_live     = 0u;
	if (g_ctx.sh != NULL)
		cli_transport_notify_tx(g_ctx.sh);
	/* A command may still be running for the client that just left.  0x03 is exactly
	 * what Ctrl+C delivers, so pushing one cancels it and leaves the instance idle for
	 * the next client instead of holding the console until it finishes on its own. */
	if (nsh_rx_put(0x03u) && g_ctx.sh != NULL)
		cli_transport_notify_rx(g_ctx.sh);

	(void)tx_mutex_get(&g_sock_lock, TX_WAIT_FOREVER);
	(void)nx_tcp_socket_disconnect(&g_sock, NSH_DISC_MS);
	(void)nx_tcp_server_socket_unaccept(&g_sock);
	(void)tx_mutex_put(&g_sock_lock);

	if (ip == NX_NULL)
		return -1;
	s = nx_tcp_server_socket_relisten(ip, g_port, &g_sock);
	if (s != NX_SUCCESS && s != NX_CONNECTION_PENDING) {
		LOG_ERR("relisten failed (0x%02x)", (unsigned)s);
		return -1;
	}
	g_peer_gone = 0u;
	if (nsh_arm_accept() != 0)
		return -1;

	g_iac   = 0;
	g_last  = why;
	g_state = NET_SHELL_LISTENING;
	LOG_INF("client disconnected (%s)", why);
	return 0;
}

/* ---- server thread ------------------------------------------------------- */

/* Complete a stop: unwind, publish the outcome, and raise NSH_EVT_DONE only if the unwind
 * was PROVED.  A caller waiting in net_shell_stop_sync() must not be told "stopped" on the
 * strength of an attempt. */
static void nsh_do_stop(const char *why)
{
	g_state = NET_SHELL_STOPPING;
	if (nsh_unwind() != 0) {
		g_last  = "the socket could not be released -- the console may still transmit";
		g_state = NET_SHELL_OFF;      /* nothing left to do here; the latch in
		                               * nsh_arm()/net_shell_start() refuses a re-arm
		                               * while the old socket is still on the list */
		LOG_ERR("stop could not be proved (%s)", why);
		return;                       /* deliberately NO NSH_EVT_DONE */
	}
	g_last  = why;
	g_state = NET_SHELL_OFF;
	LOG_INF("stopped (%s)", why);
	(void)tx_event_flags_set(&g_evt, NSH_EVT_DONE, TX_OR);
}

static void nsh_entry(ULONG arg)
{
	(void)arg;

	for (;;) {
		ULONG f;
		UINT  st;

		if (g_req_stop && g_state != NET_SHELL_ARMING) {
			g_req_stop = 0u;
			g_autoarm  = 0u;           /* an explicit stop stays stopped */
			if (g_state != NET_SHELL_OFF)
				nsh_do_stop("requested");
			else
				(void)tx_event_flags_set(&g_evt, NSH_EVT_DONE, TX_OR);
			continue;
		}

		switch (g_state) {
		case NET_SHELL_OFF:
			/* Sticky flags: a request landing between the switch and this wait
			 * still wakes us, because the requester moves the state FIRST and
			 * raises NSH_EVT_CMD second. */
			(void)tx_event_flags_get(&g_evt, NSH_EVT_CMD, TX_OR_CLEAR, &f,
			                         TX_WAIT_FOREVER);
			break;

		case NET_SHELL_ARMING:
			if (nsh_arm(g_req_port) != 0 && g_state == NET_SHELL_ARMING)
				g_state = NET_SHELL_OFF;   /* belt and braces: never spin here */
			break;

		case NET_SHELL_LISTENING:
			(void)tx_event_flags_get(&g_evt, NSH_EVT_CONN | NSH_EVT_CMD,
			                         TX_OR_CLEAR, &f, NSH_IDLE_MS);
			if (g_req_stop)
				break;                 /* handled at the top of the loop */
			/*
			 * The state is the truth, not the flag: the connection may have
			 * completed just before the wait was armed, or the wait may have timed
			 * out with it complete.  Reading it here covers both.
			 */
			st = g_sock.nx_tcp_socket_state;
			if (st != NX_TCP_ESTABLISHED) {
				if (st != NX_TCP_LISTEN_STATE && st != NX_TCP_SYN_RECEIVED) {
					LOG_WRN("left the passive open (state %u) -- re-arming",
					        (unsigned)st);
					if (nsh_end_session("re-armed") != 0)
						nsh_do_stop("could not re-arm the socket");
				}
				/* The host stack going away takes the socket with it. */
				if (!nx_net_is_up())
					nsh_do_stop("the host stack went down");
				break;
			}
			g_peer_gone = 0u;
			g_link_live = 1u;
			g_sessions++;
			g_state = NET_SHELL_SESSION;
			g_last  = "client connected";
			if (g_ctx.sh != NULL)
				cli_transport_notify_conn(g_ctx.sh);
			LOG_INF("client connected (session %lu)", (unsigned long)g_sessions);
			break;

		case NET_SHELL_SESSION:
			/* Nothing to do while a client is attached: the callbacks move the
			 * bytes.  Wait for the disconnect, waking periodically so a stop
			 * request and a vanished interface are both noticed. */
			(void)tx_event_flags_get(&g_evt, NSH_EVT_DISC | NSH_EVT_CMD,
			                         TX_OR_CLEAR, &f, NSH_IDLE_MS);
			if (g_req_stop)
				break;                 /* handled at the top of the loop */
			if (!nx_net_is_up()) {
				nsh_do_stop("the host stack went down");
				break;
			}
			if (g_peer_gone || g_sock.nx_tcp_socket_state != NX_TCP_ESTABLISHED) {
				if (nsh_end_session("peer closed") != 0)
					nsh_do_stop("could not re-arm after the client left");
			}
			break;

		default:
			g_state = NET_SHELL_OFF;
			break;
		}
	}
}

/* ---- public API ---------------------------------------------------------- */

int net_shell_init(void)
{
	if (tx_event_flags_create(&g_evt, "nshell") != TX_SUCCESS)
		return -1;
	if (tx_mutex_create(&g_sock_lock, "nshsock", TX_INHERIT) != TX_SUCCESS)
		return -1;
	if (tx_thread_create(&g_thread, "net-shell", nsh_entry, 0,
	                     g_stack, sizeof g_stack, NSH_PRIORITY, NSH_PRIORITY,
	                     TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS)
		return -1;
	g_ready = 1u;
	return 0;
}

int net_shell_start(uint16_t port, const char **why)
{
	const char *reason = NULL;   /* string literals: valid after we return */

	if (!g_ready)
		reason = "the telnet console service is not available";
	else if (g_state != NET_SHELL_OFF)
		reason = "it is already running";
	else if (g_sock_created)
		reason = "its previous socket could not be released; reset the board";
	else if (!nx_net_is_up())
		reason = "the telnet console runs on the host stack -- `net up` first "
		         "(issue #23 U4 moved it off the module's lwIP)";
	if (reason != NULL) {
		if (why != NULL)
			*why = reason;
		return -1;
	}
	g_autoarm  = 1u;
	g_req_port = port;
	g_state    = NET_SHELL_ARMING;   /* state first, then the wake-up */
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

int net_shell_stop_sync(uint32_t timeout_ms)
{
	ULONG f;
	uint32_t waited = 0u;

	if (!g_ready)
		return 0;
	/*
	 * OFF is NOT the same as "proved stopped".  A refused nx_tcp_socket_delete() leaves
	 * the state OFF -- there is nothing left for the server thread to do -- while the
	 * socket is still on the IP instance and NetX can still hand this driver a frame.
	 * Returning 0 there would let nxn_stop() detach the DATA channel underneath a live
	 * socket, which is the exact failure this whole contract exists to prevent.  There
	 * is nothing further to try, so say so at once rather than after the timeout.
	 */
	if (g_sock_created && g_state == NET_SHELL_OFF) {
		LOG_ERR("cannot prove the stop: the socket was never released");
		return -1;
	}
	if (g_state == NET_SHELL_OFF)
		return 0;

	/* Clear any stale completion before asking, so what we wait for is OURS. */
	(void)tx_event_flags_get(&g_evt, NSH_EVT_DONE, TX_OR_CLEAR, &f, TX_NO_WAIT);
	net_shell_stop();

	while (waited < timeout_ms) {
		ULONG slice = (timeout_ms - waited > NSH_IDLE_MS) ? NSH_IDLE_MS
		                                                  : (timeout_ms - waited);

		if (tx_event_flags_get(&g_evt, NSH_EVT_DONE, TX_OR_CLEAR, &f, slice) ==
		    TX_SUCCESS)
			return 0;
		waited += (uint32_t)slice;
	}
	LOG_ERR("stop could not be proved within %lu ms (state %u)",
	        (unsigned long)timeout_ms, (unsigned)g_state);
	return -1;
}

void net_shell_autoarm(void)
{
	if (!g_ready || !g_autoarm || g_state != NET_SHELL_OFF || !nx_net_is_up())
		return;
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
	/* A socket that outlived a refused delete still counts: it is still on the IP
	 * instance, and every caller of this asks in order to decide whether something
	 * else may proceed. */
	return g_state != NET_SHELL_OFF || g_sock_created != 0u;
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
	cli_print(sh, "net shell: %s (host stack)", st);
	if (g_state == NET_SHELL_LISTENING || g_state == NET_SHELL_SESSION)
		cli_print(sh, " on %u.%u.%u.%u:%u",
		          g_ip[0], g_ip[1], g_ip[2], g_ip[3], (unsigned)g_port);
	cli_print(sh, "\r\n");
	cli_print(sh, "  auto-arm  %s   port %u\r\n",
	          g_autoarm ? "on" : "off", (unsigned)g_port);
	cli_print(sh, "  sessions  %lu   rx %lu B   tx %lu B   rx-drops %lu\r\n",
	          (unsigned long)g_sessions, (unsigned long)g_rx_bytes,
	          (unsigned long)g_tx_bytes, (unsigned long)g_rx_drops);
	cli_print(sh, "  tx waits  %lu (back-pressure, normal)   no-buf %lu%s   busy %lu\r\n",
	          (unsigned long)g_tx_wait, (unsigned long)g_tx_nobuf,
	          g_tx_nobuf ? "  <-- pool too small" : "", (unsigned long)g_tx_busy);
	if (g_last != NULL)
		cli_print(sh, "  last      %s\r\n", g_last);
	/*
	 * The same report to the log.  The console's own status is most wanted exactly when
	 * that console is in trouble, and cli_tx_send_blocking() discards a handler's output
	 * once Ctrl+C is latched (cli_core.c:456-459) -- issue #23 U4-1 lost a whole
	 * hardware measurement to that.  `dmesg` consults neither the output path nor
	 * cancel_req, and survives a reset.
	 */
	LOG_INF("status: %s, port %u, auto-arm %s, sessions %lu, rx %lu B, tx %lu B, "
	        "rx-drops %lu, tx waits %lu, no-buf %lu, busy %lu, last: %s",
	        st, (unsigned)g_port, g_autoarm ? "on" : "off", (unsigned long)g_sessions,
	        (unsigned long)g_rx_bytes, (unsigned long)g_tx_bytes,
	        (unsigned long)g_rx_drops, (unsigned long)g_tx_wait,
	        (unsigned long)g_tx_nobuf, (unsigned long)g_tx_busy,
	        g_last ? g_last : "?");
}
