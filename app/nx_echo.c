/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/*
 * TCP echo server on the host's own NetX Duo stack (issue #23 U4-1).  See app/nx_echo.h.
 *
 * Everything here runs on the calling CLI instance thread, whose stack is 2048 bytes
 * (shell/include/cli_config.h).  NX_TCP_SOCKET is large and the extraction buffer is a
 * whole MSS, so both are file-scope statics -- nothing sizeable goes on that stack.  Only
 * one echo server can run at a time as a result, which is enforced by nxe_busy.
 *
 * Cancellation is real here, unlike the eRPC version this replaces: every wait is bounded
 * and Ctrl+C is checked between them, so `net echo` stops within about a quarter of a
 * second instead of one 12-second accept window.  That difference is the whole point of
 * owning the stack -- there is no far end still running a call we abandoned, so there is
 * nothing to leak and no "link dirty" state to latch.
 *
 * No clock/RCC/register work of its own (clock-safe).  Clean-room design.
 */
#include "nx_echo.h"

#include <stddef.h>

#include "nx_api.h"

#include "stm32h7xx_hal.h"     /* HAL_GetTick + CMSIS __get_PRIMASK/__disable_irq */

#include "cli.h"
#include "cli_instance.h"
#include "link_data.h"
#include "nx_link_driver.h"
#include "nx_net.h"

#define LOG_TAG "nxecho"
#include "log.h"

/* ---- tunables --------------------------------------------------------------- */

#define NXE_WINDOW        2048u    /* advertised TCP receive window                  */
#define NXE_MSS           1400u    /* bytes per transmitted packet                   */
#define NXE_ACCEPT_SLICE  250u     /* ms per connect-wait slice, so Ctrl+C is prompt */
/* Liveness heartbeat: soon enough to answer "is this wedged or just idle?" (2 s), then
 * rare enough not to bury the session lines (30 s). */
#define NXE_TICK_FIRST    8u
#define NXE_TICK_REPEAT   120u
#define NXE_RECV_SLICE    250u     /* ms per receive wait, same reason               */
#define NXE_SEND_SLICE    250u     /* ms per send attempt before re-checking Ctrl+C   */
#define NXE_SEND_TRIES    40u      /* x NXE_SEND_SLICE = 10 s before giving up        */
#define NXE_DISC_MS       1000u    /* bounded FIN handshake                          */

/* ---- state (single instance; nxe_busy is the interlock) --------------------- */

static NX_TCP_SOCKET nxe_sock;
static uint8_t       nxe_buf[NXE_MSS];
static volatile uint8_t nxe_busy;
static volatile uint8_t nxe_peer_gone;   /* set by the disconnect callback (IP thread) */

/*
 * Claim the single instance.  There are two consoles, so "read it, then set it" really can
 * be run by two threads at once; the interrupt-masked test-and-set is the same one-byte
 * critical section the CDC and telnet transports use for their rings.  Returns 1 on
 * success, 0 if an echo server is already running.
 */
static int nxe_claim(void)
{
	uint32_t pm = __get_PRIMASK();
	int got;

	__disable_irq();
	got = !nxe_busy;
	if (got)
		nxe_busy = 1u;
	__set_PRIMASK(pm);
	return got;
}

struct nxe_run {
	struct cli_instance *sh;
	uint32_t sessions;
	uint32_t rx, tx;                 /* payload bytes                              */
	uint32_t send_retries;           /* sends that had to wait for window/queue     */
	uint32_t send_fails;             /* packets dropped after NXE_SEND_TRIES        */
	uint32_t alloc_fails;            /* pool empty when building a reply            */
	ULONG    pool_low;               /* lowest observed free packet count           */
};

/* ---- helpers ---------------------------------------------------------------- */

static void nxe_pool_sample(struct nxe_run *r, NX_PACKET_POOL *pool)
{
	ULONG total = 0, avail = 0, empty = 0, waits = 0, invalid = 0;

	if (nx_packet_pool_info_get(pool, &total, &avail, &empty, &waits, &invalid) !=
	    NX_SUCCESS)
		return;
	if (avail < r->pool_low)
		r->pool_low = avail;
}

/* The peer sent FIN/RST.  IP-thread context: flag only. */
static void nxe_disconnect_cb(NX_TCP_SOCKET *s)
{
	(void)s;
	nxe_peer_gone = 1u;
}

/*
 * Arm the passive open.  CALL THIS EXACTLY ONCE PER CONNECTION.
 *
 * ---- why this is a separate function with a warning on it ----------------------
 *
 * The obvious shape -- nx_tcp_server_socket_accept(&sock, 250 ms) called round and round
 * so that Ctrl+C stays responsive -- is WRONG on NetX, and it cost a board session to
 * find out.  Every entry from NX_TCP_LISTEN_STATE regenerates the initial sequence number
 * and moves the socket to SYN_RECEIVED (nx_tcp_server_socket_accept.c:102-124), and the
 * timeout path puts it back to LISTEN (:183-194) WITHOUT unbinding it.  So a SYN that
 * arrived during the slice leaves the socket bound-but-LISTEN -- and
 * _nx_tcp_socket_packet_process()'s state switch has NO LISTEN case
 * (nx_tcp_socket_packet_process.c:334-449), so the ACK that would finish the handshake is
 * dropped on the floor.  For ever.  Observed as: the client connects and receives nothing,
 * and the server never reports a session.
 *
 * The f746 sibling sidesteps all of this by blocking in accept for ever on a thread of its
 * own (port/netxduo/nx_shell.c).  This command has no thread and must stay cancellable, so
 * it arms once with NX_NO_WAIT -- which returns NX_IN_PROGRESS and leaves the socket armed
 * exactly as the blocking call would -- and then waits with nx_tcp_socket_state_wait(),
 * which touches nothing at all: it is a plain tx_thread_sleep(1) poll
 * (nx_tcp_socket_state_wait.c:103-118) and is therefore safe to re-enter as often as we
 * like.  Cancellability without mutating the socket is the whole point.
 */
static int nxe_arm_accept(struct cli_instance *sh)
{
	UINT s = nx_tcp_server_socket_accept(&nxe_sock, NX_NO_WAIT);

	/* NX_IN_PROGRESS is the expected answer (the no-suspension branch); NX_SUCCESS
	 * means a client was already there when we asked. */
	if (s == NX_SUCCESS || s == NX_IN_PROGRESS)
		return 0;
	cli_error(sh, "net: could not arm accept (0x%02x)\r\n", (unsigned)s);
	LOG_ERR("accept arm failed (0x%02x)", (unsigned)s);
	return -1;
}

/*
 * Send @n bytes, waiting for window/queue space rather than dropping them: this is a
 * stream, and an echo server that silently discards is not a measurement of anything.
 * Bounded and cancellable.  Returns 0 sent, -1 gave up (counted by the caller).
 */
static int nxe_send(struct nxe_run *r, const uint8_t *p, ULONG n, NX_PACKET_POOL *pool)
{
	NX_PACKET *pkt = NX_NULL;
	unsigned tries;

	if (nx_packet_allocate(pool, &pkt, NX_TCP_PACKET, NXE_SEND_SLICE) != NX_SUCCESS) {
		r->alloc_fails++;
		return -1;
	}
	if (nx_packet_data_append(pkt, (VOID *)p, n, pool, NXE_SEND_SLICE) != NX_SUCCESS) {
		nx_packet_release(pkt);
		r->alloc_fails++;
		return -1;
	}

	for (tries = 0u; tries < NXE_SEND_TRIES; tries++) {
		UINT s = nx_tcp_socket_send(&nxe_sock, pkt, NXE_SEND_SLICE);

		if (s == NX_SUCCESS)
			return 0;              /* NetX took ownership and NULLed pkt      */
		/* Those two are exactly what _nx_tcp_transmit_cleanup() leaves behind when
		 * the wait expires with the window closed or the queue full; anything else
		 * (NX_NOT_CONNECTED and friends) means retrying is pointless. */
		if (s != NX_WINDOW_OVERFLOW && s != NX_TX_QUEUE_DEPTH)
			break;
		/* Back-pressure, which is the healthy case: the window or the transmit
		 * queue is full and the notify will open it again.  Keep the packet -- on
		 * failure NetX has not taken it -- and re-check Ctrl+C between attempts. */
		r->send_retries++;
		if (cli_cancel_requested(r->sh) || nxe_peer_gone)
			break;
	}
	if (pkt != NX_NULL)
		nx_packet_release(pkt);
	return -1;
}

/* One accepted connection.  Returns 0 when the peer left, 1 when Ctrl+C ended it. */
static int nxe_session(struct nxe_run *r, NX_PACKET_POOL *pool)
{
	uint32_t t0 = HAL_GetTick(), dt;
	uint32_t srx = 0, stx = 0;
	ULONG peer_ip = 0, peer_port = 0;
	int cancelled = 0;
	/*
	 * Why the loop below ended.  A measurement command that cannot say whether a
	 * transfer finished, was cancelled, or was cut short by the far end is not a
	 * measurement -- the 6 Mbaud stall was invisible for exactly this reason.
	 */
	const char *why = "peer closed";
	UINT last = NX_SUCCESS;

	if (nx_tcp_socket_peer_info_get(&nxe_sock, &peer_ip, &peer_port) == NX_SUCCESS)
		cli_print(r->sh, "  session %lu: %lu.%lu.%lu.%lu:%lu connected\r\n",
		          (unsigned long)r->sessions,
		          (unsigned long)((peer_ip >> 24) & 0xFFu),
		          (unsigned long)((peer_ip >> 16) & 0xFFu),
		          (unsigned long)((peer_ip >> 8) & 0xFFu),
		          (unsigned long)(peer_ip & 0xFFu), (unsigned long)peer_port);
	else
		cli_print(r->sh, "  session %lu connected\r\n", (unsigned long)r->sessions);

	for (;;) {
		NX_PACKET *pkt = NX_NULL;
		ULONG copied = 0;
		UINT  s;

		if (cli_cancel_requested(r->sh)) {
			cancelled = 1;
			why = "Ctrl+C";
			break;
		}
		if (!nx_net_is_up()) {
			why = "the host stack went down";
			break;
		}
		s = nx_tcp_socket_receive(&nxe_sock, &pkt, NXE_RECV_SLICE);
		if (s == NX_NO_PACKET) {
			if (nxe_peer_gone) {
				why = "peer closed (FIN, queue drained)";
				break;
			}
			continue;                  /* idle */
		}
		if (s != NX_SUCCESS) {
			/* NX_NOT_CONNECTED after a reset, most interestingly.  Keep the code:
			 * "the transfer stopped" and "the connection was reset" are different
			 * findings and only this byte tells them apart. */
			last = s;
			why  = nxe_peer_gone ? "connection ended after FIN"
			                     : "receive failed (see status)";
			break;
		}

		/*
		 * Extract rather than bounce the received packet back: a receive packet's
		 * headroom belongs to the frame it arrived in, and NetX wants to build its
		 * own headers in front of what it transmits.
		 */
		while (copied < pkt->nx_packet_length) {
			ULONG got = 0;

			if (nx_packet_data_extract_offset(pkt, copied, nxe_buf, sizeof nxe_buf,
			                                  &got) != NX_SUCCESS || got == 0)
				break;
			copied += got;
			srx += (uint32_t)got;
			if (nxe_send(r, nxe_buf, got, pool) == 0)
				stx += (uint32_t)got;
			else
				r->send_fails++;
			nxe_pool_sample(r, pool);
		}
		nx_packet_release(pkt);
	}

	dt = HAL_GetTick() - t0;
	r->rx += srx;
	r->tx += stx;
	if (dt == 0u)
		dt = 1u;
	/* To the log as well as the console -- same reason as nxe_report(). */
	LOG_INF("echo session %lu: rx %lu B, tx %lu B in %lu ms (%lu KB/s both ways), "
	        "ended: %s (0x%02x)",
	        (unsigned long)r->sessions, (unsigned long)srx, (unsigned long)stx,
	        (unsigned long)dt, (unsigned long)((stx + srx) / dt), why, (unsigned)last);
	cli_print(r->sh, "  session %lu: rx %lu B, tx %lu B in %lu ms (%lu KB/s both ways)\r\n",
	          (unsigned long)r->sessions, (unsigned long)srx, (unsigned long)stx,
	          (unsigned long)dt, (unsigned long)((stx + srx) / dt));
	cli_print(r->sh, "  session %lu ended: %s (nx status 0x%02x)\r\n",
	          (unsigned long)r->sessions, why, (unsigned)last);
	return cancelled;
}

/*
 * Report the numbers U4-3 sizes the pools from.
 *
 * ---- why this also goes to the log --------------------------------------------
 *
 * The ONLY way to stop this command is Ctrl+C, and once cancel is latched the core drops
 * every further byte a handler tries to print (cli_core.c:456-459, issue #16 -- a
 * cancelled command must stop spewing at once).  So a measurement command whose results
 * are printed at the end has them destroyed by the very keystroke that ends it: the first
 * board run produced the numbers and showed none of them.
 *
 * The log has neither property -- it does not go through the instance's output path and
 * does not consult cancel_req -- and on this board it also survives a reset (the DTCM
 * ring behind `dmesg`).  So the console copy is the convenience and the log copy is the
 * record.  Anything added here that matters should go to both.
 */
static void nxe_report(struct nxe_run *r)
{
	struct nx_link_stats st;
	struct link_data_stats ld;
	struct nx_net_modstats ms;

	nx_link_driver_get_stats(&st);
	link_data_stats(&ld);
	nx_net_modstats_get(&ms);
	LOG_INF("echo: %lu session(s), rx %lu B, tx %lu B, tx waits %lu, gave up %lu, "
	        "alloc fails %lu, pool low-water %lu",
	        (unsigned long)r->sessions, (unsigned long)r->rx, (unsigned long)r->tx,
	        (unsigned long)r->send_retries, (unsigned long)r->send_fails,
	        (unsigned long)r->alloc_fails, (unsigned long)r->pool_low);
	LOG_INF("echo: driver tx %lu frames no-buf %lu, rx %lu frames no-buf %lu",
	        (unsigned long)st.tx_frames, (unsigned long)st.tx_no_buf,
	        (unsigned long)st.rx_frames, (unsigned long)st.rx_no_buf);
	/*
	 * The link's own loss ledger, logged with the run rather than left to a separate
	 * `net info` the user may not think to type before tearing the session down.  This
	 * is the line that says whether a stalled transfer lost bytes on the wire or was
	 * TCP behaving badly above an intact link -- the two have completely different
	 * fixes and nothing else distinguishes them.
	 */
	LOG_INF("echo: host link rx drops %lu crc %lu oversize %lu, tx drops %lu",
	        (unsigned long)ld.rx_drops, (unsigned long)ld.rx_crc_err,
	        (unsigned long)ld.rx_oversize, (unsigned long)ld.tx_drops);
	LOG_INF("echo: module link rx drops %lu crc %lu oversize %lu gaps %lu, tx drops %lu",
	        (unsigned long)ms.rx_drops, (unsigned long)ms.rx_crc,
	        (unsigned long)ms.rx_oversize, (unsigned long)ms.rx_gaps,
	        (unsigned long)ms.tx_drops);

	cli_print(r->sh, "net echo: %lu session(s), rx %lu B, tx %lu B\r\n",
	          (unsigned long)r->sessions, (unsigned long)r->rx, (unsigned long)r->tx);
	cli_print(r->sh, "  tx waits %lu, gave up %lu, alloc fails %lu, pool low-water %lu\r\n",
	          (unsigned long)r->send_retries, (unsigned long)r->send_fails,
	          (unsigned long)r->alloc_fails, (unsigned long)r->pool_low);

	nx_link_driver_get_stats(&st);
	cli_print(r->sh, "  driver tx: %lu frames, %lu B, no-buf %lu, oversize %lu, down %lu\r\n",
	          (unsigned long)st.tx_frames, (unsigned long)st.tx_bytes,
	          (unsigned long)st.tx_no_buf, (unsigned long)st.tx_oversize,
	          (unsigned long)st.tx_link_down);
	cli_print(r->sh, "  driver rx: %lu frames, %lu B, no-buf %lu\r\n",
	          (unsigned long)st.rx_frames, (unsigned long)st.rx_bytes,
	          (unsigned long)st.rx_no_buf);
	cli_print(r->sh, "  link loss: host rx drops %lu crc %lu, tx drops %lu | "
	          "module rx drops %lu crc %lu gaps %lu, tx drops %lu%s\r\n",
	          (unsigned long)ld.rx_drops, (unsigned long)ld.rx_crc_err,
	          (unsigned long)ld.tx_drops, (unsigned long)ms.rx_drops,
	          (unsigned long)ms.rx_crc, (unsigned long)ms.rx_gaps,
	          (unsigned long)ms.tx_drops,
	          (ld.rx_drops || ld.rx_crc_err || ld.tx_drops || ms.rx_drops ||
	           ms.rx_crc || ms.rx_gaps || ms.tx_drops) ? "  <-- LOSS" : "  (clean)");
	/* The pass/fail line of issue #23 U4: a full DATA transmit pool is a silent drop,
	 * and a stream protocol pays for it with a retransmit timeout. */
	if (st.tx_no_buf != 0u)
		cli_warn(r->sh, "  the DATA transmit pool ran dry (%lu) -- LINK_DATA_TX_BUFS is "
		         "too shallow for this load\r\n", (unsigned long)st.tx_no_buf);
}

/* ---- entry ------------------------------------------------------------------ */

int nx_echo_run(struct cli_instance *sh, uint16_t port)
{
	NX_IP          *ip   = (NX_IP *)nx_net_ip();
	NX_PACKET_POOL *pool = (NX_PACKET_POOL *)nx_net_pool();
	struct nx_net_info ni;
	struct nxe_run  r;
	int listening = 0, stop = 0;
	unsigned rounds = 0u;          /* heartbeat counter; MUST outlive one loop pass */
	unsigned tick_at = NXE_TICK_FIRST;
	UINT s;

	if (ip == NX_NULL || pool == NX_NULL || !nx_net_is_up()) {
		cli_error(sh, "net: no host stack -- `wifi connect` brings it up\r\n");
		return 1;
	}
	if (!nxe_claim()) {
		cli_error(sh, "net: `net echo` is unavailable -- either one is already running, "
		          "or a previous one could not release its socket (reset clears that)\r\n");
		return 1;
	}

	r.sh = sh;
	r.sessions = 0u; r.rx = 0u; r.tx = 0u;
	r.send_retries = 0u; r.send_fails = 0u; r.alloc_fails = 0u;
	r.pool_low = 0xFFFFFFFFUL;
	nxe_peer_gone = 0u;

	s = nx_tcp_socket_create(ip, &nxe_sock, "net-echo", NX_IP_NORMAL, NX_FRAGMENT_OKAY,
	                         NX_IP_TIME_TO_LIVE, NXE_WINDOW, NX_NULL, nxe_disconnect_cb);
	if (s != NX_SUCCESS) {
		cli_error(sh, "net: socket create failed (0x%02x)\r\n", (unsigned)s);
		nxe_busy = 0u;
		return 1;
	}
	/* Bound what one socket can have outstanding towards the driver -- see the
	 * transmit budget note in app/nx_net.h, which also explains why the retransmit
	 * timer is left at the conservative f746 value rather than tuned to this link. */
	(void)nx_tcp_socket_transmit_configure(&nxe_sock, NXN_TCP_TX_DEPTH, NXN_TCP_RTO_MS,
	                                       NXN_TCP_RTO_RETRIES, NXN_TCP_RTO_SHIFT);

	s = nx_tcp_server_socket_listen(ip, port, &nxe_sock, 1, NX_NULL);
	if (s != NX_SUCCESS) {
		cli_error(sh, "net: listen :%u failed (0x%02x -- port in use?)\r\n",
		          (unsigned)port, (unsigned)s);
		goto out;
	}
	listening = 1;

	if (nx_net_info_get(&ni) == NXN_OK && ni.ip_valid)
		cli_print(sh, "net: TCP echo server on %u.%u.%u.%u:%u (host stack)\r\n",
		          (unsigned)((ni.ip >> 24) & 0xFFu), (unsigned)((ni.ip >> 16) & 0xFFu),
		          (unsigned)((ni.ip >> 8) & 0xFFu),  (unsigned)(ni.ip & 0xFFu),
		          (unsigned)port);
	else
		cli_print(sh, "net: TCP echo server on port %u (host stack)\r\n", (unsigned)port);
	cli_print(sh, "     connect with `nc <ip> %u`; Ctrl+C stops (within ~%u ms)\r\n",
	          (unsigned)port, (unsigned)NXE_ACCEPT_SLICE);

	nxe_peer_gone = 0u;
	if (nxe_arm_accept(sh) != 0)
		goto out;

	while (!stop) {
		UINT st;

		if (cli_cancel_requested(sh))
			break;
		/*
		 * The other console can take the interface down while this is listening.  Checking the
		 * predicate every round -- not just once at entry -- is what keeps this
		 * command from outliving the stack it runs on; the interface going away is a
		 * normal end, not an error.
		 */
		if (!nx_net_is_up()) {
			cli_warn(sh, "net: the host stack went down -- stopping the echo "
			         "server\r\n");
			break;
		}

		/* Poll, do not re-arm: see the warning on nxe_arm_accept(). */
		if (nx_tcp_socket_state_wait(&nxe_sock, NX_TCP_ESTABLISHED,
		                             NXE_ACCEPT_SLICE) != NX_SUCCESS) {
			st = nxe_sock.nx_tcp_socket_state;
			/*
			 * A visible heartbeat.  It is the difference between "no client yet"
			 * and "this command is wedged", which is precisely the distinction
			 * that was impossible to make from the console the first time round.
			 */
			if (++rounds >= tick_at) {
				rounds  = 0u;
				tick_at = NXE_TICK_REPEAT;
				cli_print(sh, "  waiting for a client (tcp state %u)\r\n",
				          (unsigned)st);
			}
			/*
			 * The connection can complete between the poll giving up and this
			 * read.  Losing that race must not cost the client its session: go
			 * round again and the next state_wait() returns at once.  (Getting
			 * this wrong would DISCONNECT a connection that had just arrived --
			 * rare, load-dependent, and exactly the kind of thing that would have
			 * been blamed on the link.)
			 */
			if (st == NX_TCP_ESTABLISHED)
				continue;
			/*
			 * LISTEN and SYN_RECEIVED are the two healthy passive-open states.
			 * Anything else (a handshake NetX gave up on, a reset) means the arm
			 * is gone and polling for ESTABLISHED would wait for ever.
			 */
			if (st != NX_TCP_LISTEN_STATE && st != NX_TCP_SYN_RECEIVED) {
				LOG_WRN("socket left the passive open (state %u) -- re-arming",
				        (unsigned)st);
				(void)nx_tcp_socket_disconnect(&nxe_sock, NXE_DISC_MS);
				(void)nx_tcp_server_socket_unaccept(&nxe_sock);
				s = nx_tcp_server_socket_relisten(ip, port, &nxe_sock);
				if (s != NX_SUCCESS && s != NX_CONNECTION_PENDING) {
					cli_error(sh, "net: relisten failed (0x%02x)\r\n",
					          (unsigned)s);
					break;
				}
				if (nxe_arm_accept(sh) != 0)
					break;
			}
			continue;
		}

		r.sessions++;
		stop = nxe_session(&r, pool);

		/* End this connection and re-arm the listen slot.  unaccept() is what puts
		 * the socket back to CLOSED and off the bound port list; without it the
		 * relisten -- and later the delete -- would be refused. */
		(void)nx_tcp_socket_disconnect(&nxe_sock, NXE_DISC_MS);
		(void)nx_tcp_server_socket_unaccept(&nxe_sock);
		if (!stop && !cli_cancel_requested(sh)) {
			s = nx_tcp_server_socket_relisten(ip, port, &nxe_sock);
			if (s != NX_SUCCESS && s != NX_CONNECTION_PENDING) {
				cli_error(sh, "net: relisten failed (0x%02x)\r\n", (unsigned)s);
				break;
			}
			nxe_peer_gone = 0u;
			if (nxe_arm_accept(sh) != 0)
				break;
		}
	}

out:
	/*
	 * Teardown.  nx_tcp_socket_delete() only succeeds on an UNBOUND socket in
	 * NX_TCP_CLOSED, and there is more than one way to be neither -- so the
	 * disconnect/unaccept pair runs unconditionally rather than only when a session was
	 * accepted.  The case that makes this mandatory: nx_tcp_server_socket_relisten()
	 * returns NX_CONNECTION_PENDING when a SYN was already queued, and that path BINDS
	 * the socket into the TCP port table (nx_tcp_server_socket_relisten.c:343-396) while
	 * leaving it in LISTEN.  A Ctrl+C in that window used to reach delete with the socket
	 * still bound, and delete refuses that (NX_STILL_BOUND).
	 *
	 * The pair handles every state we can be in:
	 *   ESTABLISHED       disconnect -> a state >= CLOSE_WAIT, or back to LISTEN via
	 *                     _nx_tcp_socket_block_cleanup(); unaccept accepts both
	 *                     (nx_tcp_server_socket_unaccept.c:86-109) and unbinds
	 *   bound LISTEN      (the pending-SYN relisten above) disconnect is a harmless
	 *                     NX_NOT_CONNECTED; unaccept unbinds it via bound_next
	 *   unbound LISTEN    unaccept finds the socket on the active listen request and
	 *                     clears the slot (nx_tcp_server_socket_unaccept.c:179-185)
	 * unaccept must come BEFORE unlisten: it clears the listen request's socket pointer,
	 * unlisten then removes the request itself.
	 */
	(void)nx_tcp_socket_disconnect(&nxe_sock, NXE_DISC_MS);
	(void)nx_tcp_server_socket_unaccept(&nxe_sock);
	if (listening)
		(void)nx_tcp_server_socket_unlisten(ip, port);
	s = nx_tcp_socket_delete(&nxe_sock);

	if (r.pool_low == 0xFFFFFFFFUL)
		r.pool_low = 0u;
	nxe_report(&r);

	if (s != NX_SUCCESS) {
		/*
		 * A successful delete is a POSTCONDITION, not a nicety: nxe_sock is static
		 * storage that is still on the IP instance's created-socket list, and letting
		 * a second run call nx_tcp_socket_create() on it would corrupt that list.  So
		 * the claim is NOT released -- the echo server is out of order until a reset,
		 * and says so instead of failing in NetX's bookkeeping later.
		 */
		LOG_ERR("socket delete refused (0x%02x) -- echo disabled until reset",
		        (unsigned)s);
		cli_error(sh, "net: the echo socket could not be released (0x%02x). "
		          "`net echo` is disabled until the board is reset; nothing else is "
		          "affected.\r\n", (unsigned)s);
		return 1;
	}

	nxe_busy = 0u;
	return 0;
}
