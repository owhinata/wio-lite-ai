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
 *   server thread (14)   owns the socket: the ONLY caller of create / listen / accept /
 *                        relisten / disconnect / unaccept / unlisten / delete, and the ONLY
 *                        thread that transmits.  That is why the teardown can prove what it
 *                        proves.  (It is not the only caller of every socket API -- the
 *                        receive callback below runs nx_tcp_socket_receive() on the IP
 *                        thread, which is NetX's own idiom.)
 *   NetX IP thread (11)  runs the receive / disconnect / establish / queue-depth callbacks.
 *                        They only move bytes into the RX ring and set flags.
 *   CLI threads (16)     the shell instance and any background job, inside write().
 *
 * ---- why there is a TX ring, and what happens without one (issue #48) -------------
 *
 * The RX ring is strict SPSC: the IP thread is the only producer, the CLI instance thread
 * the only consumer.  Output goes through a TX ring in the other direction, drained by the
 * server thread.  Until issue #48 it did not: write() built an NX_PACKET and called
 * nx_tcp_socket_send() itself, leaving back-pressure to TCP -- a refused send returned
 * short, the core waited on CLI_EVT_TX, and NetX's window-update / queue-depth callbacks
 * were supposed to wake it.  That failed in three ways at once, and telnet lost the tail of
 * every long report:
 *
 *   1. ONE call, ONE segment.  The core stages output in CLI_PRINTF_BUFFER_SIZE (32 B)
 *      chunks, so a 5 kB report became ~180 tiny TCP segments against a 4-deep transmit
 *      queue (NXN_TCP_TX_DEPTH).  The console lived permanently at the queue limit, one
 *      peer ACK per 32 bytes of output.
 *   2. TWO of the three refusals had NO wake-up source at all.  nx_tcp_socket_window_
 *      update_notify is only ever invoked from _nx_tcp_socket_state_transmit_check(), whose
 *      whole body sits inside `if (socket_ptr -> nx_tcp_socket_transmit_suspension_list)`
 *      (nx_tcp_socket_state_transmit_check.c:75-130) -- and an NX_NO_WAIT send never joins
 *      that list, so the callback we registered could not fire, ever.  Packet-pool
 *      exhaustion has no callback in NetX at all.  Both stalled until the deadline.
 *   3. Even the working path (NX_TX_QUEUE_DEPTH -> queue-depth notify on ACK) needs a
 *      retransmit when a segment is lost on the link, and NXN_TCP_RTO_MS is 2 s against a
 *      1 s CLI_TX_TIMEOUT.  One lost segment was a guaranteed truncation, and cli_printf.c
 *      latches tx_failed, so it cost the whole REST of the command's output, not a line.
 *
 * The ring fixes all three: the CLI hands bytes to a local buffer and is answered at once,
 * the server thread coalesces them into MSS-sized segments, and every byte it moves fires
 * cli_transport_notify_tx() unconditionally -- which is exactly why the USB CDC backend
 * (shell/backend/cli_backend_usbcdc.c) never had this problem.  Refusals are retried on a
 * short bounded wait, so the two paths NetX cannot signal are covered by construction
 * rather than by a callback that turns out to be dead.  The transport additionally raises
 * its own no-progress deadline above the RTO (tx_timeout, below), because a ring cannot
 * absorb output larger than itself.
 *
 * TCP is still what paces the wire: the packet pool and the link's DATA transmit pool are
 * sized around NXN_TCP_TX_DEPTH (see the transmit budget note in app/nx_net.h), and none of
 * that changed.
 *
 * No clock/RCC/register work of its own (clock-safe).  Clean-room design.
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
#include "mem_sections.h"  /* DTCM_BSS: CPU-only data out of AXI-SRAM (issue #46) */

/* ---- tunables ------------------------------------------------------------ */

/* Below the nx-net owner (13) and the NetX IP thread (11) it depends on, above the CLI
 * instances (16) so a connection is accepted while a command computes.  Same slot the
 * f746 port uses. */
#define NSH_PRIORITY        14u
/*
 * Issue #48 moved the whole NetX transmit path onto this thread --
 * nx_tcp_socket_send -> _nx_tcp_socket_send_internal -> _nx_ip_packet_send ->
 * _nx_ip_driver_packet_send -> app/nx_link_driver.c -> link_data_send -- so issue #23
 * U4-3's 596 B (measured when it only orchestrated) stopped bounding it.
 *
 * MEASURED AFTER THE MOVE: 492 B, on a session that filled the TX ring to its 4095 B
 * ceiling with repeated `dmesg` / `coremark` / `membench`.  Lower than the old figure
 * despite the deeper call chain, because command execution still happens in the bound CLI
 * instance and -Os + LTO (issue #39) reworked the frames.  2560 keeps 5x on that, which is
 * more than this thread needs -- if DTCM ever gets tight (it is the scarce region, see
 * issue #46), this is a safe 1 KB to give back, not a number to defend.
 */
#define NSH_STACK           2560u

#define NSH_RX_RING         512u
#define NSH_WINDOW          2048u    /* advertised TCP receive window                */
#define NSH_MSS             1400u    /* bytes per transmitted packet                 */
#define NSH_EXTRACT         1500u    /* per-packet receive extraction buffer         */

/*
 * Output the CLI threads can hand over before they have to wait (issue #48).  It is what
 * turns ~32-byte writes into MSS-sized segments, so it wants to be several MSS; 4 KB holds
 * a `coremark` report or a `membench` table outright.  It is NOT big enough for everything
 * (a `dmesg` is larger, and cli_uart_ring's usable depth is size-1), which is why the
 * deadline below matters as well -- the two together are the fix, not either alone.
 */
#define NSH_TX_RING         4096u

/*
 * How long the drain waits for the socket to take more when it has just refused.
 *
 * This is the belt to the queue-depth callback's braces, and it is what makes the refusals
 * NetX cannot signal (a window overflow, an empty packet pool) recoverable at all rather
 * than fatal.  Only ever armed while the ring is non-empty, so an idle console still sleeps
 * NSH_IDLE_MS and the 98.9 % idle of issue #23 U4 is unchanged.
 */
#define NSH_DRAIN_POLL_MS   50u

/* Segments pushed per drain pass before going back round the loop (so a stop request is
 * still noticed promptly during a long report). */
#define NSH_TX_BURST        8u

/*
 * The console's no-progress deadline, in ticks, published through cli_transport::tx_timeout
 * (the default CLI_TX_TIMEOUT of 1 s applies to every other backend).
 *
 * It MUST exceed NXN_TCP_RTO_MS (2 s): recovering a segment the link dropped means waiting
 * out a retransmit, and a shorter deadline made that loss discard the rest of the command's
 * output (issue #48).  2.5x covers the first retransmit with margin.  It deliberately does
 * NOT cover a second consecutive loss of the same segment (2 s + 4 s of backoff) -- past
 * that the cost is being tied to a peer that is alive but not reading.  The wait stays
 * interruptible throughout: cli_tx_send_blocking() also wakes on CLI_EVT_RX and polls for
 * Ctrl+C, and a session ending discards the ring and closes the write gate.
 */
#define NSH_TX_DEADLINE     5000u

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
#define NSH_EVT_TX          0x10u   /* output queued, or the socket took more */

/* ---- state --------------------------------------------------------------- */

/* PRIMASK critical section, as in shell/backend/cli_backend_usbcdc.c.  Nests safely inside
 * ThreadX's own PRIMASK sections. */
#define NSH_CRIT_ENTER()  do { uint32_t _pm = __get_PRIMASK(); __disable_irq()
#define NSH_CRIT_EXIT()   __set_PRIMASK(_pm); } while (0)

struct nsh_ctx {
	struct cli_instance *sh;             /* set by nsh_init() from tr->sh          */
	struct cli_uart_ring rx;             /* IP thread -> CLI thread (SPSC)         */
	struct cli_uart_ring tx;             /* CLI/bg threads -> server thread (MPSC) */
	uint8_t              rx_buf[NSH_RX_RING];
	/*
	 * The write gate.  SET by the CLI thread (nsh_session_begin, once the connection is
	 * confirmed), CLEARED by the server thread's teardown and by the disconnect callback
	 * on the NetX IP thread.  Producers read it inside their critical section, which is
	 * what makes a clear a hard cut-off -- see the gate note above nsh_tx_put_raw().
	 */
	volatile uint8_t     connected;
};

static struct nsh_ctx       g_ctx;
static TX_THREAD            g_thread;
static TX_EVENT_FLAGS_GROUP g_evt;
static UCHAR                g_stack[NSH_STACK] DTCM_BSS __attribute__((aligned(8)));
static uint8_t              g_ready;

/*
 * The TX ring's storage.  CPU-only -- no bus master reads it; the server thread copies out
 * of it into an NX_PACKET, and the packet pool itself is in DTCM too (app/nx_net.c) -- so
 * DTCM is where the issue-#46 policy puts it.
 *
 * It is a STANDALONE symbol rather than a member of g_ctx on purpose: the post-link gate
 * cmake/check_dtcm_residency.py matches by symbol name, so a buffer hidden inside a struct
 * could silently fall back to AXI-SRAM with the build still green.  Keep `nsh_tx_buf` in
 * that script's REQUIRED_DTCM whenever this line changes.
 */
static uint8_t              nsh_tx_buf[NSH_TX_RING] DTCM_BSS;

static NX_TCP_SOCKET        g_sock;
static uint8_t              g_sock_created;   /* server thread only */

/* Owned by the server thread (others only read). */
static volatile enum net_shell_state g_state;
static uint16_t  g_port = NET_SHELL_PORT_DEFAULT;
static int       g_iac;                          /* telnet IAC receive state          */
static uint8_t   g_extract[NSH_EXTRACT];         /* IP thread only                    */
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
 * Why the socket would not take a segment.  Kept APART, because merging them is what hid
 * issue #48: a single "tx waits 1402 (back-pressure, normal)" could not distinguish the
 * healthy answer from the one with no wake-up source, and the report read as healthy while
 * the console was dropping output.  Same lesson as issue #23 U4's tx-refused counter --
 * never collapse quantities whose answers point in opposite directions.
 *
 *   g_tx_qdepth  the socket's transmit queue was at NXN_TCP_TX_DEPTH.  The DESIGNED
 *                back-pressure: a peer ACK releases a packet and the queue-depth callback
 *                wakes the drain.  A large number is normal.
 *   g_tx_win     the peer's window (or congestion window) had no room.  NetX cannot signal
 *                this to an NX_NO_WAIT sender at all (see the header), so it is the
 *                NSH_DRAIN_POLL_MS retry that recovers it.  Expected to be ~0 in practice;
 *                a large number means the peer is not reading.
 *   g_tx_nobuf   the packet pool was empty or below this console's reserve.  THIS is the
 *                sizing signal: output was competing with the receive path for packets,
 *                which is what NXN_TCP_TX_DEPTH and the pool were sized to avoid.
 */
static uint32_t g_tx_qdepth, g_tx_win, g_tx_nobuf;
/*
 * TCP segments this console has transmitted.  Reported next to the byte count because
 * BYTES PER SEGMENT is the health number for an interactive console, and issue #49 had to
 * infer it from bytes / refusals -- which only worked because refusals happened to be
 * near-universal at the time.  17 B/seg means one peer ACK per keystroke fragment; a
 * redraw should be one segment.
 */
static uint32_t g_tx_segs;
/* High-water mark of the TX ring, so `net shell status` can say whether the CLI ever had
 * to wait on it at all (bytes; the usable depth is NSH_TX_RING - 1). */
static uint32_t g_tx_hiwater;

/* ---- TX ring, producer side (any CLI thread; PRIMASK) -------------------- */

/*
 * The ring has THREE producers -- the instance thread, a background job writing through
 * sh->fg, and session_begin(), which the core calls WITHOUT its output lock
 * (cli_core.c) -- so it is MPSC, and they serialise with the same short interrupt-masked
 * region the USB CDC backend uses.  A mutex is not an option: write() is a non-blocking
 * contract and is already called under the instance's tx_lock.  The server thread is the
 * sole consumer and touches only `tail`, so it never contends with them.
 *
 * 🔴 THE WRITE GATE IS READ INSIDE THAT REGION, not before it, and that is load-bearing.
 * `connected` is cleared by the server thread (or the disconnect callback) and followed by
 * nsh_tx_discard(); if a producer could test it and enqueue as two separate steps, a CLI
 * thread preempted between them -- it runs at 16, below both the server (14) and the IP
 * thread (11) -- could append AFTER the discard, and those bytes, belonging to a client
 * that is gone, would be the first thing the NEXT client saw.  Testing and storing in one
 * critical section makes the clear a hard cut-off instead: the store is atomic, so every
 * region that begins after it refuses, and every byte accepted before it is already visible
 * to the discard that follows.
 */

/*
 * Store @n bytes verbatim and INDIVISIBLY -- all of them or none.  Returns 1 on success.
 *
 * Indivisible matters: the only caller sends telnet negotiation, and another producer
 * (a background job from the previous session is the realistic one, since it keeps writing
 * through sh->fg) landing in the middle of IAC WILL ECHO would turn a command into
 * garbage on the wire.  The pre-U4 code got this for free by emitting the whole sequence in
 * one nx_tcp_socket_send() under the socket mutex; with a ring the equivalent guarantee has
 * to be spelled out.
 */
static int nsh_tx_put_raw(const uint8_t *p, size_t n)
{
	int ok;

	NSH_CRIT_ENTER();
	ok = g_ctx.connected && cli_uart_ring_free(&g_ctx.tx) >= n;
	if (ok)
		(void)cli_uart_ring_put_buf(&g_ctx.tx, p, n);
	NSH_CRIT_EXIT();
	return ok;
}

/*
 * Store one SHELL byte, telnet-encoded: a literal 0xFF must go out as IAC IAC or the client
 * reads it as the start of a command (f746's nx_shell strips IAC on receive but never
 * escaped its output, which corrupts any binary the shell prints).  The pair goes in inside
 * ONE critical section, so another producer can never be interleaved between the halves.
 *
 * NOT for the telnet negotiation in nsh_charmode: those bytes ARE IAC sequences and must go
 * through nsh_tx_put_raw() unescaped.
 */
static int nsh_tx_put(uint8_t b)
{
	int ok;

	NSH_CRIT_ENTER();
	if (!g_ctx.connected)              /* see the gate note above */
		ok = 0;
	else if (b == 0xFFu)
		ok = (cli_uart_ring_free(&g_ctx.tx) >= 2u) &&
		     cli_uart_ring_put(&g_ctx.tx, 0xFFu) &&
		     cli_uart_ring_put(&g_ctx.tx, 0xFFu);
	else
		ok = cli_uart_ring_put(&g_ctx.tx, b);
	NSH_CRIT_EXIT();
	return ok;
}

/* Wake the drain.  The ring, never this flag, is the source of truth (see nsh_entry). */
static void nsh_tx_kick(void)
{
	(void)tx_event_flags_set(&g_evt, NSH_EVT_TX, TX_OR);
}

/*
 * Drop everything queued.  Advances `tail` by a count it has already read, so it cannot run
 * past a producer's `head` -- but that alone would NOT make it a clean cut: what stops a
 * producer appending straight after it is the write gate being read inside the producers'
 * critical section (see the note above).  CALL IT ONLY WITH `connected` ALREADY CLEARED.
 *
 * Always followed by the TX notify, so a writer blocked on CLI_EVT_TX wakes, re-enters
 * write() and returns at once on the closed gate instead of waiting out the deadline.
 */
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
	cli_uart_ring_init(&g_ctx.tx, nsh_tx_buf, sizeof nsh_tx_buf);
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
 * ---- the write path ------------------------------------------------------------
 *
 * cli_transport_api.write() is a NON-BLOCKING contract (cli_instance.h): enqueue what fits,
 * return the count, and let the core wait on CLI_EVT_TX for the rest.  Since issue #48 that
 * is all it does -- no NetX call, no lock, no socket.  Everything that can refuse, block or
 * be torn down lives on the server thread, which is what makes the contract honest instead
 * of merely non-blocking.
 */
static int nsh_write(struct cli_transport *tr, const uint8_t *data, size_t len)
{
	size_t i, count;

	(void)tr;
	/* Not connected: swallow it.  Returning `len` is what keeps the shell from ever
	 * wedging on a console nobody is attached to. */
	if (!g_ctx.connected)
		return (int)len;

	for (i = 0; i < len; i++) {
		if (!nsh_tx_put(data[i]))
			break;                       /* full, or the gate shut under us */
	}
	/* Sample the depth BEFORE waking the drain: afterwards the server thread may already
	 * have emptied it, and the peak -- whose whole job is to say how close this console
	 * came to making the CLI wait -- would read low. */
	count = cli_uart_ring_count(&g_ctx.tx);
	if ((uint32_t)count > g_tx_hiwater)
		g_tx_hiwater = (uint32_t)count;

	/*
	 * Deliberately NO kick here in the normal case (issue #49).  The core stages in
	 * 32-byte chunks, so waking the drain now would transmit a third of a redraw and
	 * pay a peer ACK for it; nsh_flush() does the waking when the whole unit is in.
	 *
	 * 🔴 EXCEPT when we could not take it all.  Then the core parks on CLI_EVT_TX
	 * waiting for room, the only source of room is the drain, and the drain's only
	 * wake-up would be an end-of-unit that cannot arrive because the unit is stuck
	 * here.  This one line is what keeps the flow-control contract from deadlocking.
	 */
	if (i < len)
		nsh_tx_kick();
	/* Told to stop rather than told to wait: the session ended while we were in the loop,
	 * so swallow the remainder exactly as the entry check does.  Reporting it as a short
	 * write instead would send the core to wait for room that this session will never
	 * need. */
	if (i < len && !g_ctx.connected)
		return (int)len;
	return (int)i;                               /* INPUT bytes consumed */
}

/*
 * "The unit of output is complete" -- the core calls this from cli_unlock() (issue #49).
 * THIS is what transmits: nsh_write() only fills the ring, so a whole line-editor redraw
 * reaches the drain in one piece and leaves as one TCP segment instead of one per 32-byte
 * staging chunk.  Nothing is delayed to get that -- the boundary was already known, the
 * old code just could not see it.
 */
static void nsh_flush(struct cli_transport *tr)
{
	(void)tr;
	if (cli_uart_ring_count(&g_ctx.tx) != 0u)
		nsh_tx_kick();
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
	/* RAW, not nsh_tx_put(): these ARE IAC sequences.  Escaping them would send
	 * IAC IAC WILL ECHO and the client would print 0xFF instead of negotiating.  In one
	 * call, so the six bytes cannot be split by another producer. */
	(void)nsh_tx_put_raw(nsh_charmode, sizeof nsh_charmode);
	nsh_tx_kick();
}

static const struct cli_transport_api nsh_api = {
	.init          = nsh_init,
	.enable        = nsh_enable,
	.write         = nsh_write,
	.read          = nsh_read,
	.session_begin = nsh_session_begin,
	.flush         = nsh_flush,
};

struct cli_transport net_shell_transport = {
	.api = &nsh_api,
	.sh  = NULL,                   /* set by cli_init() */
	.ctx = &g_ctx,
	/* Longer than every other backend, and specifically longer than NXN_TCP_RTO_MS --
	 * see NSH_TX_DEADLINE (issue #48). */
	.tx_timeout = NSH_TX_DEADLINE,
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

/*
 * The socket's transmit queue dropped back below NXN_TCP_TX_DEPTH, because a peer ACK
 * released packets (nx_tcp_socket_state_ack_check.c:567-582).  IP-thread context: flag only.
 *
 * It wakes the DRAIN, not the CLI.  The CLI is waiting for room in the ring, and only the
 * drain can make room; conflating the two directions is how the old code ended up telling
 * the core "space freed" when nothing had been sent.
 */
static void nsh_queue_notify(NX_TCP_SOCKET *s)
{
	(void)s;
	nsh_tx_kick();
}

/* ---- socket lifecycle (server thread only) ------------------------------- */

/*
 * Push queued output.  SERVER THREAD ONLY -- it is the sole consumer of the ring and the
 * sole caller of nx_tcp_socket_send().
 *
 * Returns the number of segments sent.  Stops at the first refusal and leaves the bytes in
 * the ring; the caller then arms a short wait (see nsh_entry) and tries again, which is
 * what covers the refusals NetX has no callback for.
 */
static unsigned nsh_tx_drain(void)
{
	NX_PACKET_POOL *pool = (NX_PACKET_POOL *)nx_net_pool();
	unsigned n;

	if (pool == NX_NULL || !g_sock_created || !g_link_live)
		return 0;

	for (n = 0u; n < NSH_TX_BURST; n++) {
		const uint8_t *p;
		size_t         run = cli_uart_ring_contig(&g_ctx.tx, &p);
		NX_PACKET     *pkt = NX_NULL;
		UINT           s;

		if (run == 0u)
			break;
		if (run > NSH_MSS)
			run = NSH_MSS;

		/* Leave the receive path some pool.  Advisory: an unlocked read of a counter
		 * NetX maintains, which is the right weight for a policy whose only job is to
		 * keep an output burst from being the reason a frame was dropped. */
		if (pool->nx_packet_pool_available <= NSH_POOL_RESERVE) {
			g_tx_nobuf++;
			break;
		}
		if (nx_packet_allocate(pool, &pkt, NX_TCP_PACKET, NX_NO_WAIT) != NX_SUCCESS) {
			g_tx_nobuf++;
			break;
		}
		if (nx_packet_data_append(pkt, (VOID *)p, (ULONG)run, pool,
		                          NX_NO_WAIT) != NX_SUCCESS) {
			nx_packet_release(pkt);
			g_tx_nobuf++;
			break;
		}
		s = nx_tcp_socket_send(&g_sock, pkt, NX_NO_WAIT);
		if (s != NX_SUCCESS) {
			/* NetX did not take it, so we still own it.  The three answers are
			 * counted apart on purpose -- see the counters' comment.  (A
			 * NX_NOT_CONNECTED from a disconnect race lands in g_tx_nobuf, which is
			 * harmless: the session is ending anyway.) */
			nx_packet_release(pkt);
			if (s == NX_TX_QUEUE_DEPTH)
				g_tx_qdepth++;
			else if (s == NX_WINDOW_OVERFLOW)
				g_tx_win++;
			else
				g_tx_nobuf++;
			break;
		}
		/* Sent: free the ring space and tell the core, which may be parked in
		 * cli_tx_send_blocking() waiting for exactly this. */
		cli_uart_ring_advance_tail(&g_ctx.tx, run);
		g_tx_bytes += (uint32_t)run;
		g_tx_segs++;
		if (g_ctx.sh != NULL)
			cli_transport_notify_tx(g_ctx.sh);
	}
	return n;
}

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
	/*
	 * Enabled by NX_ENABLE_TCP_QUEUE_DEPTH_UPDATE_NOTIFY (port/netxduo/nx_user.h), and the
	 * ONLY transmit-side callback worth registering here.
	 *
	 * nx_tcp_socket_window_update_notify_set() is deliberately NOT called: NetX invokes
	 * that callback from one place, _nx_tcp_socket_state_transmit_check(), whose entire
	 * body is guarded by `if (socket_ptr -> nx_tcp_socket_transmit_suspension_list)`
	 * (nx_tcp_socket_state_transmit_check.c:75-130).  A sender using NX_NO_WAIT never joins
	 * that list, so registering it produces a callback that can never fire -- which read as
	 * a live wake-up source for two years and cost issue #48 its diagnosis.  A window
	 * overflow is recovered by the drain's NSH_DRAIN_POLL_MS retry instead.
	 */
	(void)nx_tcp_socket_queue_depth_notify_set(&g_sock, nsh_queue_notify);
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
		if (nx_tcp_socket_delete(&g_sock) == NX_SUCCESS)
			g_sock_created = 0u;
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

	/*
	 * Stop producers first, throw away whatever they queued, and wake anyone parked in
	 * the core's TX wait so it re-enters write() and returns on the closed gate.
	 *
	 * No lock is needed around the socket calls below: since issue #48 this thread is the
	 * only one that transmits, so there is no CLI thread to be caught inside
	 * nx_tcp_socket_send() while the socket is deleted -- which was the single race the
	 * old g_sock_lock existed for.
	 */
	g_ctx.connected = 0;
	g_link_live     = 0u;
	nsh_tx_discard();

	if (!g_sock_created)
		return 0;

	(void)nx_tcp_socket_disconnect(&g_sock, NSH_DISC_MS);
	(void)nx_tcp_server_socket_unaccept(&g_sock);
	if (ip != NX_NULL)
		(void)nx_tcp_server_socket_unlisten(ip, g_port);
	s = nx_tcp_socket_delete(&g_sock);
	if (s == NX_SUCCESS)
		g_sock_created = 0u;

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
	nsh_tx_discard();
	/* A command may still be running for the client that just left.  0x03 is exactly
	 * what Ctrl+C delivers, so pushing one cancels it and leaves the instance idle for
	 * the next client instead of holding the console until it finishes on its own. */
	if (nsh_rx_put(0x03u) && g_ctx.sh != NULL)
		cli_transport_notify_rx(g_ctx.sh);

	(void)nx_tcp_socket_disconnect(&g_sock, NSH_DISC_MS);
	(void)nx_tcp_server_socket_unaccept(&g_sock);

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
			/* The auto-arm latch is NOT touched here.  Whether the console
			 * should come back on its own depends on WHY it is stopping, which
			 * only the requester knows: net_shell_stop() (the operator said so)
			 * clears it before asking, net_shell_stop_sync() (the interface is
			 * going away) leaves it alone.  Deciding it here as well made
			 * `net down` disable telnet for good -- one policy, one place. */
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
			/*
			 * Throw away anything still queued BEFORE opening the write gate.
			 * The teardown discarded once already, but a producer that was
			 * inside nsh_write() at the time could have appended after it, and
			 * those bytes belong to a client that is gone -- without this they
			 * would be the first thing the NEW client sees.  The RX side has the
			 * same guarantee in nsh_session_begin().
			 */
			nsh_tx_discard();
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
			/*
			 * Push whatever the shell queued, then wait -- briefly if the ring
			 * still holds something, otherwise until something happens.  Bytes
			 * left over mean either the socket refused (and NetX may not call
			 * anything back) or NSH_TX_BURST capped the pass; both want another
			 * pass soon rather than a full idle sleep.
			 *
			 * The RING, not the flag, decides: producers fill it and THEN raise
			 * NSH_EVT_TX, so reading the count after draining and before arming
			 * the wait cannot miss a producer.  One that lands in between finds
			 * the flag sticky and the wait returns at once.
			 */
			(void)nsh_tx_drain();
			(void)tx_event_flags_get(&g_evt,
			                         NSH_EVT_DISC | NSH_EVT_CMD | NSH_EVT_TX,
			                         TX_OR_CLEAR, &f,
			                         cli_uart_ring_count(&g_ctx.tx) != 0u
			                                 ? NSH_DRAIN_POLL_MS : NSH_IDLE_MS);
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

/* Ask the service thread to close the session.  Leaves the auto-arm latch alone: WHY we
 * are stopping decides whether the console should come back on its own, and only the
 * caller knows that. */
static void nsh_request_stop(void)
{
	g_req_stop = 1u;
	(void)tx_event_flags_set(&g_evt, NSH_EVT_CMD, TX_OR);
}

void net_shell_stop(void)
{
	if (!g_ready)
		return;
	g_autoarm  = 0u;      /* an explicit stop must not be undone by the next `net dhcp` */
	nsh_request_stop();
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
	/*
	 * nsh_request_stop(), NOT net_shell_stop(): this is the INTERFACE going away
	 * (nx_net.c calls it while unwinding), not the operator saying "I do not want a
	 * console".  Clearing the auto-arm latch here made `net down` disable telnet for
	 * good -- the following `net up` + `net dhcp` armed nothing and the only way back
	 * was an explicit `net shell start`.  Found on board #2 (issue #30 B1 testing).
	 */
	nsh_request_stop();

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
	/* Bytes per segment is the interactive-health number: a whole redraw should leave as
	 * one segment, and 17 B/seg (issue #49) meant a peer ACK per keystroke fragment. */
	cli_print(sh, "  tx segs   %lu   avg %lu B/seg\r\n",
	          (unsigned long)g_tx_segs,
	          (unsigned long)(g_tx_segs ? g_tx_bytes / g_tx_segs : 0u));
	cli_print(sh, "  tx queue  %lu (back-pressure, normal)   window %lu%s   no-buf %lu%s\r\n",
	          (unsigned long)g_tx_qdepth, (unsigned long)g_tx_win,
	          g_tx_win ? "  <-- peer is not reading" : "",
	          (unsigned long)g_tx_nobuf,
	          g_tx_nobuf ? "  <-- pool too small" : "");
	/* The ring high-water says whether the CLI ever had to wait at all, and dropped says
	 * whether that wait ran out -- the pair issue #48 had no way to ask for.  Non-zero
	 * `dropped` means output was LOST, which is otherwise entirely silent. */
	cli_print(sh, "  tx ring   %lu/%u B peak   dropped %lu B%s\r\n",
	          (unsigned long)g_tx_hiwater, (unsigned)(NSH_TX_RING - 1u),
	          (unsigned long)(g_ctx.sh ? g_ctx.sh->tx_dropped : 0u),
	          (g_ctx.sh && g_ctx.sh->tx_dropped) ? "  <-- output was truncated" : "");
	if (g_last != NULL)
		cli_print(sh, "  last      %s\r\n", g_last);
	/*
	 * The same report to the log.  The console's own status is most wanted exactly when
	 * that console is in trouble, and cli_tx_send_blocking() discards a handler's output
	 * once Ctrl+C is latched (cli_core.c:456-459) -- issue #23 U4-1 lost a whole
	 * hardware measurement to that.  `dmesg` consults neither the output path nor
	 * cancel_req, and survives a reset.
	 */
	LOG_INF("status: %s, port %u, auto-arm %s, sessions %lu, rx %lu B, tx %lu B in "
	        "%lu segs (%lu B/seg), rx-drops %lu, tx queue %lu, window %lu, no-buf %lu, "
	        "ring peak %lu B, dropped %lu B, last: %s",
	        st, (unsigned)g_port, g_autoarm ? "on" : "off", (unsigned long)g_sessions,
	        (unsigned long)g_rx_bytes, (unsigned long)g_tx_bytes,
	        (unsigned long)g_tx_segs,
	        (unsigned long)(g_tx_segs ? g_tx_bytes / g_tx_segs : 0u),
	        (unsigned long)g_rx_drops, (unsigned long)g_tx_qdepth,
	        (unsigned long)g_tx_win, (unsigned long)g_tx_nobuf,
	        (unsigned long)g_tx_hiwater,
	        (unsigned long)(g_ctx.sh ? g_ctx.sh->tx_dropped : 0u),
	        g_last ? g_last : "?");
}
