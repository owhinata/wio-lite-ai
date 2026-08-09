/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    net_shell.h
 * @brief   telnet shell console on the host's own TCP/IP stack (issue #23 U4-2).
 *
 * A second `struct cli_instance` bound to a transport whose bytes travel over a NetX Duo
 * TCP socket on this MCU, so `telnet <board-ip>` gets the same shell as the USB CDC
 * console, concurrently with it.  Port of ../stm32f746g-disco's port/netxduo/nx_shell.c:
 * same transport vtable, `connected` write gate, telnet IAC strip and single-session
 * (N=1) instance reuse.
 *
 * ---- what U4-2 changed, and why it matters more than it looks -------------------
 *
 * Until U4-2 this console ran on the RTL8720DN's own lwIP, reached by an eRPC round trip
 * per operation (issue #21).  That shaped everything: one blocking module call at a time,
 * because the module's firmware has two worker tasks and ours permanently occupied one;
 * no aborting an accept or a receive, because the module keeps running a call the host
 * gave up on; a "dirty" latch and deliberately leaked file descriptors when a call did not
 * come back; and every output chunk bounded by the module's UART ring.
 *
 * None of that exists now.  The stack is on this side of the link, so there is no far end
 * still running an operation we abandoned, nothing to leak, and no worker to compete for.
 * What replaces it is smaller and stricter: a server thread that owns the socket AND is the
 * only thing that transmits, NetX callbacks that only push bytes into a ring or set a flag,
 * and a teardown that must PROVE it happened.
 *
 * U4-2 also deleted the TX ring, on the reasoning that TCP's own back-pressure could
 * replace it.  Issue #48 put it back: the callbacks that were supposed to signal that
 * back-pressure do not all exist, and a deadline shorter than the TCP retransmit timeout
 * turned one lost segment into a truncated report.  The mechanism, and what makes the ring
 * load-bearing rather than a buffer, is documented at the top of net_shell.c.
 *
 * ---- the console REQUIRES the host stack ---------------------------------------
 *
 * `net shell start` is refused unless nx_net_is_up(), because the socket lives on that
 * interface.  Conversely nx_net_down() cannot simply drop the interface out from under a
 * live socket: app/link_data.h's detach ordering says that once the DATA channel's
 * consumer is detached, the link service thread resumes flushing "stale" bytes -- which
 * are the middle of a frame if anything can still transmit.  So the interface's teardown
 * calls net_shell_stop_sync() FIRST and refuses to proceed if it cannot be proved.
 *
 * ---- one invariant this file owes the rest of the firmware ----------------------
 *
 * The server thread takes NEITHER the rtl_link coarse mutex NOR nx_net.c's nxn_addr_lock.
 * It touches only NetX.  That is what lets nx_net.c call net_shell_stop_sync() while
 * holding the coarse mutex without creating a lock-order cycle.  Do not add a link call
 * to this thread.
 *
 * No clock/RCC/register work of its own (clock-safe).  Layering:
 * HAL/CMSIS/ThreadX <- NetX <- nx_net/link_data <- shell <- here.
 */
#ifndef APP_NET_SHELL_H
#define APP_NET_SHELL_H

#include <stdbool.h>
#include <stdint.h>

struct cli_instance;
struct cli_transport;

/* The transport a second CLI_INSTANCE_DEFINE() binds to (app/main.c). */
extern struct cli_transport net_shell_transport;

/* Default listening port. */
#define NET_SHELL_PORT_DEFAULT 23u

/* Service state, as reported by net_shell_state() / `net shell status`. */
enum net_shell_state {
	NET_SHELL_OFF = 0,     /* not armed (never started, stopped, or torn down)        */
	NET_SHELL_ARMING,      /* creating the socket and starting to listen              */
	NET_SHELL_LISTENING,   /* the passive open is armed, waiting for a client         */
	NET_SHELL_SESSION,     /* a client is connected                                   */
	NET_SHELL_STOPPING,    /* unwinding the socket                                    */
};

/*
 * Create the service thread and its ThreadX objects.  Call ONCE from
 * tx_application_define() -- object creation only (no HAL_GetTick dependency, see issue
 * #12) and the thread parks on its event flags until something arms it.  Returns 0, or -1
 * if an object could not be created (fail-soft: the telnet console is then simply
 * unavailable and the rest of the firmware runs).
 */
int net_shell_init(void);

/*
 * "An IPv4 address just came up on the HOST stack" -- arm the console if it is enabled and
 * idle.  Called from the success path of `net dhcp` / `net ip` when nx_net_is_up().  Only
 * posts a request; it never waits and never fails loudly (`net shell status` reports the
 * outcome).
 */
void net_shell_autoarm(void);

/*
 * Request arm / teardown from a shell command.  Both only POST the request; the caller
 * polls net_shell_state() (see shell/cmds/cmd_net.c).
 * net_shell_start() returns 0 when the request was posted, -1 when it was refused (already
 * armed, or the host stack is not up) with the reason in *@why.
 * net_shell_stop() also disables auto-arm, so a later `net dhcp` does not silently bring
 * the console back.  It is the ONLY thing that clears that latch: stopping because the
 * INTERFACE is going away (net_shell_stop_sync(), below) must leave it set, or one
 * `net down` kills telnet until an explicit `net shell start`.
 */
int  net_shell_start(uint16_t port, const char **why);
void net_shell_stop(void);

/*
 * Stop, and PROVE the console can no longer queue TCP output.
 *
 * Returns 0 when that is proved -- which means specifically that
 * nx_tcp_socket_delete() returned NX_SUCCESS, so the socket is unbound, CLOSED and off the
 * IP instance's list, and no thread is inside the write path.  Returns -1 when it could
 * not be proved within @timeout_ms; the caller must then treat the console as still
 * capable of transmitting (app/nx_net.c takes its FAILED path and names `wifi reset`).
 *
 * "Delete was attempted" is NOT the contract.  A socket left bound would keep NetX able to
 * hand this driver a frame, which is exactly what the DATA channel's detach ordering
 * forbids.
 *
 * Safe to call while holding the rtl_link coarse mutex: the server thread takes no link
 * lock of any kind (see the header comment).  It is a no-op returning 0 when already OFF.
 *
 * ⚠️ It blocks the caller for up to @timeout_ms -- typically a few ms, since the server
 * is woken by an event rather than polled.  nx_net.c calls it with the coarse mutex held,
 * so another console's `wifi` / `net` / `link` command can be delayed by that much.
 */
int net_shell_stop_sync(uint32_t timeout_ms);

enum net_shell_state net_shell_state(void);

/* True while the service owns a socket (ARMING..STOPPING). */
bool net_shell_armed(void);

/* Print the `net shell status` report to @sh.  Also emitted to the log, because the
 * console's own status is most wanted exactly when that console is failing -- and
 * cli_tx_send_blocking() discards a handler's output once Ctrl+C is latched. */
void net_shell_print_status(struct cli_instance *sh);

/* True when @sh is the telnet console instance -- or a background-job worker launched from
 * it (`sh->fg`), which shares its transport. */
bool net_shell_is_console(const struct cli_instance *sh);

/*
 * Command guard.  Prints the reason and returns non-zero when the command must NOT run:
 * @sh is the telnet console and @what would destroy the transport it is running on
 * (power-cycle the module, take the host stack down, or take over the console byte stream
 * for YMODEM).  Not a policy -- the shell deliberately has no command restrictions (f746
 * issue #49 P4); this is only self-destruct avoidance, so it refuses on the telnet console
 * alone.
 *
 * (issue #21's net_shell_guard_link() is gone: since issue #23 U4-2 the console is a
 * NetX socket and no longer occupies one of the module firmware's workers, so there is
 * nothing left to protect module-side eRPC callers from.)
 */
int net_shell_guard(struct cli_instance *sh, const char *what);

#endif /* APP_NET_SHELL_H */
