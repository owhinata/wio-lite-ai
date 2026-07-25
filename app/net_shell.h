/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    net_shell.h
 * @brief   telnet shell console over the RTL8720DN socket offload (issue #21 increment 9).
 *
 * A second `struct cli_instance` bound to a transport whose bytes travel over the module's
 * eRPC BSD-socket offload (app/wifi_rpc.c), so `telnet <board-ip>` gets the same shell as
 * the USB CDC console, concurrently with it.  Port of ../stm32f746g-disco's
 * port/netxduo/nx_shell.c: same transport vtable, `connected` write gate, telnet IAC strip
 * and single-session (N=1) instance reuse -- only the transport underneath differs (there,
 * NetX Duo on an on-chip MAC; here, RPCs to a companion chip).
 *
 * One resident service thread owns the whole socket side: it is the only caller of
 * accept/recv/send/close for these fds and the only producer of the RX ring.  It issues at
 * most ONE blocking module call at a time, which is what keeps the issue-#20 N3 firmware's
 * second worker free for the shell's own RPCs (fw/rtl8720/README.md, "Constraints when
 * building real concurrent TCP on top of N3").
 *
 * No clock/RCC/register work of its own -- pure marshalling over the existing link
 * (XIP-safe).  Layering: HAL/CMSIS/ThreadX <- erpc/wifi_rpc/rtl_link <- shell <- here.
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
	NET_SHELL_ARMING,      /* taking the link and opening the listening socket        */
	NET_SHELL_LISTENING,   /* waiting for a client (a blocking accept is outstanding) */
	NET_SHELL_SESSION,     /* a client is connected                                   */
	NET_SHELL_STOPPING,    /* closing the sockets and dropping the UART reference     */
};

/*
 * Create the service thread and its ThreadX objects.  Call ONCE from
 * tx_application_define() -- like erpc_service_init() it only creates objects (no
 * HAL_GetTick dependency, see issue #12) and the thread parks on its event flags until
 * something arms it.  Returns 0, or -1 if an object could not be created (fail-soft: the
 * telnet console is then simply unavailable and the rest of the firmware runs).
 */
int net_shell_init(void);

/*
 * "An IPv4 address just came up" -- arm the console if it is enabled and idle.  Called from
 * the success path of `wifi connect` / `net dhcp` / `net ip`, AFTER rtl_link_end() so the
 * coarse link mutex is free for the service thread to take.  Only posts a request; it never
 * waits and never fails loudly (`net shell status` reports the outcome).
 */
void net_shell_autoarm(void);

/*
 * Request arm / teardown from a shell command.  Both only POST the request; the caller polls
 * net_shell_state() (see shell/cmds/cmd_net.c) rather than blocking here, because the
 * service thread may first have to finish a blocking accept (up to ~10 s) or wait for the
 * coarse link mutex that another console's command is holding.
 * net_shell_start() returns 0 when the request was posted, -1 when it was refused (already
 * armed, or the link is dirty and needs `wifi reset`) with the reason in *@why.
 * net_shell_stop() also disables auto-arm, so a later `net dhcp` does not silently bring the
 * console back and block the flash / bridge commands again.
 */
int  net_shell_start(uint16_t port, const char **why);
void net_shell_stop(void);

enum net_shell_state net_shell_state(void);

/* True while the service owns module sockets (LISTENING or SESSION) -- i.e. while a
 * blocking accept/recv of ours may be outstanding on the module. */
bool net_shell_armed(void);

/* Print the `net shell status` report (state, address/port, counters, eRPC diagnostics and
 * the last failure reason) to @sh. */
void net_shell_print_status(struct cli_instance *sh);

/* True when @sh is the telnet console instance -- or a background-job worker launched from
 * it (`sh->fg`), which shares its transport. */
bool net_shell_is_console(const struct cli_instance *sh);

/*
 * Command guards.  Both print the reason and return non-zero when the command must NOT run;
 * 0 means "carry on".
 *
 * net_shell_guard(): @sh is the telnet console and @what would destroy the transport it is
 *   running on (power-cycle the module, or take over the console byte stream for YMODEM).
 *   Not a policy -- the shell deliberately has no command restrictions (f746 issue #49 P4);
 *   this is only self-destruct avoidance, so it refuses on the telnet console alone.
 * net_shell_guard_link(): the console is armed and @what would put a SECOND long-blocking
 *   socket call on the module.  The N3 firmware has two workers and ours already owns one,
 *   so a second blocking call starves everything else on the link -- refuse from EVERY
 *   console until `net shell stop`.
 */
int net_shell_guard(struct cli_instance *sh, const char *what);
int net_shell_guard_link(struct cli_instance *sh, const char *what);

#endif /* APP_NET_SHELL_H */
