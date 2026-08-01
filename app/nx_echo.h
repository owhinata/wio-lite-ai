/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/*
 * Wio Lite AI (STM32H725AEI6) -- TCP echo server on the host's own stack (issue #23 U4-1).
 *
 * This is the bring-up rehearsal for the telnet console, exactly as the eRPC version was
 * for issue #21: one shell command, one thread, no concurrency of its own, exercising the
 * whole path a console will later ride on -- listen, accept, receive, send, disconnect,
 * relisten -- and reporting the numbers the console's sizing depends on.
 *
 * What it replaces did the same job over the module's lwIP via eRPC round-trips.  Here
 * every byte is framed by NetX Duo on this MCU, handed to app/nx_link_driver.c, and put on
 * the wire by the module as a raw Ethernet frame.  There is no RPC in the data path at all,
 * so what this measures is the link and the host stack rather than the request/reply
 * turnaround the old version was dominated by.
 *
 * It lives in app/ rather than shell/cmds/ for one reason: nothing above app/nx_net.h is
 * supposed to include nx_api.h (see that header), and this needs NX_TCP_SOCKET.
 *
 * No clock/RCC/register work of its own (clock-safe).
 */
#ifndef APP_NX_ECHO_H
#define APP_NX_ECHO_H

#include <stdint.h>

struct cli_instance;

/* Default listening port.  Not 23 -- that belongs to the telnet console. */
#define NX_ECHO_PORT_DEFAULT 2323u

/*
 * Run the echo server on @port until the client goes away and Ctrl+C is pressed, printing
 * a per-session and a final report to @sh.  Runs entirely on the calling CLI thread.
 *
 * The caller must have established that the host stack is up (nx_net_is_up()); this only
 * re-checks it defensively.  Returns 0 on a clean run, 1 if it could not start.
 */
int nx_echo_run(struct cli_instance *sh, uint16_t port);

#endif /* APP_NX_ECHO_H */
