/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    host_glue.h
 * @brief   Host-side, ThreadX-free stand-in for cli_core.c's tx_* glue + a
 *          synchronous RX pump, shared by all host tests.
 *
 * On target, cli_core.c owns the instance thread, the TX mutex, the
 * CLI_EVT_RX/TX event flags and the backend notify functions.  None of that
 * exists on the host, so host_glue.c provides ThreadX-free replacements:
 *   - cli_lock/cli_unlock           -> no-ops (single-threaded host)
 *   - cli_transport_notify_rx/tx    -> no-ops (the pump drives RX synchronously)
 *   - cli_tx_send_blocking          -> faithful flow control over tr->api->write
 *   - cli_test_pump                 -> mirrors cli_core.c's RX drain loop
 *
 * Only the prototypes that tests call directly live here.
 */
#ifndef HOST_GLUE_H
#define HOST_GLUE_H

struct cli_instance;

/**
 * Drain the instance's transport and feed every byte to the line state machine,
 * exactly like cli_core.c's thread loop does on a CLI_EVT_RX signal.  Call after
 * cli_dummy_inject() to run injected input "through the backend".
 */
void cli_test_pump(struct cli_instance *sh);

/**
 * TX-wait hook.  cli_tx_send_blocking() calls it every time the transport's
 * write() reports "full" (returned 0) -- the host analogue of blocking on
 * CLI_EVT_TX until the backend's cli_transport_notify_tx().  A hook that frees TX
 * capacity (cli_dummy_free_tx) lets the send resume; a NULL hook means no space
 * is ever granted, so the send times out and drops the remainder (req §11).
 */
typedef void (*cli_test_tx_wait_fn)(struct cli_instance *sh, void *arg);

/** Install (or clear, with NULL) the TX-wait hook.  Reset it between tests. */
void cli_test_set_tx_wait_hook(cli_test_tx_wait_fn fn, void *arg);

/**
 * Sleep hook (issue #16).  The host cli_sleep() calls it once between its two
 * cancel polls, so a test can inject a 0x03 to model a Ctrl+C arriving during
 * the delay (a NULL hook means the delay simply elapses).  Reset between tests.
 */
void cli_test_set_sleep_hook(cli_test_tx_wait_fn fn, void *arg);

#endif /* HOST_GLUE_H */
