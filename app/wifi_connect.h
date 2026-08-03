/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    wifi_connect.h
 * @brief   The association sequence, shared by `wifi connect` and boot (issue #37).
 *
 * Associating with an AP is not one call.  It is a fixed order of claims and
 * releases across four modules -- the coarse link mutex, the module's bridge
 * watchdog, its lwIP init, the WiFi mode cycle, the association itself, then the
 * host stack's bridge -- and several of those steps only work in one position.
 * The comments inside say which and why; the important thing here is that there is
 * exactly ONE copy of that order.
 *
 * It lived in shell/cmds/cmd_wifi.c until the configuration store grew a boot
 * sequence that needs the same thing (issue #37 step 4).  Re-deriving it there
 * would have put the ordering constraints in two places, which is how they drift.
 *
 * Reporting goes through a `struct cli_instance` rather than a log call, because
 * every module this drives already reports that way.  The boot caller supplies an
 * instance on the log transport (shell/backend/cli_backend_log.h), so its account
 * of what happened lands in `dmesg` instead of on a console nobody is watching.
 * The instance is never optional: passing NULL would mean a NULL check at every
 * reporting call in four modules that currently have none.
 */
#ifndef APP_WIFI_CONNECT_H
#define APP_WIFI_CONNECT_H

#include <stdbool.h>
#include <stdint.h>

#include "erpc.h"
#include "wifi_rpc.h"

struct cli_instance;

/* Firmware generation the L2 bridge needs (2.1.3+wio-n7), mirroring nx_net.c's own
 * gate.  Checked before the association rather than after it. */
#define WIFI_BRIDGE_MIN_GEN 7u

/**
 * Put the module in STA mode, associate with @p ssid, and arm the L2 bridge.
 *
 * @param pass      NULL for an open network.
 * @param security  WIFI_RPC_SEC_*; the caller picks the default.
 * @param security_explicit  the caller was GIVEN that value rather than deriving
 *                  it.  Only affects the failure message: a wrong security is the
 *                  usual reason an association fails, so the failure suggests
 *                  naming one -- which is noise for a caller that already did.
 * @param earn_gen  what to do when the host does not know which firmware is on the
 *                  module.  The bridge refuses to run against an unknown one, and
 *                  that proof is lost on every reset because CHIP_EN is driven low
 *                  at every boot -- so an unattended sequence would never get past
 *                  it.  false (the interactive path) reports that and stops, which
 *                  keeps `wifi ver` the deliberate act issue #20 made it: the
 *                  version query corrupts the heap of pre-N2 firmware.  true (the
 *                  boot path) queries the version itself.
 *
 * Associating is RETRIED (issue #40): the radio fails intermittently and a repeat is
 * what recovers it, so a single failure is not reported as one.  See the loop for the
 * measurements behind that and for the one reason that is never retried.
 *
 * Returns 0 on success.  Reports everything it does through @p sh.
 */
int wifi_connect_run(struct cli_instance *sh, const char *ssid, const char *pass,
                     uint32_t security, bool security_explicit, bool earn_gen);

/* ---- pieces the `wifi` command shares with the sequence ------------------ */

/** Extract N from a "2.1.3+wio-nN" build id; 0 if the string is not one of ours. */
uint8_t wifi_fw_gen_of(const char *ver);

/** The Ctrl+C-abortable option block every eRPC call in these flows uses. */
void wifi_opts(struct wifi_rpc_opts *o, struct cli_instance *sh,
               struct erpc_diag *diag);

/** The failure epilogue: what the link saw, and the way out. */
void wifi_fail_diag(struct cli_instance *sh, const struct erpc_diag *diag);

/** Hold the module's bridge watchdog open across a long flow.  0 to proceed. */
int wifi_hold_bridge(struct cli_instance *sh, const char *what);

/** Land the radio in STA mode.  0, -4 on Ctrl+C, or -1 with the reason printed. */
int wifi_enter_sta(struct cli_instance *sh, struct wifi_rpc_opts *o);

#endif /* APP_WIFI_CONNECT_H */
