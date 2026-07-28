/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    cmd_wifi_priv.h
 * @brief   Private seams of the `wifi` command family (issue #28).
 *
 * The `wifi` root command spans three translation units -- cmd_wifi.c (power / L2 /
 * eRPC), cmd_wifi_flash.c (issue #19 download-mode flashing) and cmd_wifi_link.c
 * (issue #23 link diagnostics, the `wifi link` subtree) -- but registers ONE table in
 * cmd_wifi.c.  This header is the seam between them and nothing else includes it.
 */
#ifndef CMD_WIFI_PRIV_H
#define CMD_WIFI_PRIV_H

#include "cli.h"

/* The two nested tables (CLI_SUBCMD_SET_END-terminated) the wifi table points at, the
 * same way `net shell` nests its own.  Each subtree's handlers are static to its TU --
 * the table pointer is the whole seam.
 *   cmd_wifi_flash.c: `wifi flash <sub>`, the issue-#19 download-mode flasher
 *   cmd_wifi_link.c:  `wifi link <sub>`,  the issue-#23 link diagnostics
 */
extern const struct cli_cmd wifi_flash_subcmds[];
extern const struct cli_cmd wifi_link_subcmds[];

#endif /* CMD_WIFI_PRIV_H */
