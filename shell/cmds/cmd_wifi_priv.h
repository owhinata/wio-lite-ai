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

/* cmd_wifi_flash.c: the issue-#19 flash/image handlers, registered by cmd_wifi.c. */
int cmd_wifi_flashread(struct cli_instance *sh, int argc, char **argv);
int cmd_wifi_flashtest(struct cli_instance *sh, int argc, char **argv);
int cmd_wifi_flashinfo(struct cli_instance *sh, int argc, char **argv);
int cmd_wifi_flashbackup(struct cli_instance *sh, int argc, char **argv);
int cmd_wifi_imgload(struct cli_instance *sh, int argc, char **argv);
int cmd_wifi_imginfo(struct cli_instance *sh, int argc, char **argv);
int cmd_wifi_flashwrite(struct cli_instance *sh, int argc, char **argv);

/* cmd_wifi_link.c: the `wifi link <sub>` table (CLI_SUBCMD_SET_END-terminated), nested
 * into the wifi table the same way `net shell` nests its own. */
extern const struct cli_cmd wifi_link_subcmds[];

#endif /* CMD_WIFI_PRIV_H */
