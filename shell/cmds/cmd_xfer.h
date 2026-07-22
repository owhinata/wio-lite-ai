/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    cmd_xfer.h
 * @brief   Shared YMODEM-over-console send entry point.
 *
 * cmd_xfer.c owns the console hand-over + YMODEM driving for an injected byte
 * source, so every future sender (issue #19's `wifi flashbackup`, and an `xfer
 * send <path>` once #6 brings a filesystem) shares one copy of the RX flush /
 * progress messaging / result mapping.
 *
 * Ported from ../stm32f746g-disco (its issue #50) for issue #19 M4, minus the
 * FileX-backed `xfer send sd|fs` command -- this repo has no filesystem yet, so
 * only the reusable helper is here and no `xfer` command is registered.
 */
#ifndef CMD_XFER_H
#define CMD_XFER_H

#include "ymodem.h"

#ifdef __cplusplus
extern "C" {
#endif

struct cli_instance;

/**
 * Send @p src to the PC over the shell's console using YMODEM, assuming the
 * caller ALREADY holds the console (cli_console_claim).  Flushes RX, runs
 * ymodem_send(), flushes RX again and prints a one-line result.
 *
 * Use this from a command that must keep the console claimed across the whole
 * operation (e.g. `wifi flashbackup`, which holds an RTL8720 download session
 * open around the transfer): cli_console_claim() is NOT reentrant -- it sets the
 * plain `cli_xfer_active` flag that cli_console_release() unconditionally clears
 * (shell/core/cli_core.c), so a nested claim/release pair would drop raw mode
 * while the outer caller still needs it.
 *
 * CRITICAL -- console RX ownership: for the duration of ymodem_send() the ONLY
 * permitted reader of the console RX ring is this function's io_getc().  A data
 * source must NOT poll cli_cancel_requested() (directly or via an abort hook):
 * cli_cancel_poll() drains the RX ring and DISCARDS every non-0x03 byte, which
 * would swallow the receiver's ACK/'C'/CAN and break the protocol.  Sources run
 * with their abort hook disabled; a local Ctrl+C is picked up by io_getc() and
 * mapped to YM_IO_ABORT instead.
 *
 * Returns 0 on a completed transfer, 1 on any failure / cancel.
 */
int xfer_send_source_locked(struct cli_instance *sh, const struct ym_source *src);

/**
 * Claim the console, call xfer_send_source_locked(), then release it.  This is
 * the f746 `xfer_send_source()` semantic, for callers that do not already own
 * the console.  Returns 0 on a completed transfer, 1 on any failure / cancel
 * (including a background-job rejection).
 */
int xfer_send_source(struct cli_instance *sh, const struct ym_source *src);

#ifdef __cplusplus
}
#endif

#endif /* CMD_XFER_H */
