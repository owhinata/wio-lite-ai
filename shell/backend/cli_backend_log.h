/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    cli_backend_log.h
 * @brief   Write-only shell transport that lands in the RAM log (issue #37).
 *
 * A `struct cli_transport` with no terminal behind it: everything written goes
 * into the reset-persistent log ring, one record per line, and nothing is ever
 * read.  It exists so code that reports through the shell API can be driven with
 * no console attached -- specifically the boot configuration sequence, which runs
 * before anyone can be watching and whose account of what it did therefore has to
 * survive into `dmesg`.
 *
 * The alternative would have been to make every reporting call in that path
 * tolerate a NULL instance, which means a NULL check at each of them in code that
 * already works.  A transport is the smaller change and keeps the messages, at the
 * cost of one `struct cli_instance`.
 *
 * Use it with cli_init() and WITHOUT cli_start(): there is nothing to read, so an
 * instance thread would have nothing to do.  cli_init() creates the output mutex
 * and event flags that cli_print()/cli_sleep() need, which is all this is for.
 *
 * write() always accepts everything, so the core never suspends waiting for TX
 * space -- important precisely because there is no instance thread to be woken.
 */
#ifndef CLI_BACKEND_LOG_H
#define CLI_BACKEND_LOG_H

#include "cli_instance.h"
#include "log.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Backend-private context: the line assembler and the tag records get. */
struct cli_log_backend {
	struct log_line line;
	const char     *tag;
};

extern const struct cli_transport_api cli_backend_log_api;

/** Define a log transport named @p _name, tagging its records with @p _tag. */
#define CLI_BACKEND_LOG_DEFINE(_name, _tag)                                    \
	static struct cli_log_backend _name##_ctx = { .tag = (_tag) };         \
	static struct cli_transport _name = {                                  \
		.api = &cli_backend_log_api,                                   \
		.ctx = &_name##_ctx,                                           \
	}

/** Emit any partial line the producer left behind (end of a boot sequence). */
void cli_backend_log_flush(struct cli_transport *tr);

#ifdef __cplusplus
}
#endif

#endif /* CLI_BACKEND_LOG_H */
