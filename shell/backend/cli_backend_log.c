/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    cli_backend_log.c
 * @brief   Write-only shell transport into the RAM log (issue #37).
 *
 * See cli_backend_log.h for what this is for.  The whole implementation is the
 * four mandatory transport entry points; the interesting work (reassembling lines,
 * dropping ANSI escapes) belongs to svc/log.c's assembler, which the FlashDB
 * logging shim shares.
 */
#include "cli_backend_log.h"

static int log_be_init(struct cli_transport *tr)
{
	struct cli_log_backend *b = (struct cli_log_backend *)tr->ctx;

	log_line_init(&b->line);
	return 0;
}

static int log_be_enable(struct cli_transport *tr)
{
	(void)tr;
	return 0;   /* nothing to enable: there is no receiver */
}

/*
 * Accept everything, always.  A short accept would oblige this backend to call
 * cli_transport_notify_tx() when space frees, and the core would suspend on
 * CLI_EVT_TX until it did -- with no instance thread here, and no hardware that
 * could ever drain, that wait would never end.  The log ring's own
 * oldest-overwrite policy is what bounds the memory instead.
 */
static int log_be_write(struct cli_transport *tr, const uint8_t *data, size_t len)
{
	struct cli_log_backend *b = (struct cli_log_backend *)tr->ctx;

	log_line_feed(&b->line, b->tag, data, len);
	return (int)len;
}

static int log_be_read(struct cli_transport *tr, uint8_t *data, size_t cap)
{
	(void)tr; (void)data; (void)cap;
	return 0;   /* write-only: nothing ever arrives */
}

const struct cli_transport_api cli_backend_log_api = {
	.init   = log_be_init,
	.enable = log_be_enable,
	.write  = log_be_write,
	.read   = log_be_read,
};

void cli_backend_log_flush(struct cli_transport *tr)
{
	struct cli_log_backend *b = (struct cli_log_backend *)tr->ctx;

	log_line_flush(&b->line, b->tag);
}
