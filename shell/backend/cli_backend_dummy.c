/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    cli_backend_dummy.c
 * @brief   In-memory loopback transport implementation.
 *
 * Implements struct cli_transport_api against the per-instance struct cli_dummy
 * context.  Calls no tx_* API (only cli_transport_notify_rx from inject), so it
 * builds and runs on the host as well as on target.  See cli_backend_dummy.h for
 * the capture-log vs TX-capacity model and the threading assumptions.
 */
#include "cli_backend_dummy.h"

/* ---- transport vtable -------------------------------------------------- */

static int dummy_init(struct cli_transport *tr)
{
	struct cli_dummy *d = (struct cli_dummy *)tr->ctx;
	d->rx_head = d->rx_tail = 0;
	d->rx_dropped = 0;
	d->cap_len = 0;
	d->cap[0] = '\0';
	d->tx_unlimited = 1;     /* sane default: accept everything until bounded */
	d->tx_free = 0;
	d->tx_fail = 0;
	return 0;
}

static int dummy_enable(struct cli_transport *tr)
{
	(void)tr;
	return 0;
}

static int dummy_read(struct cli_transport *tr, uint8_t *data, size_t cap)
{
	struct cli_dummy *d = (struct cli_dummy *)tr->ctx;
	size_t n = 0;

	while (n < cap && d->rx_tail != d->rx_head) {
		data[n++] = d->rx[d->rx_tail];
		d->rx_tail = (d->rx_tail + 1u) % CLI_DUMMY_RX_BUFFER_SIZE;
	}
	return (int)n;
}

/*
 * Non-blocking write (req §11 contract).  Accept up to the current TX capacity,
 * append the accepted bytes to the capture log, and return the count accepted
 * (0..len); 0 means "TX full" and the core will block on TX space.  In
 * immediate-failure mode return -1.  The amount accepted is governed by TX
 * capacity, not by the capture log -- the log is only a recording (sized so
 * tests never overflow it; a full log silently stops recording).
 */
static int dummy_write(struct cli_transport *tr, const uint8_t *data, size_t len)
{
	struct cli_dummy *d = (struct cli_dummy *)tr->ctx;

	if (d->tx_fail)
		return -1;

	size_t accept = d->tx_unlimited ? len
	                                : (d->tx_free < len ? d->tx_free : len);

	for (size_t i = 0; i < accept; i++) {
		if (d->cap_len < CLI_DUMMY_CAP_BUFFER_SIZE) {
			d->cap[d->cap_len++] = (char)data[i];
			d->cap[d->cap_len] = '\0';
		}
	}
	if (!d->tx_unlimited)
		d->tx_free -= accept;

	return (int)accept;
}

const struct cli_transport_api cli_dummy_api = {
	dummy_init, dummy_enable, dummy_write, dummy_read, NULL, NULL, NULL,
};

/* ---- test/driver helpers ----------------------------------------------- */

void cli_dummy_inject(struct cli_transport *tr, const void *data, size_t len)
{
	struct cli_dummy *d = (struct cli_dummy *)tr->ctx;
	const uint8_t   *p = (const uint8_t *)data;

	for (size_t i = 0; i < len; i++) {
		size_t next = (d->rx_head + 1u) % CLI_DUMMY_RX_BUFFER_SIZE;
		if (next == d->rx_tail) {       /* ring full: drop (req §9/§18 10e) */
			d->rx_dropped++;
			if (tr->sh)
				tr->sh->rx_dropped++;
			continue;
		}
		d->rx[d->rx_head] = p[i];
		d->rx_head = next;
	}

	/* Like a real backend ISR: only signal; never touch line state (req §10). */
	if (tr->sh)
		cli_transport_notify_rx(tr->sh);
}

const char *cli_dummy_output(struct cli_transport *tr, size_t *len)
{
	struct cli_dummy *d = (struct cli_dummy *)tr->ctx;
	if (len)
		*len = d->cap_len;
	return d->cap;
}

const char *cli_dummy_output_str(struct cli_transport *tr)
{
	struct cli_dummy *d = (struct cli_dummy *)tr->ctx;
	return d->cap;
}

void cli_dummy_free_tx(struct cli_transport *tr, size_t n)
{
	struct cli_dummy *d = (struct cli_dummy *)tr->ctx;
	if (!d->tx_unlimited)
		d->tx_free += n;
}

void cli_dummy_set_tx_cap(struct cli_transport *tr, size_t cap)
{
	struct cli_dummy *d = (struct cli_dummy *)tr->ctx;
	if (cap == 0) {
		d->tx_unlimited = 1;
		d->tx_free = 0;
	} else {
		d->tx_unlimited = 0;
		d->tx_free = cap;
	}
}

void cli_dummy_set_tx_fail(struct cli_transport *tr, int on)
{
	struct cli_dummy *d = (struct cli_dummy *)tr->ctx;
	d->tx_fail = on ? 1 : 0;
}

void cli_dummy_clear_output(struct cli_transport *tr)
{
	struct cli_dummy *d = (struct cli_dummy *)tr->ctx;
	d->cap_len = 0;
	d->cap[0] = '\0';
}

void cli_dummy_clear_rx(struct cli_transport *tr)
{
	struct cli_dummy *d = (struct cli_dummy *)tr->ctx;
	d->rx_head = d->rx_tail = 0;
}

void cli_dummy_reset_stats(struct cli_transport *tr)
{
	struct cli_dummy *d = (struct cli_dummy *)tr->ctx;
	d->rx_dropped = 0;
}
