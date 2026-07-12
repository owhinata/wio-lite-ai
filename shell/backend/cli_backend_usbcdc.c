/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    cli_backend_usbcdc.c
 * @brief   USB CDC transport implementation (see cli_backend_usbcdc.h).
 *
 * Vtable (shell-thread side) touches only the rings; cli_usbcdc_pump()
 * (usb-thread side) is the sole caller of tud_cdc_* and bridges the rings to the
 * CDC FIFOs.  _write routes printf through the same TX ring (single TX owner).
 */
#include "cli_backend_usbcdc.h"
#include "cli_internal.h"   /* cli_out_begin/cli_out_end: serialise printf with cli_print */
#include "stm32h7xx_hal.h"  /* CMSIS __get_PRIMASK / __disable_irq / __set_PRIMASK */
#include "tusb.h"

/* PRIMASK critical section.  Nests safely inside ThreadX's own PRIMASK section. */
#define CLI_USBCDC_CRIT_ENTER()  do { uint32_t _pm = __get_PRIMASK(); __disable_irq()
#define CLI_USBCDC_CRIT_EXIT()   __set_PRIMASK(_pm); } while (0)

/* The one active CDC console (set in init).  _write and the pump reach the context
 * through this; NULL until the first cli_init(). */
static struct cli_usbcdc *g_usbcdc_console;

/* ---- transport vtable (shell-thread side: rings only) ------------------- */

static int usbcdc_init(struct cli_transport *tr)
{
	struct cli_usbcdc *u = (struct cli_usbcdc *)tr->ctx;

	if (u == NULL)
		return -1;

	cli_uart_ring_init(&u->rx_ring, u->rx_buf, sizeof u->rx_buf);
	cli_uart_ring_init(&u->tx_ring, u->tx_buf, sizeof u->tx_buf);
	u->enabled       = 0;
	u->was_connected = 0;
	u->conn_synced   = 0;
	u->rx_dropped_hw = 0;
	u->sh            = tr->sh;      /* cli_init() set tr->sh before calling init */

	g_usbcdc_console = u;           /* become the console so _write routes here */
	return 0;
}

static int usbcdc_enable(struct cli_transport *tr)
{
	struct cli_usbcdc *u = (struct cli_usbcdc *)tr->ctx;

	/* No hardware to arm: the usb thread pumps RX/TX.  Just go live so _write
	 * routes through the ring and the pump starts shuttling bytes. */
	u->enabled = 1;
	return 0;
}

static int usbcdc_write(struct cli_transport *tr, const uint8_t *data, size_t len)
{
	struct cli_usbcdc *u = (struct cli_usbcdc *)tr->ctx;
	size_t acc;

	/* Non-blocking (req §11): enqueue what fits, return the count.  A short/zero
	 * return makes the core block on CLI_EVT_TX; the usb thread frees space and
	 * fires cli_transport_notify_tx() to wake it. */
	CLI_USBCDC_CRIT_ENTER();
	acc = cli_uart_ring_put_buf(&u->tx_ring, data, len);
	CLI_USBCDC_CRIT_EXIT();

	return (int)acc;
}

static int usbcdc_read(struct cli_transport *tr, uint8_t *data, size_t cap)
{
	struct cli_usbcdc *u = (struct cli_usbcdc *)tr->ctx;

	/* SPSC: the usb thread is the only producer, this (the shell thread) the only
	 * consumer, so draining the ring needs no lock. */
	return (int)cli_uart_ring_get_buf(&u->rx_ring, data, cap);
}

const struct cli_transport_api cli_usbcdc_api = {
	usbcdc_init, usbcdc_enable, usbcdc_write, usbcdc_read, NULL, NULL, NULL,
};

/* ---- usb-thread bridge: rings <-> TinyUSB CDC -------------------------- */

void cli_usbcdc_pump(struct cli_transport *tr)
{
	struct cli_usbcdc *u = (struct cli_usbcdc *)tr->ctx;
	bool connected = tud_cdc_connected();
	int  woke_rx = 0, woke_tx = 0;

	/* On a host connect edge, ask the shell thread to reset the editor and redraw
	 * the prompt (CLI_EVT_CONN), so opening the terminal shows a fresh prompt.  Skip
	 * the very first observation (just establish the baseline): if a host is already
	 * attached at boot, the instance's initial prompt is drained to it, and posting
	 * CONN too would draw a duplicate second prompt.  (A sub-1 ms close/reopen
	 * between pump samples can still miss an edge; acceptable for v1.) */
	if (!u->conn_synced)
		u->conn_synced = 1;
	else if (connected && !u->was_connected && u->sh != NULL)
		cli_transport_notify_conn(u->sh);
	u->was_connected = (uint8_t)connected;

	/* TX: drain tx_ring -> CDC FIFO in contiguous slices (this is the sole tail
	 * mutator, so no lock; producers only touch head, under PRIMASK). */
	for (;;) {
		const uint8_t *p;
		size_t   run = cli_uart_ring_contig(&u->tx_ring, &p);
		uint32_t w;

		if (run == 0u)
			break;
		if (connected) {
			w = tud_cdc_write(p, (uint32_t)run);
			if (w)
				tud_cdc_write_flush();
		} else {
			w = (uint32_t)run;      /* no host: drop so the shell never wedges */
		}
		if (w == 0u)
			break;                  /* CDC FIFO full this round; retry next pump */
		cli_uart_ring_advance_tail(&u->tx_ring, w);
		woke_tx = 1;
		if (w < run)
			break;                  /* FIFO filled mid-slice */
	}

	/* RX: pull received bytes into rx_ring (sole producer -> no lock). */
	while (connected && tud_cdc_available()) {
		uint8_t b;

		if (tud_cdc_read(&b, 1) != 1u)
			break;
		if (!cli_uart_ring_put(&u->rx_ring, b)) {
			u->rx_dropped_hw++;             /* rx_ring full: byte lost */
			if (u->sh != NULL)
				u->sh->rx_dropped++;
			break;                          /* stop; retry next pump */
		}
		woke_rx = 1;
	}

	if (woke_tx && u->sh != NULL)
		cli_transport_notify_tx(u->sh);
	if (woke_rx && u->sh != NULL)
		cli_transport_notify_rx(u->sh);
}

/* ---- printf / _write retarget (single TX owner: the tx_ring) ----------- */

/* Resolve a shell instance to its USB CDC backend, but only when this backend owns
 * it (api identity) and the console is live.  A non-CDC transport, un-enabled or
 * NULL instance returns NULL so _write falls back to the global console. */
static struct cli_usbcdc *usbcdc_ctx_from_instance(struct cli_instance *sh)
{
	struct cli_usbcdc *u;

	if (sh == NULL || sh->tr == NULL || sh->tr->api != &cli_usbcdc_api)
		return NULL;
	u = (struct cli_usbcdc *)sh->tr->ctx;
	return (u != NULL && u->enabled) ? u : NULL;
}

/*
 * Strong _write overriding libnosys': route printf through the same TX ring as the
 * shell so the CDC has exactly one TX owner (the usb thread drains it).  Best-effort
 * (a full ring drops the remainder -- the shell's own output uses the transport
 * write() path with proper CLI_EVT_TX backpressure; printf is secondary).  Bare LF
 * -> CR+LF so a raw terminal shows printf output without staircasing.  When the
 * running thread owns a CDC shell instance, bracket the drain with cli_out_begin/
 * cli_out_end to serialise with cli_print + the line editor; from the usb thread /
 * ISR / pre-kernel, cli_current_instance() is NULL and we fall back to the global
 * console with no lock.
 */
int _write(int file, char *ptr, int len)
{
	struct cli_instance *sh = cli_current_instance();
	struct cli_usbcdc   *u  = usbcdc_ctx_from_instance(sh);
	int locked = 0;
	(void)file;

	if (len <= 0)
		return len;

	/* A raw binary transfer (issue #50) owns the console: drop printf so it cannot
	 * splice into the byte stream (its own bytes go via the transport write() path,
	 * not _write).  Matches the UART backend. */
	if (cli_xfer_active)
		return len;

	if (u == NULL)
		u = g_usbcdc_console;   /* usb thread / ISR / pre-kernel / non-shell thread */
	else
		locked = (cli_out_begin(sh) == 0);   /* serialise with cli_print + editor */

	if (u != NULL && u->enabled) {
		const uint8_t *d = (const uint8_t *)ptr;
		uint8_t prev = 0;
		int i;

		for (i = 0; i < len; i++) {
			uint8_t b = (uint8_t)d[i];

			CLI_USBCDC_CRIT_ENTER();
			if (b == (uint8_t)'\n' && prev != (uint8_t)'\r')
				(void)cli_uart_ring_put(&u->tx_ring, (uint8_t)'\r');
			(void)cli_uart_ring_put(&u->tx_ring, b);   /* best-effort: drop if full */
			CLI_USBCDC_CRIT_EXIT();
			prev = b;
		}
	}

	if (locked)
		cli_out_end(sh);
	return len;
}
