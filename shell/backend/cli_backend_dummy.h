/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    cli_backend_dummy.h
 * @brief   In-memory loopback transport (testing / multi-instance demo).
 *
 * A portable `struct cli_transport_api` implementation with no hardware and no
 * ThreadX dependency, used to drive the shell core end-to-end without a UART:
 *   - the test injects input with cli_dummy_inject() (which, like a real backend
 *     ISR, only pushes bytes and fires cli_transport_notify_rx());
 *   - read() drains that input FIFO; the core feeds each byte to the line state
 *     machine;
 *   - write() appends accepted bytes to a capture log the test inspects.
 *
 * The capture log (what was sent, for verification) is kept separate from the TX
 * *pending capacity* (how much room the backend currently has).  This lets a
 * test reproduce req §11 backpressure deterministically: a bounded TX capacity
 * makes write() return a short count / 0 (full), and capacity is freed ONLY when
 * the test asks (cli_dummy_free_tx), modelling a real backend that calls
 * cli_transport_notify_tx() when its TX buffer drains.  Nothing is auto-freed.
 *
 * Threading: this backend is single-producer / test-only -- the FIFO and
 * capacity counters are updated without locking.  On target (#8) it is driven by
 * one shell thread; if a future use injects from an ISR or several producers, the
 * ring head/tail updates would need their own exclusion.
 *
 * Clean-room design; no third-party code reused.
 */
#ifndef CLI_BACKEND_DUMMY_H
#define CLI_BACKEND_DUMMY_H

#include <stddef.h>
#include <stdint.h>

#include "cli_instance.h"   /* struct cli_transport[_api], cli_transport_notify_rx */

#ifdef __cplusplus
extern "C" {
#endif

/* Input FIFO depth (bytes).  A burst larger than this overflows and is dropped
 * (req §9 / §18 10e); the ring holds CLI_DUMMY_RX_BUFFER_SIZE-1 bytes. */
#ifndef CLI_DUMMY_RX_BUFFER_SIZE
#define CLI_DUMMY_RX_BUFFER_SIZE 256
#endif

/* Output capture-log capacity (bytes).  Sized so tests never overflow it; +1 is
 * reserved internally for a NUL terminator (cli_dummy_output_str). */
#ifndef CLI_DUMMY_CAP_BUFFER_SIZE
#define CLI_DUMMY_CAP_BUFFER_SIZE 4096
#endif

/** Backend-private context (the `ctx` of a dummy transport). */
struct cli_dummy {
	/* RX: bytes injected by the test, drained by read(). */
	uint8_t  rx[CLI_DUMMY_RX_BUFFER_SIZE];
	size_t   rx_head;
	size_t   rx_tail;
	uint32_t rx_dropped;          /**< bytes dropped on RX-ring overflow */

	/* TX capture log: every byte write() accepted, in order; not auto-cleared. */
	char     cap[CLI_DUMMY_CAP_BUFFER_SIZE + 1];
	size_t   cap_len;

	/* TX backpressure model (independent of the capture log). */
	int      tx_unlimited;        /**< 1: accept all; 0: bounded by tx_free */
	size_t   tx_free;             /**< free TX capacity when bounded */
	int      tx_fail;             /**< 1: write() returns -1 immediately */
};

/** The dummy transport vtable (init/enable/write/read; uninit/update are NULL). */
extern const struct cli_transport_api cli_dummy_api;

/**
 * Statically define a dummy transport @p _name with its own zeroed context.
 * Bind it to an instance with CLI_INSTANCE_DEFINE(inst, &_name, "prompt> ").
 */
#define CLI_BACKEND_DUMMY_DEFINE(_name)                                       \
	static struct cli_dummy     _name##_ctx;                              \
	static struct cli_transport _name = { &cli_dummy_api, NULL, &_name##_ctx }

/* ---- test/driver helpers ----------------------------------------------- */

/**
 * Inject @p len bytes of input as if received by the backend, then fire
 * cli_transport_notify_rx() (matching a real ISR).  Excess beyond the FIFO is
 * dropped and counted in both the dummy and tr->sh->rx_dropped (req §9/§15).
 */
void cli_dummy_inject(struct cli_transport *tr, const void *data, size_t len);

/** Borrow the capture log; *@p len gets its length (may contain NULs). */
const char *cli_dummy_output(struct cli_transport *tr, size_t *len);

/** NUL-terminated view of the capture log (convenience; prefer the length form). */
const char *cli_dummy_output_str(struct cli_transport *tr);

/**
 * Grant @p n bytes of TX capacity (models a backend's cli_transport_notify_tx()
 * after its TX buffer drains).  No effect when capacity is unlimited.
 */
void cli_dummy_free_tx(struct cli_transport *tr, size_t n);

/** Set initial TX capacity: 0 == unlimited (accept everything), n>0 == bounded. */
void cli_dummy_set_tx_cap(struct cli_transport *tr, size_t cap);

/** Arm/disarm immediate-failure mode (write() returns -1). */
void cli_dummy_set_tx_fail(struct cli_transport *tr, int on);

void cli_dummy_clear_output(struct cli_transport *tr);   /**< empty the capture log */
void cli_dummy_clear_rx(struct cli_transport *tr);       /**< empty the input FIFO */
void cli_dummy_reset_stats(struct cli_transport *tr);    /**< zero rx_dropped */

#ifdef __cplusplus
}
#endif

#endif /* CLI_BACKEND_DUMMY_H */
