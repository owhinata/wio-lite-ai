/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    cli_backend_usbcdc.h
 * @brief   USB CDC (ACM) transport for the shell on the Wio Lite AI (STM32H725).
 *
 * A `struct cli_transport_api` implementation over TinyUSB's device CDC class.
 * Unlike the stm32f746g-disco UART backend (whose ISR fills/drains the rings),
 * here a single dedicated ThreadX "usb thread" is the SOLE caller of tud_task()
 * and the tud_cdc_* API, so there are no cross-thread TinyUSB races.  The backend
 * therefore splits into two halves:
 *
 *   - the transport vtable (write/read), called from the SHELL thread and the
 *     printf _write retarget, only touches the byte rings; and
 *   - cli_usbcdc_pump(), called from the USB thread after tud_task(), shuttles
 *     bytes between the CDC FIFOs and the rings and fires cli_transport_notify_rx
 *     / _tx (so a TX-blocked or RX-waiting shell thread wakes).
 *
 * Concurrency (same PRIMASK discipline as the UART backend):
 *   - TX ring: two producers (shell write() + the _write retarget) / one consumer
 *     (the usb thread) -> producer puts are wrapped in a PRIMASK critical section.
 *   - RX ring: one producer (the usb thread) / one consumer (the shell thread's
 *     read()) -> lock-free SPSC.
 * PRIMASK is safe because this ThreadX port uses PRIMASK critical sections
 * (TX_PORT_USE_BASEPRI undefined).
 *
 * The ring helpers (cli_uart_ring.h) are reused verbatim (HAL/ThreadX-free).
 */
#ifndef CLI_BACKEND_USBCDC_H
#define CLI_BACKEND_USBCDC_H

#include <stddef.h>
#include <stdint.h>

#include "cli_instance.h"    /* struct cli_transport[_api], cli_transport_notify_* */
#include "cli_uart_ring.h"   /* struct cli_uart_ring + lock-free helpers */

#ifdef __cplusplus
extern "C" {
#endif

/* RX ring depth (bytes).  Holds CLI_USBCDC_RX_BUFFER_SIZE-1 bytes; a burst that
 * outruns the shell thread overflows and is dropped + counted. */
#ifndef CLI_USBCDC_RX_BUFFER_SIZE
#define CLI_USBCDC_RX_BUFFER_SIZE 256
#endif

/* TX ring depth (bytes).  write() returns short when full and the core blocks on
 * CLI_EVT_TX until the usb thread drains it and fires cli_transport_notify_tx(). */
#ifndef CLI_USBCDC_TX_BUFFER_SIZE
#define CLI_USBCDC_TX_BUFFER_SIZE 1024
#endif

/** Backend-private context (the `ctx` of a USB CDC transport). */
struct cli_usbcdc {
	struct cli_instance *sh;            /**< owning instance (cached in init) */
	struct cli_uart_ring rx_ring;
	uint8_t              rx_buf[CLI_USBCDC_RX_BUFFER_SIZE];
	struct cli_uart_ring tx_ring;
	uint8_t              tx_buf[CLI_USBCDC_TX_BUFFER_SIZE];
	volatile uint8_t     enabled;       /**< 1 after enable(); gates _write routing */
	uint8_t              was_connected; /**< usb-thread edge detect for CLI_EVT_CONN */
	uint8_t              conn_synced;   /**< 1 once the connect baseline is established */
	uint32_t             rx_dropped_hw; /**< bytes lost to a full rx_ring */
};

/** The USB CDC transport vtable (init/enable/write/read). */
extern const struct cli_transport_api cli_usbcdc_api;

/**
 * Statically define a USB CDC transport @p _name over TinyUSB device CDC 0.  Bind
 * it to an instance with CLI_INSTANCE_DEFINE(_sh, &_name, "prompt> ").
 */
#define CLI_BACKEND_USBCDC_DEFINE(_name)                                       \
	static struct cli_usbcdc    _name##_ctx;                                \
	static struct cli_transport _name = { &cli_usbcdc_api, NULL, &_name##_ctx }

/**
 * USB-thread bridge: shuttle bytes between this transport's rings and the TinyUSB
 * CDC FIFOs, detect the host connect edge (-> CLI_EVT_CONN for a fresh prompt) and
 * fire cli_transport_notify_rx / _tx.  Call once per usb-thread iteration, AFTER
 * tud_task().  MUST run only on the usb thread (sole owner of tud_cdc_*).
 */
void cli_usbcdc_pump(struct cli_transport *tr);

#ifdef __cplusplus
}
#endif

#endif /* CLI_BACKEND_USBCDC_H */
