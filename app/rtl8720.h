/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/*
 * Wio Lite AI (STM32H725AEI6) -- onboard RTL8720DN WiFi/BLE companion driver
 * (issue #17 originally; the host's whole side of the module now sits on top of it).
 *
 * The RTL8720DN is reached from the STM32 host over (schematic sheets 2/5/8):
 *   - CHIP_EN  : PC3                       (module power/enable; board has NO pull,
 *                                           so the host must drive it -- Low=off,
 *                                           Low->High=restart)
 *   - LOG UART : UART9  PD14(RX)/PD15(TX)  (module UART_LOG, boot log, default 115200)
 *   - AT  UART : USART1 PA10(RX)/PB14(TX)  (module HS/BLE UART, AT default 38400)
 *
 * This is the bottom layer: power (CHIP_EN) plus ONE open host UART at a time, with an
 * interrupt-driven RX ring and a TX ring, and a notify hook so a waiter wakes on arrival
 * instead of polling.  It started out (issue #17) as an investigation aid that only
 * bridged a module UART to the USB CDC console for the boot banner -- that bridge is
 * still here as `wifi log` -- but everything above it now rides the same driver: the
 * eRPC service thread (app/erpc.c) and its CTRL/DATA channels, and the issue-#19 flash
 * downloader.  Which of them owns the UART is arbitrated one layer up, in app/rtl_link.h.
 *
 * It touches only GPIO + UART9/USART1 + peripheral clock gates -- never the RCC clock
 * tree (baud is derived from the inherited PCLK2 = 137.5 MHz) -- so it is clock-safe.
 */
#ifndef APP_RTL8720_H
#define APP_RTL8720_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Which of the two RTL8720DN host UARTs is opened. */
enum rtl8720_uart {
	RTL8720_UART_LOG = 0,   /* module LOG UART  -> STM32 UART9  (PD14 RX / PD15 TX) */
	RTL8720_UART_AT  = 1,   /* module HS/AT UART -> STM32 USART1 (PA10 RX / PB14 TX) */
};

/*
 * Configure PC3 (CHIP_EN) as a push-pull output driven LOW so the RTL8720DN is held
 * in power-off and never floats after reset.  MUST be called once from the app
 * start-up path (app/main.c) -- NOT lazily from a command -- so the module state is
 * deterministic before any `wifi` command runs.  Register-only (RCC gate + GPIO);
 * safe to call from tx_application_define() before the scheduler starts.
 */
void rtl8720_init(void);

/* Drive CHIP_EN: on = High (enable/power on), off = Low (power off). */
void rtl8720_power(bool on);

/* 1 if CHIP_EN is currently driven High (module enabled). */
bool rtl8720_powered(void);

/* Power-cycle the module: CHIP_EN Low (>=80 ms) then High.  Reboots the RTL8720DN. */
void rtl8720_reset(void);

/*
 * Bring up one of the two host UARTs (GPIO AF + clock gate + UART + RX ring + RX
 * interrupt) at @p baud and start capturing received bytes into the ring.  Only one
 * UART is active at a time; opening closes any previously-open one and clears the
 * ring.  Returns 0 on success, -1 if the UART did not come ready.
 *
 * OWNERSHIP (issue #21 increment 8) -- open/close and the ring have several would-be
 * users: the eRPC service thread (app/erpc.c), the `wifi log` console bridge
 * and the issue-#19 flash downloader.  Because open() closes the current UART and
 * resets the shared SPSC ring, it must only be called from a path that
 *   (a) holds the coarse link mutex (app/rtl_link.h) for the whole session, AND
 *   (b) either goes through rtl_link_uart_ref() -- which brackets the call with
 *       erpc_link_lock() so it cannot race the service thread -- or has established
 *       that no eRPC session is live (rtl_link_uart_busy() == false, which is what
 *       rtl_link_hw_claim() checks; with no live token the service thread is parked
 *       and touches neither the UART nor the ring).
 * The flash downloader (app/rtl8720_flash.c) opens/closes UART9 at several baud rates
 * internally and relies on (b)'s second form.  Since issue #30 B2b it no longer INHERITS
 * that condition from its caller -- the L2 bridge is permanent, so the interface owner
 * holds a reference whenever the host stack is up and the caller claims with allow_busy.
 * rtl_dl_enter() therefore ESTABLISHES it, by calling rtl_link_force_quiesce() before it
 * touches the peripheral: that revokes the reference under the eRPC lock and leaves
 * rtl_link_uart_busy() == false for the rest of the session.
 */
int rtl8720_uart_open(enum rtl8720_uart which, uint32_t baud);

/* Drain up to @p n received bytes from the ring into @p buf; returns the count
 * (0 if empty).  Non-blocking; foreground/thread context. */
size_t rtl8720_uart_read(uint8_t *buf, size_t n);

/*
 * Queue @p n bytes for transmission on the open UART (issue #23 U1).
 *
 * INTERRUPT-DRIVEN since U1: the bytes are copied into a TX ring and the TXFIFO
 * threshold interrupt drains it, so this RETURNS BEFORE THE BYTES ARE ON THE WIRE.
 * Before U1 it was a polling spin, which at 6 Mbaud held the caller for 2.5 ms per
 * 1500-byte frame -- with the link service thread at priority 10 that starves the
 * shell and the telnet service, and burns ~60 % of the CPU spinning once Ethernet
 * frames flow continuously.
 *
 * Two consequences for callers:
 *   - Anything that must observe the bytes having LEFT (closing the UART, changing
 *     the baud rate, handing the pins to another user) must call
 *     rtl8720_uart_flush() first.  rtl8720_uart_close() does this itself, and every
 *     baud change in the tree goes through a close, so this is normally automatic.
 *   - Bytes are NEVER dropped: with a full ring the call waits (yielding, bounded) for
 *     the interrupt to make room.  Dropping mid-frame would desynchronise the link
 *     framing at the far end, which is far worse than the wait.
 *
 * SINGLE WRITER.  Callers must be serialised by the link ownership rules (app/erpc.c's
 * mutex for the eRPC path, app/rtl_link.c's coarse mutex + console claim for the
 * bridge / flash paths) -- the ring is strictly single-producer.  That was already
 * true when this was a spin loop; it is now load-bearing.
 */
void rtl8720_uart_write(const uint8_t *buf, size_t n);

/*
 * Wait until everything handed to rtl8720_uart_write() has physically left the UART:
 * TX ring empty, then USART_ISR TXFE (TXFIFO empty) AND TC (the last frame has left
 * the shift register).  RM0468 sec 53.8.9: TC is set when transmission of the last
 * data is complete AND TXFE is set -- TXFE alone would cut the final character.
 *
 * Returns 0 when the line is idle, -1 on timeout (or with no UART open, which is
 * vacuously flushed).  Bounded on purpose: a wedged peripheral must not hang the shell.
 */
int rtl8720_uart_flush(uint32_t timeout_ms);

/*
 * Bytes rtl8720_uart_write() can take right now without waiting.
 *
 * For a caller that must not block: the link service thread holds its own mutex across
 * the send step, so a wait there would stall every other thread that needs the link --
 * including the `wifi reset` recovery path.  It checks this first and simply leaves a
 * bulk frame queued for the next pass instead (app/erpc.c erpc_data_send_one).
 */
uint32_t rtl8720_uart_tx_space(void);

/* RX bytes dropped by ring overflow since open (diagnostic). */
uint32_t rtl8720_uart_overflows(void);

/*
 * RX interrupt-latency diagnostics (issue #23 U0-1), reset on every open.
 *
 * Since the RXFIFO is drained on a threshold interrupt, the interesting number is how
 * much of the FIFO's remaining grace one interrupt actually consumed: @isr_max_bytes
 * is the most bytes any single interrupt pulled out of USART_RDR (dropped ones
 * included -- it is FIFO occupancy that measures the grace), and @isr_grace is how
 * many it could have taken before an overrun (18 - threshold; see app/rtl8720.c).  A
 * high-water approaching the grace means the threshold is too high or something is
 * masking interrupts for too long.
 *
 * THREE different losses, do not conflate them:
 *   @ore   the USART's own RXFIFO overran -- the byte never reached software.  This is
 *          the failure the threshold scheme exists to prevent; it must stay 0.
 *   @drops OUR ring was full when the ISR had a byte to store, i.e. the consumer fell
 *          behind rather than the ISR.  Bigger ring / faster consumer.
 *   @ferr  framing / noise errors -- a wrong or marginal baud rate on either end.
 *
 * @isr_max_cycles is DWT cycles at SystemCoreClock (550 MHz).  It is NOT a grace
 * budget -- the drain loop keeps consuming bytes that arrive while it runs -- it is
 * the per-byte drain cost, which must stay well under one byte time on the wire
 * (see condition (b) in app/rtl8720.c).
 */
struct rtl8720_uart_stats {
	uint32_t isr_count;       /* UART interrupts taken since open (RX and/or TX) */
	uint32_t isr_max_bytes;   /* most bytes pulled from RDR in one interrupt */
	uint32_t isr_grace;       /* bytes the FIFO can still take after the threshold */
	uint32_t isr_max_cycles;  /* longest time spent inside the ISR (DWT cycles) */
	uint32_t drops;           /* == rtl8720_uart_overflows(): our ring overflowed */
	uint32_t ore;             /* USART overrun: the hardware FIFO overflowed */
	uint32_t ferr;            /* framing / noise errors (baud mismatch indicator) */
	uint32_t ring_size;       /* RX ring capacity in bytes */

	/* TX side (issue #23 U1).  Kept SEPARATE from the RX numbers on purpose: the two
	 * share one interrupt, and time spent refilling the TXFIFO is time subtracted from
	 * @isr_grace.  If @isr_max_bytes ever climbs towards the grace, @tx_max_bytes says
	 * whether the transmitter is the reason. */
	uint32_t tx_isr_count;    /* interrupts that pushed at least one byte into TDR */
	uint32_t tx_max_bytes;    /* most bytes pushed into TDR in one interrupt */
	uint32_t tx_bytes;        /* bytes handed to the hardware since open */
	uint32_t tx_waits;        /* rtl8720_uart_write() calls that had to wait for room */
	uint32_t tx_ring_size;    /* TX ring capacity in bytes */
};
void rtl8720_uart_stats(struct rtl8720_uart_stats *out);

/*
 * Install a callback the RX ISR runs once per interrupt that stored bytes, right after
 * it publishes the ring head (issue #23 U0-3).  @cb == NULL disarms it.
 *
 * It exists so a reader that would otherwise poll can be woken as soon as bytes land:
 * the eRPC service thread polls every 1 ms, which for a 1500-byte frame at 6 Mbaud
 * (2.5 ms on the wire) adds up to 1 ms per frame -- enough to hide the difference
 * between 4 and 6 Mbaud in the U0-3 link measurements.  The poll is KEPT as the
 * backstop, so a missed notification only costs latency, never correctness.
 *
 * RULES.  @cb runs in interrupt context at the UART's NVIC priority (5) and must be
 * short and non-blocking; ThreadX signalling (tx_event_flags_set) is fine here because
 * this port uses PRIMASK critical sections.  Both open and close clear the callback, so
 * it can never outlive the session that installed it -- re-arm after every open.
 */
void rtl8720_uart_set_rx_notify(void (*cb)(void));

/* Disable the open UART + its NVIC interrupt. */
void rtl8720_uart_close(void);

#endif /* APP_RTL8720_H */
