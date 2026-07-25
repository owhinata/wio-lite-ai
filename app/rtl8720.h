/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/*
 * Wio Lite AI (STM32H725AEI6) -- onboard RTL8720DN WiFi/BLE companion driver
 * (issue #17: factory-firmware identification).
 *
 * The RTL8720DN is reached from the STM32 host over (schematic sheets 2/5/8):
 *   - CHIP_EN  : PC3                       (module power/enable; board has NO pull,
 *                                           so the host must drive it -- Low=off,
 *                                           Low->High=restart)
 *   - LOG UART : UART9  PD14(RX)/PD15(TX)  (module UART_LOG, boot log, default 115200)
 *   - AT  UART : USART1 PA10(RX)/PB14(TX)  (module HS/BLE UART, AT default 38400)
 *
 * This is a minimal, safe investigation driver: it powers the module and bridges
 * one of its UARTs to the USB CDC shell so the boot banner identifies the factory
 * firmware (eRPC / AT / raw Realtek).  It touches only GPIO + UART9/USART1 +
 * peripheral clock gates -- never the RCC clock tree (baud is derived from the
 * inherited PCLK2 = 137.5 MHz) -- so it is XIP-safe.
 */
#ifndef APP_RTL8720_H
#define APP_RTL8720_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Which RTL8720DN host UART the sniffer bridges to. */
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
 * users: the eRPC service thread (app/erpc.c), the `wifi log`/`probe` console bridge
 * and the issue-#19 flash downloader.  Because open() closes the current UART and
 * resets the shared SPSC ring, it must only be called from a path that
 *   (a) holds the coarse link mutex (app/rtl_link.h) for the whole session, AND
 *   (b) either goes through rtl_link_uart_ref() -- which brackets the call with
 *       erpc_link_lock() so it cannot race the service thread -- or has established
 *       that no eRPC session is live (rtl_link_uart_busy() == false, which is what
 *       rtl_link_hw_claim() checks; with no live token the service thread is parked
 *       and touches neither the UART nor the ring).
 * The flash downloader (app/rtl8720_flash.c) opens/closes UART9 at several baud rates
 * internally and relies on (b)'s second form.
 */
int rtl8720_uart_open(enum rtl8720_uart which, uint32_t baud);

/* Drain up to @p n received bytes from the ring into @p buf; returns the count
 * (0 if empty).  Non-blocking; foreground/thread context. */
size_t rtl8720_uart_read(uint8_t *buf, size_t n);

/* Send @p n bytes out of the open UART (bounded poll on TXFNF; never hangs). */
void rtl8720_uart_write(const uint8_t *buf, size_t n);

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
	uint32_t isr_count;       /* RX interrupts taken since open */
	uint32_t isr_max_bytes;   /* most bytes pulled from RDR in one interrupt */
	uint32_t isr_grace;       /* bytes the FIFO can still take after the threshold */
	uint32_t isr_max_cycles;  /* longest time spent inside the ISR (DWT cycles) */
	uint32_t drops;           /* == rtl8720_uart_overflows(): our ring overflowed */
	uint32_t ore;             /* USART overrun: the hardware FIFO overflowed */
	uint32_t ferr;            /* framing / noise errors (baud mismatch indicator) */
	uint32_t ring_size;       /* RX ring capacity in bytes */
};
void rtl8720_uart_stats(struct rtl8720_uart_stats *out);

/* Disable the open UART + its NVIC interrupt. */
void rtl8720_uart_close(void);

#endif /* APP_RTL8720_H */
