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

/* Disable the open UART + its NVIC interrupt. */
void rtl8720_uart_close(void);

#endif /* APP_RTL8720_H */
