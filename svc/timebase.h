/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    timebase.h
 * @brief   Microsecond busy-wait via the Cortex-M7 DWT cycle counter (svc/ layer).
 *
 * Ported to the Wio Lite AI (STM32H725): udelay() runs on the DWT cycle counter
 * (CPU clock, SystemCoreClock = 550 MHz).  A foreground busy-wait never runs while
 * the core is asleep, so DWT freezing under WFI does not affect it, and it needs no
 * peripheral or RCC touch.  The ThreadX execution-profile kit (issue #2) reads a
 * free-running, WFI-safe TIM2 for its time source instead (port/threadx).
 */
#ifndef TIMEBASE_H
#define TIMEBASE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Enable the DWT cycle counter used by udelay().  Touches only CoreDebug/DWT (no
 * RCC), so it is safe under the app's clock-inheritance contract.  Call once early
 * (e.g. from main() before tx_kernel_enter()).
 */
void timebase_init(void);

/**
 * Busy-wait @p us microseconds on the DWT cycle counter (CPU clock, set up by
 * timebase_init()).  Does NOT yield -- short delays only; the `usleep` command
 * caps it (issue #21).
 */
void udelay(uint32_t us);

#ifdef __cplusplus
}
#endif

#endif /* TIMEBASE_H */
