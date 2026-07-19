/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    timebase.c
 * @brief   Microsecond busy-wait via the Cortex-M7 DWT cycle counter.
 *
 * Re-based on the Cortex-M7 DWT cycle counter rather than the stm32f746g-disco
 * TIM2 timebase: udelay() is a foreground busy-wait that never runs while the core
 * is asleep, so DWT freezing under WFI does not affect it and it needs no
 * peripheral.  The ThreadX execution-profile kit (issue #2) instead reads a
 * free-running, WFI-safe TIM2 as its time source (see port/threadx/tx_user.h and
 * tx_glue.c).  Pure CoreDebug/DWT register access -- no RCC touch (clock-inheritance
 * contract).
 */
#include "timebase.h"

#include "stm32h7xx_hal.h"

void timebase_init(void)
{
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;   /* enable trace/DWT */
	DWT->CYCCNT = 0u;
	DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;             /* start the cycle counter */
}

/**
 * @brief  Busy-wait @p us microseconds on the DWT cycle counter.
 *
 * DWT->CYCCNT advances at the CPU clock (SystemCoreClock = 550 MHz), so
 * SystemCoreClock/1000000 == 550 cycles per microsecond.  Unsigned 32-bit
 * subtraction makes the wait wrap-safe as long as the delay is shorter than the
 * ~7.8 s CYCCNT wrap at 550 MHz; the `usleep` command caps @p us far below that.
 * Pure busy loop -- it does NOT yield.  Interrupts (SysTick, OTG_HS) still run, so
 * the ThreadX tick and higher-priority threads are unaffected.
 */
void udelay(uint32_t us)
{
	uint32_t start  = DWT->CYCCNT;
	uint32_t cycles = us * (SystemCoreClock / 1000000u);   /* 550 cycles per us */

	while ((DWT->CYCCNT - start) < cycles)
		;
}
