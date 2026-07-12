/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    timebase.c
 * @brief   Microsecond busy-wait via the Cortex-M7 DWT cycle counter.
 *
 * Ported to the Wio Lite AI (STM32H725) from the stm32f746g-disco TIM2 timebase:
 * the app drops the ThreadX execution-profile kit and WFI, so DWT (which would
 * otherwise freeze under WFI) is a simpler time base at the CPU clock.  Pure
 * CoreDebug/DWT register access -- no RCC touch (clock-inheritance contract).
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
