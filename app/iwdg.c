/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    iwdg.c
 * @brief   IWDG1 independent watchdog driver (issue #4).
 *
 * The IWDG1 is clocked by the LSI (~32 kHz typ, 17-47 kHz over tolerance),
 * independent of the inherited HSE+PLL 550 MHz tree, so it keeps counting even if
 * the main clock, the OCTOSPI2 XIP instruction fetch or the ThreadX scheduler
 * stalls.  Prescaler /64 + reload 1499 gives T = 64 * (1499 + 1) / f_LSI: ~3.0 s at
 * the typical LSI and 2.04 s at the 47 kHz fast corner.  The priority-5 petter
 * thread (app/main.c) refreshes every ~1 s (<= T/2 at every LSI corner), so it
 * survives CoreMark (~12 s in the priority-16 shell thread) without any extra pet --
 * the petter preempts it.
 *
 * iwdg_init() is called from the petter thread entry, AFTER the scheduler starts, so
 * HAL_IWDG_Init()'s HAL_GetTick()-based PR/RLR poll runs with SysTick live.
 *
 * The whole file compiles to nothing when BSP_ENABLE_IWDG == 0, so no IWDG symbol
 * (and no LSI dependency) reaches the image.
 *
 * Clean-room design; no third-party code reused.
 */
#include "iwdg.h"

#if BSP_ENABLE_IWDG

#include "stm32h7xx_hal.h"

#define LOG_TAG "iwdg"
#include "log.h"

static IWDG_HandleTypeDef hiwdg;
static uint8_t            g_iwdg_init_failed;

void iwdg_init(void)
{
	hiwdg.Instance       = IWDG1;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
	hiwdg.Init.Reload    = 1499u;             /* ~3.0 s @32 kHz, 2.04 s @47 kHz */
	hiwdg.Init.Window    = IWDG_WINDOW_DISABLE;

	/* HAL_IWDG_Init() starts IWDG1 (KR=0xCCCC; the LSI turns on automatically) and
	   then polls the PR/RLR update (SR PVU/RVU) with a HAL_GetTick() timeout; a
	   failure here does NOT prove the watchdog is unarmed, so flag it for `wdt info`
	   ("init failed (may be armed)") and keep going.  Never halt: the petter still
	   refreshes, and an unrefreshed watchdog would only reset us. */
	if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
		g_iwdg_init_failed = 1u;
		LOG_ERR("HAL_IWDG_Init failed (may be armed)");
	}
}

void iwdg_refresh(void)
{
	HAL_IWDG_Refresh(&hiwdg);   /* == IWDG1->KR = 0xAAAA; a single register write */
}

int iwdg_init_failed(void)
{
	return (int)g_iwdg_init_failed;
}

#endif /* BSP_ENABLE_IWDG */
