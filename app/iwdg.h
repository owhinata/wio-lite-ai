/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    iwdg.h
 * @brief   IWDG1 independent watchdog driver + compile-time gate (issue #4).
 */
#ifndef IWDG_H
#define IWDG_H

/* Compile-time gate for the IWDG watchdog (issue #4).  1 == IWDG1 armed + petter
 * thread + `wdt starve`; 0 == compiled out entirely (no IWDG symbols, LSI left
 * untouched, `wdt info` reports "disabled (build)").  The value is injected by the
 * CMake option of the same name via target_compile_definitions(shell ...); this
 * #ifndef default is the fall-back for TUs that do not get it (e.g. host tests). */
#ifndef BSP_ENABLE_IWDG
#define BSP_ENABLE_IWDG 0
#endif

_Static_assert(BSP_ENABLE_IWDG == 0 || BSP_ENABLE_IWDG == 1,
               "BSP_ENABLE_IWDG must be 0 or 1");

/* Petter thread knobs.  Priority 5 sits above usb(8) / cli(16) / bg(17) so the
 * petter preempts every app thread: it keeps feeding through a ~12 s CoreMark run
 * and only stops when the whole system is stuck (scheduler/tick death, IRQ-off
 * lockup, OCTOSPI2 XIP fetch stall).  Feed every 1000 ms (1 tick == 1 ms); the
 * nominal timeout is ~3.0 s (2.04 s at the 47 kHz LSI fast corner), so a 1 s feed
 * keeps margin at every LSI corner.  This assumes normal code never masks IRQs for
 * > ~1 s nor adds a non-blocking CPU hog at priority <= 5. */
#define IWDG_PETTER_PRIORITY    5u
#define IWDG_PETTER_STACK_SIZE  512u   /* register write + tx_thread_sleep only */
#define IWDG_PETTER_PERIOD_MS   1000u

_Static_assert(IWDG_PETTER_PRIORITY <= 31u,
               "IWDG_PETTER_PRIORITY out of ThreadX range (0..31)");
_Static_assert(IWDG_PETTER_STACK_SIZE >= 256u,
               "IWDG_PETTER_STACK_SIZE too small");

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Arm the IWDG1: LSI-clocked, prescaler /64, reload 1499 -- ~3.0 s timeout at the
 * 32 kHz typical LSI (2.04 s at the 47 kHz fast corner, 5.65 s at the 17 kHz slow
 * corner).  HAL_IWDG_Init() performs the first refresh itself and turns on the LSI
 * automatically (no RCC write, so the inherited clock tree is untouched).
 *
 * Call ONCE from tx_application_define(), right after the petter thread is created
 * (issue #12).  HAL_IWDG_Init() polls the PR/RLR update with a HAL_GetTick() timeout,
 * so SysTick must be running: it is, because tx_application_define() now runs with
 * interrupts enabled (no __disable_irq) and SysTick_Handler calls HAL_IncTick()
 * unconditionally.  Compiled to nothing when BSP_ENABLE_IWDG == 0.
 */
void iwdg_init(void);

/**
 * Refresh (pet) the IWDG1 counter -- a single IWDG1->KR write, safe from thread, ISR
 * or fault context.  The priority-5 petter thread calls this every ~1 s; the fault
 * halt loop writes IWDG1->KR directly (no handle) while a debugger is attached.
 */
void iwdg_refresh(void);

/**
 * Non-zero if HAL_IWDG_Init() reported an error in iwdg_init().  The watchdog may
 * still be armed (HAL starts it before polling the prescaler/reload update), so
 * `wdt info` reports "init failed (may be armed)" rather than "disabled".
 */
int iwdg_init_failed(void);

#ifdef __cplusplus
}
#endif

#endif /* IWDG_H */
