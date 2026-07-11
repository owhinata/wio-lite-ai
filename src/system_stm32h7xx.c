/*
 * Minimal CMSIS system layer for the Wio Lite AI blink (STM32H725AEI6) running
 * XIP from external OCTOSPI2 flash under the TinyUF2 bootloader.
 *
 * WHY THIS REPLACES THE STOCK system_stm32h7xx.c
 * ----------------------------------------------
 * The application executes in place from the external OCTOSPI2 NOR flash mapped
 * at 0x70000000.  That flash's kernel clock is PLL2R (266 MHz), derived from the
 * 25 MHz HSE, and the whole clock tree (HSE -> PLL1 550 MHz CPU / PLL2 -> OCTOSPI2
 * XIP / PLL3 -> 48 MHz USB) plus OCTOSPI2 memory-mapped mode were ALL configured
 * by the bootloader before it jumped here (verified by disassembly + live RCC read).
 *
 * The stock CMSIS SystemInit() RESETS the RCC to HSI and disables HSE/PLL2 --
 * that would stall the very instruction fetch feeding this code and hang the MCU.
 * So SystemInit() here touches NO clock register: it only enables the FPU and
 * points VTOR at our vector table.  We inherit the bootloader's 550 MHz clock.
 */

#include "stm32h7xx.h"

/* Our vector table lives at the start of the XIP flash window. */
#define VECT_TAB_BASE_ADDRESS  0x70000000UL

/* Inherited from the bootloader: SYSCLK = HSE(25 MHz) * 22 = 550 MHz (CPU/FCLK). */
uint32_t SystemCoreClock = 550000000UL;

/* Referenced by stm32h7xx_hal_rcc.c (HAL_RCC_GetHCLKFreq etc.). */
const uint8_t D1CorePrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};

void SystemInit(void)
{
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
  SCB->CPACR |= ((3UL << (10U * 2U)) | (3UL << (11U * 2U)));  /* CP10/CP11 full access */
#endif

  /* Aim exceptions at our vector table in the XIP flash.  The bootloader already
   * set this before jumping, but re-assert it so SysTick/IRQs are unambiguously
   * ours.  Deliberately NO RCC writes (see file header). */
  SCB->VTOR = VECT_TAB_BASE_ADDRESS;
}

/* The CMSIS startup calls this before SystemInit to configure the power supply.
 * The bootloader already selected the board's supply (SMPS), so this is a no-op:
 * we intentionally do not define any USE_PWR_*_SUPPLY macro. */
void ExitRun0Mode(void)
{
}

void SystemCoreClockUpdate(void)
{
  /* We never reprogram the clock tree, so the value is fixed to what the
   * bootloader left us.  (Recomputing from RCC would also work, but this keeps
   * the "hands off the clocks" contract explicit.) */
  SystemCoreClock = 550000000UL;
}
