/*
 * Wio Lite AI (STM32H725AEI6) -- STANDALONE DFU bootloader, runs from internal
 * flash at 0x08000000 (replaces TinyUF2).  See boot/phase2_config_dump.md.
 *
 * MILESTONE 1 (this file): clock bring-up + LED heartbeat.  Proves the standalone
 * clock tree (SystemClock_Config, reproducing TinyUF2's 550/266/48 MHz) works and
 * the image links/boots from internal flash.  It touches NO option bytes / RDP /
 * DBGMCU / SWD pins, so a bad clock config leaves the board re-flashable over SWD.
 *
 * Next milestones: OCTOSPI2 memory-mapped init (bring up the external flash),
 * USB DFU + CDC (reuse boot/), boot flow (DFU trigger / valid app -> jump to
 * 0x70000000).  Only flashed to 0x08000000 in Phase 3 (brick risk) after an
 * objdump SWD-safety audit.
 */

#include "stm32h7xx_hal.h"

void SystemClock_Config(void);

#define LED_PORT   GPIOC
#define LED_PIN    GPIO_PIN_13

int main(void)
{
  HAL_Init();               // MPU/cache off, SysTick at the reset (HSI) clock
  SystemClock_Config();     // HSE -> PLL1 550 (CPU), PLL2 266 (OCTOSPI2), PLL3 48 (USB)

  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitTypeDef led = {0};
  led.Pin   = LED_PIN;
  led.Mode  = GPIO_MODE_OUTPUT_PP;
  led.Pull  = GPIO_NOPULL;
  led.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_PORT, &led);

  // ~2.5 Hz heartbeat (HAL_Delay is ms via SysTick, reconfigured to 550 MHz by
  // HAL_RCC_ClockConfig).  Distinct cadence from the app-first firmwares.
  for (;;)
  {
    HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
    HAL_Delay(200);
  }
}
