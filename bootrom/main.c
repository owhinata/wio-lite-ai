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
int  octospi2_init(uint8_t jedec[3]);   // bootrom/octospi.c

#define LED_PIN    GPIO_PIN_13   /* PC13 red LED */

int main(void)
{
  HAL_Init();               // MPU/cache off, SysTick at the reset (HSI) clock
  SystemClock_Config();     // HSE -> PLL1 550 (CPU), PLL2 266 (OCTOSPI2), PLL3 48 (USB)

  // Bring up the external OCTOSPI2 flash in memory-mapped mode.
  uint8_t jedec[3] = {0};
  int ospi_ok = octospi2_init(jedec);

  // Read the app vector through the memory-mapped window: a valid image has its MSP
  // in AXI-SRAM (0x24050000).  Confirms mmap reads work end-to-end.
  uint32_t app_msp = *(volatile uint32_t *)0x70000000u;
  int mmap_ok = ((app_msp & 0xFF000000u) == 0x24000000u);

  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitTypeDef led = {0};
  led.Pin   = LED_PIN;
  led.Mode  = GPIO_MODE_OUTPUT_PP;
  led.Pull  = GPIO_NOPULL;
  led.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &led);

  // Milestone-2 verdict on PC13:  1 Hz slow = clock + OCTOSPI2 mmap both OK;
  // ~10 Hz fast strobe = OCTOSPI2 init failed (JEDEC != EF or mmap read wrong).
  uint32_t period = (ospi_ok && mmap_ok) ? 500u : 50u;
  for (;;)
  {
    HAL_GPIO_TogglePin(GPIOC, LED_PIN);
    HAL_Delay(period);
  }
}
