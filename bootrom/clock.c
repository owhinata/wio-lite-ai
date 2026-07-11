/*
 * Standalone clock bring-up for the Wio Lite AI DFU bootloader (STM32H725AEI6).
 *
 * The app-first firmware inherited the clock tree from TinyUF2; the standalone
 * bootloader at 0x08000000 is the first code to run after reset, so it configures
 * everything itself.  The parameters here REPRODUCE the exact tree TinyUF2 sets
 * (captured live over CDC, see boot/phase2_config_dump.md), so the app that boots
 * afterwards sees the environment it expects:
 *
 *   HSE 25 MHz crystal
 *   PLL1 : /M2 (12.5) xN44 -> VCO 550 ; P/1 -> SYSCLK 550 (CPU) ; Q/5 110 ; R/2 275
 *   PLL2 : /M25 (1)   xN266-> VCO 266 ; R/1 -> 266 -> OCTOSPI2 kernel (DCR2 /3 ~88.7)
 *   PLL3 : /M5 (5)    xN48 -> VCO 240 ; Q/5 -> 48 MHz -> USB
 *   HPRE /2 (AXI/AHB 275), APBx /2 ; FLASH latency 3 ; VOS0 ; SMPS direct supply.
 *   OCTOSPI kernel <- PLL2R ; USB <- PLL3Q.
 */

#include "stm32h7xx_hal.h"

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef       osc  = {0};
  RCC_ClkInitTypeDef       clk  = {0};
  RCC_PeriphCLKInitTypeDef pclk = {0};

  // Supply: the Wio powers VCORE directly from the SMPS (LDO off).
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  // Voltage scaling 0 for 550 MHz (H72x: no SYSCFG overdrive step needed).
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE0);
  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) { }

  // HSE crystal + PLL1 (system clock).
  osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  osc.HSEState       = RCC_HSE_ON;
  osc.PLL.PLLState   = RCC_PLL_ON;
  osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
  osc.PLL.PLLM       = 2;
  osc.PLL.PLLN       = 44;
  osc.PLL.PLLP       = 1;
  osc.PLL.PLLQ       = 5;
  osc.PLL.PLLR       = 2;
  osc.PLL.PLLRGE     = RCC_PLL1VCIRANGE_3;   // 8-16 MHz input (12.5)
  osc.PLL.PLLVCOSEL  = RCC_PLL1VCOWIDE;      // VCO 550 MHz
  osc.PLL.PLLFRACN   = 0;
  if (HAL_RCC_OscConfig(&osc) != HAL_OK) { while (1) {} }

  // Bus clocks: SYSCLK = PLL1 (550), AHB /2 (275), APBx /2.  Flash latency 3.
  clk.ClockType      = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_D1PCLK1 |
                       RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1;
  clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  clk.SYSCLKDivider  = RCC_SYSCLK_DIV1;
  clk.AHBCLKDivider  = RCC_HCLK_DIV2;
  clk.APB3CLKDivider = RCC_APB3_DIV2;
  clk.APB1CLKDivider = RCC_APB1_DIV2;
  clk.APB2CLKDivider = RCC_APB2_DIV2;
  clk.APB4CLKDivider = RCC_APB4_DIV2;
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_3) != HAL_OK) { while (1) {} }

  // PLL2 -> OCTOSPI2 kernel (PLL2R 266), PLL3 -> USB (PLL3Q 48).
  pclk.PeriphClockSelection = RCC_PERIPHCLK_OSPI | RCC_PERIPHCLK_USB;
  pclk.PLL2.PLL2M = 25; pclk.PLL2.PLL2N = 266; pclk.PLL2.PLL2P = 2;
  pclk.PLL2.PLL2Q = 2;  pclk.PLL2.PLL2R = 1;
  pclk.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_0;    // 1-2 MHz input (1)
  pclk.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;  // VCO 266 MHz
  pclk.PLL2.PLL2FRACN = 0;
  pclk.PLL3.PLL3M = 5;  pclk.PLL3.PLL3N = 48; pclk.PLL3.PLL3P = 2;
  pclk.PLL3.PLL3Q = 5;  pclk.PLL3.PLL3R = 2;
  pclk.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_2;    // 4-8 MHz input (5)
  pclk.PLL3.PLL3VCOSEL = RCC_PLL3VCOMEDIUM;  // VCO 240 MHz
  pclk.PLL3.PLL3FRACN = 0;
  pclk.OspiClockSelection = RCC_OSPICLKSOURCE_PLL2;
  pclk.UsbClockSelection  = RCC_USBCLKSOURCE_PLL3;
  if (HAL_RCCEx_PeriphCLKConfig(&pclk) != HAL_OK) { while (1) {} }
}
