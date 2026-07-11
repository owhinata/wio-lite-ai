/*
 * Wio Lite AI (STM32H725AEI6) -- LED blink, XIP from external OCTOSPI2 flash.
 *
 * Board wiring (from _ref/...Schematic.pdf, sheet 8 "LED & Button"):
 *   LED2 (red,   "LED0") <- PC13 -> NPN digital transistor (DDC114VU) -> LED to 3V3.
 *                           PC13 is a low-drive backup-domain pin, hence the buffer.
 *   LED3 (yellow,"LED1") <- PF0  (same buffered arrangement)
 *
 * We toggle PC13, so the red LED blinks regardless of the buffer's polarity.
 *
 * Clocking: we run entirely on the clock tree the TinyUF2 bootloader left us
 * (CPU 550 MHz).  We do NOT call HAL_Init()/SystemClock_Config() -- reprogramming
 * the clocks would kill the OCTOSPI2 XIP fetch (see src/system_stm32h7xx.c).  The
 * delay is a self-contained DWT cycle-counter busy-wait (CPU-clock based), so it
 * needs no SysTick interrupt and no tick configuration.
 */

#include "stm32h7xx_hal.h"

#define LED_PORT   GPIOC
#define LED_PIN    GPIO_PIN_13
#define LED_CLK_EN __HAL_RCC_GPIOC_CLK_ENABLE

/* --- DWT cycle-counter delay (CPU clock = SystemCoreClock = 550 MHz) --------- */
static void dwt_delay_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;   /* enable trace/DWT */
  DWT->CYCCNT = 0U;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;             /* start the cycle counter */
}

static void delay_ms(uint32_t ms)
{
  const uint32_t cycles = ms * (SystemCoreClock / 1000U);
  const uint32_t start  = DWT->CYCCNT;
  while ((DWT->CYCCNT - start) < cycles)
  {
    /* busy wait */
  }
}

int main(void)
{
  LED_CLK_EN();

  GPIO_InitTypeDef led = {0};
  led.Pin   = LED_PIN;
  led.Mode  = GPIO_MODE_OUTPUT_PP;
  led.Pull  = GPIO_NOPULL;
  led.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_PORT, &led);

  dwt_delay_init();

  for (;;)
  {
    HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
    delay_ms(500U);   /* ~1 Hz blink */
  }
}
