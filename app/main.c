/*
 * Wio Lite AI (STM32H725AEI6) -- ThreadX shell app entry (Phase 1 skeleton).
 *
 * Runs XIP from the external OCTOSPI2 flash at 0x70000000, launched by the DFU
 * bootloader.  It INHERITS the bootloader's 550 MHz clock tree + OCTOSPI2
 * memory-mapped mode and must NOT reprogram the RCC: src/system_stm32h7xx.c gives
 * a clock-free SystemInit (FPU + VTOR=0x70000000 only).  HAL_Init() only sets the
 * NVIC priority grouping and SysTick (reload = SystemCoreClock/1000 = 550000 for a
 * 1 ms tick); it does not touch the PLLs.
 *
 * Phase 1 starts ThreadX with two threads -- a USB CDC console (banner + tick) and
 * an LED heartbeat -- to prove clock-inheritance / SysTick / XIP / USB coexist.
 * No shell yet (Phase 2).
 */
#include <stdio.h>
#include "stm32h7xx_hal.h"
#include "tx_api.h"
#include "tusb.h"
#include "tx_glue.h"
#include "app.h"

/* --- LED (PC13, red) ---------------------------------------------------- */
#define LED_PORT   GPIOC
#define LED_PIN    GPIO_PIN_13

static void led_thread_entry(ULONG arg);

/* Static ThreadX objects + stacks (no byte pool; each thread owns its stack). */
static TX_THREAD usb_thread;
static UCHAR     usb_stack[4096] __attribute__((aligned(8)));  /* printf + tud_task headroom */
static TX_THREAD led_thread;
static UCHAR     led_stack[512]  __attribute__((aligned(8)));

void tx_application_define(void *first_unused_memory)
{
  (void) first_unused_memory;

  /* usb console at priority 8, led heartbeat at 10 (lower = higher priority). */
  tx_thread_create(&usb_thread, "usb", usb_thread_entry, 0,
                   usb_stack, sizeof(usb_stack),
                   8, 8, TX_NO_TIME_SLICE, TX_AUTO_START);

  tx_thread_create(&led_thread, "led", led_thread_entry, 0,
                   led_stack, sizeof(led_stack),
                   10, 10, TX_NO_TIME_SLICE, TX_AUTO_START);

  /* Timer lists exist now -> let the SysTick ISR drive the ThreadX scheduler. */
  tx_glue_timer_enable();
}

static void led_thread_entry(ULONG arg)
{
  (void) arg;
  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitTypeDef led = {0};
  led.Pin   = LED_PIN;
  led.Mode  = GPIO_MODE_OUTPUT_PP;
  led.Pull  = GPIO_NOPULL;
  led.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_PORT, &led);

  for (;;)
  {
    HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
    tx_thread_sleep(125);   /* 125 ms at 1 kHz tick -> ~4 Hz toggle = ~2 Hz blink */
  }
}

int main(void)
{
  HAL_Init();   /* NVIC grouping + SysTick from SystemCoreClock (550 MHz); no PLL touch */

  usb_hw_init();
  /* OTG_HS above SysTick(14)/PendSV(15); PRIMASK critical sections mask it anyway. */
  NVIC_SetPriority(OTG_HS_IRQn, 6);
  tusb_rhport_init_t dev_init = { .role  = TUSB_ROLE_DEVICE,
                                  .speed = TUSB_SPEED_AUTO };
  tusb_init(BOARD_TUD_RHPORT, &dev_init);
  setvbuf(stdout, NULL, _IONBF, 0);   /* unbuffered so printf reaches _write */

  tx_kernel_enter();   /* does not return */
  return 0;
}
