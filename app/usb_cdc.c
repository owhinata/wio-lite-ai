/*
 * USB CDC console for the Wio Lite AI ThreadX shell (Phase 1 skeleton).
 *
 * The USB device stack is pumped by exactly ONE ThreadX thread (usb_thread_entry)
 * -- the sole caller of tud_task() / tud_cdc_* -- so there are no cross-thread
 * TinyUSB races.  Phase 1 just prints a banner + heartbeat here to prove ThreadX,
 * XIP and USB coexist; Phase 2 replaces console_task with the ring-buffered
 * cli_transport backend + a separate shell thread.
 */
#include <stdio.h>
#include "stm32h7xx_hal.h"
#include "tx_api.h"
#include "tusb.h"
#include "app.h"

/* USB1_OTG_HS interrupt -> TinyUSB device stack (rhport 0). */
void OTG_HS_IRQHandler(void) { tud_int_handler(0); }

void usb_hw_init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef usb_pins = {0};
  usb_pins.Pin       = GPIO_PIN_11 | GPIO_PIN_12;   /* PA11 DM / PA12 DP */
  usb_pins.Mode      = GPIO_MODE_AF_PP;
  usb_pins.Pull      = GPIO_NOPULL;
  usb_pins.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  usb_pins.Alternate = GPIO_AF10_OTG1_FS;
  HAL_GPIO_Init(GPIOA, &usb_pins);
  HAL_PWREx_EnableUSBVoltageDetector();
  __HAL_RCC_USB1_OTG_HS_CLK_ENABLE();
  __HAL_RCC_USB1_OTG_HS_ULPI_CLK_DISABLE();   /* FS internal PHY, no ULPI */
}

/* Banner on the connect edge + a ~2 s heartbeat.  Runs only in the usb thread. */
static void console_task(void)
{
  static bool was_connected = false;
  static uint32_t last_ms = 0;
  bool connected = tud_cdc_connected();

  if (connected && !was_connected)
  {
    printf("\r\n=== Wio Lite AI -- ThreadX shell (Phase 1 skeleton) ===\r\n");
    printf("XIP app @0x70000000, CPU %lu MHz.\r\n",
           (unsigned long) (SystemCoreClock / 1000000u));
    printf("heartbeat every 2 s; LED (PC13) blinks ~2 Hz.\r\n");
  }
  was_connected = connected;

  uint32_t now = HAL_GetTick();
  if (connected && (now - last_ms >= 2000u))
  {
    last_ms = now;
    printf("[tick] uptime %lu ms, ThreadX ticks %lu\r\n",
           (unsigned long) now, (unsigned long) tx_time_get());
  }
}

void usb_thread_entry(ULONG arg)
{
  (void) arg;
  for (;;)
  {
    tud_task();          /* process pending USB device events */
    console_task();
    tx_thread_sleep(1);  /* 1 ms poll; USB FS control transfers tolerate this */
  }
}
