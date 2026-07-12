/*
 * USB CDC device bring-up + pump thread for the Wio Lite AI ThreadX shell.
 *
 * The usb thread is the sole caller of tud_task() / the tud_cdc_* API: each
 * iteration it services the device stack (tud_task) then bridges the shell's USB
 * CDC transport rings to the CDC FIFOs (cli_usbcdc_pump).  No other thread touches
 * TinyUSB, so there are no cross-thread races.
 */
#include "stm32h7xx_hal.h"
#include "tx_api.h"
#include "tusb.h"
#include "cli_backend_usbcdc.h"
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

/* arg = the shell's struct cli_transport * (the CDC backend to bridge). */
void usb_thread_entry(ULONG arg)
{
  struct cli_transport *tr = (struct cli_transport *) arg;
  for (;;)
  {
    tud_task();                /* service the USB device stack */
    cli_usbcdc_pump(tr);       /* bridge CDC FIFOs <-> shell transport rings */
    tx_thread_sleep(1);        /* 1 ms poll; USB FS transfers tolerate this */
  }
}
