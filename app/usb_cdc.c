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
#include "tx_glue.h"
#include "app.h"

/* USB1_OTG_HS interrupt -> TinyUSB device stack (rhport 0).  Bracketed by the EPK
 * ISR hooks (issue #2) so the kit charges this handler's time to the (isr) row of
 * `thread` instead of the interrupted thread; both no-op until profiling is armed. */
void OTG_HS_IRQHandler(void)
{
  tx_glue_isr_enter();
  tud_int_handler(0);
  tx_glue_isr_exit();
}

/* Bring up the OTG_HS pins/clock only (no NVIC, no stack init): called from main()
 * before the kernel starts.  The device stack (tusb_init) is brought up later, in
 * the owning usb thread's entry, so the OTG_HS interrupt is not enabled until the
 * ThreadX objects its ISR path can reach already exist (issue #12). */
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

  /* Bring up the device stack from its owning thread (issue #12): tusb_init()
   * enables OTG_HS_IRQn internally (dwc2 dcd_int_enable -> NVIC_EnableIRQ), so it
   * must run only after cli_init() created the shell's event flags -- which it did,
   * back in tx_application_define, before this thread was ever scheduled.  Set the
   * IRQ priority first (OTG_HS above SysTick(14)/PendSV(15); a PRIMASK critical
   * section masks it anyway), then init. */
  NVIC_SetPriority(OTG_HS_IRQn, 6);
  tusb_rhport_init_t dev_init = { .role  = TUSB_ROLE_DEVICE,
                                  .speed = TUSB_SPEED_AUTO };
  tusb_init(BOARD_TUD_RHPORT, &dev_init);

  /* The scheduler and _tx_execution_initialize() have run by the time any thread
   * entry executes, so it is safe to arm the EPK ISR hooks now (issue #2). */
  tx_glue_profile_enable();

  for (;;)
  {
    tud_task();                /* service the USB device stack */
    cli_usbcdc_pump(tr);       /* bridge CDC FIFOs <-> shell transport rings */
    tx_thread_sleep(1);        /* 1 ms poll; USB FS transfers tolerate this */
  }
}
