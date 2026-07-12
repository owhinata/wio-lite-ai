/*
 * Wio Lite AI (STM32H725AEI6) -- ThreadX shell app entry (Phase 2).
 *
 * Runs XIP from the external OCTOSPI2 flash at 0x70000000, launched by the DFU
 * bootloader.  It INHERITS the bootloader's 550 MHz clock tree + OCTOSPI2
 * memory-mapped mode and must NOT reprogram the RCC: src/system_stm32h7xx.c gives
 * a clock-free SystemInit (FPU + VTOR=0x70000000 only).  HAL_Init() only sets the
 * NVIC priority grouping and SysTick (reload = SystemCoreClock/1000 = 550000 for a
 * 1 ms tick); it does not touch the PLLs.
 *
 * Starts ThreadX with the interactive shell over USB CDC, a USB device pump thread
 * and an LED heartbeat.  The shell instance (cdc_sh) is bound to the CDC transport
 * (cdc_tr); the usb thread bridges the CDC FIFOs to that transport's rings.
 */
#include <stdio.h>
#include "stm32h7xx_hal.h"
#include "tx_api.h"
#include "tusb.h"
#include "tx_glue.h"
#include "cli.h"
#include "cli_instance.h"
#include "cli_backend_usbcdc.h"
#include "timebase.h"   /* timebase_init: DWT cycle counter for udelay (usleep) */
#include "log.h"        /* log_init: reset-persistent RAM log (dmesg / crash record) */
#include "app.h"

/* --- interactive shell over USB CDC ------------------------------------- */
CLI_BACKEND_USBCDC_DEFINE(cdc_tr);
CLI_INSTANCE_DEFINE(cdc_sh, &cdc_tr, "wio> ");

/* --- LED (PC13, red): driven off ---------------------------------------- */
/* The bring-up heartbeat blink was removed on request; the LED is now held off
 * (PC13 low; the bootloader uses PC13 high = on for the DFU indicator). */
#define LED_PORT   GPIOC
#define LED_PIN    GPIO_PIN_13

static void led_init_off(void);

/* Static ThreadX objects + stacks (no byte pool; each thread owns its stack).
 * The shell instance's own thread/stack come from CLI_INSTANCE_DEFINE. */
static TX_THREAD usb_thread;
static UCHAR     usb_stack[4096] __attribute__((aligned(8)));  /* tud_task + printf headroom */

void tx_application_define(void *first_unused_memory)
{
  (void) first_unused_memory;

  /* Shell instance: create its ThreadX objects + backend, then spawn its thread.
   * Fail-soft -- a failed cli_init just skips the shell; the usb/led threads and
   * the rest still run. */
  if (cli_init(&cdc_sh) == 0)
    cli_start(&cdc_sh);
  cli_job_pool_init();          /* background-job worker pool (`cmd &`) */

  /* USB device pump thread (priority 8): the sole owner of tud_task()/tud_cdc_*.
   * Above the shell instance thread (CLI_INSTANCE_PRIORITY=16) so USB stays
   * responsive; arg carries the shell's CDC transport for cli_usbcdc_pump(). */
  tx_thread_create(&usb_thread, "usb", usb_thread_entry, (ULONG)(void *)&cdc_tr,
                   usb_stack, sizeof(usb_stack),
                   8, 8, TX_NO_TIME_SLICE, TX_AUTO_START);

  led_init_off();               /* configure PC13 and hold the red LED off */

  /* Timer lists exist now -> let the SysTick ISR drive the ThreadX scheduler. */
  tx_glue_timer_enable();
}

/* Drive PC13 as a push-pull output at the LED-off level (low).  Register-only
 * (RCC clock enable + GPIO config); no ThreadX API, safe from tx_application_define. */
static void led_init_off(void)
{
  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitTypeDef led = {0};
  led.Pin   = LED_PIN;
  led.Mode  = GPIO_MODE_OUTPUT_PP;
  led.Pull  = GPIO_NOPULL;
  led.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_PORT, &led);
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);   /* PC13 low = LED off */
}

int main(void)
{
  /* Caches on: the app executes XIP from OCTOSPI2 and keeps its working data in
   * AXI-SRAM, both slow uncached.  Enable I-cache (read-only instruction memory ->
   * no coherency concern) then D-cache.  Neither touches the RCC/clock.
   *
   * D-cache is safe here because the app is single-CPU with NO DMA master: USB
   * dwc2 runs slave/FIFO (the CPU copies buffer<->FIFO by MMIO; app/tusb_config.h
   * pins CFG_TUD_DWC2_DMA_ENABLE=0), and there is no camera/SD/eth in v1.  One CPU
   * behind one D-cache is self-coherent across threads/ISRs, so no MPU / clean /
   * invalidate is needed.  The reset-persistent crash log lives in DTCM, which
   * bypasses the D-cache, so it stays coherent and survives a reset.  CMSIS
   * invalidates each cache before enabling; .data/.bss were written to SRAM with
   * the caches off, so there are no stale lines.  A reset (DFU reboot / crash)
   * disables the caches in HW, and the app persists no cacheable-SRAM state across
   * a reset, so the boot handoff is unchanged.
   *
   * FUTURE: when a DMA peripheral is added (PSRAM/OCTOSPI1, DCMI camera, SDMMC,
   * Ethernet, or USB in DMA mode), its DMA buffers MUST be made non-cacheable via
   * the MPU (or clean/invalidate around each transfer) or D-cache will corrupt them. */
  SCB_EnableICache();
  SCB_EnableDCache();

  /* RAM log first: validate the reset-persistent DTCM ring and record the reset
   * cause before anything else can log, so a fault during the rest of bring-up is
   * captured.  log_init() only reads RCC->RSR + HAL_GetTick() (0 pre-HAL_Init) --
   * neither needs HAL_Init().  fault_init() arms the classified faults right after,
   * so the handler always finds a valid ring. */
  log_init();
  fault_init();

  HAL_Init();   /* NVIC grouping + SysTick from SystemCoreClock (550 MHz); no PLL touch */
  timebase_init();   /* DWT cycle counter for usleep's udelay (CoreDebug/DWT only, no RCC) */

  usb_hw_init();
  /* OTG_HS above SysTick(14)/PendSV(15); PRIMASK critical sections mask it anyway. */
  NVIC_SetPriority(OTG_HS_IRQn, 6);
  tusb_rhport_init_t dev_init = { .role  = TUSB_ROLE_DEVICE,
                                  .speed = TUSB_SPEED_AUTO };
  tusb_init(BOARD_TUD_RHPORT, &dev_init);
  setvbuf(stdout, NULL, _IONBF, 0);   /* unbuffered so printf reaches _write */

  /* ThreadX initialization assumes interrupts are locked out; mask them until the
   * scheduler starts (the Cortex-M7 GNU port re-enables interrupts when it
   * schedules the first thread).  SysTick/OTG_HS events simply queue until then. */
  __disable_irq();
  tx_kernel_enter();   /* does not return */
  return 0;
}
