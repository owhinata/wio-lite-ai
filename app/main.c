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
#include "tx_glue.h"
#include "cli.h"
#include "cli_instance.h"
#include "cli_backend_usbcdc.h"
#include "timebase.h"   /* timebase_init: DWT cycle counter for udelay (usleep) */
#include "log.h"        /* log_init: reset-persistent RAM log (dmesg / crash record) */
#include "iwdg.h"       /* IWDG petter (issue #4): armed from its own thread entry */
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

#if BSP_ENABLE_IWDG
/* IWDG petter (issue #4).  Static objects; the thread only refreshes -- the watchdog
 * is armed by iwdg_init() in tx_application_define (issue #12), after this thread is
 * created.  H7 HAL_IWDG_Init() polls the PR/RLR update with a HAL_GetTick() timeout,
 * so SysTick must be live: it is, because tx_application_define now runs with
 * interrupts enabled (no __disable_irq) and SysTick_Handler calls HAL_IncTick()
 * unconditionally.  Priority 5 preempts usb(8)/cli(16)/bg(17), so the petter keeps
 * feeding through a ~12 s CoreMark run and only stops on a whole-system stall
 * (scheduler/tick death, IRQ-off lockup, OCTOSPI2 XIP fetch stall) -> the IWDG then
 * resets the board. */
static TX_THREAD iwdg_thread;
static UCHAR     iwdg_stack[IWDG_PETTER_STACK_SIZE] __attribute__((aligned(8)));

static void iwdg_entry(ULONG arg)
{
  (void) arg;
  iwdg_refresh();   /* pet before the first sleep: minimise the arm->pet window */
  for (;;) {
    tx_thread_sleep(IWDG_PETTER_PERIOD_MS);
    iwdg_refresh();
  }
}
#endif

void tx_application_define(void *first_unused_memory)
{
  (void) first_unused_memory;

  /* Make the newlib heap thread-safe before any thread runs: membench/coremark/bg
   * jobs and per-thread printf(%f) all allocate, and the stock malloc lock is a
   * no-op (see app/malloc_lock.c).  Created here (single-threaded, pre-scheduler) so
   * the mutex exists before the first concurrent malloc. */
  malloc_lock_init();

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

#if BSP_ENABLE_IWDG
  /* IWDG petter thread (priority 5), then arm the watchdog (issue #12: arm here, not
   * in the entry, matching f746).  HAL_IWDG_Init()'s HAL_GetTick() timeout works
   * because interrupts are enabled and SysTick is ticking (HAL_IncTick) throughout
   * tx_application_define.  Fail-soft: arm ONLY if the petter was created -- an armed
   * watchdog with nothing to refresh it would just reset the board. */
  UINT iwdg_rc = tx_thread_create(&iwdg_thread, "iwdg", iwdg_entry, 0,
                                  iwdg_stack, sizeof(iwdg_stack),
                                  IWDG_PETTER_PRIORITY, IWDG_PETTER_PRIORITY,
                                  TX_NO_TIME_SLICE, TX_AUTO_START);
  if (iwdg_rc == TX_SUCCESS)
    iwdg_init();                /* arm now; the petter (above) will refresh it */
#endif

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

  usb_hw_init();   /* OTG_HS pins/clock only; the device stack (tusb_init, which
                    * enables OTG_HS_IRQn) comes up later in the usb thread entry so
                    * no interrupt is armed before its ThreadX objects exist (#12). */
  setvbuf(stdout, NULL, _IONBF, 0);   /* unbuffered so printf reaches _write */

  /* No __disable_irq() here (issue #12): interrupts stay enabled through ThreadX
   * init, matching f746.  This is safe because every interrupt source is gated until
   * its ThreadX objects exist -- SysTick only calls HAL_IncTick() until
   * tx_glue_timer_enable() opens the tx_timer_active gate, and OTG_HS_IRQn stays
   * disabled at the NVIC until the usb thread calls tusb_init().  Threads created in
   * tx_application_define() are merely READY; the scheduler does not run them until
   * _tx_thread_schedule() after define returns. */
  tx_kernel_enter();   /* does not return */
  return 0;
}
