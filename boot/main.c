/*
 * Wio Lite AI (STM32H725AEI6) -- standalone USB DFU bootloader.
 *
 * Runs from internal flash sector 0 (0x08000000; it replaced the stock TinyUF2).
 * On reset it brings up its own clock tree (SystemClock_Config), then decides:
 *
 *   - Enter DFU mode if the USER button (PF1, active-low) is held, OR there is
 *     no valid app in the internal flash.  It enumerates as a composite DFU +
 *     CDC device; a download (dfu-util -a 0 -D app.bin) is programmed into the
 *     app partition, sectors 1-3 at 0x08020000, via iflash.c.  After the
 *     download manifests, the board reboots into the new app.  The red LED
 *     (PC13) is held on while in DFU mode.
 *   - Otherwise jump to the app at 0x08020000 (set VTOR + MSP, branch to its
 *     reset vector) -- with SysTick stopped first so a stray tick cannot
 *     vector into the app.
 *
 * SINCE ISSUE #25 the app executes from the internal flash rather than XIP from
 * the external OCTOSPI2 window (905 vs 54 MB/s measured), so this bootloader no
 * longer touches OCTOSPI2 at all: no memory-mapped bring-up, no external
 * programming path, no external app to jump to.  The external flash is simply
 * not mapped while the app runs; bringing it back up, if it is ever wanted for
 * storage, belongs to the app now (issue #10).  The clock tree still programs
 * PLL2R, which is that flash's kernel clock, because clock.c is deliberately
 * left untouched.
 *
 * DFU mode is the safe fallback: an erased / invalid app always lands here, so
 * the board can always be re-loaded over DFU.  Reading a never-programmed app
 * vector is safe -- erased flash reads back as all-ones with no ECC error
 * (RM0468 sec 4.3.10).  It touches NO option bytes / RDP / DBGMCU / SWD pins,
 * so a bad config leaves the board re-flashable over SWD.
 *
 * Caches stay OFF the whole time (reset default), which keeps every read of the
 * flash the DFU path just programmed coherent with no maintenance.
 */

#include <stdio.h>
#include "stm32h7xx_hal.h"
#include "tusb.h"
#include "iflash.h"

void SystemClock_Config(void);   /* boot/clock.c */

/* --- printf over USB CDC (newlib retargeting) --------------------------- */
extern char end;                 /* heap start (from the linker script) */
void *_sbrk(int incr)
{
  static char *heap = 0;
  if (heap == 0) heap = &end;
  char *prev = heap;
  heap += incr;
  return prev;
}

/*
 * newlib calls this for stdout/stderr; route it to the CDC TX FIFO.  When the
 * FIFO fills, pump tud_task() so bursts are not dropped; a guard bounds the
 * wait if the host is not reading.  Only ever called from the main loop, so
 * re-entering tud_task() here is safe.
 */
int _write(int fd, char *buf, int len)
{
  (void) fd;
  if (!tud_cdc_connected()) return len;
  int sent = 0;
  uint32_t guard = 0;
  while (sent < len && guard < 100000u)
  {
    uint32_t w = tud_cdc_write((uint8_t const *) buf + sent,
                               (uint32_t) (len - sent));
    sent += (int) w;
    tud_cdc_write_flush();
    if (w == 0u) { tud_task(); guard++; }
  }
  return len;
}

/* --- LED (PC13, red): held on while in DFU mode ------------------------- */
#define LED_PORT   GPIOC
#define LED_PIN    GPIO_PIN_13

/* --- internal-flash status (published: DFU alt-0 name + CDC banner) ------ */
static volatile int g_iflash_ok;     /* device flash size matches the map */
static volatile int g_app_present;   /* app vector table looks like an app */

char g_dfu_alt0_str[48] = "Wio Lite AI app @0x08020000";  /* dfu-util -l */

/*
 * Deferred reboot: the DFU manifest callback requests one so the freshly
 * downloaded app boots.  The delay must clear dfu-util's manifest handling: on
 * seeing dfuMANIFEST it sleeps ~1000 ms, then re-reads GET_STATUS (by then we
 * are in dfuMANIFEST-WAIT-RESET, which it accepts cleanly).  Rebooting sooner
 * makes that re-read hit a vanished device -> LIBUSB_ERROR_NO_DEVICE.  So we
 * wait comfortably past 1 s, then reset from the main loop (not the callback,
 * so the status responses flush first).
 */
static volatile uint32_t g_reboot_at_ms;   /* 0 = none pending */

void boot_request_reboot(void)           /* called from dfu_callbacks.c */
{
  g_reboot_at_ms = HAL_GetTick() + 1500u;
}

/* --- interrupt handlers ------------------------------------------------- */
void SysTick_Handler(void)   { HAL_IncTick(); }   /* HAL tick */
void OTG_HS_IRQHandler(void) { tud_int_handler(0); }

static void usb_hw_init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef usb_pins = {0};
  usb_pins.Pin       = GPIO_PIN_11 | GPIO_PIN_12;
  usb_pins.Mode      = GPIO_MODE_AF_PP;
  usb_pins.Pull      = GPIO_NOPULL;
  usb_pins.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  usb_pins.Alternate = GPIO_AF10_OTG1_FS;
  HAL_GPIO_Init(GPIOA, &usb_pins);
  HAL_PWREx_EnableUSBVoltageDetector();
  __HAL_RCC_USB1_OTG_HS_CLK_ENABLE();
  __HAL_RCC_USB1_OTG_HS_ULPI_CLK_DISABLE();
}

/* Drive the red LED (PC13) on: steady = "in DFU mode". */
static void led_on(void)
{
  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitTypeDef led = {0};
  led.Pin   = LED_PIN;
  led.Mode  = GPIO_MODE_OUTPUT_PP;
  led.Pull  = GPIO_NOPULL;
  led.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_PORT, &led);
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);   /* PC13 high = on */
}

/*
 * Compose the DFU alt-0 name shown by `dfu-util -l`.  Names the internal app
 * partition, or says so loudly when the device's own flash-size register does
 * not match the sector map this bootloader was built for -- in which case the
 * programming path stays disabled rather than erasing an unknown layout.
 */
static void build_alt0_str(void)
{
  char *p = g_dfu_alt0_str;
  const char *s = g_iflash_ok ? "Wio Lite AI app @0x08020000"
                              : "UNSUPPORTED flash size";
  while (*s) *p++ = *s++;
  *p = '\0';
}

/* USB CDC console: banner on connect + periodic heartbeat, plus the measured
 * cost of the last sector erase -- the number the DFU bwPollTimeout is sized
 * from, reported from the environment that actually pays it. */
static void console_task(void)
{
  static bool was_connected = false;
  static uint32_t last_ms = 0;
  static uint32_t last_erase_cyc = 0;
  bool connected = tud_cdc_connected();

  if (connected && !was_connected)
  {
    const volatile uint32_t *vec = (const volatile uint32_t *) IFLASH_APP_BASE;

    printf("\r\n=== Wio Lite AI standalone DFU bootloader ===\r\n");
    printf("internal flash     : %lu KB (%s)\r\n",
           (unsigned long) iflash_device_kb(),
           g_iflash_ok ? "sectors 1-3 = app partition"
                       : "UNSUPPORTED -- programming disabled");
    printf("app vector (MSP/PC): %08lX %08lX (%s)\r\n",
           (unsigned long) vec[0], (unsigned long) vec[1],
           g_app_present ? "app present" : "no valid app");
    printf("DFU -> app partition 0x%08lX + %lu KB.  "
           "dfu-util -d 0483:df11 -a 0 -D app.bin\r\n",
           (unsigned long) IFLASH_APP_BASE,
           (unsigned long) (IFLASH_APP_SIZE / 1024u));
  }
  was_connected = connected;

  uint32_t erase_cyc = iflash_last_erase_cycles();
  if (connected && erase_cyc && erase_cyc != last_erase_cyc)
  {
    last_erase_cyc = erase_cyc;
    /* SystemCoreClock is 550 MHz here, so cycles/550000 is milliseconds. */
    printf("[erase] 128 KB sector in %lu ms (%lu cycles)\r\n",
           (unsigned long) (erase_cyc / (SystemCoreClock / 1000u)),
           (unsigned long) erase_cyc);
  }

  uint32_t now = HAL_GetTick();
  if (connected && (now - last_ms >= 2000u))
  {
    last_ms = now;
    printf("[tick] %lu ms\r\n", (unsigned long) now);
  }
}

/* --- boot flow: DFU trigger / app validation / jump --------------------- */
#define BTN_PORT   GPIOF
#define BTN_PIN    GPIO_PIN_1    /* USER button, active-low (10K pull-up) */

/*
 * Force DFU if the USER button (PF1) is held at reset.  This is the escape hatch
 * that survives any bad app image, so it is checked before the app is even
 * looked at.
 */
static int dfu_button_held(void)
{
  __HAL_RCC_GPIOF_CLK_ENABLE();     /* nothing else enables GPIOF now */
  GPIO_InitTypeDef btn = {0};
  btn.Pin  = BTN_PIN;
  btn.Mode = GPIO_MODE_INPUT;
  btn.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN_PORT, &btn);
  for (volatile int i = 0; i < 2000; i++) { }   /* let the pull settle */
  return HAL_GPIO_ReadPin(BTN_PORT, BTN_PIN) == GPIO_PIN_RESET;  /* low */
}

/*
 * A valid app image has its initial MSP in an on-chip RAM region and its reset
 * vector inside the app partition (thumb bit set).  This rejects blank
 * (0xFFFFFFFF) / unprogrammed (0) flash -- and equally an image built for the
 * old external XIP window, whose reset vector points at 0x70000000.
 *
 * Reading the partition before anything has ever been programmed there is safe:
 * an erased flash word reads back as all-ones with no ECC error (RM0468
 * sec 4.3.10), so this cannot fault on a virgin board.
 */
static int app_valid(void)
{
  uint32_t msp = *(volatile uint32_t *)(IFLASH_APP_BASE + 0u);
  uint32_t rst = *(volatile uint32_t *)(IFLASH_APP_BASE + 4u);
  uint32_t msp_hi = msp & 0xFF000000u;
  int msp_ok = (msp_hi == 0x24000000u) ||   /* AXI-SRAM (D1) */
               (msp_hi == 0x20000000u) ||   /* DTCM / ITCM */
               (msp_hi == 0x30000000u) ||   /* D2 SRAM */
               (msp_hi == 0x38000000u);     /* D3 SRAM */
  int rst_ok = (rst >= IFLASH_APP_BASE) &&
               (rst < IFLASH_APP_BASE + IFLASH_APP_SIZE) && (rst & 1u);
  return msp_ok && rst_ok;
}

/* Hand off to the application in the internal flash.  Never returns. */
static void jump_to_app(void)
{
  volatile uint32_t const *vec = (volatile uint32_t const *) IFLASH_APP_BASE;
  uint32_t app_msp   = vec[0];
  uint32_t app_reset = vec[1];

  __disable_irq();
  /*
   * HAL_Init started SysTick; stop it and clear any pending exception so it
   * cannot fire into the app (blink, e.g., leaves SysTick as the default
   * infinite-loop handler).
   */
  SysTick->CTRL = 0;
  SysTick->LOAD = 0;
  SysTick->VAL  = 0;
  SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk;

  SCB->VTOR = (uint32_t) vec;       /* app owns the vector table now */
  __DSB();
  __ISB();

  /*
   * MSP/PSP switch + PRIMASK restore + branch as ONE register-only asm block:
   * once MSP moves, no compiler-scheduled stack access (spill/reload) can
   * sneak in between.  cpsie i restores the reset-state PRIMASK=0 for the app;
   * nothing is pending (cleared above) and no NVIC source is enabled here.
   */
  __asm volatile (
      "msr msp, %0\n\t"
      "msr psp, %0\n\t"
      "cpsie i\n\t"
      "bx %1\n\t"
      :: "r" (app_msp), "r" (app_reset) : "memory");
  __builtin_unreachable();
}

int main(void)
{
  HAL_Init();               /* NVIC grouping + SysTick at the reset clock */
  SystemClock_Config();     /* HSE -> PLL1 550, PLL2 266, PLL3 48 (USB) */

  g_iflash_ok = iflash_available();
  build_alt0_str();

  int forced    = dfu_button_held();
  g_app_present = app_valid();

  /*
   * Boot the app unless DFU is forced, the device's flash map is not the one
   * this bootloader was built for, or the app is missing/invalid.  DFU mode is
   * the safe fallback everywhere else, so the board can always be re-loaded.
   *
   * g_iflash_ok gates the JUMP as well as the programming path on purpose: if
   * the flash-size register does not report the expected 512 KB then the sector
   * map underneath 0x08020000 is not the one assumed here, and staying in DFU is
   * the recoverable answer.
   */
  if (!forced && g_iflash_ok && g_app_present)
    jump_to_app();          /* never returns */

  /* ---- DFU mode ---- */
  led_on();                 /* steady red LED = DFU mode */
  usb_hw_init();
  tusb_rhport_init_t dev_init = { .role  = TUSB_ROLE_DEVICE,
                                  .speed = TUSB_SPEED_AUTO };
  tusb_init(BOARD_TUD_RHPORT, &dev_init);
  setvbuf(stdout, NULL, _IONBF, 0);   /* unbuffered: printf reaches _write */

  for (;;)
  {
    tud_task();
    console_task();
    if (g_reboot_at_ms && (int32_t)(HAL_GetTick() - g_reboot_at_ms) >= 0)
      NVIC_SystemReset();   /* boot the freshly downloaded app */
  }
}
