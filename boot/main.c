/*
 * Wio Lite AI (STM32H725AEI6) -- standalone USB DFU bootloader.
 *
 * Runs from internal flash at 0x08000000 (it replaces the stock TinyUF2).  On
 * reset it brings up its own clock tree (SystemClock_Config) and the external
 * OCTOSPI2 flash in memory-mapped mode (octospi2_init), then decides:
 *
 *   - Enter DFU mode if the USER button (PF1, active-low) is held, OR there is
 *     no valid app in the external flash, OR OCTOSPI2 failed to come up.  It
 *     enumerates as a composite DFU + CDC device; a download
 *     (dfu-util -a 0 -D app.bin) is written straight to the app base
 *     (OCTOSPI2 offset 0 = 0x70000000) via octospi.c.  After the download
 *     manifests, the board reboots into the new app.  The red LED (PC13) is
 *     held on while in DFU mode.
 *   - Otherwise jump to the app at 0x70000000 (set VTOR + MSP, branch to its
 *     reset vector) -- with SysTick stopped first so a stray tick cannot
 *     vector into the app.
 *
 * DFU mode is the safe fallback: an erased / invalid app always lands here, so
 * the board can always be re-loaded over DFU.  It touches NO option bytes /
 * RDP / DBGMCU / SWD pins, so a bad config leaves the board re-flashable over
 * SWD.
 *
 * Caches stay OFF the whole time (reset default): this keeps the OCTOSPI2
 * register / memory-mapped accesses coherent and avoids speculative reads of
 * 0x70000000 while the flash is briefly out of memory-mapped mode during a
 * program.
 */

#include <stdio.h>
#include "stm32h7xx_hal.h"
#include "tusb.h"
#include "octospi.h"

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

/* --- OCTOSPI2 status (published: DFU alt-0 name + CDC banner) ------------ */
static uint8_t      g_jedec[3];
static volatile int g_ospi_ok;       /* octospi2_init() verdict (mfr seen) */
static volatile int g_app_present;   /* app MSP lands in an on-chip RAM */

char g_dfu_alt0_str[48] = "OCTOSPI2 app @0x70000000";  /* dfu-util -l name */

/*
 * Deferred reboot: the DFU manifest callback requests one so the freshly
 * downloaded app boots.  The delay must clear dfu-util's manifest handling: on
 * seeing dfuMANIFEST it sleeps ~1000 ms, then re-reads GET_STATUS (by then we
 * are in dfuMANIFEST-WAIT-RESET, which it accepts cleanly).  Rebooting sooner
 * makes that re-read hit a vanished device -> "LIBUSB_ERROR_NO_DEVICE".  So wait
 * comfortably past 1 s, then reset from the main loop (not the callback, so the
 * status responses flush first).
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

static void hex2(char *o, uint8_t b)
{
  static const char h[] = "0123456789ABCDEF";
  o[0] = h[b >> 4];
  o[1] = h[b & 0xF];
}

/*
 * Compose the DFU alt-0 name: "id=EF4018 ok" / " NG" so `dfu-util -l` shows
 * the flash id and whether OCTOSPI2 came up.
 */
static void build_alt0_str(void)
{
  char *p = g_dfu_alt0_str;
  const char *a = "id=";
  while (*a) *p++ = *a++;
  hex2(p, g_jedec[0]); p += 2;
  hex2(p, g_jedec[1]); p += 2;
  hex2(p, g_jedec[2]); p += 2;
  const char *b = g_ospi_ok ? " ok" : " NG";
  while (*b) *p++ = *b++;
  *p = '\0';
}

/* USB CDC console: banner on connect + periodic heartbeat. */
static void console_task(void)
{
  static bool was_connected = false;
  static uint32_t last_ms = 0;
  bool connected = tud_cdc_connected();

  if (connected && !was_connected)
  {
    printf("\r\n=== Wio Lite AI standalone DFU bootloader ===\r\n");
    printf("flash JEDEC id     : %02X %02X %02X (%s)\r\n",
           g_jedec[0], g_jedec[1], g_jedec[2],
           g_ospi_ok ? "OCTOSPI2 up" : "OCTOSPI2 FAIL");
    printf("app vector[0] (MSP): %08lX (%s)\r\n",
           (unsigned long) *(volatile uint32_t *)0x70000000u,
           g_app_present ? "app present" : "no valid app");
    printf("DFU alt 0 -> app base 0x70000000.  "
           "dfu-util -d 0483:df11 -a 0 -D app.bin\r\n");
  }
  was_connected = connected;

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
 * Force DFU if the USER button (PF1) is held at reset.  PF1 is not an OCTOSPI2
 * pin, so reconfiguring it as an input does not disturb the flash bus.
 */
static int dfu_button_held(void)
{
  __HAL_RCC_GPIOF_CLK_ENABLE();     /* already on from octospi2_init */
  GPIO_InitTypeDef btn = {0};
  btn.Pin  = BTN_PIN;
  btn.Mode = GPIO_MODE_INPUT;
  btn.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN_PORT, &btn);
  for (volatile int i = 0; i < 2000; i++) { }   /* let the pull settle */
  return HAL_GPIO_ReadPin(BTN_PORT, BTN_PIN) == GPIO_PIN_RESET;  /* low */
}

/*
 * A valid app image (read through the OCTOSPI2 mmap window) has its initial MSP
 * in an on-chip RAM region and its reset vector inside the XIP window (thumb
 * bit set).  This rejects blank (0xFFFFFFFF) / unprogrammed (0) flash.
 */
static int app_valid(void)
{
  uint32_t msp = *(volatile uint32_t *)0x70000000u;
  uint32_t rst = *(volatile uint32_t *)0x70000004u;
  uint32_t msp_hi = msp & 0xFF000000u;
  int msp_ok = (msp_hi == 0x24000000u) ||   /* AXI-SRAM (D1) */
               (msp_hi == 0x20000000u) ||   /* DTCM / ITCM */
               (msp_hi == 0x30000000u) ||   /* D2 SRAM */
               (msp_hi == 0x38000000u);     /* D3 SRAM */
  int rst_ok = ((rst & 0xFF000000u) == 0x70000000u) && (rst & 1u);
  return msp_ok && rst_ok;
}

/* Hand off to the application in the OCTOSPI2 XIP flash.  Never returns. */
static void jump_to_app(void)
{
  volatile uint32_t const *vec = (volatile uint32_t const *)0x70000000u;
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

  /* Bring up the external OCTOSPI2 flash in memory-mapped mode. */
  g_ospi_ok = octospi2_init(g_jedec);
  build_alt0_str();

  int forced    = dfu_button_held();
  g_app_present = app_valid();

  /*
   * Boot the app unless DFU is forced, the app is missing/invalid, or OCTOSPI2
   * (needed to reach the app) failed to come up.  DFU mode is the safe
   * fallback everywhere else, so the board can always be re-loaded.
   */
  if (!forced && g_ospi_ok && g_app_present)
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
