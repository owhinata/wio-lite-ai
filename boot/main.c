/*
 * Wio Lite AI (STM32H725AEI6) -- custom DFU bootloader firmware.  See HANDOFF.md.
 *
 * PHASE 1.3: RAM-resident OCTOSPI2 self-programming driver, proven with an on-boot
 * erase -> program -> memory-mapped-read-back round trip against a dedicated scratch
 * sector, WITHOUT disturbing the app executing XIP from the same flash.
 *
 * Two essentials for self-programming the XIP flash on this MCU:
 *   1. The erase/program code runs from RAM (.RamFunc) with IRQs disabled, since the
 *      OCTOSPI is out of memory-mapped mode during the operation (see ospi_ram.c).
 *   2. The I/D caches are OFF: with them on, RAM-context accesses to the OCTOSPI
 *      register block are incoherent and speculative reads of the 0x70000000 window
 *      fault while mmap is off.  We disable them first thing in main().
 *
 * A RAM-resident fault handler (relocated vector table) is a safety net for the
 * mmap-off window: a fault there can't reach a handler at 0x70000000, so we vector
 * to RAM, re-arm mmap, and report "FLT pc=.. cf=.." over USB instead of locking up.
 *
 * Verdict: LED PC13 slow/mount-state on PASS, fast ~6 Hz strobe on FAIL/fault; and
 * the DFU alt-0 name (dfu-util -l) is "id=<jedec> st=OK|NG" or "FLT ...".  DFU UPLOAD
 * returns the scratch sector for byte-exact host verification.
 */

#include <stdio.h>
#include "stm32h7xx_hal.h"
#include "tusb.h"
#include "ospi_ram.h"

#define RAMFUNC __attribute__((section(".RamFunc"), noinline))

//--------------------------------------------------------------------+
// printf over USB CDC (newlib retargeting)
//--------------------------------------------------------------------+
extern char end;                          // heap start (from the linker script)
void *_sbrk(int incr)
{
  static char *heap = 0;
  if (heap == 0) heap = &end;
  char *prev = heap;
  heap += incr;
  return prev;
}

// newlib calls this for stdout/stderr; route to the CDC TX FIFO.  When the FIFO
// fills, pump tud_task() so large bursts (the config dump) aren't dropped; a guard
// bounds the wait if the host isn't reading.  (Only ever called from the main loop,
// never from a USB callback, so re-entering tud_task() here is safe.)
int _write(int fd, char *buf, int len)
{
  (void) fd;
  if (!tud_cdc_connected()) return len;
  int sent = 0;
  uint32_t guard = 0;
  while (sent < len && guard < 100000u)
  {
    uint32_t w = tud_cdc_write((uint8_t const *) buf + sent, (uint32_t) (len - sent));
    sent += (int) w;
    tud_cdc_write_flush();
    if (w == 0u) { tud_task(); guard++; }
  }
  return len;
}

//--------------------------------------------------------------------+
// LED
//--------------------------------------------------------------------+
#define LED_PORT   GPIOC
#define LED_PIN    GPIO_PIN_13
enum { BLINK_FAIL = 90, BLINK_NOT_MOUNTED = 250, BLINK_MOUNTED = 1000, BLINK_SUSPENDED = 2500 };
static volatile uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

//--------------------------------------------------------------------+
// Self-test state (published over USB)
//--------------------------------------------------------------------+
#define SCRATCH_FLASH_ADDR   0x00400000u   // AXI 0x70400000 (4 MB in, clear of the app)
#define SCRATCH_AXI_ADDR     0x70400000u
#define SELFTEST_LEN         512u          // spans two 256 B pages

static uint8_t  g_pattern[SELFTEST_LEN];
static uint8_t  g_jedec[3];
static volatile int g_selftest_ok;         // 0 = fail/pending, 1 = pass

char g_dfu_alt0_str[48] = "OCTOSPI2 XIP @0x70000000";  // dfu-util -l name

static volatile uint32_t g_faulted, g_fault_pc, g_cfsr, g_hfsr, g_bfar;

static void app_report_and_loop(void);     // fwd

//--------------------------------------------------------------------+
// RAM vector table + RAM fault handler (reachable with mmap off)
//--------------------------------------------------------------------+
static uint32_t g_ram_vt[256] __attribute__((aligned(1024)));

void RAMFUNC ram_fault_c(uint32_t *sp)     // sp -> {r0..r3,r12,lr,pc,xpsr}
{
  g_fault_pc = sp[6];
  g_cfsr = SCB->CFSR;
  g_hfsr = SCB->HFSR;
  g_bfar = SCB->BFAR;
  g_faulted = 1;
  ospi_ram_recover();                      // re-arm mmap so XIP runs again
  SCB->CFSR = g_cfsr;                       // clear sticky bits (write-1-to-clear)
  SCB->HFSR = g_hfsr;
  sp[6] = (uint32_t) app_report_and_loop;   // redirect exception return into reporter
}

__attribute__((naked)) void RAMFUNC ram_fault_entry(void)
{
  __asm volatile("tst lr,#4\n ite eq\n mrseq r0,msp\n mrsne r0,psp\n b ram_fault_c\n");
}

static void setup_ram_vectors(void)
{
  const uint32_t *xip_vt = (const uint32_t *)0x70000000u;
  for (int i = 0; i < 256; i++) g_ram_vt[i] = xip_vt[i];
  uint32_t h = (uint32_t)ram_fault_entry | 1u;
  g_ram_vt[3] = h; g_ram_vt[4] = h; g_ram_vt[5] = h; g_ram_vt[6] = h;  // Hard/MM/Bus/Usage
  __DMB();
  SCB->VTOR = (uint32_t)g_ram_vt;
  __DSB();
  __ISB();
}

//--------------------------------------------------------------------+
void SysTick_Handler(void)   { HAL_IncTick(); }
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

static void hex2(char *o, uint8_t b) { static const char h[]="0123456789ABCDEF"; o[0]=h[b>>4]; o[1]=h[b&0xF]; }
static void hex8(char *o, uint32_t v) { for (int i=0;i<4;i++) hex2(o+i*2,(uint8_t)(v>>(24-i*8))); }

static void run_octospi_selftest(void)
{
  ospi_flash_snapshot();   // capture live mmap config (mmap still active)

  for (uint32_t i = 0; i < SELFTEST_LEN; i++) g_pattern[i] = (uint8_t)(i ^ 0xA5u);

  ospi_ram_erase_program(SCRATCH_FLASH_ADDR, g_pattern, SELFTEST_LEN, g_jedec);
  // (on a fault inside the above, ram_fault_c redirects to app_report_and_loop)

  int ok = 1;
  const volatile uint8_t *rb = (const volatile uint8_t *)SCRATCH_AXI_ADDR;
  for (uint32_t i = 0; i < SELFTEST_LEN; i++)
    if (rb[i] != g_pattern[i]) { ok = 0; break; }
  g_selftest_ok = ok;

  // "id=EF4018 st=OK"
  char *p = g_dfu_alt0_str;
  const char *a = "id="; while (*a) *p++ = *a++;
  hex2(p, g_jedec[0]); p += 2; hex2(p, g_jedec[1]); p += 2; hex2(p, g_jedec[2]); p += 2;
  const char *b = ok ? " st=OK" : " st=NG"; while (*b) *p++ = *b++;
  *p = '\0';
}

static void led_task(void)
{
  static uint32_t last_ms = 0;
  uint32_t interval = (g_selftest_ok && !g_faulted) ? blink_interval_ms : BLINK_FAIL;
  uint32_t now = HAL_GetTick();
  if (now - last_ms < interval) return;
  last_ms = now;
  HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
}

// Phase 2 prep: dump the clock/OCTOSPI2/GPIO config that TinyUF2 left us, so the
// standalone bootloader (running from internal flash) can reproduce it exactly.
#define R32(x) ((unsigned long)(x))
static void dump_config(void)
{
  printf("--- RCC ---\r\n");
  printf("CR=%08lX CFGR=%08lX PLLCKSELR=%08lX PLLCFGR=%08lX\r\n",
         R32(RCC->CR), R32(RCC->CFGR), R32(RCC->PLLCKSELR), R32(RCC->PLLCFGR));
  printf("PLL1DIVR=%08lX PLL1FRACR=%08lX PLL2DIVR=%08lX PLL2FRACR=%08lX PLL3DIVR=%08lX PLL3FRACR=%08lX\r\n",
         R32(RCC->PLL1DIVR), R32(RCC->PLL1FRACR), R32(RCC->PLL2DIVR), R32(RCC->PLL2FRACR),
         R32(RCC->PLL3DIVR), R32(RCC->PLL3FRACR));
  printf("D1CFGR=%08lX D2CFGR=%08lX D3CFGR=%08lX D1CCIPR=%08lX D2CCIP1R=%08lX D2CCIP2R=%08lX D3CCIPR=%08lX\r\n",
         R32(RCC->D1CFGR), R32(RCC->D2CFGR), R32(RCC->D3CFGR), R32(RCC->D1CCIPR),
         R32(RCC->D2CCIP1R), R32(RCC->D2CCIP2R), R32(RCC->D3CCIPR));
  printf("--- PWR --- CR1=%08lX CR3=%08lX D3CR=%08lX   --- FLASH --- ACR=%08lX\r\n",
         R32(PWR->CR1), R32(PWR->CR3), R32(PWR->D3CR), R32(FLASH->ACR));
  printf("--- OCTOSPI2 --- CR=%08lX DCR1=%08lX DCR2=%08lX DCR3=%08lX\r\n",
         R32(OCTOSPI2->CR), R32(OCTOSPI2->DCR1), R32(OCTOSPI2->DCR2), R32(OCTOSPI2->DCR3));
  printf("  CCR=%08lX TCR=%08lX IR=%08lX ABR=%08lX LPTR=%08lX PIR=%08lX\r\n",
         R32(OCTOSPI2->CCR), R32(OCTOSPI2->TCR), R32(OCTOSPI2->IR), R32(OCTOSPI2->ABR),
         R32(OCTOSPI2->LPTR), R32(OCTOSPI2->PIR));
  printf("--- RCC clk enables --- AHB3ENR=%08lX AHB4ENR=%08lX APB1LENR=%08lX AHB1ENR=%08lX\r\n",
         R32(RCC->AHB3ENR), R32(RCC->AHB4ENR), R32(RCC->APB1LENR), R32(RCC->AHB1ENR));
  __HAL_RCC_OCTOSPIM_CLK_ENABLE();   // IOMNGR clock was gated -> enable so PnCR is readable
  __DSB();
  printf("--- OCTOSPIM --- CR=%08lX P1CR=%08lX P2CR=%08lX\r\n",
         R32(OCTOSPIM->CR), R32(OCTOSPIM->PCR[0]), R32(OCTOSPIM->PCR[1]));
  printf("--- GPIO: MODER OTYPER OSPEEDR PUPDR AFRL AFRH ---\r\n");
  GPIO_TypeDef *const banks[] = { GPIOA,GPIOB,GPIOC,GPIOD,GPIOE,GPIOF,GPIOG,GPIOH,GPIOJ,GPIOK };
  const char nm[] = "ABCDEFGHJK";
  for (int i = 0; i < 10; i++)
    printf("P%c %08lX %08lX %08lX %08lX %08lX %08lX\r\n", nm[i],
           R32(banks[i]->MODER), R32(banks[i]->OTYPER), R32(banks[i]->OSPEEDR),
           R32(banks[i]->PUPDR), R32(banks[i]->AFR[0]), R32(banks[i]->AFR[1]));
  printf("--- end dump ---\r\n");
}

// USB CDC console: banner on connect + periodic heartbeat, to demonstrate printf.
static void console_task(void)
{
  static bool was_connected = false;
  static uint32_t last_ms = 0;
  bool connected = tud_cdc_connected();

  if (connected && !was_connected)
  {
    printf("\r\n=== Wio Lite AI DFU bootloader (app-first, Phase 1.4 + CDC) ===\r\n");
    printf("flash JEDEC id     : %02X %02X %02X\r\n", g_jedec[0], g_jedec[1], g_jedec[2]);
    printf("OCTOSPI2 self-test : %s\r\n", g_faulted ? "FAULT" : (g_selftest_ok ? "PASS" : "FAIL"));
    printf("printf over USB CDC is live. DFU on alt 0 (dfu-util).\r\n");
    dump_config();   // Phase 2: capture TinyUF2's live clock/OCTOSPI2/GPIO setup
  }
  was_connected = connected;

  uint32_t now = HAL_GetTick();
  if (connected && (now - last_ms >= 2000u))
  {
    last_ms = now;
    printf("[tick] %lu ms\r\n", (unsigned long) now);
  }
}

static void app_report_and_loop(void)
{
  __enable_irq();   // the driver ran with IRQs masked

  if (g_faulted)
  {
    // "FLT pc=xxxxxxxx cf=xxxxxxxx" (which RAM function + fault type)
    char *p = g_dfu_alt0_str;
    const char *a = "FLT pc="; while (*a) *p++ = *a++;
    hex8(p, g_fault_pc); p += 8;
    const char *b = " cf="; while (*b) *p++ = *b++;
    hex8(p, g_cfsr); p += 8;
    *p = '\0';
  }

  usb_hw_init();
  tusb_rhport_init_t dev_init = { .role = TUSB_ROLE_DEVICE, .speed = TUSB_SPEED_AUTO };
  tusb_init(BOARD_TUD_RHPORT, &dev_init);

  setvbuf(stdout, NULL, _IONBF, 0);   // unbuffered: each printf reaches _write now

  for (;;) { tud_task(); led_task(); console_task(); }
}

int main(void)
{
  SCB_DisableICache();
  SCB_DisableDCache();

  SysTick_Config(SystemCoreClock / 1000U);

  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitTypeDef led = {0};
  led.Pin = LED_PIN; led.Mode = GPIO_MODE_OUTPUT_PP; led.Pull = GPIO_NOPULL;
  led.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_PORT, &led);

  setup_ram_vectors();       // RAM fault handler in place before touching the flash
  run_octospi_selftest();    // a fault redirects into app_report_and_loop
  app_report_and_loop();     // normal path
}

//--------------------------------------------------------------------+
void tud_mount_cb(void)                    { blink_interval_ms = BLINK_MOUNTED; }
void tud_umount_cb(void)                   { blink_interval_ms = BLINK_NOT_MOUNTED; }
void tud_suspend_cb(bool remote_wakeup_en) { (void) remote_wakeup_en; blink_interval_ms = BLINK_SUSPENDED; }
void tud_resume_cb(void)                   { blink_interval_ms = tud_mounted() ? BLINK_MOUNTED : BLINK_NOT_MOUNTED; }
