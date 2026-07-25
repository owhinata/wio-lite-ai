/*
 * Minimal CMSIS system layer for the Wio Lite AI blink (STM32H725AEI6) running
 * XIP from external OCTOSPI2 flash under the TinyUF2 bootloader.
 *
 * WHY THIS REPLACES THE STOCK system_stm32h7xx.c
 * ----------------------------------------------
 * The application executes in place from the external OCTOSPI2 NOR flash mapped
 * at 0x70000000.  That flash's kernel clock is PLL2R (266 MHz), derived from the
 * 25 MHz HSE, and the whole clock tree (HSE -> PLL1 550 MHz CPU / PLL2 -> OCTOSPI2
 * XIP / PLL3 -> 48 MHz USB) plus OCTOSPI2 memory-mapped mode were ALL configured
 * by the bootloader before it jumped here (verified by disassembly + live RCC read).
 *
 * The stock CMSIS SystemInit() RESETS the RCC to HSI and disables HSE/PLL2 --
 * that would stall the very instruction fetch feeding this code and hang the MCU.
 * So SystemInit() here touches NO clock register: it only enables the FPU and
 * points VTOR at our vector table.  We inherit the bootloader's 550 MHz clock.
 */

#include "stm32h7xx.h"

/* Our vector table lives at the start of the XIP flash window. */
#define VECT_TAB_BASE_ADDRESS  0x70000000UL

/* ITCM geometry -- mirrors the ITCM entry of the MEMORY block in
 * ldscript/STM32H725AEIx_XIP.ld (64 KB because the TCM_AXI_SHARED option byte is
 * at its default, which is also why AXI-SRAM is 320 KB).  Both are multiples of 8,
 * which the 64-bit zero-fill below relies on. */
#define ITCM_BASE  0x00000000UL
#define ITCM_SIZE  (64UL * 1024UL)

/* .itcm bounds from the linker script (issue #24): run image in ITCM, load image
 * in the XIP flash.  Declared as arrays so a bare reference yields the address. */
extern uint32_t _sitcm[], _eitcm[], _sitcm_load[];

/*
 * Move the interrupt paths into ITCM (issue #24).
 *
 * Every ISR in this app is otherwise fetched from the external OCTOSPI2 flash
 * through a 16 KB I-cache, so each burst that arrives after the lines were evicted
 * pays a cold fetch: measured 8.7 us vs 3.3 us for the same UART ISR on board #2.
 * ITCM is zero-wait-state and outside both caches.  The CMSIS startup only copies
 * .data, so .itcm needs its own copy -- done here, the earliest point in the boot
 * flow (Reset_Handler: ExitRun0Mode -> SystemInit -> .data copy -> .bss zero ->
 * __libc_init_array -> main), because ITCM holds garbage until we fill it and
 * running that garbage as code is fatal.  On the normal path nothing dispatches
 * while we run: SysTick is stopped and every NVIC line is disabled out of reset.
 * Note the window is not fully closed -- VTOR already points at a table whose fault
 * entries are ITCM addresses, so a fault raised *inside* this function would branch
 * into memory that is still being zeroed or copied.  That is accepted: it needs a
 * fault in ~40 instructions of straight-line code that touches only SCB and ITCM,
 * and the pre-#24 behaviour (spin forever in Default_Handler) was no more
 * recoverable.  A DFU reflash (hold PF1 at reset) always gets the board back.
 *
 * This function reads and writes only ITCMCR.  It does NOT touch the RCC / PWR /
 * FLASH ACR -- the app inherits the bootloader's clock tree and OCTOSPI2 XIP map
 * (see the file header), and this code is itself running from that flash.
 */
static void itcm_init(void)
{
  uint32_t itcmcr;
  uint64_t volatile *zp;
  uint32_t volatile *dst;
  const uint32_t *src;
  uint32_t n;

  /* Enable the ITCM and, unconditionally, its ECC read-modify-write + retry
   * (PM0253 4.9.1 -- a TCM with error detection/correction wants both).  ST leaves
   * the TCMs enabled out of reset, but the ITCM has never been used on this board,
   * so do not assume it: setting bits that are already set is idempotent.  RMW
   * matters most -- ITCM ECC covers a 64-bit word (RM0468), so the 32-bit copy
   * below is a partial write and needs the hardware read-modify-write to compute
   * the new ECC instead of corrupting it. */
  itcmcr = SCB->ITCMCR;
  if ((itcmcr & SCB_ITCMCR_EN_Msk) == 0u)
    itcmcr |= SCB_ITCMCR_EN_Msk;
  itcmcr |= SCB_ITCMCR_RMW_Msk | SCB_ITCMCR_RETEN_Msk;
  SCB->ITCMCR = itcmcr;
  __DSB();
  __ISB();

  /* Initialise the ECC over the WHOLE 64 KB, not just the resident range: the
   * Code region is Normal memory, so the core may speculatively prefetch past the
   * last resident instruction, and reading an ITCM word that was never written
   * reads an undefined ECC.  64-bit stores match the ECC granule exactly (and STRD
   * always faults if misaligned -- ITCM base and length are both multiples of 8,
   * so the loop stays aligned).  ~8 k stores, tens of microseconds once at boot. */
  for (zp = (uint64_t volatile *)ITCM_BASE;
       zp < (uint64_t volatile *)(ITCM_BASE + ITCM_SIZE); zp++)
    *zp = 0u;

  /* Copy the resident image.  32-bit words: the load address only has to be
   * 4-byte aligned, and the destination ECC is already initialised above. */
  dst = (uint32_t volatile *)_sitcm;
  src = (const uint32_t *)_sitcm_load;
  n   = (uint32_t)(_eitcm - _sitcm);      /* pointer difference -> words */
  while (n-- != 0u)
    *dst++ = *src++;

  /* Read the image back before executing it.  RM0468 requires a read-back for data
   * to be reliably committed to a TCM -- the same trap that made the DTCM crash log
   * lose bytes across a reset (issue #13, svc/log.c persist_*).  The accesses are
   * volatile so the compiler cannot drop the loop. */
  dst = (uint32_t volatile *)_sitcm;
  n   = (uint32_t)(_eitcm - _sitcm);
  while (n-- != 0u)
    (void) *dst++;

  /* TCM writes bypass the caches, but the pipeline still has to be told that the
   * instruction memory it is about to fetch from has changed. */
  __DSB();
  __ISB();
}

/* Inherited from the bootloader: SYSCLK = HSE(25 MHz) * 22 = 550 MHz (CPU/FCLK). */
uint32_t SystemCoreClock = 550000000UL;

/* D2/AHB domain clock (HCLK) = SYSCLK / D1CPRE(/1) / HPRE(/2) = 275 MHz.  The H7
 * HAL clock helpers (HAL_RCC_GetHCLKFreq) return this symbol, so define it
 * explicitly for the inherited clock rather than relying on incidental linkage. */
uint32_t SystemD2Clock = 275000000UL;

/* Referenced by stm32h7xx_hal_rcc.c (HAL_RCC_GetHCLKFreq etc.). */
const uint8_t D1CorePrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};

void SystemInit(void)
{
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
  SCB->CPACR |= ((3UL << (10U * 2U)) | (3UL << (11U * 2U)));  /* CP10/CP11 full access */
#endif

  /* Aim exceptions at our vector table in the XIP flash.  The bootloader already
   * set this before jumping, but re-assert it so SysTick/IRQs are unambiguously
   * ours.  Deliberately NO RCC writes (see file header).  The table stays in the
   * XIP window even though the handlers move to ITCM below: the bootloader loads
   * the MSP from 0x70000000's vector[0], so that address is a fixed contract. */
  SCB->VTOR = VECT_TAB_BASE_ADDRESS;

  /* Load the ITCM-resident interrupt paths (issue #24).  Must happen before any
   * exception can be dispatched, and before main() enables the caches / MPU. */
  itcm_init();
}

/* The CMSIS startup calls this before SystemInit to configure the power supply.
 * The bootloader already selected the board's supply (SMPS), so this is a no-op:
 * we intentionally do not define any USE_PWR_*_SUPPLY macro. */
void ExitRun0Mode(void)
{
}

void SystemCoreClockUpdate(void)
{
  /* We never reprogram the clock tree, so the value is fixed to what the
   * bootloader left us.  (Recomputing from RCC would also work, but this keeps
   * the "hands off the clocks" contract explicit.) */
  SystemCoreClock = 550000000UL;
}
