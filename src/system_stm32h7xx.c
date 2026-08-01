/*
 * Minimal CMSIS system layer for Wio Lite AI (STM32H725AEI6) applications, which
 * run from the internal flash app partition (0x08020000) under the standalone DFU
 * bootloader.
 *
 * WHY THIS REPLACES THE STOCK system_stm32h7xx.c
 * ----------------------------------------------
 * The bootloader configures the entire clock tree before it jumps here (verified
 * by disassembly + live RCC read): HSE 25 MHz -> PLL1 550 MHz CPU, PLL2R 266 MHz,
 * PLL3Q 48 MHz for USB, VOS0 + SMPS, and FLASH_ACR latency 3 / WRHIGHFREQ 3 --
 * which is the read AND programming timing this flash needs at this frequency
 * (RM0468 Table 16).
 *
 * The stock CMSIS SystemInit() RESETS the RCC to HSI and disables the PLLs.  That
 * would drop the core to 64 MHz while the flash latency stays configured for 550,
 * kill the USB clock, and leave the app on a clock tree nobody set up for it.  So
 * SystemInit() here touches NO clock register: it only enables the FPU, points
 * VTOR at our vector table and loads the ITCM.  We inherit what we were handed.
 *
 * (Until issue #25 the app executed in place from the external OCTOSPI2 flash at
 * 0x70000000 and the same rule was even more immediate -- resetting the RCC stalled
 * the instruction fetch itself.  Execution moved to the internal flash for the ~17x
 * read bandwidth; the hands-off-the-clocks contract did not change.)
 */

#include "stm32h7xx.h"

/* The vector table, as the linker actually placed it: the CMSIS startup file
 * defines g_pfnVectors at the head of .isr_vector, which the linker script puts at
 * the start of the FLASH region.  Taking VTOR from the symbol rather than from a
 * hard-coded base means the two can never disagree -- the previous 0x70000000
 * literal here was exactly the kind of duplication issue #25 had to go and fix. */
extern uint32_t g_pfnVectors[];

/* ITCM geometry -- mirrors the ITCM entry of the MEMORY block in
 * ldscript/STM32H725AEIx_IROM.ld (64 KB because the TCM_AXI_SHARED option byte is
 * at its default, which is also why AXI-SRAM is 320 KB).  Both are multiples of 8,
 * which the 64-bit zero-fill below relies on. */
#define ITCM_BASE  0x00000000UL
#define ITCM_SIZE  (64UL * 1024UL)

/* .itcm bounds from the linker script (issue #24): run image in ITCM, load image
 * in the app's flash partition.  Declared as arrays so a bare reference yields the
 * address. */
extern uint32_t _sitcm[], _eitcm[], _sitcm_load[];

/*
 * Move the interrupt paths into ITCM (issue #24).
 *
 * Every ISR is otherwise fetched from the app's flash through a 16 KB I-cache, so a
 * burst arriving after the lines were evicted pays a cold fetch.  That cost was
 * brutal when the app ran from the external OCTOSPI2 flash (8.7 us vs 3.3 us for
 * the same UART ISR on board #2) and is small now that it runs from the internal
 * one (issue #25), but ITCM is zero-wait-state, outside both caches and never
 * evicted, so the residency still wins.  The CMSIS startup only copies
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
 * FLASH ACR -- the app inherits the bootloader's clock tree (see the file header),
 * and the flash this code is itself running from is timed by that ACR setting.
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

  /* Aim exceptions at our vector table.  The bootloader already set this before
   * jumping, but re-assert it so SysTick/IRQs are unambiguously ours.
   * Deliberately NO RCC writes (see file header).  The table stays in the flash
   * even though the handlers move to ITCM below: the bootloader loads the MSP from
   * the partition base's vector[0], so that address is a fixed contract. */
  SCB->VTOR = (uint32_t) g_pfnVectors;

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
