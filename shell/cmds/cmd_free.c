/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    cmd_free.c
 * @brief   `free` shell command: per-region memory usage at runtime.
 *
 * A runtime, dynamic counterpart to the build-time `size` output.  Ported to the
 * Wio Lite AI (STM32H725) memory map: the app runs from the internal flash app
 * partition with RAM in AXI-SRAM (D1); DTCM holds the reset-persistent
 * log ring (.log_noinit) and the membench scratch (.dtcm_bench).  Pure
 * introspection -- it reads linker-provided boundary symbols and the C library's
 * malloc accounting; it changes no state and touches only the shell instance
 * passed to it, so it stays reentrant across instances (req §10).
 *
 * Per-region accounting (linker symbols in ldscript/STM32H725AEIx_IROM.ld):
 *   Flash  used = LOADADDR(.data) + sizeof(.data) - ORIGIN(FLASH).  .data's load
 *          image is the last thing placed in the flash, so this is the whole
 *          footprint (== `size`'s text+data).  The region bounds come from the
 *          linker too (__app_flash_start/_size), so `free` cannot disagree with
 *          the link.
 *   RAM    static = _end - ORIGIN(RAM) (.data + .bss + ThreadX stacks/objects);
 *          the heap grows up from _end and the MSP/ISR stack grows down from
 *          _estack, so used = (heap break) - ORIGIN(RAM), free = _estack - break.
 *   DTCM   used = _dtcm_used_end - ORIGIN(DTCM).  The resident block (.log_noinit
 *          then .dtcm_bench) is bump-placed from ORIGIN, so its high-water mark
 *          _dtcm_used_end (emitted at the end of the last DTCM section) is the used
 *          count -- read from the linker, not the log/membench internals.
 *   ITCM   used = _itcm_used_end - ORIGIN(ITCM), i.e. the .itcm interrupt-path
 *          residents (issue #24) plus the .itcm_bench membench buffer, bump-placed
 *          from ORIGIN exactly like the DTCM block above.  The trailing "itcm:" line
 *          breaks out the resident code size on its own and prints SCB->ITCMCR raw,
 *          which is how the ITCM enable / ECC read-modify-write / retention bits are
 *          confirmed on real hardware (SystemInit sets them; PM0253 sec 4.9.1).
 *
 * Region ORIGIN/LENGTH are compile-time constants mirroring the linker MEMORY
 * block (single source of truth: the .ld).
 *
 * Heap is read via newlib's mallinfo() rather than sbrk(0): the stock _sbrk()
 * compares the requested break against the *current* stack pointer, which in a
 * ThreadX thread is the thread's PSP (in .bss, below the heap) -- so sbrk(0) is
 * unreliable from thread context.  mallinfo() reads malloc's own accounting.
 * arena == 0 (malloc never called) prints a zero heap line.
 *
 * Clean-room design; no third-party code reused.
 */
#include "cli.h"

#include <malloc.h>   /* mallinfo / struct mallinfo */
#include <stdint.h>

#include "stm32h7xx.h"   /* CMSIS core only (SCB->ITCMCR); no HAL dependency */

/* Region geometry -- mirrors the MEMORY block of ldscript/STM32H725AEIx_IROM.ld.
 * Flash is the exception: its bounds come from the linker script itself
 * (PROVIDE __app_flash_start/_size), so `free` cannot disagree with the link. */
#define RAM_ORIGIN    0x24000000u          /* AXI-SRAM (D1) */
#define RAM_LENGTH    (320u * 1024u)
#define DTCM_ORIGIN   0x20000000u
#define DTCM_LENGTH   (128u * 1024u)
#define ITCM_ORIGIN   0x00000000u          /* .itcm ISR residents (issue #24) */
#define ITCM_LENGTH   (64u * 1024u)
#define PSRAM_ORIGIN  0x90000000u          /* external OCTOSPI1 APS6408 (issue #3) */
#define PSRAM_LENGTH  (8u * 1024u * 1024u)

/*
 * Linker boundary symbols.  Their *addresses* carry the values; _Min_Stack_Size
 * is an ABSOLUTE symbol whose address IS the byte count.  Declared as arrays so a
 * bare reference already yields the address without &.
 */
extern uint8_t _sdata[], _edata[];   /* .data run image in RAM   */
extern uint8_t _sidata[];            /* .data load image in FLASH */
extern uint8_t _end[];               /* top of static RAM = heap base */
extern uint8_t _estack[];            /* top of RAM (initial MSP)      */
extern uint8_t _Min_Stack_Size[];    /* reserved main-stack bytes     */
extern uint8_t _dtcm_used_end[];     /* top of the DTCM resident block */
extern uint8_t _psram_end[];         /* top of PSRAM residents (.psram_noinit) */
extern uint8_t _sitcm[], _eitcm[];   /* .itcm run image in ITCM (issue #24) */
/* ORIGIN(FLASH) / LENGTH(FLASH) of the app partition, PROVIDEd by the linker
 * script.  Absolute symbols: the ADDRESS carries the value, hence sym(). */
extern uint8_t __app_flash_start[], __app_flash_size[];
extern uint8_t _itcm_used_end[];     /* top of the ITCM resident block */

static uint32_t sym(const uint8_t s[])
{
	return (uint32_t)(uintptr_t)s;
}

/* One region row: name, start, total, used; free = total - used, use% = used/total. */
static void print_region(struct cli_instance *sh, const char *name,
                         uint32_t start, uint32_t total, uint32_t used,
                         const char *note)
{
	uint32_t freeb = (used <= total) ? (total - used) : 0u;
	uint32_t pct   = total ? (uint32_t)(((uint64_t)used * 100u) / total) : 0u;

	cli_print(sh, "%-6s 0x%08lX %9lu %9lu %9lu %3lu%%  %s\r\n",
	          name, (unsigned long)start, (unsigned long)total,
	          (unsigned long)used, (unsigned long)freeb, (unsigned long)pct, note);
}

static int cmd_free(struct cli_instance *sh, int argc, char **argv)
{
	struct mallinfo mi = mallinfo();   /* heap accounting (arena/uordblks/fordblks) */

	uint32_t flash_origin = sym(__app_flash_start);
	uint32_t flash_length = sym(__app_flash_size);
	uint32_t flash_used   = (sym(_sidata) - flash_origin)
	                      + (sym(_edata) - sym(_sdata));

	uint32_t heap_arena = (uint32_t)(unsigned)mi.arena;   /* bytes sbrk'd from system */
	uint32_t heap_base  = sym(_end);
	uint32_t heap_break = heap_base + heap_arena;
	uint32_t ram_used   = heap_break - RAM_ORIGIN;        /* static + heap */

	uint32_t dtcm_used  = sym(_dtcm_used_end) - DTCM_ORIGIN;  /* .log_noinit + .dtcm_bench */
	uint32_t itcm_used  = sym(_itcm_used_end) - ITCM_ORIGIN;   /* .itcm + .itcm_bench */
	uint32_t itcm_isr   = sym(_eitcm) - sym(_sitcm);           /* .itcm residents alone */
	uint32_t itcmcr     = SCB->ITCMCR;

	(void)argc;
	(void)argv;

	cli_print(sh, "%-6s %-10s %9s %9s %9s %4s\r\n",
	          "region", "start", "total", "used", "free", "use%");
	/* Memory-hierarchy order, not address order: the tightly-coupled RAMs first,
	 * then on-chip AXI-SRAM, then the internal flash the code runs from, then the
	 * external window.  `membench` prints its rows in the same order so the two
	 * can be read side by side (issue #33). */
	print_region(sh, "ITCM",  ITCM_ORIGIN,  ITCM_LENGTH,  itcm_used,
	             ".itcm ISR paths + .itcm_bench (membench)");
	print_region(sh, "DTCM",  DTCM_ORIGIN,  DTCM_LENGTH,  dtcm_used,
	             ".log_noinit (dmesg) + .dtcm_bench (membench)");
	print_region(sh, "RAM",   RAM_ORIGIN,   RAM_LENGTH,   ram_used,
	             ".data/.bss + ThreadX stacks + heap");
	print_region(sh, "Flash", flash_origin, flash_length, flash_used,
	             ".isr/.text/.rodata/.data (internal)");
	print_region(sh, "PSRAM", PSRAM_ORIGIN, PSRAM_LENGTH,
	             sym(_psram_end) - PSRAM_ORIGIN,          /* .psram_noinit residents */
	             "ext OCTOSPI1 APS6408 (free scratch pool)");

	cli_print(sh, "\r\n");
	cli_print(sh, "heap:  base 0x%08lX  arena %lu  in-use %lu  free-pool %lu\r\n",
	          (unsigned long)heap_base, (unsigned long)heap_arena,
	          (unsigned long)(unsigned)mi.uordblks, (unsigned long)(unsigned)mi.fordblks);
	cli_print(sh, "stack: top  0x%08lX  main-reserve %lu B (MSP/ISR grow down into RAM free)\r\n",
	          (unsigned long)sym(_estack), (unsigned long)sym(_Min_Stack_Size));
	/* ITCMCR as the hardware reports it, decoded.  EN must be 1 for the .itcm
	 * residents to be reachable at all, and RMW is what makes the 32-bit copy in
	 * SystemInit safe against the 64-bit ECC granule (PM0253 sec 4.9.1).  SZ is the
	 * TCM size code the part reports (0 would mean "no ITCM"). */
	cli_print(sh, "itcm:  ISR paths %lu B (.itcm)  ITCMCR 0x%08lX  EN=%lu RMW=%lu RETEN=%lu SZ=%lu\r\n",
	          (unsigned long)itcm_isr, (unsigned long)itcmcr,
	          (unsigned long)((itcmcr & SCB_ITCMCR_EN_Msk) >> SCB_ITCMCR_EN_Pos),
	          (unsigned long)((itcmcr & SCB_ITCMCR_RMW_Msk) >> SCB_ITCMCR_RMW_Pos),
	          (unsigned long)((itcmcr & SCB_ITCMCR_RETEN_Msk) >> SCB_ITCMCR_RETEN_Pos),
	          (unsigned long)((itcmcr & SCB_ITCMCR_SZ_Msk) >> SCB_ITCMCR_SZ_Pos));
	return 0;
}

CLI_CMD_REGISTER(free, NULL, "show per-region memory usage", cmd_free, 1, 0);
