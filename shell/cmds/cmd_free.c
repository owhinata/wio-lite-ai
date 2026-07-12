/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    cmd_free.c
 * @brief   `free` shell command: per-region memory usage at runtime.
 *
 * A runtime, dynamic counterpart to the build-time `size` output.  Ported to the
 * Wio Lite AI (STM32H725) memory map: the app runs XIP from external OCTOSPI2
 * flash at 0x70000000 with RAM in AXI-SRAM (D1); DTCM is currently unused.  Pure
 * introspection -- it reads linker-provided boundary symbols and the C library's
 * malloc accounting; it changes no state and touches only the shell instance
 * passed to it, so it stays reentrant across instances (req §10).
 *
 * Per-region accounting (linker symbols in ldscript/STM32H725AEIx_XIP.ld):
 *   Flash  used = LOADADDR(.data) + sizeof(.data) - ORIGIN(FLASH).  .data's load
 *          image is the last thing placed in the XIP flash, so this is the whole
 *          footprint (== `size`'s text+data).
 *   RAM    static = _end - ORIGIN(RAM) (.data + .bss + ThreadX stacks/objects);
 *          the heap grows up from _end and the MSP/ISR stack grows down from
 *          _estack, so used = (heap break) - ORIGIN(RAM), free = _estack - break.
 *   DTCM   unused by the app (no resident) -> 0 used.
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

/* Region geometry -- mirrors the MEMORY block of ldscript/STM32H725AEIx_XIP.ld. */
#define FLASH_ORIGIN  0x70000000u          /* external OCTOSPI2 XIP window */
#define FLASH_LENGTH  (8u * 1024u * 1024u) /* linked window (device is 16 MB) */
#define RAM_ORIGIN    0x24000000u          /* AXI-SRAM (D1) */
#define RAM_LENGTH    (320u * 1024u)
#define DTCM_ORIGIN   0x20000000u
#define DTCM_LENGTH   (128u * 1024u)

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

	uint32_t flash_used = (sym(_sidata) - FLASH_ORIGIN)
	                    + (sym(_edata) - sym(_sdata));

	uint32_t heap_arena = (uint32_t)(unsigned)mi.arena;   /* bytes sbrk'd from system */
	uint32_t heap_base  = sym(_end);
	uint32_t heap_break = heap_base + heap_arena;
	uint32_t ram_used   = heap_break - RAM_ORIGIN;        /* static + heap */

	(void)argc;
	(void)argv;

	cli_print(sh, "%-6s %-10s %9s %9s %9s %4s\r\n",
	          "region", "start", "total", "used", "free", "use%");
	print_region(sh, "Flash", FLASH_ORIGIN, FLASH_LENGTH, flash_used,
	             ".isr/.text/.rodata/.data (XIP)");
	print_region(sh, "RAM",   RAM_ORIGIN,   RAM_LENGTH,   ram_used,
	             ".data/.bss + ThreadX stacks + heap");
	print_region(sh, "DTCM",  DTCM_ORIGIN,  DTCM_LENGTH,  0u,
	             "(unused)");

	cli_print(sh, "\r\n");
	cli_print(sh, "heap:  base 0x%08lX  arena %lu  in-use %lu  free-pool %lu\r\n",
	          (unsigned long)heap_base, (unsigned long)heap_arena,
	          (unsigned long)(unsigned)mi.uordblks, (unsigned long)(unsigned)mi.fordblks);
	cli_print(sh, "stack: top  0x%08lX  main-reserve %lu B (MSP/ISR grow down into RAM free)\r\n",
	          (unsigned long)sym(_estack), (unsigned long)sym(_Min_Stack_Size));
	return 0;
}

CLI_CMD_REGISTER(free, NULL, "show per-region memory usage", cmd_free, 1, 0);
