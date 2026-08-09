/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    mem_sections.h
 * @brief   Placement attributes for the non-default memories (issue #46).
 *
 * The rule this firmware follows, and the reason these exist:
 *
 *     AXI-SRAM = only what a BUS MASTER has to reach
 *     DTCM     = CPU-only hot data (thread stacks, ISR rings, packet pool)
 *     PSRAM    = bulk / cold
 *     ITCM     = interrupt code (issues #24 / #29, placed in the linker script)
 *
 * AXI-SRAM is the scarce one, and not because it is small.  It is the only RAM a
 * bus master can address: DMA1/DMA2 and the SDMMC1 internal IDMA cannot see either
 * TCM (RM0468 sec 2.1.2 / 2.1.5 / 2.1.6), which is why port/sd/sd_card.c bounces
 * every SD transfer through a buffer in .axi_dma.  A survey of every *_DMA() call
 * in this firmware found exactly three buffers a master ever touches, totalling
 * 4 KB in AXI-SRAM -- the other 217 KB was CPU-only data sitting in the one memory
 * the camera's DCMI band (issue #35) has no substitute for.
 *
 * 🔴 NEVER put a buffer a DMA engine will touch in DTCM.  It does not fault: the
 * transfer simply moves nothing, and the failure surfaces much later as data that
 * was never written.
 *
 * WHY AN ATTRIBUTE HERE, WHEN .itcm IS SELECTED IN THE LINKER SCRIPT.  The .itcm
 * rule exists to keep upstream submodules unedited and the residency list readable
 * in one place.  Both still hold for code.  For data the trade runs the other way:
 * a linker pattern has to name `.bss.<symbol>`, which only exists thanks to
 * -fdata-sections and which LTO is free to rename (`foo` -> `foo.lto_priv.0`), so
 * the selector can stop matching with no diagnostic -- the variable just returns to
 * AXI-SRAM.  An attribute names the output section at the definition, so renaming
 * the symbol cannot break it.  The linker script therefore selects by name only
 * where an attribute is impossible: lib/threadx is a read-only mirror.
 *
 * Either way the post-link gate is what actually holds the line -- see
 * cmake/check_dtcm_residency.py.
 */
#ifndef MEM_SECTIONS_H
#define MEM_SECTIONS_H

/**
 * Put a definition in DTCM (.dtcm_bss).
 *
 * The section is NOLOAD, so the CMSIS startup's .bss zero-fill does not reach it;
 * SystemInit() zeroes the whole span instead, which keeps the "statics start as
 * zero" contract these variables were written against.  Use it for CPU-only data
 * only -- see the 🔴 above.
 *
 * Alignment is left to the caller: ThreadX wants 8-byte-aligned stacks, and
 * spelling that at each definition keeps the requirement visible where it applies
 * rather than hidden in this macro.
 */
#define DTCM_BSS  __attribute__((section(".dtcm_bss")))

/**
 * Put a definition in the CACHEABLE PSRAM carve-out (.psram_ai, issue #9 phase 2a).
 *
 * The top 2 MB of the OCTOSPI1 window is mapped Normal write-back by app/mpu.c
 * region 3, unlike the rest of it.  This is where the NN runtime's large CPU-only
 * working set goes -- activations, input staging, model slots -- because through the
 * non-cacheable window that working set costs one bus transaction per access instead
 * of one per cache line.
 *
 * 🔴 CPU-ONLY, WITH NO EXCEPTIONS.  Everywhere else in this firmware, "PSRAM" and
 * "safe for DMA" mean the same thing, because the window is non-cacheable and the
 * camera and display drivers rely on that for coherency with no maintenance at all.
 * That is exactly untrue in here.  A buffer a bus master writes will still transfer
 * correctly and then be read back stale from the data cache -- so this failure does
 * not announce itself the way the DTCM one does (there the transfer moves nothing,
 * every time).  It depends on cache pressure, which means it can pass every test on
 * a quiet system.  cmake/check_psram_ai_residency.py refuses the build instead.
 *
 * The section is NOLOAD like .psram_noinit, so nothing zeroes it: a resident must
 * not assume statics start at zero, and a measurement over it should fill it first.
 * Alignment is left to the caller; 32 bytes keeps a buffer on cache-line boundaries
 * so a neighbour cannot share a dirty line with it.
 */
#define PSRAM_AI  __attribute__((section(".psram_ai")))

/**
 * Fill word SystemInit() stamps over the unused main stack, so `free` can report a
 * high-water mark by finding the lowest address that still holds it.
 *
 * This is the main stack's only safety net, and it is a measuring tape rather than
 * a trap on purpose.  The obvious alternative -- an MPU no-access guard page below
 * the stack -- does not work on ARMv7-M: there is no MSPLIM, so an overflow is
 * detected while the hardware is stacking the exception frame, the stacking fault
 * escalates, and the handler that would have reported it cannot be entered either
 * (PM0253 sec 2.5.1 / 2.5.2 / 2.5.5).  The result is a lockup with nothing in
 * `dmesg`, which is worse than the silent corruption it replaced.  A high-water
 * number warns before the wall instead of trapping at it.
 *
 * Defined here rather than twice: src/system_stm32h7xx.c writes it and
 * shell/cmds/cmd_free.c looks for it, and a mismatch would silently report the
 * stack as untouched.
 */
#define MSP_FILL_PATTERN  0xA5A5A5A5u

#endif /* MEM_SECTIONS_H */
