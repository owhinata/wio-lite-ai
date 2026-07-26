/*
 * Linker script for the Wio Lite AI (STM32H725AEI6) blink, running XIP from the
 * external OCTOSPI2 NOR flash (W25Q128, 16 MB) that the TinyUF2 bootloader
 * memory-maps and boots.
 *
 *   FLASH : 8 MB  @ 0x70000000  -- external OCTOSPI2 XIP window (the bootloader
 *                                  validates .uf2 writes against 0x70000000+8 MB
 *                                  and jumps to the vector table it finds here).
 *   RAM   : 320 KB @ 0x24000000 -- AXI-SRAM (D1).  _estack = 0x24050000, exactly
 *                                  the MSP the bootloader loads from vector[0].
 *   DTCM  : 128 KB @ 0x20000000 -- tightly-coupled RAM (unused by the blink).
 *   ITCM  :  64 KB @ 0x00000000 -- tightly-coupled instruction RAM; holds the ISR
 *                                  paths (.itcm, issue #24) so they are not fetched
 *                                  from OCTOSPI2 through a 16 KB I-cache.
 *
 * The app does NOT own the internal 0x08000000 flash (that is the TinyUF2
 * bootloader) and must NOT reconfigure the clock tree: it executes straight out
 * of OCTOSPI2, whose kernel clock is PLL2R -- resetting the RCC (as the stock
 * CMSIS SystemInit does) would stall the instruction fetch.  See src/system_stm32h7xx.c.
 *
 * Symbols (_estack, _sdata, _edata, _sidata, _sbss, _ebss) match the CMSIS gcc
 * startup file startup_stm32h725xx.s.
 */

ENTRY(Reset_Handler)

/* Top of AXI-SRAM -> initial stack pointer (matches the bootloader's MSP load) */
_estack = ORIGIN(RAM) + LENGTH(RAM);

_Min_Heap_Size  = 0x200;   /* required heap  */
_Min_Stack_Size = 0x400;   /* required stack */

MEMORY
{
  RAM   (xrw) : ORIGIN = 0x24000000, LENGTH = 320K   /* AXI-SRAM (D1) */
  DTCM  (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
  /* ITCM is 64 KB because the TCM_AXI_SHARED option byte is at its default (all
   * 192 KB of the shared block assigned to AXI-SRAM -- which is why RAM above is
   * 320 KB, not 128 KB).  We never touch option bytes (CLAUDE.md), so 64 KB is fixed. */
  ITCM  (rx)  : ORIGIN = 0x00000000, LENGTH = 64K
  FLASH (rx)  : ORIGIN = 0x70000000, LENGTH = 8M     /* external OCTOSPI2 XIP */
  PSRAM (xrw) : ORIGIN = 0x90000000, LENGTH = 8M     /* external OCTOSPI1 APS6408 (issue #3) */
}

SECTIONS
{
  /* Interrupt vector table at the very start of the XIP flash -- the bootloader
   * reads vector[0] (MSP) / vector[1] (Reset_Handler) from here. */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    /* ALIGN(8), not 4, so the .itcm load image that follows starts on an 8-byte
     * boundary (ITCM is 64-bit wide and its ECC granule is a 64-bit word).  Only
     * trailing padding changes; vector[0]/vector[1] stay exactly where the
     * bootloader expects them. */
    . = ALIGN(8);
  } >FLASH

  /* ---- ITCM-resident interrupt paths (issue #24) --------------------------
   * Every ISR in this app used to be fetched from the external OCTOSPI2 flash.
   * The M7 I-cache is only 16 KB, so once coremark / USB / shell / telnet run, the
   * ISR lines are evicted and each burst pays a cold XIP fetch again: measured on
   * board #2, rtl_uart_isr() took 8.7 us cold vs 3.3 us warm (#23 U0-1).  ITCM is
   * zero-wait-state and outside both caches, so residents here have a fixed cost.
   *
   * VMA in ITCM, LMA in the XIP flash; src/system_stm32h7xx.c copies the image in
   * SystemInit() before anything can take an interrupt (the CMSIS startup only
   * copies .data, so this section needs its own copy loop).
   *
   * TWO PLACEMENT RULES, both learned the hard way:
   *
   *  1. This section MUST come BEFORE .text.  Input-section matching is
   *     first-match-wins and .text's `*(.text*)` catch-all otherwise claims
   *     everything -- a version of this placed after .text linked with ITCM at 0 B.
   *
   *  2. Selection is done HERE, in the linker script, never with
   *     __attribute__((section)) in the sources.  That keeps the whole residency
   *     list readable in one place AND leaves the upstream submodules unedited
   *     (CLAUDE.md: lib/threadx and lib/tinyusb are read-only mirrors).  Two
   *     different selectors are needed because -ffunction-sections is a C-only
   *     flag: hand-written ThreadX port assembly lands in a plain `.text`, so it
   *     can only be picked by OBJECT FILE name, while C functions get their own
   *     `.text.<name>` and are picked by section name.
   *
   * The ASSERTs at the end of this file catch a pattern that silently stops
   * matching (e.g. after a submodule bump renames a file) -- otherwise the code
   * just quietly moves back to XIP and the win disappears with no build error.
   *
   * NOT here, deliberately: the vector table itself (the bootloader reads MSP from
   * 0x70000000, so VTOR stays there and the vector fetch is still an OCTOSPI2
   * read) and the dwc2 USB ISR (~1.5 KB with a wide call closure -- separate
   * increment).  Empty for firmwares with no ISRs of their own (blink). */
  .itcm :
  {
    . = ALIGN(8);
    _sitcm = .;

    /* ThreadX Cortex-M7/GNU port assembly -- by object name (plain .text).
     * The single biggest win: PendSV runs on EVERY context switch and the SysTick
     * body on every tick, so unlike the UART ISR these pay the cold cost
     * continuously.  tx_thread_schedule.S also carries the idle spin (__tx_ts_wait). */
    *tx_thread_schedule.S.obj(.text .text.*)
    *tx_timer_interrupt.S.obj(.text .text.*)

    /* The two C functions the timer ISR tail-calls when a timer actually expires
     * or a time slice runs out.  Their own callees (timer thread wakeup, thread
     * resume) stay in XIP: this closes the tick path, not the whole scheduler. */
    *(.text._tx_timer_expiration_process)
    *(.text._tx_thread_time_slice)

    /* Execution Profile Kit (issue #2, `thread` cpu%): called from every ISR entry
     * and every context switch, so it is on both hot paths above.  The setup and
     * *_time_get query helpers stay in XIP -- they run from thread context. */
    *(.text._tx_execution_isr_enter)
    *(.text._tx_execution_isr_exit)
    *(.text._tx_execution_thread_enter)
    *(.text._tx_execution_thread_exit)

    /* SysTick vector -> HAL tick + the ThreadX timer gate (port/threadx/tx_glue.c). */
    *(.text.SysTick_Handler)
    *(.text.HAL_IncTick)
    *(.text.tx_glue_isr_enter)
    *(.text.tx_glue_isr_exit)

    /* RTL8720DN UART RX (issue #23): ~75 k interrupts/s once the link goes to a
     * high baud rate.  rtl_uart_isr is static and may be inlined into the two
     * handlers -- either way it ends up in ITCM, so it is not ASSERTed below. */
    *(.text.USART1_IRQHandler)
    *(.text.UART9_IRQHandler)
    *(.text.rtl_uart_isr*)

    /* ...and the wake-up it performs (issue #23 U0-3).  The RX ISR signals the eRPC
     * service thread so it starts reading a frame immediately instead of waiting out
     * its 1 ms poll.  Measured on board #2, adding that call took the ISR from 0.7 us
     * to 4.3 us warm and 12.9 us cold -- all of it XIP fetch, at ~24 k interrupts/s at
     * 6 Mbaud, i.e. around a tenth of the CPU spent fetching instructions.
     *
     * _tx_thread_system_resume is the tail of that path (the service thread is
     * suspended on the event flag, so every notification really does resume it).  It
     * is shared with every other ISR-to-thread wakeup in the system, which makes
     * placing it here a general win rather than a special case. */
    *(.text.erpc_rx_ready)
    *(.text._txe_event_flags_set)
    *(.text._tx_event_flags_set)
    *(.text._tx_thread_system_resume)

    /* Fault handlers (app/fault.c).  This buys the fetch cost of the handler body
     * only -- it does NOT make a wedged OCTOSPI2 survivable, because the vector
     * read itself still comes from 0x70000000 and fault_handler_c calls log_write
     * in XIP (the linker emits a veneer for it).  See the file header. */
    *(.text.HardFault_Handler)
    *(.text.fault_handler_c)
    *(.text.fault_rest*)

    . = ALIGN(8);
    _eitcm = .;
  } >ITCM AT> FLASH
  _sitcm_load = LOADADDR(.itcm);

  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)
    KEEP(*(.init))
    KEEP(*(.fini))
    . = ALIGN(4);
    _etext = .;
  } >FLASH

  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  /* Shell command registry (CLI_CMD_REGISTER): each root command lands in
     .shell_root_cmds.<name>; KEEP + SORT packs them in name order between the
     boundary symbols so the core walks the section as a plain array.  Empty for
     firmwares that register no commands (e.g. blink). */
  .shell_root_cmds :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__cli_root_cmds_start = .);
    KEEP (*(SORT(.shell_root_cmds.*)))
    PROVIDE_HIDDEN (__cli_root_cmds_end = .);
    . = ALIGN(4);
  } >FLASH

  .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
  .ARM :
  {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH

  .preinit_array :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } >FLASH
  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } >FLASH
  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } >FLASH

  /* Initialized data: LMA in XIP flash (_sidata), copied to AXI-SRAM at reset. */
  _sidata = LOADADDR(.data);
  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    /* Code that must execute from RAM, not XIP: the OCTOSPI2 self-programming
     * driver runs while the flash is switched out of memory-mapped mode, so it
     * (and anything it calls) cannot be fetched from 0x70000000.  Placed at the
     * head of .data so the CMSIS startup's _sidata->_sdata.._edata copy loads it
     * into AXI-SRAM before main().  (Empty for firmwares that use no .RamFunc.) */
    *(.RamFunc)
    *(.RamFunc*)
    . = ALIGN(4);
    *(.data)
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } >RAM AT> FLASH

  /* Zero-initialized data */
  . = ALIGN(4);
  .bss :
  {
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } >RAM

  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM

  /* Reset-persistent RAM log ring (svc/log.c).  Placed in DTCM and marked NOLOAD
   * so the CMSIS startup -- which only copies .data and zeroes .bss -- never
   * touches it: the bytes carry over every *system* reset (reboot / fault /
   * NRST), letting `dmesg` replay a crash after the board comes back.  A *power*
   * reset (POR/BOR) loses them, which log_init() detects via its magic word.
   * DTCM (not AXI-SRAM) so a write bypasses the D-cache; but a bare store does NOT
   * durably land until it is read back (svc/log.c persist_*, issue #13).  Kept out
   * of .bss so it is not zero-filled at boot. */
  .log_noinit (NOLOAD) :
  {
    . = ALIGN(4);
    *(.log_noinit)
    *(.log_noinit*)
    . = ALIGN(4);
  } >DTCM

  /* NetX Duo packet pool (issue #23 U3), in DTCM after the log ring.  Frames are
   * memcpy'd in and out by the CPU alone -- the module relays them over a UART, there
   * is no DMA anywhere in this firmware -- so DTCM is not merely allowed here, it is
   * the better home: zero wait states, no D-cache line to evict, and 32 KB of 1.5 KB
   * frames kept out of the AXI-SRAM budget.  NOLOAD because nx_packet_pool_create()
   * builds every block header itself; zero-filling it at boot would be wasted work.
   * The reset-persistent read-back rule (issue #13) applies to surviving a reset, not
   * to ordinary runtime access, and nothing here outlives one. */
  .nx_pool (NOLOAD) :
  {
    . = ALIGN(32);
    *(.nx_pool)
    *(.nx_pool*)
    . = ALIGN(4);
  } >DTCM

  /* Scratch working buffer for the `membench` command (cmd_membench.c), in DTCM
   * after the log ring and the packet pool.  NOLOAD: uninitialized diagnostic
   * scratch, never loaded or zeroed at boot. */
  .dtcm_bench (NOLOAD) :
  {
    . = ALIGN(32);
    *(.dtcm_bench)
    *(.dtcm_bench*)
    . = ALIGN(4);
    _dtcm_used_end = .;   /* DTCM high-water mark for the `free` command; keep
                           * .dtcm_bench the last DTCM section so this stays the top
                           * of the resident block (issue #11).  Passive symbol -- no
                           * bytes emitted, no section moves. */
  } >DTCM

  /* Scratch working buffer for `membench`'s ITCM read row (cmd_membench.c).  ITCM
   * continues from the end of .itcm above -- ld keeps a location counter per memory
   * region, so putting this down here with the other NOLOAD sections (rather than
   * next to .itcm) does not disturb the contiguous FLASH load images.  NOLOAD, and
   * never written at runtime either: app/mpu.c marks the whole ITCM read-only, so
   * membench only measures a READ rate here (the pointer-chase latency rows need to
   * build a chain in the buffer, so ITCM gets no latency column).  The bytes are
   * valid all the same -- SystemInit() zero-fills all 64 KB of ITCM to initialise
   * its ECC. */
  .itcm_bench (NOLOAD) :
  {
    . = ALIGN(32);
    *(.itcm_bench)
    *(.itcm_bench*)
    . = ALIGN(4);
    _itcm_used_end = .;   /* ITCM high-water mark for `free`; keep .itcm_bench the
                           * last ITCM section so this stays the top of the resident
                           * block (mirrors _dtcm_used_end above). */
  } >ITCM

  /* External OCTOSPI1 APS6408 PSRAM window (issue #3), 8 MB @ 0x90000000.  The DFU
   * bootloader brings the PSRAM up in memory-mapped mode (or, during app-first
   * bring-up, app/psram.c does); the MPU marks this window Normal non-cacheable
   * (app/mpu.c) so future DMA buffers stay coherent under the D-cache.  NOLOAD:
   * volatile external RAM -- never part of the .bin, never loaded or zeroed at boot.
   * Empty for v1 (no named PSRAM residents yet); named buffers land in .psram* as
   * consumers (#5 SD / #6 LCD / #7 camera) arrive.  _psram_start/_psram_end bound
   * the free pool for a future PSRAM allocator. */
  .psram_noinit (NOLOAD) :
  {
    . = ALIGN(32);
    _psram_start = .;
    *(.psram_noinit)
    *(.psram_noinit*)
    . = ALIGN(4);
    _psram_end = .;
  } >PSRAM

  .ARM.attributes 0 : { *(.ARM.attributes) }

  /* Anything placed in PSRAM must fit inside the 8 MB OCTOSPI1 window. */
  ASSERT(_psram_end <= ORIGIN(PSRAM) + LENGTH(PSRAM),
         ".psram_noinit overflows the 8 MB OCTOSPI1 PSRAM window")

  /* The DTCM residents (.log_noinit + .nx_pool + .dtcm_bench) must fit in 128 KB.
   * ld does not diagnose an overflowing NOLOAD region on its own, and the first two
   * are a crash log and a network packet pool -- both would fail silently and
   * confusingly. */
  ASSERT(_dtcm_used_end <= ORIGIN(DTCM) + LENGTH(DTCM),
         "DTCM residents overflow the 128 KB DTCM")

  /* ---- .itcm residency guards (issue #24) ---------------------------------
   * An input-section pattern that stops matching does NOT fail the link -- the
   * code just silently reverts to XIP and the interrupt-latency win vanishes
   * unnoticed.  Assert that the named entry points really did land in ITCM.
   *
   * DEFINED(...) guards each one because both app firmwares share this script and
   * blink links neither ThreadX nor the UART/fault handlers; ORIGIN(ITCM) is 0, so
   * a plain "> ORIGIN" test would also be vacuous -- compare against the top.
   *
   * The two vector-table entry points are gated on a STRONG companion symbol from
   * the same module, not on themselves: the CMSIS startup declares every vector
   * .weak and aliases it to Default_Handler, so DEFINED(SysTick_Handler) is true
   * even in blink -- where it correctly lives in XIP -- and asserting on it
   * directly breaks the blink link. */
  ASSERT(DEFINED(_tx_thread_schedule) ?
         (_tx_thread_schedule < ORIGIN(ITCM) + LENGTH(ITCM)) : 1,
         "_tx_thread_schedule (ThreadX PendSV) left ITCM: does tx_thread_schedule.S.obj still match .itcm?")
  ASSERT(DEFINED(_tx_timer_interrupt) ?
         (_tx_timer_interrupt < ORIGIN(ITCM) + LENGTH(ITCM)) : 1,
         "_tx_timer_interrupt (ThreadX tick) left ITCM: does tx_timer_interrupt.S.obj still match .itcm?")
  ASSERT(DEFINED(tx_glue_timer_enable) ?          /* strong, tx_glue.c only */
         (SysTick_Handler < ORIGIN(ITCM) + LENGTH(ITCM)) : 1,
         "SysTick_Handler is not in ITCM")
  ASSERT(DEFINED(rtl8720_uart_open) ?             /* strong, rtl8720.c only */
         (USART1_IRQHandler < ORIGIN(ITCM) + LENGTH(ITCM)) : 1,
         "USART1_IRQHandler (RTL8720 RX) is not in ITCM")
  ASSERT(DEFINED(fault_handler_c) ?
         (fault_handler_c < ORIGIN(ITCM) + LENGTH(ITCM)) : 1,
         "fault_handler_c is not in ITCM")
  /* The RX ISR's wake-up path (issue #23 U0-3).  erpc_rx_ready itself is static and may
   * be inlined, like rtl_uart_isr, so it is not asserted -- these two are what actually
   * cost the fetch time, and they are ThreadX globals. */
  ASSERT(DEFINED(_tx_event_flags_set) ?
         (_tx_event_flags_set < ORIGIN(ITCM) + LENGTH(ITCM)) : 1,
         "_tx_event_flags_set (RTL8720 RX wake-up) is not in ITCM")
  ASSERT(DEFINED(_tx_thread_system_resume) ?
         (_tx_thread_system_resume < ORIGIN(ITCM) + LENGTH(ITCM)) : 1,
         "_tx_thread_system_resume (RTL8720 RX wake-up) is not in ITCM")
}
