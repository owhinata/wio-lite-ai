/*
 * Linker script for Wio Lite AI (STM32H725AEI6) applications, running from the
 * INTERNAL flash app partition that the DFU bootloader programs and boots.
 *
 *   FLASH : 384 KB @ 0x08020000 -- internal flash sectors 1-3 (issue #25).  The
 *                                  bootloader owns sector 0 (0x08000000, 128 KB)
 *                                  and jumps to the vector table it finds here.
 *   RAM   : 320 KB @ 0x24000000 -- AXI-SRAM (D1).  Reserved for what a BUS MASTER
 *                                  has to reach: DMA1/DMA2 and the SDMMC1 IDMA
 *                                  cannot see either TCM (issue #46).
 *   DTCM  : 128 KB @ 0x20000000 -- tightly-coupled RAM: reset-persistent log, the
 *                                  NetX packet pool, every thread stack, and the
 *                                  main (ISR) stack at its top -- _estack =
 *                                  0x20020000, the MSP the bootloader loads from
 *                                  vector[0] and accepts as on-chip RAM.
 *   ITCM  :  64 KB @ 0x00000000 -- tightly-coupled instruction RAM; holds the ISR
 *                                  paths (.itcm, issue #24), which are zero-wait
 *                                  and outside both caches.
 *
 * WHY 0x08020000 AND NOT 0x70000000 (issue #25): the app used to execute in place
 * from the external OCTOSPI2 NOR flash mapped at 0x70000000.  That window reads at
 * ~54 MB/s (88.7 MHz Quad, membench) against ~905 MB/s for the internal flash, and
 * every cold path, every interrupt that had fallen out of the 16 KB I-cache, and
 * the whole of startup paid for it.  The bootloader no longer brings OCTOSPI2 up at
 * all, so 0x70000000 is not mapped while this app runs -- app/mpu.c fences it off.
 *
 * The app must NOT reconfigure the clock tree.  That rule outlived the move: the
 * bootloader hands over HSE 25 MHz -> PLL1 550 MHz CPU, PLL3Q 48 MHz for USB and
 * FLASH_ACR latency 3 / WRHIGHFREQ 3 for exactly this frequency.  The stock CMSIS
 * SystemInit resets the RCC to HSI and would take all of that away.
 * See src/system_stm32h7xx.c.
 *
 * The 128 KB sector size is also the erase granule, so the bootloader's own sector
 * is not reachable from anything that programs this partition (RM0468 Table 15).
 *
 * Symbols (_estack, _sdata, _edata, _sidata, _sbss, _ebss) match the CMSIS gcc
 * startup file startup_stm32h725xx.s.
 */

ENTRY(Reset_Handler)

/*
 * The main stack -- the one every exception and every ISR runs on -- lives at the
 * TOP OF DTCM, not at the top of AXI-SRAM (issue #46).
 *
 * Two reasons, and the second is the one that forced it:
 *
 *  1. It is the hottest data in the system and the CPU is its only user, so DTCM is
 *     where it belongs: zero wait states, and it stops evicting D-cache lines that
 *     belong to real work.  It pairs with the ISR code already in ITCM (issue #24).
 *
 *  2. Sitting at ORIGIN(RAM)+LENGTH(RAM) it grew DOWNWARD through whatever AXI-SRAM
 *     was still free -- and nothing accounted for it.  The linker does not know the
 *     stack exists, and a Cortex-M7 has no MSPLIM to catch the collision (that is
 *     ARMv8-M).  So "free AXI-SRAM" was simultaneously the stack's headroom and the
 *     space issue #35 wants for the camera's DMA band: placing the band would have
 *     silently handed it to both.  Moving the stack out makes the free space real.
 *
 * The bootloader accepts this: it validates the app's vector[0] against the on-chip
 * RAM regions and 0x20000000 (DTCM) is one of them -- boot/main.c app_image_valid()
 * and boot/dfu_callbacks.c msp_plausible().  boot itself is unchanged.
 *
 * The CMSIS startup reaches _estack twice, as `.word _estack` in the vector table
 * and as `ldr sp, =_estack` in Reset_Handler, so redefining the symbol moves both
 * and lib/cmsis_device_h7 stays unedited.
 */
_estack     = ORIGIN(DTCM) + LENGTH(DTCM);
_msp_size   = 0x2000;                    /* 8 KB; see the ASSERT and `free` below */
_smsp_stack = _estack - _msp_size;       /* bottom of the main stack */

/*
 * Top of AXI-SRAM, for the code that used to ask _estack for it.  Two distinct facts
 * shared one symbol until now -- "where the stack starts" and "where AXI-SRAM ends" --
 * and separating them is not cosmetic: app/retarget.c bounds the newlib heap with it,
 * so leaving _estack in place there would have set the heap ceiling to a DTCM address
 * below the heap itself and made every malloc fail, in a build that links cleanly.
 */
__ram_end = ORIGIN(RAM) + LENGTH(RAM);

_Min_Heap_Size  = 0x200;   /* required heap  */
/* Vestigial: the real main stack is _msp_size at the top of DTCM (above).  This only
 * reserves a slot inside ._user_heap_stack and is kept so the section keeps the shape
 * the CMSIS template gives it. */
_Min_Stack_Size = 0x400;

MEMORY
{
  RAM   (xrw) : ORIGIN = 0x24000000, LENGTH = 320K   /* AXI-SRAM (D1) */
  DTCM  (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
  /* ITCM is 64 KB because the TCM_AXI_SHARED option byte is at its default (all
   * 192 KB of the shared block assigned to AXI-SRAM -- which is why RAM above is
   * 320 KB, not 128 KB).  We never touch option bytes (CLAUDE.md), so 64 KB is fixed. */
  ITCM  (rx)  : ORIGIN = 0x00000000, LENGTH = 64K
  /* Internal flash sectors 1-3.  Sector 0 (0x08000000) is the DFU bootloader and
   * is deliberately outside this region: overflowing the app cannot reach it, it
   * is a link error (issue #25). */
  FLASH (rx)  : ORIGIN = 0x08020000, LENGTH = 384K
  PSRAM (xrw) : ORIGIN = 0x90000000, LENGTH = 8M     /* external OCTOSPI1 APS6408 (issue #3) */
}

/* Published so the app can report its own partition instead of hard-coding it
 * (shell `free` / `version`): a second copy of these numbers in C would be one
 * more thing to forget when the map changes. */
PROVIDE(__app_flash_start = ORIGIN(FLASH));
PROVIDE(__app_flash_size  = LENGTH(FLASH));

SECTIONS
{
  /* Interrupt vector table at the very start of the partition -- the bootloader
   * reads vector[0] (MSP) / vector[1] (Reset_Handler) from here, and refuses to
   * boot an image whose reset vector is outside this window.  The region origin
   * is 128 KB aligned, which more than covers the VTOR alignment requirement. */
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
   * When this app executed XIP from the external OCTOSPI2 flash, every ISR was
   * fetched through a 16 KB I-cache at ~54 MB/s: once coremark / USB / shell /
   * telnet had evicted the ISR lines, each burst paid the cold fetch again --
   * rtl_uart_isr() measured 8.7 us cold vs 3.3 us warm on board #2 (#23 U0-1), and
   * moving it here made it a flat 0.7 us.
   *
   * Issue #25 moved execution to the internal flash (~905 MB/s, roughly the AXI-SRAM
   * refill rate), which shrinks that gap enormously -- but does not close it.  ITCM
   * is still strictly better for an interrupt path: zero wait states, outside both
   * caches, and never evicted by whatever the threads are doing.  So this section
   * stays, with its win now measured in hundreds of nanoseconds rather than
   * microseconds.  (The historical numbers in the comments below were taken on the
   * external-XIP build; they document why each entry was chosen, not what it costs
   * today.)
   *
   * VMA in ITCM, LMA in the app partition; src/system_stm32h7xx.c copies the image
   * in SystemInit() before anything can take an interrupt (the CMSIS startup only
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
   *  3. Every C function is selected as the PAIR `.text.NAME .text.NAME.*`
   *     (issue #39).  Under LTO the compiler clones and renames functions --
   *     `_tx_event_flags_set` becomes `_tx_event_flags_set.isra.0`, and the
   *     suffixes stack (`__NVIC_SystemReset.lto_priv.0.lto_priv.0` was observed) --
   *     so a bare `.text.NAME` stops matching and the function silently returns to
   *     the flash.  Every clone suffix starts with a dot, which is why the pattern
   *     is `.text.NAME.*` and NOT `.text.NAME*`: the latter would also drag in
   *     genuinely different functions such as _tx_thread_time_slice_change.
   *
   * The ASSERTs at the end of this file catch a pattern that silently stops
   * matching (e.g. after a submodule bump renames a file) -- otherwise the code
   * just quietly moves back to the flash and the win disappears with no build
   * error.  They are an EARLY warning, not the real guard: see the comment on
   * them, and cmake/check_itcm_residency.py.
   *
   * NOT here, deliberately: the vector table itself (the bootloader reads MSP from
   * the partition base, so VTOR stays there) and the dwc2 USB ISR (~1.5 KB with a
   * wide call closure -- separate increment).  Empty for firmwares with no ISRs of
   * their own (blink). */
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
     * resume) stay in the flash: this closes the tick path, not the whole scheduler. */
    *(.text._tx_timer_expiration_process .text._tx_timer_expiration_process.*)
    *(.text._tx_thread_time_slice .text._tx_thread_time_slice.*)

    /* Execution Profile Kit (issue #2, `thread` cpu%): called from every ISR entry
     * and every context switch, so it is on both hot paths above.  The setup and
     * *_time_get query helpers stay in the flash -- they run from thread context. */
    *(.text._tx_execution_isr_enter .text._tx_execution_isr_enter.*)
    *(.text._tx_execution_isr_exit .text._tx_execution_isr_exit.*)
    *(.text._tx_execution_thread_enter .text._tx_execution_thread_enter.*)
    *(.text._tx_execution_thread_exit .text._tx_execution_thread_exit.*)

    /* SysTick vector -> HAL tick + the ThreadX timer gate (port/threadx/tx_glue.c). */
    *(.text.SysTick_Handler .text.SysTick_Handler.*)
    *(.text.HAL_IncTick .text.HAL_IncTick.*)
    *(.text.tx_glue_isr_enter .text.tx_glue_isr_enter.*)
    *(.text.tx_glue_isr_exit .text.tx_glue_isr_exit.*)

    /* RTL8720DN UART RX (issue #23): ~75 k interrupts/s once the link goes to a
     * high baud rate.  rtl_uart_isr is static and may be inlined into the two
     * handlers -- either way it ends up in ITCM, so it is not ASSERTed below. */
    *(.text.USART1_IRQHandler .text.USART1_IRQHandler.*)
    *(.text.UART9_IRQHandler .text.UART9_IRQHandler.*)
    *(.text.rtl_uart_isr*)

    /* ...and the wake-up it performs (issue #23 U0-3).  The RX ISR signals the eRPC
     * service thread so it starts reading a frame immediately instead of waiting out
     * its 1 ms poll.  Measured on board #2, adding that call took the ISR from 0.7 us
     * to 4.3 us warm and 12.9 us cold -- all of it external-flash fetch, at ~24 k interrupts/s at
     * 6 Mbaud, i.e. around a tenth of the CPU spent fetching instructions.
     *
     * _tx_thread_system_resume is the tail of that path (the service thread is
     * suspended on the event flag, so every notification really does resume it).  It
     * is shared with every other ISR-to-thread wakeup in the system, which makes
     * placing it here a general win rather than a special case. */
    *(.text.erpc_rx_ready .text.erpc_rx_ready.*)
    *(.text._txe_event_flags_set .text._txe_event_flags_set.*)
    *(.text._tx_event_flags_set .text._tx_event_flags_set.*)
    *(.text._tx_thread_system_resume .text._tx_thread_system_resume.*)

    /* ...and the two tails of that same path (issue #29).  Both are only reached
     * when a waiting thread is really made ready, which is exactly the request/reply
     * case (`wifi ver`, `wifi scan`): the eRPC service thread is parked in
     * tx_event_flags_get() WITH A TIMEOUT, so every wake-up has to unlink its timer
     * (_tx_timer_system_deactivate) and then re-check preemption.  With only the
     * heads above resident, those two were still fetched from the flash through veneers and
     * held the measured worst-case ISR at 3.5 us, versus 0.1 us for an ISR that wakes
     * nobody.  106 bytes together, and in this Cortex-M7/GNU configuration both link
     * as leaves, so nothing further follows them: preempt_check's
     * _tx_thread_system_return() is the inline PENDSVSET store from tx_port.h, not a
     * call (it would become one again under TX_DISABLE_INLINE). */
    *(.text._tx_thread_system_preempt_check .text._tx_thread_system_preempt_check.*)
    *(.text._tx_timer_system_deactivate .text._tx_timer_system_deactivate.*)

    /* Fault handlers (app/fault.c).  This buys the fetch cost of the handler body
     * only: the vector read itself still comes from the flash and fault_handler_c
     * calls log_write there too (the linker emits a veneer for it).  See the file
     * header. */
    *(.text.HardFault_Handler .text.HardFault_Handler.*)
    *(.text.fault_handler_c .text.fault_handler_c.*)
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

  /* Initialized data: LMA in the app partition (_sidata), copied to AXI-SRAM at
   * reset. */
  _sidata = LOADADDR(.data);
  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    /* Code that must execute from RAM rather than from the flash it is operating
     * on.  Kept as a placement facility -- nothing in the app uses it today, and
     * the internal flash does not need it (a program/erase stalls the fetch and
     * then resumes, RM0468 sec 4.3.8; the bootloader relies on exactly that).
     * Placed at the head of .data so the CMSIS startup's _sidata->_sdata.._edata
     * copy loads it into AXI-SRAM before main(). */
    *(.RamFunc)
    *(.RamFunc*)
    . = ALIGN(4);
    *(.data)
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } >RAM AT> FLASH

  /* ==== DTCM residents ====================================================
   *
   * These four sit HERE, ahead of .bss, and the position is load-bearing: input
   * sections are matched in the order the OUTPUT sections are written, first
   * match wins, and .bss's `*(.bss*)` below is a catch-all.  .dtcm_bss selects
   * an upstream symbol by its `.bss.<name>` section name, so placed after .bss
   * it would match nothing -- silently, with the stack simply staying in
   * AXI-SRAM.  Exactly the trap the .itcm block documents (it must precede .text
   * for the same reason).
   *
   * Their DTCM addresses are unaffected by the move: ld keeps a location counter
   * per memory region, so what fixes an address here is the order of the DTCM
   * sections relative to each OTHER, and that is unchanged.  .log_noinit stays at
   * ORIGIN(DTCM), which matters -- it is the reset-persistent log, and moving it
   * would throw away the previous boot's `dmesg` on the first run after a flash.
   */

  /* Reset-persistent RAM log ring (svc/log.c).  Placed in DTCM and marked NOLOAD
   * so the CMSIS startup -- which only copies .data and zeroes .bss -- never
   * touches it: the bytes carry over every *system* reset (reboot / fault /
   * NRST), letting `dmesg` replay a crash after the board comes back.  A *power*
   * reset (POR/BOR) loses them, which log_init() detects via its magic word.
   * DTCM (not AXI-SRAM) so a write bypasses the D-cache; but a bare store does NOT
   * durably land until it is read back (svc/log.c persist_*, issue #13).  Kept out
   * of .bss so it is not zero-filled at boot -- and out of .dtcm_bss below, which
   * SystemInit() does zero. */
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
  } >DTCM

  /*
   * CPU-only hot data moved out of AXI-SRAM (issue #46): every ThreadX thread
   * stack and the RTL8720 UART rings.
   *
   * WHY.  AXI-SRAM is the only RAM a bus master can reach -- DMA1/DMA2 and the
   * SDMMC1 IDMA cannot see either TCM (RM0468 sec 2.1.2 / 2.1.5 / 2.1.6), which is
   * why port/sd/sd_card.c has to bounce through .axi_dma above.  Of the 221 KB this
   * firmware had in AXI-SRAM, exactly 4 KB actually needed to be there; the rest was
   * CPU-only data occupying the one memory the camera's DCMI band (issue #35) has no
   * substitute for.  Stacks are also the hottest data in the system, so DTCM is not
   * a consolation prize: zero wait states and no D-cache line to evict.
   *
   * NOT here: anything a bus master touches, ever.  A DTCM buffer handed to a DMA
   * engine does not fault -- it silently transfers nothing.
   *
   * INITIALISATION.  NOLOAD, so the CMSIS startup's .bss zero-fill does not reach
   * it; SystemInit() zeroes [_sdtcm_bss, _edtcm_bss) instead, which keeps the .bss
   * contract these variables were written against (ThreadX initialises thread stacks
   * itself, but the UART rings are ordinary statics that assume zero).  The word
   * stores also initialise the DTCM ECC over this range.
   *
   * SELECTION.  Two mechanisms, on purpose:
   *   - app-owned variables carry __attribute__((section(".dtcm_bss"))) at their
   *     definition (see DTCM_BSS in include/mem_sections.h).  An attribute names the
   *     output section directly, so LTO renaming the symbol cannot break it.
   *   - three stacks are picked by section name here, because their definitions are
   *     not ours to annotate.  -fdata-sections gives each one `.bss.<name>`, and LTO
   *     may add a clone suffix -- hence the `.bss.NAME .bss.NAME.*` PAIR.  Not
   *     `.bss.NAME*`: every suffix starts with a dot, and the loose form would also
   *     swallow a different variable whose name merely starts the same way.
   *
   *     _tx_timer_thread_stack_area is upstream (lib/threadx is a read-only mirror).
   *     cdc_sh_stack / net_sh_stack are emitted by CLI_INSTANCE_DEFINE in
   *     shell/include/cli_instance.h, and that macro is portable shell code shared in
   *     spirit with the f746 firmware -- baking a board-specific section name into it
   *     would push a placement decision down a layer, against the dependency rule in
   *     CLAUDE.md.  Naming the two instances here keeps the choice in the port.
   *
   * cmake/check_dtcm_residency.py verifies after the link that all of it really
   * landed here.  A selector that stops matching is not a link error -- the data
   * just moves back to AXI-SRAM and the budget quietly regresses.
   */
  .dtcm_bss (NOLOAD) :
  {
    . = ALIGN(8);
    _sdtcm_bss = .;
    *(.dtcm_bss)
    *(.dtcm_bss.*)
    *(.bss._tx_timer_thread_stack_area .bss._tx_timer_thread_stack_area.*)
    *(.bss.cdc_sh_stack .bss.cdc_sh_stack.*)
    *(.bss.net_sh_stack .bss.net_sh_stack.*)
    . = ALIGN(8);
    _edtcm_bss = .;
    _dtcm_used_end = .;   /* DTCM high-water mark for the `free` command; keep
                           * .dtcm_bss the last DTCM section so this stays the top
                           * of the resident block (issue #11).  Passive symbol --
                           * no bytes emitted, no section moves. */
  } >DTCM

  /* ==== end DTCM residents ================================================ */

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

  /* Bus-master DMA scratch, in AXI-SRAM (issue #6).  The SDMMC1 IDMA is the first
   * bus master this firmware enables, so its buffer is the first memory the CPU and
   * something else both write.  It stays plain cacheable AXI-SRAM and the driver
   * does the clean/invalidate (port/sd/sd_card.c) -- but that only works if no OTHER
   * variable shares a 32-byte cache line with it, or maintaining this buffer would
   * silently discard or flush a neighbour.  Hence its own output section with
   * ALIGN(32) at BOTH ends, rather than an aligned array inside .bss.
   *
   * AXI-SRAM and not DTCM: the IDMA is an AHB master and the TCMs are reachable only
   * by the core and by MDMA through AHBS (RM0468 sec 2.4), so a DTCM bounce buffer
   * would fail silently.  NOLOAD: transfer scratch, never loaded or zeroed at boot. */
  .axi_dma (NOLOAD) :
  {
    . = ALIGN(32);
    *(.axi_dma)
    *(.axi_dma*)
    . = ALIGN(32);
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

  /* The cacheable carve-out (issue #9 phase 2a): the top 2 MB of the same window,
   * mapped Normal write-back by app/mpu.c region 3 for the NN runtime's CPU-only
   * working set.  Pinned to an absolute address rather than packed after the
   * residents above, because an MPU region must be a power of two at a natural
   * boundary -- 0x90600000 is 2 MB-aligned and reaches exactly the top of the
   * window, so the region needs no sub-region masking.  The ~5 MB left between the
   * two is unallocated address space, not wasted RAM.
   *
   * NOTHING A BUS MASTER TOUCHES MAY GO HERE.  The camera and display buffers rely
   * on the window being non-cacheable for coherency with no maintenance; in here
   * they would read stale lines, intermittently, only under cache pressure.  The
   * ASSERTs below cannot see that -- cmake/check_psram_ai_residency.py is what
   * checks it, in both directions, after every link. */
  .psram_ai 0x90600000 (NOLOAD) :
  {
    . = ALIGN(32);
    _psram_ai_start = .;
    *(.psram_ai)
    *(.psram_ai*)
    . = ALIGN(32);
    _psram_ai_end = .;
  } >PSRAM

  .ARM.attributes 0 : { *(.ARM.attributes) }

  /* Anything placed in PSRAM must fit inside the 8 MB OCTOSPI1 window. */
  ASSERT(_psram_end <= ORIGIN(PSRAM) + LENGTH(PSRAM),
         ".psram_noinit overflows the 8 MB OCTOSPI1 PSRAM window")

  /* The carve-out's base is duplicated in app/mpu.c (the MPU region) and in
   * cmake/check_psram_ai_residency.py.  It has to be: a linker script cannot include
   * a C header.  So each states it independently and this catches a divergence at
   * link time -- a region whose base does not match its section maps the wrong
   * bytes cacheable, which is silent until it corrupts something. */
  ASSERT(_psram_ai_start == ORIGIN(PSRAM) + 0x600000,
         ".psram_ai must start at 0x90600000 to match MPU region 3 in app/mpu.c")
  ASSERT(_psram_ai_end <= ORIGIN(PSRAM) + 0x800000,
         ".psram_ai overflows the 2 MB cacheable carve-out")
  /* Growing .psram_noinit into the carve-out would make DMA buffers cacheable. */
  ASSERT(_psram_end <= _psram_ai_start,
         ".psram_noinit has grown into the .psram_ai cacheable carve-out")

  /* The DTCM residents must fit UNDER the main stack, not merely inside DTCM.  ld
   * does not diagnose an overflowing NOLOAD region on its own, and it has no idea
   * the stack is there at all -- so without this, growing .nx_pool or .dtcm_bss past
   * _smsp_stack links cleanly and then corrupts whichever of them the stack reaches
   * first at run time.  There is no hardware backstop: ARMv7-M has no MSPLIM, and an
   * MPU guard page would turn the overflow into a lockup rather than a diagnosable
   * fault (the exception stacking itself would fail -- PM0253 sec 2.5.1/2.5.2/2.5.5),
   * which is why the runtime defence is the high-water report in `free` instead. */
  ASSERT(_dtcm_used_end <= _smsp_stack,
         "DTCM residents collide with the main stack: shrink them or _msp_size")

  /* ---- .itcm residency guards (issue #24) ---------------------------------
   * An input-section pattern that stops matching does NOT fail the link -- the
   * code just silently reverts to the flash and the interrupt-latency win vanishes
   * unnoticed.  Assert that the named entry points really did land in ITCM.
   *
   * 🔴 THESE ARE AN EARLY WARNING, NOT THE FINAL GUARD (issue #39).  Every one of
   * them is written against a SYMBOL NAME, and LTO renames symbols: once
   * `_tx_event_flags_set` has become `_tx_event_flags_set.isra.0`, DEFINED() below
   * is false, the conditional collapses to `1`, and the ASSERT passes while the
   * function sits in the flash.  The guard fails in exactly the situation it exists
   * for.  They are kept because they still fire early -- at link time, and for the
   * non-LTO and blink builds.  What actually holds the line is the post-link check
   * cmake/check_itcm_residency.py, which matches on suffix-stripped base names and
   * additionally proves that nothing inside .itcm branches out through a veneer.
   *
   * DEFINED(...) guards each one because both app firmwares share this script and
   * blink links neither ThreadX nor the UART/fault handlers; ORIGIN(ITCM) is 0, so
   * a plain "> ORIGIN" test would also be vacuous -- compare against the top.
   *
   * The two vector-table entry points are gated on a STRONG companion symbol from
   * the same module, not on themselves: the CMSIS startup declares every vector
   * .weak and aliases it to Default_Handler, so DEFINED(SysTick_Handler) is true
   * even in blink -- where it correctly lives in the flash -- and asserting on it
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
  /* The tails of the same path (issue #29) -- these are the branches that only run
   * when a thread is actually woken, i.e. the ones that dominate request/reply
   * latency. */
  ASSERT(DEFINED(_tx_thread_system_preempt_check) ?
         (_tx_thread_system_preempt_check < ORIGIN(ITCM) + LENGTH(ITCM)) : 1,
         "_tx_thread_system_preempt_check (RTL8720 RX wake-up) is not in ITCM")
  ASSERT(DEFINED(_tx_timer_system_deactivate) ?
         (_tx_timer_system_deactivate < ORIGIN(ITCM) + LENGTH(ITCM)) : 1,
         "_tx_timer_system_deactivate (RTL8720 RX wake-up) is not in ITCM")
}
