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
    . = ALIGN(4);
  } >FLASH

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

  /* Scratch working buffer for the `membench` command (cmd_membench.c), in DTCM
   * after the log ring.  NOLOAD: uninitialized diagnostic scratch, never loaded or
   * zeroed at boot.  Sits above .log_noinit; the two together use ~24 KB of DTCM. */
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
}
