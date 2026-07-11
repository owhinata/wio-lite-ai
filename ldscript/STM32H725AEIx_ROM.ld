/*
 * Linker script for the Wio Lite AI (STM32H725AEI6) STANDALONE DFU bootloader,
 * running from the internal flash at 0x08000000 (replaces TinyUF2).
 *
 *   FLASH : 128 KB @ 0x08000000 -- bootloader lives in the first internal-flash
 *                                  bank sectors.  (The H725AE has 512 KB total;
 *                                  the bootloader is kept small and low so a bad
 *                                  image is easy to reason about / re-flash.)
 *   RAM   : 320 KB @ 0x24000000 -- AXI-SRAM (D1).  _estack = 0x24050000.
 *   DTCM  : 128 KB @ 0x20000000 -- tightly-coupled RAM (unused).
 *
 * Unlike the XIP (app-first) script, the vector table and code live in the internal
 * flash; this bootloader configures the clock tree and OCTOSPI2 memory-mapped mode
 * itself (see bootrom/), then jumps to the application in the external OCTOSPI2 flash
 * at 0x70000000.
 *
 * A .RamFunc region (loaded into AXI-SRAM at reset) is kept for the OCTOSPI2
 * self-programming driver shared with the app-first firmware.
 */

ENTRY(Reset_Handler)

_estack = ORIGIN(RAM) + LENGTH(RAM);

_Min_Heap_Size  = 0x200;
_Min_Stack_Size = 0x400;

MEMORY
{
  RAM   (xrw) : ORIGIN = 0x24000000, LENGTH = 320K   /* AXI-SRAM (D1) */
  DTCM  (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 128K   /* internal flash */
}

SECTIONS
{
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

  /* Initialized data + RAM-resident code: LMA in internal flash, copied to AXI-SRAM. */
  _sidata = LOADADDR(.data);
  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.RamFunc)
    *(.RamFunc*)
    . = ALIGN(4);
    *(.data)
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } >RAM AT> FLASH

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

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
