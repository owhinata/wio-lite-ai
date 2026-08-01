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
 * The bootloader configures the clock tree itself (see boot/), programs downloads
 * into the app partition at 0x08020000 (sectors 1-3) and jumps there.  Since issue
 * #25 it does not touch the external OCTOSPI2 flash at all.
 *
 * NOTE this file describes SECTOR 0 ONLY (128 KB of the 512 KB device).  Sectors
 * 1-3 belong to the app (ldscript/STM32H725AEIx_IROM.ld); the 128 KB erase granule
 * is what keeps the two apart.
 *
 * A .RamFunc region (loaded into AXI-SRAM at reset) is kept as a placement
 * facility; the internal-flash programming path does not need it (a program or
 * erase stalls the fetch and then resumes, RM0468 sec 4.3.8).
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
