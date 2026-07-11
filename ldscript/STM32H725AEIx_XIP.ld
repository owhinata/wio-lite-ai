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

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
