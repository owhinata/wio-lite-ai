# Wio Lite AI — LED blink (bare-metal, XIP from external flash)

A minimal LED blink for the **Seeed Wio Lite AI** (STM32H725AEI6), built with the
STM32 HAL and packaged as a `.uf2` for the on-board **TinyUF2** bootloader.

The firmware runs **execute-in-place (XIP) from the external OCTOSPI2 NOR flash
(W25Q128, 16 MB) mapped at `0x70000000`** — the same place the stock application
lives. See [`BACKUP_README.md`](BACKUP_README.md) for the full flash map and the
reverse-engineering behind these numbers.

> **Note:** this repo also contains a **standalone USB DFU bootloader** in
> [`boot/`](boot/README.md) that replaces TinyUF2 at internal flash `0x08000000`.
> On the development board (board #2) it is already flashed, so `blink` is now
> loaded over DFU (hold **PF1** at reset for DFU mode, then
> `dfu-util -d 0483:df11 -a 0 -D build/blink.bin`) rather than via a UF2 drive.

## What it does
Blinks **LED2 (red, "LED0") on `PC13`** at ~1 Hz (via its on-board NPN buffer).

## Key design points
- **Does not touch the clock tree.** The TinyUF2 bootloader configures HSE 25 MHz →
  550 MHz CPU, PLL2R → OCTOSPI2 XIP (~89 MHz), and OCTOSPI2 memory-mapped mode
  before jumping here. Reprogramming the RCC (as the stock CMSIS `SystemInit`
  does) would stall the XIP instruction fetch and hang. `src/system_stm32h7xx.c`
  is a custom, clock-free `SystemInit` (FPU + VTOR only).
- **Linked at `0x70000000`** (`ldscript/STM32H725AEIx_XIP.ld`); stack in AXI-SRAM
  (`_estack = 0x24050000`, matching the MSP the bootloader loads).
- **DWT cycle-counter delay** (CPU-clock based) — no SysTick/interrupt setup.

## Layout
```
cmake/     ARM GNU toolchain file (auto-downloads gcc 15.2.rel1 into tools/)
ldscript/  STM32H725AEIx_XIP.ld  (FLASH @ 0x70000000, RAM = AXI-SRAM)
lib/       git submodules: cmsis_core, cmsis_device_h7, stm32h7xx_hal_driver
src/       main.c (blink) + system_stm32h7xx.c (custom SystemInit)
```

## Build
```bash
cmake -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-toolchain.cmake
cmake --build build            # -> build/blink.elf/.bin/.hex
```

## Flash
The board now runs the custom DFU bootloader (see [`boot/`](boot/README.md)):

1. Enter DFU mode: hold the **USER button (PF1)** and reset the board.
2. `dfu-util -d 0483:df11 -a 0 -D build/blink.bin`

The bootloader writes it to OCTOSPI2 `0x70000000` and reboots into the blink.
(`cmake --build build --target dfu-blink` runs that dfu-util command for you.)
To restore the stock TinyUF2, re-flash per `BACKUP_README.md` /
[`boot/README.md`](boot/README.md).
