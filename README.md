# Wio Lite AI — Eclipse ThreadX CLI shell (XIP from external flash)

An interactive **command-line shell over USB CDC** for the **Seeed Wio Lite AI**
(STM32H725AEI6, Cortex-M7 @ 550 MHz), running on **Eclipse ThreadX** and executing
**in-place (XIP) from the external OCTOSPI2 NOR flash (W25Q128, 16 MB) at
`0x70000000`**. Built with the ST HAL + CMake/Ninja; the ARM toolchain, HAL/CMSIS,
ThreadX, TinyUSB and CoreMark are fetched automatically on first configure.

The app is loaded over DFU by a **standalone USB DFU bootloader** in
[`boot/`](boot/README.md) (internal flash `0x08000000`), which configures the
clocks + OCTOSPI2 memory-mapped mode and jumps here. The bootloader is **invariant**
and the app **inherits its clock tree** — see *Key design points*.

> Ported from the sibling [`stm32f746g-disco`](https://github.com/owhinata/stm32f746g-disco)
> ThreadX shell; the HW-independent core is a verbatim port.

## What it does

Presents a `wio> ` prompt on **`/dev/ttyACM0`** (USB CDC, `0483:5740`, "CDC in FS
Mode") with line editing, history, and Tab completion. 18 commands:

| Group | Commands |
|---|---|
| system | `version` · `uptime` · `reboot` · `free` · `thread` |
| shell | `help` · `echo` |
| timing / jobs | `sleep` · `usleep` · `watch` · `jobs` · `kill` |
| diagnostics | `devmem` (peek/poke/dump) · `dmesg` · `crash` (bus/undef/div0) · `wdt` (info/starve) |
| benchmarks | `coremark` · `membench` |

- **`dmesg` / `crash`** — a reset-persistent RAM log in DTCM records faults
  (HardFault/MemManage/BusFault/UsageFault) before a reset; `dmesg` replays them
  after the board comes back. `crash` deliberately triggers a fault to test it.
- **`wdt`** — the IWDG1 independent watchdog (LSI-clocked, ~3 s) auto-recovers from a
  *non-faulting* hang (scheduler/tick stall, IRQ-off lockup, OCTOSPI2 XIP fetch stall);
  a priority-5 petter thread feeds it every ~1 s. `wdt info` shows state / timeout /
  last reset cause; `wdt starve` stops feeding to prove the reset (afterwards `dmesg`
  and `wdt info` report `reset cause: IWDG`). Build `-DBSP_ENABLE_IWDG=OFF` to compile
  it out — e.g. for SWD sessions that hold a breakpoint past the timeout (the app does
  not touch DBGMCU to freeze it under debug).
- **`coremark`** — EEMBC CoreMark. **≈2333 (4.24 CoreMark/MHz)** with both L1 caches on.
- **`membench`** — DWT-cycle-precise read/write/copy bandwidth + pointer-chase
  latency for DTCM / AXI-SRAM (cached vs refill) / internal + external flash.

## Key design points

- **Never reprograms the clock tree.** The DFU bootloader sets HSE 25 MHz → PLL1
  → 550 MHz CPU, PLL2R → OCTOSPI2 XIP (~89 MHz), and OCTOSPI2 memory-mapped mode
  before jumping here. Reprogramming the RCC (as stock CMSIS `SystemInit` /
  `HAL_Init` / `SystemClock_Config` do) stalls the XIP instruction fetch and hangs.
  `src/system_stm32h7xx.c` is a custom clock-free `SystemInit` (FPU + VTOR only);
  the ThreadX SysTick reload is computed from the inherited `SystemCoreClock`.
- **Both L1 caches on** (`SCB_EnableICache` + `SCB_EnableDCache` in `app/main.c`).
  Safe because the app is single-CPU with **no DMA master** (USB dwc2 is slave/FIFO
  — CPU ↔ FIFO by MMIO, no system-memory DMA), so one D-cache is self-coherent and
  needs no MPU/maintenance. The reset-persistent log is in **DTCM, which bypasses
  the D-cache**. *A future DMA peripheral (PSRAM/camera/SD/eth) must add MPU
  non-cacheable buffers.*
- **ThreadX**: SysTick priority **>** PendSV (PendSV lowest) so the tick can preempt
  the idle PendSV spin; PRIMASK-based critical sections. The shared SysTick feeds
  both `HAL_IncTick` and `_tx_timer_interrupt` (`port/threadx/tx_glue.c`).
- **USB CDC console** = USB1_OTG_HS driven as **FS (internal PHY)**; TinyUSB dwc2
  `rhport0` aliased to the `OTG_HS` base + `OTG_HS_IRQHandler`. A single USB thread
  owns `tud_task()` / `tud_cdc_*` **and brings up the stack** (`tusb_init`) from its
  own entry; `_write`/`printf` are retargeted to the CDC.
- **Interrupts are armed only after their ThreadX objects exist.** ThreadX init runs
  with interrupts enabled (no `__disable_irq`, matching the reference port); each
  source is instead gated until its state is ready — SysTick only calls `HAL_IncTick`
  until `tx_glue_timer_enable()` opens the ThreadX-tick gate, and `OTG_HS_IRQn` stays
  NVIC-disabled until the USB thread's `tusb_init()` (after `cli_init` created the
  shell's event flags). The IWDG is likewise armed from `tx_application_define` once
  its petter thread exists.
- **Static allocation** (no heap for the shell); each thread owns its stack.

## Layout

```
app/        main + USB CDC wiring, fault handlers, USB descriptors, retarget
shell/      core/    HW-independent CLI engine (parse/edit/history/complete/...)
            include/ public CLI API + cli_config.h
            backend/ USB CDC transport (+ dummy loopback), byte rings
            cmds/    command implementations
            test/    host unit tests (run with host gcc; see below)
port/       threadx/ ThreadX low-level init + shared SysTick glue
            coremark/ EEMBC CoreMark port (core_portme.*)
svc/        freestanding services: fmt (printf), log (DTCM ring), timebase (DWT)
src/        custom clock-free SystemInit  (also the minimal `blink` example's main)
ldscript/   STM32H725AEIx_XIP.ld  (FLASH @ 0x70000000, RAM = AXI-SRAM, DTCM log)
cmake/      ARM GNU toolchain file (auto-downloads gcc into tools/)
lib/        git submodules: cmsis_core/device_h7, stm32h7xx_hal_driver, tinyusb,
            threadx, coremark
boot/       standalone USB DFU bootloader (internal 0x08000000) — see boot/README.md
```

## Memory map (`ldscript/STM32H725AEIx_XIP.ld`)

| Region | Address | Notes |
|---|---|---|
| FLASH (XIP) | `0x70000000` | external OCTOSPI2. Chip is 16 MB; the **app owns the first 8 MB** (boot validates writes there), the upper 8 MB is reserved for a future filesystem. |
| AXI-SRAM (D1) | `0x24000000` | 320 KB; `_estack = 0x24050000` (the MSP the bootloader loads). |
| DTCM | `0x20000000` | 128 KB; holds the reset-persistent `.log_noinit` crash-log ring + `membench` scratch (bypasses the D-cache). |
| ITCM | `0x00000000` | 64 KB. |
| internal Flash | `0x08000000` | 512 KB — **DFU bootloader only**; the app does not own it. |

## Build

```bash
cmake -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-toolchain.cmake
cmake --build build            # -> build/shell.{elf,bin,hex}  (also boot.* and blink.*)
```

## Flash (over DFU)

```bash
# 1. Enter DFU mode: hold the USER button (PF1) and reset  (red LED lit; `dfu-util -l` enumerates)
# 2. Write the app; it auto-reboots into the shell:
dfu-util -d 0483:df11 -a 0 -D build/shell.bin
# then open the console:
picocom -b 115200 /dev/ttyACM0     # (any 8N1 terminal; baud is nominal for USB CDC)
```

`cmake --build build --target dfu-shell` runs the `dfu-util` step. Flashing the app
is **not** a brick risk — it writes external OCTOSPI2, never the internal
bootloader. (Re-flashing the internal `0x08000000` bootloader is a separate,
higher-risk procedure documented in [`boot/README.md`](boot/README.md).)

## Host tests

The HW-independent shell core is exercised on the build host (no board, no cross
toolchain) with plain `gcc`:

```bash
sh shell/test/run_host_tests.sh    # -> "host tests passed"
```

Covers command registration, the parser, the RX state machine / dispatch, the
output API, dummy-backend integration + flow control, the line editor, the history
ring, the byte ring, and Tab completion.

## `blink` — minimal example

The repo also builds a bare-metal LED blink (`src/main.c` → `build/blink.bin`) that
shares the clock-free `SystemInit` and XIP linker script — a minimal reference for
XIP-from-external-flash bring-up. Flash it the same way
(`dfu-util … -D build/blink.bin`).
