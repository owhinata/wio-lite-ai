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
Mode") with line editing, history, and Tab completion. 20 commands:

| Group | Commands |
|---|---|
| system | `version` · `uptime` · `reboot` · `free` · `thread` |
| shell | `help` · `echo` |
| timing / jobs | `sleep` · `usleep` · `watch` · `jobs` · `kill` |
| diagnostics | `devmem` (peek/poke/dump) · `dmesg` · `crash` (bus/undef/div0) · `wdt` (info/starve) · `psram` (info/test/mmapscan/…) |
| wireless | `wifi` (info/on/off/reset/log/probe/rpc) |
| benchmarks | `coremark` · `membench` |

- **`thread`** — lists the ThreadX threads with state / stack use and a **`top`-style
  `cpu%` column** (ThreadX Execution Profile Kit): each thread's share of the window
  since the previous `thread` run, plus `(idle)` and `(isr)` pseudo-rows that sum to
  ~100 %. The time source is a free-running **TIM2** (not DWT — see *Key design points*).
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
  latency for DTCM / AXI-SRAM (cached vs refill) / PSRAM / internal + external flash.
- **`psram`** — the on-board **8 MB APS6408 Octal DDR PSRAM** on OCTOSPI1, memory-mapped
  at `0x90000000`, running **133 MHz Fixed Latency** (≈113 read / 154 write MB/s; see
  *Key design points*). `psram info` shows the operating state, `psram test [bytes]`
  write/verifies patterns over the window (default all 8 MB), and a set of tuning
  subcommands (`clk`/`set`/`mr0`/`phase`/`wtune`/`mmapscan`) re-derive an operating
  point at a different clock without a reflash. `mmapscan` maps the true
  memory-mapped read eye across IWDG-recovered auto-reboots (issue #16).
- **`wifi`** — an investigation probe for the on-board **RTL8720DN** Wi-Fi/BLE
  companion (issue #17). The host reaches it over `CHIP_EN` (PC3), a **LOG UART**
  (UART9 PD14/PD15) and an **AT/HS UART** (USART1 PA10/PB14); the module is held
  powered-off (PC3 low) at boot. `wifi probe` powers it up and streams its boot log
  to the console from `t=0` to identify the factory firmware (eRPC / AT / raw
  Realtek); `wifi on`/`off`/`reset`/`log` control power and open a live bridge.
  Register-only (GPIO + UART9/USART1 clock gates); the baud is derived from the
  inherited PCLK2 = 137.5 MHz — it never touches the RCC clock tree.
  `wifi rpc [baud]` (default 2 Mbaud) is the **eRPC link test** (issue #5): the
  factory firmware is Seeed's eRPC image (UART @2 Mbaud on its `Serial3` = USART1),
  and this round-trips a byte through `rpc_system_ack` — a valid CRC-framed echo
  proves the eRPC transport end to end. It is the foundation for the WiFi/BLE
  (NetXDuo) work: a hand-written clean-room C eRPC client (`app/erpc.c`) speaking the
  firmware's exact wire format (FramedTransport + BasicCodec + CRC16/0xEF4A).

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
  the D-cache**. The **PSRAM window is MPU Normal non-cacheable** (`app/mpu.c`,
  configured between I- and D-cache enable), so future DMA peripherals
  (camera/SD/eth) can place their buffers there coherently — new regions go in
  `mpu_regions[]` rather than per-transfer cache maintenance.
- **PSRAM (issues #3 / #16)**: `app/psram.c` brings up the **APS6408L Octal DDR PSRAM
  on OCTOSPI1** (memory-mapped 8 MB @ `0x90000000`) without touching OCTOSPI2/OCTOSPIM/
  RCC, so it is XIP-safe. Shipped operating point: **133 MHz Fixed Latency** (kernel
  266 MHz / 2), MR0 = 0x24 (Fixed LC8), read dummy 8, write dummy 4, DLYB phase 3 /
  unit 8 — ≈2.4× the 53.2 MHz point (113 read / 154 write MB/s). High clocks *require*
  Fixed Latency: the power-up variable latency's refresh push-out jitter collapses the
  read eye to nothing at 133 MHz. Because register writes are only reliable at low
  clock, `psram_hw_init()` is **two-stage**: Global Reset + mode-register reads + the
  MR0 write all run at a safe 53.2 MHz, then it raises the clock and re-centres the
  DLYB. Read dummy = the datasheet latency code LC (not the ST driver's LC−1, which
  opens the DQS-gated capture window a clock early and misses the burst). Init uses the
  datasheet 4-clock Global Reset (RESET# is not wired) and is fail-soft: bounded polls,
  no unbounded mmap reads on an unresponsive device (an unanswered DQS-gated mmap read
  stalls the AXI bus until the IWDG resets — diagnostics read back via the abortable
  indirect path, and `psram mmapscan` maps the true mmap eye across IWDG-recovered
  auto-reboots). Two lower points (88.7 MHz, 53.2 MHz) are documented at the `PSRAM_*`
  defines for a wider-margin rebuild.
- **ThreadX**: SysTick priority **>** PendSV (PendSV lowest) so the tick can preempt
  the idle PendSV spin; PRIMASK-based critical sections. The shared SysTick feeds
  both `HAL_IncTick` and `_tx_timer_interrupt` (`port/threadx/tx_glue.c`).
- **Idle power saving (WFI) + thread cpu%.** When no thread is ready the scheduler
  sleeps the core with `WFI` (`TX_ENABLE_WFI`) instead of busy-spinning; any enabled
  IRQ (SysTick, OTG_HS RX) wakes it. The `thread` cpu% column uses the ThreadX
  Execution Profile Kit, whose time source is a **free-running TIM2** (`TIM2->CNT`,
  started clock-tree-untouched in `_tx_initialize_low_level`, `TIM2LPEN` keeps it
  counting through sleep) — **not** DWT/CYCCNT, which *freezes* while the core clock
  is gated in WFI. DWT stays the `udelay`/`membench` timebase (those busy-wait in the
  foreground and never run while asleep). The app **XIP-executes from OCTOSPI2**, which
  keeps its clock in CSleep (`OCTO2LPEN`), so wake-path fetches resume normally. Build
  `-DBSP_ENABLE_WFI=OFF` for a busy-idle variant — a WFI-sleeping core needs
  connect-under-reset to attach over SWD (the app does not touch DBGMCU), so a no-sleep
  build is handy for SWD debugging.
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
- **Static allocation** for the shell engine (no heap); each thread owns its stack.
  The `membench`/`coremark` benchmarks are the only heap users — they `malloc` their
  working buffers on demand and free them after (nothing reserved when idle). The
  newlib heap is made **thread-safe** with a ThreadX mutex (`app/malloc_lock.c` backs
  `__malloc_lock`/`__malloc_unlock`), so a backgrounded benchmark and a foreground
  one can allocate concurrently without corrupting the arena.

## Layout

```
app/        main + USB CDC wiring, fault handlers, USB descriptors, retarget,
            OCTOSPI1 PSRAM bring-up (psram.c), MPU regions (mpu.c)
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
| PSRAM | `0x90000000` | external OCTOSPI1 **APS6408 8 MB Octal DDR PSRAM**, memory-mapped @ 133 MHz Fixed Latency; MPU Normal non-cacheable (DMA-coherent scratch; `.psram_noinit`). |
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
