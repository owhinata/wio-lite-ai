# Wio Lite AI — Eclipse ThreadX CLI shell

An interactive **command-line shell over USB CDC** for the **Seeed Wio Lite AI**
(STM32H725AEI6, Cortex-M7 @ 550 MHz), running on **Eclipse ThreadX** from the
**internal flash app partition at `0x08020000`** (sectors 1-3, 384 KB). Built with
the ST HAL + CMake/Ninja; the ARM toolchain, HAL/CMSIS, ThreadX, TinyUSB and
CoreMark are fetched automatically on first configure.

The app is loaded over DFU by a **standalone USB DFU bootloader** in
[`boot/`](boot/README.md) (internal flash sector 0, `0x08000000`), which configures
the clocks and jumps here. The bootloader is **invariant** and the app **inherits
its clock tree** — see *Key design points*.

> Until issue #25 the app executed in place (XIP) from the external OCTOSPI2 NOR
> flash at `0x70000000`. That window reads at ~54 MB/s against ~905 MB/s for the
> internal flash, so execution moved; the bootloader no longer brings OCTOSPI2 up at
> all. Issue #37 brought it back up **app-side and indirect-only**, as storage rather
> than as an execution target — the window itself stays fenced off in the MPU.

> Ported from the sibling [`stm32f746g-disco`](https://github.com/owhinata/stm32f746g-disco)
> ThreadX shell; the HW-independent core is a verbatim port.

## What it does

Presents a `wio> ` prompt on **`/dev/ttyACM0`** (USB CDC, `0483:5740`, "CDC in FS
Mode") with line editing, history, and Tab completion — and, once the board is on WiFi,
the same shell over **telnet** (`wio-net> `, see `net shell` below), usable at the same
time as the USB console. 24 commands:

| Group | Commands |
|---|---|
| system | `version` · `uptime` · `reboot` · `free` · `thread` |
| shell | `help` · `echo` |
| timing / jobs | `sleep` · `usleep` · `watch` · `jobs` · `kill` |
| storage | `sd` (card: info/read/format · FAT: ls/cat/write/rm/mkdir/df/umount) · `nor` (info/read/erase/write/test) · `kv` (list/get/set/desc/del/info/format) |
| display | `lcd` (info/on/off · fill/bar/grad/clear/anim/blit) |
| diagnostics | `devmem` (peek/poke/dump) · `dmesg` · `crash` (bus/undef/div0) · `wdt` (info/starve) · `psram` (info/test/mmapscan/…) |
| wireless | `wifi` (L2: info/on/off/reset/log/ver · connect/status/disconnect/scan · **link** info/baud/bench/dbench · **flash** info/read/backup/imgload/imginfo/write) · `net` (L3 = **host NetX Duo only**: info/ip/dhcp/ping/echo/shell) |
| benchmarks | `coremark` · `membench` |

- **`thread`** — lists the ThreadX threads with state / stack use and a **`top`-style
  `cpu%` column** (ThreadX Execution Profile Kit): each thread's share of the window
  since the previous `thread` run, plus `(idle)` and `(isr)` pseudo-rows that sum to
  ~100 %. The time source is a free-running **TIM2** (not DWT — see *Key design points*).
- **`dmesg` / `crash`** — a reset-persistent RAM log in DTCM records faults
  (HardFault/MemManage/BusFault/UsageFault) before a reset; `dmesg` replays them
  after the board comes back. `crash` deliberately triggers a fault to test it.
- **`wdt`** — the IWDG1 independent watchdog (LSI-clocked, ~3 s) auto-recovers from a
  *non-faulting* hang (scheduler/tick stall, IRQ-off lockup, a stalled external-memory access);
  a priority-5 petter thread feeds it every ~1 s. `wdt info` shows state / timeout /
  last reset cause; `wdt starve` stops feeding to prove the reset (afterwards `dmesg`
  and `wdt info` report `reset cause: IWDG`). Build `-DBSP_ENABLE_IWDG=OFF` to compile
  it out — e.g. for SWD sessions that hold a breakpoint past the timeout (the app does
  not touch DBGMCU to freeze it under debug).
- **`coremark`** — EEMBC CoreMark. **≈2333 (4.24 CoreMark/MHz)** with both L1 caches on.
- **`membench`** — DWT-cycle-precise read/write/copy bandwidth + pointer-chase
  latency for DTCM / AXI-SRAM (cached vs refill) / PSRAM / internal + external flash.
- **`kv`** — the **persistent configuration store**: a FlashDB key-value database in
  the first megabyte of that NOR. Each entry carries its **type** (`str`/`u32`/`bool`/
  `bytes`) and a **description**, both stored on the flash next to the value, so a
  listing explains itself:

  ```
  wio> kv list
    net.shell.port         u32   2323 (0x913)
    wifi.psk               str   (hidden, 7 B)
    net.mode               str   static   -- dhcp / static / manual
    wifi.enable            bool  true     -- power the radio at boot
  ```

  `kv set <key> <value> [type]` (type defaults to `str`, and is never guessed from
  the text — an SSID of `12345` is a string); `kv desc <key> <text>` documents one,
  and a later `kv set` keeps that description. The stored settings are applied at
  boot by the configuration thread (`app/kv_boot.c`): `wifi.enable` powers the
  radio, `wifi.autoconnect` associates, `net.mode` takes an address, and
  `net.shell.autoarm` arms the telnet console — each step independent and
  fail-soft, and each one still doable by hand. **`wifi.autoconnect` does not
  currently work end to end** (issue #40): associating within ~2 s of powering the
  module fails inside the module, which predates this sequence but is met every
  time by one with no typing delay in it. `kv info` reports state and usage;
  `kv format yes` erases everything (~15 s — FlashDB erases all 256 sectors
  individually as it lays down headers). A store it cannot open is **reported, never
  silently reformatted** — the first boot on this board found the leftover XIP
  application image in that partition and said so, which is exactly the intent.
  **`wifi.psk` is stored in the clear**; see *Key design points*.
- **`nor`** — the on-board **16 MB W25Q128JV serial NOR flash** on OCTOSPI2, driven
  **indirect-only** (never memory-mapped; the `0x70000000` window stays fenced off in
  the MPU — see *Key design points*). `nor info` reports the JEDEC id, geometry and the
  live controller registers; `read`/`erase`/`write` are the raw operations; `nor test`
  is the acceptance test for the partial-page programming the coming configuration
  store depends on. Addresses are device offsets, not pointers.
- **`psram`** — the on-board **8 MB APS6408 Octal DDR PSRAM** on OCTOSPI1, memory-mapped
  at `0x90000000`, running **133 MHz Fixed Latency** (≈113 read / 154 write MB/s; see
  *Key design points*). `psram info` shows the operating state, `psram test [bytes]`
  write/verifies patterns over the window (default all 8 MB), and a set of tuning
  subcommands (`clk`/`set`/`mr0`/`phase`/`wtune`/`mmapscan`) re-derive an operating
  point at a different clock without a reflash. `mmapscan` maps the true
  memory-mapped read eye across IWDG-recovered auto-reboots (issue #16).
- **`sd`** — the on-board **microSD slot** (J4) on **SDMMC1**, 4-bit bus at 18.33 MHz,
  with an **Eclipse FileX** FAT filesystem on top (issue #6). Two layers, and the
  command reflects both:
  - *card*: `sd info` identifies the card and prints type/capacity/geometry/bus
    width/clock/CID/CSD, `sd read <lba>` hexdumps one 512 B block, and
    `sd format yes` writes a fresh whole-card FAT32 superfloppy (destructive; the
    literal `yes` is the latch).
  - *filesystem*: `sd ls [path]` · `cat` · `write` · `rm` · `mkdir` · `df` · `umount`,
    mounted **on first use** and left mounted. Both MBR-partitioned and superfloppy
    cards work: FileX only ever reads logical sector 0, so the media driver detects
    the layout at mount and adds the partition offset to every access itself.
  - The card is probed **lazily** throughout, so a boot with no card costs nothing.
    `info`/`read`/`format` may re-identify the card, which would corrupt a live
    filesystem underneath them, so they take an exclusive slot and are refused while
    the filesystem is mounted (`sd umount` first).
  - There is **no card-detect line on this board** (the socket's CD is tied to DAT3,
    and the f746 reference design's CD pin is the red LED here), so presence is
    inferred from the bus: anything ending in a command-response timeout is reported
    as `no card in slot`, and a card removed while mounted is noticed by the next
    command, which evicts the stale media and re-probes.
- **`lcd`** — the **RGB panel on the 40-pin FPC connector**, driven by the **LTDC**
  with **DMA2D** (Chrom-ART) acceleration and a tear-free double buffer (issue #7).
  Intended use is camera preview plus overlays, so there is no GUI toolkit — just a
  frame buffer and blits.
  - *patterns*: `lcd bar` (eight colour bars — the one image that proves the RGB
    channel wiring and the RGB565 bit order), `grad`, `fill <colour>`, `clear`,
    `anim` (bouncing rectangle, paced by the flip), `blit` (a strided DMA2D M2M copy).
  - *recovery*: `lcd reset` resends the ST7789 power-on sequence to a live panel
    (see *the panel keeps its state across an MCU reset* below).
  - *state*: `lcd info` reports geometry, blanking, the real PLL3R pixel clock, the
    resulting refresh rate, the frame-buffer address and the sticky **FIFO-underrun /
    transfer-error** flags. `lcd off` stops scanout **and frees OCTOSPI1** (see below);
    `lcd on` restarts it.
  - *the panel*: BL28005-B / JS28019H, 2.8" 240x320, **ST7789** controller. It has
    no datasheet in circulation, so its timing, polarity, pixel clock and — the
    part that actually mattered — its **serial power-on sequence** were recovered
    by disassembling the board's factory Arduino firmware (see *Key design
    points*). 6.00 MHz pixel clock, 298x336 total frame, 59.9 Hz.
  - *the panel keeps its state across an MCU reset* (issue #43). It has its own
    supply, so `crash` / `reboot` / a DFU reboot restart the STM32 but hand the
    bring-up an ST7789 that is still Sleep Out + DISPON. The factory transcription
    was never written against that — an Arduino sketch only ever starts cold — and
    the panel went **uniformly white and stayed white until the board was
    unplugged**, while `lcd info` reported `scanout active`, no underrun, no
    transfer error, and `camera preview` counted `dropped 0`. The sequence is
    therefore prefixed with a **SWRESET + 120 ms**, which puts the controller back
    into the state the transcription assumes, and the serial lines (which are
    LTDC_R2/R0/R1) are now **driven to their idle levels before the RESX pulse**
    rather than left floating while an awake panel listens to them.
    `lcd reset` resends the whole sequence to a live panel — stop scanout, take
    the pins back from the LTDC, pulse LCD_RST, replay, hand them back, restore
    the previous scanout state. It blocks ~0.3 s. `lcd off` / `lcd on` is *not*
    a substitute: it toggles LTDCEN and the backlight and never speaks to the
    panel at all, which is why there was no in-band recovery before this.
  - *the panel's CS is a red data line, and that is a live hazard* (issue #43,
    second half). With RGB565 the LTDC does not zero-pad the low bits — RM0468
    sec 38.3.2 says narrow components are widened **by bit replication**, and
    spells out that a 5-bit red comes out as bit positions `43210432`. So the
    controller drives `R2 = r4`, `R1 = r3`, `R0 = r2` — which on this board are
    the ST7789's **CS, SCL and SDA**. Scan-out therefore bangs the panel's
    command port with the top three bits of every pixel's red: any pixel with
    red MSB 0 asserts CS, and each 0→1 step of red bit 3 clocks in a bit. Nine
    of them and the panel executes whatever they spell (DISPOFF, SLPIN, …) and
    goes permanently blank — **uniformly white, with the LTDC reporting a
    flawless scan-out**.
    Every test pattern is safe by construction (`bar`, `grad` and `anim` are
    either red = 0 or piecewise constant), which is why #7 and #8 both passed;
    **live camera video kills it within seconds**. It was pinned down with
    `camera stream start test`, the OV2640's own colour bars: identical DVP →
    DCMI → DMA → PSRAM path at an identical rate, differing only in pixel
    values — that never fails, live video always does.
    The fix is to keep PA1 (R2/CS) out of the LTDC and park it as a GPIO driven
    high, so the command port is permanently deselected (ST7789V sec 8.4.2; the
    RGB path does not involve CSX, sec 8.9 — and the board had already proved
    both, back when CS still followed red bit 4: `lcd bar`'s bright bars drew
    with CS high and `lcd anim` ran entirely with CS low, and both displayed
    correctly). It costs the cheapest bit there is: `R[7:3]` carry the real red and
    `R[2:0]` are only replication padding, so pinning R2 adds at most 4/255 =
    1.6% red.
  - The two RGB565 frame buffers (2 × 76800 px = 300 KB) live in the **PSRAM**, which
    the MPU already maps non-cacheable — so the CPU, the DMA2D and the LTDC's read DMA
    are coherent with no cache maintenance. AXI-SRAM has no room for them.
  - Because scanout reads OCTOSPI1 *continuously*, the `psram`/`membench`/`devmem`/
    `wifi flash` paths that retune or re-enter that bus are **refused while the display
    is on** — they tell you to run `lcd off` first. Retuning the octal bus under a live
    scanout starves the LTDC FIFO, and a memory-mapped read of a half-configured
    OCTOSPI stalls the AXI until the IWDG fires.
- **`camera`** — the **OV2640 DVP camera on the 24-pin FPC connector** (J7), feeding
  the **DCMI** (issue #8). **Phases 1–2 so far**: the sensor's master clock, SCCB
  control bus, PWDN/RESETB lines and chip-ID probe (phase 1), then QVGA RGB565
  single-frame capture over DCMI + DMA into the PSRAM (phase 2), then continuous
  capture through `svc/frame_pipeline` and a live LCD preview (phase 3).
  - *capture*: `camera capture` snapshots one 320x240 RGB565 frame (153600 B) and
    prints **per-channel min/max/mean**. That statistic is the whole point — a frame
    that never arrived reads all-zero, a saturated one pins at max, and a DCMI
    latching on the wrong edge scatters min/max across the range with a mid-grey
    mean. Covering the lens and pointing at a light must move the means in opposite
    directions. `camera capture test` uses the sensor's internal colour bars, which
    exercise DVP → DCMI → DMA → PSRAM without involving optics or exposure.
  - *warm-up frames*: the sensor's exposure loop starts from the register table's
    defaults, so the frames right after a power cycle are badly under-exposed. Back
    to back after `camera off; camera probe`, means measured **R2 G11 B0 → R4 G15 B0
    → R6 G22 B3** with no warm-up (still climbing after three captures) versus
    **R14 G28 B12** on the *first* kept frame with 15 discarded. So a capture throws
    away **15 frames** (~1 s) and keeps the sixteenth. The count was a run-time knob
    while it was being measured — sweeping it down to 0 is what made the effect
    measurable rather than assumed — and is a constant now that the answer is known.
  - *`seam:`*: `camera capture` also reports the largest step between adjacent rows'
    channel means, and its row. It exists because one early capture showed a sharp
    band — at row 27 the red mean dropped 37 % while blue did not move, with the
    scene correlating 0.996 across the boundary (one frame, one scene, two gains;
    nothing the DCMI or DMA can produce). **That has never reproduced** — not with a
    lighting transient, not from a cold power cycle, not with warm-up disabled — so
    it is recorded as unexplained rather than fixed, and the metric is there to catch
    it if it returns. A settled frame reads well under 1.
  - *streaming* (`camera stream start [test] [--frames N] [--secs S] | stop | stats`):
    the DCMI runs continuously with the DMA in **double-buffer mode** over a 4-slot
    PSRAM ring, and a dedicated producer thread publishes each finished frame into
    `svc/frame_pipeline`. The tear-free trick is the ordering: the producer reads
    `CT` to find the slot the DMA just left, takes a free slot, **repoints that
    memory register first**, and only then publishes — so a slot handed to a sink
    is never a live DMA target. `camera stream stats` reports fps and the
    overrun/FIFO-error counters; **`dma fe/s` is the figure of merit** for whether
    the LTDC can scan out at the same time (phase 3c). Stopping copies the last
    streamed frame into the snapshot buffer, so `save`/`send` still work afterwards.
  - *live preview* (`camera preview on|off`, needs `lcd on` + a running stream):
    a thread in `app/cam_preview.c` pins the newest published frame, blits the
    **centre 240x240** into the LTDC back buffer and presents it. It **pulls**
    rather than registering a `frame_sink`, because a sink's `consume()` runs on
    the producer thread and the ~20 ms of blit + vertical-blanking wait would eat
    the very margin that keeps the DBM repoint safe. The crop is free: passing the
    *source* width to `ltdc_blit()` makes its existing clipping copy 240 columns
    and step the source by 320.
  - *the display and the camera share OCTOSPI1*: `psram_acquire()` refuses while
    either is live, but `psram_acquire_shared()` — what `lcd on` and the camera
    commands take — only refuses a command that is **reconfiguring** the bus. So
    scan-out and streaming coexist, while `psram`/`membench`/`devmem`/`wifi flash`
    stay locked out of both. Watch `lcd info`'s underrun flag and `camera stream
    stats`' overrun/FIFO counters: those are the instruments that say whether the
    arbitration is holding.
  - *getting the frame out*: `camera send [name]` streams it over YMODEM,
    `camera save <path>` writes it to the microSD. View with

    ```
    rz -y -b                                    # -y: overwrite, -b: binary
    ffmpeg -y -f rawvideo -pix_fmt rgb565le -s 320x240 -i frame.raw frame.png
    ```

    **Both `-y` flags matter.** `rz` will not overwrite an existing file by default
    (YMODEM block 0 carries only a name and a size, no mtime), and `ffmpeg` without
    `-y` stops at an overwrite prompt and leaves the old PNG in place. Either one
    silently gives you the *previous* capture, which looks exactly like a camera
    that has stopped updating — it cost an evening here before the raw's channel
    means were compared against the PNG's and did not match.
  - *sensor setup*: the QVGA RGB565 register sequence is ST's, vendored as data into
    `port/camera/ov2640_regs.c` (BSD-3-Clause, see `NOTICE`). Only the table is
    reused — ST's driver returns `void` everywhere, so a NAKed SCCB write is
    invisible to it; `port/camera/camera.c` walks the table with error-checked
    writes and stops at the row that failed. It runs lazily on the first capture
    (~350 ms) and again after any power cycle.
  - *orientation, and why there is no free rotation*: the camera is 320x240
    landscape, the panel is 240x320 portrait, and the H725 has **no rotation
    engine** — no GFXMMU, no GPU2D; DMA2D and MDMA *can* transpose, but only by
    turning one side of the copy into ~77k scattered 2-byte accesses, the worst
    possible shape for a serial PSRAM. The ST7789's MADCTL bit 5 (MV, row/column
    exchange) looked like a free way out, so it was **measured on the board and it
    does not work** — see the comment above `st_send_sequence()` in
    `port/ltdc/st7789_rgb.c` for exactly what was tried and what the panel did.
    Rotation therefore has to happen host-side (issue #8 phase 3c).
  - *the host must generate XCLK*. Schematic sheet 7 leaves the 24 MHz oscillator
    OSC1 and its series R15 **DNP**; the only populated path to the module's clock
    pin is R11 (0R) from `DCMI_XCLK` = **PA2**, driven by **TIM5_CH3** (AF2) at
    275 MHz / 12 = **22.92 MHz** with an exact 50 % duty. TIM2, the other candidate
    on PA2, is already the ThreadX execution-profile time source. An OmniVision
    sensor clocks its SCCB off XVCLK, so **the I2C bus stays dead until XCLK runs**
    — the single most confusing failure mode here.
  - *the camera supply cannot be switched off*. U8 (ME6216A28M3G) feeds VDD_2V8
    straight from SYS_5V with no enable pin, so `camera off` can only assert PWDN
    (PE7), hold RESETB (PH12) low and stop XCLK. The f746 firmware this port comes
    from cuts a GPIO-controlled rail; this board has none.
  - *identification*: `camera probe` power-cycles the module and tries OV2640
    (SCCB 0x30, 8-bit banked registers, PID/MID) then OV5640 (0x3C, 16-bit
    registers, ID 0x5640). If neither matches it reports **which address ACKed**
    and leaves the sensor powered so `camera scan` / `camera reg <addr> <reg>` can
    keep digging. SCCB is I2C4 on **PF14/PF15** (AF4, 4.7k board pull-ups), kernel
    clock `rcc_pclk4` = 137.5 MHz at 100 kHz nominal.
  - *the bring-up knobs are gone*. `camera xclk <hz>` and
    `camera tune {pwdn|rst|pull|i2c|sccb|swap|warm}` made every unproven board
    assumption switchable at run time, because the internal flash is rated for only
    ~10k erase cycles and sweeping an unknown by reflashing burns them (the lesson
    from the LCD bring-up above). The board answered all seven and never contradicted
    itself, so they are compile-time constants now and the commands are deleted.
    `camera reg` stays — poking the sensor's registers is still how you learn
    anything about it.
- **`wifi`** — the on-board **RTL8720DN** Wi-Fi/BLE companion (issues #17/#5/#23).
  The host reaches it over `CHIP_EN` (PC3), a **LOG UART** (UART9 PD14/PD15) and an
  **AT/HS UART** (USART1 PA10/PB14); the module is held powered-off (PC3 low) at boot.
  `wifi on`/`off`/`reset` control power; `wifi log` opens a live console bridge onto the
  LOG UART, and `wifi log reset` power-cycles the module *after* the bridge is
  listening, so the boot banner is captured from `t=0` (a separate `wifi reset; wifi
  log` cannot do that — the banner is gone before the second command starts).
  Register-only (GPIO + UART9/USART1 clock gates); the baud is derived from the
  inherited PCLK2 = 137.5 MHz — it never touches the RCC clock tree.
  RX is **FIFO-threshold driven** (issue #23): `CR3.RXFTIE` with `RXFTCFG` at half of the
  16-deep RXFIFO plus `CR1.IDLEIE` for the tail of a burst, rather than an interrupt per
  byte — at 6 Mbaud the latter would be ~600 k IRQ/s. The threshold is a latency budget:
  RM0468 puts the overrun at the 18th datum, so `18 - threshold` byte times is all the
  slack the ISR has, and `wifi ver` / `wifi scan` print how much of it was used
  (`max N/G B per irq`) together with the three distinct losses — `ore` (the hardware
  FIFO overran), `ring-drops` (the consumer fell behind) and `framing` (a marginal baud).
  The ISR runs at NVIC priority 5, above OTG_HS, so a dwc2 interrupt cannot eat that slack.
  `wifi ver` is the **eRPC link test and firmware proof** (issues #5/#20/#23): it
  round-trips a byte through `rpc_system_ack` at the link's current rate — a valid
  CRC-framed echo proves the eRPC transport end to end — and then reads the module's
  firmware build id (`rpc_system_version`), printing `fw version: …`
  (`2.1.3+wio-n7` for the firmware currently on the board). That build id is where every
  firmware-gated capability is earned, so **the L2 bridge requires a `wifi ver` first**.
  CAUTION: against pre-N2 / stock firmware the version query corrupts the module heap
  (RAM only — recoverable with `wifi reset`; see `fw/rtl8720/`). The
  eRPC path is a hand-written clean-room C client (`app/erpc.c`, FramedTransport +
  BasicCodec + CRC16/0xEF4A) with typed WiFi/tcpip wrappers (`app/wifi_rpc.c`) — no C++
  eRPC runtime.
  The link is owned by a **resident eRPC service thread** (`app/erpc.c`, priority 10,
  issue #21 increment 8): it is the only reader of the USART1 RX ring and the only
  writer of request frames, and it routes replies to whoever is waiting by **sequence
  number**, so several requests can be outstanding at once — which is what the N3
  firmware's worker pool enables. (The telnet console used to be the reason that mattered,
  parking a blocking `accept`/`recv` on the module; since issue #23 U4-2 it makes no RPCs at
  all.) It sleeps
  (touching neither the UART nor the ring) whenever nothing is in flight, so the
  `wifi log` bridge and the `wifi flash` downloader can own the same peripheral.
  Ownership of the module as a whole — the coarse mutex that serialises whole flows
  (`wifi connect` = lwIP init → off → on(STA) → associate), the reference count on
  the eRPC UART, and the lwIP lifecycle state — lives in `app/rtl_link.c`.
  Two consequences worth knowing: a command that needs the UART to itself (`wifi log` /
  `probe`, every `wifi flash <sub>`) is **refused while an eRPC session holds it**, while the
  recovery commands (`wifi on`/`off`/`reset`) deliberately are not — they take the link
  away first (abandoning in-flight calls, whose callers get a transport error), because
  "run `wifi reset`" has to work exactly when the link is stuck. And since several
  frames may be in flight, the service thread caps the request bytes it leaves
  unanswered on the wire at the module's 127-byte input ring (the eRPC asymmetry note under
  `net echo` below) — a lone frame is still sent whatever its size.
  `wifi connect <ssid> [password] [security_hex]` then actually **joins an AP**
  (issue #5): it brings up the module's lwIP stack (the factory firmware leaves it
  uninitialised at boot, and the L2 bridge is a tap on the netif that creates), switches
  to STA mode and associates. **That is all it does** — since issue #30 B1 it runs no
  DHCP and prints no address, because L3 belongs entirely to `net` on the host stack.
  It **retries** (issue #40): the radio fails intermittently — 4 of 8 associations to a
  5 GHz AP at −63 dBm failed, while 11 of 11 to the same router's 2.4 GHz SSID at −83 dBm
  succeeded — so a single failure means nothing. An operator retyping the command was
  always the retry loop; the boot configuration (#37) had nobody to do that, got one
  attempt and so appeared to be broken outright. Three attempts, bounded at 60 s by
  admitting one only when its whole worst case still fits. On a failure the module is
  asked **why** (`rpc_wifi_get_last_error`), which is the only way to tell the reasons
  apart — the SDK's `wifi_connect()` collapses all of them into one `RTW_ERROR`. Beware
  that `RTW_WRONG_PASSWORD` does **not** mean "wrong key": the SDK classifies a completed
  join by the AP's own disconnect reason, so a wrong 16-character passphrase reports
  `RTW_4WAY_HANDSHAKE_TIMEOUT` — the same code a correct one gets on a bad draw. A wrong
  passphrase therefore costs all three attempts.
  `wifi status` reports connected state, RSSI, MAC and the channel plan; `wifi disconnect`
  drops the association.
  `wifi autoreconnect [on|off]` (issue #32) makes the **host** re-associate by itself when
  the AP goes away — the module's own `wifi_set_autoreconnect()` cannot be used, because
  the handler it installs runs the module's DHCP and then AutoIP, and any non-zero address
  on that netif makes the WLAN driver drop host-bound unicast IP before it reaches
  `netif_rx()` (issue #31's failure mode, confirmed by disassembling `lib_arduino.a`).
  Instead the resident owner's 8-second association poll re-issues `rpc_wifi_connect`
  itself: the module's DHCP never runs, its netif address stays zero and the bridge is
  untouched. It fires on a **definite** "not associated" only — a module that did not
  answer is a transport failure, not a lost AP — and it fires *before* link-down is
  declared (that needs two consecutive samples), so a re-association inside one refresh
  means NetX never sees the link drop and no socket, telnet console included, is torn
  down. It is **on by default**, which means an ordinary `wifi connect` leaves its SSID and
  passphrase in host RAM — a real exposure, since `devmem` and the telnet console can read
  it. That was an opt-in when the feature landed, and is a default now because the
  preference could only ever be set *before* the outage it protects against. `off` opts out
  and wipes what is held; so do `wifi on|off|reset`, `wifi disconnect` and any flash
  session. Nothing is persisted, so a reset comes back on. Failures back off 8 → 60 s. The address is not re-acquired automatically — `net info` flags the lease as
  possibly stale and `net dhcp` is still the operator's call.
  `wifi chplan [set <hex> confirm]` reads — and, spelled out in full, installs — the
  module's **channel plan**, which is its regulatory domain: the value decides which
  channels the radio may use and actively probe. Writing it is **provisioning, not
  configuration**: the plan survives a full board power-down (measured), so it is written
  once per module and never on a boot path, hence the `confirm` token that `wifi flash
  write` also uses. Board #2 ships as `0x7f` (`RT_CHANNEL_DOMAIN_REALTEK_DEFINE`) because
  nothing ever sets one — the module SDK's `wifi_set_country_code()` is a weak function
  with both of its calls commented out. The module exposes two setters and **only one
  works**: `rpc_wifi_change_channel_plan` returns `RTW_SUCCESS` and changes nothing, so
  the read-back after a write is not a formality. Whether a plan can be freely rewritten
  is unresolved (issue #42) — recovering `0x25` → `0x7f` only ever *set* bits, which a
  write-once fuse array could also do.
  `wifi scan` lists the visible APs — channel, band, RSSI, security mode, BSSID and SSID
  (the module is dual-band). The **band is derived from the channel number**, because the
  scan record's own `band` field came back 0 (`RTW_802_11_BAND_5GHZ`) for every result on
  hardware, 2.4 GHz APs included — on this firmware 0 evidently means "unset". That is an
  observation, not something the sources show: the field is filled in (or not) inside
  Realtek's prebuilt `libameba` scan path. The module's scan is asynchronous, so the shell starts
  it, polls `rpc_wifi_is_scaning` until it clears, then reads the count **before** the
  records: the firmware's `wifi_scan_get_ap_records()` copies as many records as you ask
  for regardless of how many were actually found, so asking for more than the reported
  count returns the previous scan's leftovers. Scanning needs an STA interface and boot
  leaves the radio in `RTW_MODE_NONE`, so `scan` cycles `off → on(STA)` first — but only
  when *not* associated, since that cycle would drop a working link. SSIDs are
  attacker-controlled bytes from the air, so they are printed as **printable ASCII only**
  (anything else becomes `?`, with the raw bytes dumped as hex on the next line) — that
  rules out escape/control sequences reaching the terminal without needing to trust a
  UTF-8 decoder. connect/DHCP block on the
  module for seconds, so those calls carry a long timeout and a Ctrl+C abort hook; the
  eRPC subcommands take the console (`cli_console_claim`) for single-owner access to
  the SPSC RX ring. Register-only on the STM32 side (GPIO + UART clock gates, baud from
  the inherited PCLK2 = 137.5 MHz) — it never touches the RCC clock tree.
  The `wifi flash <sub>` subtree is the **on-device RTL8720DN flasher** (issue #19):
  the STM32 alone drives the module's mask-ROM **UART download mode** and speaks the
  AmebaD download protocol itself, so its firmware can be replaced without a host PC or
  any soldering. Entry is a strap: the ROM samples `PA[7]` (= the module's `UART_LOG_TXD`,
  wired to our **PD14**) at reset, so PD14 is held low across the `CHIP_EN` (PC3) rising
  edge — a hold of **≥ 20 ms** is required — and the protocol then runs on the **LOG UART
  (UART9)**, raised to 1.5 Mbaud (entry and the flashloader-stub upload live in the
  protocol layer, `app/rtl8720_flash.c` — the M1/M2 bring-up commands that exposed them
  were retired by issue #28).
  `wifi flash read <off> [n]` surveys sectors (erased vs data); `wifi flash info` reports
  the **real chip capacity** — measured by address wrap, comparing 8 KB at offset 0
  against 8 KB at 1/2/4/8 MB — plus the status registers and a device-side checksum;
  `wifi flash backup [off] [len]` streams the whole chip to the PC over **YMODEM**
  (`svc/ymodem.c`; receive with `rb`, or `Ctrl+A Ctrl+R` in picocom) so the factory image
  can be saved before anything is written. All of those are **read-only**.
  Going the other way, `wifi flash imgload` **receives** an image from the PC (`sb <file>`) into
  the PSRAM staging buffer at `0x90000000` (`app/rtl8720_img.c`; AXI-SRAM is far too small
  for a 2 MB image) and `wifi flash imginfo` re-verifies it against PSRAM. Those are read-only
  too — they never power the module up. Use **`sb -k`**: plain `sb` sends 128-byte blocks and spends
  most of the transfer waiting for per-block ACKs (~42 kB/s for 2 MB), while `-k` selects
  1024-byte blocks. A transfer's post-mortem (blocks accepted, CRC/sequence/short-read
  counts, console RX-ring drops) goes to **`dmesg`** as well as the console, because while
  `sb`/`rb` is running it owns the terminal and eats anything the board prints.
  There is exactly one destructive entry point: `wifi flash write <off> confirm`
  programs the staged image for real — including the module's boot sectors — and is what
  makes the saved factory image restorable. (M3's `wifi flashtest`, a hard-gated
  erase/write/verify self-test on a single unused sector, was retired in issue #28: every
  real write has exercised that path since.) It power-cycles the module mid-operation,
  because the flashloader stops responding after a flash program, and uses that
  second session to verify by asking the module for its own checksum of the written range
  (a **sum of 32-bit little-endian words**, identified on hardware) and comparing it with
  ours, so a multi-megabyte image needs no read-back. Its gates: 4 KB alignment, a 2 MB
  destructive cap, the chip capacity re-measured before the erase, the staged image's
  digest re-checked against PSRAM, an explicit `confirm`, and — at offset 0 — the AmebaD
  `km0_boot` magic, so a non-firmware blob can never land on the boot sector. The
  RTL8720DN is a separate chip from the STM32 and its download mode lives in mask ROM, so
  it stays recoverable regardless.
- **`net`** — the IPv4 (L3) layer on top of a `wifi connect` association (issue #5),
  the Wio counterpart of `../stm32f746g-disco`'s `net` command. Since **issue #30 B1**
  there is exactly **one L3 backend**: **NetX Duo on this MCU**, over the issue-#23 L2
  bridge (below). The module's own lwIP still exists — the bridge is a tap on
  its netif, so it has to — but nothing drives it any more, and the L2 side (power +
  association) stays in `wifi`. The module-side socket offload it replaced was retired in
  two steps: TCP in #23 U4, then the datagram side, the module raw-ICMP ping and the
  `net conc` probe in #28/#30; their wire-level record is in this repo's history and in
  `fw/rtl8720/README.md`.
  `net info` shows association + MAC (asked of the WiFi **driver**, which the tap does not
  touch) plus the host stack's IP/mask/gw and dhcp-vs-static; `net ip <a.b.c.d/mask> [gw]`
  sets a static address; `net dhcp` (re)acquires a lease; `net ping <a.b.c.d> [count]`
  sends **real ICMP echoes** built by NetX Duo — no eRPC round trip in the measured RTT.
  All of them need the interface up, which `wifi connect` does. Recovering from an air
  outage is an **L2** act (`wifi autoreconnect`, issue #32): it restores the association,
  not the address. A lease taken before the outage may belong to a network we are no
  longer on, so `net info` marks it stale and leaves the re-acquisition to `net dhcp` —
  running a DHCP client from the owner thread would close the lock-order cycle documented
  at `nxn_link_status_cb()`.
  `net echo [port]` (default 2323, Ctrl+C stops) runs a **TCP echo server on the host
  stack** (issue #23 U4-1, `app/nx_echo.c`) — it needs the interface up, and it is the bring-up
  rehearsal the telnet console follows. There is no RPC in its data path: bytes arrive as
  raw Ethernet frames and NetX Duo reassembles them here. It reports throughput, the
  packet-pool low-water mark, the link's own loss ledger and why the session ended, to the
  console **and to the log** — Ctrl+C is the only way to stop it and the core discards a
  cancelled handler's output, so a report printed only to the console is a report you never
  see. Measured on board #2: 512 kB echoed losslessly at 305 kB/s over a 2 Mbaud link and
  391–505 kB/s at 6 Mbaud, with every host- and module-side loss counter at zero.

  It also taught the increment two things. **`nx_tcp_server_socket_accept()` may not be
  re-entered**: every entry from LISTEN regenerates the ISN, and the timeout path restores
  LISTEN without unbinding, so a SYN arriving in between leaves a bound-but-LISTEN socket
  whose completing ACK NetX drops for ever (its state machine has no LISTEN case). The
  passive open is therefore armed exactly once with `NX_NO_WAIT` and the waiting is done by
  `nx_tcp_socket_state_wait()`, which touches nothing. And the eRPC version it replaced had
  established the constraint that shaped issues #21–#23: **eRPC is asymmetric — a big reply
  is fine, a big request is not.**
- **The host's own TCP/IP stack** (issue #23 U3, `app/nx_net.c` + `app/nx_link_driver.c`;
  armed by `wifi connect` since #30 B2). Arming turns the module into the L2 relay
  U2 built and brings **Eclipse NetX Duo up on the STM32**: our own ARP cache, our own
  DHCP client, our own ICMP. Everything with an address in it requires it (one predicate,
  `nx_net_is_up()`, so no subcommand can disagree with its neighbour about who answered).
  **Issue #30 B2 will fold the arm into `wifi connect`**, at which point these two stop
  being user commands and the bridge is simply always there.

  NetX Duo's "Ethernet MAC" here is the link's DATA channel: a transmit is a 14-byte
  header prepended in place and one `link_data_send()`, a receive is a copy into an
  `NX_PACKET` and `_nx_ip_packet_deferred_receive()`. The 32 kB packet pool lives in
  **DTCM** — the only bus master that ever touches a frame is the CPU, so it costs
  nothing from AXI-SRAM and evicts nothing from the D-cache.

  A resident owner thread holds the link, keeps the module's bridge watchdog fed every
  8 s, and accumulates the module's DATA counters **before** each re-arm (a re-arm zeroes
  them). The teardown obeys one rule: **once the bridge has been asked for, only the full
  stop may end the session** — `CFG(OFF)` → both ends proved quiet → detach. If that
  cannot be proved the interface goes to `FAILED` and says `wifi reset`, because
  detaching early would re-arm a stale-byte flush in the middle of a frame.

  Arming also **raises the link to 6 Mbaud** (issue #23 U4-3). Every module boots at
  2 Mbaud — that is its firmware — so the rate can only be reached by asking, once per
  module boot, and associating is the point at which throughput starts to matter
  (measured: 305 vs 500 kB/s of echoed TCP). This deliberately reverses U0-3's "the rate
  change is exclusive and manual"; what changed is the evidence, not the risk analysis. A
  module that refuses gets a warning and the bridge continues at 2 Mbaud.

  While the bridge is up the module's own lwIP receives nothing. Since **#30 B2** that no
  longer means refusing the L2 commands: `wifi connect`/`disconnect`/`scan` hold the
  module's bridge watchdog open across their long eRPC flows instead (and refuse only if
  that cannot be arranged, because starting anyway would drop the bridge mid-flow). The
  `wifi link` benches still need it down — they hold the coarse mutex for seconds and
  `dbench` wants the DATA channel, whose only consumer is the NetX driver — so they run
  before associating, and `wifi reset` is the way back to that state. `wifi on/off/reset`
  stay allowed — they are the recovery path, and the owner notices the link being taken
  away and stands down cleanly. **The device firmware is unchanged** (`2.1.3+wio-n7`).

  **Since #41 the owner is told, rather than left to notice.** It always could — it
  compares the link's uart generation before every refresh — but only when it next woke,
  which is up to `NXN_REFRESH_MS` (8 s) later. Anything typed inside that window saw a
  stale `up` and `wifi connect` refused, because renewing the L2 bridge means asking a
  module that was power-cycled a second ago; with a repro taking 1–2 s, that was about
  half the time. `wifi on/off/reset` now call `nx_net_link_taken()` right after the
  force-quiesce and before CHIP_EN moves. It is **not** `nx_net_down()`: that is the
  graceful stop, which takes the coarse mutex for up to 20 s and speaks eRPC to prove
  the module went quiet. `wifi flash` can afford that (it may refuse and say so);
  the recovery commands cannot, and a teardown that talks to the module is the wrong
  shape for one that is about to lose power anyway. **Two teardowns, on purpose.**

- **`net shell` — the telnet console** (issue #23 U4-2, `app/net_shell.c`).
  `telnet <board-ip>` gets **the same shell as USB CDC, at the same time**: a second
  `cli_instance` (prompt `wio-net> `) on a **NetX Duo TCP socket on this MCU**. It
  therefore **requires the interface up**, and **arms itself when it takes an address**
  (`net dhcp` / `net ip`); `net shell start [port]` / `stop` / `status` are the manual
  controls. Ported from `../stm32f746g-disco`'s `port/netxduo/nx_shell.c` — same transport
  vtable, `connected` write gate, telnet IAC handling and single-session instance reuse.

  Until U4-2 it rode the module's lwIP over eRPC, and everything about it was shaped by
  that: one blocking module call at a time (the firmware has two workers and ours held
  one), no aborting an accept or a receive, a "dirty" latch and deliberately leaked file
  descriptors when a call did not come back, and output chunked to the module's UART ring.
  **None of that exists now** — the stack is on this side of the link. What replaced it:

  - **Three threads, strictly divided.** A server thread (priority 14) is the only caller
    of create/listen/accept/relisten/disconnect/unaccept/unlisten/delete — which is what
    lets the teardown prove what it proves. The NetX callbacks on the IP thread only push
    bytes into a ring and set flags. The CLI threads appear solely inside `write()`.
  - **No transmit ring and no polling.** Output goes straight into an `NX_PACKET`, and
    back-pressure is TCP's own: a refused send makes `write()` return short, the core waits
    on `CLI_EVT_TX`, and NetX's window-update / queue-depth notifications resume it. The
    server waits on `nx_tcp_socket_establish_notify()` rather than polling for a connection
    — `nx_tcp_socket_state_wait()` is a `tx_thread_sleep(1)` loop, which a command can
    afford and a resident service cannot. The board idles at 98.9 % with the console up.
  - **Teardown is a proof, not an attempt.** The unwind cannot drop the interface under a
    live socket: detaching the DATA consumer restores the link service thread's stale-byte
    flush, and those bytes are the middle of a frame if anything can still transmit. So
    `net_shell_stop_sync()` returns success **only when `nx_tcp_socket_delete()` returned
    `NX_SUCCESS`** — the socket gone, not merely idle — and the interface takes its `FAILED`
    path if that cannot be shown. On hardware the log reads `nshell: stopped (requested)`
    two milliseconds before `host stack down`.

  Self-destruct avoidance only — there is no command policy. The telnet console refuses
  what would destroy the transport it runs on: `wifi on/off/reset` (CHIP_EN), any YMODEM
  transfer (`xfer`, `wifi flash imgload`), `wifi connect`/`disconnect`, and `net shell stop`. The
  issue-#21 restriction on module-side blocking calls is **gone** — the console no
  longer occupies a module worker.

  `net shell status` separates the three reasons a write can be refused, because they mean
  opposite things: `tx waits` is the designed back-pressure (a 64 kB `dmesg` over telnet
  produces thousands and that is health), `no-buf` means output was competing with the
  receive path for packets, and `busy` means the teardown held the socket. It is emitted to
  the log as well as the console — the console's own status is most wanted exactly when
  that console is in trouble.

  Known limits: `printf()` follows the console of the thread that ran the command —
  `_write` hands a non-CDC instance's output to that instance's transport — which is what
  puts the CoreMark report (`ee_printf` → `printf`) on the telnet session that started it.
  Because two interactive instances share priority 16, `cli_start()` gives them a **time
  slice** (`CLI_INSTANCE_TIME_SLICE`); otherwise `coremark` on one console would freeze the
  other for its whole run, and the flip side is that benchmark scores drop while both are
  busy.

- **`wifi link` — the UART link itself** (issue #23, `shell/cmds/cmd_wifi_link.c`). Where
  `wifi` is L2 and `net` is L3, this is the wire between the STM32 and the companion chip:
  the numbers here are what proved the L2-bypass road before the host stack shipped, and they
  remain the link's health monitors and regression witnesses.
  `wifi link info` prints **both ends'** counters — the host's RX interrupt budget and error
  tally, and the module's own (interrupt count, ring peak, the three loss counters, free
  heap, current rate, and **how much of its FIFO grace one interrupt consumed**). Reading
  the module's side while traffic flows was impossible before: they go to the LOG UART,
  which cannot be attached while the eRPC link is held. Note `entry` and `burst` are
  different questions — `entry` is FIFO occupancy when the interrupt *started*, which is
  the real margin; `burst` also counts bytes that landed during the drain, and those are
  free.
  `wifi link baud <2000000|3000000|4000000|6000000>` changes the line rate of the link.
  `wifi link bench [bytes] [secs] [rx|tx|both|all]` (default 1500 B / 3 s / both;
  `all` runs the three directions back to back)
  generates measured traffic and reports throughput plus the round-trip distribution
  (min / avg / p99 / max) — the **max** is the number that matters for carrying frames.
  All of it rides a **LINK-CTRL channel** (`u16 0xFFFF | u16 len | u16 crc | body`)
  multiplexed onto the same wire and owned by the link layer at both ends, so none of it
  required touching the generated eRPC server shim. It needs module firmware
  `2.1.3+wio-n5` (`fw/rtl8720/patches/0005`) and, because the host only learns which
  firmware is loaded from `wifi ver`, every subcommand asks you to run that first.
  CTRL is only issued on a **quiescent** link, so `wifi link` refuses while anything else
  holds the eRPC UART, and while the host stack is up.
  `wifi link baud` is the one command whose failure costs the link: the module acknowledges on
  the *old* rate and only then switches, so a lost acknowledgement changes nothing, but if
  the *new* rate does not work the host's attempt to put both ends back is best effort
  (the message telling the module to return is sent at that same bad rate). The sequence
  itself lives in `rtl_link_set_rate()` (`app/rtl_link.c`) because issue #23 U4-3 made
  arming raises the rate too, and two implementations of something this delicate would
  eventually disagree. **`wifi reset`
  is the guaranteed recovery** — the module boots at 2 Mbaud and the host resets its own
  belief to match.

- **The DATA channel** (issue #23 U1, `app/link_data.{c,h}` + `fw/rtl8720/patches/0006`).
  A third frame type on the same wire — `u16 0xFFFE | u16 len | u16 crc | body`, body =
  `u8 chan | u8 flags | payload` — and unlike eRPC and CTRL it is **unsolicited,
  bidirectional and unacknowledged**, which is what a raw Ethernet frame needs. U2 will
  hang the module's WiFi netif off it; U1 builds the plumbing and measures it against a
  sink/source bench on the module (`wifi link dbench [bytes] [secs] [rx|tx|both]`). Both magic
  numbers are safe for the same reason, and it is about the SENDER, not the receiver: an
  eRPC frame's leading u16 is its message size, and neither end can produce one that large
  (host ≤ 8 + 320 B, module ≤ its 4096-byte buffer).
  **Measured on board #2: 875 KB/s full duplex at 6 Mbaud, with the transmit direction at
  100 % of the wire rate** (602.7 kB/s against a theoretical 597.8) and zero loss at every
  rate. The 366 KB/s the U0-3 CTRL bench reported was never a property of the wire — it was
  the half-duplex turnaround of a request/reply exchange, and removing the reply removes it.
  Three things had to change around it:
  **(1)** the host's UART transmit became **interrupt-driven** (`app/rtl8720.c`, TX ring +
  `CR3.TXFTIE`) — the polling spin it replaces held the link service thread for 2.5 ms per
  1500-byte frame at 6 Mbaud, which starves the shell and the telnet console once frames
  flow continuously. Anything that tears the peripheral down now flushes first
  (`rtl8720_uart_flush()`, called from `rtl8720_uart_close()`, which every baud change and
  every recovery path already goes through).
  **(2)** the pre-request **RX flush is suppressed while the channel is live** — those
  "stale" bytes are then most likely the middle of a frame arriving right now. The
  matching rule is that the channel is only detached once **both** ends are known quiet:
  `LINK_DATA_CFG(off)` does not answer until the module has stopped its source and drained
  its transmit queue, which is a promise no amount of waiting on the host side could
  otherwise establish.
  **(3)** `erpc_crc16()` is now table-driven (same polynomial, same values) — the
  bit-at-a-time loop was ~110 µs per 1500-byte frame on the link service thread.

- **The L2 bridge** (issue #23 U2, `fw/rtl8720/patches/0007`; permanent since #30 B2).
  The DATA channel stops carrying a bench pattern and starts carrying **real Ethernet
  frames**: the module hands what it receives from the air to the host instead of to its own
  lwIP, and frames sent on `LINK_DATA_CHAN_ETH` go out over the air — which is what the
  host stack above runs on. U2 proved it with a transient `wifi link arp` session whose
  **`is-at` reply** showed the whole path in both directions at once: the host built a
  frame, the module transmitted it with its own MAC, a real machine answered, the driver
  accepted the answer, the tap caught it before lwIP, and it survived the link. That
  command went in **#30 B2c** — with the bridge permanent, `net dhcp` (DISCOVER out, OFFER
  back) and `net ping` (NetX resolving the gateway by ARP itself) prove the same round
  trip, and a transient session cannot coexist with the resident one anyway: the DATA
  channel has exactly one consumer.
  It is a **tap, not a rewrite**: `LwIP_Init()` still runs and the bridge swaps
  `xnetif[0].input`, so switching it off leaves the module's own stack — and therefore
  `net info`/`ip`/`dhcp` — intact. Two things had to be got right, and
  neither is guessable: the WLAN driver **filters received IP packets against the netif's
  address** (so a bridge session zeroes it and restores it), and this lwIP is `NO_SYS=0` with
  `LWIP_TCPIP_CORE_LOCKING=0`, so both mutations run **on the tcpip thread** and the host stops
  the DHCP client first. The session is foreground and bounded, and the module runs its own
  watchdog over it, because while the tap is in the module's lwIP is deaf; it says to run
  `net dhcp` afterwards.

## Key design points

- **Never reprograms the clock tree.** The DFU bootloader sets HSE 25 MHz → PLL1
  → 550 MHz CPU, PLL2R 266 MHz (the OCTOSPI kernel clock the PSRAM runs on), PLL3Q
  → 48 MHz USB and `FLASH_ACR` latency 3 / WRHIGHFREQ 3 before jumping here.
  Reprogramming the RCC (as stock CMSIS `SystemInit` / `HAL_Init` /
  `SystemClock_Config` do) would drop the core to HSI while the flash latency stays
  configured for 550 MHz and kill the USB clock.
  `src/system_stm32h7xx.c` is a custom clock-free `SystemInit` (FPU + VTOR + the
  ITCM load only, with VTOR taken from the linker's `g_pfnVectors`);
  the ThreadX SysTick reload is computed from the inherited `SystemCoreClock`.
  Inheriting still means *reporting* the tree correctly, and that has one trap:
  `D1CorePrescTable` in the same file is the table ST's HAL indexes for **both** the
  4-bit `HPRE`/`D1CPRE` fields and the 3-bit APB prescalers, whose encodings disagree
  over indices 4–7. Writing it out for `HPRE` alone — which this file did until issue
  #8 — leaves every `HAL_RCC_GetPCLKxFreq()` returning HCLK unshifted, so a 137.5 MHz
  APB4 read back as 275 MHz. It carries ST's values now.
- **The one exception: the LTDC pixel clock** (issue #7, `port/ltdc/ltdc_display.c`).
  RM0468 Figure 55 wires `ltdc_ker_ck` straight to `pll3_r_ck` — no kernel-clock mux —
  and sec 8.7.16 makes `DIVR3` writable **only with PLL3 stopped** (`PLL3ON = 0 &&
  PLL3RDY = 0`). PLL3 is also the 48 MHz behind the USB console, so retuning the
  display clock necessarily stops the console's clock for the duration. Two rules make
  that safe, and both are load-bearing:
  - `ltdc_clock_init()` runs in `main()` **before `usb_hw_init()`**, while nothing has
    enumerated and OTG_HS is not even clocked, so the few-tens-of-µs outage is
    unobservable. (Note `usb_hw_init()` comes *before* `psram_hw_init()`, so "next to
    the PSRAM bring-up" would already be too late.)
  - It does **not** use `HAL_RCCEx_PeriphCLKConfig(RCC_PERIPHCLK_LTDC)`. That HAL path
    stops PLL3 and then rewrites M/N/P/Q/R *from the caller's struct*, which would mean
    hardcoding the bootloader's values here — and any divergence breaks USB for good.
    Instead it is a bare read-modify-write that touches the `DIVR3` field and `DIVR3EN`
    only; every other bit of `RCC_PLL3DIVR` is written back exactly as read, so
    `DIVQ3EN` and the VCO cannot drift from whatever the bootloader chose. The register
    field is the divide ratio **minus one** (`DIVR3 = 0` means /1), which the API hides:
    everything above speaks the ratio, default 40 → 240/40 = **6.00 MHz**.

  It follows that the pixel clock cannot be retuned at run time at all — there is no
  safe moment to stop PLL3 once the console is up — so the divide ratio is a build-time
  constant (40 → 6.00 MHz, the rate the factory firmware used).
- **Both L1 caches on** (`SCB_EnableICache` + `SCB_EnableDCache` in `app/main.c`).
  Ordinary app memory needs no mitigation: the app is single-CPU, so one D-cache is
  self-coherent across threads and ISRs, and USB dwc2 is slave/FIFO (CPU ↔ FIFO by
  MMIO, no system-memory DMA). The reset-persistent log is in **DTCM, which bypasses
  the D-cache**. Since issue #6 there *is* one bus master — the **SDMMC1 IDMA** — and
  the two mitigations are chosen per buffer rather than globally:
  - the **PSRAM window is MPU Normal non-cacheable** (`app/mpu.c`, configured between
    I- and D-cache enable), so a large or long-lived DMA buffer (a camera framebuffer)
    placed there is coherent with no per-transfer work;
  - the SD driver instead keeps a **4 KB bounce buffer in ordinary cacheable AXI-SRAM**
    (`.axi_dma`) and cleans/invalidates around each transfer — a small scratch buffer
    does not justify one of the 16 MPU regions, and the caller's buffer never reaches
    the DMA, so callers need no alignment discipline.
- **microSD (issue #6)**: `port/sd/sd_card.c` drives the slot on **SDMMC1** through the
  controller's **own IDMA** — there is no DMA stream and no `__HAL_LINKDMA`, so a single
  `SDMMC1_IRQHandler` carries every completion. The kernel clock is muxed to the
  inherited **PLL1Q 110 MHz** (`RCC_D1CCIPR.SDMMCSEL = 0`, which is the register's reset
  value — this asserts the default rather than reconfiguring anything; PLL2R 266 MHz is
  both over the mux's 250 MHz ceiling and the PSRAM's clock), giving
  `SDMMC_CK = 110/(2×3) = 18.33 MHz`, inside the 25 MHz Default Speed limit. High Speed
  via CMD6 is a separate, measured change. **No card-detect line exists on this board**,
  so removal is detected from the bus: a command-response timeout surfaces *only* in
  `hsd.ErrorCode` (the H7 HAL's `HAL_SD_GetCardState()` returns a state of 0 rather than
  an error), and one `classify_err()` turns that into `no card` on every path —
  identification, bus-width setup, both DMA submissions and the completion callback.
  Build `-DBSP_ENABLE_SD=OFF` to compile the driver, FileX and the command out
  entirely, which also removes the only DMA engine the app enables — useful when
  bisecting a suspected D-cache coherency problem.

  Having no card-detect pin also reshapes the **mount** path, and not in the obvious
  way. The f746 code this was ported from checks its CD pin immediately before
  `fx_media_open()` and returns "no card" early. The nearest thing this board has is
  a *cached* "have we successfully talked to a card" flag — and gating the mount on it
  would deadlock by construction, because the probe that would set the flag happens
  inside the driver's `FX_DRIVER_INIT`, which only that same `fx_media_open()` reaches.
  A cold boot with a perfectly good card would report "no card" forever. So there is
  no pre-check: the probe decides. Since `fx_media_open()` flattens every INIT failure
  into `FX_IO_ERROR`, the driver records its probe result and the glue reads it back
  afterwards to tell an empty slot from a card with no FAT on it.
- **PSRAM (issues #3 / #16)**: `app/psram.c` brings up the **APS6408L Octal DDR PSRAM
  on OCTOSPI1** (memory-mapped 8 MB @ `0x90000000`) without touching OCTOSPI2/OCTOSPIM/
  the RCC, so it cannot disturb the inherited clock tree. Shipped operating point: **133 MHz Fixed Latency** (kernel
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
- **External NOR (issue #37)**: `port/nor/nor_flash.c` brings up the **W25Q128JV 16 MB
  serial NOR on OCTOSPI2** — the flash the app used to execute from, unused since
  issue #25 — as ordinary storage. It is deliberately **indirect-only**: a
  memory-mapped window at `0x70000000` would be Normal memory that the core may read
  *speculatively*, and a speculative read arriving while the controller is out of
  memory-mapped mode is a slave error (RM0468 §25.4.16) that no lock can prevent,
  because the CPU never asked for the access. So the MPU keeps its no-access + XN fence
  over that quarter and every byte moves through explicit transactions — which also
  means no DMA, no cache maintenance and no mmap⇄indirect transition to get right.
  Three details are not guesses: **`CR.FSEL = 1`** because this flash is bonded to Port
  2's *high* nibble and that bit chooses which nibble carries the transfer (RM0468
  §25.7.1, matching `OCTOSPIM_P2CR`'s reset `IOHSRC = 0b11`) — with it clear the
  transaction still completes and reads back zeros, which is how it first failed here;
  **Fast Read `0x0B`** rather than `0x03`, since the datasheet caps plain Read Data at
  50 MHz while this bus runs at 88.7 MHz; and **`DCR1.CSHT = 5`** (≈68 ns) because
  erase/program need tSHSL2 ≥ 50 ns between deassertions, where the bootloader's
  read-only capture only ever needed 10 ns. Erase and program wait by *sleeping*
  between status polls, so a 400 ms sector erase never starves the IWDG petter. On this
  device a partial page program leaves the rest of the page at `0xFF` **and** a
  same-byte overwrite (`0xFF`→`0xFE`→`0xFC`) is accepted — both measured by `nor test`.
- **Configuration store (issue #37)**: `app/kv.c` runs a **FlashDB KVDB** (Apache-2.0,
  pinned to 2.2.0) in the NOR's first megabyte, so a setting is readable at every
  boot — before any filesystem, on a board whose microSD slot may be empty and has
  no card-detect line to say so. Three decisions carry it:
  - **`FDB_WRITE_GRAN = 8`.** FlashDB marks record state by writing a status field;
    at granularity 1 it re-programs the *same byte* (`0xFF`→`0x7F`→`0x3F`), at 8 it
    gives each status its *own* byte. The W25Q128JV specifies programming "at
    previously erased (FFh) memory locations" (§8.2.13) — a constraint on the state
    of the target bytes, not the history of the page — so granularity 8 stays inside
    what the datasheet promises for 4 extra bytes per record. `nor test` measured
    this device accepting same-byte overwrites too, but "probably, on this chip,
    today" is not what a configuration store should rest on.
  - **It never formats on its own.** Initialisation runs with
    `FDB_KVDB_CTRL_SET_NOT_FORMAT`, and only a *second* attempt — reached solely when
    a full scan shows the partition is erased — is allowed to create a database.
    Anything else is reported as corrupt and left for `kv format yes`. This paid for
    itself immediately: the first boot found the old XIP application image still
    sitting in that partition and refused it instead of erasing the evidence.
  - **The type and the description live on the external flash**, not in a table
    compiled into the firmware. What FlashDB stores under a key is a 12-byte header
    (magic, format version, type, description length, total length) followed by the
    description and the value — so adding a setting never means reflashing, and the
    internal flash holds only the pack/unpack. Records are packed byte by byte in
    little-endian order rather than by overlaying a struct, which would make the
    on-flash layout depend on the compiler's padding for a format meant to outlive
    builds. Decoding checks its fields **in a fixed order** — header present, magic
    and version and type known, `total_len` equal to the real length, `desc_len`
    within the remainder, and only then the value length derived — because every
    length is unsigned, so a subtraction taken out of turn wraps to something
    enormous instead of failing, and that is precisely how a corrupt record becomes
    an out-of-bounds read. A record that fails any of these is *shown* as
    unreadable, never skipped: a listing that hid what it could not decode would
    make a half-corrupt store look healthy.
  - **One lock around whole operations.** FlashDB locks inside
    `fdb_kv_set_blob()`/`fdb_kv_get_blob()` but *not* inside its iterator or
    `fdb_blob_read()`, so `app/kv.c` takes the NOR device mutex for the duration of
    every entry point; the mutex is recursive, so FlashDB re-taking it costs nothing.
    `fdb_kv_get()` is never called (it returns a pointer into a `static` buffer), and
    neither is `fdb_kv_get_blob()` — it reports a failed value read as a successful
    one, so reads go through `fdb_kv_get_obj()` + `fdb_blob_read()` instead.

  **`wifi.psk` is stored in the clear.** Encrypting it would need somewhere to keep
  a key, and this board has nowhere: the option bytes and RDP are deliberately never
  touched (see *Brick safety*), and the external flash is readable over SWD or with a
  hot-air gun regardless. `kv list` masks the value so it does not end up in a pasted
  log, which is shoulder-surfing hygiene and not protection — do not treat it as such.
- **LCD (issue #7)**: `port/ltdc/ltdc_display.c` drives the FPC-40 RGB panel. All 28
  LTDC signals are **AF14 except `LTDC_R3` on PA15, which is AF9** — a detail taken
  from ST's own machine-readable pin data (`_ref/stm32_open_pin_data/`, cross-checked
  ball-by-ball against the schematic), not guessed: on this package LTDC appears on
  seven different alternate functions. PA15 is JTDI, which costs nothing because debug
  here is SWD (PA13/PA14) only.
- **The panel is not a dumb RGB panel** (`port/ltdc/st7789_rgb.c`). It carries an
  **ST7789** that boots asleep with its RGB interface disabled, and whose CS/SDA/SCL
  are **multiplexed onto LTDC_R2/R0/R1** (PA1/PH2/PH3) — which is why the FPC appears
  to have no serial pins at all. Until a 9-bit bit-banged serial sequence wakes it
  (`SLPOUT` → `RAMCTRL` = RGB interface → `RGBCTRL` = DE mode → gamma/power → `COLMOD`
  16 bpp → `DISPON`), the panel ignores every pixel and sits backlit and blank **while
  the LTDC reports a healthy scanout with zero FIFO underruns**. That combination is a
  near-perfect disguise: every register readback says the display controller is fine,
  because it is — it is driving a panel that is not listening.
  `ltdc_init()` therefore runs the sequence *between* the reset pulse and the
  alternate-function handover, while R0/R1/R2 are still plain GPIO.
  There is no datasheet for this panel. The sequence, the pin multiplexing, the
  timing (298x336 frame) and the 6.00 MHz pixel clock were all recovered by
  disassembling the board's **factory Arduino image** (`_ref/wio_APP_0x70000000_*.bin`,
  which links Seeed's `RGBLCD.cpp`) and mapping its Arduino pin numbers through Seeed's
  `ArduinoCore-stm32` variant table. When a board ships with working firmware, that
  binary *is* the missing datasheet.
  Presentation is tear-free: stage the back buffer with `HAL_LTDC_SetAddress_NoReload`,
  request a vertical-blanking reload, and commit the swap only once **`SRCR.VBR` reads
  back 0** — the reload-ready IRQ is a wake-up hint, the register is the truth. A reload
  that never lands latches the display down rather than tearing.
  One HAL behaviour had to be undone: `HAL_LTDC_Init()` enables the transfer-error and
  FIFO-underrun interrupts, and `HAL_LTDC_IRQHandler()` *clears* those flags. Left
  alone, a bandwidth-starved scanout would both storm the NVIC every line and wipe the
  very flags `lcd info` exists to report — so both are masked after init and the sticky
  ISR bits are polled instead (hardware sets them regardless of the enables).
  Build `-DBSP_ENABLE_LCD=OFF` to compile the driver, the command and the OCTOSPI1
  interlock out — useful when bisecting a suspected PLL3/USB or PSRAM-bandwidth problem.
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
  foreground and never run while asleep). The internal flash the core wakes back up
  into keeps its clock in CSleep (`AHB3LPENR.FLASHLPEN`, set out of reset and never
  cleared), so wake-path fetches resume normally. Build
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
            OCTOSPI1 PSRAM bring-up (psram.c), MPU regions (mpu.c),
            configuration store: kv.c (the only FlashDB caller) +
            kv_boot.c (the thread that opens it, issue #37),
            RTL8720DN link: erpc.c (service thread) / link_data.c (DATA channel) /
            wifi_rpc.c (typed wrappers) / rtl_link.c (ownership) /
            net_shell.c (telnet console transport + server) /
            nx_net.c + nx_link_driver.c (NetX Duo over the L2 bridge, issue #23 U3)
shell/      core/    HW-independent CLI engine (parse/edit/history/complete/...)
            include/ public CLI API + cli_config.h
            backend/ USB CDC transport (+ dummy loopback), byte rings
            cmds/    command implementations
            test/    host unit tests (run with host gcc; see below)
port/       threadx/ ThreadX low-level init + shared SysTick glue
            nor/     external W25Q128 NOR on OCTOSPI2, indirect-only, with the
                     recursive device mutex the config store shares (issue #37)
            flashdb/ FlashDB build config + its FAL device over port/nor,
                     and the log/assert hooks that keep it fail-soft (issue #37)
            sd/      microSD block driver over SDMMC1 + its internal IDMA (issue #6)
            filex/   FileX media driver on that block API + the lazy-mount
                     singleton and its ownership gates (issue #6)
            ltdc/    LTDC + DMA2D display driver: PLL3R pixel clock, double buffer
                     in PSRAM, tear-free flip, draw primitives, and the ST7789
                     serial wake-up the panel needs first (issue #7)
            camera/  OV2640 over DCMI: XCLK on TIM5_CH3, SCCB on I2C4,
                     PWDN/RESETB, sensor identification (issue #8 phase 1) and
                     QVGA RGB565 snapshot over DMA2_Stream1 (phase 2).
                     ov2640_regs.c holds ST's register table (BSD-3, see NOTICE)
            coremark/ EEMBC CoreMark port (core_portme.*)
            netxduo/ nx_user.h (NetX Duo build configuration; the driver is in app/,
                     because what it sits on is the RTL8720 link, not a MAC)
svc/        freestanding services: fmt (printf), log (DTCM ring), timebase (DWT),
            ymodem, frame_pipeline (camera frame ring + sink dispatch, issue #8
            phase 3a — ported byte-identical from the f746 firmware and covered by
            shell/test, so it is verified without the board; phase 3b wires the
            DCMI producer to it)
src/        custom clock-free SystemInit  (also the minimal `blink` example's main)
ldscript/   STM32H725AEIx_IROM.ld (FLASH @ 0x08020000, RAM = AXI-SRAM, DTCM log, ITCM ISRs)
            STM32H725AEIx_ROM.ld  (the bootloader's own script: FLASH @ 0x08000000, 128 KB)
cmake/      ARM GNU toolchain file (auto-downloads gcc into tools/)
lib/        git submodules: cmsis_core/device_h7, stm32h7xx_hal_driver, tinyusb,
            threadx, netxduo, filex, coremark
boot/       standalone USB DFU bootloader (internal 0x08000000) — see boot/README.md
fw/rtl8720/ reproducible build of the RTL8720DN's own firmware (the eRPC server that
            wifi/net drive) — host-side only, flashed by `wifi flash write`; see its README
```

## Memory map (`ldscript/STM32H725AEIx_IROM.ld`)

Listed in memory-hierarchy order — the same order `free` and `membench` print
(issue #33), which is not address order.

| Region | Address | Notes |
|---|---|---|
| ITCM | `0x00000000` | 64 KB; holds the **interrupt paths** (`.itcm`, 2,784 B): ThreadX PendSV + SysTick, the RTL8720 UART RX ISR **and the whole wake-up it signals** (`tx_event_flags_set` + `_tx_thread_system_resume`, issue #23 U0-3 — that call alone had put the ISR back to 4.3 µs warm / 12.9 µs cold — plus their two tails `_tx_thread_system_preempt_check` + `_tx_timer_system_deactivate`, issue #29), and the fault handlers, plus the 4 KB `.itcm_bench` scratch. Zero-wait-state and outside both caches, so an ISR never pays a cold fetch through the 16 KB I-cache: measured against the external-XIP build the same UART ISR went from **8.7 µs cold / 3.3 µs warm to a flat 0.7 µs** (issue #24). The tails matter because they only run when a thread is *really* woken — with them still in the flash, request/reply commands (`wifi ver`, `wifi scan`) measured 3.5 µs against 0.1 µs for an ISR that wakes nobody; 106 bytes of ITCM took that to **1.5 µs**, and `wifi link bench` improved 2.1 → 1.5 µs too. Those figures were taken while the app ran from the external flash; issue #25 shrank the gap without closing it (ITCM is still zero-wait and never evicted). Loaded from its load image by `SystemInit()`, which also zero-fills all 64 KB to initialise the TCM ECC; kept read-only by the MPU so a NULL write raises MemManage instead of corrupting ISR code. |
| DTCM | `0x20000000` | 128 KB, and since issue #46 it holds **all the CPU-only hot data**: the reset-persistent `.log_noinit` crash-log ring, the **32 KB NetX Duo packet pool** (`.nx_pool`, issue #23 U3), `membench` scratch, `.dtcm_bss` = **every ThreadX thread stack and the two RTL8720 UART rings** (48,640 B), and at the very top the **main (MSP/ISR) stack**, 8 KB ending at `_estack = 0x20020000`. All of it bypasses the D-cache — which for stacks is the point, not a side effect. The bootloader accepts a DTCM MSP: it validates the app's `vector[0]` against the on-chip RAM regions and `0x20000000` is one of them (`boot/main.c`), so `boot/` is unchanged. There is **no MPU guard page** below the main stack: ARMv7-M has no MSPLIM, so a guard faults while stacking the very exception meant to report it and locks up (PM0253 sec 2.5.1/2.5.2/2.5.5). `SystemInit()` stamps the unused stack with a pattern instead and `free` reports the high-water mark. |
| AXI-SRAM (D1) | `0x24000000` | 320 KB, **reserved for what a bus master has to reach** (issue #46): `.data`/`.bss`, the heap (up to `__ram_end`), and `.axi_dma` — the 4 KB SDMMC1 bounce buffer (issue #6) in its **own output section, 32 B-aligned at both ends**, so the driver's per-transfer clean/invalidate cannot disturb a neighbouring variable's cache line. DMA1/DMA2 and the SDMMC1 IDMA **cannot address either TCM** (RM0468 sec 2.1.2/2.1.5/2.1.6), which makes this the scarce memory: a survey of every `*_DMA()` call found only three master-touched buffers in the whole firmware, so the other 217 KB sitting here was CPU-only data occupying the one region the camera's DCMI band (issue #35) has no substitute for. Moving it out took usage from **221,024 B to 135,520 B**. 🔴 Putting a DMA buffer in DTCM does not fault — the transfer silently moves nothing — so both directions are checked after every link by `cmake/check_dtcm_residency.py`. |
| FLASH | `0x08020000` | internal flash **sectors 1-3 = 384 KB** (issue #25). Sector 0 (`0x08000000`, 128 KB) is the DFU bootloader; the 128 KB erase granule is what keeps the two apart, so no programming path can reach it. ~905 MB/s read (`membench`). The image currently uses **272,112 B = 69.2%**; see [Build](#build) for why it is built `-Os` + LTO. |
| PSRAM | `0x90000000` | external OCTOSPI1 **APS6408 8 MB Octal DDR PSRAM**, memory-mapped @ 133 MHz Fixed Latency; MPU Normal non-cacheable (DMA-coherent scratch; `.psram_noinit`). Since issue #7 the first 300 KB are the **LTDC double frame buffer**, which the display controller reads continuously while scanout is on — hence the `lcd off` interlock on every path that retunes OCTOSPI1. |
| *(bootloader)* | `0x08000000` | internal flash **sector 0, 128 KB** — the DFU bootloader. The app does not own it and no app-side path can erase it (the 128 KB erase granule is the sector granule). |
| *(external flash)* | `0x70000000` | the 16 MB W25Q128 the app used to execute from, now storage: its **first 1 MB is the `kv` configuration partition** (FlashDB) and the remaining 15 MB is reserved for the blob region issue #10 still has to design — deliberately with no FAL partition entry, so nothing can reach it by accident. **Deliberately still not mapped**, even though issue #37 brought OCTOSPI2 back up: `port/nor` reaches the device only through indirect transactions, and `app/mpu.c` keeps the 256 MB window no-access + execute-never so a stray or *speculative* access raises MemManage (recorded in `dmesg`) instead of hitting a controller that is not in memory-mapped mode. Opening a window here is a separate design with its own MPU region. |

## Build

```bash
cmake -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-toolchain.cmake
cmake --build build            # -> build/shell.{elf,bin,hex}  (also boot.* and blink.*)
```

### Optimisation: built for size, not speed (issue #39)

`shell` is compiled `-Os -flto=auto`. The app partition is 384 KB and the image had
reached **82.5%** of it with issue #9 (on-device AI) not started; this gives back
**53 KB** (324,320 → 271,228 B) *without touching the memory map, the MPU or the DFU
update path* — which every other option in issue #39 (moving the 50 KB string pool to
the external NOR, reviving XIP, feature toggles) would have had to. `-flto` alone at
`-O2` is counter-productive: cross-TU inlining makes the image **bigger**.

Speed is protected where it is actually measured, not by the `-O` level:

- The interrupt paths live in **ITCM** (issues #24/#29) and are held there by
  `cmake/check_itcm_residency.py`, below.
- **The two benchmarks keep their own flags**, because they are instruments and a
  reading has to mean the same thing across builds:
  - `coremark` — `-O3 -funroll-loops` (`coremark_obj` is a separate object library and
    is not LTO'd). The score *is* the deliverable and the flags are part of what a
    CoreMark result means. Costs about 6 KB over plain `-O3`.
  - `membench` — `-O2 -fno-lto` (`cmd_membench.c` only, 1.1 KB). Its bandwidth loops
    are the yardstick issue #3 tuned the PSRAM against and issue #25 used to justify
    moving execution to the internal flash. `-Os` disables loop unrolling, which turns
    the read loop from 73 instructions into 47 and makes it **loop-bound rather than
    memory-bound**: measured that way ITCM, DTCM and cached SRAM all report an
    identical 1453 MB/s and the internal flash reads "859 MB/s" instead of 905 —
    numbers about the benchmark, not about the memory.

Two flags disable optimisations that are legal C but wrong for this firmware. Both
cost almost nothing (1,008 B and 224 B) and both only start to matter under LTO,
because LTO propagates these inferences *across translation units* for the first time:

| flag | why |
|---|---|
| `-fno-strict-aliasing` | NetX Duo and FileX cast packet buffers to protocol headers constantly. Whole-program alias analysis is exactly the condition under which that type punning stops being harmless. |
| `-fno-delete-null-pointer-checks` | **ITCM is at `0x00000000`**, so address 0 is a real location this firmware deliberately touches — `SystemInit()` writes and reads back all 64 KB. |

For an SWD debugging session, where `-Os -flto` makes single-stepping painful:

```bash
cmake -B build-dbg -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-toolchain.cmake \
      -DBSP_OPT_LEVEL=-O2 -DBSP_ENABLE_LTO=OFF
```

### `cmake/check_itcm_residency.py` — the ITCM guard

Runs after every `shell` link and fails the build if an interrupt path left ITCM. It
exists because the **linker-script ASSERTs cannot do this job under LTO**. They are
written against symbol *names*, and LTO renames what it clones: once
`_tx_event_flags_set` has become `_tx_event_flags_set.isra.0`, `DEFINED()` reports
false, the guard's conditional collapses to `1`, and the ASSERT **passes because the
thing it guards has vanished**. Meanwhile the linker-script selector
`*(.text._tx_event_flags_set)` has stopped matching and the RTL8720 RX wake-up path is
back in the flash — with no build error. (The selectors are now written as the pair
`.text.NAME .text.NAME.*` to match clones; the ASSERTs are kept as an early warning
for the non-LTO and `blink` builds.)

So the check works on the linked image instead, on suffix-stripped base names:

1. **residency** — every symbol whose base name is a required interrupt path lies
   inside `[_sitcm, _eitcm)`. *All* matching symbols, not just one: LTO can leave both
   `foo` and `foo.isra.0` alive. A name that matches nothing is also a failure — that
   is precisely the case the ASSERT goes blind to.
2. **no leakage** — nothing inside ITCM branches out through a long-branch veneer
   other than the two accepted ones (`log_write` from the fault handler,
   `__NVIC_SystemReset` at the tail of the reset path). This catches residency loss
   for code that is not on the required list at all.

## Flash (over DFU)

```bash
# 1. Enter DFU mode: hold the USER button (PF1) and reset  (red LED lit; `dfu-util -l` enumerates)
# 2. Write the app; it auto-reboots into the shell:
dfu-util -d 0483:df11 -a 0 -D build/shell.bin
# then open the console:
picocom -b 115200 /dev/ttyACM0     # (any 8N1 terminal; baud is nominal for USB CDC)
```

`cmake --build build --target dfu-shell` runs the `dfu-util` step. Flashing the app
is **not** a brick risk: it writes sectors 1-3 only, and the bootloader's erase path
rejects any sector outside 1-3 before the HAL is called, so `0x08000000` is
unreachable from DFU. A download that is interrupted leaves the app's first flash
word erased (the bootloader writes it last, only after the whole image has arrived
and verified), so the board comes back up in DFU mode ready for another attempt.
(Re-flashing the `0x08000000` bootloader itself is a separate, higher-risk procedure
documented in [`boot/README.md`](boot/README.md).)

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
shares the clock-free `SystemInit` and the app linker script — a minimal reference
for clock-inheriting bring-up. Flash it the same way
(`dfu-util … -D build/blink.bin`).
