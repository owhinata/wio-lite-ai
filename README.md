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
Mode") with line editing, history, and Tab completion — and, once the board is on WiFi,
the same shell over **telnet** (`wio-net> `, see `net shell` below), usable at the same
time as the USB console. 22 commands:

| Group | Commands |
|---|---|
| system | `version` · `uptime` · `reboot` · `free` · `thread` |
| shell | `help` · `echo` |
| timing / jobs | `sleep` · `usleep` · `watch` · `jobs` · `kill` |
| diagnostics | `devmem` (peek/poke/dump) · `dmesg` · `crash` (bus/undef/div0) · `wdt` (info/starve) · `psram` (info/test/mmapscan/…) |
| wireless | `wifi` (L2: info/on/off/reset/log/probe/rpc · connect/status/disconnect/scan · flash*/img*) · `net` (L3: **up/down** · info/ip/dhcp/ping/conc/echo · **shell** = telnet console) · `link` (info/baud/bench/dbench/sweep/eth/arp) |
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
  RX is **FIFO-threshold driven** (issue #23): `CR3.RXFTIE` with `RXFTCFG` at half of the
  16-deep RXFIFO plus `CR1.IDLEIE` for the tail of a burst, rather than an interrupt per
  byte — at 6 Mbaud the latter would be ~600 k IRQ/s. The threshold is a latency budget:
  RM0468 puts the overrun at the 18th datum, so `18 - threshold` byte times is all the
  slack the ISR has, and `wifi rpc` / `wifi scan` print how much of it was used
  (`max N/G B per irq`) together with the three distinct losses — `ore` (the hardware
  FIFO overran), `ring-drops` (the consumer fell behind) and `framing` (a marginal baud).
  The ISR runs at NVIC priority 5, above OTG_HS, so a dwc2 interrupt cannot eat that slack.
  `wifi rpc [ver] [baud]` (default 2 Mbaud) is the **eRPC link test** (issue #5): the
  factory firmware is Seeed's eRPC image (UART @2 Mbaud on its `Serial3` = USART1),
  and this round-trips a byte through `rpc_system_ack` — a valid CRC-framed echo
  proves the eRPC transport end to end. `wifi rpc ver` additionally reads the module's
  firmware build id (`rpc_system_version`) and prints `fw version: …` (`2.1.3+wio-n3`
  for the issue-#20 N3 firmware currently on the board). `ver` is opt-in because the pre-N2 shim cannot answer
  `version` safely — it corrupts the module heap (recoverable with `wifi reset`), so
  only send it once N2 is flashed (see `fw/rtl8720/`); plain `wifi rpc` never does. The
  eRPC path is a hand-written clean-room C client (`app/erpc.c`, FramedTransport +
  BasicCodec + CRC16/0xEF4A) with typed WiFi/tcpip wrappers (`app/wifi_rpc.c`) — no C++
  eRPC runtime.
  The link is owned by a **resident eRPC service thread** (`app/erpc.c`, priority 10,
  issue #21 increment 8): it is the only reader of the USART1 RX ring and the only
  writer of request frames, and it routes replies to whoever is waiting by **sequence
  number**, so several requests can be outstanding at once — which is what the N3
  firmware's worker pool enables, and what lets the telnet console (`net shell`, below)
  keep a blocking `accept`/`recv` parked on the module while the shell runs other commands. It sleeps
  (touching neither the UART nor the ring) whenever nothing is in flight, so the
  `wifi log` bridge and the `wifi flash*` downloader can own the same peripheral.
  Ownership of the module as a whole — the coarse mutex that serialises whole flows
  (`wifi connect` = init → off → on → connect → DHCP → get_ip), the reference count on
  the eRPC UART, and the lwIP/IP-mode lifecycle state — lives in `app/rtl_link.c`.
  Two consequences worth knowing: a command that needs the UART to itself (`wifi log` /
  `probe`, every `wifi flash*`) is **refused while an eRPC session holds it**, while the
  recovery commands (`wifi on`/`off`/`reset`) deliberately are not — they take the link
  away first (abandoning in-flight calls, whose callers get a transport error), because
  "run `wifi reset`" has to work exactly when the link is stuck. And since several
  frames may be in flight, the service thread caps the request bytes it leaves
  unanswered on the wire at the module's 127-byte input ring (see the asymmetry note
  under `net echo` below) — a lone frame is still sent whatever its size.
  `wifi connect <ssid> [password] [security_hex]` then actually **joins an AP**
  (issue #5): it brings up the module's lwIP stack (the factory firmware leaves it
  uninitialised at boot), switches to STA mode, associates, runs the DHCP client and
  prints the leased `ip`/`mask`/`gw`. `wifi status` reports connected state, RSSI, IP
  config and MAC; `wifi disconnect` drops the association.
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
  The `wifi flash*` subcommands are the **on-device RTL8720DN flasher** (issue #19):
  the STM32 alone drives the module's mask-ROM **UART download mode** and speaks the
  AmebaD download protocol itself, so its firmware can be replaced without a host PC or
  any soldering. Entry is a strap: the ROM samples `PA[7]` (= the module's `UART_LOG_TXD`,
  wired to our **PD14**) at reset, so PD14 is held low across the `CHIP_EN` (PC3) rising
  edge — a hold of **≥ 20 ms** is required — and the protocol then runs on the **LOG UART
  (UART9)**, raised to 1.5 Mbaud. `wifi flashprobe` proves download-mode entry;
  `wifi flashload` uploads Realtek's flashloader stub to module SRAM and reads flash;
  `wifi flashread <off> [n]` surveys sectors (erased vs data); `wifi flashinfo` reports
  the **real chip capacity** — measured by address wrap, comparing 8 KB at offset 0
  against 8 KB at 1/2/4/8 MB — plus the status registers and a device-side checksum;
  `wifi flashbackup [off] [len]` streams the whole chip to the PC over **YMODEM**
  (`svc/ymodem.c`; receive with `rb`, or `Ctrl+A Ctrl+R` in picocom) so the factory image
  can be saved before anything is written. All of those are **read-only**.
  Going the other way, `wifi imgload` **receives** an image from the PC (`sb <file>`) into
  the PSRAM staging buffer at `0x90000000` (`app/rtl8720_img.c`; AXI-SRAM is far too small
  for a 2 MB image), `wifi imginfo` re-verifies it against PSRAM and `wifi imgsend` streams
  it back so a `cmp` on the host can prove byte equality. Those are read-only too — they
  never power the module up. Use **`sb -k`**: plain `sb` sends 128-byte blocks and spends
  most of the transfer waiting for per-block ACKs (~42 kB/s for 2 MB), while `-k` selects
  1024-byte blocks. A transfer's post-mortem (blocks accepted, CRC/sequence/short-read
  counts, console RX-ring drops) goes to **`dmesg`** as well as the console, because while
  `sb`/`rb` is running it owns the terminal and eats anything the board prints.
  There are two destructive entry points. `wifi flashtest <off> confirm` is a hard-gated
  erase/write/verify self-test restricted to a single unused 4 KB sector in
  `[0x100000, 0x200000)` that it restores to `0xFF`. `wifi flashwrite <off> confirm`
  programs the staged image for real — including the module's boot sectors — and is what
  makes the saved factory image restorable. Both power-cycle the module mid-operation,
  because the flashloader stops responding after a flash program; `flashwrite` uses that
  second session to verify by asking the module for its own checksum of the written range
  (a **sum of 32-bit little-endian words**, identified on hardware) and comparing it with
  ours, so a multi-megabyte image needs no read-back. Its gates: 4 KB alignment, a 2 MB
  destructive cap, the chip capacity re-measured before the erase, the staged image's
  digest re-checked against PSRAM, an explicit `confirm`, and — at offset 0 — the AmebaD
  `km0_boot` magic, so a non-firmware blob can never land on the boot sector. The
  RTL8720DN is a separate chip from the STM32 and its download mode lives in mask ROM, so
  it stays recoverable regardless.
- **`net`** — the IPv4 (L3) layer on top of a `wifi connect` association (issue #5),
  the Wio port of `../stm32f746g-disco`'s `net` command. Where f746 drives NetX Duo over
  the on-chip Ethernet MAC, here the backend is the RTL8720DN's **eRPC socket-offload**
  (`app/wifi_rpc.c`) — the module runs lwIP internally, so `net` is L3-only and the L2
  side (power + association) stays in `wifi`. `net info` shows link + MAC + IP/mask/gw
  (and dhcp-vs-static); `net ip <a.b.c.d/mask> [gw]` sets a static address (stops DHCP);
  `net dhcp` (re)acquires a lease; `net ping <a.b.c.d> [count]` sends **real ICMP echoes**
  over a raw socket (`rpc_lwip_socket(SOCK_RAW, IPPROTO_ICMP)`) — the shell builds the
  ICMP message + checksum itself, and reports a **host-observed RTT** (it includes the
  two eRPC UART round-trips, not just the network path). `net conc [ms]` is a diagnostic
  (issue #20 N3): it holds a `recvfrom` (no data) open on the module and round-trips a
  foreground ack while it is outstanding — a serial server (stock/N2) cannot answer the
  ack until the receive returns, the N3 worker-dispatch server answers it in a few ms.
  `net echo [port] [txchunk]` (default 2323 / 64 B, Ctrl+C stops) runs a **TCP echo
  server** on the module — socket/bind/listen/accept/recv/send over the same offload — as
  the bring-up rehearsal for the telnet shell console (issue #21): it reports accept
  latency, the firmware's 10 s accept cap, whether `MSG_DONTWAIT` works, the echo
  round-trip time and the eRPC diagnostics, which is exactly what that console's design
  needs measured. It also established a constraint nothing before it had hit: **eRPC is
  asymmetric — a big reply is fine, a big request is not.** The module's transport reads
  the UART one byte at a time behind a 127-byte Arduino ring that silently drops on
  overflow, and our side writes a frame as a gap-free polled burst, so a request larger
  than that ring loses its tail and is never answered (measured on board #2: a 264-byte
  reply arrives intact, a 280-byte request does not). The swept cliff sits between 184 and
  280 bytes rather than at 127 — the module drains while we write, so what must stay under
  the ring is the backlog during its longest mid-frame stall, which makes anything above
  127 a load-dependent race. Sends therefore chunk at `WIFI_RPC_SEND_SAFE` (96 B → a
  120-byte frame, the largest round size that still fits **entirely** inside the ring and
  so cannot be dropped however long the reader stalls) while receives keep using the full
  256 B; `txchunk` exists so the sweep stays reproducible. Two more rules
  it establishes carry over: **accept/recv must never be aborted host-side** (the module
  keeps running the call, so an aborted accept yields a socket whose fd the host never
  learns and cannot close — hence Ctrl+C is honoured only between calls, up to ~12 s),
  and a host-side timeout leaves the link **dirty**, in which case the sockets are
  deliberately not closed (a close racing an in-flight accept/recv is unsafe on this
  firmware) and `wifi reset` reclaims them.
  ip/dhcp/ping/conc/echo require an active association (they never power the module) and
  share the same `app/rtl_link.c` session (console claim + coarse mutex + eRPC UART
  reference) as `wifi`. Pure marshalling on the STM32 side — no RCC/register work.

  The RTL8720 device firmware itself is rebuilt (non-blocking socket handlers, then a
  worker-dispatch eRPC server so multiple requests run at once) under `fw/rtl8720/`
  (issue #20); the STM32 client keeps several requests in flight over the one link with
  `app/erpc.c`'s `erpc_begin`/`erpc_wait`/`erpc_cancel`.

- **`net up` / `net down` — the host's own TCP/IP stack** (issue #23 U3,
  `app/nx_net.c` + `app/nx_link_driver.c`). `net up` turns the module into the L2 relay
  U2 built and brings **Eclipse NetX Duo up on the STM32**: our own ARP cache, our own
  DHCP client, our own ICMP. `net` keeps its command names and picks the backend from
  **one predicate**, so `net info` prints `stack: host (NetX Duo, module bridged)` or
  `stack: module (lwIP offload)` and `net dhcp` / `net ip` / `net ping` follow it. With
  the host stack up, `net ping` is a real echo request built by NetX and put on the wire
  by our driver — no eRPC round trip in the RTT at all.

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

  While the host stack is up the module's own lwIP receives nothing, so `net conc`,
  `net echo`, `net shell start`, `wifi connect/disconnect/scan` and every `link`
  subcommand are refused with a message saying to run `net down`. `wifi on/off/reset`
  stay allowed — they are the recovery path, and the owner notices the link being taken
  away and stands down cleanly. **The device firmware is unchanged** (`2.1.3+wio-n7`).

- **`net shell` — the telnet console** (issue #21, `app/net_shell.c`). `telnet <board-ip>`
  gets **the same shell as USB CDC, at the same time**: a second `cli_instance`
  (prompt `wio-net> `) bound to a transport whose bytes ride the eRPC socket offload.
  It **arms itself when an address comes up** (`wifi connect` / `net dhcp` / `net ip`);
  `net shell start [port]` / `stop` / `status` are the manual controls. Ported from
  `../stm32f746g-disco`'s `port/netxduo/nx_shell.c` — same transport vtable, `connected`
  write gate, telnet IAC handling and single-session instance reuse — but with an eRPC
  link instead of a local TCP stack underneath, which changes three things:

  - **One service thread, one blocking call.** The N3 firmware has a receive task plus
    **two** workers, and only the blocking socket receives run in parallel. Our
    `accept`/`recv` permanently occupies one; a second blocking call would leave the shell's
    own RPCs no worker at all. So a single thread (priority 12) owns the sockets, and
    **no `accept` is issued while a session is live** — a second telnet client simply waits
    in the listen backlog and is served the moment the first leaves. For the same reason
    `net ping`, `net conc` and `net echo` are refused **from either console** while it is
    armed — each of them parks a blocking receive of its own, and with both workers busy the
    console's output would queue past `CLI_TX_TIMEOUT` and lose characters. `net shell stop`
    frees the worker; building the device firmware with a larger `N3_WORKERS`
    (`fw/rtl8720/`) would lift the restriction altogether.
  - **Latency vs. link occupancy.** With output pending it sends up to 4 × 96 B and then
    polls RX (`MSG_DONTWAIT`) so a Ctrl+C interrupts a long report within ~26 ms; idle, it
    parks in a 250 ms receive (≈4 RPC/s, under 1 % of the link). After handing keystrokes
    to the shell it waits up to 20 ms for the answer before re-arming that receive —
    without it the echo would wait out the whole idle window, because the service thread
    runs *above* the shell instance and would otherwise re-block before the shell ran.
  - **Host timeouts are deliberately huge** (module time + 45 s). Everything except the
    blocking receives takes the module's single serial mutex, so while the other console
    runs `wifi connect`/`net dhcp` our output can be stuck behind it for 20–30 s. A shorter
    timeout would be a *false* "link dirty" verdict, costing the session and leaking fds.
    Recovery stays instant regardless: `wifi reset` force-quiesces the link and wakes every
    waiting call immediately.

  Self-destruct avoidance only — there is no command policy. What the telnet console
  refuses is what would destroy the transport it is running on: `wifi on/off/reset`
  (CHIP_EN), any YMODEM transfer (`xfer`, `wifi imgload`/`imgsend`), and `net shell stop`.
  `wifi connect`/`disconnect` are **allowed**, and will drop your session — the module stays
  up, so the console re-arms on the new address. Conversely, while the console is armed it
  holds the eRPC UART referenced, so `wifi log`/`probe`/`rpc` and every `wifi flash*` are
  refused **from either console** until `net shell stop`.

  Known limits: telnet freezes for the duration of a `wifi connect`/`net dhcp` on the other
  console (module serial mutex + the 127-byte wire budget), and output may be dropped after
  `CLI_TX_TIMEOUT`; a host-side timeout leaks the module's sockets and latches the console
  until a `wifi reset` (`net shell status` says so). `printf()` follows the console of the
  thread that ran the command — `_write` hands a non-CDC instance's output to that
  instance's transport — which is what puts the CoreMark report (`ee_printf` → `printf`) on
  the telnet session that started it. Because two interactive
  instances now share priority 16, `cli_start()` gives them a **time slice**
  (`CLI_INSTANCE_TIME_SLICE`) — otherwise `coremark` on one console would freeze the other
  for its whole run; the flip side is that benchmark scores drop while both are busy.

- **`link` — the UART link itself** (issue #23 U0-3 / U1, `shell/cmds/cmd_link.c`). Where
  `wifi` is L2 and `net` is L3, this is the wire between the STM32 and the companion chip,
  and it exists to produce a number: can a UART link carry 1500-byte Ethernet frames well
  enough to justify putting a real TCP/IP stack on the host?
  `link info` prints **both ends'** counters — the host's RX interrupt budget and error
  tally, and the module's own (interrupt count, ring peak, the three loss counters, free
  heap, current rate, and **how much of its FIFO grace one interrupt consumed**). Reading
  the module's side while traffic flows was impossible before: they go to the LOG UART,
  which cannot be attached while the eRPC link is held. Note `entry` and `burst` are
  different questions — `entry` is FIFO occupancy when the interrupt *started*, which is
  the real margin; `burst` also counts bytes that landed during the drain, and those are
  free.
  `link baud <2000000|3000000|4000000|6000000>` changes the line rate of the link.
  `link bench [bytes] [secs] [rx|tx|both]` (default 1500 B / 3 s / both) and `link sweep`
  generate measured traffic and report throughput plus the round-trip distribution
  (min / avg / p99 / max) — the **max** is the number that matters for carrying frames.
  All of it rides a **LINK-CTRL channel** (`u16 0xFFFF | u16 len | u16 crc | body`)
  multiplexed onto the same wire and owned by the link layer at both ends, so none of it
  required touching the generated eRPC server shim. It needs module firmware
  `2.1.3+wio-n5` (`fw/rtl8720/patches/0005`) and, because the host only learns which
  firmware is loaded from `wifi rpc ver`, every subcommand asks you to run that first.
  CTRL is only issued on a **quiescent** link, so `link` refuses while anything else holds
  the eRPC UART — in practice, stop the telnet console (`net shell stop`).
  `link baud` is the one command whose failure costs the link: the module acknowledges on
  the *old* rate and only then switches, so a lost acknowledgement changes nothing, but if
  the *new* rate does not work the host's attempt to put both ends back is best effort
  (the message telling the module to return is sent at that same bad rate). **`wifi reset`
  is the guaranteed recovery** — the module boots at 2 Mbaud and the host resets its own
  belief to match.

- **The DATA channel** (issue #23 U1, `app/link_data.{c,h}` + `fw/rtl8720/patches/0006`).
  A third frame type on the same wire — `u16 0xFFFE | u16 len | u16 crc | body`, body =
  `u8 chan | u8 flags | payload` — and unlike eRPC and CTRL it is **unsolicited,
  bidirectional and unacknowledged**, which is what a raw Ethernet frame needs. U2 will
  hang the module's WiFi netif off it; U1 builds the plumbing and measures it against a
  sink/source bench on the module (`link dbench [bytes] [secs] [rx|tx|both]`). Both magic
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

- **The L2 bridge** (issue #23 U2, `fw/rtl8720/patches/0007` + `link eth` / `link arp`).
  The DATA channel stops carrying a bench pattern and starts carrying **real Ethernet
  frames**: the module hands what it receives from the air to the host instead of to its own
  lwIP, and frames sent on `LINK_DATA_CHAN_ETH` go out over the air. These two are the
  **diagnostic** view of that channel — the stack that consumes it for real is `net up`
  above — so `link eth [secs] [max]` just decodes what arrives (ARP in full, IPv4
  addresses and protocol, everything else by ethertype) and `link arp <ip> [secs] [sender-ip]`
  puts one well-formed request on the wire once a second. **An `is-at` reply is the test**: it
  proves the host built a frame, the module transmitted it with its own MAC, a real machine
  answered, the driver accepted the answer, the tap caught it before lwIP, and it survived the
  link — passively watching broadcasts only ever exercises one direction.
  It is a **tap, not a rewrite**: `LwIP_Init()` still runs and the bridge swaps
  `xnetif[0].input`, so switching it off leaves the module's own stack — and therefore
  `net ping`, `net echo` and the telnet console — intact. Two things had to be got right, and
  neither is guessable: the WLAN driver **filters received IP packets against the netif's
  address** (so a bridge session zeroes it and restores it), and this lwIP is `NO_SYS=0` with
  `LWIP_TCPIP_CORE_LOCKING=0`, so both mutations run **on the tcpip thread** and the host stops
  the DHCP client first. The session is foreground and bounded, and the module runs its own
  watchdog over it, because while the tap is in the module's lwIP is deaf — `link eth` refuses
  to start while a telnet console is armed, and says to run `net dhcp` afterwards.

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
            OCTOSPI1 PSRAM bring-up (psram.c), MPU regions (mpu.c),
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
            coremark/ EEMBC CoreMark port (core_portme.*)
            netxduo/ nx_user.h (NetX Duo build configuration; the driver is in app/,
                     because what it sits on is the RTL8720 link, not a MAC)
svc/        freestanding services: fmt (printf), log (DTCM ring), timebase (DWT)
src/        custom clock-free SystemInit  (also the minimal `blink` example's main)
ldscript/   STM32H725AEIx_XIP.ld  (FLASH @ 0x70000000, RAM = AXI-SRAM, DTCM log, ITCM ISRs)
cmake/      ARM GNU toolchain file (auto-downloads gcc into tools/)
lib/        git submodules: cmsis_core/device_h7, stm32h7xx_hal_driver, tinyusb,
            threadx, netxduo, coremark
boot/       standalone USB DFU bootloader (internal 0x08000000) — see boot/README.md
fw/rtl8720/ reproducible build of the RTL8720DN's own firmware (the eRPC server that
            wifi/net drive) — host-side only, flashed by `wifi flashwrite`; see its README
```

## Memory map (`ldscript/STM32H725AEIx_XIP.ld`)

| Region | Address | Notes |
|---|---|---|
| FLASH (XIP) | `0x70000000` | external OCTOSPI2. Chip is 16 MB; the **app owns the first 8 MB** (boot validates writes there), the upper 8 MB is reserved for a future filesystem. |
| PSRAM | `0x90000000` | external OCTOSPI1 **APS6408 8 MB Octal DDR PSRAM**, memory-mapped @ 133 MHz Fixed Latency; MPU Normal non-cacheable (DMA-coherent scratch; `.psram_noinit`). |
| AXI-SRAM (D1) | `0x24000000` | 320 KB; `_estack = 0x24050000` (the MSP the bootloader loads). |
| DTCM | `0x20000000` | 128 KB; holds the reset-persistent `.log_noinit` crash-log ring, the **32 KB NetX Duo packet pool** (`.nx_pool`, issue #23 U3 — no DMA touches a frame, so DTCM costs nothing from AXI-SRAM and evicts nothing from the D-cache) and `membench` scratch. All of it bypasses the D-cache. |
| ITCM | `0x00000000` | 64 KB; holds the **interrupt paths** (`.itcm`, ~3 KB): ThreadX PendSV + SysTick, the RTL8720 UART RX ISR **and the wake-up it signals** (`tx_event_flags_set` + `_tx_thread_system_resume`, issue #23 U0-3 — that call alone had put the ISR back to 4.3 µs warm / 12.9 µs cold), and the fault handlers, plus the 4 KB `.itcm_bench` scratch. Zero-wait-state and outside both caches, so an ISR no longer pays a cold OCTOSPI2 fetch through the 16 KB I-cache: the same UART ISR went from **8.7 µs cold / 3.3 µs warm to a flat 0.7 µs** (issue #24). Loaded from its XIP load image by `SystemInit()`, which also zero-fills all 64 KB to initialise the TCM ECC; kept read-only by the MPU so a NULL write raises MemManage instead of corrupting ISR code. |
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
