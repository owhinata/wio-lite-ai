# RTL8720DN device firmware — reproducible build

Build materials for the firmware that runs on the board's **RTL8720DN** WiFi/BLE module
(Realtek AmebaD, KM0 + KM4). That firmware is Seeed's
[`seeed-ambd-firmware`](https://github.com/Seeed-Studio/seeed-ambd-firmware): an **eRPC
server** that exports the module's WiFi and lwIP sockets over UART, which is what the
STM32's [`wifi`](../../README.md) (L2) and `net` (L3) commands drive.

Nothing here touches the STM32 firmware. This directory only produces an image for the
*other* chip; issue #19 gave us the on-device flasher (`wifi flash imgload` +
`wifi flash write`) that puts it there, with no host programmer and no soldering.

> **Shell command names below are as they were at the time** (this file doubles as the
> per-generation acceptance record). Issue #28 later reshaped the STM32 command surface;
> when following any procedure here, map: `wifi rpc ver` / `wifi rpc` → **`wifi ver`**,
> `link <sub>` → **`wifi link <sub>`**, `link sweep` → **`wifi link bench … all`**,
> `wifi probe` → **`wifi log reset`**, and the flat `wifi flashXXX` / `wifi imgXXX`
> commands → the **`wifi flash <sub>`** subtree (`flashread` → `flash read`,
> `flashinfo` → `flash info`, `flashbackup` → `flash backup`, `imgload`/`imginfo` →
> `flash imgload`/`flash imginfo`, `flashwrite` → `flash write`). `net conc`, the
> module-side `net ping` path, `wifi flashprobe` / `flashload` / `imgsend` /
> `flashtest` and `link eth` were removed (their wire-level behaviour is recorded here
> and in git history).

**Why rebuild it at all** (issue #20): the shipped firmware has two structural problems
that cap what `net` can do.

1. The eRPC server is a **single FreeRTOS task processing requests serially**
   (`src/erpc_setup.cpp` `run_erpc_server()` → `SimpleServer::poll()` → receive →
   handler → send). While a handler blocks, no other RPC is accepted — so **concurrent
   TCP is impossible**.
2. The socket handlers **ignore the `timeout` argument the IDL gives them**
   (`erpc_idl/rpc_wifi_lwip.erpc` declares `uint32 timeout` on `rpc_lwip_recv`/`read`/
   `recvfrom`; `src/wifi/wifi_api.c` drops it and calls the blocking lwIP function).
   If data never arrives the server **wedges permanently** until `wifi reset`. The
   STM32 side works around this today by setting `SO_RCVTIMEO` itself.

Fixing those means patching and reflashing the module firmware, which means first being
able to rebuild it *exactly*. That is what N1 (this milestone) establishes.

## Quick start

```sh
./build.sh setup     # one-time: fetch sources, install core + toolchain, swap in the fork
./build.sh build     # verify pins -> apply patches/ -> compile -> collect -> run the gates
```

Artifacts land in `out/` (git-ignored). The one that gets flashed is
`out/km0_km4_image2.bin`.

Requires `arduino-cli` on `PATH` (tested with 1.5.1), `git`, `python3`, and ~1 GB of
disk for the core and toolchain.

## What is pinned

Everything, because the artifact ends up in the flash of the only surviving board.
`build.sh` verifies each of these before it compiles anything and refuses to run
otherwise.

| Component | Pin |
|---|---|
| Firmware source | `Seeed-Studio/seeed-ambd-firmware` branch `Wio-Lite-AI` @ `fc9526d` (2021-07-23) |
| Arduino core | `Seeed-Studio/ArduinoCore-ambd` @ `f81bca7` (last functional change `2582acf`, 2021-03-19) |
| Toolchain | `realtek:ameba_d_asdk_toolchain@1.0.1` (`arm-none-eabi-gcc` 6.5.0, "Realtek ASDK-6.5.0 Build 3292") |
| Image tools | `realtek:ameba_d_tools@1.0.4` (`postbuild_img2_arduino_linux`, prebuilt KM0/KM4 boot images) |
| Board index | `https://files.seeedstudio.com/arduino/package_realtek.com_amebad_index.json` |
| FQBN | `realtek:AmebaD:ameba_rtl8721d` |
| arduino-cli | 1.5.1 (see *Known quirks* — nothing here depends on the version) |

Everything is installed under `_ref/ambd/` (git-ignored) with
`ARDUINO_DIRECTORIES_DATA`/`_USER` pointed at it, so **your `~/.arduino15` is never
touched**.

### The board-manager core is the wrong one

Installing `realtek:AmebaD@3.0.5` from the board index and building against it **fails**:

```
src/wifi/wifi_api.c:45:25: error: dereferencing pointer to incomplete type 'struct rpc_tcp_pcb'
src/wifi/wifi_api.c:47:8:  error: 'struct tcp_pcb' has no member named 'master_addr'
```

The firmware compiles against a **patched lwIP** — `struct rpc_tcp_pcb`, plus
`master_addr` / `client_addr` / `connected_external` / `poll_external` /
`accept_external` on `tcp_pcb` — that exists only in the `ArduinoCore-ambd` git
repository, not in the released tarball. Upstream's README says to `rm -rf` the
board-manager platform and `git clone` the fork over it; `build.sh setup` does exactly
that, but keeps the tarball's `tools/` (toolchain + image tools) and stashes the
discarded platform at `_ref/ambd/core-bm-3.0.5/` for reference.

`build.sh` guards against silently regressing to the wrong core two ways: a
`.wio-core-commit` stamp inside the installed core, and a direct `grep` for
`struct rpc_tcp_pcb` in its lwIP headers.

### Build flags

Upstream's `arduino-build.sh` passes the sketch's `src/` include paths as
`--build-property build.extra_flags=...`, which **replaces** the board's own
`build.extra_flags` (`-mthumb -DBOARD_RTL8721D {build.usb_flags}`). `build.sh`
reproduces that exactly rather than "fixing" it — it is how the shipped firmware was
built. It is harmless here: `-mthumb` is already in `compiler.c.flags`, and the only
consumer of `BOARD_RTL8721D` in the core is the `Wire` library, which this sketch does
not use.

Upstream's script is also cwd-dependent (relative sketch path, no `--build-path`).
`build.sh` uses absolute paths and an explicit `--build-path` throughout.

## Reproducibility

The build is **bit-for-bit deterministic for a given absolute source path**: two clean
builds from the same path produce an identical `km0_km4_image2.bin`.

It is **not** path-independent. `assert()` in the vendored eRPC C++ sources expands
`__FILE__`, so the absolute path of the sketch ends up as a string inside the image (7
occurrences, around `0x9bd98`). Building the same commit from a different directory
gives a functionally identical image with a different digest and a slightly different
length. `build.sh` always exports to `out/sketch/seeed-ambd-firmware/`, so within one
checkout the digest is stable; across checkouts at different paths, compare behaviour
and the gate output rather than digests.

The firmware source is never built in place. `build.sh` exports the pinned commit with
`git archive` into `out/sketch/` and applies `patches/*.patch` there, so
`_ref/seeed-ambd-firmware` stays an untouched upstream mirror. To change the firmware,
add a patch — do not edit the mirror.

**Patch application must not be swallowed.** The export lives under `out/`, which is
git-ignored *inside the wio-lite-ai repo*. A plain `git apply` run there discovers the
outer repo, sees the target as ignored, and silently no-ops (exit 0, nothing changed) —
producing an unpatched image that still looks built. (N1 had no patches, so this only
surfaced with N2.) `build.sh` defeats it two ways: it stops git's repo discovery at
`out/` with `GIT_CEILING_DIRECTORIES` so the export is treated as the plain tree it is,
and after each apply it runs a reverse `git apply -R --check`, which can only pass if the
patch is actually present — turning a silent no-op into a hard failure.

## The static gates

`gate.py` runs after every build (`./build.sh gate` re-runs it standalone). Each gate
covers a specific way the module could be left unreachable.

1. **IMG2 headers** — both sub-images carry the AmebaD magic `81958711` and the run
   addresses the boot ROM jumps to: `0x0c000020` (KM0) and `0x0e000020` (KM4), with a
   payload length that fits.
2. **Write range** — 4 KB aligned, at or above `0x6000`, and ending below `0x105000`,
   the factory WiFi-settings sector that holds the SSID and the **plaintext PSK**.
3. **Boot sectors vs the chip** (blocking) — the core's prebuilt `km0_boot_all.bin` /
   `km4_boot_all.bin` are compared against the full-chip backup of board #2.
   The comparison uses **the boot image's own length**, not the partition length: the
   images are 4412 B and 4068 B inside 0x4000/0x2000 partitions, so comparing whole
   partitions against short files would be meaningless. The remainder of each partition
   is separately required to be erased (`0xFF`).
4. **Drift** — how many bytes the build differs from the image currently on the chip.
   Report only; a byte-identical rebuild is not expected (different core build date).
   A difference in the **KM0 part** does get flagged, because that part is a prebuilt
   binary shipped with the core — if it changed, the core version changed.

The summary also prints the **device digest**: the module's own checksum algorithm (sum
of 32-bit little-endian words, identified on hardware in issue #19 and reimplemented on
the STM32 as `rtl_dl_digest_*`). This is what ties the gate report to the bytes that
actually get programmed. `wifi flash imgload` stages whatever you send it — the gates cannot
reach across the YMODEM transfer — so compare this number against `wifi flash imginfo` before
typing `confirm`. As a check of the implementation, the 2 MB stock backup digests to
`0x464A5FFD`, the value the module itself reported in issue #19 M5.

Failure is fail-closed throughout: a failed compile removes the previous artifacts before
it starts, so `./build.sh gate` can never validate a stale image, and any failed gate
exits non-zero.

**If gate 3 fails, stop.** It means the core's boot images are not what is on the chip,
so flashing only `0x6000` would pair a new image2 with an old boot. The fix is *not* to
automatically write all three images at `0x0`: reflashing the boot sectors of the only
surviving module is the most dangerous operation available here. Record the core
version, the boot headers and the stock restore procedure as an explicit approval on
issue #20, then do it by hand.

### N1 result on board #2

All gates passed, and gate 3 passed in the strongest possible way: the core's prebuilt
KM0 and KM4 boot images are **byte-identical to what is on the chip**, which confirms
the pinned core and toolchain are the ones Seeed shipped from.

```
km0_boot   4412 B  identical to chip 0x0000,  0x113c..0x4000 erased
km4_boot   4068 B  identical to chip 0x4000,  0x4fe4..0x6000 erased
image2   876544 B  (0xd6000) -> 0x6000..0xdbfff, 167936 B clear of 0x105000
  km0 part (102400 B)  identical to chip  (it is the core's prebuilt km0_image2_all.bin)
  km4 part             310115 B differ    (expected: different core build)
```

For reference, the prebuilt `km0_km4_image2.bin` kept at `_ref/ambd/firmware/` differs
from the on-chip image in 675789 bytes — more than our rebuild does. Its KM0 part is the
same core prebuilt as ours, so it comes from this same core, but the image on board #2 is
not a byte-match for it either.

## Flashing

The module is programmed by the STM32 itself (issue #19). Nothing else is needed — no
host programmer, no soldering.

```
wio> wifi flash imgload                 # then send out/km0_km4_image2.bin with `sb -k`
wio> wifi flash imginfo                 # re-verify the staged image against PSRAM
wio> wifi flash write 0x6000 confirm    # DESTRUCTIVE: erase + program + device-side digest
```

Use **`sb -k`** (1024-byte YMODEM blocks); plain `sb` sends 128-byte blocks and is
roughly an order of magnitude slower. The image is exactly `0xd6000` bytes, which is
already sector-aligned and covers the whole image2 partition, so the erase removes every
sector the previous image2 occupied — no manual `0xFF` padding is needed.

`flash write` verifies by asking the module for its own checksum of the written range and
comparing, so the write is proven byte-correct before the module is reset.

### Acceptance (N1)

Rebuilt-but-unmodified firmware must behave exactly like the factory one:

```
wifi rpc          # eRPC link ack (0x5A)
wifi connect ...  # station association
wifi status
net info
net dhcp
net ping <gw> 4
wifi log          # boot banner from the module's LOG UART
```

### Recovery

The RTL8720DN's download mode lives in **mask ROM** and is entered by a strap plus
`CHIP_EN`, so it is reachable even from an all-`0xFF` flash. This is effectively
unbrickable, and the restore path was proven end-to-end in issue #19 (M5):

```
wio> wifi flash imgload                 # send _ref/ambd/board2-stock/rtl8720_000000_200000.bin
wio> wifi flash imginfo                 # 2 MB, digest 0x464A5FFD
wio> wifi flash write 0x0 confirm       # full-chip restore, boot sectors included
```

> The full-chip backup contains the factory WiFi settings sector at `0x105000` with the
> **PSK in the clear**. It is git-ignored and must never be committed or shared.

## Safety rules

- The STM32's `boot/` and its clock tree are **out of scope** — N1 changes no STM32 code
  at all.
- Write **only `0x6000..0xdbfff`**. The boot sectors (`0x0`, `0x4000`) are not touched
  because gate 3 proves they already match, and `0x105000` is far outside the range.
- Gate 3 failing means stop and get explicit approval on issue #20 (above), never an
  automatic fallback to writing the boot sectors.
- Build artifacts are not committed; they are reproducible from the pins in `build.sh`.

## Layout

```
fw/rtl8720/
  README.md          this file
  build.sh           setup | build | gate | clean
  gate.py            the static gates
  patches/           N2..N6 patches, applied in filename order (empty for N1)
  out/               git-ignored build output
    km0_km4_image2.bin   <- the image that gets flashed
    km0_boot_all.bin     }  core prebuilts, copied out for the gates
    km4_boot_all.bin     }
    application.axf      ELF with symbols (for addr2line on a module crash)
    compile.log
    sketch/          pristine export of the pinned commit + patches
    build/           arduino-cli build directory
```

## Known quirks

- **`setup` and `build` take a lock** (`_ref/ambd/.build.lock`). They mutate shared state
  — the arduino-cli data directory, and the vendor postbuild step below — so two
  concurrent runs would interleave their artifacts. The core is staged beside the old one
  and swapped in at the end, with the `.wio-core-commit` stamp written last, so an
  interrupted `setup` leaves a state that `build` refuses rather than one it trusts.
- **The postbuild step writes into the shared tools directory.**
  `postbuild_img2_arduino_linux` runs with its cwd set to
  `_ref/ambd/arduino15/packages/realtek/tools/ameba_d_tools/1.0.4/` and drops
  `application.axf`, `km0_km4_image2.bin` and friends there, ignoring `--build-path`.
  `build.sh` copies them out into `out/`.
- **It also runs `rm -f bsp/image/*.bin`**, which deletes
  `imgtool_flashloader_amebad.bin` — the AmebaD flashloader stub that the *STM32* build
  embeds at configure time (`CMakeLists.txt` reads
  `_ref/ambd/imgtool_flashloader_amebad.bin`). The canonical copy lives at
  `_ref/ambd/`, outside the core, so the STM32 build is unaffected; `build.sh` restores
  the core's copy afterwards anyway.
- **PMU variant.** The postbuild tool hardcodes `bsp/image/PMU_bins/NONE/` as the source
  of the boot images (KM4 boot 4068 B). The other variants (`TL_RTC`, `D27_PA20`, …) ship
  a 4196 B KM4 boot and are not used. This matters for gate 3: `NONE` is the variant that
  matches board #2.
- **arduino-cli 1.5.1 works fine** with this 2021-vintage `platform.txt`, contrary to the
  concern recorded when the milestone was planned — no version pinning workaround was
  needed.
- The build is noisy (`-Wall -Wextra` over the vendor SDK). The full log is kept at
  `out/compile.log`; `build.sh` only echoes lines matching `error:` and the size summary.

## Roadmap

- **N1** — toolchain, reproducible unmodified build, flash-back and regression.
- **N2** (done) — bounded socket handlers, `patches/0001` + `patches/0002`. The wire
  format is unchanged; the one STM32-side change is additive (`wifi rpc` now prints the
  build id). See *N2 result* below.
- **N3** (built) — worker dispatch on the module (`patches/0003`: receive task + worker
  pool, concurrency limited to an allow-list of socket calls) plus multi-in-flight on the
  STM32 (`erpc_begin`/`erpc_wait`/`erpc_cancel`). This is where concurrent TCP becomes
  possible. See *N3 result* below.
- **N4** (built) — the link UART is driven by us, not by Arduino (`patches/0004`,
  issue #23 U0-2). See *N4 result* below.
- **N5** (built) — a LINK-CTRL channel on the same wire (`patches/0005`, issue #23 U0-3):
  read the module's own UART counters, change the line rate, and generate measured
  traffic. See *N5 result* below.
- **N6** (built) — a DATA channel on the same wire (`patches/0006`, issue #23 U1): the
  unsolicited, unacknowledged frame type that raw Ethernet needs, plus the pools, queue
  and tasks that U2's L2 bridge will plug into. See *N6 result* below.

### N2 result

`patches/0001-n2-bounded-socket-handlers.patch` + `patches/0002-n2-system-version-build-id.patch`
(both against `src/`, generated with `git diff`, applied by `build.sh`):

- `rpc_lwip_recv`/`read`/`recvfrom` honour the IDL `timeout` (ms) by saving, setting and
  restoring `SO_RCVTIMEO` (an int of ms here — `LWIP_SO_SNDRCVTIMEO_NONSTANDARD=1`,
  clamped to `INT_MAX`). `timeout==0` keeps the socket's own value, so the existing STM32
  caller (which sets `SO_RCVTIMEO` itself and passes a matching timeout) sees identical
  behaviour; if a non-zero bound cannot be armed the handler returns `-1` rather than
  blocking unbounded.
- `rpc_lwip_connect`/`accept` (no IDL timeout) get an internal cap (`20 s` / `10 s`) via
  `O_NONBLOCK` + `lwip_select`, restoring the original socket flags exactly. If the
  non-blocking mode cannot be installed they fail closed (`-1`) rather than run an
  unbounded blocking call.
- `rpc_system_version` returns an `erpc_malloc` copy of `2.1.3+wio-n2` instead of a string
  literal, so the shim's `erpc_free` no longer corrupts the module heap and the STM32 can
  read the build id (`wifi rpc ver`). The generated shim `rpc_system_server.cpp` is
  untouched.

Build: `km0_km4_image2.bin` = 876544 B, **device digest `0xC325D4D3`** (was `0xFF7EA39A`
for the N1 baseline), md5 `eb6f2a90…`, bit-reproducible from this path. Only the KM4
part changed (the KM0 part is the core prebuilt); gate 3 still finds the boot sectors
byte-identical to the chip, so the write stays at `0x6000` (image2 only).

Acceptance beyond the N1 regression set: `wifi rpc ver` prints `fw version: 2.1.3+wio-n2`,
and a `recvfrom(timeout=N)` on a socket with **no** `SO_RCVTIMEO` set now returns after
`N` ms instead of wedging the module until `wifi reset`.

### N3 result

`patches/0003-n3-worker-dispatch.patch` (against `src/erpc_setup.cpp` only — the vendored
eRPC runtime is not modified) turns the stock single poll()-loop server into a receive
task plus a worker pool, so a blocking handler no longer stalls every other RPC:

- A thin `DispatchServer : SimpleServer` exposes the runtime's own `runInternalBegin`
  (receive frame + allocate buffer/codec + parse header) and `runInternalEnd` (run handler
  + send reply + dispose). One receive task runs Begin and enqueues the parsed request;
  `N3_WORKERS` (default **2**) workers run End. Each request already gets its own 4 KB
  buffer (`createServerBuffer`), sends are lock-protected (`FramedTransport`), and
  malloc/free are the thread-safe RTOS heap (`-Wl,-wrap,malloc`), so parallel handling is
  buffer- and frame-safe.
- Concurrency is **allow-listed**: only the per-socket blocking receives
  (`rpc_lwip_accept`/`connect`/`recv`/`read`/`recvfrom`) and `rpc_system_ack` run in
  parallel; every other handler (association, tcpip/DHCP, TLS, mDNS, socket create/close/
  sendto/setsockopt) takes a single serial mutex and stays effectively serial, so shared
  WiFi/lwIP state is never raced.
- Queue-full is `xQueueSend(portMAX_DELAY)` back-pressure (degrades to the stock serial
  rate, never leaks); the device→host callback path stays the stock oneway one, so the
  arbitrator's mutex-free pending-client scan stays single-threaded.
- The pinned core has `INCLUDE_uxTaskGetStackHighWaterMark=0`, so stacks cannot be measured
  at runtime; they are sized generously (recv 6 KB, worker 12 KB — the stock shared task was
  24 KB) and the always-on `configCHECK_FOR_STACK_OVERFLOW=2` hook traps any shortfall.
  Free heap **is** observable and its low-water is logged to the LOG UART (`wifi log`).

The wire format is unchanged; what changes is that replies may now come back **out of
order**, which the N3 host client (`app/erpc.c` `erpc_begin`/`erpc_wait`/`erpc_cancel`)
routes by sequence.

Build: `km0_km4_image2.bin` = 880640 B (one 4 KB sector larger than N2), **device digest
`0x48961411`**, md5 `97c12e8f…`, sha256 `53c3e308…`, bit-reproducible from this path. Only
the KM4 part changed; gate 3 still finds the boot sectors byte-identical to the chip, so
the write stays at `0x6000` (image2 only).

Acceptance beyond the N2 regression set (`wifi rpc ver` must read `2.1.3+wio-n3`): run
`net conc` while associated. It holds a `recvfrom` (no data) open on the module and then
round-trips a foreground ack — on the serial stock/N2 server the ack cannot be answered
until the receive returns (so it times out), on N3 a spare worker answers it in a few ms
(`=> server is CONCURRENT`).

#### Constraints when building real concurrent TCP on top of N3

N3 is deliberately scoped to the socket-receive + ack allow-list. Two invariants must be
kept when that scope grows (both flagged in the N3 review):

1. **No non-oneway device→host callbacks.** Only one task (the receive task) calls
   `TransportArbitrator::receive()`, whose pending-client-list scan is mutex-free, so it is
   safe *only* while nothing else mutates that list. The stock `rpc_wifi_event_callback` is
   `oneway` (registers no pending client), so it does not. A generated **non-oneway**
   callback (e.g. a raw-lwIP TCP/DNS callback that waits for a host reply) would run
   `ArbitratedClientManager::performClientRequest` → `prepareClientReceive` on a *worker*
   thread (`m_serverThreadId` is the receive task, so a worker is not recognised as server
   context) and mutate `m_clientList` while the receive task scans it. Before adding any
   such callback, guard `m_clientList` with `m_clientListMutex` in `receive()`, or disable
   the callback client path.
2. **Serialize a socket's lifecycle per fd.** The blocking receives run outside
   `g_serialMutex`; `close`/`setsockopt`/teardown are only serialized against each other,
   not against a receive already blocked on the same fd. Today the STM32 host is a single
   owner and never interleaves teardown with an in-flight receive, so this is safe. A
   future multi-in-flight socket API must not `close`/mutate an fd while another worker is
   blocked in `recvfrom`/`accept` on it (lwIP fd-reuse / close-wakeup semantics make that
   racy).

### N4 result

`patches/0004-n4-usi-link-driver.patch` — new `src/link/wio_usi_uart.{c,h}` and
`src/link/wio_uart_transport.h`, plus the transport swap and build id in
`src/erpc_setup.cpp`. **The wire format does not change**, so every existing host feature
is a regression test.

The link was never limited by the UART. It was limited by three things stacked in the
Arduino layer *below* the eRPC transport, all of which this removes:

| | stock | N4 |
|---|---|---|
| RX interrupt | `USI_UARTRxFifoTrigLevel = 1`, handler reads **one** byte | threshold 16 of 64 (32 as first built, see N5), handler drains until the FIFO is empty |
| buffer | Arduino `RingBuffer`, **127 usable**, `store_char()` drops silently when full | 8 kB, drops the newest byte **and counts it** |
| transport read | one `read()` per byte, `vTaskDelay(1)` when empty | bulk `memcpy` out of the ring |

That 127-byte ring is why a big *request* frame was unreliable while a big *reply* was
fine (the ASYMMETRY note in `app/wifi_rpc.h`): the module stalls mid-frame to allocate its
4 KB message buffer, and anything that does not fit in the ring loses its tail, fails CRC
and is never answered. The measured cliff was between 184 and 280 bytes and was
**load-dependent**, which is why the host had to chunk sends at 96 bytes and keep a
ledger of unanswered bytes on the wire.

Facts the driver depends on, read out of the SDK source
(`Seeed-Studio/seeed-ambd-sdk`, `component/soc/realtek/amebad/fwlib/ram_hp/rtl8721dhp_usi_uart.c`)
rather than assumed — the fwlib entry points are ROM `_LONG_CALL_` stubs, so the headers
alone do not answer any of them:

- `USI_UARTStructInit()` defaults to **odd parity enabled**; 8N1 has to be set explicitly.
- `USI_UARTRxTimeOutCnt` is in **bit periods** (default 64 = 6.4 byte times). This is what
  ends a burst whose tail is shorter than the FIFO threshold, i.e. it is the tail latency
  of every reply, and it scales with the baud rate on its own.
- `USI_UARTWritable()` means "TX FIFO **empty**", not "not full" — polling on it sends one
  byte per full FIFO drain. `USI_UARTGetTxFifoEmptyCnt()` is what actually fills it.
- Almost-full and RX-timeout are hardware-cleared by draining the FIFO; RX-FIFO-overflow
  and stop-bit error are latched and **must** be written to `INTERRUPT_STATUS_CLR`
  (`IS_USI_UART_CLEAR_IT`) or they re-assert forever.

Module-side counters go to the LOG UART (`wifi log`) and are printed **only when they
move**, so a healthy link leaves the boot banner clean:
`[U0-2] usi burst N/64 ringpeak N drops N ovf N framing N`. `burst` is the high-water
bytes taken in one interrupt; `drops`/`ovf`/`framing` must all stay 0. **`burst` is not
the margin** — see the N5 section, which adds the counter that is.

#### The host side is version-gated

The host and the module are flashed separately, and the recovery path exists precisely to
put an *older* image back — so "may I send a 280-byte frame?" is a runtime question. The
host keeps one latch, the eRPC wire budget (`app/erpc.c`), which:

- starts at `ERPC_WIRE_BUDGET_SAFE` (127) and is raised to `ERPC_WIRE_BUDGET_FAST` only
  when `wifi rpc ver` reads back `wio-n4`;
- is dropped back by `rtl_dl_enter()` (any flash session, even an aborted one) and by
  `rtl_link_force_quiesce()` (any CHIP_EN move, i.e. the `wifi reset` recovery path);
- decides `wifi_rpc_send_chunk()` too, which is what the telnet console chunks its output
  at — 96 on a stock module, the full 256 on N4.

`wifi rpc` prints the current state (`wire: budget N B, send chunk N B`). Note this is
deliberately *not* tied to the UART open/close generation: `wifi rpc ver` drops its own
UART reference before returning, so a link-scoped latch would revert before it could ever
be used.

Acceptance (beyond the N3 regression set, with `wifi rpc ver` reading `2.1.3+wio-n4`):
`net echo 2323 256` must complete. That is a 280-byte request frame — the exact size that
gets no reply at all on N3 and earlier.

### N5 result

`patches/0005-n5-link-ctrl.patch` — `src/link/wio_uart_transport.h` gains a `receive()`
override, `src/link/wio_usi_uart.{c,h}` gain `wio_usi_set_baud()`/`wio_usi_baud()`, and
the build id becomes `2.1.3+wio-n5`. **The eRPC wire format is again unchanged**, so the
whole existing host feature set is the regression test.

Issue #23 needs a number before it can commit to putting a real TCP/IP stack on the STM32:
can this UART carry 1500-byte Ethernet frames well enough? Three things were missing.

1. **The module's own counters were unreadable.** N4 prints them to the LOG UART, but the
   LOG UART cannot be attached while the host holds the eRPC link — so they could never be
   seen *while traffic was flowing*, which is the only time they mean anything.
2. **There was no way to change the line rate.**
3. **There was no traffic generator.**

All three are answered by a second frame type multiplexed onto the same wire, owned by the
link layer at both ends:

```
CTRL frame = u16 0xFFFF | u16 body_len | u16 crc16(body) | body[body_len]
body       = u8 cmd | u8 seq | u8 status | u8 rsvd | payload[]
```

`0xFFFF` cannot collide with an eRPC frame, whose leading `u16` is the message size: no
*sender* can produce one that large (a host request is a few hundred bytes, a module reply
is bounded by its 4 KB `MessageBuffer`), and both receivers independently reject anything
above 4096. Adding these as eRPC methods instead would have meant hand-editing the
*generated* server shim — the same shim whose `erpc_free()` of a string literal corrupted
the module heap in N2.

Commands: `LINK_PING`, `LINK_STATS` (10 × u32 LE: the N4 counters plus free heap, current
baud and ring size), `LINK_SETBAUD`, `LINK_BENCH`.

Two rules in `receive()` are load-bearing:

- the whole read, including CTRL consumption, stays under `m_receiveLock`, as the base
  class does — and it **loops**, returning only when a real eRPC message is in hand,
  because consuming a CTRL frame is not something the eRPC server can be told about;
- a CTRL reply is written under `m_sendLock`, because the N3 workers call `send()`
  concurrently and two writers would interleave bytes inside a frame. This is the only
  place that takes both, receive→send, and nothing takes them the other way, so the order
  cannot cycle.

**A desynchronised stream can align on `0xFFFF` by chance**, so the length is bounded
before it is believed (never drained on faith), the CRC is checked, every command checks
its own payload length, and the one command with a side effect that costs the link —
`LINK_SETBAUD` — additionally requires a magic word (`'BAUD'`) and a rate from a fixed
allow-list. It acknowledges on the **old** rate and only then reprograms, so a lost
acknowledgement leaves both ends exactly where they were.

`USI_UARTSetBaud()` is safe to call on the fly: it is a plain read-modify-write of the TX
and RX baud registers, with no FIFO reset and no disable requirement
(`rtl8721dhp_usi_uart.c`). The driver waits for TX FIFO empty (`USI_UARTWritable()`, which
means *empty*, not *not full*), sleeps a tick for the shift register, reprograms, then
discards the RX FIFO and ring — bytes sampled across a rate change are meaningless.

**Recovery from a rate mismatch is `wifi reset` and nothing else.** The host tries to put
both ends back at 2 Mbaud, but that is best effort by construction: if the new rate failed
because of signal quality, the `LINK_SETBAUD(2M)` telling the module to come back is sent
at that same bad rate. A CHIP_EN power cycle always works — the module boots at 2 Mbaud,
and `rtl_link_forget_module()` puts the host there too.

#### What the first 6 Mbaud sweep changed here

Board #2 ran clean at every rate — 2/3/4/6 Mbaud, all three directions, **zero** losses on
either end — but the module counters that N5 exists to expose showed `max burst` climbing
41 → 46 → **56 of 64** as the rate went up. Two changes followed, and they are the reason
the driver in `patches/0005` is not quite the one `patches/0004` built:

- **`burst` cannot answer the question it looks like it answers.** It counts every byte one
  interrupt took, which is entry occupancy *plus* everything that arrived while the drain
  loop ran — and arrivals during the drain are free, because the loop absorbs them. 56/64
  could equally be a late interrupt (bad) or a busy drain (fine). `max_entry` is now
  sampled from `USI_UARTGetRxFifoValidCnt()` *before* the drain, which is exactly the
  grace the interrupt latency consumed, and `link info` prints it as `entry N/M grace`.
- **The threshold went 32 → 16.** The margin is `64 - threshold` byte times: 32 bytes is
  160 us at 2 Mbaud but only 53 us at 6 Mbaud, and 6 Mbaud is where this link is heading.
  16 restores it to 80 us. The cost is double the interrupt rate, which this handler —
  a drain loop — can afford; it is still 16x fewer interrupts than the stock trigger of 1.

`LINK_STATS` therefore returns 12 words, not 10 (`max_entry` and the configured threshold
are appended), and an older N5 build is rejected by the host with a message saying so.

Acceptance (beyond the N4 regression set, with `wifi rpc ver` reading `2.1.3+wio-n5`):
`link info` shows the module counters, `link bench`/`link sweep` run with `ctrl_bad` 0, and
`link baud 6000000` followed by `link baud 2000000` round-trips.

### N6 result

`patches/0006-n6-link-data.patch` — new `src/link/wio_link_data.{c,h}`, a `handleData()`
branch and a public `writeFramed()` in `src/link/wio_uart_transport.h`, the receive ring
doubled to 16 kB in `src/link/wio_usi_uart.c`, and the build id becomes `2.1.3+wio-n6`.
**The eRPC wire format is unchanged for the third time**, so the whole existing host
feature set is again the regression test.

CTRL answered "how fast is this wire". N6 answers the question after it: **can the three
channels interleave continuously without losing anything?** That needs a frame type that
is nothing like the other two — unsolicited, bidirectional, unacknowledged:

```
DATA frame = u16 0xFFFE | u16 body_len | u16 crc16(body) | body[body_len]
body       = u8 chan | u8 flags | payload[body_len - 2]
```

`0xFFFE` is safe for exactly the reason `0xFFFF` is, and it is a property of the SENDER,
not of the receiver: an eRPC frame's leading `u16` is its message size and neither end can
produce one that large. The two-byte body header earns its keep — with the buffer 8-byte
aligned, an Ethernet frame at +2 puts its IP header on a 4-byte boundary, which is what
the host's stack will want in U3.

In U1 the endpoint is a bench: `WIO_DATA_MODE_SINK` counts what arrives (checking the
sequence number in the payload, which is how loss becomes visible — the link has no
retransmission, and one lost *byte* costs a resynchronisation and therefore several
frames) and `WIO_DATA_MODE_SOURCE` generates. U2 replaces the body of the receive task
with `netif->linkoutput()` and feeds `wio_link_data_send()` from the driver's receive
path; nothing else about the file changes.

Three design points that are not obvious:

- **The transmit task exists because `wio_usi_write()` spins.** 1500 bytes at 6 Mbaud is
  2.5 ms of polling the TX FIFO. In U2 the producer is the WiFi driver's receive path, and
  spinning there would stall the driver, so frames are queued and a task at
  `tskIDLE_PRIORITY + 6` — below the eRPC receive task and the workers, so link control and
  RPC replies always win — does the writing. It writes through the transport's
  `writeFramed()`, which takes **`m_sendLock`**: the N3 workers send replies concurrently,
  and that lock is the only thing keeping two writers from interleaving bytes inside a
  frame.
- **The transmit queue is an index ring under the module's own mutex, not a FreeRTOS
  queue**, and that is a correctness requirement. `LINK_DATA_CFG(off)` promises the host
  that no further DATA frame can be on the way — the host's teardown depends on it, because
  detaching restores a flush that would eat a frame still in flight. That promise is only
  provable if "take the next frame" and "mark the writer busy" happen in ONE locked step.
  With a FreeRTOS queue they cannot: the task returns from `xQueueReceive()` holding an
  index and *then* takes the lock, and a `CFG(off)` landing in that window sees an empty
  queue, an idle writer — and a frame that goes out immediately afterwards.
- **Three outcomes in `handleData()`, deliberately different.** A length outside the
  channel's bounds is not a frame at all (a desynchronised stream that landed on `0xFFFE`)
  → resynchronise, because draining a length we do not believe would swallow real frames.
  A believable length with no pool buffer free IS a frame, just one we cannot hold → drain
  exactly it; losing a frame is an ordinary Ethernet event, losing stream synchronisation
  is not. Otherwise the body is read straight into the pool buffer, with no copy through a
  scratch.

Cost: `.ram_image2.bss` 83,252 → 108,532 B (+25,280 — six receive and four transmit
buffers of 1544 B, the generator's staging frame, and the 8 kB of extra ring). The
FreeRTOS heap array is unchanged at 208,896 B, so `link info`'s free heap only moves by
the two new task stacks (3 kB each). The image grows by one sector, 880,640 → 884,736 B.

#### Measured on board #2

| baud | rx | tx | both | loss |
|---|---|---|---|---|
| 2 M | 187.9 KB/s | 198.2 | 375.4 | 0 |
| 3 M | 270.5 | 295.4 | 531.7 | 0 |
| 4 M | 349.6 | 397.4 | 674.8 | 0 |
| **6 M** | **489.2** | **585.4** | **875.4** | one module-side pool drop (below) |

**The transmit direction runs at the wire rate.** 1199 frames × (1500 + 8) B in 3.000 s is
602.7 kB/s against a theoretical 597.8 kB/s at BRR 23 — 100 %, with the excess inside the
tick granularity of the measurement. That settles what U0-3 could not: the 366 KB/s the
CTRL bench reached was **not** a property of the wire, it was the half-duplex turnaround of
a request/reply exchange. Remove the reply and the gap disappears.

The single loss, at 6 Mbaud with both directions saturated, is a **module-side pool drop**
(`drops 1`, one sequence gap) with `crc 0`, `ring-drops 0` and `fifo-overrun 0` — so no
byte was lost on the wire. The receive pipeline was momentarily behind, once in 1198
frames, and dropped a frame the way an Ethernet device is entitled to. The pool stays at
six buffers: real traffic does not sit at 100 % saturation, and enlarging it would cost a
reflash for something already counted and correct. It is the first knob to reach for in U2
if the netif turns out to be a slower consumer than this bench.

Acceptance (beyond the N5 regression set, with `wifi rpc ver` reading `2.1.3+wio-n6`):
`link dbench 1500 3 rx | tx | both` completes with **zero** loss on either end — no
sequence gaps, no CRC failures, no pool drops, no ring drops, no FIFO overruns — and a
10 s `both` run does the same with a telnet console attached and running a command.

### N7 result

`patches/0007-n7-l2-bridge.patch` — new `src/link/wio_link_eth.{c,h}`, a `WIO_DATA_MODE_BRIDGE`
mode and a `tx_claim`/`tx_commit`/`tx_drop` split in `src/link/wio_link_data.{c,h}`, a
`LINK_ETH_INFO` command in `src/link/wio_uart_transport.h`, and the build id becomes
`2.1.3+wio-n7`. **The eRPC wire format is unchanged for the fifth time.**

N6 proved the DATA channel can carry 1500-byte frames continuously. N7 puts **real** ones on
it: the module stops handing received Ethernet frames to its own lwIP and sends them to the
host instead, and frames arriving on `WIO_DATA_CHAN_ETH` go out over the air. That is issue
#23 road B — the module becomes a relay and the STM32 gets to run the IP stack (U3).

#### The tap, and why it is not a new netif

The SDK's receive chain indexes a global array **inside the prebuilt libraries**, so adding a
netif of our own would never receive anything:

```
lib_wlan.a:rtk_wlan_if.o -> netif_rx(idx, len)             (lib_arduino.a:lwip_intf.o)
                         -> ethernetif_recv(&xnetif[idx])  (lib_arduino.a:ethernetif.o)
                         -> netif->input(pbuf, netif)      <-- the one hookable point
```

So `LwIP_Init()` still runs exactly as before and the bridge **swaps `xnetif[0].input`**. That
also makes it reversible: switch it off and the module's own lwIP is intact, which is what
keeps `net ping`, `net echo` and the telnet console usable as the regression test for
everything else. `ethernetif_recv()` frees the pbuf only when `input` returns something other
than `ERR_OK`, so the tap owns it: copy out, free, return `ERR_OK`.

#### Two things that are not obvious

- 🔴 **The WLAN driver filters received IP packets against the netif's address.**
  `netif_is_valid_IP()` is an undefined reference in `lib_wlan.a:rtk_wlan_if.o` and
  `ethernet_rswlan.o`, and it returns 1 unconditionally only when the netif's IPv4 address is
  0. A bridge session therefore has to **zero `xnetif[0]`'s address** and put it back
  afterwards, or unicast IP addressed to whatever the host's stack calls itself is dropped
  before it ever reaches `netif_rx()`. ARP is ethertype 0x0806, not IP, and is unaffected —
  which is exactly why "ARP is visible" is a weaker claim than "the bridge works".
- 🔴 **lwIP here is `NO_SYS=0` with `LWIP_TCPIP_CORE_LOCKING=0`** (`lwipopts.h:37`, `:385`), so
  a netif may not be mutated from an arbitrary task — the firmware's own
  `wifi_tcpip_adapter.c` precedent for calling `netif_set_addr()` off the tcpip thread is not
  good enough, and the adapter header itself says to stop DHCP before changing IP information.
  Both the address change and the input swap therefore run **on the tcpip thread** via
  `tcpip_callback_with_block()`, which blocks only until the message is *posted* — hence the
  completion semaphore and its 500 ms bound — and the **host stops the DHCP client first**
  over the existing eRPC path (`wifi_rpc_dhcpc_stop`).

#### Why BRIDGE is a mode and not a command

`WIO_DATA_MODE_BRIDGE` (0x04) goes through `wio_link_data_cfg()`, so N6's `CFG(off)` contract —
"no further DATA frame can be on the way" — covers the bridge's producer too, without a second
teardown protocol that could disagree with it. That producer is new: the **WiFi receive
thread**, which claims a transmit slot, spends a 1500-byte copy outside the lock, and commits.
`wio_link_data_send()` is re-expressed as `tx_claim` + copy + `tx_commit` so there is one
enqueue path, and the argument still holds in one locked step:

- the ring is drained under the lock and nothing can be pushed back, because every
  `tx_commit` now fails the `d_tx_enabled` test;
- the writer is proved idle by `d_tx_busy == 0` read in that same step;
- a slot still `LD_BUSY` in a receive-thread caller mid-copy **can never be enqueued**, so it
  is not a frame on the way. Its holder releases it itself when the commit is refused.

`CFG(off)` deliberately does **not** reclaim held slots — that would hand one to a new claimant
while the old holder still believed it owned it (the same ABA argument as `app/link_data.c`).

Two more safety properties: the `@ms` field of `CFG` doubles as a **watchdog** (the DATA
transmit task takes the bridge down itself if the host dies mid-session, so the module cannot
be left forwarding into a link nobody reads while its own lwIP starves), and the pointer swap
is ordered against the WiFi receive thread by publishing the state before the pointer, with the
tap dropping and counting any frame that reaches it after the bridge is down.

`LINK_ETH_INFO` (CTRL command 7) returns the MAC — the only source address the host may put on
a frame — plus "is the radio up" and eight counters. `rx_nobuf` is kept **separate** from the
DATA channel's own drops: a full transmit pool under a LAN broadcast burst is an Ethernet-legal
drop, and a DATA loss would mean the link failed.

Cost: `.ram_image2.bss` 108,532 → 108,608 B (+76). The image grows by one sector,
884,736 → 888,832 B, so the write range is 0x6000..0xdefff — still 155,648 B clear of the
`0x105000` WiFi-settings sector.

#### The watchdog cannot be exercised by resetting the host

Worth writing down, because the obvious test does not test it. `rtl8720_init()` presets
CHIP_EN (PC3) Low before switching the pad to output and drives it Low again immediately
(`app/rtl8720.c`), so **a host reset power-cycles the module** — it comes back unbridged
whatever the watchdog does, and the `wifi rpc ver` that follows prints "powering on
RTL8720DN". On this board a host that dies takes the module with it.

So the watchdog is not covering "the host lost power". What it does cover is the host
staying alive while stopping asking: a `CFG(OFF)` that fails or times out, a future resident
owner that goes away, or a bridge left up by a path that skips the session epilogue. Those
are the cases where the module would otherwise sit bridged with its own lwIP starved and
nothing left to notice.

Acceptance (beyond the N6 regression set, with `wifi rpc ver` reading `2.1.3+wio-n7`):
`link arp <gateway>` shows a `who-has` going out and an **`is-at` reply coming back** — which
proves the whole path in both directions at once — and after the session `net info`,
`net dhcp`, `net ping <gateway>` and the telnet console all work again unchanged.
