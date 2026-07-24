# RTL8720DN device firmware — reproducible build

Build materials for the firmware that runs on the board's **RTL8720DN** WiFi/BLE module
(Realtek AmebaD, KM0 + KM4). That firmware is Seeed's
[`seeed-ambd-firmware`](https://github.com/Seeed-Studio/seeed-ambd-firmware): an **eRPC
server** that exports the module's WiFi and lwIP sockets over UART, which is what the
STM32's [`wifi`](../../README.md) (L2) and `net` (L3) commands drive.

Nothing here touches the STM32 firmware. This directory only produces an image for the
*other* chip; issue #19 gave us the on-device flasher (`wifi imgload` +
`wifi flashwrite`) that puts it there, with no host programmer and no soldering.

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
actually get programmed. `wifi imgload` stages whatever you send it — the gates cannot
reach across the YMODEM transfer — so compare this number against `wifi imginfo` before
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
wio> wifi imgload                    # then send out/km0_km4_image2.bin with `sb -k`
wio> wifi imginfo                    # re-verify the staged image against PSRAM
wio> wifi flashwrite 0x6000 confirm  # DESTRUCTIVE: erase + program + device-side digest
```

Use **`sb -k`** (1024-byte YMODEM blocks); plain `sb` sends 128-byte blocks and is
roughly an order of magnitude slower. The image is exactly `0xd6000` bytes, which is
already sector-aligned and covers the whole image2 partition, so the erase removes every
sector the previous image2 occupied — no manual `0xFF` padding is needed.

`flashwrite` verifies by asking the module for its own checksum of the written range and
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
wio> wifi imgload                    # send _ref/ambd/board2-stock/rtl8720_000000_200000.bin
wio> wifi imginfo                    # 2 MB, digest 0x464A5FFD
wio> wifi flashwrite 0x0 confirm     # full-chip restore, boot sectors included
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
  patches/           N2/N3 patches, applied in filename order (empty for N1)
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
- **N3** — worker dispatch on the module (receive task + worker pool, concurrency limited
  to an allow-list of socket calls) plus multi-in-flight on the STM32
  (`erpc_begin`/`erpc_wait`/`erpc_cancel`). This is where concurrent TCP becomes possible.

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
