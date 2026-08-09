#!/usr/bin/env python3
"""Post-link guard for the AXI-SRAM / DTCM split (issue #46).

WHY THIS EXISTS
---------------
AXI-SRAM is the only RAM a bus master can address on this part: DMA1/DMA2 and the
SDMMC1 internal IDMA cannot reach either TCM (RM0468 sec 2.1.2 / 2.1.5 / 2.1.6).
Issue #46 acts on that by reserving AXI-SRAM for buffers a master touches and
moving every thread stack and the RTL8720 UART rings into DTCM, which is what
frees the space the camera's DCMI band needs (issue #35).

Both halves of that split fail SILENTLY, in opposite directions:

  * A DTCM selector that stops matching is not a link error.  The variable simply
    goes back to AXI-SRAM, the budget regresses by tens of kilobytes, and the only
    symptom is a future link failure in a change that has nothing to do with it.
    Two of the selectors are section-name patterns in the linker script
    (`.bss.<name>`), which exist only because of -fdata-sections and which LTO is
    free to rename -- exactly the failure mode documented for the .itcm ASSERTs.

  * The reverse is worse.  A buffer a DMA engine writes does NOT fault if it ends
    up in DTCM: the transfer moves nothing, and the corruption surfaces later as
    data that was never written.  There is no runtime check for this anywhere.

So this asserts both directions against the linked image, on suffix-stripped base
names:

  1. RESIDENCY -- every symbol in REQUIRED_DTCM lies inside [_sdtcm_bss,
     _edtcm_bss).  A base name matching NOTHING is also a failure: that is the
     vanished symbol a linker-script ASSERT cannot see.

  2. DMA REACHABILITY -- every symbol in REQUIRED_AXI lies inside AXI-SRAM.  These
     are the buffers a bus master owns; if one is ever moved by a well-meaning
     "this is CPU-only too" edit, the build stops here instead of the hardware
     going quiet.

Exit status 0 = pass; 1 = a placement failure; 2 = the check could not be
performed (which is also a failure -- a gate that skips itself reports the same
silence as a passing one).
"""

import argparse
import re
import subprocess
import sys

CANNOT_CHECK = 2   # exit code for "the check could not run", never confused with pass

# AXI-SRAM (D1).  Mirrors the MEMORY block of ldscript/STM32H725AEIx_IROM.ld.
RAM_ORIGIN = 0x24000000
RAM_LENGTH = 320 * 1024

# Must be in DTCM (.dtcm_bss).  Every one is CPU-only: thread stacks, and the
# RTL8720 UART rings the byte-at-a-time ISR fills.
#
# NOT listed: cli_job_stacks, which stays in AXI-SRAM on purpose.  Background jobs
# run CoreMark and the like -- latency-insensitive by design (shell/include/
# cli_config.h) -- and DTCM is the scarcer of the two once the stacks land there.
# 🔴 ONLY the residents that exist in EVERY build are listed here.  Anything a
# BSP_ENABLE_* toggle can compile out is passed in from CMakeLists.txt with
# --require-dtcm / --require-axi, because CMake is the only place that knows which
# features are in this image.
#
# The reason is not tidiness.  A name in this list that the build did not compile is
# reported as "no such object in the image" -- which reads exactly like a placement
# regression, and is not one.  `BSP_ENABLE_SD=OFF` used to fail that way on
# `sd_bounce`, so the toggle could not be used at all: its whole purpose is to build a
# firmware with no bus master while bisecting a D-cache suspicion, and the gate refused
# precisely the build someone reaches for when something is already wrong.  Same flaw,
# same fix as cmake/check_psram_ai_residency.py (issue #9 phase 2c).
REQUIRED_DTCM = (
    "usb_stack",                    # TinyUSB device task
    "cdc_sh_stack",                 # console shell instance   (CLI_INSTANCE_DEFINE)
    "net_sh_stack",                 # telnet shell instance    (CLI_INSTANCE_DEFINE)
    "nxn_stack",                    # NetX Duo owner thread
    "nxn_ip_stack",                 # NetX Duo IP thread
    "g_stack",                      # net_shell listener
    "erpc_svc_stack",               # eRPC link service
    "iwdg_stack",                   # watchdog petter
    "_tx_timer_thread_stack_area",  # ThreadX timer thread (upstream; linker pattern)
    "rtl_ring",                     # RTL8720 UART RX ring, ~75 k interrupts/s
    "rtl_tx_ring",                  # RTL8720 UART TX ring
    "nsh_tx_buf",                   # telnet console TX ring (issue #48)
)

# Must stay in AXI-SRAM: a bus master writes or reads these directly.
#
#   sd_bounce  SDMMC1 IDMA (port/sd/sd_card.c).  An AHB master -- it cannot see the
#              TCMs at all, and moving this buffer would make every SD transfer a
#              silent no-op rather than an error.
#   cam_band   DCMI via DMA2_Stream1 (port/camera/camera.c, issue #35).  Same
#              reachability rule, and the same silent failure: in DTCM the camera
#              would keep running and the panel would show whatever was in the
#              buffer at boot.
#
# cam_frame / cam_ring / ltdc_fb are the other master-touched buffers.  They live in
# PSRAM and are checked by cmake/check_psram_ai_residency.py, which asserts they stay
# OUT of the cacheable carve-out issue #9 opened in the top 2 MB of that window.  (An
# earlier version of this comment said the linker script's .psram_noinit ASSERT covered
# them; it never did -- that ASSERT only bounds the total against the 8 MB window and
# says nothing about which sub-range a buffer landed in.  Harmless while the whole
# window was non-cacheable, wrong the moment part of it was not.)
# Both of these belong to optional peripherals (BSP_ENABLE_SD / BSP_ENABLE_CAMERA), so
# like the DTCM list above they arrive from CMakeLists.txt via --require-axi rather than
# being named here.  The comment stays because the REASON is not conditional: whichever
# of them is in a given image must be in AXI-SRAM.
#
# 🔴 MAINTENANCE OBLIGATION, and it is the weak point of moving the lists out.  A NEW
# bus-master buffer added to this firmware is not protected until someone adds it to
# CMakeLists.txt -- and nothing here can notice the omission, because a buffer that was
# never named looks exactly like a build where the feature is off.  Adding a *_DMA()
# call or an IDMA/DCMI target means adding its buffer there, in the same commit.  If
# this file ever grows a way to enumerate DMA-touched buffers from the image itself,
# that would be strictly better than a list anyone has to remember.
REQUIRED_AXI = ()

# GCC clone/localisation suffixes; they stack, so stripping repeats.  Same set as
# cmake/check_itcm_residency.py -- kept in step with it deliberately.
CLONE_SUFFIX_RE = re.compile(
    r"\.(?:isra|constprop|part|lto_priv|cold|localalias)(?:\.\d+)?$")


def strip_clone_suffixes(name):
    """`foo.lto_priv.0` -> `foo`."""
    while True:
        stripped = CLONE_SUFFIX_RE.sub("", name)
        if stripped == name:
            return name
        name = stripped


def die(message):
    """Exit with CANNOT_CHECK: the guard did not run, which is not the same as passing."""
    print(f"check_dtcm_residency: {message}", file=sys.stderr)
    sys.exit(CANNOT_CHECK)


def run(cmd):
    try:
        return subprocess.run(cmd, capture_output=True, text=True, check=True).stdout
    except (OSError, subprocess.CalledProcessError) as exc:
        die(f"cannot run {cmd[0]}: {exc}")


def read_symbols(nm, elf):
    """[(address, type, name)] for every symbol nm reports with an address."""
    syms = []
    for line in run([nm, elf]).splitlines():
        parts = line.split(maxsplit=2)
        if len(parts) != 3 or not re.fullmatch(r"[0-9a-fA-F]+", parts[0]):
            continue
        syms.append((int(parts[0], 16), parts[1], parts[2]))
    return syms


def bounds(syms, lo_name, hi_name):
    found = {name: addr for addr, _t, name in syms if name in (lo_name, hi_name)}
    if len(found) != 2:
        die(f"{lo_name}/{hi_name} not found -- wrong linker script?")
    return found[lo_name], found[hi_name]


def check_in_range(syms, required, start, end, where, hint):
    """Every symbol matching a base name in @required must sit inside [start, end)."""
    failures = []
    for name in required:
        matches = [
            (addr, sym)
            for addr, typ, sym in syms
            if typ in "bBdD" and strip_clone_suffixes(sym) == name
        ]
        if not matches:
            failures.append(
                f"{name}: no such object in the image.  Either it was renamed beyond "
                f"the suffixes this script strips, or it was optimised away -- check "
                f"its definition before relaxing this."
            )
            continue
        for addr, sym in matches:
            if not start <= addr < end:
                failures.append(
                    f"{name}: {sym} is at 0x{addr:08x}, outside {where} "
                    f"[0x{start:08x}, 0x{end:08x}).  {hint}"
                )
    return failures


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("elf")
    ap.add_argument("--nm", required=True)
    ap.add_argument("--require-dtcm", action="append", metavar="SYMBOL", default=[],
                    help="feature-conditional object that must live in .dtcm_bss; "
                         "repeat once per object (CMakeLists.txt supplies these)")
    ap.add_argument("--require-axi", action="append", metavar="SYMBOL", default=[],
                    help="feature-conditional DMA buffer that must stay in AXI-SRAM")
    args = ap.parse_args()

    required_dtcm = tuple(REQUIRED_DTCM) + tuple(args.require_dtcm)
    required_axi = tuple(REQUIRED_AXI) + tuple(args.require_axi)

    # No --require-axi at all would mean a build with no bus-master buffers to check.
    # That is legitimate (BSP_ENABLE_SD=OFF plus BSP_ENABLE_CAMERA=OFF) but rare enough
    # to be worth saying out loud, so a missing argument is never mistaken for a
    # thorough check that found nothing.
    if not required_axi:
        print("check_dtcm_residency: note -- no bus-master buffers were named, so the "
              "DMA-reachability half of this gate checked nothing (expected only with "
              "BSP_ENABLE_SD=OFF and BSP_ENABLE_CAMERA=OFF)")

    syms = read_symbols(args.nm, args.elf)
    dtcm_start, dtcm_end = bounds(syms, "_sdtcm_bss", "_edtcm_bss")
    if dtcm_start == dtcm_end:
        die(".dtcm_bss is empty (0 B) -- it links that way silently when its "
            "selectors stop matching or it is placed after .bss.  See the placement "
            "rules in the linker script.")

    failures = check_in_range(
        syms, required_dtcm, dtcm_start, dtcm_end, ".dtcm_bss",
        "It fell back to AXI-SRAM: either its DTCM_BSS attribute was dropped, or "
        "its .bss.<name> pattern in ldscript/STM32H725AEIx_IROM.ld stopped matching "
        "(LTO renames symbols).",
    ) + check_in_range(
        syms, required_axi, RAM_ORIGIN, RAM_ORIGIN + RAM_LENGTH, "AXI-SRAM",
        "A bus master owns this buffer and cannot reach the TCMs (RM0468 sec 2.1.6) "
        "-- in DTCM the transfer would silently move nothing.",
    )

    if failures:
        print(f"check_dtcm_residency: FAIL ({len(failures)} problem(s)) in {args.elf}",
              file=sys.stderr)
        for f in failures:
            print(f"  - {f}", file=sys.stderr)
        return 1

    print(f"check_dtcm_residency: OK -- {dtcm_end - dtcm_start} B in .dtcm_bss, "
          f"{len(required_dtcm)} required residents, "
          f"{len(required_axi)} DMA buffer(s) still bus-master reachable")
    return 0


if __name__ == "__main__":
    sys.exit(main())
