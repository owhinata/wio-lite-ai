#!/usr/bin/env python3
"""Post-link guard for the cacheable PSRAM carve-out (issue #9 phase 2a).

WHY THIS EXISTS
---------------
The external PSRAM window used to be uniform: app/mpu.c mapped all 8 MB Normal
non-cacheable, and that is what lets the LTDC, the DMA2D and the DCMI share buffers
with the CPU without a single clean or invalidate.  Everywhere in this firmware
"it is in PSRAM" has therefore meant "it is safe for DMA".

Phase 2a breaks that equivalence for the top 2 MB, which becomes Normal write-back
so the NN runtime's working set is not billed one bus transaction per access.  The
rule that replaces it -- only the CPU may touch the carve-out -- has no runtime
enforcement whatsoever, and it fails in the nastiest available way.  A DMA buffer
that lands in there still transfers correctly at the bus level; the CPU then reads
it back out of the data cache, stale, but only once something has evicted or dirtied
the relevant lines.  Compare the DTCM mistake this script's sibling catches, where a
misplaced buffer moves no data at all, every single time: that one is at least
deterministic.  This one can pass every test on an idle board and corrupt a frame
under load.

So the checks below, on suffix-stripped base names (LTO renames these: the linked
image really does contain `cam_ring.lto_priv.0`):

  1. CACHE SAFETY -- every symbol in MUST_STAY_NONCACHEABLE lies OUTSIDE the MPU
     region, not merely outside the .psram_ai section.  The region is what makes
     memory cacheable, and it covers the whole 2 MB whether or not the linker put
     anything there, so a raw address or a future section inside the window would be
     just as fatal.  This is the check that matters; the other two protect it.

  2. RESIDENCY -- every symbol in REQUIRED_PSRAM_AI lies inside [_psram_ai_start,
     _psram_ai_end).  A base name matching NOTHING is also a failure: that is the
     vanished symbol a linker-script ASSERT cannot see.

  3. AGREEMENT -- the section starts exactly at the MPU region base.  That address is
     stated three times in three languages (the linker script, app/mpu.c, and the
     constant below) because none of them can include the others.  If they ever drift
     apart the hardware maps the wrong bytes cacheable, which nothing else notices.

Exit status 0 = pass; 1 = a placement failure; 2 = the check could not be
performed (which is also a failure -- a gate that skips itself reports the same
silence as a passing one).
"""

import argparse
import re
import subprocess
import sys

CANNOT_CHECK = 2   # exit code for "the check could not run", never confused with pass

# The cacheable carve-out.  Mirrors MPU region 3 in app/mpu.c and the .psram_ai
# output section in ldscript/STM32H725AEIx_IROM.ld -- see check 3 above.
PSRAM_AI_ORIGIN = 0x90600000
PSRAM_AI_LENGTH = 2 * 1024 * 1024

# Must be INSIDE the carve-out: the CPU-only buffers it exists for.
REQUIRED_PSRAM_AI = (
    "null_in_buf",         # `null` NN backend stub tensors (port/nn/nn_null.c)
    "null_box_buf",
    "null_scr_buf",
    "psram_ai_bench_buf",  # membench's cacheable row (shell/cmds/cmd_membench.c)
)

# Must stay OUTSIDE it: every PSRAM buffer a bus master touches.  All three are
# coherent today only because the window they sit in is non-cacheable, and none of
# the drivers issues any cache maintenance at all.
#
#   ltdc_fb    LTDC read DMA + DMA2D writes (port/ltdc/ltdc_display.c).  The worst of
#              the three: the display controller reads it CONTINUOUSLY while scan-out
#              is on, so there is no quiet moment in which a clean could catch up.
#   cam_ring   DCMI via DMA2_Stream1, double-buffered while streaming (port/camera).
#   cam_frame  DCMI target for a single capture, same file, same reasoning -- its own
#              declaration spells the cache contract out.
MUST_STAY_NONCACHEABLE = (
    "ltdc_fb",
    "cam_ring",
    "cam_frame",
)

# GCC clone/localisation suffixes; they stack, so stripping repeats.  Same set as
# cmake/check_dtcm_residency.py -- kept in step with it deliberately.
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
    print(f"check_psram_ai_residency: {message}", file=sys.stderr)
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


def matching(syms, name):
    """Every data symbol whose suffix-stripped base name is @name."""
    return [(addr, sym) for addr, typ, sym in syms
            if typ in "bBdD" and strip_clone_suffixes(sym) == name]


def check_in_range(syms, required, start, end, where, hint):
    """Every symbol matching a base name in @required must sit inside [start, end)."""
    failures = []
    for name in required:
        matches = matching(syms, name)
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


def check_out_of_range(syms, forbidden, start, end, where, hint):
    """Every symbol matching a base name in @forbidden must sit OUTSIDE [start, end)."""
    failures = []
    for name in forbidden:
        matches = matching(syms, name)
        if not matches:
            failures.append(
                f"{name}: no such object in the image.  This list names the buffers a "
                f"bus master owns; if one was renamed or removed, update the list "
                f"rather than letting the check silently guard nothing."
            )
            continue
        for addr, sym in matches:
            if start <= addr < end:
                failures.append(
                    f"{name}: {sym} is at 0x{addr:08x}, INSIDE {where} "
                    f"[0x{start:08x}, 0x{end:08x}).  {hint}"
                )
    return failures


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("elf")
    ap.add_argument("--nm", required=True)
    args = ap.parse_args()

    syms = read_symbols(args.nm, args.elf)
    ai_start, ai_end = bounds(syms, "_psram_ai_start", "_psram_ai_end")

    if ai_start != PSRAM_AI_ORIGIN:
        die(f"_psram_ai_start is 0x{ai_start:08x}, not the MPU region base "
            f"0x{PSRAM_AI_ORIGIN:08x} -- ldscript/STM32H725AEIx_IROM.ld and app/mpu.c "
            f"disagree about where the cacheable carve-out is, so the hardware is "
            f"caching a different range than the linker filled.")
    if ai_start == ai_end:
        die(".psram_ai is empty (0 B) -- it links that way silently when the PSRAM_AI "
            "attribute is dropped from every resident.  See include/mem_sections.h.")

    region_end = PSRAM_AI_ORIGIN + PSRAM_AI_LENGTH

    failures = check_out_of_range(
        syms, MUST_STAY_NONCACHEABLE, PSRAM_AI_ORIGIN, region_end,
        "the cacheable PSRAM carve-out",
        "A bus master owns this buffer, and every driver that shares it with the CPU "
        "relies on the PSRAM window being non-cacheable instead of issuing cache "
        "maintenance.  In here the transfer still happens and the CPU reads stale "
        "lines back -- intermittently, only under cache pressure.",
    ) + check_in_range(
        syms, REQUIRED_PSRAM_AI, ai_start, ai_end, ".psram_ai",
        "It fell back to the non-cacheable pool: its PSRAM_AI attribute "
        "(include/mem_sections.h) was dropped or the definition moved.",
    )

    if failures:
        print(f"check_psram_ai_residency: FAIL ({len(failures)} problem(s)) in {args.elf}",
              file=sys.stderr)
        for f in failures:
            print(f"  - {f}", file=sys.stderr)
        return 1

    print(f"check_psram_ai_residency: OK -- {ai_end - ai_start} B in .psram_ai at "
          f"0x{ai_start:08x}, {len(REQUIRED_PSRAM_AI)} required residents, "
          f"{len(MUST_STAY_NONCACHEABLE)} DMA buffer(s) still non-cacheable")
    return 0


if __name__ == "__main__":
    sys.exit(main())
