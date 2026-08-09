#!/usr/bin/env python3
"""Post-link guard for the C++ containment of the TFLM backend (issue #9 phase 2c).

WHY THIS EXISTS
---------------
Until phase 2c this firmware had no C++ at all: `project(wio-lite-ai C ASM)`, no
`enable_language(CXX)` anywhere, and a toolchain file that named no C++ compiler.
TensorFlow Lite Micro is C++17, so that phase brought a language runtime into a
384 KB image that is otherwise entirely C.  Two things then go wrong, and neither of
them announces itself at link time.

1. STATIC CONSTRUCTORS RUN BEFORE THE WORLD EXISTS.  The CMSIS startup calls
   __libc_init_array immediately before `bl main` (startup_stm32h725xx.s), which is:
   before app/mpu.c programs the MPU, with both caches off, before the OCTOSPI1 PSRAM
   window is usable, before log_init() gives `dmesg` somewhere to write, before
   fault_init() installs the fault reporter, and with no ThreadX at all.  MPU region 3
   -- the one that makes the .psram_ai carve-out cacheable -- is set up from main().
   So a global constructor that touches the carve-out does not fault and does not log:
   the AXI transaction to an unconfigured external memory simply never completes, and
   the board hangs with an empty `dmesg` after the next reset.  There is no worse
   failure mode available on this part.

   The design answer is that the backend has no non-trivial globals -- the arena and
   the model slots are plain C arrays, the interpreter is placement-new'd inside
   open(), and the op resolver is a function-local static under
   -fno-threadsafe-statics.  This check is what keeps that true, because nothing else
   would notice a `static SomeClass g_thing;` appearing in a future edit.

2. THE EXCEPTION MACHINERY COMES BACK IN THROUGH THE SIDE DOOR.  Everything is
   compiled -fno-exceptions -fno-rtti, but that governs code GENERATION, not what the
   linker extracts.  One unresolved reference to the standard throwing `operator new`
   is enough to pull libstdc++_nano's version, and with it __cxa_throw, the
   _Unwind_* personality machinery and std::bad_alloc's type-info -- tens of KB of
   code that -fno-exceptions guarantees can never execute.  port/nn/tflm/cxx_runtime.cc
   defines our own operator new precisely so that never happens; this verifies it did
   not happen rather than trusting that it did not.

3. MANGLED NAMES DEFEAT THE OTHER GATES.  cmake/check_psram_ai_residency.py matches
   objects by exact (suffix-stripped) name, and the linker script's ASSERTs use
   `DEFINED(sym) ? ... : 1`, which passes vacuously when the name it asks about does
   not exist.  A C++ file-scope `static` can be emitted as `_ZL...`, so a buffer that
   moved into C++ would leave both of those checking nothing at all.  This is exactly
   how LTO's `foo.lto_priv.0` renaming broke ASSERTs before (issue #39), and it is why
   the carve-out's residents are defined in nn_tflm_bufs.c -- a C file -- on purpose.
   Here we confirm no mangled name has appeared in the carve-out.

Exit status 0 = pass; 1 = a containment failure; 2 = the check could not be
performed (which is also a failure -- a gate that skips itself reports the same
silence as a passing one).
"""

import argparse
import re
import subprocess
import sys

CANNOT_CHECK = 2   # exit code for "the check could not run", never confused with pass

# Bytes each static-initialisation table already carries in the DEFAULT (`null`, pure C)
# firmware.  .init_array's single entry is crtbegin's frame_dummy, which registers the
# exception-frame tables and is emitted for every image whether or not any C++ exists;
# .fini_array's is its counterpart.
#
# 🔴 ALL FIVE ARE CHECKED, not just .init_array.  The linker script KEEPs
# .preinit_array, .init_array and .fini_array (ldscript/STM32H725AEIx_IROM.ld), and GCC
# still honours the legacy .ctors/.dtors spellings -- so a guard that watched only
# .init_array would leave four other doors into "code that runs before main" open.  A
# `__attribute__((constructor(101)))` lands in .preinit_array, and that is the WORST of
# them: preinit constructors run even earlier than the rest.
#
# 🔴 These are CEILINGS measured from the C-only build, not exact values.  Writing
# `== 4` would turn a toolchain upgrade that adds a second crtbegin entry into a build
# failure blamed on a source change that is perfectly correct.  What we care about is
# that OUR code contributed nothing, and "did not grow past the C-only baseline" says
# that without asserting anything about the toolchain's own entries.  If a toolchain
# bump does raise one, re-measure on the `null` build
# (`arm-none-eabi-objdump -h build/shell.elf`) and raise it here -- after checking with
# --list that the new entry is not one of ours.
BASELINE_BYTES = {
    ".preinit_array": 0,
    ".init_array":    4,
    ".fini_array":    4,
    ".ctors":         0,
    ".dtors":         0,
}

# Symbols that must not be in the image.  Each entry is (regex, why-it-matters); the
# message is the point, because the fix differs completely between them.
FORBIDDEN = (
    (r"^__cxa_(throw|rethrow|begin_catch|end_catch|allocate_exception|"
     r"free_exception|call_unexpected|bad_cast|bad_typeid)$",
     "the exception runtime was linked in.  Something references the THROWING "
     "operator new (or a real `throw`): check that port/nn/tflm/cxx_runtime.cc still "
     "defines all six operator new/delete forms and that it is still compiled into "
     "the tflm library."),
    (r"^_Unwind_",
     "the stack unwinder was linked in -- see the __cxa_* note above; it arrives "
     "with the exception runtime and is useless in a -fno-exceptions build."),
    (r"^__gxx_personality",
     "an exception personality routine was linked in: some translation unit was "
     "compiled WITHOUT -fno-exceptions."),
    (r"^__cxa_guard_(acquire|release|abort)$",
     "thread-safe static initialisation was emitted: -fno-threadsafe-statics was "
     "dropped from the tflm target.  These guards would also be the wrong primitive "
     "here -- they are pthread-shaped, and this is ThreadX."),
    (r"^__cxa_atexit$|^__dso_handle$|^atexit$|^__register_exitproc$",
     "static destructor registration was emitted.  Something has a non-trivial "
     "destructor at namespace or function-local static scope: nothing in this "
     "firmware ever returns from main, so this costs flash and a struct _atexit in "
     "AXI-SRAM to run destructors that cannot run.  Fix it by placement-new'ing the "
     "object into a plain array (port/nn/tflm/nn_tflm.cc does this for both the "
     "interpreter and the op resolver), NOT by removing -fno-use-cxa-atexit.\n"
     "      Note both spellings are listed on purpose: -fno-use-cxa-atexit makes GCC "
     "register with newlib's plain atexit() instead of __cxa_atexit(), so a check "
     "written against the __cxa_ name alone sees nothing.  That is exactly what "
     "happened on this backend's first link -- the flag that was supposed to remove "
     "the registration only renamed it."),
    (r"^_ZTISt9bad_alloc$|^_ZTVSt9bad_alloc$|^_ZTVN10__cxxabiv1",
     "C++ type-info / abi vtables were linked in, which means RTTI or the exception "
     "runtime survived: check -fno-rtti and the operator new definitions."),
)


def die(message):
    """Exit with CANNOT_CHECK: the guard did not run, which is not the same as passing."""
    print(f"check_cxx_runtime: {message}", file=sys.stderr)
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


def section(objdump, elf, name):
    """(size, vma) of section @name, or None when the image has no such section."""
    for line in run([objdump, "-h", elf]).splitlines():
        parts = line.split()
        if len(parts) >= 4 and parts[1] == name:
            try:
                return int(parts[2], 16), int(parts[3], 16)
            except ValueError:
                return None
    return None


def table_entries(objdump, elf, name):
    """The function addresses in section @name, in order, with the Thumb bit cleared."""
    out = run([objdump, "-s", "-j", name, elf])
    words = []
    for line in out.splitlines():
        # " 80643cc f90f0208                             ...."  -- addr then hex groups
        m = re.match(r"\s+[0-9a-f]+\s((?:[0-9a-f]{2,8}\s)+)\s", line)
        if not m:
            continue
        blob = m.group(1).replace(" ", "")
        for i in range(0, len(blob) - 7, 8):
            # objdump -s prints raw bytes; this image is little-endian.
            b = blob[i:i + 8]
            words.append(int(b[6:8] + b[4:6] + b[2:4] + b[0:2], 16) & ~1)
    return words


def name_at(syms, addr):
    """Best-effort symbol name for a code address (the constructor's own name)."""
    best = None
    for a, typ, name in syms:
        if typ in "tTwW" and a <= addr and (best is None or a > best[0]):
            best = (a, name)
    return f"{best[1]}+0x{addr - best[0]:x}" if best else "?"


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("elf")
    ap.add_argument("--nm", required=True)
    ap.add_argument("--objdump", required=True)
    ap.add_argument("--list", action="store_true",
                    help="print every static-initialisation entry with its symbol")
    args = ap.parse_args()

    syms = read_symbols(args.nm, args.elf)
    failures = []

    # --- 1. no constructors (or destructors) of ours -------------------------
    # .init_array must exist -- the linker script always emits it (KEEP).  The other
    # four are absent from a clean image, and an absent section is a passing one.
    if section(args.objdump, args.elf, ".init_array") is None:
        die(".init_array is missing from the image -- the linker script always emits "
            "it (KEEP), so this is not a build that can be checked")

    n_entries = 0
    for name, baseline in BASELINE_BYTES.items():
        found = section(args.objdump, args.elf, name)
        if found is None:
            continue
        size, _vma = found
        entries = table_entries(args.objdump, args.elf, name)
        n_entries += len(entries)

        if args.list:
            for i, addr in enumerate(entries):
                print(f"  {name}[{i}] 0x{addr:08x}  {name_at(syms, addr)}")

        if size > baseline:
            listing = ", ".join(f"0x{a:08x} {name_at(syms, a)}" for a in entries) or "(none)"
            failures.append(
                f"{name} is {size} B, above the {baseline} B C-only baseline: static "
                f"initialisation was added.  Entries: {listing}.  These run from "
                f"__libc_init_array BEFORE main -- no MPU, no cache, no usable PSRAM "
                f"window, no dmesg, no ThreadX -- so one that touches the .psram_ai "
                f"carve-out hangs the board leaving nothing behind.  Move the work "
                f"into open() (placement-new) or make the object a plain array."
            )

    # --- 2. no exception / unwind / guard machinery --------------------------
    for pattern, why in FORBIDDEN:
        rx = re.compile(pattern)
        hits = sorted({name for _a, _t, name in syms if rx.search(name)})
        if hits:
            failures.append(f"{', '.join(hits)}: {why}")

    # --- 3. nothing mangled in the cacheable carve-out -----------------------
    bounds = {name: addr for addr, _t, name in syms
              if name in ("_psram_ai_start", "_psram_ai_end")}
    if len(bounds) != 2:
        die("_psram_ai_start/_psram_ai_end not found -- wrong linker script?")
    lo, hi = bounds["_psram_ai_start"], bounds["_psram_ai_end"]
    mangled = sorted({name for addr, typ, name in syms
                      if typ in "bBdD" and lo <= addr < hi and name.startswith("_Z")})
    if mangled:
        failures.append(
            f"{', '.join(mangled)}: C++-mangled object(s) in the .psram_ai carve-out.  "
            f"cmake/check_psram_ai_residency.py matches residents by exact name, so a "
            f"mangled one is invisible to it -- the buffer would be unguarded while the "
            f"gate still reported OK.  Define carve-out buffers in "
            f"port/nn/tflm/nn_tflm_bufs.c (C) and reference them with extern \"C\"."
        )

    if failures:
        print(f"check_cxx_runtime: FAIL ({len(failures)} problem(s)) in {args.elf}",
              file=sys.stderr)
        for f in failures:
            print(f"  - {f}", file=sys.stderr)
        return 1

    # Reported, NOT enforced.  Whether operator new survives --gc-sections says
    # something worth knowing that no runtime counter can: absent, the heap is
    # unreachable from C++ in this image, which is a far stronger statement than
    # `ai info` showing "cxx new: 0" (that reads 0 by construction when the operator was
    # dropped).  It is not a failure either way -- a future backend may legitimately
    # allocate -- so this line exists to make the transition VISIBLE the build it
    # happens in, rather than to forbid it.
    new_linked = any(name.startswith("_Znw") or name.startswith("_Zna")
                     for _a, _t, name in syms)
    heap = ("operator new IS linked (the C++ heap path is reachable; watch "
            "`ai info`'s cxx new counter)" if new_linked else
            "operator new was dropped by --gc-sections (no reachable C++ heap path)")

    print(f"check_cxx_runtime: OK -- {n_entries} static-init entry/entries across "
          f"{len(BASELINE_BYTES)} tables, all within the C-only baseline; no "
          f"exception, unwind or atexit symbols; no mangled names in .psram_ai; "
          f"{heap}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
