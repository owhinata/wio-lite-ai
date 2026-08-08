#!/usr/bin/env python3
"""Post-link guard for the ITCM-resident interrupt paths (issues #24, #29, #39).

WHY THIS EXISTS
---------------
The interrupt paths are placed in ITCM by input-section patterns in
ldscript/STM32H725AEIx_IROM.ld, and that placement fails OPEN: a pattern that
stops matching does not break the link, it just moves the code back to the flash
and quietly gives back the interrupt latency that issues #24/#29 bought.

The linker script has ASSERTs for this, but they cannot be trusted once the
firmware is built with LTO (issue #39).  Each is shaped like

    ASSERT(DEFINED(sym) ? (sym < ORIGIN(ITCM) + LENGTH(ITCM)) : 1, "...")

and LTO renames the symbols it clones -- `_tx_event_flags_set` becomes
`_tx_event_flags_set.isra.0`.  DEFINED() then reports false, the conditional
collapses to `1`, and the ASSERT passes *because* the thing it guards has
disappeared.  The guard is at its most useless exactly when it is needed.

So this checks the linked image instead, on suffix-stripped base names, and adds
a second test the ASSERTs never could:

  1. RESIDENCY -- every symbol whose base name is in REQUIRED_RESIDENTS lies
     inside [_sitcm, _eitcm).  All of them, not just one: LTO can leave `foo` and
     `foo.isra.0` both alive, and a check satisfied by either one would pass while
     the vector table or an external call still reached the flash copy.  A base
     name that matches NOTHING is also an error -- that is precisely the vanished
     symbol the linker-script ASSERT cannot see.

  2. NO LEAKAGE -- nothing inside [_sitcm, _eitcm) references a long-branch veneer
     other than the two we have accepted.  A veneer is the linker's way of
     reaching something too far away to branch to directly, which from inside ITCM
     means "this call left ITCM for the flash".  This catches residency loss for
     code that is NOT in the required list at all, including functions that do not
     exist yet.  It looks at every reference, not at a list of branch opcodes: ITCM
     already contains b.n / bne.n / beq.n / cbz / cbnz / bl, and an opcode whitelist
     is a guard that fails open the first time a veneer is reached by a conditional
     branch or an indirect call.

Exit status 0 = pass; 1 = a residency/leakage failure; 2 = the check could not be
performed (which is also a failure -- a gate that skips itself reports the same
silence as a passing one).
"""

import argparse
import re
import subprocess
import sys

CANNOT_CHECK = 2   # exit code for "the check could not run", never confused with pass

# Base names that must be ITCM-resident.  Deliberately limited to symbols that
# survive LTO as real, callable symbols.
#
# NOT listed, and why: rtl_uart_isr and erpc_rx_ready are static and may be inlined
# out of existence -- under LTO rtl_uart_isr really is inlined into
# UART9_IRQHandler, and USART1_IRQHandler folds to a 4-byte branch into it because
# app/rtl8720.c defines the two handlers with identical bodies.  Requiring them
# would fail a build that is correct.  Test 2 covers whatever they became.
REQUIRED_RESIDENTS = (
    "_tx_thread_schedule",              # ThreadX PendSV (port assembly)
    "_tx_timer_interrupt",              # ThreadX tick (port assembly)
    "SysTick_Handler",                  # tick vector -> HAL_IncTick + ThreadX gate
    "USART1_IRQHandler",                # RTL8720 UART RX (issue #23)
    "UART9_IRQHandler",
    "fault_handler_c",                  # fault handler body (app/fault.c)
    "_txe_event_flags_set",             # RX ISR wake-up path (issue #23 U0-3)
    "_tx_event_flags_set",
    "_tx_thread_system_resume",
    "_tx_thread_system_preempt_check",  # ...and its two tails (issue #29)
    "_tx_timer_system_deactivate",
)

# Veneer targets that are allowed to be reached from inside .itcm.  Both are
# documented one-shot exits, not hot-path leaks:
#
#   log_write         fault_handler_c writes the crash record to the DTCM log ring.
#                     Already noted in the linker script: the fault path buys the
#                     handler's own fetch cost, not log_write's, and it runs once.
#   __NVIC_SystemReset  the reset instruction at the tail of fault_rest().  Inlined
#                     at -O2; under LTO it becomes a call.  It is the last thing
#                     the CPU executes before the reset takes effect.
ALLOWED_VENEER_TARGETS = ("log_write", "__NVIC_SystemReset")

# GCC clone/localisation suffixes.  They stack -- `.lto_priv.0.lto_priv.0` is real
# (observed on __NVIC_SystemReset) -- so stripping is applied repeatedly.  The
# trailing number is optional because `.cold` is emitted without one.
CLONE_SUFFIX_RE = re.compile(
    r"\.(?:isra|constprop|part|lto_priv|cold|localalias)(?:\.\d+)?$")


def strip_clone_suffixes(name):
    """`foo.isra.0.lto_priv.1` -> `foo`."""
    while True:
        stripped = CLONE_SUFFIX_RE.sub("", name)
        if stripped == name:
            return name
        name = stripped


def die(message):
    """Exit with CANNOT_CHECK: the guard did not run, which is not the same as passing."""
    print(f"check_itcm_residency: {message}", file=sys.stderr)
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


def itcm_bounds(syms):
    bounds = {name: addr for addr, _t, name in syms if name in ("_sitcm", "_eitcm")}
    if len(bounds) != 2:
        die("_sitcm/_eitcm not found -- wrong linker script?")
    return bounds["_sitcm"], bounds["_eitcm"]


def check_residency(syms, start, end):
    """Test 1: every individual matching REQUIRED_RESIDENTS is inside ITCM."""
    failures = []
    for required in REQUIRED_RESIDENTS:
        matches = [
            (addr, name)
            for addr, typ, name in syms
            if typ in "tT" and strip_clone_suffixes(name) == required
        ]
        if not matches:
            failures.append(
                f"{required}: no such symbol in the image.  Either it was renamed "
                f"beyond the suffixes this script strips, or it was inlined away -- "
                f"check whether its .itcm selector still matches before relaxing this."
            )
            continue
        for addr, name in matches:
            if not start <= addr < end:
                failures.append(
                    f"{required}: {name} is at 0x{addr:08x}, outside ITCM "
                    f"[0x{start:08x}, 0x{end:08x}) -- its .itcm selector in "
                    f"ldscript/STM32H725AEIx_IROM.ld stopped matching."
                )
    return failures


# An instruction or data line: "  648:\tf000 b956 \tb.w\t8f8 <name>".  Deliberately
# NOT a list of branch mnemonics -- ITCM already contains b.n/bne.n/beq.n/cbz/cbnz/bl
# and a whitelist of opcodes is a guard that fails open the day a veneer is reached by
# a conditional branch or an indirect call.  Any reference to a veneer from inside
# ITCM is a leak, whatever instruction makes it.
INSN_LINE_RE = re.compile(r"^\s*([0-9a-f]+):\s")
SYMREF_RE = re.compile(r"<([^>]+)>")
# objdump's comment separator is the ARM comment character, preceded by a tab or a
# space depending on the operand ("ldr.w pc, [pc]\t@ adc <__log_write_veneer+0x4>").
# Everything after it names where a literal LIVES, not what the instruction calls.
COMMENT_RE = re.compile(r"[ \t]@[ \t]")


def check_veneers(objdump, elf, start, end):
    """Test 2: nothing inside ITCM reaches out through an unexpected veneer."""
    disasm = run([
        objdump, "-d",
        f"--start-address=0x{start:x}", f"--stop-address=0x{end:x}", elf,
    ])
    failures = []
    for line in disasm.splitlines():
        m = INSN_LINE_RE.match(line)
        if not m:
            continue                       # blank line or a "0000ad8 <sym>:" label
        # Drop the annotation, so the two-instruction body of a veneer does not report
        # itself as a call site.
        code = COMMENT_RE.split(line, maxsplit=1)[0]
        for ref in SYMREF_RE.findall(code):
            target = ref.split("+")[0]     # "<sym+0x4>" -> "sym"
            if not target.endswith("_veneer"):
                continue
            base = strip_clone_suffixes(target[: -len("_veneer")])
            # The linker prefixes veneer symbols with "__"; the underlying symbol may
            # itself start with underscores, so compare against the exact expected
            # spellings rather than stripping leading underscores greedily (that would
            # accept a genuinely different symbol such as _log_write as log_write).
            if any(base in (a, "__" + a) for a in ALLOWED_VENEER_TARGETS):
                continue
            failures.append(
                f"0x{m.group(1)}: reference to {target} leaves ITCM.  The callee fell "
                f"back to the flash -- widen its .itcm selector, or add it to "
                f"ALLOWED_VENEER_TARGETS with the reason it may cost a fetch."
            )
    return failures


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("elf")
    ap.add_argument("--nm", required=True)
    ap.add_argument("--objdump", required=True)
    args = ap.parse_args()

    syms = read_symbols(args.nm, args.elf)
    start, end = itcm_bounds(syms)
    if start == end:
        die(".itcm is empty (0 B) -- it links that way silently when its selectors "
            "stop matching or it is placed after .text.  See the placement rules in "
            "the linker script.")

    failures = check_residency(syms, start, end) + check_veneers(
        args.objdump, args.elf, start, end
    )
    if failures:
        print(f"check_itcm_residency: FAIL ({len(failures)} problem(s)) in {args.elf}",
              file=sys.stderr)
        for f in failures:
            print(f"  - {f}", file=sys.stderr)
        return 1

    print(f"check_itcm_residency: OK -- {end - start} B resident, "
          f"{len(REQUIRED_RESIDENTS)} required paths in ITCM, no unexpected veneers")
    return 0


if __name__ == "__main__":
    sys.exit(main())
