#!/usr/bin/env python3
"""Report the deepest stack frames in the TFLM library (issue #9 phase 2c).

THIS IS A REPORT, NOT A GATE.  It always exits 0.  It exists because the stack budget
on this board is the one resource the TFLM port cannot simply be given more of:

  - the CLI thread that runs `ai bench` has a 4,096 B stack (CLI_INSTANCE_STACK_SIZE),
  - every thread stack lives in .dtcm_bss, and DTCM has under 8 KB free once the main
    stack is accounted for (issue #46), so there is very little to hand over,
  - and the donor firmware's evidence that this fits is its own 4,096 B shell thread
    running the same model -- but at GCC 13.3, with a different CMSIS-NN revision.
    Kernel frame sizes are exactly the sort of thing that changes across those.

A `free` high-water reading from the board answers the question for the paths that
ACTUALLY RAN.  This answers it for the paths that exist, before the image is flashed,
which is the cheaper order to learn it in.  Neither is redundant: -fstack-usage reports
per-function frames and cannot see call depth, so a deep chain of small frames does not
show up here.  Read the two together.

Frames are collected from the .su files -fstack-usage writes next to each object.
"""

import argparse
import os
import sys

# Frames at or above this are worth a second look on a 4 KB thread stack: a single one
# is already 12% of it, and inference nests kernels several deep.
NOTABLE_BYTES = 512


def collect(root):
    """[(bytes, qualifier, function, file)] from every .su file under @root."""
    frames = []
    for dirpath, _dirs, files in os.walk(root):
        for fn in files:
            if not fn.endswith(".su"):
                continue
            path = os.path.join(dirpath, fn)
            try:
                with open(path, encoding="utf-8", errors="replace") as fh:
                    for line in fh:
                        # "file:line:col:signature\tbytes\tqualifier".  The signature
                        # itself contains colons (C++ scope resolution), so split from
                        # the right on tabs first and never on colons.
                        parts = line.rstrip("\n").split("\t")
                        if len(parts) < 2:
                            continue
                        loc, size = parts[0], parts[1]
                        qual = parts[2] if len(parts) > 2 else ""
                        try:
                            size = int(size)
                        except ValueError:
                            continue
                        bits = loc.split(":", 3)
                        func = bits[3] if len(bits) == 4 else loc
                        frames.append((size, qual, func, os.path.basename(bits[0])))
            except OSError:
                continue
    return frames


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("objdir", help="the tflm object directory (holds the .su files)")
    ap.add_argument("--top", type=int, default=10, help="how many frames to list")
    args = ap.parse_args()

    frames = collect(args.objdir)
    if not frames:
        # Not fatal, and deliberately not silent: -fstack-usage may have been dropped,
        # or the objects may live somewhere this path does not reach.
        print(f"report_tflm_stack: no .su files under {args.objdir} -- "
              f"stack usage was NOT measured (is -fstack-usage still on the tflm "
              f"target?)")
        return 0

    frames.sort(key=lambda f: -f[0])
    worst = frames[0][0]
    notable = sum(1 for f in frames if f[0] >= NOTABLE_BYTES)

    print(f"report_tflm_stack: {len(frames)} functions, deepest single frame {worst} B, "
          f"{notable} at/above {NOTABLE_BYTES} B (CLI thread stack is 4096 B)")
    for size, qual, func, src in frames[:args.top]:
        tag = f" [{qual}]" if qual and qual != "static" else ""
        print(f"    {size:6d} B  {func}  ({src}){tag}")
    if any(q.startswith("dynamic") for _s, q, _f, _x in frames):
        print("    note: a `dynamic` frame uses alloca/VLA -- its real size is not a "
              "build-time constant and is NOT bounded by the number above")
    return 0


if __name__ == "__main__":
    sys.exit(main())
