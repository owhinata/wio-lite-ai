#!/usr/bin/env python3
"""Build a minimal PERFORMANCE-ONLY stimulus set for the MLPerf Tiny runner (issue #55).

WHY THIS EXISTS
---------------
The runner needs, for each benchmark, a `y_labels.csv` and the stimulus files it
names.  Upstream ships the CSVs (benchmark/evaluation/datasets/) but not the files:
those are generated from the source datasets, which between them are several tens of
gigabytes -- Speech Commands, COCO/Visual Wake Words, ToyADMOS -- and take a
TensorFlow install to process.

A LATENCY measurement does not need any of that.  None of these four models branches
on its input: the same operators run over the same tensor shapes whatever the bytes
are, so wall-clock per inference is data-independent.  What the runner actually
requires to produce a throughput number is files of the RIGHT SIZE, enough of them to
feed the loop, and a CSV that names them.

🔴 SO THIS IS NOT AN ACCURACY DATASET, AND MUST NEVER BE USED AS ONE.
The classifications the board returns from these bytes are meaningless.  `--mode a`
against this set will produce a number, and that number will be nonsense -- which is
exactly the kind of result that gets copied into a table and believed.  For accuracy,
generate the real stimulus files; README.md says how, per benchmark.

The files are deterministic (seeded), so two runs on two machines compare like for
like, and a latency difference is a difference in the board rather than in the input.

USAGE
-----
    python3 scripts/mlperf/make_perf_dataset.py --out ~/mlperf-datasets
    # then
    python3 main.py --dataset_path ~/mlperf-datasets --mode p ...
"""

import argparse
import os
import struct

# One entry per benchmark the board can run.  `nbytes` is what the runner sends per
# stimulus, taken from the benchmark definitions in
# lib/mlperf-tiny/benchmark/evaluation/datasets/README.md:
#
#   ic01   32x32x3   u8 RGB, upper-left first
#   vww01  96x96x3   u8 RGB
#   kws01  49x10     int8 MFCC, already quantized by the host
#   ad01   5x128     float32 LE spectrogram slices, sent as a sliding window
#
# `count` is how many distinct files the performance script consumes: every
# tests_performance.yaml entry is `loop 5` over `download` + `infer`, so five is
# enough and more would only sit unused on disk.
BENCHMARKS = {
    "ic01":  dict(nbytes=32 * 32 * 3, count=5, classes=10),
    "vww01": dict(nbytes=96 * 96 * 3, count=5, classes=2),
    "kws01": dict(nbytes=49 * 10,     count=5, classes=12),
    # ad01 is the odd one: the CSV carries a window width and a stride, and the runner
    # slices the file rather than sending it whole.  The file therefore has to be at
    # least one window long; three windows' worth keeps the sliding-window path honest
    # without making accuracy-shaped promises.
    "ad01":  dict(nbytes=2560 * 3,    count=5, classes=2, window=2560, stride=512),
}


def name_seed(name):
    """A stable per-benchmark seed (FNV-1a over the name)."""
    h = 0x811C9DC5
    for b in name.encode():
        h = ((h ^ b) * 0x01000193) & 0xFFFFFFFF
    return h


def prng(seed):
    """A tiny deterministic byte source.

    Not `random` and not numpy: this has to produce the identical file on any Python
    on any machine, and the standard library's generator is explicitly allowed to
    change between releases.  A 32-bit xorshift is a page of arithmetic that cannot.
    """
    x = seed & 0xFFFFFFFF or 0x12345678
    while True:
        x ^= (x << 13) & 0xFFFFFFFF
        x ^= x >> 17
        x ^= (x << 5) & 0xFFFFFFFF
        yield x & 0xFF


def make_bytes(n, seed):
    g = prng(seed)
    return bytes(next(g) for _ in range(n))


def make_floats(n_floats, seed):
    """float32 samples in a plausible log-spectrogram range.

    The magnitude matters a little even for a latency run: ad01's input is quantized
    on the board with the model's own scale, and values far outside the range would
    all saturate to one code.  That would still time correctly, but a saturated tensor
    is a confusing thing to find when debugging something else.
    """
    g = prng(seed)
    out = bytearray()
    for _ in range(n_floats):
        v = -6.0 + (next(g) / 255.0) * 12.0
        out += struct.pack("<f", v)
    return bytes(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True,
                    help="dataset root; one subdirectory per benchmark is created")
    ap.add_argument("--only", nargs="*", choices=sorted(BENCHMARKS),
                    help="generate only these benchmarks (default: all)")
    args = ap.parse_args()

    wanted = args.only or sorted(BENCHMARKS)
    for name in wanted:
        spec = BENCHMARKS[name]
        d = os.path.join(args.out, name)
        os.makedirs(d, exist_ok=True)

        rows = []
        for i in range(spec["count"]):
            fn = f"perf_{i:03d}.bin"
            # NOT hash(name): Python randomises string hashing per process unless
            # PYTHONHASHSEED is set, so that would give a different file every run --
            # the exact opposite of what this generator promises.
            seed = name_seed(name) + i * 7919 + 1
            if name == "ad01":
                blob = make_floats(spec["nbytes"] // 4, seed)
            else:
                blob = make_bytes(spec["nbytes"], seed)
            with open(os.path.join(d, fn), "wb") as f:
                f.write(blob)

            # The CSV columns the runner reads (benchmark/runner/datasets.py):
            # file, total classes, true class, and -- for ad01 only -- the window
            # width and stride it slices the file with.
            label = i % spec["classes"]
            if name == "ad01":
                rows.append(f"{fn},{spec['classes']},{label},"
                            f"{spec['window']},{spec['stride']}")
            else:
                rows.append(f"{fn},{spec['classes']},{label}")

        with open(os.path.join(d, "y_labels.csv"), "w") as f:
            f.write("\n".join(rows) + "\n")

        print(f"{name}: {spec['count']} x {spec['nbytes']} B -> {d}")

    print()
    print("PERFORMANCE ONLY.  These bytes are synthetic; the classifications the")
    print("board returns from them mean nothing.  Do not run --mode a against this")
    print("set -- see scripts/mlperf/README.md for the real stimulus files.")


if __name__ == "__main__":
    main()
