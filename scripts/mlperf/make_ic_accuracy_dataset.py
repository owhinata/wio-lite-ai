#!/usr/bin/env python3
"""Build the REAL ic01 accuracy stimulus set from CIFAR-10 (issue #55).

WHY A SCRIPT INSTEAD OF UPSTREAM'S
----------------------------------
Upstream generates these with
`lib/mlperf-tiny/benchmark/training/image_classification/perf_samples_loader.py`,
which does `import train` -- and train.py imports TensorFlow at module scope, purely
because the same file also trains the model.  Extracting 200 test images needs pickle
and numpy and nothing else, so requiring a TensorFlow install for it is an accident of
how that file is organised rather than a real dependency.

🔴 THIS IS NOT A REIMPLEMENTATION THAT HOPES TO MATCH.  Upstream ships the expected
answer -- benchmark/evaluation/datasets/ic01/y_labels.csv, 200 filenames and labels --
and this script CHECKS ITS OUTPUT AGAINST IT and refuses to write anything if the two
disagree.  So either the extraction is byte-identical to upstream's or you get an
error, and there is no third outcome where the accuracy number is quietly measured
against the wrong images.

The one transform that matters: CIFAR-10 stores each image as 1024 R, then 1024 G,
then 1024 B (planar).  The benchmark wants interleaved RGB, "[0]=ulc and [1024]=lrc"
(datasets/README.md) -- which is what upstream's `np.rollaxis(data, 1, 4)` produces.
Get this wrong and every image is still 3,072 bytes, the run still completes, and the
accuracy is merely bad.

USAGE
-----
    # CIFAR-10 python batches: https://www.cs.toronto.edu/~kriz/cifar-10-python.tar.gz
    tar xzf cifar-10-python.tar.gz
    python3 scripts/mlperf/make_ic_accuracy_dataset.py \
        --cifar cifar-10-batches-py --out ~/mlperf-datasets
"""

import argparse
import os
import pickle
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
UPSTREAM = os.path.join(REPO, "lib", "mlperf-tiny", "benchmark")
IDXS = os.path.join(UPSTREAM, "training", "image_classification", "perf_samples_idxs.npy")
REFERENCE_CSV = os.path.join(UPSTREAM, "evaluation", "datasets", "ic01", "y_labels.csv")


def unpickle(path):
    with open(path, "rb") as f:
        return pickle.load(f, encoding="bytes")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cifar", required=True,
                    help="the extracted cifar-10-batches-py directory")
    ap.add_argument("--out", required=True,
                    help="dataset root; ic01/ is created under it")
    args = ap.parse_args()

    for p in (IDXS, REFERENCE_CSV):
        if not os.path.exists(p):
            sys.exit(f"missing {p}\n"
                     f"  the upstream mirror is not checked out; run\n"
                     f"  git submodule update --init --depth 1 -- lib/mlperf-tiny")

    test = unpickle(os.path.join(args.cifar, "test_batch"))

    # (N, 3072) planar -> (N, 32, 32, 3) interleaved, exactly as upstream's
    # load_cifar_10_data() does it.
    data = np.rollaxis(test[b"data"].reshape((-1, 3, 32, 32)), 1, 4)
    names = test[b"filenames"]
    # Upstream one-hots the labels and then argmaxes them back, which is the identity
    # on the integer label; taken directly here.
    labels = np.array(test[b"labels"])

    idxs = np.load(IDXS)

    rows = []
    blobs = []
    for i in idxs:
        # .png -> .bin, upstream's `[:-3] + 'bin'`
        fn = names[i].decode("utf-8")[:-3] + "bin"
        rows.append(f"{fn},10,{labels[i]}")
        blobs.append((fn, bytes(data[i].flatten().tolist())))

    # THE CHECK.  Compare against upstream's own CSV before writing a single file.
    with open(REFERENCE_CSV) as f:
        want = [ln.strip() for ln in f if ln.strip()]
    if rows != want:
        n = sum(1 for a, b in zip(rows, want) if a != b)
        first = next((f"\n  ours     {a}\n  upstream {b}"
                      for a, b in zip(rows, want) if a != b), "")
        sys.exit(f"REFUSING TO WRITE: generated labels differ from upstream's "
                 f"({len(rows)} vs {len(want)} rows, {n} mismatched){first}\n"
                 f"  The CIFAR-10 archive may not be the canonical one.")

    d = os.path.join(args.out, "ic01")
    os.makedirs(d, exist_ok=True)
    for fn, blob in blobs:
        assert len(blob) == 32 * 32 * 3
        with open(os.path.join(d, fn), "wb") as f:
            f.write(blob)
    with open(os.path.join(d, "y_labels.csv"), "w") as f:
        f.write("\n".join(rows) + "\n")

    print(f"ic01: {len(blobs)} x 3072 B -> {d}")
    print(f"labels match upstream's {os.path.relpath(REFERENCE_CSV, REPO)} exactly")
    print()
    print("This IS an accuracy dataset.  Run it with --mode a; the closed-division")
    print("target for ic01 is 85% top-1.")


if __name__ == "__main__":
    main()
