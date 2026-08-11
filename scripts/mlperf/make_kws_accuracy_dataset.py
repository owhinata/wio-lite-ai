#!/usr/bin/env python3
"""Build the REAL kws01 accuracy stimulus set from Speech Commands v2 (issue #56).

THIS IS THE ONE THAT ACTUALLY NEEDS TENSORFLOW
----------------------------------------------
ic01, vww01 and ad01 all turned out not to (see the other three scripts in this
directory).  kws01 does, twice over, and neither is avoidable by being careful:

  1. THE FEATURES.  The stimulus is a 49x10 MFCC spectrogram produced by tf.signal --
     STFT, a mel weight matrix, log, DCT.  Every one of those has a numpy equivalent,
     and a reimplementation would agree to within a float rounding error that mostly
     survives the cast to int8.  "Mostly" is the problem: there is no way to tell a
     one-LSB reimplementation difference from a real one, because unlike ic01 there is
     no upstream artefact to compare the FEATURES against -- only the labels.

  2. THE ORDER.  The 1,000 samples are the first 1,000 of tensorflow_datasets'
     `speech_commands` test split, and that order is not the order of the files in the
     archive: TFDS shuffles records into shards by a hash of each example key when it
     writes them.  Upstream's y_labels.csv is a list of labels IN THAT ORDER, so
     reproducing it means reproducing TFDS's shuffle -- which is a version-fragile
     reimplementation of a hash bucketing scheme, to avoid installing the library that
     performs it.

So this script imports upstream's own get_dataset.py and calls it, exactly as their
make_bin_files.py does.  What it adds is the check, and not writing the plots.

WHAT IS CHECKED
---------------
Upstream ships benchmark/evaluation/datasets/kws01/y_labels.csv: 1,000 rows of
`tst_<index>_<word>_<label>.bin,12,<label>`.  Every field of every row is a consequence
of the split order -- the index, the spelled-out word and the numeric label all come
from the same iteration this script performs.  It builds those rows and compares them
against upstream's before writing a single file, and stops if they differ.  1,000
positions over 12 classes do not line up by accident, so a match means the iteration
landed on the same samples in the same order that produced the shipped ground truth.

🔴 IT SAYS NOTHING ABOUT THE MFCC VALUES, which is why the features come from upstream's
code rather than from a reimplementation, and why the output should still be scored on
the host before the board is asked for half an hour:

    python3 scripts/mlperf/score_dataset.py --bench kws01 \
        --dataset ~/mlperf-datasets/kws01 \
        --model _ref/mlperf-tiny/models/kws01_dscnn_int8.tflite

The reference model is documented at 92% on this test set and the closed-division
target is 90%, so a feature extraction that has drifted has very little room to hide.

USAGE
-----
    # A TensorFlow environment; see scripts/mlperf/README.md.  TFDS downloads
    # ~2.5 GB the first time and prepares it into data_dir.
    python3 scripts/mlperf/make_kws_accuracy_dataset.py \\
        --tfds-dir ~/mlperf-src/tfds --out ~/mlperf-datasets
"""

import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
UPSTREAM_KWS = os.path.join(REPO, "lib", "mlperf-tiny", "benchmark", "training",
                            "keyword_spotting")
DEFAULT_MODEL = os.path.join(UPSTREAM_KWS, "trained_models", "kws_ref_model.tflite")

NUM_LABELS = 12
SAMPLE_BYTES = 49 * 10      # int8 MFCCs, one byte each


def y_labels_path():
    for p in (os.path.join(REPO, "lib", "mlperf-tiny", "benchmark", "evaluation",
                           "datasets", "kws01", "y_labels.csv"),
              os.path.join(REPO, "_ref", "mlperf-tiny", "y_labels", "kws01",
                           "y_labels.csv")):
        if os.path.exists(p):
            return p
    sys.exit("no y_labels.csv for kws01; check out the upstream mirror with\n"
             "  git submodule update --init --depth 1 -- lib/mlperf-tiny")


def read_rows(path):
    """Rows as (name, classes, label), whitespace-normalised.

    make_bin_files.py writes ", " separators; the copy upstream ships under
    evaluation/datasets has none.  Comparing raw lines would fail on that alone.
    """
    out = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            p = [x.strip() for x in line.split(",")]
            out.append((p[0], int(p[1]), int(p[2])))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tfds-dir", required=True,
                    help="tensorflow_datasets data_dir for speech_commands")
    ap.add_argument("--out", required=True,
                    help="dataset root; kws01/ is created under it")
    ap.add_argument("--model", default=DEFAULT_MODEL,
                    help="the reference .tflite whose input scale/zero point quantizes "
                         "the features (default: upstream's kws_ref_model.tflite)")
    ap.add_argument("--count", type=int, default=1000,
                    help="how many test samples (upstream's default is 1000, and the "
                         "shipped ground truth has exactly that many)")
    args = ap.parse_args()

    truth_csv = y_labels_path()
    want = read_rows(truth_csv)

    if not os.path.isdir(UPSTREAM_KWS):
        sys.exit(f"missing {UPSTREAM_KWS}\n"
                 f"  git submodule update --init --depth 1 -- lib/mlperf-tiny")

    # Upstream's modules read quant_cal_idxs.txt and default --bg_path from the cwd,
    # so they are imported and called from their own directory.  Nothing writes there:
    # there is no _background_noise_ directory to find, and the plotting in
    # make_bin_files.py is the part this script does not reproduce.
    os.chdir(UPSTREAM_KWS)
    sys.path.insert(0, UPSTREAM_KWS)

    import tensorflow as tf
    import get_dataset as kws_data
    import kws_util

    # parse_command() rather than a hand-built Namespace: it is where every default
    # that shapes the features lives (16 kHz, 30 ms window, 20 ms stride, 10 DCT
    # coefficients), and restating them here is how those quietly drift apart.
    sys.argv = ["make_kws_accuracy_dataset",
                "--data_dir", os.path.abspath(os.path.expanduser(args.tfds_dir)),
                "--feature_type", "mfcc",
                "--target_set", "test",
                "--num_bin_files", str(args.count)]
    Flags, _ = kws_util.parse_command()

    # 🔴 The quantization comes from the MODEL, as make_bin_files.py does it -- the
    # runner sends kws01 already quantized (upstream's datasets/README.md: "49 frames x
    # 10 MFCCs as INT8"), so this is the one benchmark where the host decides the
    # tensor's representation and the firmware copies the bytes verbatim.
    interpreter = tf.lite.Interpreter(model_path=os.path.abspath(args.model))
    interpreter.allocate_tensors()
    input_scale, input_zero_point = interpreter.get_input_details()[0]["quantization"]
    print(f"kws01: scale={input_scale} zero_point={input_zero_point} "
          f"(from {os.path.relpath(os.path.abspath(args.model), REPO)})")

    print("building the test split (the first run prepares ~2.5 GB) ...")
    _, ds_test, _ = kws_data.get_training_data(Flags, val_cal_subset=True)

    rows, blobs = [], []
    eval_data = ds_test.unbatch().batch(1).take(args.count).as_numpy_iterator()
    for count, (dat, label) in enumerate(eval_data):
        # Verbatim from make_bin_files.py, including the truncating cast: np.array(...,
        # dtype=int8) rounds toward zero rather than to nearest.  That is not what
        # anyone would choose, but it IS what produced the reference stimulus set, and
        # this script's job is to produce that set rather than a better one.
        dat_q = np.array(dat / input_scale + input_zero_point, dtype=np.int8)
        label_str = kws_data.word_labels[label[0]]
        fname = f"tst_{count:06d}_{label_str}_{label[0]}.bin"
        rows.append((fname, NUM_LABELS, int(label[0])))
        blobs.append(dat_q.flatten().tobytes())

    # THE CHECK.  Against upstream's own CSV, before writing a single file.
    if rows != want:
        n = sum(1 for a, b in zip(rows, want) if a != b)
        first = next((f"\n  ours     {a}\n  upstream {b}"
                      for a, b in zip(rows, want) if a != b), "")
        sys.exit(f"REFUSING TO WRITE: generated labels differ from upstream's "
                 f"({len(rows)} vs {len(want)} rows, {n} mismatched){first}\n"
                 f"  The test split is not iterating in the order that produced\n"
                 f"  {os.path.relpath(truth_csv, REPO)}.  A tensorflow_datasets version\n"
                 f"  that shards speech_commands differently would do this.")

    out_dir = os.path.join(os.path.abspath(os.path.expanduser(args.out)), "kws01")
    os.makedirs(out_dir, exist_ok=True)
    for (fname, _, _), blob in zip(rows, blobs):
        if len(blob) != SAMPLE_BYTES:
            sys.exit(f"REFUSING TO WRITE: {fname} is {len(blob)} B, expected "
                     f"{SAMPLE_BYTES} (49 frames x 10 coefficients)")
        with open(os.path.join(out_dir, fname), "wb") as f:
            f.write(blob)
    with open(os.path.join(out_dir, "y_labels.csv"), "w") as f:
        for fname, classes, label in rows:
            f.write(f"{fname},{classes},{label}\n")

    print(f"kws01: {len(blobs)} x {SAMPLE_BYTES} B -> {out_dir}")
    print(f"labels match upstream's {os.path.relpath(truth_csv, REPO)} exactly")
    print()
    print("This IS an accuracy dataset.  Run it with --mode a; the closed-division")
    print("target for kws01 is 90% top-1.  Score it on the host first.")


if __name__ == "__main__":
    main()
