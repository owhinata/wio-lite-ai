#!/usr/bin/env python3
"""Build the REAL vww01 accuracy stimulus set from the Visual Wake Words images (#56).

WHY THIS IS NOT THE 13 GB JOB THE ISSUE EXPECTED
------------------------------------------------
Upstream's visual_wake_words/download_and_train_vww.sh does not start from COCO2014 at
all -- it fetches Silicon Labs' `vw_coco2014_96.tar.gz`, which is the Visual Wake Words
selection ALREADY cropped and resized to 96x96 and already sorted into person/ and
non_person/.  That tarball is 235 MB, and it is the only download this script needs.
No TensorFlow: turning a 96x96 JPEG into 96x96x3 raw bytes is PIL and nothing else.

(The 13 GB figure is what it would cost to rebuild that tarball from COCO2014 plus the
VWW annotations.  Doing so would also mean guessing at Silicon Labs' crop, which is not
published as code here -- so it would be strictly worse than using their output.)

WHAT IS CHECKED, AND WHAT THAT DOES AND DOES NOT PROVE
-----------------------------------------------------
Upstream ships benchmark/evaluation/datasets/vww01/y_labels.csv: 1,000 COCO image ids
with a person/no-person label each.  The tarball sorts its 109,619 images into two
directories by the same property.  This script requires, for all 1,000 rows, that the
id resolves to exactly one image in the tarball AND that the directory it lives in
agrees with the CSV's label -- and writes nothing at all if even one disagrees.  Two
independent sources of the same ground truth have to match before any bytes are
produced, which is what rules out the failure that matters: silently pairing the right
label with the wrong picture.

🔴 IT DOES NOT PROVE THE PIXELS ARE BYTE-IDENTICAL TO EEMBC'S .bin FILES, AND THEY ARE
NOT.  Those were written straight from the resize; ours are decoded from the JPEGs
Silicon Labs saved from that same resize, so every sample carries one generation of
JPEG loss.  That is a real difference and the reason this script has a --verify mode:
run the reference model over the output before spending half an hour on the board.  A
95%-quality 96x96 JPEG costs a fraction of a point, but "should be fine" is not a
measurement.

USAGE
-----
    curl -LO https://www.silabs.com/public/files/github/machine_learning/benchmarks/\
datasets/vw_coco2014_96.tar.gz
    python3 scripts/mlperf/make_vww_accuracy_dataset.py \
        --vww vw_coco2014_96.tar.gz --out ~/mlperf-datasets

    # then, before running the board (needs a tflite interpreter -- see README):
    python3 scripts/mlperf/score_dataset.py \
        --bench vww01 --dataset ~/mlperf-datasets/vww01 \
        --model _ref/mlperf-tiny/models/vww01_mobilenet_int8.tflite
"""

import argparse
import io
import os
import re
import sys
import tarfile

from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))

WIDTH = HEIGHT = 96
CHANNELS = 3
SAMPLE_BYTES = WIDTH * HEIGHT * CHANNELS

# vw_coco2014_96/person/COCO_train2014_000000308086.jpg -- the tarball keeps COCO's
# 2014 filenames, upstream's CSV uses the bare 12-digit image id (COCO's 2017 naming
# for the same images), so the id is the join key.
MEMBER_RE = re.compile(
    r"(?:^|/)vw_coco2014_96/(person|non_person)/"
    r"COCO_(?:train|val)2014_(\d{12})\.jpg$"
)


def y_labels_path(bench):
    """Upstream's ground truth, from the submodule or from the _ref copy of it."""
    for p in (os.path.join(REPO, "lib", "mlperf-tiny", "benchmark", "evaluation",
                           "datasets", bench, "y_labels.csv"),
              os.path.join(REPO, "_ref", "mlperf-tiny", "y_labels", bench,
                           "y_labels.csv")):
        if os.path.exists(p):
            return p
    sys.exit(f"no y_labels.csv for {bench}; check out the upstream mirror with\n"
             f"  git submodule update --init --depth 1 -- lib/mlperf-tiny")


def read_y_labels(path):
    """[(filename, num_classes, true_class)], whitespace-normalised.

    Upstream's generator writes ", " separators and the shipped copies have none, so
    the fields are stripped rather than compared as raw lines.
    """
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = [p.strip() for p in line.split(",")]
            if len(parts) < 3:
                sys.exit(f"malformed row in {path}: {line!r}")
            rows.append((parts[0], int(parts[1]), int(parts[2])))
    return rows


def collect(source, wanted):
    """id -> (directory, decoded RGB bytes), for the ids in @p wanted.

    🔴 ONE PASS, AND THAT IS WHAT MAKES THE CHECK MEAN ANYTHING.  An earlier version
    walked the tarball once to record where each id lived, checked those directories
    against upstream's labels, and then walked it AGAIN to read the pixels -- so what
    was verified and what was written came from two different reads of the file, and
    anything that changed in between would be written unverified.  Decoding during the
    same pass that records the directory removes the gap rather than narrowing it: the
    bytes that get written are the bytes that were checked.

    It is also simply less work.  The tarball holds 109,619 images and only 1,000 are
    wanted, so filtering before decoding costs one pass instead of two over 235 MB.
    """
    found = {}
    dups = []

    def add(image_id, folder, raw, name):
        if image_id not in wanted:
            return
        if image_id in found:
            dups.append(image_id)
            return
        found[image_id] = (folder, decode(raw, name))

    if os.path.isdir(source):
        for folder in ("person", "non_person"):
            d = os.path.join(source, folder)
            if not os.path.isdir(d):
                continue
            for name in sorted(os.listdir(d)):
                m = MEMBER_RE.search(f"vw_coco2014_96/{folder}/{name}")
                if not m or m.group(2) not in wanted:
                    continue
                with open(os.path.join(d, name), "rb") as f:
                    add(m.group(2), folder, f.read(), name)
    else:
        with tarfile.open(source, "r:gz") as tar:
            for member in tar:
                if not member.isfile():
                    continue
                m = MEMBER_RE.search(member.name)
                if not m or m.group(2) not in wanted:
                    continue
                add(m.group(2), m.group(1), tar.extractfile(member).read(),
                    member.name)

    if dups:
        sys.exit(f"REFUSING TO WRITE: {len(dups)} image ids appear more than once in "
                 f"{source} (first: {dups[0]}).\n"
                 f"  The id is the only join key to upstream's labels, so a duplicate "
                 f"means the pairing is ambiguous.")
    return found


def decode(raw, name):
    with Image.open(io.BytesIO(raw)) as im:
        if im.size != (WIDTH, HEIGHT):
            sys.exit(f"REFUSING TO WRITE: {name} is {im.size[0]}x{im.size[1]}, "
                     f"expected {WIDTH}x{HEIGHT}")
        # "U8C3, RGB, where [0]=ulc and [9215]=lrc" (upstream datasets/README.md), i.e.
        # interleaved and row-major from the top left, which is PIL's own byte order.
        blob = im.convert("RGB").tobytes()
    if len(blob) != SAMPLE_BYTES:
        sys.exit(f"REFUSING TO WRITE: {name} decoded to {len(blob)} B, "
                 f"expected {SAMPLE_BYTES}")
    return blob


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--vww", required=True,
                    help="vw_coco2014_96.tar.gz, or the directory it extracts to")
    ap.add_argument("--out", required=True,
                    help="dataset root; vww01/ is created under it")
    args = ap.parse_args()

    truth_csv = y_labels_path("vww01")
    rows = read_y_labels(truth_csv)

    wanted = {name[:-4] if name.endswith(".bin") else name for name, _, _ in rows}

    print(f"reading {args.vww} ...")
    images = collect(args.vww, wanted)
    print(f"  {len(images)} of {len(wanted)} wanted images found")

    # THE CHECK, over every row, before a single file is written -- against the very
    # bytes collect() decoded, not against a second look at the source.
    missing, mislabelled = [], []
    for name, _, true_class in rows:
        image_id = name[:-4] if name.endswith(".bin") else name
        entry = images.get(image_id)
        if entry is None:
            missing.append(name)
            continue
        folder = entry[0]
        if (1 if folder == "person" else 0) != true_class:
            mislabelled.append((name, folder, true_class))

    if missing or mislabelled:
        msg = [f"REFUSING TO WRITE: the images and upstream's "
               f"{os.path.relpath(truth_csv, REPO)} disagree."]
        if missing:
            msg.append(f"  {len(missing)} of {len(rows)} ids are not in {args.vww} "
                       f"(first: {missing[0]})")
            msg.append("  Is this vw_coco2014_96.tar.gz and not some other VWW build?")
        if mislabelled:
            n, folder, want = mislabelled[0]
            msg.append(f"  {len(mislabelled)} ids sit in the wrong directory "
                       f"(first: {n} is under {folder}/ but labelled {want})")
        sys.exit("\n".join(msg))

    out_dir = os.path.join(args.out, "vww01")
    os.makedirs(out_dir, exist_ok=True)
    for name, _, _ in rows:
        with open(os.path.join(out_dir, name), "wb") as f:
            f.write(images[name[:-4]][1])
    with open(os.path.join(out_dir, "y_labels.csv"), "w") as f:
        for name, n_classes, true_class in rows:
            f.write(f"{name},{n_classes},{true_class}\n")

    n_person = sum(1 for _, _, c in rows if c == 1)
    print(f"vww01: {len(rows)} x {SAMPLE_BYTES} B -> {out_dir}")
    print(f"  {n_person} person / {len(rows) - n_person} non-person, every id resolved "
          f"to one image whose directory agrees with "
          f"{os.path.relpath(truth_csv, REPO)}")
    print()
    print("This IS an accuracy dataset.  Run it with --mode a; the closed-division")
    print("target for vww01 is 80% top-1.  Score it on the host first -- these pixels")
    print("come through Silicon Labs' JPEGs, not from EEMBC's own .bin files.")


if __name__ == "__main__":
    main()
