#!/usr/bin/env python3
"""Build the REAL ad01 accuracy stimulus set from ToyADMOS ToyCar (issue #56).

NO TENSORFLOW, AND NOT BY CLEVERNESS
------------------------------------
The other two benchmarks in this issue needed a workaround to avoid TensorFlow.  This
one does not: upstream's own converter is
`lib/mlperf-tiny/benchmark/training/anomaly_detection/convert_dataset.py`, and the
feature extraction it calls -- `common.file_to_vector_array()` -- is librosa, numpy and
nothing else.  Anomaly detection is DCASE 2020 Task 2 code; TensorFlow enters only when
the autoencoder is trained, which is not what this does.

So this script does not reimplement the transform, it CALLS upstream's.  The mel
spectrogram, the log, the [:, 50:250] crop and the float32 byte order all come from
their file, and the parameters come from their baseline.yaml (128 mels, 1024-point FFT,
512 hop, power 2.0) rather than from constants retyped here.  The one thing it adds is
the check below and the plumbing to put the output where the runner looks.

WHAT IS CHECKED
---------------
Upstream ships benchmark/evaluation/datasets/ad01/y_labels.csv -- 248 filenames with a
normal/anomaly label each -- and the ToyCar archive names its own files `normal_*` and
`anomaly_*`.  That is two independent statements of the same ground truth, and this
script requires every one of the 248 rows to agree with the archive's own naming, and
every named wav to exist, before it writes anything.  It then requires each generated
file to be exactly 102,400 B, which is 200 frames x 128 bins x 4 B -- the size is a
consequence of the mel parameters and the crop, so getting it means those were right.

🔴 WHAT IT DOES NOT CHECK IS THE FLOAT VALUES.  librosa is not bit-stable across major
versions (0.10 changed the default STFT padding, among others), and upstream pinned
0.6.0 in 2020.  The centre crop makes padding irrelevant here, but "irrelevant" is an
argument, not a measurement -- so score the output on the host before running the
board:

    python3 scripts/mlperf/score_dataset.py --bench ad01 \
        --dataset ~/mlperf-datasets/ad01 \
        --model lib/mlperf-tiny/benchmark/training/anomaly_detection/\
trained_models/ad01_int8.tflite

The closed-division target is 0.85 AUC and upstream's reference reports 0.864 averaged
per machine id, so there is not much room: a materially different feature extraction
shows up as a materially lower AUC, and this is where you would see it.

USAGE
-----
    # ~1.8 GB; only ToyCar/test is used, and only 248 files of it
    curl -L -o dev_data_ToyCar.zip \\
         'https://zenodo.org/record/3678171/files/dev_data_ToyCar.zip?download=1'
    unzip -q dev_data_ToyCar.zip -d dev_data
    python3 scripts/mlperf/make_ad_accuracy_dataset.py \\
        --toycar dev_data/ToyCar --out ~/mlperf-datasets

Both of upstream's evaluation sets are written: y_labels.csv (248 files) and
y_labels_alt.csv (72 of the same files, accepted for submissions since v1.2).  The
stimulus files are shared -- alt is a strict subset -- and which one runs is a choice
made in the test script, not here.  It matters: ad01 accuracy sends every 2,560-byte
window of every file separately, so the full set is 48,608 inferences and the alt set
is 14,112.
"""

import argparse
import atexit
import os
import shutil
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
UPSTREAM_AD = os.path.join(REPO, "lib", "mlperf-tiny", "benchmark", "training",
                           "anomaly_detection")

FRAMES_KEPT = 200          # log_mel_spectrogram[:, 50:250] in file_to_vector_array()
BYTES_PER_FLOAT = 4


def y_labels_path(name):
    p = os.path.join(REPO, "lib", "mlperf-tiny", "benchmark", "evaluation",
                     "datasets", "ad01", name)
    if not os.path.exists(p):
        sys.exit(f"no {name} for ad01; check out the upstream mirror with\n"
                 f"  git submodule update --init --depth 1 -- lib/mlperf-tiny")
    return p


def read_y_labels(path):
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            p = [x.strip() for x in line.split(",")]
            if len(p) != 5:
                sys.exit(f"{path}: ad01 rows carry a window and a stride; got {line!r}")
            rows.append(dict(file=p[0], classes=int(p[1]), cls=int(p[2]),
                             window=int(p[3]), stride=int(p[4])))
    return rows


def import_upstream():
    """common.file_to_vector_array() and the parameters baseline.yaml sets for it.

    common.py points logging at ./baseline.log and yaml_load() reads ./baseline.yaml,
    both relative to the cwd -- it is written to be run from its own directory.  So the
    import happens from a scratch directory (the log is an artefact of importing, not
    something we want in the caller's tree or in the read-only mirror), and the
    parameters are read directly from upstream's file by path.

    The scratch directory is removed at exit rather than immediately: the FileHandler
    that import installs holds baseline.log open for the life of the process, and
    pulling the directory out from under it would leave it writing to an unlinked file.
    sys.path is restored for the same reason the cwd is -- upstream's directory has
    modules named common, keras_model and train, which are exactly the names a later
    import in a bigger program would collide with.
    """
    import yaml

    if not os.path.isdir(UPSTREAM_AD):
        sys.exit(f"missing {UPSTREAM_AD}\n"
                 f"  git submodule update --init --depth 1 -- lib/mlperf-tiny")
    with open(os.path.join(UPSTREAM_AD, "baseline.yaml")) as f:
        param = yaml.safe_load(f)["feature"]

    cwd = os.getcwd()
    saved_path = list(sys.path)
    tmp = tempfile.mkdtemp(prefix="mlperf-ad-")
    atexit.register(shutil.rmtree, tmp, True)
    try:
        os.chdir(tmp)
        sys.path.insert(0, UPSTREAM_AD)
        import common
    finally:
        os.chdir(cwd)
        sys.path[:] = saved_path
    return common, param


def wav_for(bin_name):
    """normal_id_01_00000003_hist_librosa.bin -> normal_id_01_00000003.wav

    The suffix is not decoration: file_to_vector_array() derives it as
    `file_name.replace('.wav', '_hist_' + method + '.bin')`, so the name records that
    the features came from the librosa path rather than one of the alternatives that
    function used to offer.
    """
    stem = bin_name
    for suffix in ("_hist_librosa.bin", ".bin"):
        if stem.endswith(suffix):
            stem = stem[:-len(suffix)]
            break
    return stem + ".wav"


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--toycar", required=True,
                    help="the extracted ToyCar directory (it must contain test/)")
    ap.add_argument("--out", required=True,
                    help="dataset root; ad01/ is created under it")
    args = ap.parse_args()

    full_csv, alt_csv = y_labels_path("y_labels.csv"), y_labels_path("y_labels_alt.csv")
    rows = read_y_labels(full_csv)
    alt_rows = read_y_labels(alt_csv)

    test_dir = os.path.join(args.toycar, "test")
    if not os.path.isdir(test_dir):
        sys.exit(f"missing {test_dir}\n"
                 f"  --toycar wants the directory holding train/ and test/, i.e. the "
                 f"ToyCar/ inside what dev_data_ToyCar.zip extracts to")

    # THE CHECK, over every row, before a single feature is computed.
    missing, mislabelled, unnamed = [], [], []
    for row in rows:
        wav = wav_for(row["file"])
        if not os.path.exists(os.path.join(test_dir, wav)):
            missing.append(wav)
        # The archive states the label twice: once in upstream's CSV, once in its own
        # filename.  Requiring both means a mismatched or reordered download is caught
        # here rather than surfacing as a mysteriously poor AUC.
        # Spelled as a three-way test rather than `1 if anomaly_ else 0`: the archive's
        # OTHER naming scheme (eval_data ships `id_05_*.wav` with no verdict in the
        # name) would read as "normal" under a two-way test and then AGREE with every
        # normal row, so the check would pass on a dataset it never inspected.
        if wav.startswith("anomaly_"):
            expect = 1
        elif wav.startswith("normal_"):
            expect = 0
        else:
            unnamed.append(wav)
            continue
        if expect != row["cls"]:
            mislabelled.append((wav, expect, row["cls"]))

    unknown = {r["file"] for r in alt_rows} - {r["file"] for r in rows}
    if missing or mislabelled or unknown or unnamed:
        msg = [f"REFUSING TO WRITE: {test_dir} and upstream's "
               f"{os.path.relpath(full_csv, REPO)} disagree."]
        if missing:
            msg.append(f"  {len(missing)} of {len(rows)} wavs are missing "
                       f"(first: {missing[0]})")
            msg.append("  Is this dev_data_ToyCar.zip, and fully extracted?")
        if mislabelled:
            w, expect, got = mislabelled[0]
            msg.append(f"  {len(mislabelled)} labels contradict the filename "
                       f"(first: {w} reads as {expect}, CSV says {got})")
        if unknown:
            msg.append(f"  y_labels_alt.csv names {len(unknown)} files that "
                       f"y_labels.csv does not; it is supposed to be a subset")
        if unnamed:
            msg.append(f"  {len(unnamed)} names are neither normal_ nor anomaly_ "
                       f"(first: {unnamed[0]}); the label cannot be cross-checked, "
                       f"so this is the eval_data set rather than dev_data")
        sys.exit("\n".join(msg))

    common, param = import_upstream()
    n_mels = param["n_mels"]
    expect_bytes = FRAMES_KEPT * n_mels * BYTES_PER_FLOAT

    print(f"ad01: {len(rows)} files, n_mels={n_mels} n_fft={param['n_fft']} "
          f"hop={param['hop_length']} power={param['power']}")

    # 🔴 EVERY FILE COMPUTED AND CHECKED BEFORE THE OUTPUT DIRECTORY IS TOUCHED.
    #
    # The label checks above all run up front, but the SIZE check is per file and can
    # only run once that file exists -- and so can a corrupt wav, a librosa failure, a
    # full disk or a Ctrl-C.  Writing as it went would leave the output directory
    # holding some of this run's files, some of a previous run's, and a y_labels.csv
    # naming all of them, which the runner and score_dataset.py would then measure
    # without complaint.  That is the failure this whole script is built to prevent, so
    # it must not come back in through the writing.
    #
    # 25 MB of held bytes (248 x 102,400) buys that, and makes this script's guarantee
    # the same as the other three's rather than "true until the first bad element".
    blobs = []
    for i, row in enumerate(rows, 1):
        wav_path = os.path.join(test_dir, wav_for(row["file"]))
        # save_bin=True is upstream's own writer, and it puts the .bin beside the wav
        # (the path is derived from the wav's).  Computing it any other way would mean
        # reimplementing the one line this script exists to avoid reimplementing.  The
        # intermediate is read back and removed immediately, so a failure anywhere in
        # this loop leaves neither the source tree nor the output directory changed.
        produced = wav_path[:-len(".wav")] + "_hist_librosa.bin"
        try:
            try:
                common.file_to_vector_array(wav_path,
                                            n_mels=n_mels,
                                            frames=param["frames"],
                                            n_fft=param["n_fft"],
                                            hop_length=param["hop_length"],
                                            power=param["power"],
                                            save_bin=True)
            except TypeError:
                # common.file_load() swallows every read error and returns None, so the
                # unpacking one frame up is where a truncated or corrupt wav surfaces --
                # as an unpacking error that says nothing about the file.  Name it.
                sys.exit(f"could not read {wav_path}\n"
                         f"  librosa failed on it; the archive is probably truncated "
                         f"(check the zip before re-extracting)")
            if not os.path.exists(produced):
                sys.exit(f"upstream's file_to_vector_array() wrote no {produced}\n"
                         f"  the wav is probably shorter than one 5-frame window")
            with open(produced, "rb") as f:
                blob = f.read()
        finally:
            if os.path.exists(produced):
                os.remove(produced)

        if len(blob) != expect_bytes:
            sys.exit(f"REFUSING TO WRITE: {produced} was {len(blob)} B, expected "
                     f"{expect_bytes} ({FRAMES_KEPT} frames x {n_mels} bins x "
                     f"{BYTES_PER_FLOAT} B).\n"
                     f"  The size follows from the mel parameters and the centre crop, "
                     f"so a different one means the features are not the benchmark's.")
        blobs.append(blob)
        if i % 20 == 0 or i == len(rows):
            print(f"  {i}/{len(rows)}", end="\r", flush=True)
    print()

    if len(blobs) != len(rows):
        sys.exit(f"REFUSING TO WRITE: produced {len(blobs)} of {len(rows)} files")

    out_dir = os.path.join(args.out, "ad01")
    os.makedirs(out_dir, exist_ok=True)
    for row, blob in zip(rows, blobs):
        with open(os.path.join(out_dir, row["file"]), "wb") as f:
            f.write(blob)
    # The ground truth last: it is what selects the set, so until it lands, a directory
    # left behind by an interrupted write still describes the previous run.
    for csv_rows, name in ((rows, "y_labels.csv"), (alt_rows, "y_labels_alt.csv")):
        with open(os.path.join(out_dir, name), "w") as f:
            for r in csv_rows:
                f.write(f"{r['file']},{r['classes']},{r['cls']},"
                        f"{r['window']},{r['stride']}\n")

    n_anom = sum(1 for r in rows if r["cls"] == 1)
    windows = 1 + (expect_bytes - rows[0]["window"]) // rows[0]["stride"]
    print(f"ad01: {len(rows)} x {expect_bytes} B -> {out_dir}")
    print(f"  {n_anom} anomaly / {len(rows) - n_anom} normal, every label agreeing "
          f"with the archive's own filenames")
    print(f"  y_labels.csv ({len(rows)} files = {len(rows) * windows} inferences) and "
          f"y_labels_alt.csv ({len(alt_rows)} = {len(alt_rows) * windows})")
    print()
    print("This IS an accuracy dataset.  Run it with --mode a; the closed-division")
    print("target for ad01 is 0.85 AUC.  Score it on the host first.")


if __name__ == "__main__":
    main()
