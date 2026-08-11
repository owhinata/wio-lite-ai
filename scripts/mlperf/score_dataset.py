#!/usr/bin/env python3
"""Score an MLPerf Tiny accuracy stimulus set on the HOST, the way the board would (#56).

WHY
---
An accuracy run costs board time measured in tens of minutes -- vww01 is 892 `db`
round trips per sample times 1,000 samples, ad01 is 196 windows per file -- and the
number that comes out cannot distinguish "the firmware is wrong" from "the stimulus
files are wrong".  This runs the same reference .tflite over the same generated files,
through the same byte transforms the firmware applies and the same aggregation
benchmark/runner/main.py applies, and prints the same two figures.

So the host number is what the board SHOULD say.  Run it before the board:

  - host well below target  -> the dataset generation is wrong.  Fix it here; the board
                               would only have told you the same thing an hour later.
  - host at target, board below -> now it is the firmware, and you know that for free.

🔴 IT WILL NOT MATCH THE BOARD TO THE LAST DIGIT, AND IT IS NOT SUPPOSED TO.  The board
runs TFLM with CMSIS-NN int8 kernels; this runs LiteRT's.  Both implement the same
quantized arithmetic, but they round intermediate accumulators differently in places,
so a sample sitting on a decision boundary can land either way.  Fractions of a point
are expected.  Points are a finding.

Everything that is NOT kernel arithmetic is mirrored exactly, deliberately, so that a
difference has only one place left to come from:

  - the input byte transform per benchmark, from port/mlperf/mlperf_th.cc
  - the 3-decimal wire format th_results() prints (put_fixed3)
  - ad01's sliding window, from benchmark/runner/script.py
  - the per-file aggregation and the two metrics, IMPORTED from upstream's main.py
    rather than reimplemented

USAGE
-----
    python3 scripts/mlperf/score_dataset.py \
        --bench vww01 --dataset ~/mlperf-datasets/vww01 \
        --model _ref/mlperf-tiny/models/vww01_mobilenet_int8.tflite

Needs an interpreter: `pip install ai-edge-litert` (small), or any environment that
already has tflite_runtime or tensorflow.
"""

import argparse
import atexit
import os
import shutil
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
RUNNER = os.path.join(REPO, "lib", "mlperf-tiny", "benchmark", "runner")

# Mirrors mlperf_table[] in port/mlperf/mlperf_th.cc.  The load transform is a property
# of the BENCHMARK, not of the model's quantization -- see that file's note, and
# upstream's benchmark/evaluation/datasets/README.md, which is where both come from.
BENCHES = {
    "ic01":  dict(load="offset_u8", report="classes", target="85% top-1"),
    "ic02":  dict(load="offset_u8", report="classes", target="91% top-1"),
    "kws01": dict(load="int8",      report="classes", target="90% top-1"),
    "vww01": dict(load="offset_u8", report="classes", target="80% top-1"),
    "ad01":  dict(load="f32_quant", report="mse",     target="0.85 AUC"),
}


def load_interpreter(model_path):
    """Whichever interpreter this environment has; they share the API we use."""
    errors = []
    for module, attr in (("ai_edge_litert.interpreter", "Interpreter"),
                         ("tflite_runtime.interpreter", "Interpreter"),
                         ("tensorflow.lite", "Interpreter")):
        try:
            mod = __import__(module, fromlist=[attr])
            interp = getattr(mod, attr)(model_path=model_path)
            interp.allocate_tensors()
            return interp, module
        except ImportError as e:
            errors.append(f"  {module}: {e}")
    sys.exit("no tflite interpreter available:\n" + "\n".join(errors) +
             "\n  pip install ai-edge-litert")


def import_upstream_scoring():
    """main.calculate_accuracy / normalize_probabilities, from upstream itself.

    Imported rather than copied for the same reason make_ic_accuracy_dataset.py checks
    against upstream's y_labels.csv: a scoring rule that merely resembles the runner's
    would give a number that merely resembles the board's, and the whole point of this
    tool is that the two are comparable.  ad01 in particular is not scored as accuracy
    at all -- calculate_accuracy() switches to an F1 sweep when there is one output per
    sample -- and that is exactly the kind of detail a reimplementation gets wrong.

    🔴 IMPORTING IT HAS SIDE EFFECTS, AND THEY HAVE TO BE UNDONE.  benchmark/runner/
    script.py, which main.py pulls in, is written to be RUN: at module scope it opens a
    log under ./sessions/ and then replaces sys.stdout AND sys.stderr with writers that
    funnel everything into logging (script.py:49-50).  Left in place that is not
    cosmetic -- LoggerWriter.write() calls .strip() on every message, so `end="\\r"`
    disappears and this tool's progress counter prints a thousand separate lines.  So
    the cwd is pointed at a scratch directory for the import and all three globals are
    restored afterwards.

    The scratch directory outlives the import on purpose: the FileHandler installed by
    that module keeps the log open for the life of the process, and removing the
    directory underneath it would leave it writing to an unlinked file.
    """
    if not os.path.isdir(RUNNER):
        sys.exit(f"missing {RUNNER}\n"
                 f"  git submodule update --init --depth 1 -- lib/mlperf-tiny")
    cwd = os.getcwd()
    saved_path, saved_out, saved_err = list(sys.path), sys.stdout, sys.stderr
    tmp = tempfile.mkdtemp(prefix="mlperf-score-")
    atexit.register(shutil.rmtree, tmp, True)
    try:
        os.chdir(tmp)
        sys.path.insert(0, RUNNER)
        import main as upstream_main
    finally:
        os.chdir(cwd)
        sys.path[:] = saved_path
        sys.stdout, sys.stderr = saved_out, saved_err
    return upstream_main


def read_y_labels(path, want_window):
    """Upstream's ground truth, whitespace-normalised.

    Three columns for a classifier, five for ad01 -- the extra two being the sliding
    window and its stride (upstream's datasets/README.md).  The count is checked rather
    than tolerated: a three-column ad01 file would otherwise reach the window loop as
    `None` and only be diagnosed there, and a five-column classifier file would mean
    the wrong ground truth was handed to the wrong benchmark.
    """
    rows = []
    with open(path) as f:
        for n, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            p = [x.strip() for x in line.split(",")]
            if len(p) != (5 if want_window else 3):
                sys.exit(f"{path}:{n}: {len(p)} columns, expected "
                         f"{5 if want_window else 3} "
                         f"({'file,classes,class,window,stride' if want_window else 'file,classes,class'})"
                         f"\n  {line}")
            rows.append(dict(file=p[0], classes=int(p[1]), cls=int(p[2]),
                             bytes_to_send=int(p[3]) if want_window else None,
                             stride=int(p[4]) if want_window else None))
    return rows


def fixed3(v):
    """put_fixed3() from port/mlperf/mlperf_th.cc: three decimals, half away from zero.

    The board has no %f -- svc/fmt.c does not implement one -- so every score crosses
    the wire rounded to a milli-unit.  Applying the same rounding here keeps the two
    numbers comparable; leaving it out would introduce a difference that has nothing to
    do with either the model or the data.

    In float32 throughout, because the board's is: the saturation constant, the
    multiply and the add are all float there, and 999999.999f is not 999999.999.  It
    makes no difference to any score these benchmarks produce -- probabilities are at
    most 1 and ad01's errors are tens -- but a mirror that is only a mirror in the
    common case is not one worth having.

    Checked against the C rather than reasoned about, by compiling put_fixed3() and
    running both over the same values.  All fourteen agree, including the three that
    are easy to get wrong: -0.0 takes the positive branch (`-0.0f < 0.0f` is false),
    NaN takes the saturation branch (which is why the C spells it `!(v < ...)`), and
    999999.999 saturates to 1000000.000 rather than to itself, because 999999.999f
    times 1000.0f rounds up to exactly 1e9 in float32.
    """
    v = np.float32(v)
    neg = bool(v < np.float32(0.0))
    if neg:
        v = -v
    if not bool(v < np.float32(1000000.0)):      # catches NaN too, as the C does
        v = np.float32(999999.999)
    milli = int(v * np.float32(1000.0) + np.float32(0.5))
    return -milli / 1000.0 if neg else milli / 1000.0


def quantize(values, scale, zero_point):
    """quantize() from mlperf_th.cc == upstream's QuantizeFloatToInt8(), vectorised.

    The C is `(int32_t)(v / scale + (v >= 0 ? 0.5f : -0.5f)) + zp`, saturated: add half
    and truncate TOWARD ZERO, which is round-half-away-from-zero, not numpy's
    round-half-to-even.  np.trunc is what reproduces it; np.rint would not.

    Kept in float32 because the board's FPU is doing this in float32, and ad01 puts
    31 million values through it -- one element at a time in Python would take longer
    than the board run this is meant to save.
    """
    if scale == 0.0:
        return np.full(values.shape, zero_point, dtype=np.int8)
    x = (values.astype(np.float32) / np.float32(scale)).astype(np.float32)
    q = np.trunc(x + np.where(x >= 0.0, np.float32(0.5), np.float32(-0.5)))
    return np.clip(q.astype(np.int32) + zero_point, -128, 127).astype(np.int8)


def run_one(interp, in_det, out_det, tensor):
    interp.set_tensor(in_det["index"], tensor)
    interp.invoke()
    return interp.get_tensor(out_det["index"])


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bench", required=True, choices=sorted(BENCHES),
                    help="which benchmark these stimuli are for")
    ap.add_argument("--dataset", required=True,
                    help="directory holding the .bin files and y_labels.csv")
    ap.add_argument("--model", required=True, help="the reference .tflite")
    ap.add_argument("--truth", default="y_labels.csv",
                    help="which ground-truth file in --dataset to score against.  It "
                         "must be the SAME one the test script gives the runner, or "
                         "the two numbers are of different sets: ad01 ships both "
                         "y_labels.csv (248 files) and y_labels_alt.csv (72), and "
                         "tests_accuracy_wio.yaml uses the latter.")
    ap.add_argument("--limit", type=int, default=0,
                    help="score only the first N files (a smoke test, not a result)")
    args = ap.parse_args()

    bench = BENCHES[args.bench]
    upstream = import_upstream_scoring()

    truth = os.path.join(args.dataset, args.truth)
    if not os.path.exists(truth):
        sys.exit(f"missing {truth}")
    rows = read_y_labels(truth, want_window=(bench["report"] == "mse"))
    if args.limit:
        rows = rows[:args.limit]

    interp, backend = load_interpreter(args.model)
    in_det, out_det = interp.get_input_details()[0], interp.get_output_details()[0]
    in_scale, in_zp = in_det["quantization"]
    out_scale, out_zp = out_det["quantization"]
    in_shape = tuple(int(d) for d in in_det["shape"])
    in_bytes = int(np.prod(in_shape))

    print(f"{args.bench}: {len(rows)} files, model {os.path.basename(args.model)}, "
          f"interpreter {backend}")
    print(f"  input {in_shape} {np.dtype(in_det['dtype']).name} "
          f"scale={in_scale} zp={in_zp}")

    # Same shape as main.py's file_infer_results: one entry per file, holding every
    # inference done for it (one for classification, 196 windows for ad01).
    per_file = {}
    for n, row in enumerate(rows, 1):
        path = os.path.join(args.dataset, row["file"])
        with open(path, "rb") as f:
            data = f.read()

        entry = per_file.setdefault(row["file"],
                                    {"true_class": row["cls"], "results": []})

        if bench["report"] == "mse":
            # benchmark/runner/script.py: fixed-size windows, advanced by stride, for
            # as long as a whole window fits.  Performance mode sends only the first;
            # accuracy sends them all, and every one is a separate inference.
            seg_len, stride = row["bytes_to_send"], row["stride"]
            if seg_len is None or stride is None:
                sys.exit(f"{truth}: ad01 rows need the window and stride columns")
            starts = range(0, len(data) - seg_len + 1, stride)
            for start in starts:
                floats = np.frombuffer(data[start:start + seg_len], dtype="<f4")
                # A NaN or an infinity here would reach the int32 cast in both
                # quantize() and the board's, where the result is undefined in C and
                # merely unspecified in numpy -- so the two would stop mirroring at
                # exactly the point where the answer stopped meaning anything.  Refuse
                # instead of reporting a score derived from it.
                if not np.isfinite(floats).all():
                    sys.exit(f"{path}: window at byte {start} holds a non-finite "
                             f"float.\n"
                             f"  A log-mel spectrogram cannot; regenerate the file "
                             f"with make_ad_accuracy_dataset.py.")
                q = quantize(floats, in_scale, in_zp).reshape(in_shape)
                out = run_one(interp, in_det, out_det, q).astype(np.int32).flatten()
                deq = np.float32(out_scale) * (out - out_zp).astype(np.float32)
                # th_results(): squared error against the ORIGINAL floats, not against
                # the dequantized input -- comparing to the latter would cancel the
                # input's quantization error out of both sides.
                mse = float(np.mean((deq.astype(np.float64) -
                                     floats.astype(np.float64)) ** 2))
                entry["results"].append([fixed3(mse)])
        else:
            if len(data) != in_bytes:
                sys.exit(f"{path} is {len(data)} B, model wants {in_bytes}")
            raw = np.frombuffer(data, dtype=np.uint8)
            if bench["load"] == "offset_u8":
                # th_load_tensor(): XOR 0x80 is the unsigned->int8 shift by 128.
                raw = raw ^ np.uint8(0x80)
            q = raw.view(np.int8).reshape(in_shape)
            out = run_one(interp, in_det, out_det, q).astype(np.int32).flatten()
            deq = [fixed3(out_scale * (v - out_zp)) for v in out]
            entry["results"].append(upstream.normalize_probabilities(deq))

        if n % 100 == 0 or n == len(rows):
            print(f"  {n}/{len(rows)} files", end="\r", flush=True)
    print()

    # main.py's aggregation, verbatim in structure.
    y_pred, y_true, correct = [], [], 0
    for data in per_file.values():
        counts = {}
        for res in data["results"]:
            k = int(np.argmax(res))
            counts[k] = counts.get(k, 0) + 1
        majority = max(counts.items(), key=lambda kv: kv[1])[0]
        if majority == data["true_class"]:
            correct += 1
        y_pred.append(np.mean(data["results"], axis=0))
        y_true.append(data["true_class"])

    y_pred, y_true = np.array(y_pred), np.array(y_true)
    accuracy = upstream.calculate_accuracy(y_pred, y_true)
    from sklearn.metrics import roc_auc_score
    if y_pred.shape[1] == 2:
        auc = roc_auc_score(y_true, y_pred[:, 1])
    else:
        auc = roc_auc_score(y_true, y_pred, multi_class="ovr")

    print()
    print(f"  Top 1% = {accuracy:2.1f}")
    print(f"  AUC    = {auc:.3f}")
    print(f"  (majority-vote files correct: {correct}/{len(y_true)})")
    print(f"  closed-division target: {bench['target']}")
    if args.limit:
        print("  ⚠ --limit was used; this is a smoke test, not a result")


if __name__ == "__main__":
    main()
