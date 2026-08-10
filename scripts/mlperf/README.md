# MLPerf Tiny v1.4 on the Wio Lite AI — host side (issue #55)

ボード側は `mlperf` シェルコマンド（`shell/cmds/cmd_mlperf.c` + `port/mlperf/`）。
ここはそれを駆動する **PC 側**の設定と手順。

upstream は **`lib/mlperf-tiny`**（`mlcommons/tiny`、ブランチ `v1p4_rc` = `ced4c3b` に pin）。
**read-only ミラーなので編集しない** — 差し替えたい設定はこのディレクトリに置いて
`--device_list` / `--test_script` で渡す。

---

## できること・できないこと

| モード | 可否 | 必要なもの |
|---|---|---|
| **performance**（レイテンシ） | ✅ | ボードと USB ケーブルだけ |
| **accuracy**（精度） | ✅ | 実データセットの生成（下記。ベンチによっては数十 GB） |
| **energy**（電力） | ❌ | LPM01A / JouleScope が要る |
| **sww01**（ストリーミング WW） | ❌ | STM32H573I-DK インタフェース基板 + I2S + SD が要る |

対象は **ic01 / ic02 / kws01 / vww01 / ad01** の 5 つ。
ic02 はボード上では `ic01` として報告される（入出力形状が同一で、upstream の
`tests_performance.yaml` 自身も ic02 エントリに `model: ic01` と書いている）。
どちらを測ったかは **blob スロットに何を入れたか**で決まる。

**公式サブミッションはしない**（ワーキンググループへの参加登録が要る）。自己計測用。

---

## 1. ファームウェア

```bash
cmake -B build-mlperf -G Ninja \
      -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-toolchain.cmake \
      -DCONFIG_NN_BACKEND=tflm -DNN_TFLM_OPS=mlperf -DCONFIG_MLPERF_TINY=ON
cmake --build build-mlperf
# PF1 を保持したままリセット → DFU モード（赤 LED）
dfu-util -d 0483:df11 -a 0 -D build-mlperf/shell.bin
```

`CONFIG_MLPERF_TINY=ON` は初回に upstream ミラー（~340 MB, shallow）を取ってくる。

## 2. モデルを NOR の blob スロットへ

`.tflite` は `lib/mlperf-tiny/benchmark/training/*/trained_models/` にある。
**送る前に PC 側で検証する** — スロットを消す前に弾ける:

```bash
cmake --build build-mlperf --target verify-model
./build-mlperf/verify_tflite <model>.tflite      # PASS と crc32 を確認
```

| ベンチ | ファイル | サイズ |
|---|---|---|
| ic01 | `image_classification/trained_models/pretrainedResnet_quant.tflite` | 260,648 |
| ic02 | `image_classification/trained_models/pretrainedResnet_large_int8.tflite` | 512,024 |
| kws01 | `keyword_spotting/trained_models/kws_ref_model.tflite` | 53,936 |
| vww01 | `visual_wake_words/trained_models/vww_96_int8.tflite` | 333,288 |
| ad01 | `anomaly_detection/trained_models/ad01_int8.tflite` | 276,976 |

ボードで `blob write <slot>`、PC で `sb <file>`（**`sz` は ZMODEM なので不可**）。
6 スロットあるので 5 つ全部＋ BlazeFace が同居できる。転送後 `blob list` の crc32 が
`verify_tflite` の表示と一致することを確認。

## 3. ホスト環境

```bash
python3 -m venv .venv-mlperf
.venv-mlperf/bin/pip install -r scripts/mlperf/requirements-runner.txt
```

TensorFlow は**要らない**（実行するだけなら）。要るのは精度用データを作るときだけ。

## 4. 刺激データ

### performance だけなら（すぐ動く）

```bash
python3 scripts/mlperf/make_perf_dataset.py --out ~/mlperf-datasets
```

決定論的な合成データを 1 ベンチ 5 ファイルだけ作る。これらのモデルは入力で分岐しない
＝ **レイテンシは中身に依存しない**ので、これで出る throughput は本物。
🔴 **精度は出せない**（返ってくる分類は無意味）。`--mode a` に使わないこと。

### ic01/ic02 の accuracy（CIFAR-10 だけで済む。**TensorFlow 不要**）

```bash
curl -O https://www.cs.toronto.edu/~kriz/cifar-10-python.tar.gz   # 170 MB
tar xzf cifar-10-python.tar.gz
python3 scripts/mlperf/make_ic_accuracy_dataset.py \
        --cifar cifar-10-batches-py --out ~/mlperf-datasets
```

upstream の `perf_samples_loader.py` は `import train` 経由で TensorFlow を要求するが、
テスト画像 200 枚を取り出すのに要るのは pickle と numpy だけ。
🔴 **このスクリプトは自分の出力を upstream の `y_labels.csv` と照合し、
食い違えば 1 バイトも書かずに終了する**ので、「たぶん一致する再実装」にはなっていない。

（一番間違えやすいのは **CIFAR-10 が planar 1024R+1024G+1024B** なのにベンチマークは
**interleaved RGB** を要求する点。取り違えてもファイルは 3,072 B のまま run も完走し、
**精度だけが下がる**。）

### 他の 3 ベンチの accuracy（実データが要る）

`y_labels.csv` は upstream の `benchmark/evaluation/datasets/<id>/` にある（`_ref/mlperf-tiny/y_labels/`
にもコピー済み）。**`.bin` の実体は入っていない**ので、`benchmark/training/<bench>/` の
スクリプトで生成する。手間はベンチごとに大きく違う:

| ベンチ | 元データ | 規模 | 生成 |
|---|---|---|---|
| **ic01/ic02** | CIFAR-10 | 数十 MB | `training/image_classification/perf_samples_loader.py`（**一番軽い。最初にこれ**）|
| kws01 | Speech Commands v2 | ~2.3 GB | `training/keyword_spotting/make_bin_files.py` |
| ad01 | ToyADMOS ToyCar | 数 GB | `training/anomaly_detection/` |
| vww01 | COCO2014 / VWW | ~13 GB+ | `training/visual_wake_words/` |

生成した `.bin` と `y_labels.csv` を `<dataset_path>/<id>/` に置く。

🔴 **accuracy は時間がかかる**。`db` は **31 バイト/コマンドの往復**なので、
vww01 の 1 サンプル 27,648 B = **892 往復**、それが 1,000 サンプル。
ic01 は 3,072 B = 100 往復 × 200 サンプルなので数十秒で終わる。

## 5. 走らせる

```
wio> ai model load 0        # 測りたいモデルのスロット
wio> mlperf                 # EEMBC モニタへ（Ctrl+] で離脱）
```

🔴 **この状態にしてから、端末を閉じて**（`/dev/ttyACM0` を離す）ランナーを起動する。
ランナーは最初に `name%` を送って DUT を同定するので、シェルプロンプトのままだと
"failed to respond to 'name%'" になる。

```bash
# ランナーは cwd に sessions/ を掘る。submodule の中で走らせると read-only ミラーが
# 汚れるので、PYTHONPATH を通して外から呼ぶ。
R=$PWD/lib/mlperf-tiny/benchmark/runner
mkdir -p ~/mlperf-runs && cd ~/mlperf-runs
PYTHONPATH=$R <repo>/.venv-mlperf/bin/python $R/main.py \
    --device_list  <repo>/scripts/mlperf/devices_wio.yaml \
    --test_script  <repo>/scripts/mlperf/tests_performance_wio.yaml \
    --dataset_path ~/mlperf-datasets \
    --mode p
```

🔴 **`tests_performance_wio.yaml` を使うこと**（upstream の `tests_performance.yaml` ではなく）。
ランナーは **1 ループ 5 回が 10 秒未満だと median を出さず `ERROR 2` にする**（測定の
妥当性の下限）。upstream の反復数は 80 MHz の参照ボード向けなので、550 MHz の本機だと
kws01 が 4 秒で終わって弾かれる。差分は kws01 の `70 → 800` **1 箇所だけ**。

`--mode a` で精度。結果は `sessions/YYYYMMDD_HHMMSS/results.txt` と `results.json`。

離脱後にボードが出すローカル要約（DWT のサイクル数）は**参考値**で、MLPerf のスコアでは
ない — ウォームアップを含むし、ホストが測る throughput の方が定義に沿った数字。

---

## 実測値（board #2, 2026-08-11, `-O2` CMSIS-NN, arena は外部 PSRAM）

| ベンチ | モデル | throughput | 1 推論 |
|---|---|---:|---:|
| ic01 として測定した **ic02** | ResNet large int8 (512 KB) | 2.301 inf/s | 434.6 ms |
| **kws01** | DS-CNN int8 (54 KB) | 98.31 inf/s | 10.17 ms |
| **vww01** | MobileNet 96×96 int8 (333 KB) | 23.34 inf/s | 42.8 ms |
| **ad01** | Deep AutoEncoder int8 (277 KB) | 232.06 inf/s | 4.31 ms |

5 window の median。ic01（小さい方の ResNet, 260 KB）は未測定。

### accuracy（`--mode a`）

| ベンチ | Top-1 | 目標（Closed Division） | |
|---|---:|---:|---|
| **ic02**（CIFAR-10 実データ 200 枚） | **94.0 %** | 91 % | ✅ クリア |

kws01 / vww01 / ad01 は実データ未取得のため未測定。
🔴 **94% が出たこと自体がハーネスの正しさの証明**でもある — 入力変換（planar/interleaved、
`^0x80`）か逆量子化のどちらかを間違えていれば、この数字にはならない。

⚠ **arena が外部 PSRAM にある構成の数字**であることを忘れないこと。#9 P2c で
BlazeFace が 6.45 cyc/MACC だったのと同じ制約下で、内蔵 SRAM に収まるモデルを持つ
他社結果と比べるときはそこが効く。

---

## つまずくところ

- **`mlperf` 中はコンソールと PSRAM を占有する**。`lcd on` / `camera capture` / `psram` は
  busy になる。accuracy の長い run では数十分続く
- **Ctrl+C はモニタを抜けない**（プロトコルにバイトを渡すため）。**Ctrl+]**
- **ボードの USB は `0483:5740`** ＝ ST の汎用 VCP ID で、upstream の
  `devices_kws_ic_vww.yaml` では **LPM01A 電力計に割り当てられている**。
  だから `devices_wio.yaml` を使うこと（upstream の device list を渡すと、
  この基板を電力計だと判定して DUT が見つからないと言う）
- **端末を掴んだままランナーを起動しない**。`Resource busy` か、静かに文字化けする
