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
| **accuracy**（精度） | ✅ | 実データセットの生成（下記。計 4.5 GB のダウンロード） |
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

TensorFlow は**要らない**（実行するだけなら）。精度用データを作るときは、これに加えて

```bash
.venv-mlperf/bin/pip install -r scripts/mlperf/requirements-datasets.txt   # 軽い
```

**kws01 だけ** TensorFlow が要り、**別 venv** に入れる（#56 でここは分かれた）:

```bash
python3 -m venv .venv-mlperf-tf
.venv-mlperf-tf/bin/pip install -r scripts/mlperf/requirements-kws-tf.txt  # 数 GB
```

## 4. 刺激データ

### performance だけなら（すぐ動く）

```bash
python3 scripts/mlperf/make_perf_dataset.py --out ~/mlperf-datasets
```

決定論的な合成データを 1 ベンチ 5 ファイルだけ作る。これらのモデルは入力で分岐しない
＝ **レイテンシは中身に依存しない**ので、これで出る throughput は本物。
🔴 **精度は出せない**（返ってくる分類は無意味）。`--mode a` に使わないこと。

### accuracy の全体像

**4 セット中 3 セットは TensorFlow 無しで作れる**（#56 で判明。着手前の見積りは
「3 つとも TF が要る・vww は 13 GB」だった）:

| ベンチ | 元データ | DL | TF | 生成 |
|---|---|---:|:-:|---|
| ic01/ic02 | CIFAR-10 | 170 MB | — | `make_ic_accuracy_dataset.py` |
| vww01 | VWW 96×96（Silicon Labs） | 235 MB | — | `make_vww_accuracy_dataset.py` |
| ad01 | ToyADMOS ToyCar | 1.8 GB | — | `make_ad_accuracy_dataset.py` |
| kws01 | Speech Commands v2 | 2.5 GB | **要** | `make_kws_accuracy_dataset.py` |

どれも**「upstream が同梱している答え」と照合してから書く**（照合できるものはベンチごとに
違う。下記）。**照合できないのは特徴量／ピクセルの値そのもの**なので、
最後に `score_dataset.py` でホスト採点する。

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

### vww01 の accuracy（235 MB。**TensorFlow 不要**）

```bash
curl -LO https://www.silabs.com/public/files/github/machine_learning/benchmarks/datasets/vw_coco2014_96.tar.gz
.venv-mlperf/bin/python scripts/mlperf/make_vww_accuracy_dataset.py \
        --vww vw_coco2014_96.tar.gz --out ~/mlperf-datasets
```

🔴 **COCO2014 の 13 GB は要らない。** upstream の `download_and_train_vww.sh` 自身が
Silicon Labs の `vw_coco2014_96.tar.gz` を取ってくる ——
**96×96 に切り出し済み・person/non_person に仕分け済み**の VWW セット。
JPEG を生 RGB にするだけなので PIL しか要らない。

検査は **id → 画像が 1 対 1 に解決すること**と、**その画像が入っているディレクトリが
upstream の `y_labels.csv` のラベルと一致すること**を 1,000 行すべてについて要求する
（＝独立な 2 つの ground truth が一致しないと 1 バイトも書かない）。
⚠ **ピクセルは EEMBC の `.bin` と bit 一致ではない**（Silicon Labs の JPEG を 1 世代
経由する）。だから下の**ホスト採点**を先にやる。

### ad01 の accuracy（1.8 GB。**TensorFlow 不要**）

```bash
curl -L -o dev_data_ToyCar.zip 'https://zenodo.org/record/3678171/files/dev_data_ToyCar.zip?download=1'
unzip -q dev_data_ToyCar.zip -d dev_data
.venv-mlperf/bin/python scripts/mlperf/make_ad_accuracy_dataset.py \
        --toycar dev_data/ToyCar --out ~/mlperf-datasets
```

🔴 **ここも TensorFlow は要らない。** anomaly detection は DCASE 2020 Task 2 のコードで、
特徴抽出 `common.file_to_vector_array()` は **librosa + numpy だけ**。TF が要るのは
オートエンコーダを学習するときだけ。このスクリプトは**その関数をそのまま呼ぶ**
（mel パラメータも upstream の `baseline.yaml` から読む）。

検査は 248 行すべてについて **wav が存在すること**、**`normal_`/`anomaly_` の接頭辞が
CSV のラベルと一致すること**、生成物が**きっかり 102,400 B**（200 フレーム × 128 bin ×
4 B）であること。サイズが合う＝ mel パラメータと中央切り出しが合っている。

`y_labels.csv`（248 ファイル）と `y_labels_alt.csv`（72 ファイル）の**両方**を書く。
刺激ファイルは共通（alt は真部分集合）で、どちらで走らせるかは test script 側の選択。

### kws01 の accuracy（2.5 GB。**ここだけ TensorFlow が要る**）

```bash
.venv-mlperf-tf/bin/python scripts/mlperf/make_kws_accuracy_dataset.py \
        --tfds-dir ~/mlperf-src/tfds --out ~/mlperf-datasets
```

🔴 **避けられない理由は 2 つあって、どちらも「丁寧にやれば回避できる」類ではない。**

1. **特徴量**が tf.signal の MFCC（STFT → mel → log → DCT）。numpy で書き直せば
   1 LSB 程度は違うが、**ic01 と違って特徴量を照合できる upstream 成果物が無い**ので、
   その差が再実装のせいかどうかを**判定する手段が無い**。
2. **並び**が `tensorflow_datasets` の test split の順で、これは**アーカイブ中の
   ファイル順ではない** —— TFDS は書き出し時に example key のハッシュで shard へ
   シャッフルする。upstream の `y_labels.csv` は**その順のラベル列**なので、
   再現するにはそのハッシュ配分の再実装が要る。

∴ upstream の `get_dataset.py` を **import して呼ぶ**。検査は
**1,000 行の `tst_<index>_<word>_<label>.bin,12,<label>` が upstream の CSV と完全一致**
すること（index も綴りも数値ラベルも全部 split の順の帰結なので、これが合えば
同じサンプルを同じ順で拾っている）。

### 🔴 ボードに投げる前に、ホストで採点する

```bash
.venv-mlperf/bin/python scripts/mlperf/score_dataset.py \
        --bench vww01 --dataset ~/mlperf-datasets/vww01 \
        --model lib/mlperf-tiny/benchmark/training/visual_wake_words/trained_models/vww_96_int8.tflite
```

同じ `.tflite` を、**ファームと同じバイト変換**（`^0x80` / int8 素通し / float32 量子化）と
**同じ 3 桁丸め**（`put_fixed3`）を通し、**runner の `main.py` から import した集計関数**で
採点する。ad01 のスライディングウィンドウも再現する。つまり**「ボードがこう言うはずの数字」**。

- ホストが目標を大きく割る → **データ生成が間違っている**。ここで直す
- ホストは目標、ボードだけ低い → **ファームの問題**だと確定する

⚠ **最後の桁まで一致はしない**（ボードは TFLM+CMSIS-NN、ホストは LiteRT のカーネル）。
0.1 ポイント級のズレは想定内、1 ポイント級は調査対象。
**較正済み**: ic02 でホストが **94.0%** と出て、これは #55 の実機実測と一致する。

### accuracy にかかる時間

`db` は **31 バイト/コマンドの往復**。

| ベンチ | 1 サンプル | 往復/サンプル | サンプル数 | 推論回数 | 実測 |
|---|---:|---:|---:|---:|---:|
| ic01/ic02 | 3,072 B | 100 | 200 | 200 | 数十秒 |
| kws01 | 490 B | 16 | 1,000 | 1,000 | 約 1 分 |
| vww01 | 27,648 B | 892 | 1,000 | 1,000 | **約 30 分** |
| ad01（alt 72） | 2,560 B | 83 | 72 × 196 窓 | 14,112 | **約 40 分** |
| ad01（full 248） | 2,560 B | 83 | 248 × 196 窓 | 48,608 | **約 2 時間** |

ad01 は**窓ごとに 1 ダウンロード + 1 推論**（`benchmark/runner/script.py`）なので桁が違う。
既定の `tests_accuracy_wio.yaml` は **alt（72 ファイル）**を指す ——
upstream 自身が時間短縮のために用意したもので、v1.2 以降サブミッションでも受理される。

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

精度は `--test_script <repo>/scripts/mlperf/tests_accuracy_wio.yaml --mode a`。
結果は `sessions/YYYYMMDD_HHMMSS/results.txt` と `results.json`。

**どのベンチを測るかは引数ではなく「どのモデルを載せたか」で決まる。** ランナーは
`test_script.get(dut.get_model())`（`main.py:83`）—— DUT が `m-model-[...]` で名乗った
id で yaml のエントリを引く。本機の id は
`mlperf_model_id()`（`port/mlperf/mlperf_th.cc`）＝**アリーナに載っているモデルの形状**の
関数なので、`ai model load <slot>` がベンチ選択そのもの。
∴ **1 ベンチ = 1 回の `ai model load` + 1 回のランナー起動**で、まとめては測れない。

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

### accuracy（`--mode a`、2026-08-11、#55 の ic02 と #56 の 3 ベンチ）

| ベンチ | 刺激データ | 実機 Top-1 | 実機 AUC | 目標 | |
|---|---|---:|---:|---:|---|
| **ic02** | CIFAR-10 200 枚 | **94.0 %** | 0.990 | 91 % | ✅ |
| **kws01** | Speech Commands v2 1,000 | **90.1 %** | 0.995 | 90 % | ✅ |
| **vww01** | VWW 96×96 1,000 | **85.8 %** | 0.937 | 80 % | ✅ |
| **ad01** | ToyADMOS ToyCar（alt 72） | (82.6) | **0.854** | 0.85 AUC | ✅ |

ad01 の Top-1 欄が括弧なのは、それが精度ではないから（「つまずくところ」参照）。
判定は AUC。**余裕は 0.004 しかない**ので、はっきりした数字が要るときは
full（248 ファイル、ホスト 0.859、約 2 時間）で測り直す。

🔴 **数字が出たこと自体がハーネスの正しさの証明**でもある — 入力変換（planar/interleaved、
`^0x80`、int8 素通し、float32 量子化）か逆量子化のどちらかを間違えていれば、
run は完走して**数字だけが**悪くなる。

#### ホスト予測との突き合わせ（`score_dataset.py`）

| ベンチ | ホスト | 実機 | 差 |
|---|---:|---:|---|
| ic02 | 94.0 % / 0.990 | 94.0 % / 0.990 | 0 |
| kws01 | 90.3 % / 0.995 | 90.1 % / 0.995 | −0.2 pt（2 サンプル）|
| vww01 | 85.6 % / 0.937 | 85.8 % / 0.937 | +0.2 pt（2 サンプル）|
| ad01（alt） | (82.6) / 0.850 | (82.6) / 0.854 | AUC +0.004 |

**分類 3 件は Top-1 が 2 サンプル以内・AUC が 3 桁とも一致**、ad01 は AUC が +0.004。
TFLM+CMSIS-NN と LiteRT のカーネル差はこの幅、という実測値であり、
**今後この幅を超えたら調査対象**という基準線。
🔴 **ズレの向きは一定ではない**（kws は下、vww と ad は上）。
∴ ホスト値は**上限でも下限でもなく中心**として扱う —— 目標ぎりぎりのときに
「ホストで通ったから実機も通る」とは言えない。

⚠ **kws01 の余裕は 0.1 ポイント（1 サンプル）しかない。** これはハーネスではなく
**参照モデルの性質**で、同じモデルを upstream 自身の `eval_quantized_model.py` で
テストセット全 4,890 サンプルに掛けると **91.7 %**（ドキュメントの「約 92%」と一致）。
ベンチが使う先頭 1,000 サンプルがたまたま難しい部分集合になっている。

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
- **ad01 の `Top 1%` 行は精度ではない**。upstream の `calculate_accuracy()` は
  出力が 1 値のとき **F1 のしきい値スイープ**に切り替わる（`main.py:128`）。
  ad01 の判定は **`AUC` 行だけ**を見る
- **Zenodo の 1.8 GB は無言で止まる**（エラーにならず 0 B/s のまま居座り、`--retry` も
  効かない）。`curl -C - --speed-limit 20000 --speed-time 30 --retry 30 --retry-all-errors`
  で「遅すぎたら切って再開」にする
- **ランナーに再開機能は無い**。vww01 の 30 分・ad01 の 40 分の途中で 1 コマンドでも
  タイムアウトすれば最初からやり直し
