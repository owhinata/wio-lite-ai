---
name: codex-debug
description: Codex による不具合解析。根本原因の仮説列挙、検証方法の提案、関連コード特定を依頼する。症状・再現手順・観測結果を入力として受け取る。Wio Lite AI (STM32H725) / 内蔵 Flash 実行 / ThreadX / USB CDC の組込み不具合向け。
argument-hint: <Issue number, symptom description, or file path>
---

# Codex 不具合解析 (Wio Lite AI / STM32H725AEI6)

Codex 呼び出しは **codex plugin（`codex@openai-codex`）のランタイム**に一本化されている
（MCP server は撤去済）。この skill は「組込み向けの解析観点」を載せた入口で、実行は
plugin のランタイムを叩く。

- **原因の仮説出し・切り分け設計**（この skill の主目的）: `codex-companion.mjs task` に
  自己完結プロンプトを投げる。
- **Codex に手を動かして調べ切ってほしい / 修正案まで書かせたい**: `/codex:rescue` に委譲する
  （codex-rescue subagent が同じランタイムで走る）。
- 設計判断そのものを疑いたい場合は `/codex:adversarial-review <focus>`。

## 対象の判定

`$ARGUMENTS` の内容に応じて解析対象を決定する:

- **Issue 番号** (例: `#12`, `12`): `gh issue view` で症状・再現手順・観測結果を取得
- **テキスト**: 症状の説明としてそのまま解析対象とする
- **ファイルパス**: 問題が発生しているコードを起点に解析

## 解析実行手順

1. 不具合の情報を収集する
   - 症状、再現手順、観測結果（USB CDC ログ、LED 挙動、SWD で読んだレジスタ/変数）
   - 関連するソースコード、`build/<app>.map`、直近の変更（`git log`, `git diff`）
   - `boot/README.md`（ブートローダが app へ渡す実測クロック値）
2. 問題の層を特定する（後述「層の切り分け」）
3. 収集情報と以下の「解析観点」を**自己完結したプロンプトファイル**に落とす
   （Codex はこの会話を見られない。「上記の症状」のような参照は書かない）
4. Codex に投げる:

```bash
node "$(ls -d "$HOME"/.claude/plugins/cache/openai-codex/codex/*/scripts/codex-companion.mjs \
        | sort -V | tail -1)" task --effort xhigh --prompt-file /abs/path/to/debug-prompt.md
```

   - `--write` は付けない（サンドボックスは `read-only`。ローカルシェルは使えるので
     `git log` / `grep` / `sed` でリポジトリの実物を読ませてよい）。
   - **`Bash(run_in_background: true)` で起動して待つ**（解析は 120s を超える）。
   - 追加で対話したいときは同じスクリプトに `task --resume-last "<追加の質問>"`。
   - コマンドは **`node` で始める**こと（先頭に変数代入を置くと `Bash(node:*)` の
     permission ルールに当たらず毎回プロンプトが出る）。
5. 結果を整理してユーザーに報告する。**仮説は仮説として報告する** — Codex の指摘を
   確定した原因のように書かない。裏を取れるものは RM0468 / 実物のコードで確認してから出す。

## 解析観点

### 1. 根本原因の仮説列挙

Codex に以下を求める:

- **複数の仮説を確度順に列挙** — 1 つに絞らない
- **各仮説の根拠コード箇所を特定** — ファイルパスと行番号
- **各仮説の検証方法を提案** — 最小実験 / SWD 観測 / ログ追加で切り分けられる手順
- **仮説を棄却できる観測**もセットで（「これが観測されたらこの仮説は死ぬ」）

### 2. 層の切り分け

| 層 | 例 |
|---|---|
| HW / 電気的 | 電源、クロック源（HSE 25MHz）、信号品質、schematic 配線・プルアップ、断線 |
| DFU ブートローダ | boot が渡すクロックの状態、jump 時の VTOR/MSP、DFU 判定 |
| MCU ペリフェラル | レジスタ設定ミス、クロック/PLL/Flash WS、キャッシュコヒーレンシ、割込み優先度、DMA 到達性 |
| CMSIS / startup | ベクタテーブル、`SystemInit`、`_estack`/`.data`/`.bss` 初期化、リンカスクリプト、ITCM/DTCM 配置 |
| ST HAL | HAL 初期化順序、`HAL_GetTick`/timebase、周辺ドライバ |
| USB (TinyUSB) | OTG_HS FS 動作、`tud_int_handler(0)`、enumerate 失敗、CDC RX/TX リング |
| RTOS (ThreadX) | スケジューラ/PendSV、tick 供給、優先度、スタックオーバーフロー、クリティカルセクション |
| アプリ / shell | app 起動、shell core、USB CDC backend、`port/*` の統合コード |

### 3. HW / 割込み起因を必ず検討する

組込み特有の問題は SW だけでは説明できないことが多い。Codex に必ず検討させる:

- **クロック継承の破壊**: app が RCC/PLL/FLASH ACR/PWR を触ってしまい、継承した
  550 MHz / PLL3Q 48 MHz USB / FLASH latency 3 が壊れる（HSI 64 MHz に落ちるのに latency は
  550 MHz 用のまま）。app の `SystemInit` が **FPU + VTOR + ITCM ロードのみ**か、
  `HAL_Init`/`SystemClock_Config` を呼んでいないかを最優先で疑う。
- 🔴 **DMA が TCM に届かない**: DMA1/DMA2・SDMMC1 IDMA は TCM 非到達（RM0468
  §2.1.2/§2.1.5/§2.1.6）。**DTCM に置いたバッファは fault せず無言で「何も転送されない」**。
  「転送は成功しているのに中身が来ない/ゼロ」はまずこれを疑う。
- 🔴 **配置が剥がれている**: リンカの `ASSERT` は LTO 下で空振りする。`.itcm`/`.dtcm_bss` の
  実配置は `build/*.map` と objdump で確認する（`cmake/check_*_residency.py` が何を見ているか）。
- 割込み優先度・プリエンプション（ThreadX: **SysTick > PendSV** か、OTG_HS/HAL ISR との関係）
- クリティカルセクション（**PRIMASK ベース**）で tick やイベントが落ちていないか
- キャッシュ（D-Cache 有効）と DMA 共有メモリのコヒーレンシ（`.axi_dma` 両端 32B align）
- HardFault の有無（`CFSR`/`HFSR`/`BFAR`、スタックフレーム）。ハンドラ未定義で
  `Default_Handler` の無限ループに落ちていないか
- スタックサイズ不足（ThreadX スレッドスタック / MSP）。`free` の high-water 表示で見る
- 外部 OCTOSPI2 領域（`0x70000000`）への意図しないアクセス — mmap されていないので
  MPU の no-access + XN に当たる。MPU を外すと**未初期化 mmap read が AXI 無期限ストール**になる
- ST 公式デモ（`_ref/STM32Cube_FW_H7_V1.13.0/.../STM32H735G-DK/`）の同等ペリフェラル設定と突き合わせ

### 4. 観測の確実性

- 確定的に再現するか、間欠的（flaky）か。**間欠なら「人間が無意識にリトライしていて
  気づかれていない経路」を疑う**（#40 の実例: 起動時経路だけが一発勝負で毎回踏んでいた）
- 🔴 **交絡を先に潰す**。「同じような入力」では何も判定できない。カラーバーは ISP を迂回しない。
  **対象が実際に何を見たかを、その瞬間に吐かせる**
- 🔴 **カウンタが全 0 なのはデータが無事な証拠にならない**
- CDC 出力が途中で止まる場合、ハングか HardFault か tick 停止かを SWD で切り分ける
  （`_tx_timer_system_clock` が進むか、`$pc` が `Default_Handler`/`__tx_ts_wait` か）
- USB が enumerate しない場合、`dfu-util -l` / `lsusb` で見えるか、`/dev/ttyACM0` が生えるか
  （app = `0483:5740`、boot = `0483:df11`）

## SWD デバッグ補助（この環境固有）

- toolchain 同梱の `arm-none-eabi-gdb` は `libncursesw.so.5` 欠如で動かない → システムの
  **`gdb-multiarch`** を使う
- GDB サーバは `st-util`（:4242）か **OpenOCD**（`openocd -f interface/stlink.cfg -f
  target/stm32h7x.cfg`、:3333）。OpenOCD の方が SCS レジスタ読み出しが安定
- **良品 Discovery ST-Link**（SN `51FF..`, FW `V2J46S0`, mode=UR 可）を焼き/接続に使う。
  V2 clone（`0483:3748`）は通常 SWD 可・UR 不可
- **SWD 限界**: app はクロックを継承して再設定しないので比較的追従するが、app 内観測の主力は
  **USB CDC の shell コンソール**（`/dev/ttyACM0`）
- CDC（picocom 等）と `st-flash`/読み出しは `/dev/ttyACM0` を奪い合うと文字化けする

## プロンプトに必ず入れるプロジェクトコンテキスト

MCU: STM32H725AEI6 / Cortex-M7 @ 550 MHz。DFU ブートローダ（内蔵 Flash セクタ0
`0x08000000`）は不変。**app は内蔵 Flash `0x08020000`（セクタ1-3, 384 KB）から実行**し、
**RCC を再設定しない**（クロックはブートローダから継承）。外部 OCTOSPI2 `0x70000000` は
mmap されず MPU で no-access + XN。RAM は AXI-SRAM 320 KB @`0x24000000`（DMA が見える唯一の
RAM）/ DTCM 128 KB @`0x20000000`（`_estack = 0x20020000`）/ ITCM 64 KB @`0x00000000`。
Eclipse ThreadX、コンソールは USB CDC（USB1_OTG_HS を FS 内蔵 PHY 動作）、ST HAL、CMake+Ninja。
