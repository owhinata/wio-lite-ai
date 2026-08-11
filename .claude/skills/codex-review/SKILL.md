---
name: codex-review
description: Codex による plan（実装計画）レビュー。ExitPlanMode ゲートの marker を更新する唯一の経路。Wio Lite AI (STM32H725) のクロック継承 / 内蔵 Flash 配置 / ThreadX 統合 / USB CDC / DFU ブートローダ境界に関わる計画に使う。実装後の diff レビューは codex plugin の /codex:review・/codex:adversarial-review を使う（この skill ではない）。
argument-hint: <plan | 設計説明>
---

# Codex plan レビュー (Wio Lite AI / STM32H725AEI6)

## この skill の役割 / 役割でないもの

Codex 呼び出しは **codex plugin（`codex@openai-codex`）のランタイムに一本化**されている。
MCP server（`mcp__codex__codex`）はもう使わない。用途ごとの入口は以下:

| レビュー対象 | 使うもの |
|---|---|
| **実装計画（plan mode / 会話中の設計）** | **この skill**（ExitPlanMode の marker を更新する） |
| 実装後の差分（working tree / branch） | `/codex:review`（Codex 内蔵レビュアー。focus text は渡せない） |
| 差分を観点付きで叩く / 設計判断そのものを疑う | `/codex:adversarial-review [--base <ref>] <focus>` |
| 不具合の原因追跡 | `codex-debug` skill、または `/codex:rescue` |

**plan だけがこの skill に残っている理由**: plugin の review コマンドは git 差分専用で、
まだコードになっていない会話中の plan をレビューできない。ブリック対策の
「Plan 確定前ゲート」（CLAUDE.md）はコード以前に効く必要があるので、ここだけ自前で持つ。

## 実行手順

### 1. plan を自己完結した 1 枚のプロンプトに落とす

Codex は**この会話のコンテキストを見られない**。「上記の plan」のような参照は書かない。
scratchpad にプロンプトファイルを書く（`$CLAUDE_SCRATCHPAD` が無ければ `/tmp` 配下でよい）:

- plan の全文（変更するファイル、追加する初期化、触るペリフェラル、割込み、メモリ配置）
- 下の「プロジェクト不変条件」（`AGENTS.md` にも置いてあるが、プロンプトにも明示する）
- 下の「3 面レビュー観点」を**明示的に 3 つとも指示**
- 「LGTM を出す場合は該当面すべてについて根拠（RM0468 の節番号 / ファイル:行）を示すこと」
- 出力形式の指定: 面ごとに `LGTM` / `CONCERN` / `BLOCKING` と、根拠と修正案

Codex は read-only サンドボックスでローカルシェルを使える（`git diff`・`grep`・`sed` が通る
ことは実測済み）。つまり**リポジトリの実物を読ませてよい** — plan 中のファイルパスは
そのまま書けば Codex が開く。

### 2. Codex に投げる

plugin のバージョン番号をハードコードしないこと（更新で壊れる）:

```bash
node "$(ls -d "$HOME"/.claude/plugins/cache/openai-codex/codex/*/scripts/codex-companion.mjs \
        | sort -V | tail -1)" task --effort xhigh --prompt-file /abs/path/to/plan-review.md
```

（コマンドは **`node` で始める**こと。先頭に変数代入を置くと `Bash(node:*)` の
permission ルールに当たらず毎回プロンプトが出る。）

- `--write` は**付けない**。付けなければサンドボックスは `read-only`（レビューに書込は不要）。
- `--effort` は `xhigh`（ブリック対策のゲートなので最大で回す）。`--model` で明示指定も可。
- **`Bash(run_in_background: true)` で起動して待つ**。plan review は毎回 120s を超えるので
  フォアグラウンドだとツール側でタイムアウトする。出力ファイルを読んで結果を取る。
  （スコープを削ってフォアグラウンドに収めると根拠の薄いレビューになる方が損。）

### 3. 結果を整理してユーザーに報告

面ごとに verdict と根拠を並べる。Codex の指摘は**そのまま鵜呑みにしない** — RM0468 や
リポジトリの実物で裏を取れる指摘かどうかを確認してから報告する。
（実測: 内蔵レビュアーが CMake の `file(DOWNLOAD)` について事実と異なる P1 を出した例がある。）

### 4. marker の更新（ここがゲート）

3 面すべて問題なしなら「実装着手 OK」とし、marker を更新する:

```bash
touch ~/.claude/.wio-lite-ai-plan-codex-reviewed
```

- 問題ありなら**marker は更新しない**。BLOCKING / CONCERN を解消して再 review し、
  LGTM に至ってから touch する。
- この marker は `ExitPlanMode` の PreToolUse gate（`.claude/settings.json`）が確認する。
  marker が無い / 古い（2h 超）と ExitPlanMode は block される。
- trivial plan で skip する場合も、**user 承認を得てから** touch する。

## 3 面レビュー観点

それぞれ**独立したチェック**として実施する。1 面が LGTM でも、他面が未確認なら全体 LGTM に
しない。

### 観点 1: 設計レビュー

- アーキテクチャの妥当性、レイヤ分離、API 設計
  （一方向依存 **HAL/CMSIS/ThreadX ← port/ ← shell/ ← app(src)**）
- ST HAL の使い方・初期化順序が HAL の前提と整合しているか
- ThreadX 統合の正しさ（`tx_application_define`、スタックサイズ、`_tx_initialize_low_level`、
  tick 供給、PendSV/SysTick、割込みは TX オブジェクト生成後に有効化しているか）
- shell の transport 抽象（`struct cli_transport_api`）を壊していないか、静的割当を維持して
  いるか、`cli_config.h` の `_Static_assert` を通すか
- エラーハンドリング、排他制御、エッジケース

### 観点 2: MCU 実機能レビュー (RM0468 / schematic 照合)

**「API がコンパイルできる」≠「H725 で期待通り動く」**。レジスタ/能力の根拠を RM0468 で確認する。

- **クロック継承（app は RCC を再設定しない）**: app 側の `SystemInit`/初期化が
  RCC/PLL/FLASH ACR/PWR を書き換えていないか。書き換えると継承した 550 MHz / PLL3Q 48 MHz USB /
  FLASH latency 3 が全部壊れる（HSI 64 MHz に落ちるのに latency は 550 MHz 用のまま）。
  カスタム `SystemInit` は **FPU + VTOR + ITCM ロードのみ**か。VTOR はリンカの `g_pfnVectors`
  から取っているか（アドレスのハードコードは不可）。`SystemCoreClock` が 550 MHz か
  （SysTick reload 計算の根拠）。
- **ThreadX tick**: SysTick を継承 `SystemCoreClock` から正しく分周しているか。
  **SysTick > PendSV**（優先度）。PendSV は最低優先度。ThreadX が自前で供給する
  `PendSV_Handler` と `stm32h7xx_it.c` のものを競合させていないか。
- **USB CDC**: 単一 USB = **USB1_OTG_HS を FS（内蔵 PHY）動作**。CMSIS に `USB2_OTG_FS` /
  `OTG_FS_IRQn` は無く、TinyUSB(dwc2) は rhport0 を OTG_HS base + `OTG_HS_IRQHandler` に
  エイリアスしている（`tud_int_handler(0)`）。GPIO = PA11/PA12 `GPIO_AF10_OTG1_FS`。
  USB クロックは PLL3Q 48 MHz。app の VID/PID は **`0483:5740`**（ST 汎用 VCP）。
- **メモリ配置と実行元**: app は**内蔵 Flash `0x08020000`（セクタ1-3, 384 KB）から実行**する
  （#25 で外部 XIP から移行済）。外部 OCTOSPI2 `0x70000000` は**どこからも mmap されない** —
  `app/mpu.c` が 256 MB を no-access + XN で塞いでいる（未初期化 mmap read は AXI 無期限
  ストールになるため）。外部 NOR を使うなら indirect ドライバ経由。
- ピンの AF 番号が schematic / RM0468 の alternate function mapping と一致するか
  （LED0=PC13, LED1=PF0, USER=PF1）。

### 観点 3: HW リソース競合レビュー

- **割込み優先度 (NVIC/SCB)**: H7 は 4 bit。ThreadX 使用時は PendSV=最低、SysTick>PendSV。
  ThreadX クリティカルセクションは **PRIMASK ベース**（`TX_PORT_USE_BASEPRI` 未定義）なので
  OTG_HS ISR が `tx_event_flags_set` を呼んでも preempt できない。HAL ISR と競合しないか。
- 🔴 **RAM 配置ポリシー（#46）**: **AXI-SRAM = バスマスタから見える必要があるものだけ /
  DTCM = CPU 専用のホットなもの / ITCM = ISR コード**。
  **DMA1/DMA2 と SDMMC1 IDMA は TCM に届かない**（RM0468 §2.1.2/§2.1.5/§2.1.6）。
  **DTCM に DMA バッファを置くと fault せず無言で「何も転送されない」**。
  新しいバッファを足す plan は必ずこの表で行き先を決めさせる。
  - AXI-SRAM (D1) 320 KB @ `0x24000000` — `.data`/`.bss`/`.axi_dma`/heap。上端は `__ram_end`
    （heap の天井。`_estack` ではない）。
  - DTCM 128 KB @ `0x20000000` — `.log_noinit`/`.nx_pool`/`.dtcm_bench`/`.dtcm_bss`
    （全スレッドスタック）、最上端がメインスタック `_estack = 0x20020000`。
  - ITCM 64 KB @ `0x00000000` — `.itcm`（ISR コード）。`.text` より前に置く。
- 🔴 **リンカスクリプトの `ASSERT` は LTO 下で空振りする**。配置の最終ガードは POST_BUILD の
  `cmake/check_itcm_residency.py` / `cmake/check_dtcm_residency.py`。配置を変える plan は
  このゲートを通るか（両方向検査: スタックが AXI に戻った / DMA バッファが DTCM に紛れた）。
- **boot 境界**: 変更が `boot/`・`ldscript/STM32H725AEIx_ROM.ld`・内蔵 Flash セクタ0 に波及して
  いないか。**`boot/iflash.c` の範囲チェックはセクタ0 を守る唯一の砦**で、緩める変更は不可。
- **ビルド入力**: `_ref/` は git 管理外なので、CMake / スクリプトが `_ref/` を読む plan は
  「クローンしただけでは configure できないリポジトリ」を作る（#58）。C コード中の
  `_ref/...` は出典コメントのみ可。
- **書換え耐久**: 内蔵 Flash は ~10k サイクル。自動ループで焼き直す plan は不可。

## 成立性の証拠

HW 依存の設計には、LGTM 前に成立性の証拠を要求する:

- RM0468 のレジスタ記述に基づく根拠（**節番号まで**）／`boot/README.md` の実測レジスタ値との整合
- 最小実機テスト or 観測（USB CDC バナー、LED 挙動、`free` の high-water、必要なら SWD/OpenOCD）
- **「コンパイルが通った」は証拠にならない**。特に「クロックを触っていないこと」「配置が
  剥がれていないこと」は実機 or objdump/`.map` 監査で裏取りする

## リファレンス（`_ref/`、git 管理外・読む専用）

- `_ref/rm0468-*.pdf` — RM0468（H723/733, H725/735, H730）レジスタ/ペリフェラル根拠
- `_ref/pm0253-*.pdf` — Cortex-M7 プログラミングマニュアル（NVIC/キャッシュ/FPU/MPU）
- `_ref/733260648-Wio-Lite-AI-v1-0-SCH-Final-*.pdf` — 基板 schematic（配線・ピン）
- `_ref/STM32Cube_FW_H7_V1.13.0/.../STM32H735G-DK/` — ST 公式参照実装（read-only）
- `boot/README.md` — ブートローダが app へ渡す実測クロック値
