---
name: codex-review
description: Codex による設計・実装レビュー。Wio Lite AI (STM32H725) のクロック継承 / XIP(OCTOSPI2) / ThreadX 統合 / USB CDC / DFU ブートローダ境界などハードウェア制約を伴う変更に使う。plan モードの計画レビューにも対応。
argument-hint: <plan | PR number | file path | design description>
---

# Codex 設計・実装レビュー (Wio Lite AI / STM32H725AEI6)

## 対象の判定

`$ARGUMENTS` の内容に応じてレビュー対象を決定する:

- **`plan`**: 現在の会話コンテキストにある実装計画をレビュー対象とする。使用予定の
  ペリフェラル・クロック依存・メモリ配置・割込み優先度・依存する HAL/ThreadX/TinyUSB API を
  抽出してから Codex に送る。**実装着手前のゲートとして機能する。**
- **PR 番号 / `git diff`**: diff と説明を取得してレビュー対象とする
- **ファイルパス**: 指定ファイルの内容をレビュー対象とする
- **その他テキスト**: 設計説明としてそのままレビュー対象とする

## レビュー実行手順

1. レビュー対象の情報を収集する（plan / diff / ファイル）
2. 必要に応じて `_ref/`（git 管理外）のリファレンスを参照する
   - `_ref/rm0468-*.pdf` — RM0468（STM32H723/733, H725/735, H730）レジスタ/ペリフェラル根拠
   - `_ref/pm0253-*.pdf` — Cortex-M7 プログラミングマニュアル（NVIC 優先度 / キャッシュ / FPU / MPU）
   - `_ref/733260648-Wio-Lite-AI-v1-0-SCH-Final-*.pdf` — 基板 schematic（配線・ピン・プルアップ）
   - `_ref/STM32Cube_FW_H7_V1.13.0/.../STM32H735G-DK/` — ST 公式 OSPI/クロック/BSP 参照実装（read-only）
   - `_ref/tinyuf2/`, `_ref/tinyusb/` — DFU/jump/valid・dwc2(OTG_HS) の参照（read-only）
   - `boot/README.md` — ブートローダが app へ渡すクロック/OCTOSPI2 の実測レジスタ値
3. 以下の「3 面レビュー観点」に基づいてレビュープロンプトを構成する
4. `mcp__codex__codex` で Codex にレビューを依頼する
5. 結果を整理してユーザーに報告する
6. **plan レビューの場合**: 3 面すべてで問題なしなら「実装着手 OK」とし、
   `touch ~/.claude/.wio-lite-ai-plan-codex-reviewed` で marker を更新する。問題ありなら
   具体的な修正提案を示し、**marker は更新しない**（BLOCKING/CONCERN を解消し再 review して
   LGTM に至ってから touch する）。
   - この marker は `ExitPlanMode` の PreToolUse gate（`.claude/settings.json`）が確認する。
     marker が無い/古い（2h 超）と ExitPlanMode は block される。**plan review を通さずに
     ExitPlanMode することは構造的に防がれる。**
   - trivial plan で skip する場合も、user 承認を得てから
     `touch ~/.claude/.wio-lite-ai-plan-codex-reviewed` すれば gate を通過できる。

## 3 面レビュー観点

それぞれ**独立したチェック**として実施する。1 面が LGTM でも、他面が未確認なら全体 LGTM に
しない。

### 観点 1: 設計レビュー

- アーキテクチャの妥当性、レイヤ分離、API 設計（一方向依存 HAL/CMSIS/ThreadX ← port ← shell ← app）
- ST HAL の使い方・初期化順序が HAL の前提と整合しているか
- ThreadX 統合の正しさ（`tx_application_define`、スタックサイズ、`_tx_initialize_low_level`、
  tick 供給、PendSV/SysTick）
- shell 移植の妥当性（transport 抽象 `struct cli_transport_api` を USB CDC backend で実装、
  静的割当の維持、`cli_config.h` の `_Static_assert` を通すか）
- エラーハンドリング、排他制御、エッジケース

### 観点 2: MCU 実機能レビュー (RM0468 / schematic 照合)

**「API がコンパイルできる」≠「H725 で期待通り動く」**。レジスタ/能力の根拠を RM0468 で確認する。

このプロジェクト固有の最重要チェック:

- **クロック継承（app は RCC を再設定しない）**: app 側の `SystemInit`/初期化が RCC/PLL/FLASH
  ACR/PWR を書き換えていないか。書き換えると OCTOSPI2 XIP の命令フェッチが止まりハングする。
  カスタム `SystemInit` は FPU + VTOR のみか。`SystemCoreClock` が 550 MHz に設定されているか
  （SysTick reload 計算の根拠）。
- **XIP(OCTOSPI2 `0x70000000`)**: VTOR が `0x70000000` を指すか。OCTOSPI2 の mmap を abort
  する処理（自己書込等）が XIP から実行されるコードを含んでいないか（`.RamFunc` へ退避が必要）。
  D-Cache 有効時のコヒーレンシ（OCTOSPI2/DMA 共有メモリ）。
- **ThreadX tick**: SysTick を 550 MHz から正しく分周しているか。**SysTick > PendSV**（優先度）。
- **USB CDC**: 単一 USB = USB1_OTG_HS を FS 内蔵 PHY で駆動。`OTG_HS_IRQHandler` →
  `tud_int_handler(0)`。`CFG_TUD_MAX_SPEED=OPT_MODE_FULL_SPEED` で FS 強制。USB クロック
  = PLL3Q 48 MHz。GPIO = PA11/PA12 AF10。
- ピンの AF 番号が schematic / RM0468 の alternate function mapping と一致するか
  （LED0=PC13, LED1=PF0, USER=PF1, OCTOSPI2=PF4/PG0/1/10/11/12 等）。

過去の事例（f746 由来の教訓・本プロジェクトにも適用）:
- **ThreadX SysTick 優先度**: SysTick を PendSV と同一優先度にすると tick が停止し、スリープ中
  スレッドが起床しないデッドロック。**SysTick > PendSV** 必須。
- **クロック継承の破壊**: HAL_Init/SystemClock_Config を安易に呼ぶと XIP が死ぬ。app は継承のみ。

### 観点 3: HW リソース競合レビュー

該当するリソースを使う場合は現状の割当と照合する。

- **割込み優先度 (NVIC/SCB)**: Cortex-M7 の優先度ビット数（H7=4bit）。ThreadX 使用時は
  PendSV=最低、SysTick>PendSV。OTG_HS ISR と ThreadX クリティカルセクション（PRIMASK ベース）
  の関係。HAL ISR と競合しないか。
- **メモリ領域**: FLASH(XIP `0x70000000`) / AXI-SRAM(`0x24000000` 320KB, `_estack`=0x24050000) /
  DTCM(`0x20000000` 128KB) / ITCM。リンカスクリプトと startup の symbol 整合
  （`_estack`, `_sidata`, `_sdata`, `_ebss` 等）。**内蔵 `0x08000000` は boot 専用（侵さない）**。
- **GPIO / AF / ペリフェラル**: ピン多重割当（LED PC13/PF0、USER PF1、OCTOSPI2 の PF/PG 群、
  USB PA11/12）。OCTOSPI2 は XIP で常時使用中——再設定は XIP を殺す。
- **boot 境界**: 変更が `boot/`・`ldscript/*ROM.ld`・内蔵フラッシュに波及していないか
  （app 変更で boot 側に触れるのは原則 NG。触れるなら別途最優先で厳格 review）。

## 成立性の証拠

HW 依存の設計には、LGTM 前に成立性の証拠を要求する:
- RM0468 のレジスタ記述に基づく根拠（節番号まで）／`boot/README.md` の実測レジスタ値との整合
- 最小実機テスト or 観測（USB CDC バナー/`[tick]`、LED 挙動、必要なら SWD/OpenOCD）
- 「コンパイルが通った」は証拠にならない。特に「クロックを触っていないこと」「XIP がハングしない
  こと」は実機 or objdump/`.map` 監査で裏取りする

## Codex へのプロンプト構成

1. レビュー対象のコード/設計の説明
2. 該当する 3 面観点を明示的に指示
3. プロジェクトコンテキスト（MCU: STM32H725AEI6 / Cortex-M7 @ 550 MHz、DFU ブートローダ不変・
   app は外部 OCTOSPI2 `0x70000000` から XIP・**RCC 再設定禁止**、Eclipse ThreadX、USB CDC
   コンソール、ST HAL、CMake+Ninja）
4. 関連するリファレンス（RM0468 節、schematic 配線、`boot/README.md` 実測値）
5. 「LGTM を出す場合は該当面すべてについて根拠を示すこと」という指示

## `mcp__codex__codex` 呼び出しパラメータ

レビュー目的の Codex 呼び出しは以下を既定にする（ユーザー承認済み）:

```
sandbox: "danger-full-access"
approval-policy: "never"
cwd: "/home/ouwa/work/wio-lite-ai"   (絶対パス)
```

### 理由

- `sandbox: "read-only"` / `"workspace-write"` は環境によって bwrap loopback で失敗し
  （`bwrap: loopback: Failed RTM_NEWADDR`）、Codex のローカル shell が起動せず「推測レビュー」
  になる。
- `danger-full-access` は bwrap を経由しないので Codex が `Read` / `git diff` / `grep` で
  ローカルファイルを参照できる。レビュー目的なら破壊操作は提案されないため実害なし。
- 避けたい場合は plan + diff を **prompt に inline で貼り付ける** fallback も可（精度はやや落ちる）。
