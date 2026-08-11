# Wio Lite AI プロジェクト

Seeed **Wio Lite AI**（STM32H725AEI6 / Cortex-M7 @ 550 MHz）向けファームウェア。
内蔵フラッシュ **セクタ0 `0x08000000`（128KB）** に**スタンドアロン USB DFU ブートローダ**
（`boot/`）が常駐し、アプリ本体は**内蔵フラッシュ セクタ1-3 `0x08020000`（384KB）から
実行**される（#25 で外部 OCTOSPI2 XIP `0x70000000` から移行。54 → 905 MB/s）。ST 公式 HAL、
ビルドは CMake + Ninja。HAL/CMSIS/ThreadX/TinyUSB のソースと ARM GNU ツールチェーンは
初回 configure で自動取得。

現行の作業目標: **`../stm32f746g-disco` の Eclipse ThreadX Shell を app 層に移植する。**
DFU ブートローダ（`boot/`）は**不変**。shell はブートローダが残したクロックツリーの上で
動く app として載せる。

## 🔴 最重要: 2 つの不変条件（読み飛ばし厳禁）

### 1. `boot/`（DFU ブートローダ）は不変

`boot/` と `ldscript/STM32H725AEIx_ROM.ld` は**触らない**。内蔵 `0x08000000` を焼き直す
操作は**ブリック本番**であり、現存する唯一の実機（board #2）を失う。app 開発で boot 側を
変更する必要は原則ない。もし触る必要が生じたら **必ず** codex-review（3 面）+ objdump 監査
+ バックアップを経てからユーザーに実機書込を依頼する（手順は `boot/README.md`）。

> **#25 は例外的にここへ触った**（app を内蔵実行へ移すため、内蔵フラッシュ書込経路の追加と
> OCTOSPI2 の撤去）。その際の作法がこのルールの実例: plan を codex-review 3 面 → app 先行で
> ドライバを実機検証 → objdump 監査（セクタ範囲チェックがバイナリに残っていることまで確認）
> → バックアップ → 良品 ST-Link(mode=UR) 接続下でユーザーが書込。**boot が内蔵フラッシュを
> 消去/書込するようになった今、`boot/iflash.c` の範囲チェックはセクタ0 を守る唯一の砦**
> なので、ここを緩める変更は絶対にしない。

### 2. app はクロックツリーを再設定しない

DFU ブートローダが `reset → clock.c → app へ jump` を済ませて渡す。app が RCC を再設定
（stock CMSIS `SystemInit` / `HAL_Init` / `SystemClock_Config` が行う）すると、**継承した
550 MHz / PLL3Q 48 MHz USB / FLASH_ACR latency 3 が全部壊れる**（HSI 64 MHz に落ちるのに
フラッシュ latency は 550 MHz 用のまま）。app の `SystemInit` はクロックを一切触らない
カスタム版（FPU + VTOR + ITCM ロードのみ）にする（`src/system_stm32h7xx.c` 参照）。
VTOR はリンカの `g_pfnVectors` から取る（アドレスをハードコードしない）。

> ThreadX は SysTick tick と PendSV を必要とする。SysTick の reload は**継承した
> `SystemCoreClock`（550 MHz）から計算する**が、その過程で **RCC は触らない**。詳細は
> 「ハードウェア要点 / ThreadX 統合」参照。

## 開発ワークフロー

### コード修正サイクル

以下を小さく繰り返す:

1. **コード修正** — 機能実装 or バグ修正（app 層のみ。`boot/` は触らない）
2. **ビルド** — `cmake --build build`
3. **フラッシュ（DFU）** — **PF1（USER ボタン）を保持したままリセット**で DFU モードに入り
   （赤 LED 点灯 / `dfu-util -l` で列挙）、`dfu-util -d 0483:df11 -a 0 -D build/<app>.bin`。
   書込先は内蔵 `0x08020000`（セクタ1-3）。manifest 後に自動 reboot して新 app 起動。
   （ST-Link 書込は boot をセクタ0 に焼く用だけ。app は DFU 経由。）
   ⚠ **内蔵の書換え耐久は ~10k サイクル**（外部は 100k だった）。自動ループで焼き直さない。
4. **動作確認** — ユーザーが実機で確認（**USB CDC シェルコンソール** = `/dev/ttyACM0` /
   LED / 必要なら SWD で観測）
5. **ドキュメント更新** — 変更に対応する `README.md` / 該当モジュールの README を更新。
   非自明な知見は永続メモリ（`~/.claude/projects/.../memory/`）へ
6. **コミット** — 動作確認後にコミット

**動作確認前にコミットしない。ドキュメント／メモリ更新を忘れない。**

### Plan + Codex review ワークフロー

**Phase 系 / architecture を変える plan は、plan 確定前と実装後の両方で codex-review を
実施する。** 本プロジェクトはブリックのリスクが高いので、以下は特に厳格に扱う:

対象となる plan:
- app のクロック継承／`SystemInit`／VTOR／起動フローに関わる変更
- ThreadX 統合方針（`_tx_initialize_low_level`、SysTick、PendSV、tick 供給、スタック）
- 新規ペリフェラル採用、割込み優先度・DMA・キャッシュ構成の変更
- USB CDC コンソール backend など複数レイヤ（HW + HAL + TinyUSB/ThreadX + shell）に跨る変更
- リンカスクリプト／メモリ配置（内蔵 `0x08020000` / AXI-SRAM）の変更
- `boot/` に触れる一切の変更（原則やらないが、やるなら最優先で厳格 review）

ゲートのタイミング:

1. **Plan 確定前**（実装着手前）: plan を `codex-review` skill で 3 面（設計 / MCU 実機能
   (RM0468 照合) / HW リソース競合）review。BLOCKING / CONCERN を全解消してから
   `ExitPlanMode`。
2. **実装後**（commit 前）: branch の diff を `codex-review` で再 review。BLOCKING 解消 →
   user に実機 verify 依頼 → commit。

`codex-review` skill は `.claude/skills/codex-review/` で定義済（sandbox は
`danger-full-access`、approval-policy は `never`、cwd は絶対パス）。

**ゲートの強制**: `ExitPlanMode` の PreToolUse hook（`.claude/settings.json`）が
`~/.claude/.wio-lite-ai-plan-codex-reviewed` marker を確認する。marker が無い/古い（2h 超）と
block される。codex-review が LGTM に至ると marker を更新して通過。trivial plan で skip する
場合も **user 承認を得てから** `touch ~/.claude/.wio-lite-ai-plan-codex-reviewed`。

不具合解析には `codex-debug` skill を使う。

## Git ワークフロー

**PR は作らない。** Issue 駆動で feature/fix ブランチを切り、ローカル `main` に `--ff-only`
merge → push → Issue へ対応コメント → Issue クローズ、の流れ。リポジトリは
`owhinata/wio-lite-ai`。

- **ブランチ**: `feat/`, `fix/`, `docs/`, `build/`, `refactor/`, `chore/`, `style/` prefix。
  `<prefix>/<N>-short-description`（`<N>` は Issue 番号）。ベースは常に `main`
- **コミット**: conventional commits 形式 `type: short description`。Issue 対応時は
  `type: #N short description` で **subject に Issue 番号**を含める（GitHub のリンク生成＋
  オートクローズ判定のため）
- 🔴 **コミットメッセージは subject も body も英語で書く。** 会話・Issue・コード内コメント・
  README は日本語のままでよい（そこは変えない）。**コミットメッセージだけが英語**。
  > 2026-07-26〜2026-08-08 の 41 件は日本語になっている。言語を規定していなかったので
  > 漂流した。**これらは書き換えない** — ハッシュが変われば 11 個の Issue のコミットレンジと
  > 永続メモリの参照が全部 stale になり、コストが利得を上回る（force push 禁止とも衝突する）。
  > 履歴に日本語の地層があるのは既知であって、直すべき不整合ではない。
- コミットメッセージ末尾に `Co-Authored-By: Claude ...` を付与
- 動作確認していない変更を commit / push しない

### 手順

```bash
# 1. Issue 作成（着手前に必ず）
gh issue create --repo owhinata/wio-lite-ai --title "short description" --body "$(cat <<'EOF'
## Summary
- 症状・問題・やりたいことの説明
## Environment
- Board: Wio Lite AI (STM32H725AEI6 / Cortex-M7)  ※実機は board #2 のみ（board #1 は文鎮化）
- Reproduced at: <commit-hash>
## Notes
- 調査メモ・仮説・設計案
EOF
)"

# 2. ブランチを切って実装 → ビルド → DFU フラッシュ → 実機で動作確認 → コミット
git checkout -b feat/<N>-short-description
git commit -m "type: #<N> short description"

# 3. ローカル main に ff-merge（merge コミットは作らない）
git checkout main
git merge --ff-only feat/<N>-short-description

# 4. push（subject の `#<N>` / `closes #<N>` でオートクローズ）
git push origin main

# 5. Issue へ対応コメント（コミットレンジ <base>..<head> を必ず含める）
gh issue comment <N> --repo owhinata/wio-lite-ai --body "..."

# 6. Issue クローズ（未クローズなら）＋ マージ済みブランチ削除
gh issue close <N> --repo owhinata/wio-lite-ai
git branch -d feat/<N>-short-description
```

### 注意

- **PR は作らない**。`gh pr create` / `gh pr merge` は使わない
- `main` への直 push なので `--ff-only` を厳守、**force push は禁止**
- 動作確認していないコミットを `main` に push しない
- Issue を立てずにコミットしない（`#<N>` 参照が無いコミットは追跡できない）。Epic / 親 Issue
  は、クローズキーワードを使わず `#<epic>` 参照のリンクのみとする（誤クローズ防止）

### Upstream submodule は read-only

以下は upstream のミラー submodule。`gh` での書き込み操作（PR/issue/comment 等）を行っては
ならない（PreToolUse hook がブロックする）。コードも編集せず、必要なら `port/` 側のグルーで
吸収する:

- `STMicroelectronics/stm32h7xx_hal_driver`, `cmsis_device_h7`, `cmsis-core`
- `hathach/tinyusb`（USB DFU/CDC スタック。`lib/tinyusb`、tag 0.21.0 に pin）
- `eclipse-threadx/threadx`（移植で追加予定。filex/levelx/netxduo/guix も同様）

## ビルド / フラッシュ

```bash
# Configure（初回に ARM ツールチェーンを ./tools へ自動 DL）
cmake -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-toolchain.cmake

# ビルド（成果物: build/{boot,shell,blink}.{elf,bin,hex}。UF2 は廃止済で出ない）
cmake --build build

# app を DFU で焼く（PF1 保持リセット → DFU モード → 書込 → 自動 reboot）
dfu-util -d 0483:df11 -a 0 -D build/<app>.bin
```

`boot`（内蔵 `0x08000000`）を焼く手順は **`boot/README.md`**（ブリック本番。通常やらない）。

## ハードウェア要点

- MCU: STM32H725AEI6 / Cortex-M7、**550 MHz**。DFU ブートローダが構成する
  クロックを継承する（HSE 25 MHz → PLL1 M2 N44 → VCO 550、P/1 = sysclk 550 CPU、
  VOS0 + SMPS 直結供給、FLASH ACR=0x33 latency 3）。**app は RCC を再設定しない**。
- FPU on。I-Cache / D-Cache は両方 ON（DMA マスタ不在なので自己コヒーレント）。
- **メモリ配置**（`ldscript/STM32H725AEIx_IROM.ld`）:
  - 内蔵 Flash 512 KB = **1 バンク・128 KB セクタ × 4**（RM0468 Table 15）。
    **セクタ0 `0x08000000` = DFU ブートローダ専用**、**セクタ1-3 `0x08020000` 384 KB = app**。
    消去単位がセクタ境界と一致するので、app 書込経路はセクタ0 に物理的に到達できない。
    app のベクタ table 先頭に MSP / Reset。
  - 🔴 **RAM 配置ポリシー（#46）**: **AXI-SRAM = バスマスタから見える必要があるものだけ /
    DTCM = CPU 専用のホットなもの / PSRAM = bulk・cold / ITCM = ISR コード**。
    DMA1/DMA2 と SDMMC1 IDMA は **TCM に届かない**（RM0468 §2.1.2/§2.1.5/§2.1.6）ので
    AXI-SRAM は唯一 DMA から見える RAM ＝希少資源。**DTCM に DMA バッファを置くと fault せず
    無言で「何も転送されない」**。新しいバッファを足すときは必ずこの表で行き先を決める。
  - RAM: **AXI-SRAM (D1) 320 KB @ 0x24000000**。`.data`/`.bss`/`.axi_dma`(SDMMC1 bounce)/heap。
    上端は `__ram_end`（heap の天井。`_estack` ではない）。
  - DTCM: 128 KB @ 0x20000000。`.log_noinit`(dmesg) / `.nx_pool` / `.dtcm_bench` /
    `.dtcm_bss`（**全スレッドスタック + RTL8720 UART リング**）、そして**最上端が
    メインスタック** = `_estack = 0x20020000`（ブートローダが vector[0] からロードする MSP。
    `boot/main.c` が `0x20000000` を valid RAM として受け入れる）。
    ARMv7-M に MSPLIM は無く MPU ガードは lockup を招くので、防御は `free` の high-water 表示。
  - ITCM 64 KB @ 0x00000000。
  - **配置は 2 つのポストリンクゲートが守る**（リンカ ASSERT は LTO 下で空振りする）:
    `cmake/check_itcm_residency.py` と `cmake/check_dtcm_residency.py`。後者は
    「スタックが AXI に戻った」と「DMA バッファが DTCM に紛れた」の**両方向**を検査する。
  - 外部 OCTOSPI2 `0x70000000`（W25Q128 16 MB）は **#25 以降どこからも mmap されない**。
    `app/mpu.c` が 256 MB を no-access + XN で塞いでいる（未初期化 mmap read は AXI 無期限
    ストールになるため）。ストレージとして使うなら app 側 bring-up から＝ #10 の範疇。
- **コンソール = USB CDC**: 単一 USB = **USB1_OTG_HS を FS（内蔵 PHY）動作**。CMSIS に
  `USB2_OTG_FS` / `OTG_FS_IRQn` は**無い** → TinyUSB(dwc2) は rhport0 を OTG_HS base +
  `OTG_HS_IRQHandler` にエイリアス（`tud_int_handler(0)`）。GPIO = PA11/PA12
  `GPIO_AF10_OTG1_FS`。USB クロックは PLL3Q 48 MHz。app 動作時 **`0483:5740`**
  "CDC in FS Mode" → `/dev/ttyACM0`（`app/usb_descriptors.c`。boot は `0483:df11` = DFU）。
  🔴 **ここは長く `8050:2886` と書かれていたが誤り**で、#55 で実機の enumeration と
  descriptor の両方から訂正した。`0483:5740` は **ST の汎用 VCP ID** なので、VID/PID で
  DUT を選ぶホスト側ツールは他の ST デバイスと衝突しうる（MLPerf のランナーはこの ID を
  LPM01A 電力計に割り当てている＝`scripts/mlperf/devices_wio.yaml` が要る理由）。
  **shell の UART backend はこの USB CDC backend に差し替える**のが移植の要。
- **LED / ボタン**（schematic sheet8）:
  - LED0（赤）= **PC13**（NPN バッファ経由。低ドライブの backup-domain ピン）
  - LED1（黄）= **PF0**
  - USER ボタン = **PF1**（active-low）。**保持リセットで DFU モード**に入る。
- リファレンス: `_ref/`（git 管理外）
  - 🔴 **`_ref/` は「読むための資料」専用で、ビルドは一切ここを読まない**（#58）。
    CMake / スクリプト / `fw/rtl8720/build.sh` が `_ref/` を参照してはいけない
    — git 管理外なので、参照した瞬間に**クローンしただけでは configure できない
    リポジトリ**になる（実際そうなっていた）。ビルド入力は「pin して fetch」、
    ビルド作業状態は `build/` か `fw/rtl8720/vendor/`（どちらも git-ignore）。
    C コードの `_ref/...` 言及は**出典コメントのみ**で、これは依存ではないので残す。
  - `_ref/rm0468-*.pdf` — RM0468（H723/733, H725/735, H730）レジスタ/ペリフェラル根拠
  - `_ref/pm0253-*.pdf` — Cortex-M7 プログラミングマニュアル（NVIC/キャッシュ/FPU）
  - `_ref/733260648-Wio-Lite-AI-v1-0-SCH-Final-*.pdf` — 基板 schematic（配線・ピン）
  - `_ref/STM32Cube_FW_H7_V1.13.0/` — ST 公式 FW（H735G-DK の OSPI/クロック/BSP 参照実装）
  - `_ref/tinyuf2/`, `_ref/tinyusb/` — DFU/jump/valid・dwc2 の参照（read-only）

### ThreadX 統合（移植の要点）

- **SysTick > PendSV**（優先度）。同一だと idle 時 PendSV スピンを tick が割り込めず tick 停止 →
  スリープ中スレッドが起床しないデッドロック（f746 で実証済みの教訓）。PendSV は最低優先度。
- SysTick reload は継承した `SystemCoreClock`（550 MHz）から算出。**RCC は触らない**。
  ThreadX は自前で PendSV_Handler を供給する（`stm32h7xx_it.c` の PendSV とは競合させない）。
- ThreadX クリティカルセクションは PRIMASK ベース（`TX_PORT_USE_BASEPRI` 未定義）にしておくと、
  USB(OTG_HS) ISR が `tx_event_flags_set` を呼んでも ThreadX クリティカルセクションを preempt
  できず安全（NVIC 優先度は echo レイテンシ調整目的にとどまる）。
- shell は静的割当（ヒープ非使用）。スタックサイズ・スレッド優先度は `shell/include/cli_config.h`
  の既定を踏襲（移植先でも `_Static_assert` を通す）。

### レイヤリング

一方向依存を守る: **HAL/CMSIS/ThreadX ← port/ ← shell/ ← app（src）**。上位が下位へ潜り込む
グルーは `port/` 側に置く。`boot/` は独立（app とソースを共有しない）。

## SWD デバッグ / デバッガ

- **良品 Discovery ST-Link**（SN `51FF72064987505349271187`, FW `V2J46S0`）: CubeProgrammer
  `mode=UR`（hard connect-under-reset）が実証済。**焼き/復旧はこれを使う**。CLI:
  `/home/ouwa/work/STM32CubeProgrammer/bin/STM32_Programmer_CLI`。V2 clone（`0483:3748`,
  `V2J17S4`）は通常 SWD 可・**UR 不可**、焼きには使わない。
- **SWD 限界**: H7 の PLL 再設定で SWD 接続が落ちることがある（boot 側）。app はクロックを
  継承して**再設定しない**ので比較的追従するが、app 内観測の主力は **USB CDC の printf /
  shell コンソール**。GDB は `gdb-multiarch`、サーバは OpenOCD
  （`-f interface/stlink.cfg -f target/stm32h7x.cfg`）。VCP と `st-flash`/読み出しは
  `/dev/ttyACM0` を奪い合うと文字化けする（SWD と CDC は別系統）。

## ブリック安全則（絶対）

- **board #1 = 恒久文鎮化**（復旧不能）。**board #2 = 現存する唯一の開発ターゲット**。
- 内蔵 `0x08000000`（boot）に触れる操作は最後の手段。焼く前に objdump 監査 + バックアップ +
  良品 ST-Link mode=UR 安全ゲート（`boot/README.md`）。
- **オプションバイト / RDP / DBGMCU / SWD 端子（PA13/14）は絶対に触らない**。悪い config でも
  SWD で再書込できる状態を必ず保つ。
- app は DFU フォールバックで常に再ロード可能（erased/invalid app は必ず DFU モードに入る。
  中断した DFU 転送も、先頭 32B を最後に書く vector-last commit により必ずここに落ちる）
  ——この安全網を壊す変更（boot の DFU 判定条件など）を app 側から入れない。
