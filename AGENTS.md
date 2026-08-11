# Wio Lite AI — Codex 向けプロジェクト指示

Seeed **Wio Lite AI**（STM32H725AEI6 / Cortex-M7 @ 550 MHz）向けベアメタル/ThreadX
ファームウェア。ST 公式 HAL、CMake + Ninja、ARM GNU ツールチェーン。

このファイルは Codex（`/codex:review` の内蔵レビュアーを含む）が**毎回読む前提の要約**。
人間向けの詳細は `CLAUDE.md` にある。**レビュー時はここの不変条件を最優先の判定基準にする。**

## 🔴 不変条件（違反はそれだけで BLOCKING）

1. **`boot/` と `ldscript/STM32H725AEIx_ROM.ld` は不変。**
   内蔵 Flash セクタ0 `0x08000000`（128 KB）に DFU ブートローダが常駐する。ここを焼き直す
   操作はブリック本番で、**現存する実機は 1 枚しかない**（board #1 は恒久文鎮化済み）。
   **`boot/iflash.c` の書込先セクタ範囲チェックはセクタ0 を守る唯一の砦**で、緩める変更は不可。

2. **app はクロックツリーを再設定しない。**
   ブートローダが `reset → clock.c → app へ jump` を済ませて渡す。app が RCC/PLL/FLASH ACR/PWR
   を書き換えると、継承した 550 MHz / PLL3Q 48 MHz USB / FLASH latency 3 が全部壊れる
   （HSI 64 MHz に落ちるのに latency は 550 MHz 用のまま）。
   - app の `SystemInit` は **FPU + VTOR + ITCM ロードのみ**のカスタム版（`src/system_stm32h7xx.c`）。
     stock CMSIS `SystemInit` / `HAL_Init` の `SystemClock_Config` 相当は呼ばない。
   - **VTOR はリンカの `g_pfnVectors` から取る**（アドレスをハードコードしない）。
   - SysTick reload は継承した `SystemCoreClock`（550 MHz）から計算するが、**その過程で RCC は
     触らない**。

3. **オプションバイト / RDP / DBGMCU / SWD 端子（PA13/PA14）は触らない。**

4. **ビルドは `_ref/` を読まない。** `_ref/` は git 管理外の資料置き場。CMake / スクリプトが
   参照した瞬間、クローンしただけでは configure できないリポジトリになる（#58）。
   C コード中の `_ref/...` 言及は出典コメントのみで、これは依存ではない。

## メモリ配置（`ldscript/STM32H725AEIx_IROM.ld`）

内蔵 Flash 512 KB = 1 バンク・128 KB セクタ × 4（RM0468 Table 15）。

| 領域 | アドレス | 用途 |
|---|---|---|
| Flash セクタ0 | `0x08000000` 128 KB | **DFU ブートローダ専用（不可侵）** |
| Flash セクタ1-3 | `0x08020000` 384 KB | **app 本体（ここから実行）** |
| AXI-SRAM (D1) | `0x24000000` 320 KB | `.data`/`.bss`/`.axi_dma`/heap。上端は `__ram_end` |
| DTCM | `0x20000000` 128 KB | `.log_noinit`/`.nx_pool`/`.dtcm_bench`/`.dtcm_bss`（全スレッドスタック）。最上端が MSP = `_estack = 0x20020000` |
| ITCM | `0x00000000` 64 KB | `.itcm`（ISR コード） |
| 外部 OCTOSPI2 | `0x70000000` | **どこからも mmap されない**。`app/mpu.c` が 256 MB を no-access + XN で封鎖 |

**RAM 配置ポリシー（#46）** — 新しいバッファやスタックを足す変更は必ずこの表で行き先を決める:

- **AXI-SRAM** = バスマスタから見える必要があるものだけ（**DMA が届く唯一の RAM ＝希少資源**）
- **DTCM** = CPU 専用のホットなもの
- **ITCM** = ISR コード

🔴 **DMA1/DMA2 と SDMMC1 IDMA は TCM に届かない**（RM0468 §2.1.2/§2.1.5/§2.1.6）。
**DTCM に DMA バッファを置くと fault せず無言で「何も転送されない」。**

🔴 **`_estack` は「スタック起点」と「DTCM 終端」を兼ねている**。動かすと `_sbrk` が壊れる。
heap の天井は `__ram_end`。

🔴 **リンカスクリプトの `ASSERT` は LTO 下で空振りする**（リンクは成功したまま配置だけ剥がれる）。
配置の最終ガードは POST_BUILD の `cmake/check_itcm_residency.py` /
`cmake/check_dtcm_residency.py`（後者は「スタックが AXI に戻った」と「DMA バッファが DTCM に
紛れた」の**両方向**を検査する）。

🔴 **内蔵 Flash の書換え耐久は ~10k サイクル**。自動ループで焼き直さない。

## ThreadX 統合

- **SysTick > PendSV**（優先度）。同一だと idle 時 PendSV スピンを tick が割り込めず tick 停止 →
  スリープ中スレッドが起床しないデッドロック。PendSV は最低優先度。
- ThreadX が自前で `PendSV_Handler` を供給する（`stm32h7xx_it.c` のものと競合させない）。
- クリティカルセクションは **PRIMASK ベース**（`TX_PORT_USE_BASEPRI` 未定義）。USB(OTG_HS) ISR が
  `tx_event_flags_set` を呼んでも ThreadX クリティカルセクションを preempt できない。
- shell は静的割当（ヒープ非使用）。スタックサイズ・優先度は `shell/include/cli_config.h`。
- `__disable_irq` 下の `tx_application_define` で `HAL_GetTick` 依存の init を呼ばない（SysTick 凍結）。

## ペリフェラル

- **コンソール = USB CDC**: 単一 USB = **USB1_OTG_HS を FS（内蔵 PHY）動作**。CMSIS に
  `USB2_OTG_FS` / `OTG_FS_IRQn` は無く、TinyUSB(dwc2) は rhport0 を OTG_HS base +
  `OTG_HS_IRQHandler` にエイリアス（`tud_int_handler(0)`）。GPIO = PA11/PA12 `GPIO_AF10_OTG1_FS`。
  USB クロックは PLL3Q 48 MHz。app = **`0483:5740`**（ST 汎用 VCP）→ `/dev/ttyACM0`、
  boot = `0483:df11`（DFU）。
- **LED0（赤）= PC13 / LED1（黄）= PF0 / USER ボタン = PF1**（active-low、保持リセットで DFU）。
- I-Cache / D-Cache 両方 ON。`.axi_dma` は両端 32B align。
- 外部 NOR（OCTOSPI2 / W25Q128）は indirect ドライバ経由のみ。**mmap は使わない**
  （無効中の投機読みは RM0468 §25.4.16 で slave error、未応答 mmap read は AXI 無期限ストール）。

## レイヤリング

一方向依存を守る: **HAL/CMSIS/ThreadX ← `port/` ← `shell/` ← app（`src/`）**。
上位が下位へ潜り込むグルーは `port/` 側に置く。`boot/` は独立（app とソースを共有しない）。

## upstream submodule は read-only

`lib/` 配下の upstream ミラー（stm32h7xx_hal_driver, cmsis_device_h7, cmsis-core, tinyusb,
threadx ほか）は編集しない。必要な調整は `port/` 側のグルーで吸収する。

## レビュー時の作法

- **「コンパイルが通る」は根拠にならない。** レジスタ/能力の主張は RM0468 の節番号で、配線の
  主張は schematic で裏を取る。裏が取れない推測は推測として明示する。
- 存在しないファイル・行・レジスタ・実機挙動を作らない。実機は 1 枚しかないので、
  「焼いて試せばわかる」は安いコストではない。
- 指摘には影響（何がどう壊れるか）と具体的な修正案を付ける。

## ビルド / フラッシュ

```bash
cmake -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-toolchain.cmake
cmake --build build                      # build/{boot,shell,blink}.{elf,bin,hex}
dfu-util -d 0483:df11 -a 0 -D build/<app>.bin   # PF1 保持リセットで DFU モードに入ってから
```

app の書込先は内蔵 `0x08020000`。ST-Link 書込は boot をセクタ0 に焼く用だけ（通常やらない）。
