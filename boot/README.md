# Wio Lite AI — スタンドアロン USB DFU ブートローダ

STM32H725AEI6 用の **TinyUSB標準DFUブートローダ**。内蔵フラッシュ `0x08000000`
（セクタ0, 128KB）に常駐し、`dfu-util` で **内蔵フラッシュのappパーティション
（`0x08020000`, セクタ1-3 = 384KB）** にアプリを焼く。ソースは `boot/`、成果物は
`build/boot.{elf,bin,hex}`。

> **2026-08-01 更新 (issue #25)**: app の実行元を外部OCTOSPI2 XIP (`0x70000000`,
> 54 MB/s) から**内蔵フラッシュ (`0x08020000`, 905 MB/s)** へ移した。これに伴い
> ブートローダから **OCTOSPI2 を完全撤去**（`octospi.c` 削除、mmap立ち上げ無し、
> 外部appへのjump無し）。ブートローダは「内蔵フラッシュ用DFU + 内蔵appへのjump」
> だけになった。DFU の alt は 1 つのまま（alt 0 = 内蔵）なので
> `dfu-util -a 0 -D app.bin` のコマンドラインは不変、書込先だけが変わる。
> 以下「ステータス」節の記述は移行前の実機検証ログ（＝当時の事実）。

## ステータス: 実機で全経路動作確認済 (2026-07-12)

> **2026-07-12 更新**: TinyUSB を tinyuf2 内包版 → **直 submodule `hathach/tinyusb`
> 0.21.0** に変更（0.21.0 は `usbd_control.c` を `usbd.c` に統合）。boot.bin
> 29496→30332B。良品 Discovery ST-Link / mode=UR で `0x08000000` へ再書込・Download
> verified・読み戻しバイト一致・RDP 0xAA（非文鎮）。**boot flow / DFU 列挙 / DFU E2E の
> 全経路を実機で再確認済**（tinyusb 0.21.0 で無回帰）。

board #2 の内蔵 `0x08000000` に焼いて全経路を実機検証:

- 書込後も SWD/UR 健在・RDP 0xAA（文鎮化せず）。読み戻しバイト一致。
- **boot flow**（当時）: reset → clock/OCTOSPI2 mmap → app検証 → jump。jump先アプリの
  CDCダンプが元TinyUF2のクロックとほぼ完全一致（差はPLLCFGRのPLL3VCOSEL 1bit
  のみ＝MEDIUM vs WIDE、240MHz VCOで両範囲有効・無害）。
- **DFUモード**（PF1保持リセット）: `dfu-util -l` で
  `name="Wio Lite AI app @0x70000000"` を列挙（OCTOSPI2失敗時は
  `"OCTOSPI2 FAIL id=XXXXXX"`）。DFUモード中は**赤LED (PC13) 点灯**。
- **E2E**: `dfu-util -D blink.bin` → 0x70000000 へ書込 → manifest → 自動reboot
  → blink 起動（LED点滅を実機目視）。焼いたバイトが正しい証明。

## どう動くか

リセット後、`main()` は自前でクロックツリー（`clock.c`）を立ち上げ、次を判定する:

```
PF1 保持?      --yes--> DFU
内蔵 app 有効? --yes--> jump 0x08020000
                 no  --> DFU
```

- **DFUモード**: 複合 DFU+CDC デバイスとして列挙。ダウンロードは app パーティ
  ション（`0x08020000`, セクタ1-3）へ `iflash.c` 経由で書込。manifest後に新app
  へreboot。DFUモード中は赤LED点灯。
- **jump**: VTOR+MSP設定 → reset vector へ分岐。jump前に SysTick を停止
  （blink等は SysTick をデフォルト無限ループハンドラのまま残すため、stray tick
  で app がハングするのを防ぐ）。

**app が「有効」の条件**: vector[0] (MSP) がオンチップRAM、vector[1] (Reset) が
`0x08020000..0x0807FFFF` 内で thumb ビット付き。未書込（消去済み）フラッシュを
読んでも**全ビット1が返り ECC エラーにならない**（RM0468 §4.3.10）ので、app を
一度も焼いていない基板でも安全に判定できる。古い外部XIP版 `.bin`（Reset が
`0x700xxxxx`）もここで弾かれる。

DFUモードは安全なフォールバック: erased/invalid app は必ずここに来るので、
基板は常に再ロードできる。**option byte / RDP / DBGMCU / SWD端子は一切触らない**
ので、悪いconfigでも SWD で再書込可能。

### 内蔵フラッシュ書込の設計（issue #25）

| | |
|---|---|
| セクタ構成 | 1バンク・128KB × 4（RM0468 Table 15）。セクタ0=boot、1-3=app |
| セクタ0の保護 | `iflash_erase_sector()` が **1..3 以外を HAL 呼び出し前に拒否**。書込オフセットは `0x08020000` 相対なので、どんな引数でもセクタ0に到達できない。HAL の `IS_FLASH_SECTOR` は `FLASH_SECTOR_TOTAL=8`（1MB品前提）まで通してしまうので自前で縛る |
| デバイス判定 | `FLASHSIZE_BASE` が 512KB を返す場合のみ内蔵経路を有効化。違えば DFU download を拒否し jump もしない |
| 書込粒度 | 32B (256bit) flash word + 10bit ECC。非virgin word の上書き不可（RM0468 §4.3.9）→ 消去 → 32B整列書込、端数は 0xFF パディング |
| **vector-last commit** | 先頭32B（MSP/Reset）は最後まで書かない。転送が中断すると MSP は消去済み `0xFFFFFFFF` のままなので、次の起動は必ず DFU に入る。**外部フォールバックが無い今、これが「中断した転送でハングしない」唯一の担保** |
| セッション検査 | `block 0` で始まった連続転送でなければ commit しない（`off == next_off` を強制）。abort/エラー時は state 破棄 |
| 同一バンク書込 | boot自身がセクタ0から実行中にセクタ1-3を消去する。read は queue に積まれ完了後に serve されるので命令フェッチは **stall して再開**（RM0468 §4.3.8, ST公式 H723 例と同じ）。**消去中は割込みも含めて何も走らない** |
| DFU poll timeout | 消去を伴うブロックだけ 2500ms、通常は 10ms。board #2 実測: 128KB消去 = **888ms**、1KB書込 = 2.685ms |

## ビルド

```
cmake -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-toolchain.cmake
cmake --build build --target boot     # -> build/boot.{elf,bin,hex}
```

## ブートローダを焼く（ST-Link mode=UR）— brick本番、慎重に

⚠ **必ず良品 Discovery ST-Link（下記）で mode=UR。書込前に安全ゲートを緑に。**

```bash
CLI=/home/ouwa/work/STM32CubeProgrammer/bin/STM32_Programmer_CLI

# 1. 良品プローブ確認（SN 51FF.., FW V2J46S0 のはず）
"$CLI" --list

# 2. 安全ゲート: UR接続 + Device ID 0x483 + RDP 0xAA を確認（read-only）
"$CLI" -c port=SWD mode=UR reset=HWrst -ob displ

# 3. 現状バックアップ（接続安定性テスト兼）
"$CLI" -c port=SWD mode=UR reset=HWrst -r 0x08000000 0x80000 backup.bin

# 4. 書込 + verify（不可逆）
"$CLI" -c port=SWD mode=UR reset=HWrst -w build/boot.bin 0x08000000 -v

# 5. 書込直後に UR 再確認（文鎮化していないか）
"$CLI" -c port=SWD mode=UR reset=HWrst -ob displ    # RDP 0xAA のまま = OK
```

## アプリを DFU で焼く

1. **PF1 (USERボタン) を保持したままリセット** → ブートローダが DFUモードに留まる
   （赤LED点灯 / `dfu-util -l` に `name="Wio Lite AI app @0x08020000"`）。
   - SWD経由でリセットするなら PF1 保持中に
     `STM32_Programmer_CLI -c port=SWD mode=UR --start`。
2. `dfu-util -d 0483:df11 -a 0 -D <app>.bin`
3. manifest 後に自動 reboot → 新app起動。
4. 読み戻し検証（任意）: `dfu-util -d 0483:df11 -a 0 -U readback.bin`。

VID/PID は **0483:DF11**（STの標準DFU ID; 本基板はSTM32なので妥当。dfu-util /
CubeProgrammer が認識）。descriptor は manifestation-**intolerant**（自分でreboot
するため）。

## 安全 / 文鎮化 / 復旧

- **board #1 = 恒久文鎮化**（RDP2/debug恒久無効。`0x08000000` に不良ブートを
  焼いて死亡。復旧不能）。
- **board #2 = 開発ターゲット**（現在: boot@0x08000000 セクタ0, app@0x08020000 セクタ1-3）。
- 文鎮化の鉄則: 内蔵ブートで **PA13/14(SWD)不変更・DBGMCU非改変・低電力なし・
  オプションバイト/RDP絶対触らない**。焼く前に objdump 監査:
  ```bash
  TC=tools/arm-gnu-toolchain-*/bin
  # 1. option byte / DBGMCU に触れる定数が無いこと（何も出なければ OK）
  $TC/arm-none-eabi-objdump -d build/boot.elf | \
    grep -iE '5c001000|52002008|52002018|5200201c|52002020'
  # 2. option byte 系 API がリンクされていないこと（何も出なければ OK）
  $TC/arm-none-eabi-nm -C build/boot.elf | grep -iE 'OB_|OPTCR|OBProgram|MassErase'
  # 3. フラッシュ消去/書込の呼び出し元が iflash 経由だけであること
  $TC/arm-none-eabi-objdump -d build/boot.elf | \
    awk '/^[0-9a-f]+ <.*>:/{fn=$2} /bl.*<(HAL_FLASHEx_Erase|HAL_FLASH_Program)>/{print fn, $NF}' | sort -u
  #   期待: <iflash_erase_sector>: <HAL_FLASHEx_Erase> / <iflash_program>: <HAL_FLASH_Program>
  # 4. セクタ範囲チェックがコードに残っていること（issue #25）
  $TC/arm-none-eabi-objdump -d build/boot.elf --start-address=<iflash_erase_sector>
  #   期待: `cmp.w r2, #512`（デバイス容量判定）と sector-1 <= 2 の符号なし範囲判定
  ```
  （唯一の AIRCR 書込は `NVIC_SystemReset` の `0x05FA0004`。**issue #25 以降、
  boot は内蔵フラッシュを消去/書込する**が、経路は `iflash.c` の 1 本だけで、
  セクタ0 には構造的に到達できない。option byte 解錠 `HAL_FLASH_OB_Unlock`
  (`FLASH_OPTKEYR`) と `OPTCR` は一切使わない。）
- **TinyUF2復元**: バックアップ `_ref/wio_flash_backup_20260710_225512.bin`
  (512KB, md5 615ac2df..) を `0x08000000` へ書けば元に戻る。

## デバッガ

- **良品 Discovery ST-Link**（SN `51FF72064987505349271187`, FW `V2J46S0`）:
  CubeProgrammer `mode=UR`（hard connect-under-reset）が実証済。**焼き/復旧は
  これを使う**。CLI: `/home/ouwa/work/STM32CubeProgrammer/bin/STM32_Programmer_CLI`。
- V2 clone（`0483:3748`, FW `V2J17S4`）: 通常SWDは動くが **UR不可**。焼きには
  使わない。
- **SWD限界**: H7 の PLL 再設定で SWD 接続が落ち、app実行に追従できない。よって
  app内デバッグは **USB CDC の printf**（DFUモードのバナー/[tick]）が主力。

---

# ハードウェア・リファレンス（元 phase2_config_dump.md）

TinyUF2 が app へ jump する直前の実測レジスタ値。スタンドアロン init はこれを
再現している（`clock.c` / `octospi.c`）。値は hex。

## 実測ダンプ（app-first firmware の CDC 出力から）

```
--- RCC ---
CR=3F03C025 CFGR=0000001B PLLCKSELR=00519022 PLLCFGR=01FF093D
PLL1DIVR=0104002B PLL2DIVR=00010309 PLL3DIVR=0104022F
D1CFGR=00000048 D2CFGR=00000440 D3CFGR=00000040
D1CCIPR=00000020 D2CCIP2R=00200000
--- PWR --- CR1=F000C000 CR3=05010044 D3CR=00002000   --- FLASH ACR=00000033
--- OCTOSPI2 --- CR=30400381 DCR1=00170008 DCR2=00000002
  CCR=03032301 TCR=00000004 IR=000000EB ABR=00000000
--- OCTOSPIM --- CR=0 P1CR=03010111 P2CR=07050333   (= reset values)
--- GPIO MODER/OTYPER/OSPEEDR/PUPDR/AFRL/AFRH ---
PF FFEAAEF3 00000000 003FF300 00000004 AA090000 000009AA
PG FEAFEFFA 00000000 03F0300F 00000000 0A000099 00039300
```

## クロックツリー（HSE 25 MHz 水晶 → sysclk = PLL1）

- 供給/電圧: SMPS 直結供給（`PWR_DIRECT_SMPS_SUPPLY`）+ VOS0。FLASH ACR=0x33
  (latency 3 + WRHIGHFREQ、550MHz用、いずれもリセット値と整合)。
- PLLCKSELR=00519022: PLLSRC=HSE, DIVM1=2, DIVM2=25, DIVM3=5。
- **PLL1**: M2 (12.5MHz) N44 → VCO 550; P/1 → **sysclk 550 (CPU)**; Q110; R275。
- **PLL2**: M25 (1MHz) N266; R/1 → **266 → OCTOSPI2 kernel**（DCR2で÷3 ≈88.7MHz）。
- **PLL3**: M5 (5MHz) N48 → VCO 240; Q/5 → **48MHz → USB**。
- D1CFGR=0x48: HPRE=/2 (AXI/AHB 275), D1CPRE=/1, D1PPRE=/2。
- D1CCIPR=0x20: OCTOSPISEL → PLL2R。D2CCIP2R=0x200000: USBSEL → PLL3Q。

## OCTOSPI2 メモリマップ設定（W25Q128 Quad I/O）

- DCR1=00170008 (DEVSIZE 16MB), DCR2=2 (prescaler /3)。
- Read: IR=0xEB (Fast Read Quad I/O), CCR=03032301 (instr 1-line / addr 4-line
  24bit / mode byte 4-line ABR=0 / data 4-line, SIOO=0), TCR=4 dummy。
- CR=30400381 (EN, FMODE=11 mmap, FSEL, FTHRES=3, APMS)。
- W25Q の Quad-Enable (QE, SR2 bit1) を確認/設定してから quad read。

## OCTOSPI2 フラッシュ端子（schematic sheet6, 2026-07-12 確認）

**W25Q128 アプリフラッシュ = "QSPI2_*" ネット = OCTOSPIM Port 2**:

| 信号 | ピン | AF |
|------|------|----|
| CLK  | PF4  | AF9 |
| NCS  | PG12 | AF3 |
| IO4  | PG0  | AF9 |
| IO5  | PG1  | AF9 |
| IO6  | PG10 | AF3 |
| IO7  | PG11 | AF9 |

（quadデータは Port2 の上位ニブル IO[7:4] に乗る。P2CR リセット値 0x07050333 が
対応。実測AFRと一致: PF4=9, PG0/PG1=9, PG10=3, PG11=9, PG12=3。CSはR90 10K
プルアップ。）

**"OSPI1_*" ネットは PSRAM (OCTOSPI1/Port1) = PF6-PF10, PG6 等**。ネット名は
peripheral番号と一致しないので注意（旧資料は両者逆の誤記だった）。**コードは
GPIOバンク F/G を丸ごと再現するので両端子群をカバー**しており、この誤記の影響は
無い（per-pin最適化する場合は上表が正）。

## OCTOSPIM ルーティング

再dump で **CR=0 (MUXEN=0), P1CR=0x03010111, P2CR=0x07050333 = リセット値**
(RM0468 26.5.2)。→ デフォルト（OCTOSPI1→Port1, OCTOSPI2→Port2, mux無し）。
**OCTOSPIM は触らない**。

## 有効化するクロック（実測）

- AHB4ENR=0xFF → GPIOA..GPIOH。AHB1ENR bit25 → USB1_OTG_HS。
- AHB3ENR: OSPI1EN(14) + OSPI2EN(19) + IOMNGREN(21)。+ PWR/SYSCFG。

---

# 開発ノート（非自明・忘れるな）

## H725 の USB（要点）

- 単一USB = **USB1_OTG_HS** を FS（内蔵PHY）動作。CMSISに `USB2_OTG_FS` /
  `OTG_FS_IRQn` は**無い**。TinyUSB(dwc2) は `USB2_OTG_FS` 未定義時に rhport0 を
  **OTG_HS base + OTG_HS_IRQn** へエイリアス。∴ IRQ = `OTG_HS_IRQHandler` →
  `tud_int_handler(0)`。GPIO = PA11/PA12 `GPIO_AF10_OTG1_FS`。
- **FS強制が肝**: OTG_HSコアは GHWCFG2 で HS PHY 有りと申告するが基板は FS 内蔵
  PHY のみ。`CFG_TUD_MAX_SPEED=OPT_MODE_FULL_SPEED` で `phy_fs_init()` 経路に。
- USBクロックは PLL3Q 48MHz（`clock.c`）。VBUS検出は
  `HAL_PWREx_EnableUSBVoltageDetector()` のみ（USBREGEN不要=VDD33USBは外部給電）。
- 将来クロックが継承できない構成向けの保険として HSI48+CRS（`stm32h7xx_ll_crs.c`
  有り、hal_crs 無し）で USB を PLL 非依存にできる。現状は不要。

## OCTOSPI2 書込

- 内蔵実行なので RAM-exec 不要。mmap を abort → indirect で W25Q コマンド
  （WREN 06 / SE 20 / PP 02 / RDSR 05,35 / WRSR2 31 / JEDEC 9F、24bit addr）→
  mmap 復帰。caches off が前提（register/mmap アクセスのコヒーレンシ）。
- DFU download_cb は同期書込（callback内でブロッキング）。4KB境界毎に
  erase_sector → program。get_timeout DNBUSY=60ms が host のポーリングを律速。

## manifest 後の自動reboot

- `tud_dfu_manifest_cb` → `boot_request_reboot()` → main loop が **1500ms 後**
  `NVIC_SystemReset`。
- descriptor は manifestation-**intolerant**（自分でresetするため）。
- **遅延が1000ms超なのが肝**: dfu-util は dfuMANIFEST を見ると ~1000ms sleep して
  から GET_STATUS を読み直す。300ms で reboot するとその読み直しが消えたデバイスに
  当たり `LIBUSB_ERROR_NO_DEVICE` 警告になっていた。1500ms 待てば読み直しが
  dfuMANIFEST-WAIT-RESET を正常に読めて警告が消える。

## 参照コード

- TinyUF2: `_ref/tinyuf2/ports/stm32h7/boards.c`（jump/valid の参考）。
- 内蔵フラッシュ消去/書込: `_ref/STM32Cube_FW_H7_V1.13.0/Projects/NUCLEO-H723ZG/
  Examples/FLASH/FLASH_EraseProgram/`（同じ1バンク品で、flash常駐 HAL が同一
  バンクのセクタを消去している ST 公式例）。
- OCTOSPI2 の実装は issue #25 で削除。復活させるなら `git show <#25以前>:boot/octospi.c`
  （app 側で使うなら `app/psram.c` と同じ流儀で、issue #10 の範疇）。
- TinyUSB DFUクラス: `lib/tinyusb/src/class/dfu/dfu_device.{c,h}`（直 submodule, 0.21.0）。
