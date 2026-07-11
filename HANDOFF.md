# HANDOFF — Wio Lite AI カスタムDFUブートローダ開発

> 次セッションはこれを最初に読むこと。承認済みプラン: `/home/ouwa/.claude/plans/wio-lite-ai-st-link-reset-sw-2-federated-cosmos.md`

## 何を作っているか
Wio Lite AI (STM32H725AEI6) 用の **TinyUSB標準DFUブートローダ**。内蔵`0x08000000`のTinyUF2を置換し、`dfu-util -a 0 -D app.bin` で **外部OCTOSPI2フラッシュ(`0x70000000`)** にアプリを焼けるようにする。

## ⚠️ ハード/安全の現状（最重要・必読）
- **board #1 = 恒久文鎮化**（RDP2/debug恒久無効。ハードリセットでもSWD応答せず）。`0x08000000`に不良ブートを焼いて死亡。**復旧不能・期待しない**。
- **board #2 = 生きているWio**（TinyUF2@0x08000000、アプリ@0x70000000 XIP）。**開発ターゲットはこれ**。
- **文鎮化リスクは現実**：内蔵ブートがSWD(PA13/14)を殺す or オプションバイト/RDPを壊す = 復旧不能。

## デバッガ（重要）
- **V2 clone ST-Link**（`0483:3748`, FW `V2J17S4`）：通常SWDは動く(OpenOCD/st-flash)が **connect-under-reset不可**（旧FW・クローンで更新不可）。**Phase 1(アプリ先行, UR不要)はこれでOK**。
- **★STM32L-Discovery のオンボードST-Link**（SN `51FF72064987505349271187`, FW `V2J46S0`。**クローンではない**）：**CubeProgrammer `mode=UR`（ハードconnect-under-reset）がWioで動作実証済み(Device ID 0x483)**。→ **Phase 3の安全網はこれ**。
  - **⚠SWD限界（実測）**: connect-under-resetでブートローダ停止までは掴めるが、**ブートローダのH7 PLL再設定でSWD接続が落ち、app実行に追従できない**（attach-without-reset/hotplugも失敗）。∴ app実行中のライブデバッグは不可。**代替=USB CDCのprintf**(Phase 1.4で実装済)がapp内デバッグの主力。
- CubeProgrammer CLI: `/home/ouwa/work/STM32CubeProgrammer/bin/STM32_Programmer_CLI`
- B-U585 V3E: UR動くがSTDC14/TC2050ケーブルが無く外部配線できず（保留）。
- F4-Disco: 外部SWD配線が通らず（ボード固有問題）。使わない。

## 復旧手段（board #2用）
- 内蔵フラッシュの消去/書込/verify は実証済み(OpenOCD)。
- TinyUF2バックアップ: `_ref/wio_flash_backup_20260710_225512.bin`(512KB) → `0x08000000`へ書けば復元。
- Phase 3書込&復旧は **良品Discovery ST-Link + `STM32_Programmer_CLI -c port=SWD mode=UR`** を使う。

## 技術的要点（board #2）
- STM32H725AEI6 Cortex-M7。内蔵512KB@`0x08000000`。外部OCTOSPI2=W25Q128 16MB@`0x70000000`(アプリXIP)。OCTOSPI1=PSRAM@`0x90000000`(フラッシュではない)。
- クロック(TinyUF2が設定・継承する。**RCCをリセットするな**): HSE25→PLL1 **550MHz CPU**/275MHz AXI; **PLL2R 266→OCTOSPI2**(÷3≈88.7MHz); **PLL3Q 48MHz→USB(OTG FS)**。
- 起動受け渡し: `SCB->VTOR=0x70000000` → `MSP=*(0x70000000)` → `bx *(0x70000004)`（逆アセンブルで確認済）。
- LED: **PC13(赤)** /PF0(黄) NPN経由。USERボタン PF1。USB OTG FS = PA11/PA12。

## 進捗
- ✅ **Blinkプロジェクト完成&push済**（`github.com/owhinata/wio-lite-ai` main）。外部XIP`0x70000000`からLED(PC13)点滅を実機確認。gcc15.2.1自動DL/CMake/HAL・CMSISサブモジュール。
- ✅ **Phase 0(安全網)達成**：UR可能なデバッガ確保。内蔵消去/書込実証。
- 🔄 **Phase 1.1**：`lib/tinyuf2` submodule **追加済**（adafruit/tinyuf2 @ `9af754c`; 入れ子 `lib/tinyusb`@`2a364ca`[DFUクラス有], `lib/uf2`; **非recursive**でhw/mcu回避）。**boot/骨組み+CMake完成**：
  - `boot/main.c`＝スケルトン（PC13を**5Hz**でheartbeat点滅＝blinkの1Hzと一目で区別。blinkと同じhands-off-clocks方針・DWT遅延、`src/system_stm32h7xx.c`を共有）。
  - `CMakeLists.txt`に`boot`ターゲット追加（blink流用・同じXIP ldscript・bsp_iface・HAL全部）。`firmware_finalize`の`flash`を**`flash-<tgt>`にリネーム**（`flash-blink`/`flash-boot`）。project名も`wio-lite-ai`に。
  - **ビルド確認済**：`cmake --build build` で `build/boot.{elf,bin,hex,uf2}` 生成（`boot.uf2`=4KB, base=0x70000000, family=0x6DB66082）。`.isr_vector`@0x70000000、**vector[0]=MSP=0x24050000**・**vector[1]=Reset_Handler|1=0x70000419** 検証済（＝bootの受け渡し条件に合致）。
  - ✅ **実機ラン確認済**：board#2でboot.uf2 drop→**PC13が5Hzで点滅**を目視確認。**Phase 1.1完了**。
- ✅ **Phase 1.2達成（USB DFU列挙）**：boot firmwareにTinyUSB device+標準DFUクラスを載せ、`dfu-util -l`で列挙成功。
  - **実機確認**: `Found DFU: [cafe:4000] alt=0, name="OCTOSPI2 XIP @0x70000000"`、`bInterfaceClass 254/Sub 1`(DFU)、`bcdDFU 1.01`、`wTransferSize 1024`、`bmAttributes 5`(CAN_DOWNLOAD|MANIFEST_TOLERANT)、serial=UUID。列挙は電源投入~1秒で成功。
  - **H725 USB要点(非自明・重要)**: 本チップはUSB単一=**USB1_OTG_HS**をFS(内蔵PHY)動作。CMSISに`USB2_OTG_FS`/`OTG_FS_IRQn`は**無い**。TinyUSB(dwc2)は`USB2_OTG_FS`未定義時にrhport0を**OTG_HS base+OTG_HS_IRQn**へエイリアス。∴ IRQは**`OTG_HS_IRQHandler`→`tud_int_handler(0)`**。GPIO=PA11/PA12 **`GPIO_AF10_OTG1_FS`**。clk=`__HAL_RCC_USB1_OTG_HS_CLK_ENABLE()`+ULPI clk disable。
  - **FS強制が肝**: OTG_HSコアはGHWCFG2でHS PHY有りと申告するが基板はFS内蔵PHYのみ。`CFG_TUD_MAX_SPEED=OPT_MODE_FULL_SPEED`で`phy_fs_init()`経路に(でないとHS PHY起動を試み失敗)。ビルド定義 `-DCFG_TUSB_MCU=OPT_MCU_STM32H7`。
  - **USBクロックは継承でOK(HSI48+CRS不要だった)**: TinyUF2自身のUSBが動くクロックをそのまま継承(UF2 drop後は`NVIC_SystemReset`→bootのclock_init再実行→RCC deinitせずapp jump)。∴クリスタル由来48MHz(PLL3Q想定)で即列挙。※将来継承できない場合の保険はHSI48+CRS(`stm32h7xx_ll_crs.c`有り, hal_crsは無し)。
  - **NVIC**: TinyUSBが`tusb_init`内で有効化(`usbd.c` dcd_int_enable)。VBUS sense無効/B-valid override/FS PHY power-up/connectも`dcd_init`が実施。VBUS検出は`HAL_PWREx_EnableUSBVoltageDetector()`のみ(TinyUF2既知良と同じ)。
  - **追加ファイル**: `boot/{tusb_config.h, usb_descriptors.c, dfu_callbacks.c}` + main.cにUSB HW init/IRQ/SysTick。CMake: TinyUSB device一式(`tusb.c/tusb_fifo.c/usbd.c/usbd_control.c/dfu_device.c/dcd_dwc2.c/dwc2_common.c`)を**bootのみ**にコンパイル。**DFUコールバックは現状スタブ**(get_timeout/download/manifest= `tud_dfu_finish_flashing(OK)`)。
- ✅ **Phase 1.3達成（OCTOSPI2 RAM実行書込ドライバ）**：`boot/ospi_ram.c`。scratch`0x70400000`(flash offset 0x400000)で**消去→書込(512B/2ページ)→mmap読出**の往復を実機検証。`dfu-util -l`=`id=EF4018 st=OK`、`dfu-util -a0 -U`で**先頭512B=pattern(i^0xA5)完全一致・残り0xFF**をホスト側バイト照合済。JEDEC=**EF 40 18**(Winbond W25Q128)。
  - **★核心の落とし穴2つ(超重要・長時間デバッグの結論)**:
    1. **I/Dキャッシュを切る事が必須**(`main`冒頭で`SCB_DisableICache/DisableDCache`)。キャッシュONだと**RAM実行コードからのOCTOSPIレジスタ(0x5200A000, M7 AXIM経由)アクセスが非コヒーレント**＝読み0/書き無効(ドライバが黙って空振り→フローティング読みでゴミ)。さらに0x70000000窓へ投機読みでフォルト。切ると全て正常(CR=0x30400381読める/JEDEC=EF4018)。
    2. **VTORをRAMへ再配置**(`setup_ram_vectors`でVTを0x70000000からRAMへコピー、Hard/MM/Bus/Usageを自前RAMハンドラに)。mmap解除中のフォルト/投機は0x70000000のVT不読→エスカレ→**ロックアップ**。VTがRAMなら回避(caches offでも必要だった)。RAMフォルトハンドラ(`ram_fault_entry`/`ram_fault_c`)はpc/cfsr/bfar捕捉→mmap復旧→`app_report_and_loop`へリダイレクトし`FLT pc=.. cf=..`をUSB報告(デバッグ安全網として常設)。
  - **ドライバ設計**: `ospi_ram.c`全RAM実行(`.RamFunc`)・IRQ off・XIP/rodata/libc非参照。`ospi_flash_snapshot()`でmmap読設定(CR/CCR/TCR/IR; 実測CCR=0x03032301=0xEB quad I/O・SIOO=0・ABR=0・4dummy)をXIPで退避→操作後に書き戻してmmap復帰(RM0468 25.4.18: mmap read後は barrier→abort)。W25Q128 1-lineコマンド: WREN`06`/RDSR1`05`(WIP=b0)/SE`20`(4K)/PP`02`(≤256B/page)/JEDEC`9F`。abort=CR.ABORT→BUSYクリア待ち。
  - **SWD不可の教訓**: クローンST-LinkはH7のPLL再設定でSWDが落ちる。connect-under-resetでブートローダ停止までは掴めるがapp実行に追従不可。故に**RAMフォルトハンドラ+DFU upload/文字列でのセルフ報告**が実質唯一のデバッグ手段だった。

- ✅ **Phase 1.4達成（DFU→OCTOSPI2実書込, エンドツーエンド）**：`dfu-util -a0 -D file`→`tud_dfu_download_cb`が**RAM実行ドライバでscratch(0x70400000)へ実書込**→`dfu-util -a0 -U`で読み戻し**バイト完全一致**を実機検証(5000Bテスト, 4KB境界跨ぎ=複数セクタ消去もOK)。
  - `boot/dfu_callbacks.c`: `download_cb`はoff=`block_num*1024`、4KB境界毎に`ospi_ram_erase_sector`→`ospi_ram_program`。upload=scratch 64KB窓読出。`get_timeout`=DNBUSY 60ms。**同期書込**(callback内でブロッキング, IRQ off中USB停止~数十ms=dfu-util許容)。
  - `boot/ospi_ram.c`に`ospi_ram_erase_sector`/`ospi_ram_program`(各々IRQ off+abort+op+mmap復帰の自己完結ラッパ)追加。
  - **app-first制約**: 本firmwareは0x70000000でXIP実行中なので**DL先はscratch 0x400000**(app base 0x70000000に書くと自己破壊)。**Phase 2で内蔵実行に移れば app base 0x70000000 が正当な書込先になる**。
- ✅ **USB CDCコンソール(printf)追加（DFU+CDC複合）**：`tusb_config.h`で`CFG_TUD_CDC 1`、`usb_descriptors.c`をIAD付き複合(device class=MISC/COMMON/IAD, iface CDC0/CDC1/DFU2, EP 0x81 notif・0x02/0x82 data)、CMakeに`class/cdc/cdc_device.c`追加。main.cに`_write`(→`tud_cdc_write`)/`_sbrk`(linker `end`)実装+`setvbuf(_IONBF)`で**printfがCDCに出力**。`console_task`で接続時バナー(JEDEC/self-test)+2秒heartbeat。実機: `/dev/ttyACM0`にbanner+tick、**DFUと同時併用OK**(download/uploadバイト一致)。→ **Phase 2のデバッグはこのprintfが主力**(SWDが不安定なため)。

## 次の手順（Phase 2〜: スタンドアロン化）
1. ✅ **Phase 1.1〜1.4 完了**（上記進捗）。**未コミット**（DFU一式+ospi_ram+caches/VTOR+DFU書込）。**大きな節目、コミット推奨**。
2. **Phase 2（重要・大仕事）**: 実ブートローダ@`0x08000000`化。**app-first期はクロック/OCTOSPI2 mmap設定をTinyUF2から継承していたが、置換後は全部自前でやる必要**：
   - ✅ **kickoff完了: TinyUF2の全設定をCDC printfでダンプ&解読済** → `boot/phase2_config_dump.md`（RCC/PLL/OCTOSPI2/GPIO実測値+クロック解読+フラッシュ端子）。スタンドアロンinitは**この実測レジスタ値を再現**すればよい(推測不要)＝de-risk済。
   - **クロック(実測)**: HSE25→PLL1(M2/N44/P1→**CPU550**,R2→AXI275)/PLL2(M25/N266/R1→**OCTOSPI2 266**,DCR2で÷3)/PLL3(M5/N48/Q5→**USB48**)。sysclk=PLL1。D1CCIPR=OCTOSPISEL=PLL2R, D2CCIP2R=USBSEL=PLL3Q。
   - **OCTOSPI2端子(schematic確認)**: CLK=PF10(AF9)/CS=PG6(AF10)/IO0-3=PF8/PF9/PF7/PF6(AF10), speed=VERY_HIGH。OCTOSPIM Port1経由。mmap=0xEB Quad I/O(CCR=0x03032301,TCR=4dummy,DCR1=16MB)。W25Q128のQEビット設定要。
   - ✅ **OCTOSPIM解決**: IOMNGRクロックON後に再dump → **CR=0(MUXEN=0)/P1CR=0x03010111/P2CR=0x07050333＝リセット値そのまま**(RM0468 26.5.2)。∴デフォルト(OCTOSPI1→Port1, OCTOSPI2→Port2)。**app flash=OCTOSPI2→Port2、OCTOSPIMは触らなくてよい**。(最初のdump=0はfluke; AHB3ENR bit21で元々ON。schematicの"OSPI1"ネット名≠peripheral番号)。
   - **有効化すべきクロック(実測)**: AHB4ENR=0xFF(GPIOA-H)、AHB3ENR=OSPI1EN(14)+OSPI2EN(19)+IOMNGREN(21)、AHB1ENR bit25(USB1_OTG_HS)。+PWR/SYSCFG。
   - **init方針=再現(de-risk)**: クロックはHALで(PLL実測値)、GPIOはF/GバンクのMODER/AFR等を実測値で再現(Port2フラッシュ端子込み)、OCTOSPIMはreset放置、OCTOSPI2はDCR+0xEB quad+CR.FMODE=11を再現、W25Q QEビット確認。DFU書込先をscratch→app base(0x70000000 offset0)へ。詳細: `boot/phase2_config_dump.md`。
   - ✅ **milestone 1完了(commit d7b128a)**: `bootrom/`新設。`ldscript/STM32H725AEIx_ROM.ld`(内蔵0x08000000)、`bootrom/clock.c`(SystemClock_Config=実測PLL値再現)、`bootrom/main.c`(HAL_Init+clock+PC13点滅)、stock CMSIS system(HSIリセット→config, VTOR=0x08000000)。CMake: per-firmware ldscript化+`rom_finalize`(.bin/.hex, UF2なし)+`bootrom`ターゲット。ビルド9KB@0x08000000、`.isr_vector`@0x08000000/MSP=0x24050000検証、**SWD安全監査clean**(option byte/DBGMCU/SWD端子書込なし)。**実行検証はPhase 3(0x08000000へ焼く)必須=未検証**。
   - **bootrom残milestone**: 2=OCTOSPI2 mmap init(Port2 GPIO+DCR/CCR 0xEB quad+W25Q QE)、3=USB DFU+CDC統合(boot/流用, DFU先をoffset0へ)、4=boot flow(DFUトリガ/app検証→0x70000000へjump)。各々untestable until Phase 3。
   - **クロック自前init**: HSE25→PLL1 550MHz CPU/275AXI・PLL2R→OCTOSPI2・PLL3Q→USB48MHz。ldscriptを`0x08000000`(内蔵512KB)へ。`system_stm32h7xx.c`の hands-off方針は破棄しフルSystemClock_Config。
   - **OCTOSPI2 mmap自前init**: W25Q128を0xEB Quad I/Oでmmap化(参照: `_ref/.../STM32H735G-DK/stm32h735g_discovery_ospi.c`のOSPI init, MX25→W25Q QE設定/読みコマンド移植)。
   - **DFU書込先を app base 0x70000000(offset 0)** に変更(scratch廃止)。
   - **起動フロー**: DFUトリガ(ボタン/マジック)判定→無ければ`board_app_valid`→`SCB->VTOR=0x70000000`→app jump。
   - **SWD安全監査**: PA13/14不変更・DBGMCU非改変・低電力なし・**オプションバイト/RDP絶対触らない**。焼く前にobjdump監査。
3. **Phase 3**: `boot.bin`を**良品Discovery ST-Link+CubeProgrammer mode=UR**で`0x08000000`へ。直後にUR再確認。TinyUF2バックアップ(`_ref/wio_flash_backup_20260710_225512.bin`)で復元可。
3. **Phase 2**: 実ブートローダ@`0x08000000`。**SWD安全**(PA13/14不変更・DBGMCU非改変・低電力なし・**オプションバイト/RDP絶対触らない**)。objdump監査してから焼く。
4. **Phase 3**: `boot.bin`を**良品Discovery ST-Link+CubeProgrammer mode=UR**で`0x08000000`へ。直後にUR再確認。

## 参照コード
- TinyUF2: `_ref/tinyuf2/ports/stm32h7/{board_api.h, boards.c:146-255(jump/valid), board_flash.c}`（QUADSPI@0x90000000 → **OCTOSPI2@0x70000000へ移植**、`HAL_QSPI`→`HAL_OSPI`）。
- OCTOSPI2ドライバ雛形: `_ref/STM32Cube_FW_H7_V1.13.0/Drivers/BSP/STM32H735G-DK/stm32h735g_discovery_ospi.c`（Instance→OCTOSPI2, MX25→W25Q, base→0x70000000）。
- DFU参考: `_ref/.../Projects/STM32H735G-DK/Applications/USB_Device/DFU_Standalone`（`usbd_dfu_flash.c`メディア, `usb_device.c`のHSI48+CRS）。
- TinyUSB DFUクラス: `lib/tinyuf2/lib/tinyusb/src/class/dfu/dfu_device.{c,h}`。コールバック: `tud_dfu_download_cb(alt,block,data,len)` / `tud_dfu_manifest_cb(alt)` / `tud_dfu_get_timeout_cb(alt,state)`。
- 解析まとめ: `BACKUP_README.md`、memory(`wio-lite-ai-flash-map`等)。

## リポジトリ/ツール
- Repo: `/home/ouwa/work/wio-lite-ai`（git, remote `github.com/owhinata/wio-lite-ai` main）。blink一式コミット済。**DFU関連は未コミット**（lib/tinyuf2 submodule追加は`.gitmodules`変更あり・未コミット）。
- Toolchain: `tools/arm-gnu-toolchain-15.2.rel1`（cmakeが自動DL）。
- `_ref/` はgitignore済（回路図・Cube FW・バックアップ等の大容量）。
- テスト用: `build/blink.uf2`（DFU化アプリのテストにも使える生成物）。

## USBモード早見
- アプリ動作: `8050:2886` CDC → `/dev/ttyACM0`（115200, '.'出力）。
- TinyUF2: user-btn+RST で `2886:0040` USBストレージ(ラベル"Arduino", `/media/ouwa/Arduino`)。ここに.uf2をdrop。
