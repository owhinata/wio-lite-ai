# HANDOFF — Wio Lite AI カスタムDFUブートローダ開発

> 次セッションはこれを最初に読むこと。承認済みプラン: `/home/ouwa/.claude/plans/wio-lite-ai-st-link-reset-sw-2-federated-cosmos.md`

## 何を作っているか
Wio Lite AI (STM32H725AEI6) 用の **TinyUSB標準DFUブートローダ**。内蔵`0x08000000`のTinyUF2を置換し、`dfu-util -a 0 -D app.bin` で **外部OCTOSPI2フラッシュ(`0x70000000`)** にアプリを焼けるようにする。

## 🎉 ステータス: **Phase 3完了＝実機で全経路動作確認済(2026-07-12, commit 42a9b1f + 実機検証)**
**カスタムDFUブートローダ完成**。board #2の内蔵`0x08000000`に`bootrom`(スタンドアロン)を焼き、実機で全経路検証:
- ✅ 書込後もSWD/UR健在・RDP 0xAA(文鎮化せず)。読み戻しbootrom.bin完全一致。
- ✅ **boot flow**: リセット→bootrom clock/OCTOSPI2 mmap→app_valid→jump。appのCDCダンプがTinyUF2実測クロックとほぼ完全一致(PLLCFGRのPLL3VCOSEL 1bitのみ差=無害)。
- ✅ **DFUモード**(PF1保持リセット): `dfu-util -l`で`[cafe:4000] name="id=EF4018 ok"`列挙。
- ✅ **E2E**: `dfu-util -a0 -D blink.bin`(-d cafe:4000指定)→bootromが0x70000000へ書込→manifest→自動reboot→blink起動(**LED PC13 1Hz点滅を実機目視確認**)。焼いたバイトが正しい証明。
- 現在board #2常駐: 内蔵=`bootrom`, 0x70000000=`blink`。**再度USBアプリを焼くにはPF1保持でリセット→DFUモード→`dfu-util -d cafe:4000 -a0 -D <app>.bin`**。
- **既知の軽微点**: manifest後の自動reboot(300ms)で`dfu-util`が`LIBUSB_ERROR_NO_DEVICE`警告(DL自体は成功)。気になればMANIFEST_TOLERANTクリア or 遅延調整。dfu-util複数デバイス時は`-d cafe:4000`必須。

## ⚠️ ハード/安全の現状（最重要・必読）
- **board #1 = 恒久文鎮化**（RDP2/debug恒久無効。ハードリセットでもSWD応答せず）。`0x08000000`に不良ブートを焼いて死亡。**復旧不能・期待しない**。
- **board #2 = 生きているWio**（**現在: bootrom@0x08000000, blink@0x70000000 XIP**。元TinyUF2は消去済＝`_ref/wio_flash_backup_20260710_225512.bin`で復元可, md5 615ac2df..)。**開発ターゲットはこれ**。
- **文鎮化リスクは現実**：内蔵ブートがSWD(PA13/14)を殺す or オプションバイト/RDPを壊す = 復旧不能。（bootromは監査済＝安全実証。今後別イメージを焼く際も同じ監査を。）

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
   - **OCTOSPI2端子(schematic sheet6レビューで訂正済 2026-07-12)**: **W25Q128アプリフラッシュ="QSPI2_*"ネット=OCTOSPIM Port 2** → CLK=PF4(AF9)/NCS=PG12(AF3)/IO4-7=PG0,PG1(AF9),PG10(AF3),PG11(AF9)（quadデータはP2上位ニブルIO[7:4]、P2CRリセット値0x07050333が対応）。**"OSPI1_*"ネットはPSRAM(OCTOSPI1/Port1)=PF6-10/PG6等**（旧記載は両者逆=誤記。コードはF/Gバンク丸ごと再現で両方カバーしており無影響）。フラッシュCSはR90 10Kプルアップ。mmap=0xEB Quad I/O(CCR=0x03032301,TCR=4dummy,DCR1=16MB)。W25Q128のQEビット設定要。
   - ✅ **OCTOSPIM解決**: IOMNGRクロックON後に再dump → **CR=0(MUXEN=0)/P1CR=0x03010111/P2CR=0x07050333＝リセット値そのまま**(RM0468 26.5.2)。∴デフォルト(OCTOSPI1→Port1, OCTOSPI2→Port2)。**app flash=OCTOSPI2→Port2、OCTOSPIMは触らなくてよい**。(最初のdump=0はfluke; AHB3ENR bit21で元々ON。schematicの"OSPI1"ネット名≠peripheral番号)。
   - **有効化すべきクロック(実測)**: AHB4ENR=0xFF(GPIOA-H)、AHB3ENR=OSPI1EN(14)+OSPI2EN(19)+IOMNGREN(21)、AHB1ENR bit25(USB1_OTG_HS)。+PWR/SYSCFG。
   - **init方針=再現(de-risk)**: クロックはHALで(PLL実測値)、GPIOはF/GバンクのMODER/AFR等を実測値で再現(Port2フラッシュ端子込み)、OCTOSPIMはreset放置、OCTOSPI2はDCR+0xEB quad+CR.FMODE=11を再現、W25Q QEビット確認。DFU書込先をscratch→app base(0x70000000 offset0)へ。詳細: `boot/phase2_config_dump.md`。
   - ✅ **milestone 1完了(commit d7b128a)**: `bootrom/`新設。`ldscript/STM32H725AEIx_ROM.ld`(内蔵0x08000000)、`bootrom/clock.c`(SystemClock_Config=実測PLL値再現)、`bootrom/main.c`(HAL_Init+clock+PC13点滅)、stock CMSIS system(HSIリセット→config, VTOR=0x08000000)。CMake: per-firmware ldscript化+`rom_finalize`(.bin/.hex, UF2なし)+`bootrom`ターゲット。ビルド9KB@0x08000000、`.isr_vector`@0x08000000/MSP=0x24050000検証、**SWD安全監査clean**(option byte/DBGMCU/SWD端子書込なし)。**実行検証はPhase 3(0x08000000へ焼く)必須=未検証**。
   - ✅ **milestone 2完了(commit fce46eb)**: `bootrom/octospi.c`。内蔵実行なのでRAM-exec不要。**GPIO F/Gバンクを実測値で丸ごと再現**(Port2端子の個別特定を回避)+OCTOSPI2 DCR/CCR/IR再現+**W25Q QEビット確認/設定**(SR2 bit1)+JEDEC読みで検証→mmap有効化。main.cでmmap経由app MSP読み→**PC13で判定表示**(1Hz=clock+OCTOSPI2 OK / 10Hz=失敗)。SWD安全監査clean(GPIO F/G/Cのみ、option byte/DBGMCU/PA13-14書込なし)。**実行検証はPhase 3**。
   - ✅ **milestone 3完了(commit 48aa84f)**: USB DFU+CDC統合。`bootrom/{tusb_config.h,usb_descriptors.c,dfu_callbacks.c}`をboot/から流用+`bootrom/octospi.c`に**内蔵実行版のerase/program追加**(RAM-exec不要=abort→indirect→SE 0x20/PP 0x02→restore mmap。W25Q 1-lineコマンド。caches offで前提coherent)。`bootrom/main.c`書き直し(clock+octospi init→USB HW init(PA11/12 AF10, USB1_OTG_HS)→`OTG_HS_IRQHandler`→**`SysTick_Handler`定義=milestone1/2の潜在HAL_Delayハングbug修正**→tud_taskループ+CDCバナー/heartbeat)。**DFU alt0書込先をscratch→app base offset0(0x70000000)へ変更**(内蔵実行なので自己破壊しない)。CMake: bootromにTINYUSB_SOURCES+CFG_TUSB_MCU追加。ビルド29KB@0x08000000、vector table検証(MSP=0x24050000/Reset/SysTick/OTG_HS各ハンドラが自前関数)、**SWD安全監査clean**(FLASH->ACR latencyのみ、option byte/OPTKEYR/OPTCR/OPTSR/DBGMCU/PA13-14/低電力/内蔵flash書込なし)。**実行検証はPhase 3**。
   - ✅ **milestone 4完了(未コミット)**: boot flow。**起動判定**(main): OCTOSPI2 mmap up後、`dfu_button_held()`(**PF1=active-low, R69 10Kプルアップ+SW2でGND=schematic sheet8で極性確認済**。入力プルアップで読む。PF1はOCTOSPI端子でない)と`app_valid()`(mmap経由でapp vector読み: MSP∈RAM{0x24/0x20/0x30/0x38}・reset∈0x70窓+thumb bit=blank 0xFF/0x00を弾く)。**!ボタン && OSPI ok && app valid → `jump_to_app()`**(=`__disable_irq`→SysTick停止(CTRL/LOAD/VAL=0)+ICSR PENDST/PENDSV clr→`SCB->VTOR=0x70000000`→MSP/PSP=vec[0]→`__enable_irq`→**reset vectorをflashから読んでbx**(スタックspill回避))。それ以外はDFUモード(=erased/invalid appは必ずここ＝常に再ロード可能な安全fallback)。**DFU manifest後**: `bootrom_request_reboot()`→main loopが300ms後`NVIC_SystemReset`(dfu-util完了+USB flush待ち)→新appをboot。descriptorはMANIFEST_TOLERANT維持(milestone3のenumと不変)。ビルド29.5KB、jump逆アセンブル検証(VTOR←0x70000000のみ=+8, AIRCR書込はNVIC_SystemResetのみ=0x05FA0004)、**SWD安全監査clean**(GPIOFはPF1のみ, PA13-14/option/DBGMCU/内蔵flash非書込)。**実行検証はPhase 3**。
   - ✅ **Phase 3前の起動不可リスクレビュー完了(2026-07-12, PM0253+RM0468+schematic全突合)**。結論: **恒久文鎮ベクタ=0件**(option byte/RDP/DBGMCU/PA13-14/AIRCR系すべて非接触をobjdumpで再確認; AIRCR書込=NVIC_SystemResetの1箇所のみ)。**残リスクは全てハング型=SWD/UR復旧可**(HSE不良→while(1)等)。主な検証結果:
     - **電源**: schematic sheet4のDesign Note「MCU_SMPS MODE: Default」+VLXSMPS→L4→VDD_SMPS→VCAP直結 → `PWR_DIRECT_SMPS_SUPPLY`正当。CR3実測0x05010044のデコード=SMPSEN(2)+予約bit6(リセット値1)+SMPSEXTRDY(16,RO)+USB33DEN(24)+USB33RDY(26,RO)。**USBREGEN(25)=0=VDD33USBは外部給電** → bootromの「検出器のみ」で完全(レギュレータ有効化は不要と確定)。
     - **VOS0**: H72xではHAL `PWR_REGULATOR_VOLTAGE_SCALE0=(0U)`=VOS[15:14]=0b00。実測D3CR=0x2000(VOS=00,VOSRDY)と一致。
     - **FLASH_ACR**: リセット値0x0000_0037(RM0468)→HALがLATENCYのみ3へ→0x33=実測一致。WRHIGHFREQ=リセット値0b11のままで正(HALは触らないが偶然でなく整合)。
     - **BOOT0=R36 10Kプルダウン**(sheet5)→常に内蔵flashブート。**HSE=X1 25MHz水晶**+22pF ✓。PDR_ON=10Kプルアップ ✓。
     - **PM0253**: VTOR TBLOFF[31:9]アライン充足(0x70000000/0x08000000)。「VTOR書換前に新テーブルのfault/NMI/有効例外整備」→ jump時IRQ全停止+SysTick停止/pending clear+NMIソース(CSS)未使用で充足。ICSR PENDSTCLR=bit25/PENDSVCLR=bit27=0x0A000000 ✓。
     - **修正3件**: (1)`jump_to_app`をMSP切替+cpsie+bxの**単一asmブロック化**(レジスタオンリー; コンパイラのスタックspillを構造的に排除。逆アセンブルで msr MSP,r0/msr PSP,r0/cpsie/bx r4 連続を確認)。(2)`octospi.c`に**OSPI1EN追加**(実測AHB3ENR再現; F/G再現でAF化されるPSRAM Port1端子を無クロックIPにぶら下げない)。(3)**端子表の誤記訂正**(上記; phase2_config_dump.mdも修正)。
   - ✅ **bootrom全milestone(1-4)完了+レビュー済+Phase 3実機検証完了(2026-07-12)**。上部「🎉ステータス」参照。良品Discovery ST-Link(SN 51FF.., V2J46S0, mode=UR)で`build/bootrom.bin`を`0x08000000`へ書込→verify成功→UR再確認clean→boot flow/DFUモード/E2E DFU書込(blink)全て実機動作(LED 1Hz目視)。**milestone4のjump/ボタン(PF1 active-low)/manifest-reboot全て実機で動作確認済**。
   - **クロック自前init**: HSE25→PLL1 550MHz CPU/275AXI・PLL2R→OCTOSPI2・PLL3Q→USB48MHz。ldscriptを`0x08000000`(内蔵512KB)へ。`system_stm32h7xx.c`の hands-off方針は破棄しフルSystemClock_Config。
   - **OCTOSPI2 mmap自前init**: W25Q128を0xEB Quad I/Oでmmap化(参照: `_ref/.../STM32H735G-DK/stm32h735g_discovery_ospi.c`のOSPI init, MX25→W25Q QE設定/読みコマンド移植)。
   - **DFU書込先を app base 0x70000000(offset 0)** に変更(scratch廃止)。
   - **起動フロー**: DFUトリガ(ボタン/マジック)判定→無ければ`board_app_valid`→`SCB->VTOR=0x70000000`→app jump。
   - **SWD安全監査**: PA13/14不変更・DBGMCU非改変・低電力なし・**オプションバイト/RDP絶対触らない**。焼く前にobjdump監査。
3. ✅ **Phase 3完了(2026-07-12)**: `build/bootrom.bin`を良品Discovery ST-Link(SN 51FF.., V2J46S0)+`STM32_Programmer_CLI -c port=SWD mode=UR -w build/bootrom.bin 0x08000000 -v`で書込→verify成功→UR再確認clean(RDP 0xAA)。boot flow/DFUモード/E2E全動作。TinyUF2復元は`... -w _ref/wio_flash_backup_20260710_225512.bin 0x08000000`。
   - **今後アプリを焼く手順**: PF1保持でリセット(or `STM32_Programmer_CLI -c port=SWD mode=UR --start`をPF1保持中に)→bootrom DFUモード→`dfu-util -d cafe:4000 -a0 -D <app>.bin`→自動reboot→起動。
   - **残タスク(任意・低優先)**: (a)manifest-reboot時のdfu-util警告をクリーン化(MANIFEST_TOLERANTクリア等)、(b)VID/PID cafe:4000は開発用→リリース前に正式値へ、(c)将来クロック継承できないアプリ用にHSI48+CRS保険。

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
