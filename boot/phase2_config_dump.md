# Phase 2 reference: TinyUF2's live config (to reproduce in the standalone bootloader)

Captured over USB CDC from the app-first firmware (`dump_config()` in `main.c`),
i.e. the exact clock / OCTOSPI2 / GPIO state TinyUF2 leaves before jumping to the
app.  The standalone bootloader at `0x08000000` must set all of this up itself
(app-first inherited it).  Values are hex.

## Raw dump
```
--- RCC ---
CR=3F03C025 CFGR=0000001B PLLCKSELR=00519022 PLLCFGR=01FF093D
PLL1DIVR=0104002B PLL1FRACR=00000000 PLL2DIVR=00010309 PLL2FRACR=00000000 PLL3DIVR=0104022F PLL3FRACR=00000000
D1CFGR=00000048 D2CFGR=00000440 D3CFGR=00000040 D1CCIPR=00000020 D2CCIP1R=00000000 D2CCIP2R=00200000 D3CCIPR=00000000
--- PWR --- CR1=F000C000 CR3=05010044 D3CR=00002000   --- FLASH --- ACR=00000033
--- OCTOSPI2 --- CR=30400381 DCR1=00170008 DCR2=00000002 DCR3=00000000
  CCR=03032301 TCR=00000004 IR=000000EB ABR=00000000 LPTR=00000000 PIR=00000010
--- OCTOSPIM --- CR=00000000 PCR1=00000000 PCR2=00000000 PCR3=00000000   (** verify: re-dump raw, see below **)
--- GPIO: MODER OTYPER OSPEEDR PUPDR AFRL AFRH ---
PA AABFFFFF 00000000 0FC00000 64000000 00000000 000AA000
PB FFFFFEAF 00000000 000000F0 00000100 00000A00 00000000
PC F7FFFFFF 00000000 00000000 00000000 00000000 00000000
PD FFFFBAFF 00000000 0000CF00 00000000 A0AA0000 00000000
PE FFFBFFFF 00000000 000C0000 00000000 00000000 000000A0
PF FFEAAEF3 00000000 003FF300 00000004 AA090000 000009AA
PG FEAFEFFA 00000000 03F0300F 00000000 0A000099 00039300
PH FFFFFFFF 00000000 00000000 00000000 00000000 00000000
PJ/PK: bogus (read-back pattern; H725AEI does not bond these) -- ignore
```

## Decoded clock tree (HSE 25 MHz crystal -> sysclk = PLL1)
- Supply/voltage: PWR CR3=05010044, D3CR=00002000 (VOSRDY set), FLASH ACR=0x33
  (latency + WRHIGHFREQ for 550 MHz).  Use HAL_PWREx_ConfigSupply + VOS0.
- PLLCKSELR=00519022: PLLSRC=HSE(2), DIVM1=2, DIVM2=25, DIVM3=5.
- PLL1: M=2 (12.5 MHz), N=44, P=1, Q=5, R=2  -> VCO 550, **sysclk 550 (CPU)**, Q=110, R=275.
- PLL2: M=25 (1 MHz), N=266, P=2, Q=2, R=1   -> **PLL2R 266 -> OCTOSPI2 kernel**.
- PLL3: M=5 (5 MHz),  N=48, P=2, Q=5, R=2    -> **PLL3Q 48 MHz -> USB**.
- D1CFGR=0x48: HPRE=/2 (AXI/AHB 275), D1CPRE=/1 (CPU 550), D1PPRE=/2.
- D1CCIPR=0x20: OCTOSPISEL -> PLL2R.   D2CCIP2R=0x200000: USBSEL -> PLL3Q.

## OCTOSPI2 memory-mapped config (W25Q128 quad I/O)
- DCR1=00170008 (DEVSIZE 16 MB, CKMODE etc.), DCR2=2 (prescaler /3 -> ~88.7 MHz).
- Read: IR=0xEB (Fast Read Quad I/O), CCR=03032301 (instr 1-line, addr 4-line/24-bit,
  mode byte 4-line ABR=0x00, data 4-line, SIOO=0), TCR=4 dummy cycles.
- CR=30400381 (EN, FMODE=11 mmap, FSEL, FTHRES=3, APMS).
- The flash also needs its Quad-Enable (QE) bit set (already set by TinyUF2; the
  standalone init must issue the W25Q QE write if starting from reset).

## OCTOSPI2 flash pins (from schematic 733260648, nets "OSPI1_*" = OCTOSPIM Port 1)
  CLK=PF10(AF9)  CS=PG6(AF10)  IO0=PF8  IO1=PF9  IO2=PF7  IO3=PF6 (AF10), speed=VERY_HIGH.
  (PSRAM on OCTOSPI1 @0x90000000 uses the "QSPI2_*" nets: PF4/PG0/PG1/PG10/PG11 -- not
  needed by the bootloader.)

## OCTOSPIM routing -- RESOLVED
Re-dumped with the IOMNGR clock on: **OCTOSPIM_CR=0 (MUXEN=0), P1CR=0x03010111,
P2CR=0x07050333 = exactly the reset values** (RM0468 26.5.2).  So OCTOSPIM is at its
default: **OCTOSPI1 -> Port 1, OCTOSPI2 -> Port 2, no multiplexing.**  The W25Q128 app
flash is OCTOSPI2 (registers 0x5200A000, mmap 0x70000000, JEDEC EF4018) => on **Port 2**.
=> The standalone init does NOT need to touch OCTOSPIM (leave at reset).
(The earlier "flash on Port1/AF10" reading was the OCTOSPI1 device/PSRAM; schematic net
names "OSPI1_*"/"QSPI2_*" don't map 1:1 to the peripheral number.  Ground truth: the app
flash is driven by the OCTOSPI2 register block, confirmed by the working RAM driver.)

## Clocks the standalone bootloader must enable (from the live dump)
- AHB4ENR=0x000000FF -> GPIOA..GPIOH.   AHB1ENR bit25 -> USB1_OTG_HS.
- AHB3ENR=0x00284000 -> OSPI1EN(14) + OSPI2EN(19) + IOMNGREN(21).
- Plus PWR (supply/VOS) and SYSCFG as needed by the clock bring-up.

## Standalone init strategy (de-risked)
Reproduce the captured state rather than re-derive:
1. PWR supply + VOS0, HSE crystal on, PLL1/2/3 per the decoded params, flash latency,
   switch sysclk to PLL1, set OCTOSPISEL=PLL2R and USBSEL=PLL3Q (use HAL for sequencing).
2. Enable GPIOA-H clocks; replay the captured GPIO MODER/OTYPER/OSPEEDR/PUPDR/AFRL/AFRH
   for the banks with OCTOSPI2 pins (F, G) -- covers the Port-2 flash pins exactly.
3. Enable OSPI2EN + IOMNGREN; leave OCTOSPIM at reset; program OCTOSPI2 DCR1/DCR2, then the
   0xEB quad read command (CCR/TCR/IR/ABR) and CR.FMODE=11 (mmap).  Ensure W25Q QE bit set.
4. DFU download target becomes app base flash offset 0 (0x70000000), scratch dropped.

## Remaining before flashing (Phase 3)
- ldscript -> 0x08000000 (internal 512 KB); full SystemClock_Config (drop the hands-off
  system_stm32h7xx.c); boot flow (DFU trigger / app-valid -> jump).
- SWD-safety audit (objdump): no RDP/option-byte/PA13-14/DBGMCU writes.
- Flash with STM32L-Discovery ST-Link + CubeProgrammer mode=UR; TinyUF2 backup restores.
