/*
 * OCTOSPI2 memory-mapped bring-up for the standalone bootloader (W25Q128).
 *
 * Runs from internal flash, so (unlike the app-first RAM driver) it can do indirect
 * commands normally -- it never executes from the flash it is bringing up, caches are
 * off, and nothing speculatively touches 0x70000000 until mmap is enabled.
 *
 * Reproduces TinyUF2's captured config (boot/phase2_config_dump.md):
 *   - OCTOSPIM stays at reset (OCTOSPI2 -> Port 2); GPIO banks F & G replayed verbatim
 *     (they carry the Port-2 flash pins).
 *   - OCTOSPI2 DCR1=0x00170008 (16 MB), DCR2=2 (/3 ~88.7 MHz).
 *   - Read command 0xEB Quad I/O: CCR=0x03032301, TCR=4 dummy, mode byte 0.
 *   - Memory-mapped: CR=0x30400381 (FMODE=11).
 * Ensures the W25Q Quad-Enable (QE, SR2 bit 1) is set before enabling quad reads.
 */

#include "stm32h7xx_hal.h"

// CCR field encodings (1-line SPI for the setup commands).
#define CCR_IMODE_1   (1u << OCTOSPI_CCR_IMODE_Pos)
#define CCR_ADMODE_1  (1u << OCTOSPI_CCR_ADMODE_Pos)
#define CCR_ADSIZE_24 (2u << OCTOSPI_CCR_ADSIZE_Pos)   // 24-bit address (W25Q128)
#define CCR_DMODE_1   (1u << OCTOSPI_CCR_DMODE_Pos)
#define FMODE_WRITE   (0u << OCTOSPI_CR_FMODE_Pos)
#define FMODE_READ    (1u << OCTOSPI_CR_FMODE_Pos)

// CR base (from the dump) minus FMODE: EN | FSEL | FTHRES=3 | APMS.
#define CR_BASE       (0x30400381u & ~OCTOSPI_CR_FMODE_Msk)
#define CR_MMAP       0x30400381u

// Memory-mapped 0xEB Quad I/O read config (from the dump); re-applied to re-enter
// mmap after an erase/program leaves it for indirect commands.
#define MMAP_CCR      0x03032301u
#define MMAP_TCR      0x00000004u
#define MMAP_IR       0x000000EBu

#define SPIN 0x00100000u

static void wait_not_busy(void)
{
  uint32_t n = SPIN;
  while ((OCTOSPI2->SR & OCTOSPI_SR_BUSY) && n) n--;
}
static void wait_flag(uint32_t f)
{
  uint32_t n = SPIN;
  while (!(OCTOSPI2->SR & f) && n) n--;
}

// Instruction-only (WREN).
static void cmd_only(uint8_t instr)
{
  wait_not_busy();
  OCTOSPI2->FCR = OCTOSPI_FCR_CTCF | OCTOSPI_FCR_CTEF;
  MODIFY_REG(OCTOSPI2->CR, OCTOSPI_CR_FMODE, FMODE_WRITE);
  OCTOSPI2->TCR = 0;
  OCTOSPI2->CCR = CCR_IMODE_1;
  OCTOSPI2->IR  = instr;
  wait_flag(OCTOSPI_SR_TCF);
  OCTOSPI2->FCR = OCTOSPI_FCR_CTCF;
}

// Instruction + data-in (JEDEC 0x9F, RDSR 0x05/0x35), 1-line, no address.
static void read_reg(uint8_t instr, uint8_t *buf, uint32_t n)
{
  wait_not_busy();
  OCTOSPI2->FCR = OCTOSPI_FCR_CTCF | OCTOSPI_FCR_CTEF;
  MODIFY_REG(OCTOSPI2->CR, OCTOSPI_CR_FMODE, FMODE_READ);
  OCTOSPI2->DLR = n - 1u;
  OCTOSPI2->TCR = 0;
  OCTOSPI2->CCR = CCR_IMODE_1 | CCR_DMODE_1;
  OCTOSPI2->IR  = instr;
  for (uint32_t i = 0; i < n; i++) { wait_flag(OCTOSPI_SR_FTF); buf[i] = *(volatile uint8_t *)&OCTOSPI2->DR; }
  wait_flag(OCTOSPI_SR_TCF);
  OCTOSPI2->FCR = OCTOSPI_FCR_CTCF;
}

// Instruction + data-out (Write Status Register-2 0x31), 1-line, no address.
static void write_reg(uint8_t instr, const uint8_t *buf, uint32_t n)
{
  wait_not_busy();
  OCTOSPI2->FCR = OCTOSPI_FCR_CTCF | OCTOSPI_FCR_CTEF;
  MODIFY_REG(OCTOSPI2->CR, OCTOSPI_CR_FMODE, FMODE_WRITE);
  OCTOSPI2->DLR = n - 1u;
  OCTOSPI2->TCR = 0;
  OCTOSPI2->CCR = CCR_IMODE_1 | CCR_DMODE_1;
  OCTOSPI2->IR  = instr;
  for (uint32_t i = 0; i < n; i++) { wait_flag(OCTOSPI_SR_FTF); *(volatile uint8_t *)&OCTOSPI2->DR = buf[i]; }
  wait_flag(OCTOSPI_SR_TCF);
  OCTOSPI2->FCR = OCTOSPI_FCR_CTCF;
}

static void wait_wip(void)
{
  uint32_t n = SPIN;
  uint8_t sr;
  do { read_reg(0x05u, &sr, 1u); n--; } while ((sr & 0x01u) && n);
}

// Ensure W25Q Quad-Enable (SR2 bit 1).  Non-volatile, so usually already set; only
// program it if missing (status-register endurance).
static void ensure_qe(void)
{
  uint8_t sr2 = 0;
  read_reg(0x35u, &sr2, 1u);           // Read Status Register-2
  if (!(sr2 & 0x02u))
  {
    cmd_only(0x06u);                   // WREN
    uint8_t v = (uint8_t)(sr2 | 0x02u);
    write_reg(0x31u, &v, 1u);          // Write Status Register-2
    wait_wip();
  }
}

static void gpio_replay(GPIO_TypeDef *g, uint32_t moder, uint32_t otyper,
                        uint32_t ospeedr, uint32_t pupdr, uint32_t afrl, uint32_t afrh)
{
  g->OTYPER = otyper; g->OSPEEDR = ospeedr; g->PUPDR = pupdr;
  g->AFR[0] = afrl;   g->AFR[1] = afrh;
  g->MODER  = moder;                   // MODER last so AF is selected before it activates
}

// Bring up OCTOSPI2 memory-mapped mode; returns 1 if the flash JEDEC id looks like a
// Winbond W25Q (mfr 0xEF).  jedec[3] receives the id.
int octospi2_init(uint8_t jedec[3])
{
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_OSPI2_CLK_ENABLE();
  // OCTOSPI1 is clocked too (captured AHB3ENR has OSPI1EN set): the bank replay below
  // routes the PSRAM's Port-1 pins (PF6-10, PG6) to OCTOSPI1 in AF mode, and an AF pin
  // should not hang off an unclocked peripheral.  Idle OCTOSPI1 keeps its NCS high, so
  // the PSRAM (external pull-up too) stays deselected -- same state TinyUF2 left.
  __HAL_RCC_OSPI1_CLK_ENABLE();
  __HAL_RCC_OCTOSPIM_CLK_ENABLE();

  // Replay TinyUF2's exact GPIO for the OCTOSPI banks.  This covers the W25Q128 app
  // flash on OCTOSPIM Port 2 -- PF4=CLK(AF9), PG12=NCS(AF3), PG0/PG1/PG10/PG11=
  // IO4-7(AF9/AF3, quad data on the P2 high nibble) per schematic sheet 6 ("QSPI2_*"
  // nets) -- and the PSRAM's Port-1 pins (PF6-10, PG6, "OSPI1_*" nets).  Net names do
  // NOT match peripheral numbers; replaying both banks verbatim sidesteps that trap.
  gpio_replay(GPIOF, 0xFFEAAEF3u, 0x00000000u, 0x003FF300u, 0x00000004u, 0xAA090000u, 0x000009AAu);
  gpio_replay(GPIOG, 0xFEAFEFFAu, 0x00000000u, 0x03F0300Fu, 0x00000000u, 0x0A000099u, 0x00039300u);

  // OCTOSPIM left at reset default (OCTOSPI2 -> Port 2).

  OCTOSPI2->DCR1 = 0x00170008u;        // 16 MB device, clock mode / CS timing
  OCTOSPI2->DCR2 = 0x00000002u;        // prescaler -> /3 (~88.7 MHz)
  OCTOSPI2->DCR3 = 0x00000000u;
  OCTOSPI2->CR   = CR_BASE;            // enable, indirect mode

  read_reg(0x9Fu, jedec, 3u);          // JEDEC id (validation)
  ensure_qe();                         // set Quad-Enable if needed

  // Configure the 0xEB Quad I/O read and switch to memory-mapped mode.
  OCTOSPI2->CCR = MMAP_CCR;
  OCTOSPI2->TCR = MMAP_TCR;
  OCTOSPI2->IR  = MMAP_IR;
  OCTOSPI2->ABR = 0x00000000u;
  __DSB();
  OCTOSPI2->CR  = CR_MMAP;             // FMODE=11 memory-mapped
  __DSB();
  __ISB();

  return (jedec[0] == 0xEFu);
}

//--------------------------------------------------------------------+
// Erase / program (standalone: this code runs from internal flash, so unlike the
// app-first RAM driver it can leave OCTOSPI2 in indirect mode normally -- nothing
// executes from or speculatively reads 0x70000000 while mmap is off, caches are off,
// and IRQ handlers live in internal flash).  W25Q128 1-line commands, 24-bit address:
// WREN 0x06, sector-erase-4K 0x20, page-program 0x02, RDSR1 0x05 (WIP = bit 0).
//--------------------------------------------------------------------+

// Leave memory-mapped mode for indirect commands (RM0468: abort, then reconfigure).
static void abort_to_indirect(void)
{
  __DSB();
  OCTOSPI2->CR |= OCTOSPI_CR_ABORT;
  { uint32_t n = SPIN; while ((OCTOSPI2->CR & OCTOSPI_CR_ABORT) && n) n--; }  // self-clears
  wait_not_busy();
  OCTOSPI2->FCR = OCTOSPI_FCR_CTCF | OCTOSPI_FCR_CTEF;
  OCTOSPI2->CR  = CR_BASE;             // FMODE=00 indirect write
}

// Re-apply the 0xEB Quad I/O read config and re-enter memory-mapped mode.
static void restore_mmap(void)
{
  wait_not_busy();
  OCTOSPI2->FCR = OCTOSPI_FCR_CTCF | OCTOSPI_FCR_CTEF;
  OCTOSPI2->CCR = MMAP_CCR;
  OCTOSPI2->TCR = MMAP_TCR;
  OCTOSPI2->IR  = MMAP_IR;
  OCTOSPI2->ABR = 0x00000000u;
  __DSB();
  OCTOSPI2->CR  = CR_MMAP;
  __DSB();
  __ISB();
}

static void erase_4k(uint32_t addr)
{
  cmd_only(0x06u);                     // WREN
  wait_not_busy();
  OCTOSPI2->FCR = OCTOSPI_FCR_CTCF | OCTOSPI_FCR_CTEF;
  MODIFY_REG(OCTOSPI2->CR, OCTOSPI_CR_FMODE, FMODE_WRITE);
  OCTOSPI2->TCR = 0;
  OCTOSPI2->CCR = CCR_IMODE_1 | CCR_ADMODE_1 | CCR_ADSIZE_24;
  OCTOSPI2->IR  = 0x20u;
  OCTOSPI2->AR  = addr;                // writing AR triggers
  wait_flag(OCTOSPI_SR_TCF);
  OCTOSPI2->FCR = OCTOSPI_FCR_CTCF;
  wait_wip();
}

// Program up to 256 bytes within a single page.
static void program_page(uint32_t addr, const uint8_t *data, uint32_t n)
{
  cmd_only(0x06u);                     // WREN
  wait_not_busy();
  OCTOSPI2->FCR = OCTOSPI_FCR_CTCF | OCTOSPI_FCR_CTEF;
  MODIFY_REG(OCTOSPI2->CR, OCTOSPI_CR_FMODE, FMODE_WRITE);
  OCTOSPI2->DLR = n - 1u;
  OCTOSPI2->TCR = 0;
  OCTOSPI2->CCR = CCR_IMODE_1 | CCR_ADMODE_1 | CCR_ADSIZE_24 | CCR_DMODE_1;
  OCTOSPI2->IR  = 0x02u;
  OCTOSPI2->AR  = addr;                // triggers; data pulled through the FIFO
  for (uint32_t i = 0; i < n; i++) { wait_flag(OCTOSPI_SR_FTF); *(volatile uint8_t *)&OCTOSPI2->DR = data[i]; }
  wait_flag(OCTOSPI_SR_TCF);
  OCTOSPI2->FCR = OCTOSPI_FCR_CTCF;
  wait_wip();
}

// Erase the 4 KB sector containing `addr` (flash-internal offset), restore mmap.
void octospi2_erase_sector(uint32_t addr)
{
  abort_to_indirect();
  erase_4k(addr & ~0xFFFu);
  restore_mmap();
}

// Program `len` bytes at `addr` (flash-internal offset), splitting on 256 B pages,
// then restore mmap.  Caller erases the covering sector(s) first.
void octospi2_program(uint32_t addr, const uint8_t *data, uint32_t len)
{
  abort_to_indirect();
  uint32_t off = 0;
  while (off < len)
  {
    uint32_t page_off = (addr + off) & 0xFFu;
    uint32_t chunk = 256u - page_off;
    if (chunk > (len - off)) chunk = len - off;
    program_page(addr + off, data + off, chunk);
    off += chunk;
  }
  restore_mmap();
}
