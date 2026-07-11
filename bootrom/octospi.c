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
#define CCR_DMODE_1   (1u << OCTOSPI_CCR_DMODE_Pos)
#define FMODE_WRITE   (0u << OCTOSPI_CR_FMODE_Pos)
#define FMODE_READ    (1u << OCTOSPI_CR_FMODE_Pos)

// CR base (from the dump) minus FMODE: EN | FSEL | FTHRES=3 | APMS.
#define CR_BASE       (0x30400381u & ~OCTOSPI_CR_FMODE_Msk)
#define CR_MMAP       0x30400381u

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
  __HAL_RCC_OCTOSPIM_CLK_ENABLE();

  // Replay TinyUF2's exact GPIO for the OCTOSPI banks (Port-2 flash pins included).
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
  OCTOSPI2->CCR = 0x03032301u;
  OCTOSPI2->TCR = 0x00000004u;
  OCTOSPI2->IR  = 0x000000EBu;
  OCTOSPI2->ABR = 0x00000000u;
  __DSB();
  OCTOSPI2->CR  = CR_MMAP;             // FMODE=11 memory-mapped
  __DSB();
  __ISB();

  return (jedec[0] == 0xEFu);
}
