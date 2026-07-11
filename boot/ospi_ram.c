/*
 * RAM-resident OCTOSPI2 self-programming driver (W25Q128 on the Wio Lite AI).
 * See ospi_ram.h.  Everything that runs while the flash is out of memory-mapped
 * mode is RAMFUNC (placed in .RamFunc -> AXI-SRAM) and uses only registers/stack/
 * RAM globals -- NO .rodata, NO libc, NO XIP calls.  Caches must be OFF (see header).
 *
 * W25Q128 commands (1-line SPI, independent of the mmap quad config):
 *   WREN 0x06 | RDSR1 0x05 (WIP=bit0) | 4K sector erase 0x20 | page program 0x02 |
 *   JEDEC id 0x9F (Winbond mfr = 0xEF).  24-bit addressing.
 *
 * The mmap read config (0xEB quad I/O) is snapshotted at boot and rewritten to
 * restore memory-mapped mode after programming (RM0468 25.4.18: barrier then abort
 * to leave mmap; reconfigure CCR/TCR/IR + CR.FMODE=11 to re-enter).
 */

#include "stm32h7xx_hal.h"
#include "ospi_ram.h"

#define RAMFUNC __attribute__((section(".RamFunc"), noinline))

// CCR field encodings for 1-line SPI (mode field value 1), 8-bit instruction,
// 24-bit address (ADSIZE=2).
#define CCR_IMODE_1     (1u << OCTOSPI_CCR_IMODE_Pos)
#define CCR_ADMODE_1    (1u << OCTOSPI_CCR_ADMODE_Pos)
#define CCR_ADSIZE_24   (2u << OCTOSPI_CCR_ADSIZE_Pos)
#define CCR_DMODE_1     (1u << OCTOSPI_CCR_DMODE_Pos)

#define FMODE_WRITE     (0u << OCTOSPI_CR_FMODE_Pos)   // indirect write
#define FMODE_READ      (1u << OCTOSPI_CR_FMODE_Pos)   // indirect read

// Spin bounds (IRQs off, so these must terminate).  SPIN_MAX covers a single fast
// command; WIP_MAX loops the status read to cover a 4 KB sector erase (~hundreds ms).
#define SPIN_MAX   0x00200000u
#define WIP_MAX    0x00100000u

// Snapshot of the live memory-mapped read configuration (RAM .bss).
static uint32_t s_cr, s_ccr, s_tcr, s_ir;

void ospi_flash_snapshot(void)
{
  // mmap is active here; capture the read command config + CR (FMODE=mmap).
  s_ccr = OCTOSPI2->CCR;
  s_tcr = OCTOSPI2->TCR;
  s_ir  = OCTOSPI2->IR;
  s_cr  = OCTOSPI2->CR;
}

RAMFUNC static void wait_sr(uint32_t mask)
{
  uint32_t n = SPIN_MAX;
  while (((OCTOSPI2->SR & mask) != mask) && (n != 0u)) n--;
}

RAMFUNC static void wait_not_busy(void)
{
  uint32_t n = SPIN_MAX;
  while ((OCTOSPI2->SR & OCTOSPI_SR_BUSY) && (n != 0u)) n--;
}

// Exit memory-mapped mode back to idle (RM 25.4.18: barrier then abort).
RAMFUNC static void ospi_abort(void)
{
  uint32_t n = SPIN_MAX;
  __DSB();
  OCTOSPI2->CR |= OCTOSPI_CR_ABORT;
  while ((OCTOSPI2->CR & OCTOSPI_CR_ABORT) && (n != 0u)) n--;   // self-clears
  wait_not_busy();
  OCTOSPI2->FCR = OCTOSPI_FCR_CTCF | OCTOSPI_FCR_CTEF;
}

// Restore the memory-mapped read config captured by ospi_flash_snapshot().
RAMFUNC static void ospi_restore_mmap(void)
{
  wait_not_busy();
  OCTOSPI2->FCR = OCTOSPI_FCR_CTCF | OCTOSPI_FCR_CTEF;
  OCTOSPI2->CCR = s_ccr;
  OCTOSPI2->TCR = s_tcr;
  OCTOSPI2->IR  = s_ir;
  OCTOSPI2->CR  = s_cr;    // re-arms mmap (FMODE=11) + original EN/FSEL/FTHRES
  __DSB();
}

// Instruction-only command, no address/data (e.g. WREN).
RAMFUNC static void ospi_cmd_only(uint8_t instr)
{
  wait_not_busy();
  OCTOSPI2->FCR = OCTOSPI_FCR_CTCF | OCTOSPI_FCR_CTEF;
  MODIFY_REG(OCTOSPI2->CR, OCTOSPI_CR_FMODE, FMODE_WRITE);
  OCTOSPI2->TCR = 0;
  OCTOSPI2->CCR = CCR_IMODE_1;
  OCTOSPI2->IR  = instr;            // no address/data -> writing IR triggers
  wait_sr(OCTOSPI_SR_TCF);
  OCTOSPI2->FCR = OCTOSPI_FCR_CTCF;
}

// Instruction + data-in, no address (RDSR1 / JEDEC id): 1-line.
RAMFUNC static void ospi_read_cmd(uint8_t instr, uint8_t *buf, uint32_t n)
{
  wait_not_busy();
  OCTOSPI2->FCR = OCTOSPI_FCR_CTCF | OCTOSPI_FCR_CTEF;
  MODIFY_REG(OCTOSPI2->CR, OCTOSPI_CR_FMODE, FMODE_READ);
  OCTOSPI2->DLR = n - 1u;
  OCTOSPI2->TCR = 0;
  OCTOSPI2->CCR = CCR_IMODE_1 | CCR_DMODE_1;   // no address
  OCTOSPI2->IR  = instr;                        // triggers
  for (uint32_t i = 0; i < n; i++)
  {
    wait_sr(OCTOSPI_SR_FTF);
    buf[i] = *(volatile uint8_t *)&OCTOSPI2->DR;
  }
  wait_sr(OCTOSPI_SR_TCF);
  OCTOSPI2->FCR = OCTOSPI_FCR_CTCF;
}

RAMFUNC static void ospi_wait_wip(void)
{
  uint32_t n = WIP_MAX;
  uint8_t sr;
  do {
    ospi_read_cmd(0x05u, &sr, 1u);   // RDSR1
    n--;
  } while ((sr & 0x01u) && (n != 0u)); // WIP
}

RAMFUNC static void ospi_erase_4k(uint32_t addr)
{
  ospi_cmd_only(0x06u);   // WREN
  wait_not_busy();
  OCTOSPI2->FCR = OCTOSPI_FCR_CTCF | OCTOSPI_FCR_CTEF;
  MODIFY_REG(OCTOSPI2->CR, OCTOSPI_CR_FMODE, FMODE_WRITE);
  OCTOSPI2->TCR = 0;
  OCTOSPI2->CCR = CCR_IMODE_1 | CCR_ADMODE_1 | CCR_ADSIZE_24;   // instr + addr
  OCTOSPI2->IR  = 0x20u;
  OCTOSPI2->AR  = addr;            // writing AR triggers
  wait_sr(OCTOSPI_SR_TCF);
  OCTOSPI2->FCR = OCTOSPI_FCR_CTCF;
  ospi_wait_wip();
}

// Program up to 256 bytes within a single page.
RAMFUNC static void ospi_program_page(uint32_t addr, const uint8_t *data, uint32_t n)
{
  ospi_cmd_only(0x06u);   // WREN
  wait_not_busy();
  OCTOSPI2->FCR = OCTOSPI_FCR_CTCF | OCTOSPI_FCR_CTEF;
  MODIFY_REG(OCTOSPI2->CR, OCTOSPI_CR_FMODE, FMODE_WRITE);
  OCTOSPI2->DLR = n - 1u;
  OCTOSPI2->TCR = 0;
  OCTOSPI2->CCR = CCR_IMODE_1 | CCR_ADMODE_1 | CCR_ADSIZE_24 | CCR_DMODE_1;
  OCTOSPI2->IR  = 0x02u;
  OCTOSPI2->AR  = addr;            // triggers; peripheral pulls data via FIFO
  for (uint32_t i = 0; i < n; i++)
  {
    wait_sr(OCTOSPI_SR_FTF);
    *(volatile uint8_t *)&OCTOSPI2->DR = data[i];
  }
  wait_sr(OCTOSPI_SR_TCF);
  OCTOSPI2->FCR = OCTOSPI_FCR_CTCF;
  ospi_wait_wip();
}

RAMFUNC void ospi_ram_recover(void)
{
  ospi_restore_mmap();
  __DSB();
  __ISB();
}

RAMFUNC void ospi_ram_erase_sector(uint32_t flash_addr)
{
  __disable_irq();
  __DSB();
  __ISB();

  ospi_abort();
  ospi_erase_4k(flash_addr & ~0xFFFu);
  ospi_restore_mmap();

  __DSB();
  __ISB();
  __enable_irq();
}

RAMFUNC void ospi_ram_program(uint32_t flash_addr, const uint8_t *data, uint32_t len)
{
  __disable_irq();
  __DSB();
  __ISB();

  ospi_abort();
  uint32_t off = 0;
  while (off < len)
  {
    uint32_t page_off = (flash_addr + off) & 0xFFu;
    uint32_t chunk = 256u - page_off;
    if (chunk > (len - off)) chunk = len - off;
    ospi_program_page(flash_addr + off, data + off, chunk);
    off += chunk;
  }
  ospi_restore_mmap();

  __DSB();
  __ISB();
  __enable_irq();
}

RAMFUNC int ospi_ram_erase_program(uint32_t flash_addr, const uint8_t *data,
                                   uint32_t len, uint8_t jedec[3])
{
  __disable_irq();
  __DSB();
  __ISB();

  ospi_abort();                       // leave memory-mapped mode

  ospi_read_cmd(0x9Fu, jedec, 3u);    // JEDEC id sanity check (Winbond mfr 0xEF)

  ospi_erase_4k(flash_addr & ~0xFFFu);

  uint32_t off = 0;
  while (off < len)
  {
    uint32_t page_off = (flash_addr + off) & 0xFFu;
    uint32_t chunk = 256u - page_off;
    if (chunk > (len - off)) chunk = len - off;
    ospi_program_page(flash_addr + off, data + off, chunk);
    off += chunk;
  }

  ospi_restore_mmap();

  __DSB();
  __ISB();
  __enable_irq();
  return 0;
}
