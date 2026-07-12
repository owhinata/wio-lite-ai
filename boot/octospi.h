/*
 * OCTOSPI2 (W25Q128) bring-up + erase/program for the standalone bootloader.
 *
 * The bootloader runs from internal flash (not XIP from OCTOSPI2), so these are
 * ordinary flash-resident functions -- no RAM-resident driver or cache juggling
 * is needed (nothing executes from or speculatively reads 0x70000000 while
 * OCTOSPI2 is out of memory-mapped mode).  Caches stay OFF throughout the
 * bootloader.  Addresses are flash-internal offsets (0-based); app base = 0.
 */

#ifndef BOOT_OCTOSPI_H_
#define BOOT_OCTOSPI_H_

#include <stdint.h>

// Bring up OCTOSPI2 memory-mapped mode (0xEB Quad I/O).  Returns 1 if the flash
// JEDEC id looks like a Winbond W25Q (mfr 0xEF); jedec[3] receives the id.
int  octospi2_init(uint8_t jedec[3]);

// Erase the 4 KB sector containing `addr`; restores mmap on return.
void octospi2_erase_sector(uint32_t addr);

// Program `len` bytes from `data` at `addr` (256 B page-split); restores mmap.
// The caller must have erased the covering sector(s) first.  `data` in RAM.
void octospi2_program(uint32_t addr, const uint8_t *data, uint32_t len);

#endif /* BOOT_OCTOSPI_H_ */
