/*
 * OCTOSPI2 (W25Q128) bring-up + erase/program for the standalone DFU bootloader.
 *
 * Runs from internal flash, so (unlike the app-first RAM driver in boot/ospi_ram.c)
 * these are ordinary flash-resident functions: nothing executes from or speculatively
 * reads 0x70000000 while OCTOSPI2 is out of memory-mapped mode.  Caches stay OFF
 * throughout the bootloader (see bootrom/main.c), which keeps register/mmap accesses
 * coherent.  Addresses are flash-internal offsets (0-based); the app base is offset 0.
 */

#ifndef BOOTROM_OCTOSPI_H_
#define BOOTROM_OCTOSPI_H_

#include <stdint.h>

// Bring up OCTOSPI2 memory-mapped mode (0xEB Quad I/O).  Returns 1 if the flash
// JEDEC id looks like a Winbond W25Q (mfr 0xEF); jedec[3] receives the id.
int  octospi2_init(uint8_t jedec[3]);

// Erase the 4 KB sector containing `addr`; restores memory-mapped mode on return.
void octospi2_erase_sector(uint32_t addr);

// Program `len` bytes from `data` at `addr` (256 B page-split); restores mmap.
// The caller must have erased the covering sector(s) first.  `data` must be in RAM.
void octospi2_program(uint32_t addr, const uint8_t *data, uint32_t len);

#endif /* BOOTROM_OCTOSPI_H_ */
