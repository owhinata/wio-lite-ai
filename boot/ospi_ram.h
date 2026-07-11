/*
 * RAM-resident OCTOSPI2 self-programming driver for the Wio Lite AI DFU bootloader.
 *
 * The application executes XIP from OCTOSPI2 (0x70000000).  To erase/program that
 * same flash the peripheral must leave memory-mapped mode, during which no code or
 * IRQ handler in XIP can run -- so the erase/program routines are RAM-resident and
 * run with interrupts disabled, then restore memory-mapped mode before returning.
 *
 * IMPORTANT: the CPU data caches must be OFF around these calls.  With the D-cache
 * on, RAM-context accesses to the OCTOSPI register block (0x5200_0000, reached via
 * the M7 AXIM port) are incoherent -- reads return stale data and writes are lost --
 * and speculative linefills from the 0x70000000 window fault while mmap is off.
 *
 * Addresses passed here are FLASH-INTERNAL offsets (0-based), e.g. 0x00400000 for
 * the AXI-mapped scratch at 0x70400000.
 */

#ifndef OSPI_RAM_H_
#define OSPI_RAM_H_

#include <stdint.h>

// Re-arm memory-mapped mode from a fault handler so XIP code can run again.
// RAM-resident; safe to call from an exception handler.
void ospi_ram_recover(void);

// Capture the bootloader's live memory-mapped read configuration so it can be
// restored after programming.  Call once at startup while mmap is still active
// (i.e. before the first erase/program), from normal (XIP) code.
void ospi_flash_snapshot(void);

// Erase one 4 KB sector (the one containing flash_addr) then program `len` bytes
// from `data` starting at flash_addr, handling 256-byte page boundaries.  Reads
// the JEDEC ID (0x9F) first into jedec[3] as a bring-up sanity check (Winbond mfr
// 0xEF).  Runs with IRQs disabled and restores mmap on return.  `data` and `jedec`
// must be in RAM.  Returns 0 on success.
int ospi_ram_erase_program(uint32_t flash_addr, const uint8_t *data, uint32_t len,
                           uint8_t jedec[3]);

// Split erase / program for per-block DFU download.  Each call wraps IRQ-disable +
// abort (leave mmap) + op + mmap restore, so it is self-contained.  Addresses are
// flash-internal offsets; `data` must be in RAM.
void ospi_ram_erase_sector(uint32_t flash_addr);
void ospi_ram_program(uint32_t flash_addr, const uint8_t *data, uint32_t len);

#endif /* OSPI_RAM_H_ */
