/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    nor_flash.h
 * @brief   External NOR flash (OCTOSPI2 / Winbond W25Q128JV) driver (issue #37).
 *
 * The 16 MB serial NOR on OCTOSPIM Port 2.  Until issue #25 the app executed in
 * place from this device and the bootloader owned it; now nothing maps it, so the
 * app brings it up itself -- the same ownership move app/psram.c made for the
 * OCTOSPI1 PSRAM, and the same idiom: bare registers, bounded poll loops,
 * fail-soft, per-pin GPIO RMW, and no RCC write anywhere.
 *
 * INDIRECT ONLY -- THE MEMORY-MAPPED WINDOW IS NEVER ENTERED.  Addresses here are
 * device offsets (0 .. 16 MB), not 0x70000000 pointers, and app/mpu.c keeps its
 * region 2 no-access + execute-never fence over the whole 0x70000000 quarter.
 * That fence is the point, not an oversight: a Normal-memory mapping of the window
 * lets the core issue *speculative* reads, and a speculative read that lands while
 * the controller is out of memory-mapped mode is a slave error (RM0468 sec 25.4.16)
 * that no lock can prevent -- the CPU never asked for it.  With the window fenced
 * off, only these functions ever reach the device, and everything they do is an
 * explicit indirect transaction.  (The sister project's f746 LevelX driver is
 * indirect-only for the same reason.)  A future mmap for bulk read-only blobs is a
 * separate design with its own MPU region; do not open one here.
 *
 * Wire protocol: 1-line SPI throughout, 24-bit addresses.  Six instructions --
 * JEDEC id 0x9F, Fast Read 0x0B (8 dummy cycles), WREN 0x06, RDSR1 0x05,
 * 4 KB sector erase 0x20, 64 KB block erase 0xD8 and page program 0x02.  Fast Read
 * rather than the plain 0x03 Read Data because 0x03 is capped at 50 MHz while
 * everything else runs to 133 MHz (W25Q128JV AC characteristics), and this bus
 * runs at 88.7 MHz.  Nothing here writes a status register, so the device's
 * non-volatile configuration (including Quad Enable) is left exactly as found.
 *
 * Concurrency: one RECURSIVE ThreadX mutex guards the device.  ThreadX counts
 * re-acquisition by the owning thread as an ownership-count increment, so a caller
 * may hold the lock across a whole multi-step sequence and still call the
 * primitives below, which take it again internally.  Two entry points exist on
 * purpose:
 *
 *   - nor_lock_blocking() waits forever.  It is what FlashDB's lock callback must
 *     use: that callback is `void (*)(fdb_db_t)` with no way to report failure, so
 *     a finite wait that gave up would let FlashDB continue *unlocked*.  Note the
 *     signatures do not match -- the callback takes the db and returns nothing --
 *     so the integration has to go through a void wrapper, and that wrapper must
 *     NOT simply drop this function's return value on the floor: an infinite wait
 *     can only fail if the object or the calling context is invalid, and if that
 *     ever happens the honest response is to stop, not to hand FlashDB a device it
 *     believes is locked.
 *   - nor_trylock() takes a timeout and reports failure, for the outer app-level
 *     API where refusing is better than blocking a shell session.
 *
 * THREAD CONTEXT ONLY.  Erase and program wait for the device by sleeping between
 * status polls (a 4 KB erase is typ 45 / max 400 ms), so these calls must never
 * run from an ISR, and never inside an interrupt-disabled or preemption-disabled
 * region -- the IWDG petter (priority 5, 1 s period, ~2.04 s timeout) has to keep
 * running through them.
 *
 * Clean-room implementation.  RM0468 (OCTOSPI), the W25Q128JV datasheet and the
 * bootloader's own deleted OCTOSPI2 driver (git show 7757823^:boot/octospi.c) were
 * used as register / opcode references only.
 */
#ifndef NOR_FLASH_H
#define NOR_FLASH_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Device geometry (Winbond W25Q128JV, 128 Mbit). */
#define NOR_SIZE_BYTES     0x01000000u   /* 16 MB                                */
#define NOR_SECTOR_SIZE    4096u         /* smallest erase unit (0x20)           */
#define NOR_BLOCK_SIZE     65536u        /* large erase unit (0xD8)              */
#define NOR_PAGE_SIZE      256u          /* program unit (0x02) -- no wrap       */

/* Error returns (negative); 0 is success. */
#define NOR_OK            0
#define NOR_ERR_PARAM    -1   /* bad argument (range, null buffer, alignment)    */
#define NOR_ERR_IO       -2   /* controller never completed the transaction      */
#define NOR_ERR_TIMEOUT  -3   /* device stayed busy past the erase/program budget*/
#define NOR_ERR_STATE    -4   /* driver not initialized / device did not answer  */
#define NOR_ERR_BUSY     -5   /* nor_trylock() timed out                         */

/**
 * One-time controller bring-up: OCTOSPI2 pins (per-pin RMW), its bus clock, the
 * device-configuration registers and a JEDEC id read to prove the flash answers.
 *
 * Register-only and self-timed (bounded spin loops, no HAL_GetTick, no libc, no
 * ThreadX call), so it is safe from main() before tx_kernel_enter() -- which is
 * where it must run, because it takes no lock.  That asymmetry is a contract, not
 * an accident: nor_flash_init() is the ONLY thing allowed to touch the device
 * before the scheduler starts, and it performs no erase or program, so nothing
 * ever needs to sleep before there is a scheduler to sleep in.
 *
 * Touches ONLY OCTOSPI2 and its six Port-2 pins.  It does not touch OCTOSPI1, the
 * PSRAM's Port-1 pins, the OCTOSPIM routing registers (P1CR/P2CR/CR -- their reset
 * values already route OCTOSPI2 to Port 2) or the RCC clock tree.
 *
 * Returns NOR_OK when a W25Q-class device answered, NOR_ERR_STATE otherwise.
 * Fail-soft: on failure the shell still comes up and `nor info` reports it down.
 */
int nor_flash_init(void);

/**
 * Create the device mutex.  Pure ThreadX object creation -- call from
 * tx_application_define().  Idempotent.  Until it runs, the locking calls below
 * are no-ops, which is exactly what nor_flash_init()'s pre-scheduler contract
 * needs.
 */
int nor_flash_lock_init(void);

/** Nonzero once nor_flash_init() found a device that answers. */
int nor_flash_ready(void);

/** JEDEC id bytes captured at bring-up (manufacturer, type, capacity). */
void nor_flash_get_id(uint8_t jedec[3]);

/** Device clock in Hz (OCTOSPI2 kernel clock / (DCR2 prescaler + 1)). */
uint32_t nor_flash_clock_hz(void);

/**
 * Live OCTOSPI2 CR / DCR1 / DCR2 / SR, for `nor info`.  A bring-up that fails
 * because the controller is configured wrong looks identical from the outside to
 * one that fails because the device is dead, so the registers have to be visible:
 * the first failure on this board was a wrong CR.FSEL, which reads out here and
 * nowhere else.
 */
void nor_flash_get_regs(uint32_t regs[4]);

/**
 * Take the device mutex, waiting forever.  Recursive: a thread that already holds
 * it returns immediately with the ownership count incremented.  Use this for
 * FlashDB's lock callback; use nor_trylock() at app-level entry points.
 */
int nor_lock_blocking(void);

/** Take the device mutex with a finite wait (ThreadX ticks = ms).  NOR_ERR_BUSY on timeout. */
int nor_trylock(uint32_t timeout_ticks);

/** Release one level of the device mutex.  A failed release is logged, not ignored. */
void nor_unlock(void);

/**
 * Read @p len bytes from device offset @p addr.  No alignment requirement and no
 * length limit beyond the device size: one Fast Read transaction streams the whole
 * range.  Takes the device mutex internally (recursive, so an outer holder pays
 * nothing).
 *
 * The whole range is copied by a CPU polling loop inside a single transaction, so
 * a long read keeps the calling thread busy for its duration (about 90 ns per byte
 * on the wire, ~1.4 s for the whole 16 MB).  That starves nothing -- the caller is
 * a thread and every higher-priority thread, the IWDG petter included, still
 * preempts it -- but it does hold the shell session that asked.  Configuration
 * reads are kilobytes and the `nor` command caps its dumps, so it does not matter
 * here; bulk blob reading would want chunking with a yield between chunks.
 */
int nor_read(uint32_t addr, void *buf, uint32_t len);

/**
 * Program @p len bytes at device offset @p addr, split into page-aligned chunks so
 * no write ever wraps within a 256 B page.  NOR semantics: this can only clear
 * bits, so the caller is responsible for having erased the range.  Programming a
 * byte that does not currently read 0xFF is outside what the datasheet specifies
 * (sec 8.2.13 programs "at previously erased (FFh) memory locations") -- the
 * FlashDB configuration this driver exists for is built to never need it
 * (FDB_WRITE_GRAN = 8 puts every status update in its own untouched byte).
 */
int nor_write(uint32_t addr, const void *buf, uint32_t len);

/**
 * Erase the range [@p addr, @p addr + @p len).  Both must be 4 KB-sector aligned.
 * Uses 64 KB block erases wherever the remaining range allows (a whole-partition
 * erase is ~6x faster that way) and 4 KB sector erases elsewhere.
 */
int nor_erase(uint32_t addr, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* NOR_FLASH_H */
