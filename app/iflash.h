/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    iflash.h
 * @brief   Internal-flash (sector 1-3) erase/program driver -- issue #25.
 *
 * The app is moving from XIP out of the external OCTOSPI2 window (0x70000000,
 * ~44 MB/s) to execution from the internal flash at 0x08020000 (~905 MB/s).  The
 * DFU bootloader is what will actually program that region, but the driver is
 * validated HERE FIRST, app-side, while the app still runs XIP from the external
 * flash: the instruction fetch then comes from OCTOSPI2, so it is completely
 * unaffected by the internal bank being busy, and sector 0 (the bootloader) is
 * never a candidate.  Same "app-first validation" pattern as the PSRAM bring-up
 * (BSP_PSRAM_INIT_IN_APP) and the DFU bootloader's own phase 1.
 *
 * The file is deliberately dependency-free (HAL only, no libc, no ThreadX, no
 * shell) so it can be copied verbatim into boot/ once validated.
 *
 * MEMORY MAP (RM0468 sec 4.3.4 Table 15 -- STM32H723/733 and H725/735):
 *   one bank, 128 KB sectors.
 *     sector 0  0x08000000-0x0801FFFF  the DFU bootloader.  NEVER TOUCHED:
 *                                      iflash_erase_sector() rejects it before
 *                                      the HAL is called, and every program
 *                                      offset is relative to sector 1, so no
 *                                      argument value can reach sector 0.
 *     sector 1  0x08020000-0x0803FFFF  app
 *     sector 2  0x08040000-0x0805FFFF  app
 *     sector 3  0x08060000-0x0807FFFF  app
 *
 * The H725AE is a 512 KB part, so only sectors 0-3 exist -- but RM0468 describes
 * the family as up to 1 MB / sectors 0-7 and CMSIS defines FLASH_SECTOR_TOTAL as
 * 8, so "4 sectors" cannot be concluded from the reference manual alone.
 * iflash_available() confirms it against the device's own flash-size register and
 * everything else refuses to run unless it agrees.
 *
 * WRITE GRANULARITY (RM0468 sec 4.3.9): a program writes a 256-bit (32 byte) flash
 * word plus 10 ECC bits; a non-virgin word may not be overwritten.  Hence
 * erase-then-program, 32-byte-aligned, with short tails padded to 0xFF.
 *
 * Clean-room design; no third-party code reused.
 */
#ifndef APP_IFLASH_H
#define APP_IFLASH_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IFLASH_BASE_ADDR      0x08000000u   /* internal flash, AXI-mapped */
#define IFLASH_SECTOR_SIZE    0x00020000u   /* 128 KB (RM0468 Table 15)   */
#define IFLASH_WORD_SIZE      32u           /* 256-bit flash word + ECC   */

/* The app partition: sectors 1-3.  Offsets passed to iflash_program() are
 * relative to IFLASH_APP_BASE, which is what makes sector 0 unreachable. */
#define IFLASH_APP_BASE       0x08020000u
#define IFLASH_APP_SIZE       0x00060000u   /* 384 KB = 3 x 128 KB */
#define IFLASH_APP_SECTOR_LO  1u
#define IFLASH_APP_SECTOR_HI  3u

/* Longest single program call.  Matches the DFU transfer size the bootloader
 * will hand down (CFG_TUD_DFU_XFER_BUFSIZE), so the bound is meaningful there. */
#define IFLASH_PROGRAM_MAX    1024u

/* Expected device flash size, in KB, read back from FLASHSIZE_BASE. */
#define IFLASH_EXPECTED_KB    512u

/** Result codes.  Negative is never returned; 0 is success. */
enum {
	IFLASH_OK = 0,
	IFLASH_ERR_UNSUPPORTED,  /* flash-size register disagrees with the map */
	IFLASH_ERR_RANGE,        /* sector / offset / length outside the app area */
	IFLASH_ERR_ALIGN,        /* offset not on a 32-byte flash-word boundary */
	IFLASH_ERR_HAL,          /* HAL reported an erase/program failure */
	IFLASH_ERR_VERIFY,       /* read-back did not match what was written */
};

/**
 * @brief  1 if the internal-flash path may be used on this device.
 *
 * Reads the flash-size register and requires IFLASH_EXPECTED_KB.  Every other
 * entry point checks this first, so a part with a different sector map refuses
 * the whole path instead of erasing something unexpected.
 */
int iflash_available(void);

/** @brief  Flash size in KB as the device reports it (diagnostics). */
uint32_t iflash_device_kb(void);

/**
 * @brief  Erase one 128 KB sector.  @p sector must be 1..3.
 *
 * Sector 0 and any non-existent sector (4-7, which the HAL's IS_FLASH_SECTOR
 * would happily accept on this 512 KB part) are rejected before the HAL is
 * called.  Blocking: on this single-bank part the CPU stalls on any fetch from
 * the internal flash until the erase completes (RM0468 sec 4.3.8) -- harmless
 * here, where the caller executes from the external XIP window.
 */
int iflash_erase_sector(uint32_t sector);

/**
 * @brief  Program @p len bytes at @p off bytes into the app partition.
 *
 * @p off must be a multiple of IFLASH_WORD_SIZE and the range must lie inside
 * the partition; @p len is capped at IFLASH_PROGRAM_MAX.  A tail shorter than a
 * flash word is padded with 0xFF.  The target must already be erased (RM0468
 * sec 4.3.9: a non-virgin word must not be re-programmed).  The written bytes are
 * read back and compared before returning, with a D-cache invalidate first when
 * the cache is on, so a caller cannot be fooled by a stale line.
 */
int iflash_program(uint32_t off, const uint8_t *data, uint32_t len);

/**
 * @brief  1 if the whole app partition range [off, off+len) reads as erased.
 *
 * Erased flash reads back as all-ones with no ECC error (RM0468 sec 4.3.10),
 * which is also why the bootloader can safely read a never-programmed vector
 * table to decide whether an app is present.
 */
int iflash_is_erased(uint32_t off, uint32_t len);

/** @brief  Read pointer to @p off in the app partition (no bounds check). */
static inline const void *iflash_ptr(uint32_t off)
{
	return (const void *)(uintptr_t)(IFLASH_APP_BASE + off);
}

/** @brief  HAL_FLASH_GetError() captured at the last IFLASH_ERR_HAL. */
uint32_t iflash_last_hal_error(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_IFLASH_H */
