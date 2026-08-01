/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    cmd_iflash.c
 * @brief   `iflash` shell command (issue #25): internal-flash driver validation.
 *
 *   iflash info            sector map, device flash size, erased/programmed state
 *   iflash erase <sector>  erase one app sector (1-3)
 *   iflash test [sector]   erase -> blank check -> program -> verify, with timings
 *
 * WHY THIS EXISTS (and why it is temporary): issue #25 moves the app from XIP out
 * of the external OCTOSPI2 window to execution from the internal flash at
 * 0x08020000, which means the DFU bootloader has to grow an internal-flash
 * programming path.  The bootloader cannot be debugged the way an app can -- a bad
 * image at 0x08000000 is how board #1 died -- so the driver is proven HERE first,
 * while the app still runs XIP from the external flash.  In that configuration the
 * instruction fetch comes from OCTOSPI2 and is entirely unaffected by the internal
 * bank being busy, and the erase/program path physically cannot reach sector 0.
 *
 * `iflash test` is also what pins down the DFU bwPollTimeout: the host has to be
 * told, before each erase, how long to wait, and a 128 KB sector erase is the one
 * operation long enough to matter.  The reported milliseconds are that number.
 *
 * !! DELETE THIS COMMAND when the app itself moves to 0x08020000 -- at that point
 * `iflash erase` would be erasing the code it is running from.
 *
 * Clean-room design; no third-party code reused.
 */
#include "cli.h"
#include "iflash.h"
#include "timebase.h"

#include "stm32h7xx_hal.h"   /* HAL_GetTick, __LDREXB / __STREXB */

/* Bytes written by `iflash test`: 32 blocks of the DFU transfer size.  Enough for
 * a per-block timing average without spending a minute on a full sector. */
#define TEST_BLOCKS   32u
#define TEST_BLOCK    IFLASH_PROGRAM_MAX

/*
 * Serialise against a second shell command (`cmd &` background workers): while a
 * sector is erasing, ANY read of the internal flash stalls until it completes
 * (RM0468 sec 4.3.8), so a concurrent `devmem 0x08000000` or `membench` would
 * block for the whole erase and look like a hang.  Same LDREX/STREX claim as
 * psram_acquire().
 */
static volatile uint8_t iflash_busy;

static int iflash_acquire(void)
{
	do {
		if (__LDREXB(&iflash_busy) != 0u) {
			__CLREX();
			return 0;
		}
	} while (__STREXB(1u, &iflash_busy) != 0u);
	__DMB();
	return 1;
}

static void iflash_release(void)
{
	__DMB();
	iflash_busy = 0u;
}

static const char *iflash_strerror(int rc)
{
	switch (rc) {
	case IFLASH_OK:              return "ok";
	case IFLASH_ERR_UNSUPPORTED: return "unsupported device (flash size)";
	case IFLASH_ERR_RANGE:       return "out of range";
	case IFLASH_ERR_ALIGN:       return "misaligned";
	case IFLASH_ERR_HAL:         return "HAL erase/program error";
	case IFLASH_ERR_VERIFY:      return "read-back mismatch";
	default:                     return "?";
	}
}

/* Deterministic, offset-dependent test pattern: a wrong-offset write shows up as
 * a mismatch instead of matching by luck the way a constant fill would. */
static uint8_t test_byte(uint32_t off)
{
	return (uint8_t)((off * 31u) ^ (off >> 8) ^ 0xA5u);
}

static int cmd_iflash_info(struct cli_instance *sh, int argc, char **argv)
{
	uint32_t s;

	(void)argc;
	(void)argv;

	cli_print(sh, "device flash: %lu KB (%s)\r\n",
	          (unsigned long)iflash_device_kb(),
	          iflash_available() ? "supported" : "UNSUPPORTED -- path disabled");
	cli_print(sh, "app partition: 0x%08lx + %lu KB (sectors %u-%u)\r\n",
	          (unsigned long)IFLASH_APP_BASE,
	          (unsigned long)(IFLASH_APP_SIZE / 1024u),
	          (unsigned)IFLASH_APP_SECTOR_LO, (unsigned)IFLASH_APP_SECTOR_HI);
	cli_print(sh, "sector 0:      0x%08lx + 128 KB  DFU bootloader (never written)\r\n",
	          (unsigned long)IFLASH_BASE_ADDR);

	for (s = IFLASH_APP_SECTOR_LO; s <= IFLASH_APP_SECTOR_HI; s++) {
		uint32_t off = (s - IFLASH_APP_SECTOR_LO) * IFLASH_SECTOR_SIZE;
		const volatile uint32_t *v =
			(const volatile uint32_t *)(IFLASH_APP_BASE + off);

		cli_print(sh, "sector %lu:      0x%08lx + 128 KB  %s  word[0]=%08lx\r\n",
		          (unsigned long)s,
		          (unsigned long)(IFLASH_BASE_ADDR + s * IFLASH_SECTOR_SIZE),
		          iflash_is_erased(off, IFLASH_SECTOR_SIZE) ? "erased    "
		                                                    : "programmed",
		          (unsigned long)v[0]);
	}
	return 0;
}

#if CLI_ENABLE_DANGEROUS_CMDS

static int parse_sector(struct cli_instance *sh, const char *s, uint32_t *out)
{
	if (cli_parse_u32(s, out) != 0) {
		cli_error(sh, "iflash: bad sector '%s'\r\n", s);
		return -1;
	}
	if (*out < IFLASH_APP_SECTOR_LO || *out > IFLASH_APP_SECTOR_HI) {
		cli_error(sh, "iflash: sector %lu out of range (%u-%u; sector 0 is the "
		              "bootloader)\r\n", (unsigned long)*out,
		          (unsigned)IFLASH_APP_SECTOR_LO, (unsigned)IFLASH_APP_SECTOR_HI);
		return -1;
	}
	return 0;
}

static int cmd_iflash_erase(struct cli_instance *sh, int argc, char **argv)
{
	uint32_t sector, t0, ms;
	int rc;

	(void)argc;
	if (parse_sector(sh, argv[1], &sector) != 0)
		return -1;
	if (!iflash_acquire()) {
		cli_error(sh, "iflash: busy\r\n");
		return -1;
	}

	/* Milliseconds, not DWT cycles: a sector erase is ~1 s, well past the
	 * ~7.8 s CYCCNT wrap only in the aggregate, but HAL_GetTick keeps running
	 * here (the CPU executes from the external flash, so it does not stall)
	 * and needs no wrap reasoning at all. */
	t0 = HAL_GetTick();
	rc = iflash_erase_sector(sector);
	ms = HAL_GetTick() - t0;
	iflash_release();

	if (rc != IFLASH_OK) {
		cli_error(sh, "iflash: erase sector %lu failed: %s (hal err 0x%08lx)\r\n",
		          (unsigned long)sector, iflash_strerror(rc),
		          (unsigned long)iflash_last_hal_error());
		return -1;
	}
	cli_print(sh, "erased sector %lu in %lu ms\r\n",
	          (unsigned long)sector, (unsigned long)ms);
	return 0;
}

static int cmd_iflash_test(struct cli_instance *sh, int argc, char **argv)
{
	static uint8_t buf[TEST_BLOCK];
	uint32_t sector = IFLASH_APP_SECTOR_HI;   /* default: the last sector */
	uint32_t base, t0, erase_ms, prog_cyc, i, b;
	int rc;

	if (argc > 1 && parse_sector(sh, argv[1], &sector) != 0)
		return -1;
	if (!iflash_available()) {
		cli_error(sh, "iflash: %s\r\n", iflash_strerror(IFLASH_ERR_UNSUPPORTED));
		return -1;
	}
	if (!iflash_acquire()) {
		cli_error(sh, "iflash: busy\r\n");
		return -1;
	}

	base = (sector - IFLASH_APP_SECTOR_LO) * IFLASH_SECTOR_SIZE;
	cli_print(sh, "testing sector %lu (0x%08lx), %lu blocks of %lu B\r\n",
	          (unsigned long)sector,
	          (unsigned long)(IFLASH_APP_BASE + base),
	          (unsigned long)TEST_BLOCKS, (unsigned long)TEST_BLOCK);

	t0 = HAL_GetTick();
	rc = iflash_erase_sector(sector);
	erase_ms = HAL_GetTick() - t0;
	if (rc != IFLASH_OK)
		goto fail;
	cli_print(sh, "  erase       : %lu ms   <- DFU bwPollTimeout is sized from this\r\n",
	          (unsigned long)erase_ms);

	if (!iflash_is_erased(base, IFLASH_SECTOR_SIZE)) {
		cli_error(sh, "  blank check : FAILED (sector not all-ones after erase)\r\n");
		iflash_release();
		return -1;
	}
	cli_print(sh, "  blank check : ok (128 KB all-ones, no ECC fault)\r\n");

	/* Program, timing the whole burst on the cycle counter.  Each block is
	 * verified inside iflash_program(); the pass below re-reads everything
	 * afterwards so a later block cannot have disturbed an earlier one. */
	t0 = DWT->CYCCNT;
	for (b = 0u; b < TEST_BLOCKS; b++) {
		uint32_t off = base + b * TEST_BLOCK;

		for (i = 0u; i < TEST_BLOCK; i++)
			buf[i] = test_byte(off + i);

		rc = iflash_program(off, buf, TEST_BLOCK);
		if (rc != IFLASH_OK)
			goto fail;
	}
	prog_cyc = DWT->CYCCNT - t0;
	cli_print(sh, "  program     : %lu KB in %lu us (%lu us per %lu B block)\r\n",
	          (unsigned long)((TEST_BLOCKS * TEST_BLOCK) / 1024u),
	          (unsigned long)(prog_cyc / (SystemCoreClock / 1000000u)),
	          (unsigned long)(prog_cyc / (SystemCoreClock / 1000000u) / TEST_BLOCKS),
	          (unsigned long)TEST_BLOCK);

	for (b = 0u; b < TEST_BLOCKS; b++) {
		uint32_t off = base + b * TEST_BLOCK;
		const volatile uint8_t *p =
			(const volatile uint8_t *)(IFLASH_APP_BASE + off);

		for (i = 0u; i < TEST_BLOCK; i++) {
			if (p[i] != test_byte(off + i)) {
				cli_error(sh, "  verify      : FAILED at 0x%08lx "
				              "(got %02x want %02x)\r\n",
				          (unsigned long)(IFLASH_APP_BASE + off + i),
				          (unsigned)p[i], (unsigned)test_byte(off + i));
				iflash_release();
				return -1;
			}
		}
	}
	cli_print(sh, "  verify      : ok (%lu KB re-read)\r\n",
	          (unsigned long)((TEST_BLOCKS * TEST_BLOCK) / 1024u));

	/* Leave the sector erased so `iflash info` reads clean and nothing is left
	 * that could be mistaken for an app image. */
	rc = iflash_erase_sector(sector);
	if (rc != IFLASH_OK)
		goto fail;

	iflash_release();
	cli_print(sh, "PASS (sector %lu left erased)\r\n", (unsigned long)sector);
	return 0;

fail:
	iflash_release();
	cli_error(sh, "iflash: %s (hal err 0x%08lx)\r\n", iflash_strerror(rc),
	          (unsigned long)iflash_last_hal_error());
	return -1;
}

#endif /* CLI_ENABLE_DANGEROUS_CMDS */

CLI_SUBCMD_SET_CREATE(iflash_subcmds,
	CLI_CMD(info, NULL, "sector map and erased/programmed state", cmd_iflash_info),
#if CLI_ENABLE_DANGEROUS_CMDS
	CLI_CMD_ARG(erase, NULL, "erase one app sector (1-3)", cmd_iflash_erase, 2, 0),
	CLI_CMD_ARG(test, NULL, "erase/program/verify a sector with timings",
	            cmd_iflash_test, 1, 1),
#endif
	CLI_SUBCMD_SET_END);

CLI_CMD_REGISTER(iflash, iflash_subcmds,
                 "internal flash (sectors 1-3) driver validation (issue #25)",
                 NULL, 1, 0);
