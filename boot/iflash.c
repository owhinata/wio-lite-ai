/*
 * Internal-flash (sector 1-3) erase/program driver for the Wio Lite AI standalone
 * DFU bootloader -- issue #25.
 *
 * See iflash.h for the memory map, the safety argument and the reference-manual
 * citations.  Two properties are worth restating where the code lives:
 *
 *   1. SECTOR 0 IS UNREACHABLE BY CONSTRUCTION.  The only erase entry point
 *      rejects anything outside 1..3 *before* calling the HAL, and program
 *      offsets are relative to IFLASH_APP_BASE (sector 1) with an
 *      overflow-safe upper bound -- there is no argument value that turns into
 *      an address below 0x08020000.
 *
 *   2. NO OPTION BYTES.  Only HAL_FLASH_Unlock()/Lock() (FLASH_KEYR + CR) and
 *      the sector-erase / flash-word-program paths are used.  The option-byte
 *      unlock (HAL_FLASH_OB_Unlock, FLASH_OPTKEYR) and OPTCR are never touched,
 *      so RDP / BOOT / SWD configuration cannot be altered by this code -- the
 *      objdump audit before flashing the bootloader checks exactly this.
 */
#include "iflash.h"

#include "stm32h7xx_hal.h"

static uint32_t iflash_hal_err;
static uint32_t iflash_erase_cycles;

/*
 * Start the DWT cycle counter, the only clock that survives an erase: the core
 * stalls on every instruction fetch from the flash while one is running, so
 * SysTick does not tick, but DWT->CYCCNT is hardware and keeps counting.  Only
 * CoreDebug->DEMCR and DWT are written -- these are ARM core registers, not the
 * STM32 DBGMCU peripheral that the project rules forbid touching.
 */
static void iflash_cyccnt_init(void)
{
	if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0u)
		return;
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0u;
	DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t iflash_last_erase_cycles(void)
{
	return iflash_erase_cycles;
}

uint32_t iflash_device_kb(void)
{
	return (uint32_t)(*(const volatile uint16_t *)FLASHSIZE_BASE);
}

int iflash_available(void)
{
	return iflash_device_kb() == IFLASH_EXPECTED_KB;
}

uint32_t iflash_last_hal_error(void)
{
	return iflash_hal_err;
}

/*
 * Drop any cached copy of [addr, addr+len) before reading flash back.  The
 * internal flash lives in the ARMv7-M default Code region, which is Normal
 * cacheable, so a read-back after a program could otherwise be served from a
 * line loaded before the write.  The maintenance range is widened to whole
 * 32-byte cache lines because the by-address operations work on lines.
 *
 * Guarded on the D-cache enable bit rather than a build flag, so the same file
 * is correct in the bootloader, which runs with both caches off.
 */
static void iflash_invalidate(uint32_t addr, uint32_t len)
{
	uint32_t start, end;

	if ((SCB->CCR & SCB_CCR_DC_Msk) == 0u)
		return;

	start = addr & ~31u;
	end   = (addr + len + 31u) & ~31u;
	SCB_InvalidateDCache_by_Addr((uint32_t *)start, (int32_t)(end - start));
}

int iflash_erase_sector(uint32_t sector)
{
	FLASH_EraseInitTypeDef erase = {0};
	uint32_t bad_sector = 0xFFFFFFFFu;
	uint32_t t0;
	HAL_StatusTypeDef st;

	if (!iflash_available())
		return IFLASH_ERR_UNSUPPORTED;

	/* The gate that keeps sector 0 (the bootloader) out of reach.  Note the
	 * HAL's own IS_FLASH_SECTOR only checks against FLASH_SECTOR_TOTAL, which
	 * CMSIS defines as 8 for the 1 MB family member -- it would accept sectors
	 * 4-7 that do not exist on this 512 KB part. */
	if (sector < IFLASH_APP_SECTOR_LO || sector > IFLASH_APP_SECTOR_HI)
		return IFLASH_ERR_RANGE;

	erase.TypeErase    = FLASH_TYPEERASE_SECTORS;
	erase.Banks        = FLASH_BANK_1;          /* single-bank part */
	erase.Sector       = sector;
	erase.NbSectors    = 1u;
	erase.VoltageRange = FLASH_VOLTAGE_RANGE_4; /* 64-bit parallelism at 3.3 V */

	iflash_cyccnt_init();

	if (HAL_FLASH_Unlock() != HAL_OK) {
		iflash_hal_err = HAL_FLASH_GetError();
		return IFLASH_ERR_HAL;
	}
	t0 = DWT->CYCCNT;
	st = HAL_FLASHEx_Erase(&erase, &bad_sector);
	iflash_erase_cycles = DWT->CYCCNT - t0;
	(void)HAL_FLASH_Lock();

	if (st != HAL_OK) {
		iflash_hal_err = HAL_FLASH_GetError();
		return IFLASH_ERR_HAL;
	}

	iflash_invalidate(IFLASH_BASE_ADDR + sector * IFLASH_SECTOR_SIZE,
	                  IFLASH_SECTOR_SIZE);
	return IFLASH_OK;
}

int iflash_program(uint32_t off, const uint8_t *data, uint32_t len)
{
	uint32_t done;
	int rc = IFLASH_OK;

	if (!iflash_available())
		return IFLASH_ERR_UNSUPPORTED;
	if (data == (const uint8_t *)0)
		return IFLASH_ERR_RANGE;
	if (len == 0u)
		return IFLASH_OK;

	if ((off % IFLASH_WORD_SIZE) != 0u)
		return IFLASH_ERR_ALIGN;
	/* Bound written so neither term can wrap: off is checked first, then the
	 * length against the space that is left. */
	if (len > IFLASH_PROGRAM_MAX || off > IFLASH_APP_SIZE ||
	    len > IFLASH_APP_SIZE - off)
		return IFLASH_ERR_RANGE;

	if (HAL_FLASH_Unlock() != HAL_OK) {
		iflash_hal_err = HAL_FLASH_GetError();
		return IFLASH_ERR_HAL;
	}

	for (done = 0u; done < len; done += IFLASH_WORD_SIZE) {
		/* One 256-bit flash word, 32-bit aligned as HAL_FLASH_Program
		 * requires.  A tail shorter than a word is padded with 0xFF: the
		 * word is programmed once and stays consistent with its ECC. */
		uint32_t word[IFLASH_WORD_SIZE / 4];
		uint8_t *wb = (uint8_t *)word;
		uint32_t n  = len - done;
		uint32_t i;

		if (n > IFLASH_WORD_SIZE)
			n = IFLASH_WORD_SIZE;

		for (i = 0u; i < n; i++)
			wb[i] = data[done + i];
		for (; i < IFLASH_WORD_SIZE; i++)
			wb[i] = 0xFFu;

		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
		                      IFLASH_APP_BASE + off + done,
		                      (uint32_t)word) != HAL_OK) {
			iflash_hal_err = HAL_FLASH_GetError();
			rc = IFLASH_ERR_HAL;
			break;
		}
	}

	(void)HAL_FLASH_Lock();
	if (rc != IFLASH_OK)
		return rc;

	/* Read back exactly what was asked for (the 0xFF padding is not the
	 * caller's data, so it is not compared). */
	iflash_invalidate(IFLASH_APP_BASE + off, len);
	{
		const volatile uint8_t *src =
			(const volatile uint8_t *)(IFLASH_APP_BASE + off);
		uint32_t i;

		for (i = 0u; i < len; i++) {
			if (src[i] != data[i])
				return IFLASH_ERR_VERIFY;
		}
	}
	return IFLASH_OK;
}

int iflash_is_erased(uint32_t off, uint32_t len)
{
	const volatile uint32_t *p;
	uint32_t i;

	if (off > IFLASH_APP_SIZE || len > IFLASH_APP_SIZE - off)
		return 0;
	if (len == 0u)
		return 1;

	iflash_invalidate(IFLASH_APP_BASE + off, len);
	p = (const volatile uint32_t *)(IFLASH_APP_BASE + (off & ~3u));

	for (i = 0u; i < ((len + 3u) / 4u); i++) {
		if (p[i] != 0xFFFFFFFFu)
			return 0;
	}
	return 1;
}
