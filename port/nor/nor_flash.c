/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    nor_flash.c
 * @brief   External NOR flash (OCTOSPI2 / W25Q128JV) driver (issue #37).
 *
 * See nor_flash.h for the API contract and for why this driver never enters
 * memory-mapped mode.  What follows is the hardware detail.
 *
 * PINS (schematic sheet 6 "QSPI2_*" nets; measured AF values recorded in
 * boot/README.md when the bootloader still owned this device):
 *
 *     CLK PF4  AF9    NCS PG12 AF3
 *     IO4 PG0  AF9    IO5 PG1  AF9    IO6 PG10 AF3    IO7 PG11 AF9
 *
 * Note the pin *numbers*: the data lines sit on OCTOSPIM Port 2's HIGH nibble,
 * IO[7:4].  That is not just a wiring detail -- it decides a controller register
 * too.  All four are configured even though every transaction below is 1-line,
 * because "1-line SPI" means the transfer uses the FIRST line of the selected
 * nibble (PG0 out, PG1 in) while the other two still belong to the interface; and
 * selecting that nibble is what CR.FSEL does (see CR_BASE below -- getting it
 * wrong is silent, not loud).
 *
 * Each pin is configured with HAL_GPIO_Init, i.e. per-pin read-modify-write.  The
 * bootloader's deleted driver replayed the whole GPIOF and GPIOG banks instead,
 * and that whole-bank write also put the PSRAM's Port-1 pins (PF6-10, PG6) into
 * AF as a side effect -- a dependency that went unnoticed until issue #25 removed
 * the bootloader's OCTOSPI2 code and the PSRAM stopped answering.  app/psram.c
 * owns those pins now.  Never write a whole GPIO bank here.
 *
 * CLOCK: the OCTOSPI kernel clock is the pll2_r_ck the bootloader started
 * (266 MHz), shared with OCTOSPI1.  DCR2 prescaler 2 divides by 3 -> 88.7 MHz,
 * the value the shipped-firmware capture used and comfortably inside the device's
 * 133 MHz ceiling for every instruction used here.  NOTHING in this file writes
 * the RCC: only the peripheral bus-clock enables, which are RMW bit sets.
 *
 * OCTOSPIM: left at its reset configuration, which already routes OCTOSPI2 to
 * Port 2 (RM0468 sec 26.4.2/26.4.3).  Its bus clock is enabled because the pins
 * hang off it.  HAL_OSPIM_Config() is deliberately NOT used -- it clears CR.EN on
 * BOTH OCTOSPIs, which would take the PSRAM down.
 *
 * TIMING: DCR1.CSHT is set to 5 (6 kernel cycles = ~68 ns at 88.7 MHz) rather
 * than the captured 0 (1 cycle = 11 ns).  The device requires tSHSL2 >= 50 ns
 * between chip-select deassertions for erase, program and write instructions
 * (W25Q128JV AC characteristics); the capture came from a firmware that only ever
 * read, where the 10 ns tSHSL1 applies.  This driver programs and erases, so the
 * longer gap is programmed rather than left to depend on how many CPU cycles the
 * software happens to spend between two transactions.
 *
 * CACHE: every byte moves through the OCTOSPI data register by CPU load/store.
 * There is no DMA and no memory-mapped window, so no buffer is ever written
 * behind the D-cache's back and no maintenance is required for any caller
 * alignment -- unlike port/sd/sd_card.c, whose IDMA forced a bounce buffer.
 */
#include "nor_flash.h"

#include "stm32h7xx_hal.h"
#include "tx_api.h"

#include <string.h>

#define LOG_TAG "nor"
#include "log.h"

/* ------------------------------------------------------------------ *
 *  Controller configuration
 * ------------------------------------------------------------------ */

/* OCTOSPI kernel clock (pll2_r_ck, started by the bootloader; shared with the
 * OCTOSPI1 PSRAM).  Used only to report the device clock -- never programmed. */
#define NOR_KERNEL_HZ    266000000u

/* DCR2.PRESCALER: device clock = kernel / (PRESCALER + 1) -> 88.7 MHz. */
#define NOR_PRESCALER    2u

/* DCR1.DEVSIZE: number of address bits - 1.  2^24 = 16 MB. */
#define NOR_DEVSIZE      23u

/* DCR1.CSHT: chip select stays high for CSHT + 1 kernel cycles (see the header
 * comment: the device wants >= 50 ns between deassertions around a program). */
#define NOR_CSHT         5u

/* ------------------------------------------------------------------ *
 *  W25Q128JV instruction set (1-line SPI, 24-bit addresses)
 * ------------------------------------------------------------------ */
#define W25_JEDEC_ID     0x9Fu   /* -> manufacturer, memory type, capacity   */
#define W25_FAST_READ    0x0Bu   /* 8 dummy cycles; good to 133 MHz          */
#define W25_WREN         0x06u   /* sets the write-enable latch (SR1 bit 1)  */
#define W25_RDSR1        0x05u   /* bit0 = WIP (busy), bit1 = WEL            */
#define W25_PAGE_PROG    0x02u   /* <= 256 B within one page                 */
#define W25_SECTOR_ERASE 0x20u   /* 4 KB                                     */
#define W25_BLOCK_ERASE  0xD8u   /* 64 KB                                    */

#define W25_SR1_WIP      0x01u
#define W25_SR1_WEL      0x02u

#define W25_FAST_READ_DUMMY  8u

#define W25_MFR_WINBOND  0xEFu
#define W25_CAP_128M     0x18u   /* capacity byte: 2^0x18 = 16 MB            */

/* Device timing budgets in milliseconds (datasheet maximums, rounded up).  These
 * bound the status polling; typical times are ~10x shorter. */
#define NOR_PP_TIMEOUT_MS      50u     /* tPP  max 3 ms                       */
#define NOR_SECTOR_TIMEOUT_MS  600u    /* tSE  max 400 ms                     */
#define NOR_BLOCK_TIMEOUT_MS   2500u   /* tBE2 max 2000 ms                    */

/* CCR field encodings for 1-line SPI phases. */
#define CCR_IMODE_1    (1u << OCTOSPI_CCR_IMODE_Pos)
#define CCR_ADMODE_1   (1u << OCTOSPI_CCR_ADMODE_Pos)
#define CCR_ADSIZE_24  (2u << OCTOSPI_CCR_ADSIZE_Pos)
#define CCR_DMODE_1    (1u << OCTOSPI_CCR_DMODE_Pos)

/*
 * CR without FMODE: enabled, FIFO threshold 1 byte (matches the byte-at-a-time
 * FIFO loops below) -- and FSEL.
 *
 * FSEL IS NOT OPTIONAL HERE, DESPITE THERE BEING ONLY ONE FLASH.  RM0468
 * sec 25.7.1 defines it as "0: FLASH 1 selected (data exchanged over IO[3:0]) /
 * 1: FLASH 2 selected (data exchanged over IO[7:4])" -- it picks which half of
 * the controller's eight IO signals carries a single-/dual-/quad-SPI transfer,
 * not which of two chips is being talked to.  On this board the flash hangs off
 * the HIGH nibble: OCTOSPIM_P2CR's reset value 0x07050333 has IOHSRC = 0b11
 * (Port 2 IO[7:4] <- OCTOSPI2_IO[7:4], sec 26.5.2) feeding PG0/PG1/PG10/PG11, so
 * with FSEL = 0 the controller would drive the instruction on IO0 and sample the
 * reply on IO1 -- neither of which is bonded to this device.
 *
 * That is exactly the failure this driver shipped with on its first run: the JEDEC
 * id read *completed* (the controller happily clocks a transaction out to unwired
 * pins) and returned 00 00 00.  "Transaction succeeded, data is nonsense" is the
 * signature of a wrong FSEL, and it is worth recognising, because a bring-up
 * failure normally looks like a timeout instead.  The bootloader's captured
 * CR value 0x30400381 had bit 7 set for the same reason.
 */
#define CR_BASE        (OCTOSPI_CR_EN | OCTOSPI_CR_FSEL | \
                        (0u << OCTOSPI_CR_FTHRES_Pos))
#define CR_FMODE_IND_W (0u << OCTOSPI_CR_FMODE_Pos)
#define CR_FMODE_IND_R (1u << OCTOSPI_CR_FMODE_Pos)

/* Bounded spin budget for controller status flags (mirrors app/psram.c).  This
 * bounds a *controller* stall, not a device erase -- those are waited on by
 * sleeping in nor_wait_wip(). */
#define SPIN           0x00100000u

/* ------------------------------------------------------------------ *
 *  State
 * ------------------------------------------------------------------ */
static int     nor_up;                          /* device answered at bring-up */
static uint8_t nor_id[3] = {0xFFu, 0xFFu, 0xFFu};

static TX_MUTEX nor_mutex;
static int      nor_mutex_ready;

/* ------------------------------------------------------------------ *
 *  Low-level controller helpers (bounded polls, no HAL timeouts)
 * ------------------------------------------------------------------ */
static void ospi2_wait_not_busy(void)
{
	uint32_t n = SPIN;

	while ((OCTOSPI2->SR & OCTOSPI_SR_BUSY) && n) n--;
}

static int ospi2_wait_flag(uint32_t f)
{
	uint32_t n = SPIN;

	while (!(OCTOSPI2->SR & f) && n) n--;
	return (OCTOSPI2->SR & f) != 0u;
}

/* Cancel a transaction that did not complete and leave the controller parked in
 * enabled indirect mode, so one failure cannot poison every later command. */
static void ospi2_abort(void)
{
	uint32_t n = SPIN;

	OCTOSPI2->CR |= OCTOSPI_CR_ABORT;
	while ((OCTOSPI2->CR & OCTOSPI_CR_ABORT) && n) n--;
	ospi2_wait_not_busy();
	OCTOSPI2->FCR = OCTOSPI_FCR_CTCF | OCTOSPI_FCR_CTEF;
	OCTOSPI2->CR  = CR_BASE;
}

/* Instruction-only frame (WREN).  With no address and no data phase the
 * transaction starts when IR is written. */
static int ospi2_cmd(uint8_t instr)
{
	ospi2_wait_not_busy();
	OCTOSPI2->FCR = OCTOSPI_FCR_CTCF | OCTOSPI_FCR_CTEF;
	MODIFY_REG(OCTOSPI2->CR, OCTOSPI_CR_FMODE, CR_FMODE_IND_W);
	OCTOSPI2->TCR = 0u;
	OCTOSPI2->CCR = CCR_IMODE_1;
	OCTOSPI2->IR  = instr;
	if (!ospi2_wait_flag(OCTOSPI_SR_TCF)) {
		ospi2_abort();
		return NOR_ERR_IO;
	}
	OCTOSPI2->FCR = OCTOSPI_FCR_CTCF;
	return NOR_OK;
}

/* Instruction + data-in, no address (JEDEC id 0x9F, RDSR1 0x05). */
static int ospi2_read_reg(uint8_t instr, uint8_t *buf, uint32_t n)
{
	uint32_t i;

	ospi2_wait_not_busy();
	OCTOSPI2->FCR = OCTOSPI_FCR_CTCF | OCTOSPI_FCR_CTEF;
	MODIFY_REG(OCTOSPI2->CR, OCTOSPI_CR_FMODE, CR_FMODE_IND_R);
	OCTOSPI2->DLR = n - 1u;
	OCTOSPI2->TCR = 0u;
	OCTOSPI2->CCR = CCR_IMODE_1 | CCR_DMODE_1;
	OCTOSPI2->IR  = instr;
	for (i = 0u; i < n; i++) {
		if (!ospi2_wait_flag(OCTOSPI_SR_FTF)) {
			ospi2_abort();
			return NOR_ERR_IO;
		}
		buf[i] = *(volatile uint8_t *)&OCTOSPI2->DR;
	}
	if (!ospi2_wait_flag(OCTOSPI_SR_TCF)) {
		ospi2_abort();
		return NOR_ERR_IO;
	}
	OCTOSPI2->FCR = OCTOSPI_FCR_CTCF;
	return NOR_OK;
}

/* Fast Read 0x0B: instruction + 24-bit address + 8 dummy cycles + data-in.
 * Writing AR starts the transaction. */
static int ospi2_fast_read(uint32_t addr, uint8_t *buf, uint32_t n)
{
	uint32_t i;

	ospi2_wait_not_busy();
	OCTOSPI2->FCR = OCTOSPI_FCR_CTCF | OCTOSPI_FCR_CTEF;
	MODIFY_REG(OCTOSPI2->CR, OCTOSPI_CR_FMODE, CR_FMODE_IND_R);
	OCTOSPI2->DLR = n - 1u;
	OCTOSPI2->TCR = (W25_FAST_READ_DUMMY << OCTOSPI_TCR_DCYC_Pos);
	OCTOSPI2->CCR = CCR_IMODE_1 | CCR_ADMODE_1 | CCR_ADSIZE_24 | CCR_DMODE_1;
	OCTOSPI2->IR  = W25_FAST_READ;
	OCTOSPI2->AR  = addr;
	for (i = 0u; i < n; i++) {
		if (!ospi2_wait_flag(OCTOSPI_SR_FTF)) {
			ospi2_abort();
			return NOR_ERR_IO;
		}
		buf[i] = *(volatile uint8_t *)&OCTOSPI2->DR;
	}
	if (!ospi2_wait_flag(OCTOSPI_SR_TCF)) {
		ospi2_abort();
		return NOR_ERR_IO;
	}
	OCTOSPI2->FCR = OCTOSPI_FCR_CTCF;
	return NOR_OK;
}

/* Instruction + 24-bit address, no data (sector / block erase). */
static int ospi2_cmd_addr(uint8_t instr, uint32_t addr)
{
	ospi2_wait_not_busy();
	OCTOSPI2->FCR = OCTOSPI_FCR_CTCF | OCTOSPI_FCR_CTEF;
	MODIFY_REG(OCTOSPI2->CR, OCTOSPI_CR_FMODE, CR_FMODE_IND_W);
	OCTOSPI2->TCR = 0u;
	OCTOSPI2->CCR = CCR_IMODE_1 | CCR_ADMODE_1 | CCR_ADSIZE_24;
	OCTOSPI2->IR  = instr;
	OCTOSPI2->AR  = addr;
	if (!ospi2_wait_flag(OCTOSPI_SR_TCF)) {
		ospi2_abort();
		return NOR_ERR_IO;
	}
	OCTOSPI2->FCR = OCTOSPI_FCR_CTCF;
	return NOR_OK;
}

/* Page program 0x02: instruction + 24-bit address + data-out.  Unlike the two
 * above, an indirect WRITE that carries data is not started by the AR write --
 * the controller waits for the first byte pushed into DR (RM0468 sec 25.4.8) --
 * which is why the FIFO loop below leads with a threshold-flag wait. */
static int ospi2_program(uint32_t addr, const uint8_t *data, uint32_t n)
{
	uint32_t i;

	ospi2_wait_not_busy();
	OCTOSPI2->FCR = OCTOSPI_FCR_CTCF | OCTOSPI_FCR_CTEF;
	MODIFY_REG(OCTOSPI2->CR, OCTOSPI_CR_FMODE, CR_FMODE_IND_W);
	OCTOSPI2->DLR = n - 1u;
	OCTOSPI2->TCR = 0u;
	OCTOSPI2->CCR = CCR_IMODE_1 | CCR_ADMODE_1 | CCR_ADSIZE_24 | CCR_DMODE_1;
	OCTOSPI2->IR  = W25_PAGE_PROG;
	OCTOSPI2->AR  = addr;
	for (i = 0u; i < n; i++) {
		if (!ospi2_wait_flag(OCTOSPI_SR_FTF)) {
			ospi2_abort();
			return NOR_ERR_IO;
		}
		*(volatile uint8_t *)&OCTOSPI2->DR = data[i];
	}
	if (!ospi2_wait_flag(OCTOSPI_SR_TCF)) {
		ospi2_abort();
		return NOR_ERR_IO;
	}
	OCTOSPI2->FCR = OCTOSPI_FCR_CTCF;
	return NOR_OK;
}

/* ------------------------------------------------------------------ *
 *  Device-level sequencing
 * ------------------------------------------------------------------ */

/* Yield for a millisecond while the device is busy.  A thread sleeps (so the
 * IWDG petter and every other thread keep running through a 400 ms erase);
 * before the scheduler exists nothing here erases, but HAL_Delay is kept as the
 * honest fallback rather than a spin that would look like a hang. */
static void nor_sleep_ms(uint32_t ms)
{
	if (__get_IPSR() == 0u && tx_thread_identify() != TX_NULL)
		tx_thread_sleep(ms);
	else
		HAL_Delay(ms);
}

/* Poll SR1.WIP until the device finishes its internal operation. */
static int nor_wait_wip(uint32_t timeout_ms)
{
	uint32_t waited = 0u;

	for (;;) {
		uint8_t sr;
		int rc = ospi2_read_reg(W25_RDSR1, &sr, 1u);

		if (rc != NOR_OK)
			return rc;
		if (!(sr & W25_SR1_WIP))
			return NOR_OK;
		if (waited >= timeout_ms)
			return NOR_ERR_TIMEOUT;
		nor_sleep_ms(1u);
		waited++;
	}
}

/* Set the write-enable latch and verify it took.  A device that refuses WREN
 * (wedged, or held by a hardware write-protect) would otherwise swallow the
 * following erase/program silently and report success. */
static int nor_write_enable(void)
{
	uint8_t sr;
	int rc = ospi2_cmd(W25_WREN);

	if (rc != NOR_OK)
		return rc;
	rc = ospi2_read_reg(W25_RDSR1, &sr, 1u);
	if (rc != NOR_OK)
		return rc;
	return (sr & W25_SR1_WEL) ? NOR_OK : NOR_ERR_IO;
}

/* ------------------------------------------------------------------ *
 *  GPIO (per-pin RMW -- never a whole-bank write; see the file header)
 * ------------------------------------------------------------------ */
static void nor_gpio_init(void)
{
	GPIO_InitTypeDef io = {0};

	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	__HAL_RCC_OSPI2_CLK_ENABLE();
	/* The I/O manager gates the Port-2 pin routing.  Its reset value already
	 * maps OCTOSPI2 to Port 2 (RM0468 sec 26.4.3), so only its bus clock is
	 * needed -- app/psram.c enables the same bit for Port 1. */
	__HAL_RCC_OCTOSPIM_CLK_ENABLE();

	io.Mode  = GPIO_MODE_AF_PP;
	io.Pull  = GPIO_NOPULL;
	io.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

	/* Split alternate functions -- PF4/PG0/PG1/PG11 are AF9, PG10/PG12 are AF3
	 * (the values the bootloader's bank replay used to write). */
	io.Alternate = GPIO_AF9_OCTOSPIM_P2;
	io.Pin = GPIO_PIN_4;   HAL_GPIO_Init(GPIOF, &io);            /* CLK  */
	io.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_11;
	HAL_GPIO_Init(GPIOG, &io);                                   /* IO4/IO5/IO7 */
	io.Alternate = GPIO_AF3_OCTOSPIM_P2;
	io.Pin = GPIO_PIN_10 | GPIO_PIN_12;
	HAL_GPIO_Init(GPIOG, &io);                                   /* IO6, NCS */
}

/* ------------------------------------------------------------------ *
 *  Bring-up
 * ------------------------------------------------------------------ */
/*
 * A reset does not stop the flash.  If the board was reset (DFU reboot, IWDG,
 * crash) while an erase or a program was in flight, the device is still busy when
 * the app reaches this point and answers nothing but RDSR1 -- a JEDEC id read
 * issued then would come back as garbage and this driver would declare a
 * perfectly healthy device dead until the next reboot.  So wait it out first.
 *
 * The subtlety is telling "busy" from "not there": with no device answering, the
 * data line floats and every status read returns 0xFF, whose WIP bit is set --
 * indistinguishable from busy by that bit alone.  0xFF as a whole is not, though:
 * it would mean SRP0, SEC, TB, all three BP bits, WEL and WIP simultaneously, and
 * nothing on this board writes the protection bits at all.  Treat it as absence
 * and return immediately, so a board without this flash costs no boot delay while
 * a real mid-erase reset gets its full 2.5 s.
 *
 * HAL_Delay is the right wait here even though everything else in this file
 * avoids it: HAL_Init() has already started SysTick, and the IWDG is not armed
 * until tx_application_define(), so a bounded spin on the tick cannot trip it.
 */
static void nor_wait_idle_at_boot(void)
{
	uint8_t sr;

	if (ospi2_read_reg(W25_RDSR1, &sr, 1u) != NOR_OK || sr == 0xFFu)
		return;
	if (!(sr & W25_SR1_WIP))
		return;
	LOG_WRN("device busy at boot (SR1=0x%02X) -- operation in flight across a reset",
	        sr);
	(void)nor_wait_wip(NOR_BLOCK_TIMEOUT_MS);
}

int nor_flash_init(void)
{
	uint8_t id[3];

	nor_gpio_init();

	OCTOSPI2->CR   = 0u;                      /* disabled while DCRs change */
	OCTOSPI2->DCR1 = (NOR_DEVSIZE << OCTOSPI_DCR1_DEVSIZE_Pos)
	               | (NOR_CSHT << OCTOSPI_DCR1_CSHT_Pos)
	               | OCTOSPI_DCR1_DLYBYP;     /* SDR 1-line: no delay block */
	OCTOSPI2->DCR2 = (NOR_PRESCALER << OCTOSPI_DCR2_PRESCALER_Pos);
	OCTOSPI2->DCR3 = 0u;
	OCTOSPI2->DCR4 = 0u;
	OCTOSPI2->CR   = CR_BASE;                 /* enabled, indirect */

	nor_wait_idle_at_boot();

	if (ospi2_read_reg(W25_JEDEC_ID, id, 3u) != NOR_OK) {
		LOG_ERR("JEDEC id read timed out -- OCTOSPI2 down");
		nor_up = 0;
		return NOR_ERR_STATE;
	}
	nor_id[0] = id[0];
	nor_id[1] = id[1];
	nor_id[2] = id[2];

	if (id[0] != W25_MFR_WINBOND || id[2] != W25_CAP_128M) {
		LOG_ERR("unexpected JEDEC id %02X %02X %02X", id[0], id[1], id[2]);
		nor_up = 0;
		return NOR_ERR_STATE;
	}

	nor_up = 1;
	LOG_INF("W25Q128 up: id %02X %02X %02X, %lu Hz", id[0], id[1], id[2],
	        (unsigned long)nor_flash_clock_hz());
	return NOR_OK;
}

int nor_flash_ready(void) { return nor_up; }

void nor_flash_get_id(uint8_t jedec[3])
{
	jedec[0] = nor_id[0];
	jedec[1] = nor_id[1];
	jedec[2] = nor_id[2];
}

uint32_t nor_flash_clock_hz(void)
{
	return NOR_KERNEL_HZ / (NOR_PRESCALER + 1u);
}

void nor_flash_get_regs(uint32_t regs[4])
{
	regs[0] = OCTOSPI2->CR;
	regs[1] = OCTOSPI2->DCR1;
	regs[2] = OCTOSPI2->DCR2;
	regs[3] = OCTOSPI2->SR;
}

/* ------------------------------------------------------------------ *
 *  Locking
 * ------------------------------------------------------------------ */
int nor_flash_lock_init(void)
{
	if (nor_mutex_ready)
		return NOR_OK;
	if (tx_mutex_create(&nor_mutex, "nor", TX_INHERIT) != TX_SUCCESS)
		return NOR_ERR_STATE;
	nor_mutex_ready = 1;
	return NOR_OK;
}

int nor_lock_blocking(void)
{
	UINT rc;

	/* Before the mutex exists only nor_flash_init() runs, single-threaded and
	 * pre-scheduler; there is nothing to serialize against. */
	if (!nor_mutex_ready)
		return NOR_OK;
	if (__get_IPSR() != 0u)
		return NOR_ERR_STATE;            /* thread context only */
	rc = tx_mutex_get(&nor_mutex, TX_WAIT_FOREVER);
	if (rc != TX_SUCCESS) {
		/* An infinite wait can only fail if the object or the caller is
		 * invalid.  Say so loudly: the caller may be FlashDB's lock
		 * callback, which cannot see this return value and would go on to
		 * touch the device unlocked. */
		LOG_ERR("mutex get failed (0x%02X)", (unsigned)rc);
		return NOR_ERR_STATE;
	}
	return NOR_OK;
}

int nor_trylock(uint32_t timeout_ticks)
{
	UINT rc;

	if (!nor_mutex_ready)
		return NOR_OK;
	if (__get_IPSR() != 0u)
		return NOR_ERR_STATE;
	rc = tx_mutex_get(&nor_mutex, timeout_ticks);
	if (rc == TX_SUCCESS)
		return NOR_OK;
	return (rc == TX_NOT_AVAILABLE) ? NOR_ERR_BUSY : NOR_ERR_STATE;
}

void nor_unlock(void)
{
	UINT rc;

	if (!nor_mutex_ready)
		return;
	rc = tx_mutex_put(&nor_mutex);
	/* Both failure modes are silent corruption if ignored: too few puts leave
	 * the ownership count set and every other thread waits forever, too many
	 * (or a put from a non-owner) returns TX_NOT_OWNED and means the lock
	 * discipline above it is broken. */
	if (rc != TX_SUCCESS)
		LOG_ERR("mutex put failed (0x%02X)", (unsigned)rc);
}

/* ------------------------------------------------------------------ *
 *  Public data-path API
 * ------------------------------------------------------------------ */

/* Shared entry gate: refuse when the device never came up, then take the
 * (recursive) device mutex.  Every public operation pairs this with nor_unlock. */
static int nor_op_begin(void)
{
	if (!nor_up)
		return NOR_ERR_STATE;
	return nor_lock_blocking();
}

int nor_read(uint32_t addr, void *buf, uint32_t len)
{
	int rc;

	if (buf == NULL || len == 0u || addr >= NOR_SIZE_BYTES ||
	    len > NOR_SIZE_BYTES - addr)
		return NOR_ERR_PARAM;

	rc = nor_op_begin();
	if (rc != NOR_OK)
		return rc;
	rc = ospi2_fast_read(addr, (uint8_t *)buf, len);
	nor_unlock();
	return rc;
}

int nor_write(uint32_t addr, const void *buf, uint32_t len)
{
	const uint8_t *src = (const uint8_t *)buf;
	uint32_t done = 0u;
	int rc;

	if (buf == NULL || len == 0u || addr >= NOR_SIZE_BYTES ||
	    len > NOR_SIZE_BYTES - addr)
		return NOR_ERR_PARAM;

	rc = nor_op_begin();
	if (rc != NOR_OK)
		return rc;

	while (done < len) {
		/* Never let a program cross a page boundary: the device wraps
		 * within the page instead of advancing, which would silently
		 * overwrite the start of the same page. */
		uint32_t page_off = (addr + done) & (NOR_PAGE_SIZE - 1u);
		uint32_t chunk = NOR_PAGE_SIZE - page_off;

		if (chunk > len - done)
			chunk = len - done;

		rc = nor_write_enable();
		if (rc == NOR_OK)
			rc = ospi2_program(addr + done, src + done, chunk);
		if (rc == NOR_OK)
			rc = nor_wait_wip(NOR_PP_TIMEOUT_MS);
		if (rc != NOR_OK) {
			LOG_ERR("program failed at 0x%06lX (%d)",
			        (unsigned long)(addr + done), rc);
			break;
		}
		done += chunk;
	}

	nor_unlock();
	return rc;
}

int nor_erase(uint32_t addr, uint32_t len)
{
	uint32_t off = addr;
	uint32_t end;
	int rc;

	if (len == 0u || addr >= NOR_SIZE_BYTES || len > NOR_SIZE_BYTES - addr)
		return NOR_ERR_PARAM;
	if ((addr % NOR_SECTOR_SIZE) != 0u || (len % NOR_SECTOR_SIZE) != 0u)
		return NOR_ERR_PARAM;
	end = addr + len;

	rc = nor_op_begin();
	if (rc != NOR_OK)
		return rc;

	while (off < end) {
		/* A 64 KB block erase costs ~150 ms typical against ~45 ms for each
		 * of the sixteen 4 KB sectors it replaces, so take it whenever the
		 * remaining range is block-aligned and long enough.  A whole 1 MB
		 * partition erase drops from ~11 s to ~2.4 s. */
		int block = ((off % NOR_BLOCK_SIZE) == 0u) &&
		            (end - off >= NOR_BLOCK_SIZE);
		uint32_t step = block ? NOR_BLOCK_SIZE : NOR_SECTOR_SIZE;

		rc = nor_write_enable();
		if (rc == NOR_OK)
			rc = ospi2_cmd_addr(block ? W25_BLOCK_ERASE
			                          : W25_SECTOR_ERASE, off);
		if (rc == NOR_OK)
			rc = nor_wait_wip(block ? NOR_BLOCK_TIMEOUT_MS
			                        : NOR_SECTOR_TIMEOUT_MS);
		if (rc != NOR_OK) {
			LOG_ERR("erase failed at 0x%06lX (%d)",
			        (unsigned long)off, rc);
			break;
		}
		off += step;
	}

	nor_unlock();
	return rc;
}
