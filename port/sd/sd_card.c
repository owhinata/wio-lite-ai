/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    sd_card.c
 * @brief   microSD low-level driver over SDMMC1 + internal IDMA (issue #6).
 *
 * See sd_card.h for the API contract.  Ported from the STM32F746 Discovery
 * firmware; the three things the H7 / this board changed for real are called out
 * inline below, because each of them silently breaks a straight copy.
 *
 * Hardware setup:
 *
 *   - Pins (schematic sheet 5/6, all AF12): PC8=D0, PC9=D1, PC10=D2, PC11=D3,
 *     PC12=CK, PD2=CMD.  Each line already carries a 33R series resistor and a
 *     10k pull-up on the board, and SD_3V3 comes off SYS_3V3 through a ferrite
 *     with no switch -- so there is nothing to power-sequence here.
 *
 *   - Kernel clock: SDMMC1 is muxed by RCC_D1CCIPR.SDMMCSEL between pll1_q_ck and
 *     pll2_r_ck (RM0468 sec 8.5.9 Table 56).  The bootloader leaves PLL1Q at
 *     110 MHz and PLL2R at 266 MHz; 266 MHz is over the 250 MHz the mux allows
 *     AND is the OCTOSPI1 PSRAM's clock, so PLL1Q is the only candidate.  The
 *     __HAL_RCC_SDMMC_CONFIG below writes SDMMCSEL = 0, which is the register's
 *     RESET value -- it re-states the default rather than reconfiguring anything,
 *     so it does not break the app's "never touch the clock tree" rule (CLAUDE.md).
 *     No PLL, no divider, no FLASH_ACR is written anywhere in this file.
 *
 *   - Transfer clock: SDMMC_CK = sdmmc_ker_ck / (2 * CLKDIV), CLKDIV = 0 meaning
 *     bypass (RM0468 sec 60.10.4).  CLKDIV = 3 gives 110/6 = 18.33 MHz, inside the
 *     25 MHz Default Speed ceiling.  27.5 MHz (CLKDIV = 2) would need a CMD6 switch
 *     to High Speed first, which is a separate, measured change.
 *
 *     Identification is NOT run at that divider and does not use the
 *     SDMMC_INIT_CLK_DIV constant either (that one is dead code in this HAL, and
 *     assuming otherwise gets the number wrong).  HAL_SD_InitCard derives its own:
 *     it queries HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SDMMC) and uses
 *     ClockDiv = clk / (2 * 400 kHz) + 1, i.e. 110e6/800e3 + 1 = 138 here, giving
 *     398.6 kHz -- just under the 400 kHz ceiling, by construction rather than by
 *     luck.  It restores Init.ClockDiv afterwards, which is why the transfer clock
 *     only actually takes hold when ConfigWideBusOperation re-runs SDMMC_Init.
 *
 *   - Data path: the SDMMC's OWN IDMA (RM0468 sec 60.5.6), not a DMA stream.  There
 *     is no DMA handle, no __HAL_LINKDMA and no stream ISR here -- HAL_SD_*_DMA
 *     just programs IDMABASE0/IDMACTRL and everything completes inside
 *     SDMMC1_IRQHandler.  IDMA is an AHB master, so it reaches AXI-SRAM but NOT
 *     the DTCM/ITCM, which only the core and MDMA-through-AHBS can see
 *     (RM0468 sec 2.4).  A bounce buffer in DTCM would fail silently.
 *
 * Completion model: HAL_SD runs the data phase on the IDMA and signals completion
 * asynchronously via HAL_SD_RxCpltCallback / HAL_SD_TxCpltCallback out of the
 * SDMMC1 ISR.  Both post a count-0 semaphore that the calling thread waits on with
 * a finite timeout.  A stale or late callback after an aborted/timed-out transfer
 * cannot fake success: the callback only posts while sd_xfer_active is set, and
 * every operation drains the semaphore before it starts.  HAL_SD_Abort() disables
 * the IDMA synchronously, so the abort paths below are as sound here as they were
 * with a DMA stream.
 *
 * Cache coherency: the IDMA only ever touches sd_bounce, a 32 B-aligned buffer in
 * AXI-SRAM (.axi_dma, its own linker section so no neighbouring variable shares a
 * cache line).  Reads invalidate it around the transfer and memcpy out; writes
 * memcpy in and clean it before the transfer.  The caller's buffer is never handed
 * to the IDMA, so any alignment is safe.
 *
 * Known erratum: ES0491 sec 2.9.1 "Command response and receive data end bits not
 * checked" -- the SDMMC does not flag a wrong end bit on its own.  The response and
 * the data themselves are still correct and ST documents no workaround, so nothing
 * is done about it here; it is recorded so the next reader does not re-derive it.
 *
 * Clean-room implementation; RM0468 / ES0491 / the ST BSP used as a register
 * reference only.
 */
#include "sd_card.h"

#include "stm32h7xx_hal.h"
#include "tx_api.h"
#include "tx_glue.h"        /* tx_glue_isr_enter/exit: EPK (issue #2) accounting */

#include <string.h>

#define LOG_TAG "sd"
#include "log.h"

/* Bounce buffer span: 8 sectors = 4 KiB per IDMA chunk. */
#define SD_BOUNCE_BLOCKS 8u

/* Transfer clock divider: SDMMC_CK = sdmmc_ker_ck / (2 * SD_CLOCK_DIV).
   3 -> 18.33 MHz off the inherited 110 MHz PLL1Q (see the file header). */
#define SD_CLOCK_DIV     3u

/* Wait ceilings.  State wait spins on HAL_GetTick (ms); the transfer wait uses the
   ThreadX tick (1 ms here). */
#define SD_STATE_TIMEOUT_MS    1000u
#define SD_XFER_TIMEOUT_TICKS  2000u

static SD_HandleTypeDef hsd;

static TX_MUTEX     sd_lock;        /* per-operation serialization         */
static TX_SEMAPHORE sd_done;        /* count 0; ISR posts on completion    */
static volatile int sd_xfer_err;    /* set by HAL_SD_ErrorCallback         */
static volatile int sd_xfer_active; /* 1 between issue and completion      */

static int sd_ready;                /* sd_card_init() done                 */
static int sd_probed;               /* a card is identified and answering  */
static struct sd_card_info info;

/* IDMA bounce buffer: 32 B aligned, AXI-SRAM, own linker section (see ldscript). */
static uint8_t sd_bounce[SD_BOUNCE_BLOCKS * SD_BLOCK_SIZE]
	__attribute__((aligned(32), section(".axi_dma")));

const struct sd_card_info *sd_card_get_info(void)
{
	return &info;
}

int sd_card_is_probed(void)
{
	return sd_probed;
}

/* ---- locking ------------------------------------------------------------ */

static int op_lock(void)
{
	if (!sd_ready)
		return SD_ERR_STATE;
	if (tx_mutex_get(&sd_lock, TX_WAIT_FOREVER) != TX_SUCCESS)
		return SD_ERR_STATE;
	return 0;
}

static void op_unlock(void)
{
	tx_mutex_put(&sd_lock);
}

/* ---- completion plumbing (all run under the operation mutex) ------------- */

/* Remove any leftover signals so a stale post cannot satisfy the next wait. */
static void drain_done(void)
{
	while (tx_semaphore_get(&sd_done, TX_NO_WAIT) == TX_SUCCESS)
		;
}

/*
 * Turn a just-reported HAL error into one of our codes.
 *
 * A command-response timeout is how a REMOVAL surfaces on this board (there is no
 * card-detect line), so it has to do two things a generic failure must not: report
 * SD_ERR_NO_CARD, and demote sd_probed so the next call re-identifies instead of
 * issuing commands to an empty slot.  Every path that sees a HAL error routes
 * through here -- identification, bus-width setup, both DMA submissions and the
 * completion callback -- so they cannot drift apart.
 *
 * Call it with a value read BEFORE HAL_SD_Abort(): the abort runs its own commands
 * and can overwrite hsd.ErrorCode, which would erase the evidence.
 */
static int classify_err(uint32_t err)
{
	if ((err & HAL_SD_ERROR_CMD_RSP_TIMEOUT) != 0u) {
		sd_probed = 0;
		return SD_ERR_NO_CARD;
	}
	return SD_ERR_HAL;
}

/*
 * Wait for the card to return to the data-transfer (tran) state, polling CMD13.
 *
 * THE PORTING TRAP (this board has no card-detect pin, so this loop is the only
 * place a removal can be noticed): HAL_SD_GetCardState() does NOT report a failed
 * CMD13 through its return value.  On a command timeout SD_SendStatus leaves resp1
 * at 0, so the function returns state 0 -- which is not any HAL_SD_CARD_* value and
 * in particular is not HAL_SD_CARD_ERROR.  The timeout appears ONLY as a bit OR'd
 * into hsd.ErrorCode.  So clear ErrorCode before polling and test it each round;
 * without this a yanked card spins here for the full second and is then reported as
 * a generic timeout instead of "no card".
 *
 * Clearing sd_probed on the way out is what lets the FileX glue evict a stale
 * mounted media (it re-checks sd_card_is_probed()).
 */
static int wait_transfer_state(void)
{
	uint32_t start = HAL_GetTick();

	for (;;) {
		hsd.ErrorCode = HAL_SD_ERROR_NONE;
		if (HAL_SD_GetCardState(&hsd) == HAL_SD_CARD_TRANSFER)
			return 0;
		if ((hsd.ErrorCode & HAL_SD_ERROR_CMD_RSP_TIMEOUT) != 0u) {
			sd_probed = 0;
			return SD_ERR_NO_CARD;
		}
		if ((HAL_GetTick() - start) >= SD_STATE_TIMEOUT_MS)
			return SD_ERR_TIMEOUT;
		tx_thread_sleep(1);
	}
}

/*
 * Wait for the in-flight transfer to complete.  On timeout or a reported error,
 * stop the controller (HAL_SD_Abort synchronously clears the SDMMC IT/flags and
 * disables the IDMA) and drain any signal the abort raced with.  Clears
 * sd_xfer_active first so a late callback after the abort is a no-op.
 */
static int wait_done(void)
{
	if (tx_semaphore_get(&sd_done, SD_XFER_TIMEOUT_TICKS) != TX_SUCCESS) {
		sd_xfer_active = 0;
		(void)HAL_SD_Abort(&hsd);
		drain_done();
		LOG_ERR("transfer timed out");
		return SD_ERR_TIMEOUT;
	}

	sd_xfer_active = 0;

	if (sd_xfer_err) {
		uint32_t err = HAL_SD_GetError(&hsd);   /* before the abort clobbers it */

		(void)HAL_SD_Abort(&hsd);
		drain_done();
		LOG_ERR("transfer error (HAL err 0x%lx)", (unsigned long)err);
		return classify_err(err);
	}
	return 0;
}

/* ---- public API ---------------------------------------------------------- */

int sd_card_read_blocks(uint32_t lba, void *buf, uint32_t count)
{
	uint8_t *dst = buf;
	int rc;

	if (buf == NULL || count == 0u)
		return SD_ERR_PARAM;

	rc = op_lock();
	if (rc != 0)
		return rc;
	if (!sd_probed) {
		op_unlock();
		return SD_ERR_STATE;
	}
	/* Range-check against the card before any CMD: HAL only tests
	   (lba + count) > LogBlockNbr, which wraps, so guard the wrap here. */
	if (lba >= info.block_count || count > info.block_count - lba) {
		op_unlock();
		return SD_ERR_PARAM;
	}

	while (count > 0u) {
		uint32_t c   = (count > SD_BOUNCE_BLOCKS) ? SD_BOUNCE_BLOCKS : count;
		uint32_t len = c * SD_BLOCK_SIZE;

		rc = wait_transfer_state();
		if (rc != 0)
			break;

		/* Discard any cached lines over the bounce before the IDMA writes it:
		   a dirty eviction mid-transfer would corrupt the fetched data. */
		SCB_InvalidateDCache_by_Addr(sd_bounce, (int32_t)len);

		drain_done();
		sd_xfer_err    = 0;
		sd_xfer_active = 1;
		if (HAL_SD_ReadBlocks_DMA(&hsd, sd_bounce, lba, c) != HAL_OK) {
			uint32_t err = HAL_SD_GetError(&hsd);

			sd_xfer_active = 0;
			(void)HAL_SD_Abort(&hsd);
			drain_done();
			rc = classify_err(err);
			break;
		}

		rc = wait_done();
		if (rc != 0)
			break;

		/* Drop speculative prefetches made during the transfer, then copy out. */
		SCB_InvalidateDCache_by_Addr(sd_bounce, (int32_t)len);
		memcpy(dst, sd_bounce, len);

		lba   += c;
		dst   += len;
		count -= c;
	}

	op_unlock();
	return rc;
}

int sd_card_write_blocks(uint32_t lba, const void *buf, uint32_t count)
{
	const uint8_t *src = buf;
	int rc;

	if (buf == NULL || count == 0u)
		return SD_ERR_PARAM;

	rc = op_lock();
	if (rc != 0)
		return rc;
	if (!sd_probed) {
		op_unlock();
		return SD_ERR_STATE;
	}
	if (lba >= info.block_count || count > info.block_count - lba) {
		op_unlock();
		return SD_ERR_PARAM;
	}

	while (count > 0u) {
		uint32_t c   = (count > SD_BOUNCE_BLOCKS) ? SD_BOUNCE_BLOCKS : count;
		uint32_t len = c * SD_BLOCK_SIZE;

		rc = wait_transfer_state();
		if (rc != 0)
			break;

		memcpy(sd_bounce, src, len);
		/* Flush the bounce to physical SRAM so the IDMA reads what we wrote. */
		SCB_CleanDCache_by_Addr((uint32_t *)sd_bounce, (int32_t)len);

		drain_done();
		sd_xfer_err    = 0;
		sd_xfer_active = 1;
		if (HAL_SD_WriteBlocks_DMA(&hsd, sd_bounce, lba, c) != HAL_OK) {
			uint32_t err = HAL_SD_GetError(&hsd);

			sd_xfer_active = 0;
			(void)HAL_SD_Abort(&hsd);
			drain_done();
			rc = classify_err(err);
			break;
		}

		rc = wait_done();
		if (rc != 0)
			break;

		lba   += c;
		src   += len;
		count -= c;
	}

	/* Make sure the card finished programming before reporting success, so the
	   written data is durable once this returns. */
	if (rc == 0)
		rc = wait_transfer_state();

	op_unlock();
	return rc;
}

int sd_card_probe(void)
{
	HAL_SD_CardInfoTypeDef ci;
	uint32_t widemode;
	uint32_t ker_hz;
	int i, rc;

	rc = op_lock();
	if (rc != 0)
		return rc;

	/* Re-identify from scratch every probe so a swapped card is picked up. */
	if (hsd.State != HAL_SD_STATE_RESET)
		(void)HAL_SD_DeInit(&hsd);

	hsd.Instance                 = SDMMC1;
	hsd.Init.ClockEdge           = SDMMC_CLOCK_EDGE_RISING;
	hsd.Init.ClockPowerSave      = SDMMC_CLOCK_POWER_SAVE_DISABLE;
	hsd.Init.BusWide             = SDMMC_BUS_WIDE_1B;   /* identify in 1-bit */
	/* Flow control off, matching both the f746 original and ST's own H735G-DK BSP
	   (the closest reference part).  With the IDMA draining the FIFO at AHB speed
	   against an 18 MHz bus there is no under/overrun pressure to relieve. */
	hsd.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
	hsd.Init.ClockDiv            = SD_CLOCK_DIV;

	sd_probed = 0;

	if (HAL_SD_Init(&hsd) != HAL_OK) {
		uint32_t err = HAL_SD_GetError(&hsd);
		/* An empty slot cannot be told apart from a dead card by any pin on
		   this board: nothing answers CMD0/CMD8/ACMD41, so the identification
		   ends in a response timeout, and classify_err() turns that into
		   SD_ERR_NO_CARD.  Not logged in that case -- an empty slot is a normal
		   thing for the user to ask about, not a fault worth a dmesg line. */
		rc = classify_err(err);

		if (rc != SD_ERR_NO_CARD)
			LOG_ERR("HAL_SD_Init failed (err 0x%lx)", (unsigned long)err);
		op_unlock();
		return rc;
	}

	/* HAL_SD_Init leaves SDMMC_CK at the identification divider; the transfer
	   clock (Init.ClockDiv) and the bus width only take effect when
	   ConfigWideBusOperation re-runs SDMMC_Init.  Call it on every path. */
	widemode = SDMMC_BUS_WIDE_4B;
	if (HAL_SD_ConfigWideBusOperation(&hsd, widemode) != HAL_OK) {
		/* A failed 4-bit attempt leaves hsd.ErrorCode sticky:
		   HAL_SD_ConfigWideBusOperation OR-accumulates into ErrorCode and never
		   clears it at entry, so any later call sees the old error, returns
		   HAL_ERROR and -- worse -- skips the SDMMC_Init that applies the
		   transfer clock.  Reset it before retrying 1-bit. */
		hsd.ErrorCode = HAL_SD_ERROR_NONE;
		if (HAL_SD_ConfigWideBusOperation(&hsd, SDMMC_BUS_WIDE_1B) == HAL_OK) {
			widemode = SDMMC_BUS_WIDE_1B;
			LOG_WRN("4-bit bus failed; running 1-bit");
		} else {
			uint32_t err = HAL_SD_GetError(&hsd);

			LOG_ERR("bus width config failed (err 0x%lx)",
			        (unsigned long)err);
			op_unlock();
			/* A card yanked between identification and this call times out
			   here too, so classify rather than assume a HAL fault. */
			return classify_err(err);
		}
	}

	(void)HAL_SD_GetCardInfo(&hsd, &ci);
	memset(&info, 0, sizeof info);
	info.type           = ci.CardType;
	info.version        = ci.CardVersion;
	info.card_class     = ci.Class;
	info.rca            = ci.RelCardAdd;
	info.block_count    = ci.LogBlockNbr;
	info.block_size     = ci.LogBlockSize;
	info.bus_width      = (widemode == SDMMC_BUS_WIDE_4B) ? 4u : 1u;
	info.capacity_bytes = (uint64_t)ci.LogBlockNbr * ci.LogBlockSize;
	/* Report the clock actually in force rather than a hard-coded constant, so
	   `sd info` stays honest if the inherited PLL1Q ever changes. */
	ker_hz = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SDMMC);
	info.clock_hz = ker_hz / (2u * SD_CLOCK_DIV);
	for (i = 0; i < 4; i++) {
		info.cid[i] = hsd.CID[i];
		info.csd[i] = hsd.CSD[i];
	}

	sd_probed = 1;
	LOG_INF("card up: %lu MiB, %u-bit, %lu kHz",
	        (unsigned long)(info.capacity_bytes / (1024u * 1024u)),
	        (unsigned)info.bus_width,
	        (unsigned long)(info.clock_hz / 1000u));
	op_unlock();
	return 0;
}

int sd_card_deinit(void)
{
	int rc = op_lock();
	if (rc != 0)
		return rc;
	if (hsd.State != HAL_SD_STATE_RESET)
		(void)HAL_SD_DeInit(&hsd);
	sd_probed = 0;
	op_unlock();
	return 0;
}

int sd_card_status(void)
{
	int rc = op_lock();
	if (rc != 0)
		return rc;
	if (!sd_probed) {
		op_unlock();
		return SD_ERR_STATE;
	}
	/* Same CMD13 caveat as wait_transfer_state(): the failure only shows in
	   ErrorCode, so clear it first and read it after. */
	hsd.ErrorCode = HAL_SD_ERROR_NONE;
	if (HAL_SD_GetCardState(&hsd) == HAL_SD_CARD_TRANSFER) {
		op_unlock();
		return SD_OK;
	}
	if ((hsd.ErrorCode & HAL_SD_ERROR_CMD_RSP_TIMEOUT) != 0u) {
		sd_probed = 0;
		op_unlock();
		return SD_ERR_NO_CARD;
	}
	op_unlock();
	return SD_ERR_TIMEOUT;
}

int sd_card_init(void)
{
	GPIO_InitTypeDef g = {0};

	if (sd_ready)
		return 0;

	if (tx_mutex_create(&sd_lock, "sd", TX_INHERIT) != TX_SUCCESS)
		return SD_ERR_STATE;
	if (tx_semaphore_create(&sd_done, "sd_done", 0) != TX_SUCCESS) {
		tx_mutex_delete(&sd_lock);
		return SD_ERR_STATE;
	}

	/* SDMMC1 kernel clock <- pll1_q_ck.  SDMMCSEL = 0 is the reset value, so this
	   asserts the default rather than changing the clock tree (see file header). */
	__HAL_RCC_SDMMC_CONFIG(RCC_SDMMCCLKSOURCE_PLL);
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_SDMMC1_CLK_ENABLE();

	/* PC8..PC12 = D0,D1,D2,D3,CK and PD2 = CMD, all AF12.  The board already has
	   10k pull-ups on every line; the internal ones are harmless in parallel and
	   keep the bus defined if a footprint is ever unpopulated. */
	g.Mode      = GPIO_MODE_AF_PP;
	g.Pull      = GPIO_PULLUP;
	g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
	g.Alternate = GPIO_AF12_SDMMC1;
	g.Pin       = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
	HAL_GPIO_Init(GPIOC, &g);
	g.Pin       = GPIO_PIN_2;
	HAL_GPIO_Init(GPIOD, &g);

	/* NVIC: alongside OTG_HS (6), below the RTL8720 UART (5) whose RX has a hard
	   byte deadline, above SysTick (14) / PendSV (15).  ThreadX masks with PRIMASK,
	   so tx_semaphore_put from this ISR is safe whatever the numeric priority --
	   the number only shapes latency. */
	HAL_NVIC_SetPriority(SDMMC1_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(SDMMC1_IRQn);

	sd_ready = 1;
	LOG_INF("SDMMC1 up: IDMA, CLKDIV %u, card I/O is lazy", (unsigned)SD_CLOCK_DIV);
	return 0;
}

/* ---- ISR + HAL completion callbacks -------------------------------------- */
/*
 * Strong override of the CMSIS weak vector.  One handler, not three: the H7 runs
 * the data phase on the SDMMC's own IDMA, so there are no DMA stream interrupts to
 * service (the F7 original needed DMA2_Stream3/6 handlers as well).
 *
 * tx_glue_isr_enter/exit is the EPK (issue #2) accounting bracket every plain-C ISR
 * in this firmware uses; no armed-yet gate is needed, since this only fires during
 * an SD operation, which can only start from a shell thread long after the profile
 * kit is armed.
 */
void SDMMC1_IRQHandler(void)
{
	tx_glue_isr_enter();
	HAL_SD_IRQHandler(&hsd);
	tx_glue_isr_exit();
}

/* Posted only while an operation is in flight; a late post after abort is
   suppressed by the sd_xfer_active gate and removed by the next drain. */
void HAL_SD_RxCpltCallback(SD_HandleTypeDef *h)
{
	(void)h;
	if (!sd_xfer_active)
		return;
	(void)tx_semaphore_put(&sd_done);
}

void HAL_SD_TxCpltCallback(SD_HandleTypeDef *h)
{
	(void)h;
	if (!sd_xfer_active)
		return;
	(void)tx_semaphore_put(&sd_done);
}

void HAL_SD_ErrorCallback(SD_HandleTypeDef *h)
{
	(void)h;
	if (!sd_xfer_active)
		return;
	sd_xfer_err = 1;
	(void)tx_semaphore_put(&sd_done);
}
