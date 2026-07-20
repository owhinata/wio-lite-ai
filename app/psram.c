/*
 * Wio Lite AI (STM32H725AEI6) -- OCTOSPI1 APS6408L-3OBM-BA Octal DDR PSRAM
 * app-side bring-up (issue #3).  Indirect operations are bounded and fail-soft
 * so a missing PSRAM response does not stop USB.  Validated on board #2 at
 * 53.2 MHz, full-8MB test pass (bring-up story: issue #3 comments).
 *
 * WHY THIS IS SAFE TO RUN FROM XIP: OCTOSPI1 and OCTOSPI2 sit on separate
 * OCTOSPIM ports (P1 vs P2, un-muxed) and separate register blocks, sharing only
 * the already-running PLL2R 266 MHz kernel clock.  This file writes ONLY OCTOSPI1,
 * GPIO B/D/E and the OCTOSPI1 delay block -- never OCTOSPI2, OCTOSPIM
 * (CR/P1CR/P2CR), or the RCC -- so it never disturbs the code memory the CPU is
 * fetching.  HAL_OSPIM_Config() is deliberately NOT used: it clears CR.EN on BOTH
 * OCTOSPIs, which would kill the XIP fetch.  OCTOSPIM is left fully at reset --
 * its P1CR reset value (0x03010111) already enables the octal IO[7:0] + DQS + CLK
 * + NCS on Port 1 (RM0468 sec 26.5.2).
 *
 * PIN MAP (schematic sheet 5/6 "OSPI1_*" nets; AF from the H735-DK BSP, same
 * silicon/pins, cross-checked vs the STM32H725 datasheet Table 8/9):
 *   IO0..3 PF8/PF9/PF7/PF6 AF10, CLK PF10 AF9, NCS PG6 AF10  -- already in AF from
 *          the bootloader (boot/octospi.c); this file leaves them alone.
 *   IO4 PD4, IO5 PD5, IO7 PD7, DQS PB2 -- all AF10 (H735-DK confirmed).
 *   IO6 PE9 AF10.  NB: IO7 is PD7, NOT PD6 (PD6 = LCD_B2, a different net).
 *   RESET# is not connected: schematic R13 (0 ohm) is DNP.  Initialization must
 *          therefore use the device's Global Reset command.
 *
 * REFERENCES: RM0468 secs 25.4.14/.16 (mmap read WCCR/write), 25.7.2 (DCR1
 * MTYP/DEVSIZE/DLYBYP/FRCK), 25.7.5 (DCR4 refresh), 26.5.2 (OCTOSPIM).  Opcodes /
 * MR encodings from stm32-aps6408.  Clean-room; mirrors boot/octospi.c's idiom.
 */
#include "stm32h7xx_hal.h"
#include "psram.h"

/* ------------------------------------------------------------------ *
 *  Tunable bring-up parameters (validate / adjust on board #2)
 * ------------------------------------------------------------------ */

/* Device clock = OSPI kernel (PLL2R 266 MHz) / (PRESCALER+1).  Operating
 * point validated on board #2: 53.2 MHz, full-8MB test pass with rd=5/wr=4
 * and DLYB phase3/unit64.  The next steps up were rejected on measurement:
 * 66.5 MHz shows single-bit read errors (0x90000000 -> 0x94000000) and at
 * 88.7 MHz register reads and linear reads need DIFFERENT DLYB settings
 * (margin collapse).  26.6 MHz (presc 9) is the known-good fallback. */
#define PSRAM_KERNEL_HZ    266000000u
#define PSRAM_PRESCALER    4u              /* DCR2.PRESCALER -> /5 = 53.2 MHz */

/* Keep the APS6408 power-up configuration.  The earlier mode-register writes
 * were diagnosed through invalid one-byte DTR reads and changed the wrong
 * register pair.  MR0 reset value is variable LC5 with 1/4 drive strength. */
#define PSRAM_MR0_RESET    0x09u

/* Read DCYC = LC = 5, NOT the ST driver's LatencyCode-1 convention: board #2
 * pscan (26.6 MHz, cold boot) showed DQS-gated reads return ZERO bytes (BUSY
 * wedge) at DCYC=4 and clean stable data at DCYC=5..10 -- the LC-1 value made
 * the DQS capture window open one clock early and miss the burst entirely.
 * Write latency has no DQS feedback; WLC5-1=4 kept until psram_wtune says
 * otherwise. */
#define PSRAM_READ_DCYC    5u
#define PSRAM_WRITE_DCYC   4u

/* Split mmap transactions every 32 bytes.  This stays below tCEM and prevents
 * the device's mandatory 1 KB row wrap from aliasing linear CPU accesses. */
#define PSRAM_CSBOUND      5u

/* Limit transaction duration so the RAM can perform internal refresh.  Zero
 * disables this feature; any nonzero value must exceed the complete minimum
 * command/address/dummy/data transaction. */
#define PSRAM_REFRESH      320u

/* AP Memory IO6 alternate function. */
#define PSRAM_IO6_AF       GPIO_AF10_OCTOSPIM_P1

/* Validated read-sampling point at 26.6 AND 53.2 MHz (full 8MB pass).  NB:
 * `psram eye` consistently SCORES unit 8 as best, but the unit-8 state it
 * then applies breaks subsequent mmap traffic at the same clock (in-scan vs
 * post-scan discrepancy, unexplained -- see PSRAM_DEBUG_HANDOVER.md).  Do
 * not replace this point from an eye run without a full `psram test` pass. */
#define PSRAM_DLYB_SEL     3u
#define PSRAM_DLYB_UNIT    64u

/* ------------------------------------------------------------------ *
 *  APS6408 opcodes + OCTOSPI field shorthands
 * ------------------------------------------------------------------ */
#define APS6408_READ_CMD         0x00u   /* synchronous read          */
#define APS6408_READ_LINEAR_CMD  0x20u   /* linear burst read         */
#define APS6408_WRITE_CMD        0x80u   /* synchronous write         */
#define APS6408_WRITE_LINEAR_CMD 0xA0u   /* linear burst write        */
#define APS6408_READ_REG_CMD     0x40u   /* mode-register read        */
#define APS6408_WRITE_REG_CMD    0xC0u   /* mode-register write       */
#define APS6408_RESET_CMD        0xFFu   /* global reset              */

#define APS6408_MR0   0x00u
#define APS6408_MR1   0x01u              /* vendor ID (read)          */
#define APS6408_MR2   0x02u              /* density/generation (read) */
#define APS6408_MR4   0x04u
#define APS6408_MR8   0x08u

/* CCR/WCCR line-count field value for 8 lines, and 32-bit address size. */
#define LINES_8       4u                 /* IMODE/ADMODE/DMODE = 8 lines */
#define ADSIZE_24     2u                 /* 24-bit address */
#define ADSIZE_32     3u                 /* 32-bit address */

/* Bounded spin budget for status polls (mirrors boot/octospi.c SPIN). */
#define SPIN          0x00100000u

/* Forward decls used by the latency-sweep recovery helper. */
static int ospi1_mem_read_ind(uint32_t off, uint8_t *d, uint32_t n);
static int ospi1_global_reset(void);

/* Health check + recovery for the write-latency sweepers.  A mis-latencied
 * write can wedge the APS6408 (seen on board #2 at >= 53 MHz: after one bad
 * write, every later transaction fails and the next mmap read stalls the AXI
 * bus into the IWDG).  Verify the device still answers a LINEAR read -- the
 * same transaction type the sweep depends on; register reads have a tighter
 * DQS-gate race and die first at high clock (88.7 MHz), which would falsely
 * abort a healthy sweep.  If it does not answer, issue a Global Reset -- out
 * of spec mid-session, but empirically the only bus-side recovery (RESET# is
 * not wired) -- and re-check.  Caller must be paused in indirect mode.
 * Returns 1 when the device answers. */
static int psram_recover(void)
{
	uint8_t buf[8];

	if (ospi1_mem_read_ind(PSRAM_SIZE_BYTES - 8u, buf, 8u))
		return 1;
	(void)ospi1_global_reset();
	return ospi1_mem_read_ind(PSRAM_SIZE_BYTES - 8u, buf, 8u);
}

/* CR without FMODE: enable + 8-deep FIFO threshold + auto-poll-stop. */
#define CR_BASE       (OCTOSPI_CR_EN | (0u << OCTOSPI_CR_FTHRES_Pos))
#define CR_FMODE_IND_W (0u << OCTOSPI_CR_FMODE_Pos)      /* indirect write */
#define CR_FMODE_IND_R (1u << OCTOSPI_CR_FMODE_Pos)      /* indirect read  */
#define CR_FMODE_MMAP  (3u << OCTOSPI_CR_FMODE_Pos)      /* memory-mapped  */

/* ------------------------------------------------------------------ *
 *  State (for `psram info`)
 * ------------------------------------------------------------------ */
static volatile int psram_up;            /* 1 once mmap entered */
static uint8_t       psram_id[2] = {0xFFu, 0xFFu};
static uint32_t      psram_rd_dcyc = PSRAM_READ_DCYC;   /* live mmap read latency  */
static uint32_t      psram_wr_dcyc = PSRAM_WRITE_DCYC;  /* live mmap write latency */
static uint32_t      psram_rd_extra = OCTOSPI_TCR_DHQC; /* read TCR SSHIFT/DHQC bits */
static uint32_t      psram_mr0_cur = PSRAM_MR0_RESET;   /* live device read-latency reg */
static uint32_t      psram_presc = PSRAM_PRESCALER;     /* live DCR2 prescaler */
static uint32_t      psram_reg_dqse = 1u;               /* DQS-gate the reg-read path */

/* Failure-stage bookkeeping (issue #3 debug): every instrumented transaction
 * records into psram_last_diag; psram_hw_init copies the first failing one
 * into psram_fail_diag so `psram info`/`psram snap` can show WHERE the cold
 * bring-up died (Global Reset TCF vs first MR-pair read FTF) and what the
 * controller state was at that instant. */
static uint32_t          psram_stage = PSRAM_STAGE_OK;
static struct psram_diag psram_last_diag;
static struct psram_diag psram_fail_diag;

/* ------------------------------------------------------------------ *
 *  Low-level OCTOSPI1 helpers (bounded polls; no HAL timeouts)
 * ------------------------------------------------------------------ */
static void ospi1_wait_not_busy(void)
{
	uint32_t n = SPIN;
	while ((OCTOSPI1->SR & OCTOSPI_SR_BUSY) && n) n--;
}

static int ospi1_wait_flag(uint32_t f)
{
	uint32_t n = SPIN;
	while (!(OCTOSPI1->SR & f) && n) n--;
	return (OCTOSPI1->SR & f) != 0u;
}

static void ospi1_abort_indirect(void)
{
	uint32_t n = SPIN;

	OCTOSPI1->CR |= OCTOSPI_CR_ABORT;
	while ((OCTOSPI1->CR & OCTOSPI_CR_ABORT) && n) n--;
	OCTOSPI1->FCR = OCTOSPI_FCR_CTCF | OCTOSPI_FCR_CTEF;
}

/* Leave any active transfer/memory-mapped window and park the controller in
 * enabled indirect mode, ready for diagnostic transactions or a DCR change. */
static void psram_pause(void)
{
	uint32_t n = SPIN;

	__DSB();
	OCTOSPI1->CR |= OCTOSPI_CR_ABORT;
	while ((OCTOSPI1->CR & OCTOSPI_CR_ABORT) && n) n--;
	ospi1_wait_not_busy();
	OCTOSPI1->FCR = OCTOSPI_FCR_CTCF | OCTOSPI_FCR_CTEF;
	OCTOSPI1->CR  = CR_BASE;
}

/* Counterpart of psram_pause: return to memory-mapped mode when the bring-up
 * succeeded, else stay parked in indirect mode so probes keep working. */
static void psram_mmap_enter(uint32_t rd_dcyc, uint32_t wr_dcyc);
static void psram_resume(void)
{
	OCTOSPI1->CR = CR_BASE;      /* re-enable first (a knob may have EN=0'd) */
	if (psram_up)
		psram_mmap_enter(psram_rd_dcyc, psram_wr_dcyc);
}

/* Snapshot the armed transaction's config into `d`.  Called right after the
 * AR write, so `ar`/`dlr` expose any ES0491 2.8.4 bit-0 clearing. */
static void diag_capture_cfg(struct psram_diag *d)
{
	d->cr   = OCTOSPI1->CR;
	d->ccr  = OCTOSPI1->CCR;
	d->tcr  = OCTOSPI1->TCR;
	d->ir   = OCTOSPI1->IR;
	d->ar   = OCTOSPI1->AR;
	d->dlr  = OCTOSPI1->DLR;
	d->dcr1 = OCTOSPI1->DCR1;
	d->dcr2 = OCTOSPI1->DCR2;
	d->dcr3 = OCTOSPI1->DCR3;
	d->dcr4 = OCTOSPI1->DCR4;
	d->dlyb_cr   = DLYB_OCTOSPI1->CR;
	d->dlyb_cfgr = DLYB_OCTOSPI1->CFGR;
}

/* The APS6408 command slot is an 8-bit SDR instruction.  A doubled 16-bit DTR
 * opcode is retained only as a diagnostic; clean readback showed that both
 * formats write the same data when DQS sampling is correctly delayed. */
#define OP16(c)  ((((uint32_t)(c)) << 8) | (uint32_t)(c))
#define ISIZE_16 1u

/* Runtime framing selector.  The APS6408 datasheet and ST component driver use
 * an 8-bit SDR instruction followed by DTR address/data.  The doubled 16-bit
 * DTR form is retained as a diagnostic because it changed the H725 write
 * signature during the initial bring-up even though the sampled wire value is
 * equivalent. */
static uint32_t psram_inst_dtr = 0u;

static uint32_t ospi1_instruction(uint8_t opcode)
{
	return psram_inst_dtr ? OP16(opcode) : opcode;
}
/* Global Reset uses a special four-clock SDR frame: one 8-bit instruction
 * clock followed by a 24-bit address/don't-care field.  This is deliberately
 * separate from normal OPI accesses, whose address and data phases are DTR. */
static int ospi1_global_reset(void)
{
	struct psram_diag *dg = &psram_last_diag;

	dg->stage = PSRAM_STAGE_GRST;
	dg->ok = 0u;
	dg->ndata = 0u;
	dg->data[0] = dg->data[1] = 0xFFu;
	dg->sr_data = 0u;
	ospi1_wait_not_busy();
	OCTOSPI1->FCR = OCTOSPI_FCR_CTCF | OCTOSPI_FCR_CTEF;
	MODIFY_REG(OCTOSPI1->CR, OCTOSPI_CR_FMODE, CR_FMODE_IND_W);
	OCTOSPI1->TCR = 0u;
	OCTOSPI1->CCR = (LINES_8 << OCTOSPI_CCR_IMODE_Pos)
	               | (LINES_8 << OCTOSPI_CCR_ADMODE_Pos)
	               | (ADSIZE_24 << OCTOSPI_CCR_ADSIZE_Pos);
	OCTOSPI1->IR = APS6408_RESET_CMD;
	dg->sr_pre = OCTOSPI1->SR;
	OCTOSPI1->AR = 0u;
	diag_capture_cfg(dg);
	if (!ospi1_wait_flag(OCTOSPI_SR_TCF)) {
		dg->sr_end = OCTOSPI1->SR;
		ospi1_abort_indirect();
		return 0;
	}
	dg->sr_end = OCTOSPI1->SR;
	dg->ok = 1u;
	OCTOSPI1->FCR = OCTOSPI_FCR_CTCF;
	HAL_Delay(1u);                         /* tRST >= 2 us */
	return 1;
}

/* Common CCR body for an 8-line command that carries an address (MR / mmap):
 * SDR instruction by default, DTR 32-bit address and DTR data.  DQS is added
 * by the caller for phases that carry read data or a memory-mapped write. */
static uint32_t ospi1_ccr_opi_addr(void)
{
	uint32_t ccr = (LINES_8 << OCTOSPI_CCR_IMODE_Pos)
	             | (LINES_8 << OCTOSPI_CCR_ADMODE_Pos) | OCTOSPI_CCR_ADDTR
	             | (ADSIZE_32 << OCTOSPI_CCR_ADSIZE_Pos)
	             | (LINES_8 << OCTOSPI_CCR_DMODE_Pos) | OCTOSPI_CCR_DDTR;

	if (psram_inst_dtr)
		ccr |= OCTOSPI_CCR_IDTR | (ISIZE_16 << OCTOSPI_CCR_ISIZE_Pos);
	return ccr;
}

/* Mode-register read (0x40): reg number in the address, read data with the
 * read latency as dummy cycles.  DQS gating follows the `psram dqs` knob
 * (default on); with it off the internal clock samples instead, so a silent
 * DQS line no longer starves the FIFO.  Fills psram_last_diag either way. */
static int ospi1_reg_read(uint8_t reg, uint8_t *buf, uint32_t n)
{
	struct psram_diag *dg = &psram_last_diag;
	uint32_t i;

	dg->stage = (reg == APS6408_MR0) ? PSRAM_STAGE_MR0 : PSRAM_STAGE_MR2;
	dg->ok = 0u;
	dg->ndata = 0u;
	dg->data[0] = dg->data[1] = 0xFFu;
	ospi1_wait_not_busy();
	OCTOSPI1->FCR = OCTOSPI_FCR_CTCF | OCTOSPI_FCR_CTEF;
	MODIFY_REG(OCTOSPI1->CR, OCTOSPI_CR_FMODE, CR_FMODE_IND_R);
	OCTOSPI1->DLR = n - 1u;
	OCTOSPI1->TCR = (psram_rd_dcyc << OCTOSPI_TCR_DCYC_Pos) | psram_rd_extra;
	OCTOSPI1->CCR = ospi1_ccr_opi_addr()
	              | (psram_reg_dqse ? OCTOSPI_CCR_DQSE : 0u);
	OCTOSPI1->IR  = ospi1_instruction(APS6408_READ_REG_CMD);
	dg->sr_pre = OCTOSPI1->SR;
	OCTOSPI1->AR  = reg;
	diag_capture_cfg(dg);
	for (i = 0u; i < n; i++) {
		int ftf = ospi1_wait_flag(OCTOSPI_SR_FTF);

		if (i == 0u)
			dg->sr_data = OCTOSPI1->SR;
		if (!ftf) {
			dg->sr_end = OCTOSPI1->SR;
			dg->ndata = (uint8_t)i;
			while (i < n)
				buf[i++] = 0xFFu;
			ospi1_abort_indirect();
			return 0;
		}
		buf[i] = *(volatile uint8_t *)&OCTOSPI1->DR;
		if (i < 2u)
			dg->data[i] = buf[i];
	}
	dg->ndata = (n < 2u) ? (uint8_t)n : 2u;
	if (!ospi1_wait_flag(OCTOSPI_SR_TCF)) {
		dg->sr_end = OCTOSPI1->SR;
		ospi1_abort_indirect();
		return 0;
	}
	dg->sr_end = OCTOSPI1->SR;
	dg->ok = 1u;
	OCTOSPI1->FCR = OCTOSPI_FCR_CTCF;
	return 1;
}

/* Indirect linear read used by DLYB tuning.  It avoids CPU/store-buffer effects
 * and exercises exactly the same DQS-gated DTR read path as mmap. */
/* Indirect-mode memory write driven through the FIFO (same DTR address/data
 * framing as mmap write).  Used by psram_scan_eye to lay down the read-eye
 * reference marker.  `dqse` selects whether the controller drives DQS (= the
 * APS6408's data-mask pin) during the data phase. */
static void ospi1_mem_write_ind(uint32_t off, const uint8_t *d, uint32_t n,
                                uint8_t opcode, int dqse)
{
	uint32_t i;

	ospi1_wait_not_busy();
	OCTOSPI1->FCR = OCTOSPI_FCR_CTCF | OCTOSPI_FCR_CTEF;
	MODIFY_REG(OCTOSPI1->CR, OCTOSPI_CR_FMODE, CR_FMODE_IND_W);
	OCTOSPI1->DLR = n - 1u;
	OCTOSPI1->TCR = (psram_wr_dcyc << OCTOSPI_TCR_DCYC_Pos) | OCTOSPI_TCR_DHQC;
	OCTOSPI1->CCR = ospi1_ccr_opi_addr() | (dqse ? OCTOSPI_CCR_DQSE : 0u);
	OCTOSPI1->IR  = ospi1_instruction(opcode);
	OCTOSPI1->AR  = off;
	/* Feed the FIFO in 32-bit words (little-endian), not bytes: the octal-DTR
	 * serializer consumes 2 bytes/clock and byte-granular DR pushes are a
	 * candidate source of byte-lane scramble. */
	for (i = 0u; i < n; i += 4u) {
		uint32_t w = 0u, k;
		for (k = 0u; k < 4u && (i + k) < n; k++)
			w |= (uint32_t)d[i + k] << (8u * k);
		ospi1_wait_flag(OCTOSPI_SR_FTF);
		*(volatile uint32_t *)&OCTOSPI1->DR = w;
	}
	ospi1_wait_flag(OCTOSPI_SR_TCF);
	OCTOSPI1->FCR = OCTOSPI_FCR_CTCF;
}

static int ospi1_mem_read_ind(uint32_t off, uint8_t *d, uint32_t n)
{
	uint32_t i;

	ospi1_wait_not_busy();
	OCTOSPI1->FCR = OCTOSPI_FCR_CTCF | OCTOSPI_FCR_CTEF;
	MODIFY_REG(OCTOSPI1->CR, OCTOSPI_CR_FMODE, CR_FMODE_IND_R);
	OCTOSPI1->DLR = n - 1u;
	OCTOSPI1->TCR = (psram_rd_dcyc << OCTOSPI_TCR_DCYC_Pos) | psram_rd_extra;
	OCTOSPI1->CCR = ospi1_ccr_opi_addr() | OCTOSPI_CCR_DQSE;
	OCTOSPI1->IR  = ospi1_instruction(APS6408_READ_LINEAR_CMD);
	OCTOSPI1->AR  = off;
	for (i = 0u; i < n; i++) {
		if (!ospi1_wait_flag(OCTOSPI_SR_FTF)) {
			ospi1_abort_indirect();
			return 0;
		}
		d[i] = *(volatile uint8_t *)&OCTOSPI1->DR;
	}
	if (!ospi1_wait_flag(OCTOSPI_SR_TCF)) {
		ospi1_abort_indirect();
		return 0;
	}
	OCTOSPI1->FCR = OCTOSPI_FCR_CTCF;
	return 1;
}

/* ------------------------------------------------------------------ *
 *  GPIO (octal-extension pins only; RMW-safe, never a whole-bank write)
 * ------------------------------------------------------------------ */
static void psram_gpio_init(void)
{
	GPIO_InitTypeDef io = {0};

	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_OSPI1_CLK_ENABLE();          /* idempotent (boot enabled it)  */

	/* Octal high nibble + DQS, all AF10 (PE9 IO6 = candidate).  HAL_GPIO_Init
	 * is per-pin RMW, so it never disturbs the OCTOSPI2 pins on these banks. */
	io.Mode  = GPIO_MODE_AF_PP;
	io.Pull  = GPIO_NOPULL;
	io.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

	/* DQS (PB2) with a weak pulldown: the APS6408 shares DQS with DM (write
	 * data mask); a floating DM when the controller is not driving it could
	 * mask write bytes.  Pulldown keeps DM=0 (unmasked) when idle. */
	io.Alternate = GPIO_AF10_OCTOSPIM_P1;
	io.Pull = GPIO_PULLDOWN;
	io.Pin = GPIO_PIN_2;  HAL_GPIO_Init(GPIOB, &io);            /* DQS */
	io.Pull = GPIO_NOPULL;
	io.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7;
	HAL_GPIO_Init(GPIOD, &io);                                 /* IO4/IO5/IO7 */
	io.Alternate = PSRAM_IO6_AF;
	io.Pin = GPIO_PIN_9;  HAL_GPIO_Init(GPIOE, &io);            /* IO6 */

}

/* ------------------------------------------------------------------ *
 *  OCTOSPI1 delay block (DLYB) -- centres the read data-eye sampling
 * ------------------------------------------------------------------ */
#define DLYB_SELECTS  12u                /* phase taps (DLYB_MAX_SELECT) */
#define DLYB_UNITS    128u               /* delay-line units (DLYB_MAX_UNIT) */

static uint32_t dlyb_sel;                /* live phase select [1..12] */
static uint32_t dlyb_unit;               /* live delay-line length [0..127] */

/* Program a specific DLYB phase/length (no measurement). */
static void dlyb_apply(uint32_t sel, uint32_t unit)
{
	DLYB_TypeDef *d = DLYB_OCTOSPI1;

	d->CR   = 0u;
	d->CR   = DLYB_CR_DEN | DLYB_CR_SEN;
	d->CFGR = (sel & 0xFu) | ((unit & 0x7Fu) << DLYB_CFGR_UNIT_Pos);
	d->CR   = DLYB_CR_DEN;
	dlyb_sel = sel;
	dlyb_unit = unit;
}

/* NOTE: no LNG-based auto-calibration here.  On the H72x the OCTOSPI delay
 * block's INPUT is the DQS line itself (RM0468 ch.27), so an FRCK-based length
 * measurement has nothing to measure (the earlier scan ran with a dead unit=0
 * line and scored 0 everywhere).  The only workable tuning is exhaustive:
 * sweep (phase, unit) and score each point by real reads (psram_scan_eye). */

/* Leave any active memory-mapped mode, then (re)program the memory-mapped read
 * (CCR/IR/TCR) and write (WCCR/WIR/WTCR) configs with the given dummy-cycle
 * counts and re-enter mmap.  Safe to call while running (OCTOSPI1 is independent
 * of the OCTOSPI2 XIP the CPU fetches from); used both at bring-up and by the
 * runtime `psram set` sweep.  Both directions are 8-line DTR data with DQS
 * (RM0468 sec 25.4.16: mmap write needs WCCR.DQSE=1). */
static void psram_mmap_enter(uint32_t rd_dcyc, uint32_t wr_dcyc)
{
	uint32_t n;

	__DSB();
	OCTOSPI1->CR |= OCTOSPI_CR_ABORT;                 /* leave mmap if active */
	n = SPIN; while ((OCTOSPI1->CR & OCTOSPI_CR_ABORT) && n) n--;
	ospi1_wait_not_busy();
	OCTOSPI1->FCR = OCTOSPI_FCR_CTCF | OCTOSPI_FCR_CTEF;
	OCTOSPI1->CR  = CR_BASE;                          /* indirect, enabled */

	OCTOSPI1->CCR  = ospi1_ccr_opi_addr() | OCTOSPI_CCR_DQSE;
	OCTOSPI1->TCR  = (rd_dcyc << OCTOSPI_TCR_DCYC_Pos) | psram_rd_extra;
	OCTOSPI1->IR   = ospi1_instruction(APS6408_READ_LINEAR_CMD);
	OCTOSPI1->ABR  = 0u;

	OCTOSPI1->WCCR = ospi1_ccr_opi_addr() | OCTOSPI_CCR_DQSE;
	OCTOSPI1->WTCR = (wr_dcyc << OCTOSPI_TCR_DCYC_Pos) | OCTOSPI_TCR_DHQC;
	OCTOSPI1->WIR  = ospi1_instruction(APS6408_WRITE_LINEAR_CMD);
	OCTOSPI1->WABR = 0u;

	__DSB();
	OCTOSPI1->CR = CR_BASE | CR_FMODE_MMAP;           /* enter memory-mapped */
	__DSB();
	__ISB();

	psram_rd_dcyc = rd_dcyc;
	psram_wr_dcyc = wr_dcyc;
}

/* ------------------------------------------------------------------ *
 *  Bring-up
 * ------------------------------------------------------------------ */
int psram_hw_init(void)
{
	psram_gpio_init();

	/* OCTOSPI1 device config.  MTYP = AP Memory (RM0468 sec 25.7.2), DEVSIZE=22
	 * (2^23 = 8 MB), and DLYB engaged for DQS input sampling. */
	OCTOSPI1->CR   = 0u;                    /* disable while programming DCR */
	OCTOSPI1->DCR1 = (2u << OCTOSPI_DCR1_MTYP_Pos)          /* 0b010 AP Memory */
	               | (22u << OCTOSPI_DCR1_DEVSIZE_Pos)
	               | (3u << OCTOSPI_DCR1_CSHT_Pos)          /* CS high 4 cyc ~45ns:
	                                                         * > APS6408 tCPH, write-
	                                                         * recovery insurance */
	               ;
	OCTOSPI1->DCR2 = (PSRAM_PRESCALER << OCTOSPI_DCR2_PRESCALER_Pos);
	OCTOSPI1->DCR3 = (PSRAM_CSBOUND << OCTOSPI_DCR3_CSBOUND_Pos);
	OCTOSPI1->DCR4 = (PSRAM_REFRESH << OCTOSPI_DCR4_REFRESH_Pos);
	dlyb_apply(PSRAM_DLYB_SEL, PSRAM_DLYB_UNIT);
	OCTOSPI1->CR   = CR_BASE;               /* enable, indirect mode */

	/* R13 is DNP, so PG2 cannot drive the PSRAM RESET# pin.  Use the official
	 * four-clock Global Reset sequence to complete Phase 2 initialization and
	 * restore the power-up mode registers. */
	if (!ospi1_global_reset()) {
		psram_stage = PSRAM_STAGE_GRST;
		psram_fail_diag = psram_last_diag;
		psram_up = 0;
		return 0;
	}

	/* Keep the power-up mode registers and read an even-addressed register pair.
	 * H72x ES0491 2.8.4 clears AR[0] and DLR[0] for DTR indirect reads, so
	 * one-byte reads and odd mode-register addresses are not valid. */
	{
		uint8_t pair[2];
		if (!ospi1_reg_read(APS6408_MR0, pair, 2u)) {
			psram_stage = PSRAM_STAGE_MR0;
			psram_fail_diag = psram_last_diag;
			psram_up = 0;
			return 0;
		}
		psram_mr0_cur = pair[0];
		psram_id[0] = pair[1];
		if (!ospi1_reg_read(APS6408_MR2, pair, 2u)) {
			psram_stage = PSRAM_STAGE_MR2;
			psram_fail_diag = psram_last_diag;
			psram_up = 0;
			return 0;
		}
		psram_id[1] = pair[0];
	}
	psram_stage = PSRAM_STAGE_OK;

	/* Configure + enter memory-mapped mode with the LIVE latency values (equal to
	 * the defaults at boot) so a `psram set` + `psram init` retry sequence works
	 * without a reflash. */
	psram_mmap_enter(psram_rd_dcyc, psram_wr_dcyc);

	psram_up = 1;
	return 1;
}

int psram_ready(void) { return psram_up; }

/* ------------------------------------------------------------------ *
 *  Concurrency guard for the single OCTOSPI1 resource
 * ------------------------------------------------------------------ *
 * The shell runs `cmd &` in background WORKER threads (shell/core/cli_job.c),
 * so a foreground command can execute while a backgrounded one is still live.
 * Every path that changes OCTOSPI1 state (indirect/mmap switch, ABORT, CR=0,
 * DCR writes) or does CPU memory-mapped access at 0x90000000 must be serialized
 * against every other, or a mutator can fire mid-transaction and corrupt data
 * or -- with a DQS-gated mmap read on a half-configured controller -- stall the
 * AXI bus until the IWDG resets the board.  A non-blocking test-and-set (LDREXB/
 * STREXB, no IRQ disable, thread-context only) lets the shell reject the second
 * user instead.  Held at the shell-command boundary: cmd_psram.c hardware
 * subcommands, cmd_membench.c PSRAM rows, and cmd_devmem.c PSRAM accesses. */
static volatile uint8_t psram_busy;

int psram_acquire(void)
{
	do {
		if (__LDREXB(&psram_busy) != 0u) {
			__CLREX();
			return 0;               /* already held by another command */
		}
	} while (__STREXB(1u, &psram_busy) != 0u);
	__DMB();
	return 1;
}

void psram_release(void)
{
	__DMB();
	psram_busy = 0u;
}

uint32_t psram_clock_hz(void) { return PSRAM_KERNEL_HZ / (psram_presc + 1u); }

/* Change the OCTOSPI1 device-clock prescaler at runtime (diagnostic: a much
 * slower clock relaxes every timing margin; if the corruption survives at
 * 33MHz it is protocol, not analog).  DCR2 is written with the OCTOSPI
 * disabled.  Touches only OCTOSPI1 -- the shared 266MHz kernel is untouched.
 * Usable in NOT-ready state for `psram probe` sweeps. */
void psram_set_prescaler(uint32_t presc)
{
	if (presc > 255u)
		return;
	psram_pause();
	OCTOSPI1->CR  = 0u;                       /* EN=0 while changing DCR2 */
	OCTOSPI1->DCR2 = (presc << OCTOSPI_DCR2_PRESCALER_Pos);
	psram_presc = presc;
	psram_resume();
}

void psram_read_id(uint8_t id[2]) { id[0] = psram_id[0]; id[1] = psram_id[1]; }

void psram_get_latency(uint32_t *rd_dcyc, uint32_t *wr_dcyc)
{
	*rd_dcyc = psram_rd_dcyc;
	*wr_dcyc = psram_wr_dcyc;
}

/* Re-enter mmap with new read/write dummy-cycle counts (runtime latency sweep).
 * In NOT-ready state just retimes the indirect probe/register path. */
void psram_set_latency(uint32_t rd_dcyc, uint32_t wr_dcyc)
{
	if (psram_up) {
		psram_mmap_enter(rd_dcyc, wr_dcyc);
		return;
	}
	psram_rd_dcyc = rd_dcyc;
	psram_wr_dcyc = wr_dcyc;
}

uint32_t psram_get_instruction_dtr(void) { return psram_inst_dtr; }

void psram_set_instruction_dtr(int dtr)
{
	psram_inst_dtr = dtr ? 1u : 0u;
	if (psram_up)
		psram_mmap_enter(psram_rd_dcyc, psram_wr_dcyc);
}

/* Current DLYB phase select + delay-line length. */
void psram_get_phase(uint32_t *sel, uint32_t *unit)
{
	*sel = dlyb_sel;
	*unit = dlyb_unit;
}

/* Override the DLYB read-sampling phase (runtime sweep to centre the data eye).
 * Pauses any active mmap window so the delay line never moves mid-transfer. */
void psram_set_phase(uint32_t sel, uint32_t unit)
{
	psram_pause();
	OCTOSPI1->CR = 0u;
	dlyb_apply(sel, unit);
	psram_resume();
}

/* Read-path timing flags: bit0 = SSHIFT (sample half a cycle later), bit1 = DHQC
 * (delay-hold quarter cycle).  Re-enters mmap. */
uint32_t psram_get_read_flags(void)
{
	return ((psram_rd_extra & OCTOSPI_TCR_SSHIFT) ? 1u : 0u)
	     | ((psram_rd_extra & OCTOSPI_TCR_DHQC)   ? 2u : 0u);
}

/* Re-write the APS6408 read-latency register MR0 (device-side latency code) and
 * re-enter mmap, to sweep the device latency without a reflash. */
uint32_t psram_get_mr0(void) { return psram_mr0_cur; }

/* Bypass (1) or engage (0) the read delay block at runtime, at the currently
 * applied phase/unit.  DCR1.DLYBYP is only ever written with the OCTOSPI
 * DISABLED -- the earlier crash came from toggling it with EN=1 under an
 * active memory-mapped window (RM0468: DCR1 is config-while-disabled).
 * Usable in NOT-ready state for `psram probe` sweeps. */
void psram_dlyb_bypass(int bypass)
{
	psram_pause();
	OCTOSPI1->CR = 0u;                       /* EN=0: DCR1 writable */
	if (bypass)
		OCTOSPI1->DCR1 |= OCTOSPI_DCR1_DLYBYP;
	else
		OCTOSPI1->DCR1 &= ~OCTOSPI_DCR1_DLYBYP;
	psram_resume();
}

/* Read-eye exhaustive tuning.  A 32-byte high-transition marker catches data
 * errors that a single-register check misses.  counts[sel] packs the best
 * full-marker pass count in bits[7:0] and its unit in bits[15:8].  The globally
 * best (phase, unit) remains applied.  NB (issue #16): the scan reliably SCORES
 * a low unit as best, but applying it can break subsequent mmap traffic at the
 * same clock (unexplained in-scan vs post-scan discrepancy) -- do not adopt an
 * eye result as the operating point without a full `psram test` pass. */
uint32_t psram_scan_eye(uint32_t counts[13], uint32_t trials)
{
	static const uint8_t marker[32] = {
		0x10,0x23,0x45,0x67,0x89,0xAB,0xCD,0xEF,
		0x00,0xFF,0x55,0xAA,0x96,0x69,0x3C,0xC3,
		0x7E,0x81,0x18,0xE7,0x42,0xBD,0x24,0xDB,
		0x0F,0xF0,0x33,0xCC,0x5A,0xA5,0x66,0x99
	};
	static uint8_t eye_nonce;
	uint8_t expect[sizeof marker];
	uint8_t got[sizeof marker];
	const uint32_t off = PSRAM_SIZE_BYTES - 64u;
	uint32_t sel, u, t, i, n;
	uint32_t best = 0u, best_cnt = 0u, best_score = 0u, best_unit = 0u;

	if (!psram_up || trials == 0u)
		return 0u;

	/* Fold a per-run nonce into the marker: the scan writes the SAME offset
	 * every run, so without it a silently-failing write reads back the stale
	 * marker from an earlier (slower-clock) run and fakes a perfect eye. */
	eye_nonce += 0x35u;
	for (i = 0u; i < sizeof marker; i++)
		expect[i] = marker[i] ^ eye_nonce;

	/* Leave mmap and engage DLYB.  Write the reference once; changing DLYB affects
	 * only the DQS input path, so every candidate reads identical device data. */
	__DSB();
	OCTOSPI1->CR |= OCTOSPI_CR_ABORT;
	n = SPIN; while ((OCTOSPI1->CR & OCTOSPI_CR_ABORT) && n) n--;
	ospi1_wait_not_busy();
	OCTOSPI1->FCR = OCTOSPI_FCR_CTCF | OCTOSPI_FCR_CTEF;
	OCTOSPI1->CR = 0u;
	OCTOSPI1->DCR1 &= ~OCTOSPI_DCR1_DLYBYP;
	OCTOSPI1->CR = CR_BASE;
	ospi1_mem_write_ind(off, expect, sizeof expect,
	                    APS6408_WRITE_LINEAR_CMD, 1);

	for (sel = 1u; sel <= DLYB_SELECTS; sel++) {
		uint32_t sel_cnt = 0u, sel_score = 0u, sel_unit = 0u;
		for (u = 0u; u < DLYB_UNITS; u += 8u) {
			uint32_t cnt = 0u, score = 0u;
			OCTOSPI1->CR = 0u;
			dlyb_apply(sel, u);
			OCTOSPI1->CR = CR_BASE;
			for (t = 0u; t < trials; t++) {
				uint32_t match = 0u;
				if (ospi1_mem_read_ind(off, got, sizeof got))
					for (i = 0u; i < sizeof marker; i++)
						if (got[i] == expect[i])
							match++;
				score += match;
				if (match == sizeof marker)
					cnt++;
			}
			if (cnt > sel_cnt || (cnt == sel_cnt && score > sel_score)) {
				sel_cnt = cnt;
				sel_score = score;
				sel_unit = u;
			}
			if (cnt > best_cnt || (cnt == best_cnt && score > best_score)) {
				best_cnt = cnt;
				best_score = score;
				best = sel;
				best_unit = u;
			}
		}
		counts[sel] = (sel_cnt & 0xFFu) | ((sel_unit & 0xFFu) << 8);
	}

	OCTOSPI1->CR = 0u;
	if (best_score != 0u) {
		dlyb_apply(best, best_unit);
	} else {
		OCTOSPI1->DCR1 |= OCTOSPI_DCR1_DLYBYP;
		dlyb_sel = 0u;
		best = 0u;
	}
	OCTOSPI1->CR = CR_BASE;
	psram_mmap_enter(psram_rd_dcyc, psram_wr_dcyc);
	return best;
}

/* Write-latency tuner: once the read eye is trusted (psram_scan_eye), sweep the
 * memory-mapped write dummy count 0..15, writing two marker words per setting
 * and reading them back (DSB drains the M7 store buffer so the readback really
 * comes from the device, not store-forwarding).  Repeats each setting `reps`
 * times and requires all to match.  Returns a pass bitmask (bit d = dummy d
 * round-tripped); leaves the lowest passing dummy applied (or the original on
 * total failure).  This measures the device's actual write latency directly.
 *
 * The readback goes through the bounded INDIRECT path, never the mmap window:
 * a mis-latencied write can leave the device unresponsive for the next read,
 * and a DQS-gated MMAP read then stalls the AXI bus forever (no timeout in
 * mmap mode) -> CPU lock -> IWDG reset.  Board #2 hit exactly that at d=0. */
uint32_t psram_wtune(uint32_t off, uint32_t reps)
{
	volatile uint32_t *p =
	    (volatile uint32_t *)(uintptr_t)(PSRAM_BASE_ADDR + off);
	static uint32_t wt_nonce;
	uint32_t d, r, i, mask = 0u, orig = psram_wr_dcyc, first = 0xFFFFFFFFu;

	if (!psram_up || off > PSRAM_SIZE_BYTES - 8u)
		return 0u;

	/* Per-run nonce folded into the markers: the sweep reuses the same
	 * address every run, so without it a silently-failing write could read
	 * back a previous run's identical marker and fake a PASS. */
	wt_nonce += 0x004D0000u;

	for (d = 0u; d <= 15u; d++) {
		uint32_t ok = 1u;
		for (r = 0u; r < reps && ok; r++) {
			uint32_t m0 = (0x5AC30000u | (d << 8) | r) ^ wt_nonce;
			uint32_t m1 = ~m0;
			uint8_t back[8];
			uint32_t g0 = 0u, g1 = 0u;

			psram_mmap_enter(psram_rd_dcyc, d);
			p[0] = m0;                      /* the path under test: mmap write */
			p[1] = m1;
			__DSB();                        /* drain store buffer -> bus */
			psram_pause();                  /* leave mmap before reading back */
			if (!ospi1_mem_read_ind(off, back, 8u)) {
				ok = 0u;                    /* timed out; abort already done */
				continue;
			}
			for (i = 0u; i < 4u; i++) {
				g0 |= (uint32_t)back[i] << (8u * i);
				g1 |= (uint32_t)back[4u + i] << (8u * i);
			}
			if (g0 != m0 || g1 != m1)
				ok = 0u;
		}
		/* Recover a device wedged by this dummy's writes before it poisons
		 * the rest of the sweep; bail with the partial mask if it stays
		 * dead so the caller sees the failure instead of 16 bogus fails. */
		if (!psram_recover()) {
			psram_mmap_enter(psram_rd_dcyc, orig);
			return mask;
		}
		if (ok) {
			mask |= 1u << d;
			if (first == 0xFFFFFFFFu)
				first = d;
		}
	}
	psram_mmap_enter(psram_rd_dcyc,
	                 (first != 0xFFFFFFFFu) ? first : orig);
	return mask;
}

/* ------------------------------------------------------------------ *
 *  NOT-ready diagnostics (issue #3 cold bring-up failure)
 * ------------------------------------------------------------------ */
uint32_t psram_init_stage(void) { return psram_stage; }

void psram_get_init_diag(struct psram_diag *out)
{
	if (out)
		*out = psram_fail_diag;
}

void psram_get_last_diag(struct psram_diag *out)
{
	if (out)
		*out = psram_last_diag;
}

/* One legal MA-pair read with the current knobs, full snapshot returned.
 * Works whether or not the bring-up succeeded (the controller is always left
 * configured by psram_hw_init, even on failure). */
int psram_probe_pair(uint32_t ma, struct psram_diag *out)
{
	uint8_t pair[2];

	if (ma != 0u && ma != 2u && ma != 4u && ma != 8u)
		return -1;
	psram_pause();
	(void)ospi1_reg_read((uint8_t)ma, pair, 2u);
	if (out)
		*out = psram_last_diag;
	psram_resume();
	return 0;
}

/* Manually re-issue the Global Reset (`psram grst`).  1 = TCF completed. */
int psram_global_reset_cmd(void)
{
	int ok;

	psram_pause();
	ok = ospi1_global_reset();
	psram_resume();
	return ok;
}

uint32_t psram_get_dqse(void) { return psram_reg_dqse; }
void     psram_set_dqse(int en) { psram_reg_dqse = en ? 1u : 0u; }

uint32_t psram_get_refresh(void)
{
	return OCTOSPI1->DCR4;
}

void psram_set_refresh(uint32_t r)
{
	psram_pause();
	OCTOSPI1->CR = 0u;                       /* EN=0 while changing DCR4 */
	OCTOSPI1->DCR4 = (r << OCTOSPI_DCR4_REFRESH_Pos);
	psram_resume();
}

void psram_snap_regs(struct psram_regs *out)
{
	if (out == NULL)
		return;
	out->cr   = OCTOSPI1->CR;
	out->sr   = OCTOSPI1->SR;
	out->dcr1 = OCTOSPI1->DCR1;
	out->dcr2 = OCTOSPI1->DCR2;
	out->dcr3 = OCTOSPI1->DCR3;
	out->dcr4 = OCTOSPI1->DCR4;
	out->ccr  = OCTOSPI1->CCR;
	out->tcr  = OCTOSPI1->TCR;
	out->ir   = OCTOSPI1->IR;
	out->ar   = OCTOSPI1->AR;
	out->dlr  = OCTOSPI1->DLR;
	out->wccr = OCTOSPI1->WCCR;
	out->wtcr = OCTOSPI1->WTCR;
	out->wir  = OCTOSPI1->WIR;
	out->om_cr   = OCTOSPIM->CR;
	out->om_p1cr = OCTOSPIM->PCR[0];
	out->om_p2cr = OCTOSPIM->PCR[1];
	out->dlyb_cr   = DLYB_OCTOSPI1->CR;
	out->dlyb_cfgr = DLYB_OCTOSPI1->CFGR;
}

/* GPIO state of every PSRAM pin (mode/AF/pull/input level), so a wrong or
 * bootloader-dependent pin config shows up without an oscilloscope. */
uint32_t psram_snap_pins(struct psram_pin *out, uint32_t max)
{
	static GPIO_TypeDef *const gpio[] = {
		GPIOF, GPIOF, GPIOF, GPIOF, GPIOD, GPIOD,
		GPIOE, GPIOD, GPIOB, GPIOF, GPIOG
	};
	static const char port[] = {
		'F', 'F', 'F', 'F', 'D', 'D', 'E', 'D', 'B', 'F', 'G'
	};
	static const uint8_t pin[] = { 8, 9, 7, 6, 4, 5, 9, 7, 2, 10, 6 };
	static const char *const name[] = {
		"IO0", "IO1", "IO2", "IO3", "IO4", "IO5",
		"IO6", "IO7", "DQS", "CLK", "NCS"
	};
	uint32_t i, n = sizeof pin / sizeof pin[0];

	if (out == NULL)
		return 0u;
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	if (n > max)
		n = max;
	for (i = 0u; i < n; i++) {
		GPIO_TypeDef *g = gpio[i];
		uint32_t p = pin[i];

		out[i].port = port[i];
		out[i].pin  = (uint8_t)p;
		out[i].mode = (uint8_t)((g->MODER >> (2u * p)) & 3u);
		out[i].af   = (uint8_t)((g->AFR[p >> 3] >> (4u * (p & 7u))) & 0xFu);
		out[i].pupd = (uint8_t)((g->PUPDR >> (2u * p)) & 3u);
		out[i].idr  = (uint8_t)((g->IDR >> p) & 1u);
		out[i].name = name[i];
	}
	return n;
}
