/*
 * Wio Lite AI (STM32H725AEI6) -- OCTOSPI1 APS6408L-3OBM-BA Octal DDR PSRAM
 * app-side bring-up (issue #3) + clock/eye characterisation (issue #16).
 * Indirect operations are bounded and fail-soft so a missing PSRAM response does
 * not stop USB.  Ships at 133 MHz Fixed Latency (read 113 / write 154 MB/s),
 * validated on board #2 with a full-8MB test pass; the operating-point table and
 * the two-stage bring-up are documented at the PSRAM_* defines below.
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

/* Device clock = OSPI kernel (PLL2R 266 MHz) / (PRESCALER+1).
 *
 * Three operating points were characterised on board #2 (issue #16, mmapscan
 * memory-mapped read-eye + membench), all full-8MB `psram test` clean:
 *
 *   presc MR0    rd wr DLYB  clock     read/write MB/s  mmap eye (DLYB units)
 *     4   0x09    5  4  u64  53.2 MHz   51 / 62         [16..68]  widest
 *     2   0x24    8  4  u16  88.7 MHz   78 / 103        [0..32]
 *   > 1   0x24    8  4  u8   133  MHz  113 / 154        [0..20]   narrow (max BW)
 *
 * SHIPPED: 133 MHz Fixed Latency -- 2.4x the 53.2 MHz point.  High clocks
 * REQUIRE Fixed Latency (MR0=0x24, LC8): with the power-up variable latency the
 * refresh push-out jitter collapses the read eye to nothing at 133 MHz (mmapscan
 * shows zero passing DLYB units).  To ship a different point, replace the
 * PSRAM_PRESCALER / PSRAM_MR0_OP / PSRAM_READ_DCYC / PSRAM_DLYB_UNIT values with
 * the matching row above.
 *
 * TWO-STAGE BRING-UP: the Global Reset, mode-register reads AND the MR0
 * Fixed-Latency write must all run at a clock where register transactions are
 * reliable (register writes at 133 MHz corrupt).  psram_hw_init() therefore
 * brings the device up at the safe INIT clock (53.2 MHz), writes MR0, and only
 * then raises the clock and re-centres the DLYB for the operating point. */
#define PSRAM_KERNEL_HZ    266000000u

/* Safe bring-up point: reliable Global Reset + MR reads + MR0 write. */
#define PSRAM_INIT_PRESCALER  4u   /* 53.2 MHz */
#define PSRAM_INIT_READ_DCYC  5u   /* DQS-gated LC = 5 (device power-up default, */
                                   /*   NOT the ST LC-1: LC-1 opens the capture   */
                                   /*   window a clock early and misses the burst) */
#define PSRAM_INIT_DLYB_UNIT  64u  /* reads MRs cleanly at 53.2 MHz */

/* Operating point: 133 MHz Fixed Latency (shipped default). */
#define PSRAM_PRESCALER    1u      /* DCR2.PRESCALER -> /2 = 133 MHz */
#define PSRAM_MR0_OP       0x24u   /* Fixed Latency LC8, 1/2 drive (ST U585 ref) */
#define PSRAM_READ_DCYC    8u      /* matches Fixed LC8 */
#define PSRAM_WRITE_DCYC   4u      /* WLC-1; psram_wtune confirms 4 at 133 MHz */
#define PSRAM_DLYB_SEL     3u
#define PSRAM_DLYB_UNIT    8u      /* centre of the 133 MHz mmap eye [0..20] */

/* APS6408 power-up default MR0 (variable LC5, 1/4 drive) -- what the device
 * holds after a Global Reset, before the Fixed-Latency MR0 write. */
#define PSRAM_MR0_RESET    0x09u

/* Split mmap transactions every 32 bytes.  This stays below tCEM and prevents
 * the device's mandatory 1 KB row wrap from aliasing linear CPU accesses. */
#define PSRAM_CSBOUND      5u

/* Limit transaction duration so the RAM can perform internal refresh.  Zero
 * disables this feature; any nonzero value must exceed the complete minimum
 * command/address/dummy/data transaction. */
#define PSRAM_REFRESH      320u

/* AP Memory IO6 alternate function. */
#define PSRAM_IO6_AF       GPIO_AF10_OCTOSPIM_P1

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

/* Mode-register write (0xC0): reg number in the address, one value byte sent as
 * the DTR pair (register writes are latency 1, APS6408 datasheet Fig.13). */
static void ospi1_reg_write(uint8_t reg, uint8_t val)
{
	ospi1_wait_not_busy();
	OCTOSPI1->FCR = OCTOSPI_FCR_CTCF | OCTOSPI_FCR_CTEF;
	MODIFY_REG(OCTOSPI1->CR, OCTOSPI_CR_FMODE, CR_FMODE_IND_W);
	OCTOSPI1->DLR = 1u;                     /* 2 bytes (DTR pair) */
	OCTOSPI1->TCR = 0u;                     /* register write latency 1 */
	OCTOSPI1->CCR = ospi1_ccr_opi_addr();
	OCTOSPI1->IR  = ospi1_instruction(APS6408_WRITE_REG_CMD);
	OCTOSPI1->AR  = reg;
	ospi1_wait_flag(OCTOSPI_SR_FTF);
	*(volatile uint8_t *)&OCTOSPI1->DR = val;
	ospi1_wait_flag(OCTOSPI_SR_FTF);
	*(volatile uint8_t *)&OCTOSPI1->DR = val;
	ospi1_wait_flag(OCTOSPI_SR_TCF);
	OCTOSPI1->FCR = OCTOSPI_FCR_CTCF;
}

/* Indirect linear read used by DLYB tuning.  It avoids CPU/store-buffer effects
 * and exercises exactly the same DQS-gated DTR read path as mmap. */
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
#define DLYB_SELECTS  12u                /* phase taps (DLYB_MAX_SELECT).  The
                                          * unit axis is swept by `psram mmapscan`
                                          * over PSRAM_SCAN_COLS*PSRAM_SCAN_STEP. */

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
 * sweep (phase, unit) and validate each by real memory-mapped reads
 * (`psram mmapscan`). */

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
	uint8_t pair[2];

	psram_gpio_init();

	/* Stage 1 -- bring up at the safe INIT clock (53.2 MHz).  Reset the live
	 * knobs to the INIT/power-up values so a `psram init` re-run after a
	 * diagnostic sweep starts from a clean, deterministic state. */
	psram_rd_dcyc  = PSRAM_INIT_READ_DCYC;
	psram_wr_dcyc  = PSRAM_WRITE_DCYC;
	psram_presc    = PSRAM_INIT_PRESCALER;
	psram_mr0_cur  = PSRAM_MR0_RESET;
	psram_rd_extra = OCTOSPI_TCR_DHQC;
	psram_inst_dtr = 0u;
	psram_reg_dqse = 1u;

	/* OCTOSPI1 device config.  MTYP = AP Memory (RM0468 sec 25.7.2), DEVSIZE=22
	 * (2^23 = 8 MB), and DLYB engaged for DQS input sampling. */
	OCTOSPI1->CR   = 0u;                    /* disable while programming DCR */
	OCTOSPI1->DCR1 = (2u << OCTOSPI_DCR1_MTYP_Pos)          /* 0b010 AP Memory */
	               | (22u << OCTOSPI_DCR1_DEVSIZE_Pos)
	               | (3u << OCTOSPI_DCR1_CSHT_Pos)          /* CS high 4 cyc ~45ns:
	                                                         * > APS6408 tCPH, write-
	                                                         * recovery insurance */
	               ;
	OCTOSPI1->DCR2 = (PSRAM_INIT_PRESCALER << OCTOSPI_DCR2_PRESCALER_Pos);
	OCTOSPI1->DCR3 = (PSRAM_CSBOUND << OCTOSPI_DCR3_CSBOUND_Pos);
	OCTOSPI1->DCR4 = (PSRAM_REFRESH << OCTOSPI_DCR4_REFRESH_Pos);
	dlyb_apply(PSRAM_DLYB_SEL, PSRAM_INIT_DLYB_UNIT);
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

	/* Read an even-addressed register pair (device present + vendor/density).
	 * H72x ES0491 2.8.4 clears AR[0] and DLR[0] for DTR indirect reads, so
	 * one-byte reads and odd mode-register addresses are not valid.  These reads
	 * run at 53.2 MHz on the power-up variable latency (matches INIT_READ_DCYC). */
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
	psram_stage = PSRAM_STAGE_OK;

	/* Enter mmap at the INIT point, then flag ready so the operating-point
	 * switch below can use the psram_set_* helpers (they gate on psram_up). */
	psram_mmap_enter(PSRAM_INIT_READ_DCYC, PSRAM_WRITE_DCYC);
	psram_up = 1;

	/* Stage 2 -- switch to the operating point.  ORDER MATTERS: write MR0 while
	 * still at the safe 53.2 MHz clock (a register write at 133 MHz corrupts),
	 * THEN raise the clock, set the matching controller latency, and re-centre
	 * the DLYB.  The intermediate (wrong-DLYB) states are never read -- the first
	 * mmap access after psram_set_phase() sees the fully-applied operating point.
	 * This mirrors psram_mmapscan_boot()'s clean per-unit apply. */
#if PSRAM_MR0_OP != PSRAM_MR0_RESET
	psram_set_mr0(PSRAM_MR0_OP);
#endif
#if PSRAM_PRESCALER != PSRAM_INIT_PRESCALER
	psram_set_prescaler(PSRAM_PRESCALER);
#endif
	psram_set_latency(PSRAM_READ_DCYC, PSRAM_WRITE_DCYC);
	psram_set_phase(PSRAM_DLYB_SEL, PSRAM_DLYB_UNIT);

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
 * re-enter mmap, to sweep the device latency without a reflash.  E.g. #16 tries
 * the ST U585 reference config MR0=0x24 (Fixed Latency LC8, matches read dummy
 * 8) to remove the variable-latency refresh-pushout jitter at high clock. */
uint32_t psram_get_mr0(void) { return psram_mr0_cur; }

void psram_set_mr0(uint32_t val)
{
	if (!psram_up)
		return;
	psram_pause();
	ospi1_reg_write(APS6408_MR0, (uint8_t)val);
	psram_mr0_cur = val;
	psram_resume();
}

/* Write-latency tuner: once the read eye is trusted (`psram mmapscan`), sweep the
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

uint32_t psram_get_dqse(void) { return psram_reg_dqse; }

uint32_t psram_get_refresh(void)
{
	return OCTOSPI1->DCR4;
}

/* ------------------------------------------------------------------ *
 *  mmapscan: reset-persistent DLYB sweep validated against MEMORY-MAPPED
 *  access (issue #16)
 * ------------------------------------------------------------------ *
 * An indirect-read DLYB eye is both non-predictive of the mmap eye
 * AND corrupts the DLYB across a full unit sweep (RM0468 sec 27: SEN-pulsing the
 * uncalibrated delay line decalibrates it).  The only authoritative validator is
 * a real mmap read, but a wrong DLYB stalls the AXI bus with no timeout -> hang.
 *
 * mmapscan sidesteps both problems: it tests ONE DLYB unit per boot, applied
 * from the just-run psram_hw_init clean state (a single, clean dlyb_apply), and
 * persists its progress in reset-persistent DTCM.  A PASS records the bit and
 * software-resets to the next unit; a HANG is caught by the IWDG (armed here)
 * whose reset lands back in this function, where the un-passed-but-tested bit
 * marks the unit FAILED before advancing.  The whole unit axis is thus swept
 * across auto-reboots, mapping the true mmap eye without a reflash or a hang that
 * needs human intervention.  Runs in main() before tx_kernel_enter (headless:
 * the sweep boots never reach USB/shell). */
#define PSRAM_SCAN_MAGIC_ACTIVE  0x50534D31u   /* "PSM1" -- sweep running */
#define PSRAM_SCAN_MAGIC_DONE    0x50534D30u   /* "PSM0" -- sweep complete */

#if BSP_ENABLE_IWDG
extern void iwdg_init(void);                   /* app/iwdg.c: arm IWDG1 (~3 s) */
#endif

/* Reset-persistent (DTCM NOLOAD, survives a system reset, lost on POR). */
static struct psram_scan_state psram_scan
	__attribute__((section(".log_noinit.psram_scan")));

/* DTCM stores need a read-back to durably land across a reset (issue #13). */
static void psram_scan_persist(void)
{
	volatile uint32_t *p = (volatile uint32_t *)&psram_scan;
	uint32_t i;

	__DSB();
	for (i = 0u; i < sizeof psram_scan / 4u; i++)
		(void)p[i];
	__DSB();
}

/* 4 KB memory-mapped write/verify: returns 1 on a clean round-trip, 0 on a
 * mismatch.  A DLYB that cannot sample the read data stalls the AXI bus here --
 * that hang is the expected "unit fails" signal, caught by the IWDG. */
static uint32_t psram_scan_mmap_ok(void)
{
	volatile uint32_t *p = (volatile uint32_t *)(uintptr_t)PSRAM_BASE_ADDR;
	uint32_t i;

	for (i = 0u; i < 1024u; i++)
		p[i] = PSRAM_BASE_ADDR + i * 4u;
	__DSB();
	for (i = 0u; i < 1024u; i++)
		if (p[i] != PSRAM_BASE_ADDR + i * 4u)
			return 0u;
	for (i = 0u; i < 1024u; i++)
		p[i] = (i & 1u) ? 0xAAAAAAAAu : 0x55555555u;
	__DSB();
	for (i = 0u; i < 1024u; i++)
		if (p[i] != ((i & 1u) ? 0xAAAAAAAAu : 0x55555555u))
			return 0u;
	return 1u;
}

void psram_mmapscan_boot(void)
{
	uint32_t presc, phase, rd, wr, ulo, ustep, ncand, idx, unit;

	if (psram_scan.magic != PSRAM_SCAN_MAGIC_ACTIVE)
		return;                          /* no sweep in progress */
	if (!psram_up) {                     /* device didn't come up: abort sweep */
		psram_scan.magic = PSRAM_SCAN_MAGIC_DONE;
		psram_scan_persist();
		return;
	}

	presc = psram_scan.cfg & 0xFFu;
	phase = (psram_scan.cfg >> 8) & 0xFFu;
	rd    = (psram_scan.cfg >> 16) & 0xFFu;
	wr    = (psram_scan.cfg >> 24) & 0xFFu;
	ulo   = psram_scan.rng & 0xFFu;
	ustep = (psram_scan.rng >> 8) & 0xFFu;
	ncand = (psram_scan.rng >> 16) & 0xFFu;
	idx   = (psram_scan.rng >> 24) & 0xFFu;

	/* A tested-but-not-passed current candidate means it hung last boot (a PASS
	 * would have advanced idx before resetting): it is already recorded FAILED,
	 * so step past it. */
	if (idx < ncand && (psram_scan.tested & (1u << idx)) &&
	    !(psram_scan.passed & (1u << idx)))
		idx++;

	if (idx >= ncand) {                  /* complete: keep results, boot normally */
		psram_scan.magic = PSRAM_SCAN_MAGIC_DONE;
		psram_scan.rng = (psram_scan.rng & 0x00FFFFFFu) | (idx << 24);
		psram_scan_persist();
		return;
	}

	/* Apply this candidate.  psram_hw_init() has already raised the clock to the
	 * shipped operating point, so FIRST drop back to the safe INIT clock -- a
	 * register (MR0) write at a high clock corrupts -- then write the scan's
	 * intended MR0 (always, so a variable-latency 0x09 scan is reproducible, not
	 * just the shipped Fixed 0x24), then raise to the scan clock, set the
	 * controller dummy, and centre the DLYB (a single clean dlyb_apply). */
	psram_set_prescaler(PSRAM_INIT_PRESCALER);
	psram_set_mr0(psram_scan.mr0);
	if (presc != PSRAM_INIT_PRESCALER)
		psram_set_prescaler(presc);
	psram_set_latency(rd, wr);
	unit = ulo + idx * ustep;
	psram_set_phase(phase, unit);

	/* Record "testing idx" BEFORE the risky access: a hang's IWDG reset returns
	 * here with tested[idx] set and passed[idx] clear. */
	psram_scan.tested |= (1u << idx);
	psram_scan.rng = (psram_scan.rng & 0x00FFFFFFu) | (idx << 24);
	psram_scan_persist();

#if BSP_ENABLE_IWDG
	iwdg_init();                         /* ~3 s: recovers a hanging mmap read */
#endif

	if (psram_scan_mmap_ok())
		psram_scan.passed |= (1u << idx);
	psram_scan.rng = (psram_scan.rng & 0x00FFFFFFu) | ((idx + 1u) << 24);
	psram_scan_persist();

	NVIC_SystemReset();                  /* clean-state boot for the next unit */
}

/* Start a sweep at the CURRENT clock/latency/MR0 over DLYB units
 * [ulo .. ulo+(ncand-1)*ustep] on `phase`.  Persists the plan and resets into
 * psram_mmapscan_boot() -- does not return.  0 on bad params (returns then). */
int psram_mmapscan_start(uint32_t phase, uint32_t ulo, uint32_t ustep,
                         uint32_t ncand)
{
	if (!psram_up || phase < 1u || phase > 12u || ustep == 0u ||
	    ncand == 0u || ncand > 32u ||
	    ulo + (ncand - 1u) * ustep > 127u)
		return 0;

	psram_scan.cfg = (psram_presc & 0xFFu)
	               | ((phase & 0xFFu) << 8)
	               | ((psram_rd_dcyc & 0xFFu) << 16)
	               | ((psram_wr_dcyc & 0xFFu) << 24);
	psram_scan.rng = (ulo & 0xFFu) | ((ustep & 0xFFu) << 8)
	               | ((ncand & 0xFFu) << 16) | (0u << 24);   /* idx = 0 */
	psram_scan.tested = 0u;
	psram_scan.passed = 0u;
	/* Store the FULL current MR0 (0x24 Fixed, 0x09 variable, ...); the boot loop
	 * re-writes it verbatim at the safe clock every candidate, so any latency
	 * mode is reproducible. */
	psram_scan.mr0 = psram_mr0_cur & 0xFFu;
	psram_scan.magic = PSRAM_SCAN_MAGIC_ACTIVE;
	psram_scan_persist();
	NVIC_SystemReset();
	return 1;                            /* not reached */
}

void psram_mmapscan_stop(void)
{
	psram_scan.magic = 0u;
	psram_scan_persist();
}

int psram_mmapscan_get(struct psram_scan_state *out)
{
	if (out == NULL)
		return 0;
	if (psram_scan.magic != PSRAM_SCAN_MAGIC_ACTIVE &&
	    psram_scan.magic != PSRAM_SCAN_MAGIC_DONE)
		return 0;
	*out = psram_scan;
	return (psram_scan.magic == PSRAM_SCAN_MAGIC_DONE) ? 2 : 1;
}
