/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    camera.c
 * @brief   DVP camera bring-up: XCLK (TIM5), SCCB (I2C4), sensor ID (issue #8
 *          phase 1).  See camera.h for the API contract and the two board facts
 *          that shape it (host-generated XCLK, ungated camera supply).
 *
 * Hardware setup, all cross-checked between the board schematic, ST's published
 * pin data for this exact package (_ref/stm32_open_pin_data/STM32H725AEIx.xml --
 * the UFBGA169 alternate-function map is NOT the H743's) and the factory Arduino
 * firmware left in the board's external flash, whose HAL_DCMI_MspInit programs
 * the same pins at AF13:
 *
 *   - XCLK: PA2 = TIM5_CH3, AF2.  PWM mode 1, PSC = 0, the duty pinned to
 *     ARR+1 over 2.  The kernel clock is rcc_tim5_ker_ck: with RCC_CFGR.TIMPRE
 *     at its reset 0 and D2PPRE1 = /2 (set by the bootloader), RM0468 sec 8.7.6 /
 *     8.7.8 make it 2 x rcc_pclk1 = 275 MHz.  cam_tim_ker_hz() derives that at
 *     run time from the HAL rather than hardcoding it, so a bootloader that ever
 *     changes the tree cannot silently skew XCLK.
 *
 *     275 MHz cannot produce exactly 24 MHz.  The default divider is 12 ->
 *     22.917 MHz, which keeps an exact 50% duty; 11 would give 25.0 MHz but a
 *     45.5% duty.  Both are inside the 6..27 MHz an OV2640 accepts.
 *
 *   - SCCB: I2C4, PF14 = SCL / PF15 = SDA, AF4, open-drain.  The board carries
 *     4.7k pull-ups (R33/R34); the MCU's internal ones can be added on top from
 *     `camera tune pull on` if that reading of the schematic turns out wrong.
 *     I2C4 lives in D3 and is muxed by RCC_D3CCIPR.I2C4SEL, whose RESET value
 *     selects rcc_pclk4 = 137.5 MHz.  The __HAL_RCC_I2C4_CONFIG below re-states
 *     that default rather than changing anything -- the same idiom, and the same
 *     justification, as port/sd/sd_card.c's __HAL_RCC_SDMMC_CONFIG.  No PLL, no
 *     divider and no FLASH_ACR is written anywhere in this file (CLAUDE.md: the
 *     app inherits the bootloader's clock tree and never reconfigures it).
 *
 *   - PWDN = PE7 and RESETB = PH12, plain GPIO outputs.  Both are driven to
 *     their asserted level while still inputs, so switching them to outputs
 *     cannot glitch the module awake.
 *
 * Why the SCCB reads are two transactions by default: OmniVision's SCCB is
 * I2C-like but documents a read as "write the sub-address and STOP, then a
 * separate read", not as a repeated START.  HAL_I2C_Mem_Read emits a repeated
 * START, which most parts tolerate but not all.  Both forms are selectable
 * (struct camera_tuning::sccb_style) because "the address ACKs but every read
 * returns 0xFF" is otherwise an unfalsifiable dead end.
 *
 * Concurrency: one TX_MUTEX serializes every public call for the whole
 * operation; the work lives in *_locked() helpers so a public entry never
 * re-takes the mutex it already holds.  No interrupt is enabled by this phase.
 *
 * Clean-room implementation; RM0468, the schematic and the OmniVision SCCB
 * description used as a register / wiring reference only.
 */
#include "camera.h"

#include "stm32h7xx_hal.h"
#include "timebase.h"   /* udelay: DWT-based, used by the bit-banged bus clear */
#include "tx_api.h"

#define LOG_TAG "cam"
#include "log.h"

#include <string.h>

/* ---- pins ---------------------------------------------------------------- */

#define CAM_XCLK_PORT     GPIOA
#define CAM_XCLK_PIN      GPIO_PIN_2
#define CAM_XCLK_AF       GPIO_AF2_TIM5

#define CAM_PWDN_PORT     GPIOE
#define CAM_PWDN_PIN      GPIO_PIN_7

#define CAM_RST_PORT      GPIOH
#define CAM_RST_PIN       GPIO_PIN_12

#define CAM_SCL_PORT      GPIOF
#define CAM_SCL_PIN       GPIO_PIN_14
#define CAM_SDA_PORT      GPIOF
#define CAM_SDA_PIN       GPIO_PIN_15
#define CAM_I2C_AF        GPIO_AF4_I2C4

/* ---- XCLK ---------------------------------------------------------------- */

#define CAM_TIM           TIM5
#define CAM_XCLK_DIV_DEF  12u    /* 275 MHz / 12 = 22.917 MHz, exact 50% duty */
/* Divider bounds.  6 -> 45.8 MHz and 69 -> 3.99 MHz off a 275 MHz kernel clock;
   the range brackets every DVP sensor's XVCLK window with margin. */
#define CAM_XCLK_DIV_MIN  6u
#define CAM_XCLK_DIV_MAX  69u

/* ---- SCCB ---------------------------------------------------------------- */

#define CAM_I2C           I2C4
#define CAM_I2C_TIMEOUT   100u   /* ms per HAL_I2C transfer */

/*
 * TIMINGR values for a 137.5 MHz kernel clock, PRESC = 15 (tPRESC = 116.36 ns):
 *
 *   fields: PRESC[31:28] SCLDEL[23:20] SDADEL[19:16] SCLH[15:8] SCLL[7:0]
 *
 *   100 kHz: SCLL=40, SCLH=44 -> tLOW 4.77 us + tHIGH 5.24 us = 99.93 kHz
 *    50 kHz: SCLL=81, SCLH=89 -> 9.54 us + 10.47 us           = 49.96 kHz
 *   400 kHz: SCLL=12, SCLH= 8 -> 1.51 us +  1.05 us           = 390.6 kHz
 *
 * Those are the register-derived periods; the bus runs slightly slower because
 * the peripheral adds its own synchronisation delay to each edge.
 *
 * cam_i2c_init_locked() re-checks the kernel clock at run time and warns if it
 * is not 137.5 MHz, because these constants would then be silently wrong.
 */
#define CAM_I2C_KER_EXPECTED  137500000u
#define CAM_I2C_TIMING_50K    0xF0425951u
#define CAM_I2C_TIMING_100K   0xF0422C28u
#define CAM_I2C_TIMING_400K   0xF031080Cu

/* Power-up delays (ms).  OmniVision's module application notes ask for ~10 ms
   between applying XCLK and releasing PWDN, and again before releasing RESETB;
   the final settle covers the sensor's internal reset before SCCB is usable.
   Generous on purpose: probe is a human-typed command, so tens of milliseconds
   cost nothing, whereas being short here produces "SCCB does not answer", the
   hardest symptom in this whole bring-up to attribute. */
#define CAM_T_XCLK_TO_PWDN  10u
#define CAM_T_PWDN_TO_RST   10u
#define CAM_T_RST_TO_SCCB   20u

/* ---- sensor identification ----------------------------------------------- */

/* OV2640: 7-bit SCCB address 0x30, 8-bit registers, banked.  Register 0xFF
   (RA_DLMT) selects the bank; bank 1 holds the ID registers. */
#define OV2640_ADDR        0x30u
#define OV2640_REG_BANK    0xFFu
#define OV2640_BANK_SENSOR 0x01u
#define OV2640_REG_PIDH    0x0Au
#define OV2640_REG_PIDL    0x0Bu
#define OV2640_REG_MIDH    0x1Cu
#define OV2640_REG_MIDL    0x1Du
#define OV2640_PIDH        0x26u
#define OV2640_VER_A       0x41u
#define OV2640_VER_B       0x42u
#define OV2640_MID         0x7FA2u

/* OV5640: 7-bit address 0x3C, 16-bit registers, chip ID 0x5640 at 0x300A. */
#define OV5640_ADDR        0x3Cu
#define OV5640_REG_IDH     0x300Au
#define OV5640_REG_IDL     0x300Bu
#define OV5640_ID          0x5640u

/* ---- state --------------------------------------------------------------- */

static TX_MUTEX cam_lock;
static I2C_HandleTypeDef hcam_i2c;

static struct camera_info info;
static struct camera_tuning tune = {
	.pwdn_active_high = 1u,
	.rst_active_low   = 1u,
	.i2c_pullup       = 0u,
	.sccb_style       = CAM_SCCB_SPLIT,
	.i2c_hz           = 100000u,
};
static uint32_t cam_xclk_div = CAM_XCLK_DIV_DEF;

/* ---- locking ------------------------------------------------------------- */

static int op_lock(void)
{
	if (!info.ready)
		return CAM_ERR_STATE;
	if (tx_mutex_get(&cam_lock, TX_WAIT_FOREVER) != TX_SUCCESS)
		return CAM_ERR_STATE;
	return CAM_OK;
}

static void op_unlock(void)
{
	(void)tx_mutex_put(&cam_lock);
}

/* ---- XCLK ---------------------------------------------------------------- */

/*
 * rcc_tim5_ker_ck.  RM0468 sec 8.7.6 (RCC_CFGR.TIMPRE) / sec 8.7.8 (D2CFGR.D2PPRE1):
 * with an APB prescaler of 1 the timer clock equals PCLK; otherwise it is
 * 2 x PCLK (TIMPRE = 0) or 4 x PCLK (TIMPRE = 1), in both cases capped at HCLK.
 * The bootloader leaves TIMPRE = 0 and D2PPRE1 = /2, so this returns
 * 2 x 137.5 MHz = 275 MHz -- but it is computed, not assumed.
 */
static uint32_t cam_tim_ker_hz(void)
{
	uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
	uint32_t hclk  = HAL_RCC_GetHCLKFreq();
	uint32_t mul   = ((RCC->CFGR & RCC_CFGR_TIMPRE) != 0u) ? 4u : 2u;

	if (pclk1 == hclk)
		return hclk;
	return (pclk1 * mul > hclk) ? hclk : pclk1 * mul;
}

static uint32_t xclk_hz_for(uint32_t div)
{
	return (div != 0u) ? cam_tim_ker_hz() / div : 0u;
}

/*
 * Program the divider, stopping the counter across the update.
 *
 * ARR has no preload here (CR1.ARPE = 0), so shrinking it while the counter is
 * above the new value would let a 32-bit CNT run all the way to 0xFFFFFFFF
 * before wrapping -- a ~15 second stuck output level, not a glitch.  Stopping,
 * clearing CNT and forcing the update event makes the retune bounded.
 */
static void xclk_apply_div(uint32_t div)
{
	uint32_t cr1 = CAM_TIM->CR1;

	CAM_TIM->CR1  = cr1 & ~TIM_CR1_CEN;
	CAM_TIM->ARR  = div - 1u;
	CAM_TIM->CCR3 = div / 2u;
	CAM_TIM->CNT  = 0u;
	CAM_TIM->EGR  = TIM_EGR_UG;
	CAM_TIM->CR1  = cr1;
}

static void xclk_start_locked(void)
{
	GPIO_InitTypeDef g = {0};

	__HAL_RCC_TIM5_CLK_ENABLE();
	/* Keep the timer clocked in CSleep so idle WFI (issue #2) does not stop the
	   sensor's master clock.  TIM5LPEN resets to 1 (RM0468 sec 8.7.53); setting it
	   explicitly documents the dependency instead of relying on a default. */
	__HAL_RCC_TIM5_CLK_SLEEP_ENABLE();

	CAM_TIM->CR1   = 0u;
	CAM_TIM->PSC   = 0u;
	/* PWM mode 1 on channel 3 with output preload.  TIM5 is a general-purpose
	   timer: there is no BDTR/MOE to release. */
	CAM_TIM->CCMR2 = (6u << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;
	CAM_TIM->CCER  = TIM_CCER_CC3E;
	CAM_TIM->CNT   = 0u;
	xclk_apply_div(cam_xclk_div);
	CAM_TIM->CR1   = TIM_CR1_CEN;

	g.Pin       = CAM_XCLK_PIN;
	g.Mode      = GPIO_MODE_AF_PP;
	g.Pull      = GPIO_NOPULL;
	g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
	g.Alternate = CAM_XCLK_AF;
	HAL_GPIO_Init(CAM_XCLK_PORT, &g);

	info.xclk_hz = xclk_hz_for(cam_xclk_div);
}

static void xclk_stop_locked(void)
{
	GPIO_InitTypeDef g = {0};

	/* Park the pin as a driven low BEFORE stopping the timer: disabling the
	   channel output would otherwise leave PA2 floating, and a floating clock
	   input on the module is not a defined state. */
	HAL_GPIO_WritePin(CAM_XCLK_PORT, CAM_XCLK_PIN, GPIO_PIN_RESET);
	g.Pin   = CAM_XCLK_PIN;
	g.Mode  = GPIO_MODE_OUTPUT_PP;
	g.Pull  = GPIO_NOPULL;
	g.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(CAM_XCLK_PORT, &g);

	/* A write to a peripheral whose APB gate is off is silently dropped, so
	   check before touching TIM5: camera_off() / camera_probe() both run this
	   path before the timer has ever been started. */
	if (__HAL_RCC_TIM5_IS_CLK_ENABLED()) {
		CAM_TIM->CR1  = 0u;
		CAM_TIM->CCER = 0u;
	}
	info.xclk_hz = 0u;
}

/* ---- power / reset lines ------------------------------------------------- */

static GPIO_PinState pwdn_level(int assert_powerdown)
{
	int high = tune.pwdn_active_high ? assert_powerdown : !assert_powerdown;

	return high ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

static GPIO_PinState rst_level(int assert_reset)
{
	int high = tune.rst_active_low ? !assert_reset : assert_reset;

	return high ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

static void power_off_locked(void)
{
	HAL_GPIO_WritePin(CAM_PWDN_PORT, CAM_PWDN_PIN, pwdn_level(1));
	HAL_GPIO_WritePin(CAM_RST_PORT, CAM_RST_PIN, rst_level(1));
	xclk_stop_locked();

	info.powered   = 0u;
	info.sensor    = CAM_SENSOR_NONE;
	info.i2c_addr  = 0u;
	info.reg_width = 0u;
	info.chip_id   = 0u;
	info.manuf_id  = 0u;
}

static void power_on_locked(void)
{
	xclk_start_locked();
	tx_thread_sleep(CAM_T_XCLK_TO_PWDN);
	HAL_GPIO_WritePin(CAM_PWDN_PORT, CAM_PWDN_PIN, pwdn_level(0));
	tx_thread_sleep(CAM_T_PWDN_TO_RST);
	HAL_GPIO_WritePin(CAM_RST_PORT, CAM_RST_PIN, rst_level(0));
	tx_thread_sleep(CAM_T_RST_TO_SCCB);
	info.powered = 1u;
}

/* ---- SCCB ---------------------------------------------------------------- */

static uint32_t i2c_timing_for(uint32_t hz)
{
	if (hz <= 50000u)
		return CAM_I2C_TIMING_50K;
	if (hz >= 400000u)
		return CAM_I2C_TIMING_400K;
	return CAM_I2C_TIMING_100K;
}

static int cam_i2c_init_locked(void)
{
	GPIO_InitTypeDef g = {0};

	__HAL_RCC_GPIOF_CLK_ENABLE();
	/* Re-state the RESET value of RCC_D3CCIPR.I2C4SEL (rcc_pclk4).  This changes
	   nothing -- it documents which kernel clock the TIMINGR constants above were
	   derived for.  Same idiom as port/sd/sd_card.c. */
	__HAL_RCC_I2C4_CONFIG(RCC_I2C4CLKSOURCE_D3PCLK1);
	__HAL_RCC_I2C4_CLK_ENABLE();

	info.i2c_ker_hz = (uint32_t)HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_I2C4);
	if (info.i2c_ker_hz != CAM_I2C_KER_EXPECTED)
		LOG_WRN("I2C4 kernel clock %lu Hz, not %lu -- SCCB bit rate will be off",
		        (unsigned long)info.i2c_ker_hz,
		        (unsigned long)CAM_I2C_KER_EXPECTED);

	g.Pin       = CAM_SCL_PIN | CAM_SDA_PIN;
	g.Mode      = GPIO_MODE_AF_OD;
	g.Pull      = tune.i2c_pullup ? GPIO_PULLUP : GPIO_NOPULL;
	g.Speed     = GPIO_SPEED_FREQ_HIGH;
	g.Alternate = CAM_I2C_AF;
	HAL_GPIO_Init(CAM_SCL_PORT, &g);

	if (hcam_i2c.Instance != NULL)
		(void)HAL_I2C_DeInit(&hcam_i2c);

	hcam_i2c.Instance              = CAM_I2C;
	hcam_i2c.Init.Timing           = i2c_timing_for(tune.i2c_hz);
	hcam_i2c.Init.OwnAddress1      = 0u;
	hcam_i2c.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
	hcam_i2c.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
	hcam_i2c.Init.OwnAddress2      = 0u;
	hcam_i2c.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hcam_i2c.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
	hcam_i2c.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hcam_i2c) != HAL_OK) {
		LOG_ERR("I2C4 init failed");
		return CAM_ERR_HAL;
	}
	return CAM_OK;
}

/*
 * Bit-bang the bus out of a stuck state: a slave interrupted mid-byte holds SDA
 * low forever, and no amount of re-initialising the peripheral makes it let go.
 * Nine SCL pulses walk it to the end of its byte, then a manual STOP (SDA rising
 * while SCL is high) resynchronises it.  The I2C peripheral has to be released
 * from the pins first, so the caller must re-init it afterwards.
 */
#define CAM_I2C_CLEAR_HALF_US 5u   /* ~100 kHz while bit-banging */

static void i2c_bus_clear_locked(void)
{
	GPIO_InitTypeDef g = {0};

	if (hcam_i2c.Instance != NULL) {
		(void)HAL_I2C_DeInit(&hcam_i2c);
		hcam_i2c.Instance = NULL;
	}

	HAL_GPIO_WritePin(CAM_SCL_PORT, CAM_SCL_PIN | CAM_SDA_PIN, GPIO_PIN_SET);
	g.Pin   = CAM_SCL_PIN | CAM_SDA_PIN;
	g.Mode  = GPIO_MODE_OUTPUT_OD;
	g.Pull  = tune.i2c_pullup ? GPIO_PULLUP : GPIO_NOPULL;
	g.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(CAM_SCL_PORT, &g);

	for (unsigned i = 0u; i < 9u; i++) {
		if (HAL_GPIO_ReadPin(CAM_SDA_PORT, CAM_SDA_PIN) == GPIO_PIN_SET)
			break;
		HAL_GPIO_WritePin(CAM_SCL_PORT, CAM_SCL_PIN, GPIO_PIN_RESET);
		udelay(CAM_I2C_CLEAR_HALF_US);
		HAL_GPIO_WritePin(CAM_SCL_PORT, CAM_SCL_PIN, GPIO_PIN_SET);
		udelay(CAM_I2C_CLEAR_HALF_US);
	}
	/* STOP: SDA low with SCL low, raise SCL, then release SDA. */
	HAL_GPIO_WritePin(CAM_SCL_PORT, CAM_SCL_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(CAM_SDA_PORT, CAM_SDA_PIN, GPIO_PIN_RESET);
	udelay(CAM_I2C_CLEAR_HALF_US);
	HAL_GPIO_WritePin(CAM_SCL_PORT, CAM_SCL_PIN, GPIO_PIN_SET);
	udelay(CAM_I2C_CLEAR_HALF_US);
	HAL_GPIO_WritePin(CAM_SDA_PORT, CAM_SDA_PIN, GPIO_PIN_SET);
	udelay(CAM_I2C_CLEAR_HALF_US);
}

/*
 * A stuck-low SDA -- exactly what a wedged sensor or a missing pull-up produces --
 * leaves the peripheral BUSY, after which every transfer fails instantly and the
 * bring-up just looks like "the bus does nothing".  Every entry point that talks
 * to the bus passes through here first.
 *
 * Both conditions are checked: the HAL's software state (which a timed-out
 * transfer leaves non-READY) AND ISR.BUSY, which reflects the wire and can be set
 * while the HAL still believes it is idle -- checking only the former is what
 * makes a stuck bus look unrecoverable.
 */
static void i2c_ensure_ready_locked(void)
{
	int hal_busy = (HAL_I2C_GetState(&hcam_i2c) != HAL_I2C_STATE_READY);
	int wire_busy = (hcam_i2c.Instance != NULL) &&
	                ((hcam_i2c.Instance->ISR & I2C_ISR_BUSY) != 0u);

	if (!hal_busy && !wire_busy)
		return;

	LOG_WRN("I2C4 stuck (state 0x%02x, err 0x%08lx, ISR 0x%08lx) -- clearing bus",
	        (unsigned)HAL_I2C_GetState(&hcam_i2c),
	        (unsigned long)HAL_I2C_GetError(&hcam_i2c),
	        (unsigned long)((hcam_i2c.Instance != NULL) ? hcam_i2c.Instance->ISR
	                                                    : 0u));
	i2c_bus_clear_locked();
	(void)cam_i2c_init_locked();
}

/* One address probe, shared by camera_scan() and the post-probe fallback. */
static int addr_acks_locked(uint8_t addr7)
{
	return HAL_I2C_IsDeviceReady(&hcam_i2c, (uint16_t)(addr7 << 1), 1u, 5u)
	       == HAL_OK;
}

/* Register address bytes, MSB first for the 16-bit form. */
static unsigned reg_bytes(uint16_t reg, unsigned width, uint8_t *buf)
{
	if (width == CAM_REG_WIDTH_16) {
		buf[0] = (uint8_t)(reg >> 8);
		buf[1] = (uint8_t)reg;
		return 2u;
	}
	buf[0] = (uint8_t)reg;
	return 1u;
}

static int sccb_write_locked(uint8_t addr7, uint16_t reg, unsigned width,
                             uint8_t val)
{
	uint8_t buf[3];
	unsigned n = reg_bytes(reg, width, buf);

	buf[n++] = val;
	if (HAL_I2C_Master_Transmit(&hcam_i2c, (uint16_t)(addr7 << 1), buf,
	                            (uint16_t)n, CAM_I2C_TIMEOUT) != HAL_OK)
		return CAM_ERR_HAL;
	return CAM_OK;
}

static int sccb_read_locked(uint8_t addr7, uint16_t reg, unsigned width,
                            uint8_t *val)
{
	uint8_t buf[2];
	unsigned n = reg_bytes(reg, width, buf);

	if (tune.sccb_style == CAM_SCCB_RESTART) {
		if (HAL_I2C_Mem_Read(&hcam_i2c, (uint16_t)(addr7 << 1), reg,
		                     (width == CAM_REG_WIDTH_16)
		                             ? I2C_MEMADD_SIZE_16BIT
		                             : I2C_MEMADD_SIZE_8BIT,
		                     val, 1u, CAM_I2C_TIMEOUT) != HAL_OK)
			return CAM_ERR_HAL;
		return CAM_OK;
	}

	/* SCCB two-phase read: sub-address write terminated by a STOP, then an
	   independent read transaction. */
	if (HAL_I2C_Master_Transmit(&hcam_i2c, (uint16_t)(addr7 << 1), buf,
	                            (uint16_t)n, CAM_I2C_TIMEOUT) != HAL_OK)
		return CAM_ERR_HAL;
	if (HAL_I2C_Master_Receive(&hcam_i2c, (uint16_t)(addr7 << 1), val, 1u,
	                           CAM_I2C_TIMEOUT) != HAL_OK)
		return CAM_ERR_HAL;
	return CAM_OK;
}

/* ---- identification ------------------------------------------------------ */

static int ident_ov2640_locked(void)
{
	uint8_t pidh, pidl, midh, midl;

	if (sccb_write_locked(OV2640_ADDR, OV2640_REG_BANK, CAM_REG_WIDTH_8,
	                      OV2640_BANK_SENSOR) != CAM_OK)
		return CAM_ERR_NO_SENSOR;
	if (sccb_read_locked(OV2640_ADDR, OV2640_REG_PIDH, CAM_REG_WIDTH_8, &pidh) != CAM_OK ||
	    sccb_read_locked(OV2640_ADDR, OV2640_REG_PIDL, CAM_REG_WIDTH_8, &pidl) != CAM_OK ||
	    sccb_read_locked(OV2640_ADDR, OV2640_REG_MIDH, CAM_REG_WIDTH_8, &midh) != CAM_OK ||
	    sccb_read_locked(OV2640_ADDR, OV2640_REG_MIDL, CAM_REG_WIDTH_8, &midl) != CAM_OK)
		return CAM_ERR_NO_SENSOR;

	/*
	 * PIDH alone would let any device that happens to answer 0x26 at register
	 * 0x0A pass as an OV2640, so a second, independent field has to agree: either
	 * the OmniVision manufacturer ID or one of the two documented version bytes.
	 * Requiring BOTH would be stricter than the parts warrant -- VER is 0x41 on
	 * some lots and 0x42 on others, and clones are inconsistent about MID.
	 */
	if (pidh != OV2640_PIDH ||
	    (((uint16_t)midh << 8 | midl) != OV2640_MID &&
	     pidl != OV2640_VER_A && pidl != OV2640_VER_B)) {
		LOG_WRN("0x%02x answered but PID 0x%02x%02x / MID 0x%02x%02x is not OV2640",
		        OV2640_ADDR, pidh, pidl, midh, midl);
		return CAM_ERR_NO_SENSOR;
	}
	info.sensor    = CAM_SENSOR_OV2640;
	info.i2c_addr  = OV2640_ADDR;
	info.reg_width = CAM_REG_WIDTH_8;
	info.chip_id   = (uint16_t)((uint16_t)pidh << 8 | pidl);
	info.manuf_id  = (uint16_t)((uint16_t)midh << 8 | midl);
	if (info.manuf_id != OV2640_MID)
		LOG_WRN("OV2640 PID matched but MID is 0x%04x, not 0x%04x",
		        info.manuf_id, OV2640_MID);
	if (pidl != OV2640_VER_A && pidl != OV2640_VER_B)
		LOG_WRN("OV2640 MID matched but VER is 0x%02x, not 0x%02x/0x%02x",
		        pidl, OV2640_VER_A, OV2640_VER_B);
	return CAM_OK;
}

static int ident_ov5640_locked(void)
{
	uint8_t idh, idl;
	uint16_t id;

	if (sccb_read_locked(OV5640_ADDR, OV5640_REG_IDH, CAM_REG_WIDTH_16, &idh) != CAM_OK ||
	    sccb_read_locked(OV5640_ADDR, OV5640_REG_IDL, CAM_REG_WIDTH_16, &idl) != CAM_OK)
		return CAM_ERR_NO_SENSOR;

	id = (uint16_t)((uint16_t)idh << 8 | idl);
	if (id != OV5640_ID) {
		LOG_WRN("0x%02x answered but chip ID 0x%04x is not OV5640",
		        OV5640_ADDR, id);
		return CAM_ERR_NO_SENSOR;
	}
	info.sensor    = CAM_SENSOR_OV5640;
	info.i2c_addr  = OV5640_ADDR;
	info.reg_width = CAM_REG_WIDTH_16;
	info.chip_id   = id;
	info.manuf_id  = 0u;
	return CAM_OK;
}

/* Any device on the bus, so an unidentified module still reports *something*
   actionable rather than a bare "no sensor". */
static uint8_t first_acking_addr_locked(void)
{
	for (uint8_t a = 0x08u; a <= 0x77u; a++) {
		if (addr_acks_locked(a))
			return a;
	}
	return 0u;
}

/* ---- public API ---------------------------------------------------------- */

int camera_init(void)
{
	GPIO_InitTypeDef g = {0};
	int rc;

	if (info.ready)
		return CAM_OK;

	if (tx_mutex_create(&cam_lock, "camera", TX_INHERIT) != TX_SUCCESS)
		return CAM_ERR_STATE;

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();

	/* Drive the asserted level into ODR while the pins are still inputs, then
	   switch them to outputs: the module can never see a glitch that wakes it. */
	HAL_GPIO_WritePin(CAM_PWDN_PORT, CAM_PWDN_PIN, pwdn_level(1));
	HAL_GPIO_WritePin(CAM_RST_PORT, CAM_RST_PIN, rst_level(1));
	g.Mode  = GPIO_MODE_OUTPUT_PP;
	g.Pull  = GPIO_NOPULL;
	g.Speed = GPIO_SPEED_FREQ_LOW;
	g.Pin   = CAM_PWDN_PIN;
	HAL_GPIO_Init(CAM_PWDN_PORT, &g);
	g.Pin   = CAM_RST_PIN;
	HAL_GPIO_Init(CAM_RST_PORT, &g);

	/* XCLK parked low (timer stopped) until camera_on(). */
	HAL_GPIO_WritePin(CAM_XCLK_PORT, CAM_XCLK_PIN, GPIO_PIN_RESET);
	g.Pin = CAM_XCLK_PIN;
	HAL_GPIO_Init(CAM_XCLK_PORT, &g);

	/* The _locked helpers are called here without holding cam_lock, which is
	   correct exactly once: camera_init() runs from tx_application_define()
	   before the scheduler exists, so no other thread can be inside them. */
	rc = cam_i2c_init_locked();
	if (rc != CAM_OK) {
		(void)tx_mutex_delete(&cam_lock);
		return rc;
	}

	info.xclk_src_hz = cam_tim_ker_hz();
	info.ready       = 1u;
	LOG_INF("ready: XCLK src %lu Hz, I2C4 ker %lu Hz",
	        (unsigned long)info.xclk_src_hz, (unsigned long)info.i2c_ker_hz);
	return CAM_OK;
}

int camera_on(void)
{
	int rc = op_lock();

	if (rc != CAM_OK)
		return rc;
	if (!info.powered)
		power_on_locked();
	op_unlock();
	return CAM_OK;
}

int camera_off(void)
{
	int rc = op_lock();

	if (rc != CAM_OK)
		return rc;
	power_off_locked();
	op_unlock();
	return CAM_OK;
}

int camera_probe(void)
{
	int rc = op_lock();

	if (rc != CAM_OK)
		return rc;

	power_off_locked();
	power_on_locked();
	i2c_ensure_ready_locked();

	rc = ident_ov2640_locked();
	if (rc != CAM_OK)
		rc = ident_ov5640_locked();

	if (rc != CAM_OK) {
		/* Leave the sensor powered: the supply cannot be cut anyway, and a live
		   bus is what lets `camera scan` / `camera reg` continue the hunt. */
		uint8_t a = first_acking_addr_locked();

		info.sensor    = (a != 0u) ? CAM_SENSOR_UNKNOWN : CAM_SENSOR_NONE;
		info.i2c_addr  = a;
		info.reg_width = 0u;
		info.chip_id   = 0u;
		info.manuf_id  = 0u;
		if (a != 0u)
			LOG_WRN("unidentified device at SCCB 0x%02x", a);
		else
			LOG_WRN("no SCCB response (module connected? XCLK %lu Hz)",
			        (unsigned long)info.xclk_hz);
		op_unlock();
		return CAM_ERR_NO_SENSOR;
	}

	LOG_INF("%s up: chip ID 0x%04x at SCCB 0x%02x",
	        camera_sensor_name(info.sensor), info.chip_id, info.i2c_addr);
	op_unlock();
	return CAM_OK;
}

int camera_get_info(struct camera_info *out)
{
	if (out == NULL)
		return CAM_ERR_PARAM;
	if (!info.ready) {
		memset(out, 0, sizeof *out);
		return CAM_ERR_STATE;
	}
	if (tx_mutex_get(&cam_lock, TX_WAIT_FOREVER) != TX_SUCCESS)
		return CAM_ERR_STATE;
	*out = info;
	op_unlock();
	return CAM_OK;
}

int camera_scan(uint8_t *addrs, unsigned max, unsigned *found)
{
	unsigned n = 0u;
	int rc = op_lock();

	if (rc != CAM_OK)
		return rc;
	if (!info.powered) {
		op_unlock();
		return CAM_ERR_STATE;
	}
	i2c_ensure_ready_locked();
	for (uint8_t a = 0x08u; a <= 0x77u; a++) {
		if (!addr_acks_locked(a))
			continue;
		if (addrs != NULL && n < max)
			addrs[n] = a;
		n++;
	}
	op_unlock();
	if (found != NULL)
		*found = n;
	return CAM_OK;
}

int camera_reg_read(uint8_t i2c_addr, uint16_t reg, unsigned reg_width,
                    uint8_t *val)
{
	int rc;

	if (val == NULL || i2c_addr > 0x7Fu ||
	    (reg_width != CAM_REG_WIDTH_8 && reg_width != CAM_REG_WIDTH_16))
		return CAM_ERR_PARAM;
	rc = op_lock();
	if (rc != CAM_OK)
		return rc;
	if (!info.powered) {
		op_unlock();
		return CAM_ERR_STATE;
	}
	i2c_ensure_ready_locked();
	rc = sccb_read_locked(i2c_addr, reg, reg_width, val);
	op_unlock();
	return rc;
}

int camera_reg_write(uint8_t i2c_addr, uint16_t reg, unsigned reg_width,
                     uint8_t val)
{
	int rc;

	if (i2c_addr > 0x7Fu ||
	    (reg_width != CAM_REG_WIDTH_8 && reg_width != CAM_REG_WIDTH_16))
		return CAM_ERR_PARAM;
	rc = op_lock();
	if (rc != CAM_OK)
		return rc;
	if (!info.powered) {
		op_unlock();
		return CAM_ERR_STATE;
	}
	i2c_ensure_ready_locked();
	rc = sccb_write_locked(i2c_addr, reg, reg_width, val);
	op_unlock();
	return rc;
}

int camera_set_xclk(uint32_t hz, uint32_t *actual)
{
	uint32_t src, div;
	int rc;

	if (hz == 0u)
		return CAM_ERR_PARAM;
	rc = op_lock();
	if (rc != CAM_OK)
		return rc;

	src = cam_tim_ker_hz();
	div = (src + hz / 2u) / hz;          /* nearest integer divider */
	if (div < CAM_XCLK_DIV_MIN)
		div = CAM_XCLK_DIV_MIN;
	if (div > CAM_XCLK_DIV_MAX)
		div = CAM_XCLK_DIV_MAX;

	cam_xclk_div     = div;
	info.xclk_src_hz = src;
	if (info.xclk_hz != 0u) {            /* running: retune in place */
		xclk_apply_div(div);
		info.xclk_hz = xclk_hz_for(div);
	}
	op_unlock();
	if (actual != NULL)
		*actual = src / div;
	return CAM_OK;
}

/*
 * The shell's tune leaves do get -> modify -> set, which is not atomic across
 * the two calls.  With two consoles live (USB CDC and telnet) that is a
 * last-writer-wins race on a bring-up knob, which is acceptable and deliberately
 * not papered over with a read-modify-write API; the copies themselves are
 * serialized so neither side ever observes a torn struct.
 */
int camera_get_tuning(struct camera_tuning *out)
{
	int rc;

	if (out == NULL)
		return CAM_ERR_PARAM;
	rc = op_lock();
	if (rc != CAM_OK)
		return rc;
	*out = tune;
	op_unlock();
	return CAM_OK;
}

int camera_set_tuning(const struct camera_tuning *in)
{
	int i2c_changed;
	int rc;

	if (in == NULL)
		return CAM_ERR_PARAM;
	if (in->sccb_style != CAM_SCCB_SPLIT && in->sccb_style != CAM_SCCB_RESTART)
		return CAM_ERR_PARAM;
	if (in->i2c_hz != 50000u && in->i2c_hz != 100000u && in->i2c_hz != 400000u)
		return CAM_ERR_PARAM;

	rc = op_lock();
	if (rc != CAM_OK)
		return rc;

	i2c_changed = (in->i2c_hz != tune.i2c_hz) ||
	              (in->i2c_pullup != tune.i2c_pullup);
	tune = *in;
	if (i2c_changed)
		rc = cam_i2c_init_locked();
	op_unlock();
	return rc;
}

const char *cam_strerror(int rc)
{
	switch (rc) {
	case CAM_OK:             return "ok";
	case CAM_ERR_PARAM:      return "bad argument";
	case CAM_ERR_HAL:        return "SCCB transfer failed";
	case CAM_ERR_TIMEOUT:    return "timeout";
	case CAM_ERR_STATE:      return "camera not initialized or not powered";
	case CAM_ERR_NO_SENSOR:  return "no sensor identified";
	default:                 return "unknown error";
	}
}

const char *camera_sensor_name(unsigned sensor)
{
	switch (sensor) {
	case CAM_SENSOR_OV2640:  return "OV2640";
	case CAM_SENSOR_OV5640:  return "OV5640";
	case CAM_SENSOR_UNKNOWN: return "unknown";
	default:                 return "none";
	}
}
