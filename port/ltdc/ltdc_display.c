/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    ltdc_display.c
 * @brief   FPC-40 RGB panel bring-up via LTDC + DMA2D (issue #7).
 *
 * See ltdc_display.h for the API contract and, more importantly, for why the
 * pixel-clock path is hand-rolled instead of using the HAL (it would rewrite the
 * whole of PLL3, which is also the USB console's clock).
 *
 * Board wiring (schematic sheet 5/7, cross-checked against ST's own pin data in
 * _ref/stm32_open_pin_data/ -- every ball position matched).  **AF14 for all 28
 * LTDC signals except LTDC_R3 on PA15, which is AF9**:
 *
 *   R0 PH2   R1 PH3   R2 PA1   R3 PA15(AF9)  R4 PA5   R5 PH11  R6 PA8   R7 PE15
 *   G0 PB1   G1 PB0   G2 PH13  G3 PE11       G4 PB10  G5 PB11  G6 PC7   G7 PB15
 *   B0 PG14  B1 PD0   B2 PD6   B3 PD10       B4 PE12  B5 PA3   B6 PB8   B7 PB9
 *   CLK PG7  HSYNC PC6  VSYNC PA7  DE PC5
 *   LCD_RST PG5 (GPIO out)   LCD_BL PF5 (GPIO out)
 *
 * PA15 is JTDI.  Debug on this board is SWD (PA13/PA14) only, so losing JTAG-DI
 * costs nothing; the schematic's choice is deliberate.  PG5 sits next to PG6 =
 * OCTOSPI1_CS, so the port G setup is per-pin (HAL_GPIO_Init does read-modify-
 * write) and never disturbs the PSRAM chip select.
 *
 * **R2 on PA1 is NOT driven by the LTDC.**  It is also the ST7789's CS, and
 * leaving it under the LTDC lets scanned-out red bits clock commands into the
 * panel (issue #43).  It is parked as a GPIO high instead -- ltdc_pin_cs_park()
 * has the full story.  R0/R1 (SDA/SCL) do stay with the LTDC.
 *
 * The panel is a 2.8" 240x320 unit with an ST7789 controller and no published
 * datasheet: its timing, polarity, pixel clock and serial wake-up sequence were
 * all recovered from the board's factory firmware (see st7789_rgb.h), so they
 * are constants here rather than tunables.
 *
 * Clean-room implementation; RM0468 sec 8 (RCC) / sec 38 (LTDC) / sec 39 (DMA2D)
 * used as a register reference only.
 */
#include "ltdc_display.h"
#include "st7789_rgb.h"
#include "gfx_rot.h"   /* the transposing blit (issue #38) */

#include "stm32h7xx_hal.h"
#include "tx_api.h"
#include "tx_glue.h"        /* tx_glue_isr_enter/exit: EPK (issue #2) accounting */

#define LOG_TAG "ltdc"
#include "log.h"

/* Manually-driven panel control pins. */
#define LCD_RST_PORT   GPIOG
#define LCD_RST_PIN    GPIO_PIN_5
#define LCD_BL_PORT    GPIOF
#define LCD_BL_PIN     GPIO_PIN_5

/*
 * Panel timing, transcribed from the board's factory Arduino firmware (see
 * st7789_rgb.h -- the register values were recovered by disassembling
 * `_ref/wio_APP_0x70000000_*.bin`, because no datasheet for this panel exists).
 * The image programmed the LTDC with the "minus one" register values
 * HSW=9 / AHBP=19 / AAW=259 / TW=297 and VSH=3 / AVBP=7 / AAH=327 / TH=335,
 * which decode to the spec numbers below.  Frame = 298 x 336 = 100128 clocks;
 * at the 6.00 MHz pixel clock that same image programmed (PLL3 M5 N48 -> VCO
 * 240 MHz, R=40) that is ~59.9 Hz.
 */
#define LTDC_DEF_W    240u
#define LTDC_DEF_H    320u
#define LTDC_DEF_HS    10u
#define LTDC_DEF_HBP   10u
#define LTDC_DEF_HFP   38u
#define LTDC_DEF_VS     4u
#define LTDC_DEF_VBP    4u
#define LTDC_DEF_VFP    8u

/*
 * Double buffer in the external OCTOSPI1 PSRAM: 2 x 76800 x 2 B = 300 KB.
 * .psram_noinit is NOLOAD and the MPU maps the window Normal non-cacheable +
 * shareable (app/mpu.c region 0), so the CPU, the DMA2D and the LTDC read DMA
 * need no cache maintenance between them.  32-byte aligned: not required for
 * coherency here (the region is uncached) but it keeps every DMA2D burst and the
 * LTDC's AXI reads on cache-line boundaries.
 */
static uint16_t ltdc_fb[2][LTDC_FB_PIXELS]
	__attribute__((aligned(32), section(".psram_noinit.ltdc")));

static LTDC_HandleTypeDef  hltdc;
static DMA2D_HandleTypeDef hdma2d;

static const struct ltdc_panel ltdc_cfg = {
	.w  = LTDC_DEF_W,   .h   = LTDC_DEF_H,
	.hs = LTDC_DEF_HS,  .hbp = LTDC_DEF_HBP, .hfp = LTDC_DEF_HFP,
	.vs = LTDC_DEF_VS,  .vbp = LTDC_DEF_VBP, .vfp = LTDC_DEF_VFP,
};

/* Everything the old runtime validation used to check, now that the timing is a
 * constant: the buffer must back the active area; HSW/VSH are encoded as
 * width-1 and the total must exceed the accumulated active area, so a zero sync
 * width or front porch is not representable; and the LTDC_TWCR/AWCR counters are
 * bounded (12-bit horizontally, 11-bit vertically -- RM0468 sec 38.7.1-38.7.4).
 * LTDC_CLK_DIV likewise has to fit DIVR3's /1../128 (sec 8.7.16). */
_Static_assert((uint32_t)LTDC_DEF_W * LTDC_DEF_H <= LTDC_FB_PIXELS,
               "panel active area exceeds the frame buffer");
_Static_assert(LTDC_DEF_HS > 0u && LTDC_DEF_VS > 0u &&
               LTDC_DEF_HFP > 0u && LTDC_DEF_VFP > 0u,
               "sync widths and front porches must be >= 1");
/* The two axes do NOT share a counter width: RM0468 sec 38.7.1-38.7.4 make every
   horizontal field 12-bit and every vertical field 11-bit, so the vertical bound
   is 2048, not 4096.  (Both current geometries are far under either; this is the
   guard being right rather than lucky.) */
_Static_assert((uint32_t)LTDC_DEF_W + LTDC_DEF_HS + LTDC_DEF_HBP + LTDC_DEF_HFP
                       <= 4096u,
               "total frame width exceeds the LTDC's 12-bit horizontal counters");
_Static_assert((uint32_t)LTDC_DEF_H + LTDC_DEF_VS + LTDC_DEF_VBP + LTDC_DEF_VFP
                       <= 2048u,
               "total frame height exceeds the LTDC's 11-bit vertical counters");
_Static_assert(LTDC_CLK_DIV >= 1u && LTDC_CLK_DIV <= 128u,
               "LTDC_CLK_DIV outside the DIVR3 range");

static uint8_t      ltdc_front;       /* index (0/1) of the displayed buffer  */
static bool         ltdc_up;          /* init succeeded                       */
static bool         ltdc_tried;       /* init ran (idempotence latch)         */
static bool         ltdc_fault;       /* reload stuck -> display latched down */
static bool         ltdc_disabled;    /* scanout stopped (`lcd off`)          */
static TX_SEMAPHORE ltdc_reload_sem;  /* posted by the reload-ready IRQ       */
static TX_MUTEX     ltdc_lock;        /* serializes drawing + flip            */

/* DMA2D interrupt-driven completion.  The engine is single and serialized on
   ltdc_lock, so at most one transfer is armed at a time; dma2d_active is the
   handle DMA2D_IRQHandler dispatches to (NULL between transfers). */
static TX_SEMAPHORE dma2d_done_sem;
static DMA2D_HandleTypeDef *volatile dma2d_active;

/* ==========================================================================
 *  Pixel clock: pll3_r_ck
 * ========================================================================== */

/* Spin budget for the PLL3 stop / relock waits.  This runs before the scheduler
   and before HAL timeouts are trustworthy, so it is a plain bounded loop.  PLL3
   locks in the low hundreds of microseconds; at 550 MHz this budget is several
   milliseconds, i.e. an order of magnitude of slack. */
#define PLL3_SPIN_LIMIT  2000000u

static bool pll3_wait_ready(bool want)
{
	for (uint32_t i = 0; i < PLL3_SPIN_LIMIT; i++) {
		bool rdy = (RCC->CR & RCC_CR_PLL3RDY) != 0u;

		if (rdy == want)
			return true;
	}
	return false;
}

static bool ltdc_clk_up;         /* pll3_r_ck routed to the LTDC and locked */

int ltdc_clock_init(void)
{
	static bool tried;
	static int  result = LTDC_ERR_STATE;
	uint32_t divr, pm;
	bool ok;

	if (tried)
		return result;
	tried = true;

	/* The bootloader is supposed to hand us a running PLL3 (VCO 240 MHz, Q ->
	   48 MHz USB).  If it is not running, something is very wrong upstream:
	   bail out having touched nothing rather than try to build one ourselves --
	   guessing M/N here is exactly the failure mode we are avoiding. */
	if ((RCC->CR & RCC_CR_PLL3ON) == 0u || (RCC->CR & RCC_CR_PLL3RDY) == 0u) {
		LOG_ERR("PLL3 not running -- LTDC clock not configured");
		result = LTDC_ERR_STATE;
		return result;
	}

	/*
	 * Stop PLL3, retune DIVR3, restart.  Interrupts are masked so the window is
	 * deterministic and nothing can observe RCC mid-edit.  This is the ONLY
	 * moment the app touches the clock tree, and it is legal exactly because:
	 *   - RM0468 sec 8.7.16 / 8.7.11: DIVR3 and DIVR3EN are writable only with
	 *     PLL3ON = 0 and PLL3RDY = 0, so the stop is mandatory, not optional;
	 *   - every other field of RCC_PLL3DIVR is written back exactly as read, so
	 *     the bootloader's M/N/P/Q survive bit-for-bit (RCC_PLLCFGR is a
	 *     read-modify-write of DIVR3EN alone, leaving DIVQ3EN for USB alone);
	 *   - FRACN3 is not written, so the PLL3FRACEN 0->1 latch dance of
	 *     RM0468 sec 8.7.17 does not apply;
	 *   - the only consumer of PLL3 at this point in main() is USB, which has
	 *     not been clocked yet, let alone enumerated.
	 */
	pm = __get_PRIMASK();
	__disable_irq();

	divr = RCC->PLL3DIVR;                       /* snapshot N3/P3/Q3 */
	RCC->CR &= ~RCC_CR_PLL3ON;
	ok = pll3_wait_ready(false);
	if (ok) {
		/* The register field is (ratio - 1): DIVR3 = 0 means /1 (sec 8.7.16). */
		RCC->PLL3DIVR = (divr & ~RCC_PLL3DIVR_R3_Msk) |
		                (((LTDC_CLK_DIV - 1u) << RCC_PLL3DIVR_R3_Pos) &
		                 RCC_PLL3DIVR_R3_Msk);
		RCC->PLLCFGR |= RCC_PLLCFGR_DIVR3EN;
	}
	RCC->CR |= RCC_CR_PLL3ON;                   /* restart even if the stop
	                                               timed out: leaving PLL3 off
	                                               would kill USB for good */
	ok = pll3_wait_ready(true) && ok;

	__set_PRIMASK(pm);

	if (!ok) {
		/* USB may or may not have survived; the dmesg ring is in DTCM and
		   outlives the reset, so this line is readable after recovery. */
		LOG_ERR("PLL3 retune failed -- USB clock may be down");
		result = LTDC_ERR_HAL;
		return result;
	}

	ltdc_clk_up = true;
	result = LTDC_OK;
	return result;
}

/* Read the divider back out of the hardware rather than reporting the constant:
   `lcd info` is a diagnostic, and it should show what the PLL is actually doing. */
uint32_t ltdc_clock_div(void)
{
	if (!ltdc_clk_up)
		return 0u;
	return ((RCC->PLL3DIVR & RCC_PLL3DIVR_R3_Msk) >> RCC_PLL3DIVR_R3_Pos) + 1u;
}

uint32_t ltdc_pixel_clock_hz(void)
{
	uint32_t r = ltdc_clock_div();

	return (r == 0u) ? 0u : LTDC_PLL3_VCO_HZ / r;
}

uint32_t ltdc_refresh_chz(void)
{
	uint32_t total_w, total_h, clk;

	if (!ltdc_up)
		return 0u;
	clk = ltdc_pixel_clock_hz();
	total_w = (uint32_t)ltdc_cfg.w + ltdc_cfg.hs + ltdc_cfg.hbp + ltdc_cfg.hfp;
	total_h = (uint32_t)ltdc_cfg.h + ltdc_cfg.vs + ltdc_cfg.vbp + ltdc_cfg.vfp;
	if (total_w == 0u || total_h == 0u)
		return 0u;
	return (uint32_t)(((uint64_t)clk * 100u) / ((uint64_t)total_w * total_h));
}

/* ==========================================================================
 *  State
 * ========================================================================== */

bool ltdc_is_up(void)
{
	return ltdc_up && !ltdc_fault;
}

bool ltdc_scanout_active(void)
{
	return ltdc_is_up() && !ltdc_disabled;
}

bool ltdc_scanout_off(void)
{
	return ltdc_is_up() && ltdc_disabled;
}

void ltdc_panel_get(struct ltdc_panel *out)
{
	if (out != NULL)
		*out = ltdc_cfg;
}

uint16_t *ltdc_framebuffer(void)
{
	return ltdc_is_up() ? &ltdc_fb[ltdc_front][0] : NULL;
}

uint16_t *ltdc_back_buffer(void)
{
	return ltdc_is_up() ? &ltdc_fb[!ltdc_front][0] : NULL;
}

uint8_t ltdc_active_buffer(void)
{
	return ltdc_front;
}

void ltdc_lock_frame(void)
{
	(void)tx_mutex_get(&ltdc_lock, TX_WAIT_FOREVER);
}

void ltdc_unlock_frame(void)
{
	(void)tx_mutex_put(&ltdc_lock);
}

void ltdc_backlight(bool on)
{
	HAL_GPIO_WritePin(LCD_BL_PORT, LCD_BL_PIN,
	                  on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

uint32_t ltdc_errors(bool clear)
{
	uint32_t mask = 0u;

	if (!ltdc_up)
		return 0u;
	if (__HAL_LTDC_GET_FLAG(&hltdc, LTDC_FLAG_FU))
		mask |= LTDC_ERRFLAG_FIFO_UNDERRUN;
	if (__HAL_LTDC_GET_FLAG(&hltdc, LTDC_FLAG_TE))
		mask |= LTDC_ERRFLAG_TRANSFER_ERROR;
	if (clear) {
		__HAL_LTDC_CLEAR_FLAG(&hltdc, LTDC_FLAG_FU);
		__HAL_LTDC_CLEAR_FLAG(&hltdc, LTDC_FLAG_TE);
	}
	return mask;
}

/* Clear both frame buffers with the CPU.  Deliberately not DMA2D: this also runs
   from ltdc_init(), which is pre-scheduler and must not wait on anything. */
static void fb_clear_all(void)
{
	for (uint32_t b = 0; b < 2u; b++)
		for (uint32_t i = 0; i < LTDC_FB_PIXELS; i++)
			ltdc_fb[b][i] = LTDC_RGB565_BLACK;
}

int ltdc_set_scanout(bool on)
{
	if (!ltdc_up)               /* lock/objects may not exist yet */
		return LTDC_ERR_STATE;
	ltdc_lock_frame();
	if (ltdc_fault) {           /* re-check under the lock: a flip can fault */
		ltdc_unlock_frame();
		return LTDC_ERR_STATE;
	}
	if (on) {
		__HAL_LTDC_ENABLE(&hltdc);
		ltdc_disabled = false;
		ltdc_backlight(true);
	} else {
		ltdc_backlight(false);
		__HAL_LTDC_DISABLE(&hltdc);
		ltdc_disabled = true;
	}
	ltdc_unlock_frame();
	return LTDC_OK;
}

/* ==========================================================================
 *  DMA2D interrupt-driven completion
 * ========================================================================== */
/*
 * HAL_DMA2D_Start_IT enables TC|TE|CE together but its IRQ handler only disables
 * the bit of the event that fired, so the disarm below MUST clear all three: a
 * leftover TEIE/CEIE could otherwise fire DMA2D_IRQHandler during a later POLLED
 * transfer (which does not touch the IT enables), where it would steal the
 * completion flag from HAL_DMA2D_PollForTransfer with a stale dma2d_active.
 * Between transfers the invariant is: dma2d_active == NULL, TC|TE|CE disabled.
 */

/* Transfer-complete AND transfer-error callback (the waiter reads h->State to
   tell success from failure).  Runs in DMA2D_IRQHandler context. */
static void dma2d_xfer_done(DMA2D_HandleTypeDef *h)
{
	(void)h;
	(void)tx_semaphore_put(&dma2d_done_sem);
}

static void dma2d_arm(DMA2D_HandleTypeDef *h)
{
	/* Drain any stale post (e.g. a late IRQ from a previous timed-out
	   transfer) so the wait below blocks on THIS transfer. */
	while (tx_semaphore_get(&dma2d_done_sem, TX_NO_WAIT) == TX_SUCCESS)
		;
	h->XferCpltCallback  = dma2d_xfer_done;
	h->XferErrorCallback = dma2d_xfer_done;
	h->ErrorCode         = HAL_DMA2D_ERROR_NONE;
	dma2d_active         = h;              /* before HAL_DMA2D_Start_IT */
}

/*
 * Tear down an arm and leave DMA2D idle + the handle unlocked.  @p completed is
 * true only when the HAL completion callback actually ran: the handle is then
 * already READY and __HAL_UNLOCK'd.  When false (timeout, or a start that never
 * delivered a callback) the handle is still BUSY + LOCKED from
 * HAL_DMA2D_Start_IT -- and the hardware may have self-cleared CR.START exactly
 * as we masked the IRQ, so a CR.START test would wrongly skip cleanup -- hence
 * the unconditional HAL_DMA2D_Abort: it disables the ITs, sets State = READY and
 * UNLOCKS the handle even when START is already 0, preventing a wedged handle
 * (the next HAL_DMA2D_Init would otherwise spin on __HAL_LOCK forever).
 */
static void dma2d_disarm(DMA2D_HandleTypeDef *h, bool completed)
{
	uint32_t pm;

	/* Stop any pending/late completion IRQ from interleaving the teardown,
	   then drop the in-flight handle, under PRIMASK.  A DMA2D IRQ taken after
	   this sees dma2d_active == NULL and silences itself. */
	pm = __get_PRIMASK();
	__disable_irq();
	__HAL_DMA2D_DISABLE_IT(h, DMA2D_IT_TC | DMA2D_IT_TE | DMA2D_IT_CE);
	dma2d_active = NULL;
	__set_PRIMASK(pm);

	if (!completed || (DMA2D->CR & DMA2D_CR_START) != 0u)
		(void)HAL_DMA2D_Abort(h);
	__HAL_DMA2D_CLEAR_FLAG(h, DMA2D_FLAG_TC | DMA2D_FLAG_TE | DMA2D_FLAG_CE);
}

/* ==========================================================================
 *  DMA2D draw primitives
 * ========================================================================== */

/*
 * Expand an RGB565 colour to ARGB8888 (0x00RRGGBB).  R2M fills MUST pass this to
 * HAL_DMA2D_Start(): the HAL takes the R2M "color" argument as an ARGB8888 value
 * and re-packs it to the output format (RM0468 sec 39), so a raw RGB565 word
 * would render as the wrong colour.  The 5/6/5 -> 8/8/8 expansion replicates the
 * high bits into the low ones and the HAL truncates straight back.
 */
static uint32_t rgb565_to_argb8888(uint16_t c)
{
	uint32_t r = (uint32_t)(c >> 11) & 0x1Fu;
	uint32_t g = (uint32_t)(c >> 5) & 0x3Fu;
	uint32_t b = (uint32_t)c & 0x1Fu;

	r = (r << 3) | (r >> 2);          /* 5 -> 8 */
	g = (g << 2) | (g >> 4);          /* 6 -> 8 */
	b = (b << 3) | (b >> 2);          /* 5 -> 8 */
	return (r << 16) | (g << 8) | b;
}

/* Kick off an already-configured 2-operand DMA2D transfer (R2M fill or M2M blit)
   and run it to completion: interrupt-driven (block) when w*h is large, else
   polled.  Caller holds ltdc_lock. */
static bool dma2d_run(uint32_t pdata, uint32_t dst, uint32_t w, uint32_t h)
{
	if ((uint64_t)w * h >= LTDC_DMA2D_IT_MIN_PIXELS) {
		bool got, ok;

		dma2d_arm(&hdma2d);
		if (HAL_DMA2D_Start_IT(&hdma2d, pdata, dst, w, h) != HAL_OK) {
			dma2d_disarm(&hdma2d, false);
			return false;
		}
		got = (tx_semaphore_get(&dma2d_done_sem, 30) == TX_SUCCESS);
		ok  = got && (hdma2d.State == HAL_DMA2D_STATE_READY);
		dma2d_disarm(&hdma2d, got);
		return ok;
	}
	if (HAL_DMA2D_Start(&hdma2d, pdata, dst, w, h) != HAL_OK)
		return false;
	return HAL_DMA2D_PollForTransfer(&hdma2d, 30) == HAL_OK;
}

/* Register-to-memory single-colour fill of a w x h RGB565 block, @p line_off u16
   of padding skipped at each row end (= dst stride - w). */
static void ltdc_dma2d_fill(uint16_t *dst, uint16_t color, uint32_t w,
                            uint32_t h, uint32_t line_off)
{
	if (w == 0u || h == 0u)
		return;       /* a zero-size DMA2D transfer is not meaningful */

	hdma2d.Instance          = DMA2D;
	hdma2d.Init.Mode         = DMA2D_R2M;
	hdma2d.Init.ColorMode    = DMA2D_OUTPUT_RGB565;
	hdma2d.Init.OutputOffset = line_off;

	if (HAL_DMA2D_Init(&hdma2d) == HAL_OK &&
	    HAL_DMA2D_ConfigLayer(&hdma2d, 1) == HAL_OK)
		(void)dma2d_run(rgb565_to_argb8888(color), (uint32_t)(uintptr_t)dst,
		                w, h);
}

/* Memory-to-memory copy of a w x h RGB565 block, with separate u16 row offsets
   for destination (@p dst_off) and source (@p src_off). */
static void ltdc_dma2d_blit(uint16_t *dst, const uint16_t *src, uint32_t w,
                            uint32_t h, uint32_t dst_off, uint32_t src_off)
{
	if (w == 0u || h == 0u)
		return;

	hdma2d.Instance          = DMA2D;
	hdma2d.Init.Mode         = DMA2D_M2M;
	hdma2d.Init.ColorMode    = DMA2D_OUTPUT_RGB565;
	hdma2d.Init.OutputOffset = dst_off;

	hdma2d.LayerCfg[1].AlphaMode      = DMA2D_NO_MODIF_ALPHA;
	hdma2d.LayerCfg[1].InputAlpha     = 0xFF;
	hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_RGB565;
	hdma2d.LayerCfg[1].InputOffset    = src_off;

	if (HAL_DMA2D_Init(&hdma2d) == HAL_OK &&
	    HAL_DMA2D_ConfigLayer(&hdma2d, 1) == HAL_OK)
		(void)dma2d_run((uint32_t)(uintptr_t)src, (uint32_t)(uintptr_t)dst,
		                w, h);
}

/* ==========================================================================
 *  Controller bring-up
 * ========================================================================== */

/* Clocks + the two manually-driven control pins, parked inactive: panel held in
   reset and backlight off, so nothing is lit until there are real pixels.  Split
   out of ltdc_gpio_init() because the reset pulse and the ST7789 serial sequence
   both have to happen BEFORE the data lines become LTDC outputs. */
static void ltdc_reset_pins_init(void)
{
	GPIO_InitTypeDef g = {0};

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();

	g.Mode      = GPIO_MODE_OUTPUT_PP;
	g.Pull      = GPIO_NOPULL;
	g.Speed     = GPIO_SPEED_FREQ_LOW;
	g.Alternate = 0;

	HAL_GPIO_WritePin(LCD_RST_PORT, LCD_RST_PIN, GPIO_PIN_RESET);
	g.Pin = LCD_RST_PIN;
	HAL_GPIO_Init(LCD_RST_PORT, &g);
	HAL_GPIO_WritePin(LCD_BL_PORT, LCD_BL_PIN, GPIO_PIN_RESET);
	g.Pin = LCD_BL_PIN;
	HAL_GPIO_Init(LCD_BL_PORT, &g);
}

static void ltdc_pin_cs_park(void);   /* PA1 = LTDC_R2 = ST7789 CS; see below */

/*
 * Hand the LTDC signals to their alternate function.  Runs last, after the
 * ST7789 has been configured over the serial link that shares R0/R1/R2.
 *
 * ALL OF THEM EXCEPT PA1.  PA1 is LTDC_R2 *and* the ST7789's CS, and it stays a
 * GPIO driven high forever -- see ltdc_pin_cs_park() below for why.  Parking it
 * is done HERE, at the end, so that every path that re-arms the LTDC pins gets
 * it for free and cannot forget.
 */
static void ltdc_gpio_init(void)
{
	GPIO_InitTypeDef g = {0};

	g.Mode  = GPIO_MODE_AF_PP;
	g.Pull  = GPIO_NOPULL;
	g.Speed = GPIO_SPEED_FREQ_HIGH;

	g.Alternate = GPIO_AF14_LTDC;
	/* PA3 B5, PA5 R4, PA7 VSYNC, PA8 R6.  (PA1 = R2 is deliberately absent.) */
	g.Pin = GPIO_PIN_3 | GPIO_PIN_5 | GPIO_PIN_7 | GPIO_PIN_8;
	HAL_GPIO_Init(GPIOA, &g);
	/* PB0 G1, PB1 G0, PB8 B6, PB9 B7, PB10 G4, PB11 G5, PB15 G7 */
	g.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_8 | GPIO_PIN_9 |
	        GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_15;
	HAL_GPIO_Init(GPIOB, &g);
	/* PC5 DE, PC6 HSYNC, PC7 G6 */
	g.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
	HAL_GPIO_Init(GPIOC, &g);
	/* PD0 B1, PD6 B2, PD10 B3 */
	g.Pin = GPIO_PIN_0 | GPIO_PIN_6 | GPIO_PIN_10;
	HAL_GPIO_Init(GPIOD, &g);
	/* PE11 G3, PE12 B4, PE15 R7 */
	g.Pin = GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_15;
	HAL_GPIO_Init(GPIOE, &g);
	/* PG7 CLK, PG14 B0 -- per-pin RMW, so PG6 (OCTOSPI1_CS) is untouched */
	g.Pin = GPIO_PIN_7 | GPIO_PIN_14;
	HAL_GPIO_Init(GPIOG, &g);
	/* PH2 R0, PH3 R1, PH11 R5, PH13 G2 */
	g.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_11 | GPIO_PIN_13;
	HAL_GPIO_Init(GPIOH, &g);

	/* The one exception to AF14: LTDC_R3 is AF9 on PA15 (JTDI). */
	g.Alternate = GPIO_AF9_LTDC;
	g.Pin       = GPIO_PIN_15;
	HAL_GPIO_Init(GPIOA, &g);

	ltdc_pin_cs_park();
}

/*
 * Park PA1 -- LTDC_R2, which is also the ST7789's CS -- as a GPIO output driven
 * HIGH, and never give it back to the LTDC (issue #43).
 *
 * THE PROBLEM.  RM0468 sec 38.3.2 ("Pixel input format", just above Table 315):
 * components narrower than 8 bits are expanded "by bit replication", and it
 * spells the RGB565 red case out -- 5 bits r4..r0 come out as bit positions
 * 43210432.  So the LTDC drives R2 = r4, R1 = r3, R0 = r2.  On this board those
 * three lines ARE the ST7789's CS, SCL and SDA (st7789_rgb.h; the pin identities
 * come from the factory firmware's Arduino pin table, not from the schematic,
 * which only names them LCD_R0/R1/R2).  Scan-out therefore drives the panel's
 * serial bus with the top three bits of red, at the pixel clock:
 *
 *     CS = red bit 4      SCL = red bit 3      SDA = red bit 2
 *
 * Every pixel whose red MSB is 0 asserts CS, and every 0->1 step of red bit 3
 * while it is asserted clocks one bit into the ST7789's command register.  Nine
 * of those and the panel executes whatever they spell -- DISPOFF, SLPIN, a
 * RAMCTRL write -- and goes blank.  Uniformly white, permanently, with the LTDC
 * reporting a flawless scan-out because nothing on this side went wrong.
 *
 * WHY IT HID FOR TWO ISSUES.  Every test pattern is safe by construction: the
 * bars, the gradient and the bouncing rectangle are all either red = 0 (CS
 * asserted but red bit 3 never rises) or piecewise constant.  So #7 and #8 both
 * passed.  A live camera frame is not: red crosses 8 constantly while the MSB
 * stays 0, and the panel dies within seconds.  It was pinned down by streaming
 * the OV2640's own colour bars (`camera stream start test`), which runs the
 * identical DVP -> DCMI -> DMA -> PSRAM path at the identical rate and differs
 * only in the pixel values -- that never fails, live video always does.
 *
 * THE FIX.  Keep CS deasserted and the command port cannot hear anything, no
 * matter what the pixels do.  ST7789V sec 8.4.2: with CSX high the serial
 * interface is held initialized and SCL/SDA have no effect; sec 8.9: the RGB
 * interface is a separate path (VSYNC/HSYNC/DOTCLK/DE/data) that does not
 * involve CSX.  The board had already proved both halves of that BEFORE this
 * parking existed, back when CS still followed red bit 4: `lcd bar`'s
 * white/yellow/red/magenta bars are drawn at red MSB 1, i.e. CS high, and
 * `lcd anim` ran entirely at red 0, i.e. CS low -- and both displayed correctly.
 * So neither CSX level disturbs the picture, and high is the safe one to pick.
 *
 * THE COST is one bit of red, and it is the cheapest bit there is: R[7:3] carry
 * the real 5-bit red (weight 248/255) and R[2:0] are only the replication
 * padding (weight 7/255).  Pinning R2 high adds at most 4/255 = 1.6% red, so
 * black leaves the LTDC as (4,0,0).  SCL and SDA stay with the LTDC: with CS
 * high they are ignored, and they are still real (if tiny) data bits.
 */
static void ltdc_pin_cs_park(void)
{
	GPIO_InitTypeDef g = {0};

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);   /* CS deasserted */
	g.Pin   = GPIO_PIN_1;
	g.Mode  = GPIO_MODE_OUTPUT_PP;
	g.Pull  = GPIO_NOPULL;
	g.Speed = GPIO_SPEED_FREQ_LOW;   /* a static level; no edges to shape */
	HAL_GPIO_Init(GPIOA, &g);
}

/* Panel reset pulse.  HAL_Delay is safe here even pre-scheduler: SysTick feeds
   HAL_IncTick() from the moment HAL_Init() runs (issue #12). */
static void ltdc_panel_reset(void)
{
	HAL_GPIO_WritePin(LCD_RST_PORT, LCD_RST_PIN, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(LCD_RST_PORT, LCD_RST_PIN, GPIO_PIN_SET);
	HAL_Delay(20);
}

/* Program the controller + layer 0 for @p p.  HAL_LTDC_Init() sets GCR.LTDCEN
   itself, so there is no separate enable step afterwards. */
static int ltdc_controller_init(const struct ltdc_panel *p)
{
	LTDC_LayerCfgTypeDef layer = {0};

	hltdc.Instance = LTDC;

	/* All four polarities as the factory firmware programmed them (GCR bits
	   31..28 all clear), which also matches the ST7789's own RGBCTRL setting of
	   active-low sync in DE mode. */
	hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
	hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
	hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
	hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;

	/* RM0468 sec 38.4.1: every one of these registers holds "value - 1". */
	hltdc.Init.HorizontalSync     = (uint32_t)p->hs - 1u;
	hltdc.Init.VerticalSync       = (uint32_t)p->vs - 1u;
	hltdc.Init.AccumulatedHBP     = (uint32_t)p->hs + p->hbp - 1u;
	hltdc.Init.AccumulatedVBP     = (uint32_t)p->vs + p->vbp - 1u;
	hltdc.Init.AccumulatedActiveW = (uint32_t)p->hs + p->hbp + p->w - 1u;
	hltdc.Init.AccumulatedActiveH = (uint32_t)p->vs + p->vbp + p->h - 1u;
	hltdc.Init.TotalWidth         = (uint32_t)p->hs + p->hbp + p->w + p->hfp - 1u;
	hltdc.Init.TotalHeigh         = (uint32_t)p->vs + p->vbp + p->h + p->vfp - 1u;

	hltdc.Init.Backcolor.Red   = 0;
	hltdc.Init.Backcolor.Green = 0;
	hltdc.Init.Backcolor.Blue  = 0;

	if (HAL_LTDC_Init(&hltdc) != HAL_OK) {
		LOG_ERR("HAL_LTDC_Init failed");
		return LTDC_ERR_HAL;
	}

	/*
	 * HAL_LTDC_Init() enables the transfer-error and FIFO-underrun interrupts
	 * for us (stm32h7xx_hal_ltdc.c: "Enable the Transfer Error and FIFO
	 * underrun interrupts").  Mask them again, for two reasons -- and the second
	 * one is the important one:
	 *
	 *   - Scanning out of OCTOSPI1 means an underrun is a *bandwidth* symptom
	 *     that can repeat every line.  Left enabled, that is an interrupt storm
	 *     at priority 9 which would starve the console right when it is needed.
	 *   - HAL_LTDC_IRQHandler CLEARS FUIF/TERRIF when their enables are set, so
	 *     leaving them on would silently wipe the very flags ltdc_errors()
	 *     exists to report -- `lcd info` would cheerfully say "underrun: no"
	 *     during a continuous underrun.
	 *
	 * The ISR flags are set by hardware regardless of the enables, so polling
	 * them in ltdc_errors() loses nothing.  LTDC_IT_RR stays available; it is
	 * armed transiently by HAL_LTDC_Reload() inside ltdc_flip().
	 */
	__HAL_LTDC_DISABLE_IT(&hltdc, LTDC_IT_TE | LTDC_IT_FU);

	layer.WindowX0        = 0;
	layer.WindowX1        = p->w;
	layer.WindowY0        = 0;
	layer.WindowY1        = p->h;
	layer.PixelFormat     = LTDC_PIXEL_FORMAT_RGB565;
	layer.Alpha           = 255;
	layer.Alpha0          = 0;
	layer.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
	layer.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
	layer.FBStartAdress   = (uint32_t)(uintptr_t)&ltdc_fb[ltdc_front][0];
	layer.ImageWidth      = p->w;
	layer.ImageHeight     = p->h;
	layer.Backcolor.Red   = 0;
	layer.Backcolor.Green = 0;
	layer.Backcolor.Blue  = 0;

	if (HAL_LTDC_ConfigLayer(&hltdc, &layer, 0) != HAL_OK) {
		LOG_ERR("HAL_LTDC_ConfigLayer failed");
		return LTDC_ERR_HAL;
	}
	return LTDC_OK;
}

int ltdc_init(void)
{
	int rc;

	if (ltdc_tried)
		return ltdc_up ? LTDC_OK : LTDC_ERR_STATE;
	ltdc_tried = true;

	/* No pixel clock, no display.  (The frame buffer also lives in PSRAM; the
	   caller is responsible for having confirmed psram_ready() -- this module
	   stays free of any psram.h dependency, see the header.) */
	if (ltdc_pixel_clock_hz() == 0u) {
		LOG_ERR("pixel clock down -- LTDC not started");
		return LTDC_ERR_STATE;
	}

	/* ThreadX objects for the tear-free flip and the DMA2D completion.  Created
	   before the controller comes up so the IRQs have somewhere to post.
	   Creation only -- nothing here waits, which is what makes this callable
	   from tx_application_define(). */
	if (tx_semaphore_create(&ltdc_reload_sem, "ltdc_rl", 0) != TX_SUCCESS) {
		LOG_ERR("ltdc reload semaphore create failed");
		return LTDC_ERR_STATE;
	}
	if (tx_mutex_create(&ltdc_lock, "ltdc", TX_INHERIT) != TX_SUCCESS) {
		LOG_ERR("ltdc lock create failed");
		tx_semaphore_delete(&ltdc_reload_sem);
		return LTDC_ERR_STATE;
	}
	if (tx_semaphore_create(&dma2d_done_sem, "dma2d", 0) != TX_SUCCESS) {
		LOG_ERR("dma2d completion semaphore create failed");
		tx_mutex_delete(&ltdc_lock);
		tx_semaphore_delete(&ltdc_reload_sem);
		return LTDC_ERR_STATE;
	}

	/* Clock gates first: several LTDC registers sit in the pixel-clock domain
	   (RM0468 sec 38.3.3), so ltdc_clock_init() must already have locked
	   pll3_r_ck -- checked above -- before any of them is written. */
	__HAL_RCC_LTDC_CLK_ENABLE();
	__HAL_RCC_DMA2D_CLK_ENABLE();

	/*
	 * Panel bring-up, and the order matters.  The ST7789 on this module powers
	 * up asleep with its RGB interface off, and its CS/SDA/SCL are multiplexed
	 * onto LTDC_R2/R0/R1 -- so the serial configuration has to happen while
	 * those three pins are still plain GPIO, before the LTDC is allowed anywhere
	 * near them.  Skipping it is not a degraded mode: the panel ignores every
	 * pixel and stays backlit and blank while the LTDC reports a perfectly
	 * healthy scanout (that is exactly how this was found).
	 *
	 * ltdc_gpio_init() then takes the pins over for the alternate function and
	 * parks LCD_RST/LCD_BL, so it must run AFTER the serial sequence.
	 *
	 * st7789_rgb_pins_init() runs BEFORE the reset pulse, not with the sequence:
	 * see issue #43 and st7789_rgb.h.  Until it does, those three lines are
	 * floating inputs -- and after a software reset the panel is powered, awake
	 * and listening to them.
	 */
	ltdc_reset_pins_init();
	st7789_rgb_pins_init();
	ltdc_panel_reset();
	st7789_rgb_init();
	ltdc_gpio_init();

	/* .psram_noinit is NOLOAD (undefined at reset): clear BOTH buffers to black
	   BEFORE the layer is configured and the backlight comes on, so neither a
	   garbage front frame nor a garbage first flip is ever displayed.  CPU loop,
	   not DMA2D -- no waiting allowed here. */
	fb_clear_all();

	ltdc_front = 0;
	ltdc_fault = false;
	ltdc_disabled = false;

	rc = ltdc_controller_init(&ltdc_cfg);
	if (rc != LTDC_OK) {
		ltdc_backlight(false);
		(void)HAL_LTDC_DeInit(&hltdc);
		__HAL_RCC_DMA2D_CLK_DISABLE();
		__HAL_RCC_LTDC_CLK_DISABLE();
		goto fail_obj;     /* ltdc_up stays false */
	}

	/* Reload-ready IRQ: below the RTL8720 UART (5) and OTG_HS/SDMMC1 (6), whose
	   ISRs have real deadlines, and above SysTick (14) / PendSV (15).  The
	   vector only fires once HAL_LTDC_Reload() arms LTDC_IT_RR inside
	   ltdc_flip(), which cannot happen before the scheduler runs.  DMA2D's
	   completion IRQ sits one step lower still -- it only wakes a blocked
	   drawing thread, so latency there is irrelevant.  ThreadX-call safety from
	   these ISRs comes from the port's PRIMASK critical sections, not from the
	   numeric priority. */
	HAL_NVIC_SetPriority(LTDC_IRQn, 9, 0);
	HAL_NVIC_EnableIRQ(LTDC_IRQn);
	HAL_NVIC_SetPriority(DMA2D_IRQn, 10, 0);
	HAL_NVIC_EnableIRQ(DMA2D_IRQn);

	/*
	 * Come up fully INITIALIZED but DARK (issue #53).  `lcd on` presents.
	 *
	 * 🔴 What is deferred is only PRESENTING.  Everything above still ran: the
	 * reset pulse, the ST7789's SWRESET + factory sequence, the CS park and the
	 * controller configuration.  Skipping any of that and doing it at `lcd on`
	 * instead is how a display comes up "LTDC perfectly healthy, screen uniformly
	 * white" -- the failure mode issues #7 and #43 each cost days to find, and no
	 * counter anywhere reports it.
	 *
	 * Here rather than in the caller because switching the backlight on and then
	 * off around a return would visibly flash the panel on every boot; the pin is
	 * still parked low from ltdc_reset_pins_init(), so it simply never lights.
	 * And not via ltdc_set_scanout(), which takes ltdc_lock: this runs from
	 * tx_application_define() with the scheduler stopped, and this driver's
	 * contract is that ltdc_init() waits on no ThreadX object.
	 *
	 * The side effect is the point as much as the darkness is: with no scan-out
	 * there is no continuous LTDC read of OCTOSPI1, so `psram` tuning, `membench`
	 * and `devmem` work from boot without an `lcd off` first, and `ai bench` does
	 * not silently carry the ~26 ms scan-out tax that issue #9 twice mistook for
	 * a change in inference cost.
	 */
	__HAL_LTDC_DISABLE(&hltdc);
	ltdc_disabled = true;

	ltdc_up = true;
	LOG_INF("ST7789 up: %ux%u RGB565 x2 @0x%08lx, %lu.%02lu MHz (div %lu), "
	        "%lu.%02lu Hz, scanout off (`lcd on`)",
	        (unsigned)ltdc_cfg.w, (unsigned)ltdc_cfg.h,
	        (unsigned long)(uintptr_t)&ltdc_fb[0][0],
	        (unsigned long)(ltdc_pixel_clock_hz() / 1000000u),
	        (unsigned long)(ltdc_pixel_clock_hz() % 1000000u / 10000u),
	        (unsigned long)ltdc_clock_div(),
	        (unsigned long)(ltdc_refresh_chz() / 100u),
	        (unsigned long)(ltdc_refresh_chz() % 100u));
	return LTDC_OK;

fail_obj:
	tx_semaphore_delete(&dma2d_done_sem);
	tx_mutex_delete(&ltdc_lock);
	tx_semaphore_delete(&ltdc_reload_sem);
	return rc;
}

/*
 * Re-run the panel bring-up on a live system (issue #43).
 *
 * The ST7789 keeps its own supply across an MCU reset, so it can be left in a
 * state the power-on sequence does not recover from -- and once ltdc_init() has
 * handed CS/SDA/SCL to the LTDC there was no way to resend that sequence short
 * of unplugging the board.  `lcd off` / `lcd on` does not help: it toggles
 * LTDCEN and the backlight and never speaks to the panel at all.
 *
 * What LTDCEN=0 actually does is worth being precise about, because it is the
 * step this function leans on.  It does NOT tri-state the LTDC pins: RM0468
 * sec 38.4.1 says the timing generator is held reset at (total width - 1,
 * total height - 1), the FIFOs are flushed, and blanking data keeps coming out.
 * So disabling the LTDC stops the scanout, the FIFO and the PSRAM fetch -- which
 * is exactly what we need -- and the pins become ours only on the next line,
 * where st7789_rgb_pins_init() rewrites MODER and the GPIO block takes over.
 *
 * HAL_LTDC_Init() is deliberately NOT re-run.  The controller and layer
 * configuration survive untouched, and re-running it would re-enable the
 * transfer-error and FIFO-underrun interrupts that ltdc_controller_init() masks
 * on purpose -- whose handler then clears the very flags ltdc_errors() reports.
 * Toggling LTDCEN is the whole of what is needed.
 *
 * ltdc_lock is held across the ~245 ms sequence, which serializes this against
 * the drawing commands and the camera preview thread (both take the same lock
 * before touching a buffer).  No new lock order is introduced.
 *
 * Thread context only -- HAL_Delay() spins here rather than sleeping, which is
 * what lets boot and this path share one code path.
 */
int ltdc_panel_recover(void)
{
	bool was_on;

	if (!ltdc_up)
		return LTDC_ERR_STATE;
	ltdc_lock_frame();
	if (ltdc_fault) {       /* a stuck VBR reload is a different failure; the
	                           display is latched down and a reset owns it */
		ltdc_unlock_frame();
		return LTDC_ERR_STATE;
	}
	was_on = !ltdc_disabled;

	ltdc_backlight(false);
	__HAL_LTDC_DISABLE(&hltdc);
	/* Publish "not scanning out" while we work: app/psram.c reads this to decide
	   whether OCTOSPI1 is being fetched from.  The caller holds the OCTOSPI1
	   guard across the whole call, so nothing can retune the bus in the window
	   this opens (see cmd_lcd.c). */
	ltdc_disabled = true;

	ltdc_reset_pins_init();     /* idempotent; also re-asserts LCD_RST */
	st7789_rgb_pins_init();
	ltdc_panel_reset();
	st7789_rgb_init();
	ltdc_gpio_init();

	/* Restore the scanout state we found, rather than forcing it on: `lcd reset`
	   after an `lcd off` must not silently start reading PSRAM again. */
	if (was_on) {
		__HAL_LTDC_ENABLE(&hltdc);
		ltdc_disabled = false;
		ltdc_backlight(true);
	}

	ltdc_unlock_frame();
	LOG_INF("panel re-initialized (scanout %s)", was_on ? "on" : "off");
	return LTDC_OK;
}

/* ==========================================================================
 *  Drawing -- LANDSCAPE 320x240 coordinates (issue #38)
 * ==========================================================================
 *
 * The public drawing API speaks a 320x240 landscape coordinate system even
 * though the panel is 240x320 portrait and the frame buffer keeps a stride of
 * 240.  Everything worth showing here is landscape -- the camera is 320x240 and
 * so was the board's factory firmware -- and this SoC cannot rotate: no GFXMMU,
 * no GPU2D, nothing in the LTDC.  Neither will the panel; MADCTL/MV and
 * RAMCTRL/RM have both been measured dead on the RGB interface (see
 * st7789_rgb.c, and do not re-open it).  So the rotation is a coordinate
 * transform, applied here, once.
 *
 * THE MAPPING is the one the factory firmware called rotation 2, recovered by
 * disassembling its image (issue #38):
 *
 *     landscape (x, y)  ->  fb[240 * (319 - x) + y]
 *
 * x is the long axis (0..319), y the short one (0..239).
 *
 * WHY THIS IS CHEAP, mostly.  A 90-degree rotation maps an axis-aligned
 * rectangle to an axis-aligned rectangle, so every fill -- ltdc_fill,
 * ltdc_fill_rect, the colour bars, the gradient -- is the same DMA2D work it
 * always was with its arguments permuted.  Rotation costs nothing there.  Only
 * ltdc_blit() has to transpose pixels, and it hands that to svc/gfx_rot.
 *
 * BUILD-TIME FIXED, not a runtime `lcd rotate`.  A runtime angle would change
 * what ltdc_blit()'s source coordinates MEAN from call to call, and would need
 * four transpose loops instead of one; the panel is bolted to a board that has
 * one sensible orientation, so the flexibility would buy nothing and cost a
 * test matrix.
 *
 * Panel-native coordinates survive only as the fb_* helpers below, which assume
 * the frame lock is held and the back buffer is non-NULL.
 */

/* Landscape rect -> frame-buffer rect.  Derived straight from the mapping above:
 * landscape x becomes the frame-buffer ROW (reversed), landscape y the column, so
 * a rect's width and height swap and its x origin measures from the far edge. */
static void surf_to_fb(uint32_t x, uint32_t y, uint32_t w, uint32_t h,
                       uint32_t *fx, uint32_t *fy, uint32_t *fw, uint32_t *fh)
{
	*fx = y;
	*fy = LTDC_SURFACE_W - x - w;
	*fw = h;
	*fh = w;
}

/* Clip a landscape rect to the surface; 0 = nothing left to draw. */
static int surf_clip(uint32_t *x, uint32_t *y, uint32_t *w, uint32_t *h)
{
	uint32_t x1, y1;

	if (*x >= LTDC_SURFACE_W || *y >= LTDC_SURFACE_H)
		return 0;
	x1 = *x + *w;
	y1 = *y + *h;
	if (x1 > LTDC_SURFACE_W)
		x1 = LTDC_SURFACE_W;
	if (y1 > LTDC_SURFACE_H)
		y1 = LTDC_SURFACE_H;
	*w = x1 - *x;
	*h = y1 - *y;
	return (*w != 0u && *h != 0u);
}

/* Panel-native rectangle fill.  Frame lock held, @back non-NULL, rect in range. */
static void fb_fill_rect(uint16_t *back, uint32_t fx, uint32_t fy,
                         uint32_t fw, uint32_t fh, uint16_t rgb565)
{
	ltdc_dma2d_fill(back + fy * LTDC_DEF_W + fx, rgb565, fw, fh,
	                (uint32_t)LTDC_DEF_W - fw);
}

uint16_t ltdc_surface_w(void) { return LTDC_SURFACE_W; }
uint16_t ltdc_surface_h(void) { return LTDC_SURFACE_H; }

void ltdc_fill(uint16_t rgb565)
{
	uint16_t *back;

	if (!ltdc_is_up())
		return;
	ltdc_lock_frame();
	back = ltdc_back_buffer();
	if (back != NULL)   /* the whole buffer, so no transform to make */
		ltdc_dma2d_fill(back, rgb565, LTDC_DEF_W, LTDC_DEF_H, 0);
	ltdc_unlock_frame();
}

void ltdc_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                    uint16_t rgb565)
{
	uint32_t cx = x, cy = y, cw = w, ch = h;
	uint32_t fx, fy, fw, fh;
	uint16_t *back;

	if (!ltdc_is_up())
		return;
	ltdc_lock_frame();
	back = ltdc_back_buffer();
	if (back != NULL && surf_clip(&cx, &cy, &cw, &ch)) {
		surf_to_fb(cx, cy, cw, ch, &fx, &fy, &fw, &fh);
		fb_fill_rect(back, fx, fy, fw, fh, rgb565);
	}
	ltdc_unlock_frame();
}

void ltdc_blit(const uint16_t *src, uint16_t x, uint16_t y,
               uint16_t w, uint16_t h)
{
	uint32_t cx = x, cy = y, cw = w, ch = h;
	uint16_t *back;

	if (!ltdc_is_up())
		return;
	ltdc_lock_frame();
	back = ltdc_back_buffer();
	if (back != NULL && src != NULL && surf_clip(&cx, &cy, &cw, &ch)) {
		/* The only primitive rotation actually costs anything for.  The source
		   stride stays the caller's full w even when clipping narrowed the copy.
		   gfx_blit_rot() reads the source with a stride and writes the frame
		   buffer in contiguous runs, which is the right way round for PSRAM --
		   but it also means a source IN PSRAM is the expensive case.  That is
		   what issue #35 stages through an AXI-SRAM band to avoid. */
		gfx_blit_rot(src, w, back, LTDC_DEF_W, LTDC_DEF_H, cx, cy, cw, ch);
	}
	ltdc_unlock_frame();
}

void ltdc_colorbar(void)
{
	static const uint16_t bars[8] = {
		LTDC_RGB565_WHITE, LTDC_RGB565_YELLOW, LTDC_RGB565_CYAN,
		LTDC_RGB565_GREEN, LTDC_RGB565_MAGENTA, LTDC_RGB565_RED,
		LTDC_RGB565_BLUE, LTDC_RGB565_BLACK,
	};
	uint16_t *back;
	uint32_t bar_w;

	if (!ltdc_is_up())
		return;
	ltdc_lock_frame();
	back = ltdc_back_buffer();
	if (back == NULL) {
		ltdc_unlock_frame();
		return;
	}
	bar_w = (uint32_t)LTDC_SURFACE_W / 8u;
	/* Eight bars across the long axis, full height.  Each is a landscape rect, so
	   it is still one DMA2D R2M fill after the transform -- rotation is free here.
	   The last bar absorbs the remainder so the eight cover the width exactly. */
	for (uint32_t i = 0; i < 8u; i++) {
		uint32_t x0 = i * bar_w;
		uint32_t w  = (i == 7u) ? ((uint32_t)LTDC_SURFACE_W - x0) : bar_w;
		uint32_t fx, fy, fw, fh;

		surf_to_fb(x0, 0u, w, LTDC_SURFACE_H, &fx, &fy, &fw, &fh);
		fb_fill_rect(back, fx, fy, fw, fh, bars[i]);
	}
	ltdc_unlock_frame();
}

void ltdc_blit_demo(void)
{
	uint16_t *back;
	uint32_t half;

	if (!ltdc_is_up())
		return;
	ltdc_lock_frame();
	back = ltdc_back_buffer();
	if (back == NULL) {
		ltdc_unlock_frame();
		return;
	}
	half = (uint32_t)LTDC_SURFACE_W / 2u;
	/* M2M copy of the left landscape half over the right one, within the back
	   buffer.  Under the transform each half is a RUN OF WHOLE FRAME-BUFFER ROWS,
	   so unlike the portrait version this needs no strides at all: rows
	   [half, 2*half) copied down over rows [0, half).  (Left maps to the high
	   rows because landscape x counts from the far edge.) */
	ltdc_dma2d_blit(back, back + half * LTDC_DEF_W,
	                LTDC_DEF_W, half, 0u, 0u);
	ltdc_unlock_frame();
}

void ltdc_gradient(void)
{
	uint16_t *back;

	if (!ltdc_is_up())
		return;
	ltdc_lock_frame();
	back = ltdc_back_buffer();
	if (back == NULL) {
		ltdc_unlock_frame();
		return;
	}
	/* One luminance per landscape column, and a landscape column is a whole
	   frame-buffer row -- so each step writes 240 CONTIGUOUS pixels.  The
	   portrait version wrote the same pixels with a 240-pixel stride; the
	   transform made this one strictly cheaper, not more expensive. */
	for (uint32_t x = 0; x < LTDC_SURFACE_W; x++) {
		uint32_t  lum = x * 255u / ((uint32_t)LTDC_SURFACE_W - 1u);
		uint16_t  px  = (uint16_t)(((lum >> 3) << 11) |   /* R5 */
		                           ((lum >> 2) << 5) |    /* G6 */
		                            (lum >> 3));          /* B5 */
		uint16_t *row = back + ((uint32_t)LTDC_SURFACE_W - 1u - x) * LTDC_DEF_W;

		for (uint32_t i = 0; i < LTDC_DEF_W; i++)
			row[i] = px;
	}
	ltdc_unlock_frame();
}

/* ==========================================================================
 *  Tear-free present
 * ========================================================================== */
/*
 * ltdc_lock is held across the whole sequence; every return path releases it.
 * SRCR.VBR is authoritative: it is set by HAL_LTDC_Reload(VERTICAL_BLANKING) and
 * self-cleared by hardware only after the register reload commits at the next
 * vertical blanking (RM0468 sec 38.7).  The reload-ready IRQ just wakes us; we
 * still poll VBR to confirm.
 */
int ltdc_flip(void)
{
	uint8_t back;
	int rc = LTDC_OK;

	if (!ltdc_is_up())
		return LTDC_ERR_STATE;
	ltdc_lock_frame();

	if (ltdc_fault || ltdc_disabled) {
		/* Scanout off: no VBR reload would ever come, and waiting for one
		   would latch a spurious fault. */
		ltdc_unlock_frame();
		return LTDC_ERR_STATE;
	}

	back = (uint8_t)!ltdc_front;

	/* Drain stale posts from a previous flip so the wait blocks on THIS
	   reload, not a leftover one. */
	while (tx_semaphore_get(&ltdc_reload_sem, TX_NO_WAIT) == TX_SUCCESS)
		;

	/* Fail-closed all the way: a HAL refusal here (handle locked, bad state)
	   means the new address may not be staged, so do not wait for a reload that
	   was never armed -- latch the display down like a stuck VBR does. */
	if (HAL_LTDC_SetAddress_NoReload(&hltdc,
	                                 (uint32_t)(uintptr_t)&ltdc_fb[back][0],
	                                 0) != HAL_OK ||
	    HAL_LTDC_Reload(&hltdc, LTDC_RELOAD_VERTICAL_BLANKING) != HAL_OK) {
		ltdc_fault = true;
		LOG_ERR("LTDC reload could not be armed, display down");
		ltdc_unlock_frame();
		return LTDC_ERR_HAL;
	}

	/* Wake-up hint (the IRQ posts on reload-ready); the truth is VBR below.  A
	   frame is ~15 ms at the default timing, so 100 ms is several frames of
	   slack. */
	(void)tx_semaphore_get(&ltdc_reload_sem, 100);

	for (uint32_t i = 0; i < 100u; i++) {
		if ((hltdc.Instance->SRCR & LTDC_SRCR_VBR) == 0u)
			break;
		tx_thread_sleep(1);
	}

	if ((hltdc.Instance->SRCR & LTDC_SRCR_VBR) == 0u) {
		ltdc_front = back;            /* swap committed */
	} else {
		/* fail-closed: the reload never landed -- leave the front buffer as-is
		   and latch the display down (recovery is a system reset). */
		ltdc_fault = true;
		LOG_ERR("LTDC reload stuck (VBR), display down");
		rc = LTDC_ERR_HAL;
	}

	ltdc_unlock_frame();
	return rc;
}

/* ==========================================================================
 *  Interrupts
 * ========================================================================== */

/* Reload-ready IRQ (only armed transiently by HAL_LTDC_Reload).  Bracketed with
   the ThreadX execution-profile hooks like the SD / USB ISRs so the kit charges
   this time to the (isr) row.  HAL_LTDC_IRQHandler() clears the RR flag,
   disables the RR interrupt and dispatches HAL_LTDC_ReloadEventCallback(). */
void LTDC_IRQHandler(void)
{
	tx_glue_isr_enter();
	HAL_LTDC_IRQHandler(&hltdc);
	tx_glue_isr_exit();
}

/* Reload-ready: wake ltdc_flip() promptly (it still re-checks VBR). */
void HAL_LTDC_ReloadEventCallback(LTDC_HandleTypeDef *h)
{
	if (h == &hltdc)
		(void)tx_semaphore_put(&ltdc_reload_sem);
}

/* DMA2D completion IRQ (armed transiently by HAL_DMA2D_Start_IT).  Between
   transfers dma2d_active is NULL; a DMA2D IRQ taken then is a late/stale
   completion from a just-disarmed transfer (e.g. a timeout that finished right
   as we tore down) -- silence every source it could have raised so it cannot
   re-pend into an interrupt storm, rather than feeding HAL a NULL handle. */
void DMA2D_IRQHandler(void)
{
	DMA2D_HandleTypeDef *h;

	tx_glue_isr_enter();
	h = dma2d_active;
	if (h != NULL) {
		HAL_DMA2D_IRQHandler(h);
	} else {
		DMA2D->CR  &= ~(DMA2D_IT_TC | DMA2D_IT_TE | DMA2D_IT_CE);
		DMA2D->IFCR =  DMA2D_FLAG_TC | DMA2D_FLAG_TE | DMA2D_FLAG_CE;
	}
	tx_glue_isr_exit();
}
