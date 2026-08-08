/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    st7789_rgb.c
 * @brief   ST7789 bring-up over the bit-banged 3-wire SPI multiplexed onto the
 *          LTDC red data lines (issue #7).  See st7789_rgb.h for why this exists
 *          and where the numbers came from.
 */
#include "st7789_rgb.h"

#include "stm32h7xx_hal.h"
#include "timebase.h"   /* udelay: DWT-based, no scheduler needed */

/* Serial pins, multiplexed with LTDC_R2 / R0 / R1 (see the header). */
#define ST_CS_PORT    GPIOA
#define ST_CS_PIN     GPIO_PIN_1     /* LTDC_R2 */
#define ST_SDA_PORT   GPIOH
#define ST_SDA_PIN    GPIO_PIN_2     /* LTDC_R0 */
#define ST_SCL_PORT   GPIOH
#define ST_SCL_PIN    GPIO_PIN_3     /* LTDC_R1 */
/* The factory firmware also parks the pixel clock low as a GPIO across the
   sequence; mirrored here so the panel sees exactly what it saw at the factory. */
#define ST_PCLK_PORT  GPIOG
#define ST_PCLK_PIN   GPIO_PIN_7     /* LTDC_CLK */

/* Half-bit time.  The factory image inserts a short software delay between edges
   and the ST7789 has no minimum clock rate, so this is generous rather than
   tuned: ~1 us per edge puts the bus around 300 kHz, and the whole 68-frame
   sequence still costs well under a millisecond of clocking. */
#define ST_HALF_BIT_US  1u

static void st_write(uint32_t dc, uint8_t val)
{
	/* 9 bits, MSB first: the data/command flag, then the byte.  Data is
	   presented while SCL is high, then SCL goes low and back high -- the
	   controller latches on the rising edge. */
	HAL_GPIO_WritePin(ST_CS_PORT, ST_CS_PIN, GPIO_PIN_RESET);
	for (uint32_t i = 0; i < 9u; i++) {
		uint32_t bit = (i == 0u) ? dc : ((uint32_t)val >> (8u - i)) & 1u;

		HAL_GPIO_WritePin(ST_SDA_PORT, ST_SDA_PIN,
		                  bit ? GPIO_PIN_SET : GPIO_PIN_RESET);
		udelay(ST_HALF_BIT_US);
		HAL_GPIO_WritePin(ST_SCL_PORT, ST_SCL_PIN, GPIO_PIN_RESET);
		udelay(ST_HALF_BIT_US);
		HAL_GPIO_WritePin(ST_SCL_PORT, ST_SCL_PIN, GPIO_PIN_SET);
	}
	HAL_GPIO_WritePin(ST_SDA_PORT, ST_SDA_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(ST_PCLK_PORT, ST_PCLK_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(ST_CS_PORT, ST_CS_PIN, GPIO_PIN_SET);
}

static void st_cmd(uint8_t c)  { st_write(0u, c); }
static void st_data(uint8_t d) { st_write(1u, d); }

/*
 * The power-on sequence, transcribed one-for-one from the factory image.  The
 * command bytes are ST7789 register names; the parameter bytes are panel-
 * specific tuning (gamma, power rails, porches) that only the module vendor
 * knows, so they are reproduced verbatim rather than derived.
 *
 * MADCTL = 0x00 IS NOT A FREE CHOICE, AND ROTATION IS NOT AVAILABLE HERE.
 *
 * Bit 5 of MADCTL (MV, row/column exchange) looks like it should let the panel
 * be driven landscape for free -- which would matter a lot, because the camera
 * is 320x240 and this panel is 240x320, the H725 has no rotation engine (no
 * GFXMMU, no GPU2D), and transposing with DMA2D or MDMA costs ~77k scattered
 * 2-byte accesses on a serial PSRAM.  It was measured on the board (issue #8
 * phase 3a) and it does not work:
 *
 *   LTDC driven at 320x240 (378x256 total, 62.00 Hz), MADCTL = 0x20:
 *     RGBCTRL[0] = 0xC0 -> the panel keeps its 240-pixel line length, so the
 *                          image wraps: a single diagonal comes out as two, 1-px
 *                          lines widen into dithered bands, and the border is no
 *                          longer a rectangle.  LTDC reports no underrun and no
 *                          transfer error throughout -- the controller is fine,
 *                          the panel simply maps the pixels elsewhere.
 *     RGBCTRL[0] = 0x40 -> worse: flat blue with a band of noise, no pattern at
 *                          all.  Clearing that bit breaks the RGB data path.
 *
 * So on THIS module MADCTL/MV does not reposition RGB-interface pixels on either
 * of the two RGBCTRL data paths.
 *
 * THE LAST KNOB IS NOW ALSO TRIED, AND IT DOES NOT WORK EITHER (issues #35/#38).
 *
 * Phase 3a stopped two flashes in and left RAMCTRL(0xB0)'s RM bit untried -- the
 * bit that selects how RGB-interface pixels reach frame memory, i.e. whether
 * MADCTL's address mapping is in the path at all.  That was the one remaining way
 * this panel might have rotated for free, and it was worth ten minutes before
 * committing to a transpose, so a throwaway `lcd rotprobe` command swept it from
 * the console (one flash, many combinations -- the internal flash is good for
 * ~10k erase cycles and reflashing per combination would have spent them for
 * nothing).  Driven at the same 320x240 / 378x256 total / 62.00 Hz as phase 3a:
 *
 *     MADCTL  RAMCTRL[0]   result
 *     0x20    0x11         negative (reproduces phase 3a's first shot)
 *     0x20    0x01         negative   RM cleared, DM still RGB
 *     0x20    0x10         negative   RM set, DM cleared
 *     0x00    0x01         negative   control: MV off, RM cleared
 *
 * None of them made the controller accept a 320-pixel line.  The knob is spent;
 * the probe was deleted with the answer, as bring-up knobs are (issue #8, 3c-2).
 *
 * >>> DO NOT RE-OPEN THIS.  Both MADCTL/MV and RAMCTRL/RM have now been measured
 * >>> on board #2 and neither rotates RGB-interface input on this module.  A
 * >>> future reader looking for a free rotation is looking at a closed door.
 *
 * Landscape therefore has to be produced on the host side, and both consumers of
 * that conclusion are designed around it: issue #38 gives the drawing layer a
 * landscape 320x240 coordinate system (the panel-native frame buffer stride stays
 * 240, exactly as the factory Arduino firmware did it), and issue #35 rotates the
 * camera by taking DCMI into AXI-SRAM bands and transposing into the PSRAM back
 * buffer, so the scattered side of the transpose lands on SRAM and the PSRAM side
 * stays a contiguous run.  The existing ltdc_flip() keeps it tear-free.
 */
static void st_send_sequence(void)
{
	static const uint8_t seq[] = {
		/* count, command, params... ; count = number of parameter bytes */
		0, 0x11,                                     /* SLPOUT (needs 120 ms) */
		2, 0xB0, 0x11, 0xF0,                         /* RAMCTRL: RGB interface */
		3, 0xB1, 0xC0, 0x02, 0x14,                   /* RGBCTRL: DE mode       */
		5, 0xB2, 0x0C, 0x0C, 0x00, 0x33, 0x33,       /* PORCTRL                */
		1, 0xB7, 0x35,                               /* GCTRL                  */
		1, 0xBB, 0x28,                               /* VCOMS                  */
		1, 0xC0, 0x2C,                               /* LCMCTRL                */
		1, 0xC2, 0x01,                               /* VDVVRHEN               */
		1, 0xC3, 0x0B,                               /* VRHS                   */
		1, 0xC4, 0x20,                               /* VDVS                   */
		1, 0xC6, 0x0F,                               /* FRCTRL2                */
		2, 0xD0, 0xA4, 0xA1,                         /* PWCTRL1                */
		14, 0xE0, 0xD0, 0x01, 0x08, 0x0F, 0x11, 0x2A, 0x36, 0x55, 0x44, 0x3A,
		           0x0B, 0x06, 0x11, 0x20,           /* PVGAMCTRL             */
		14, 0xE1, 0xD0, 0x02, 0x07, 0x0A, 0x0B, 0x18, 0x34, 0x43, 0x4A, 0x2B,
		           0x1B, 0x1C, 0x22, 0x1F,           /* NVGAMCTRL             */
		1, 0x3A, 0x55,                               /* COLMOD: 16 bit/pixel   */
		1, 0x36, 0x00,                               /* MADCTL: default scan   */
		0, 0x29,                                     /* DISPON                 */
	};
	uint32_t i = 0;

	/*
	 * SWRESET is a PREFIX to the transcription above, not part of it.
	 *
	 * The factory image never sends it, because an Arduino sketch only ever
	 * starts from a cold panel.  This firmware does not: a software reset
	 * (`crash`, `reboot`, a DFU reboot) restarts the MCU while the ST7789 keeps
	 * its own supply, so the table above can be replayed into a controller that
	 * is already Sleep Out + DISPON with its RGB interface live -- a state it
	 * was never written against, and one it does not recover from.  That is
	 * issue #43: after a soft reset the panel stayed uniformly white until the
	 * board was unplugged, while the LTDC reported a perfectly healthy scanout
	 * and every `lcd` command returned success.
	 *
	 * So restore the state the transcription DOES assume, then replay it.  This
	 * keeps issue #7's "the factory image is the specification" premise intact:
	 * the specified bytes are still sent verbatim, they are just no longer sent
	 * into an undefined initial state.
	 *
	 * The 120 ms is mandatory, not padding.  The ST7789V forbids SLPOUT for
	 * 120 ms after a reset taken while the display was in Sleep Out mode --
	 * exactly the software-reset case, for the software reset here and for the
	 * hardware RESX pulse the caller drove just before.  Both are covered,
	 * because this wait sits between them and the table's leading SLPOUT.
	 */
	st_cmd(0x01);          /* SWRESET */
	HAL_Delay(120);

	while (i < sizeof seq) {
		uint8_t n   = seq[i++];
		uint8_t cmd = seq[i++];

		st_cmd(cmd);
		if (cmd == 0x11u)
			HAL_Delay(120);   /* sleep-out settle, per the factory image */
		while (n-- > 0u)
			st_data(seq[i++]);
	}
}

void st7789_rgb_pins_init(void)
{
	GPIO_InitTypeDef g = {0};

	/* Idle levels before the pins are driven, so the first CS edge is clean. */
	HAL_GPIO_WritePin(ST_CS_PORT, ST_CS_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(ST_SCL_PORT, ST_SCL_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(ST_SDA_PORT, ST_SDA_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(ST_PCLK_PORT, ST_PCLK_PIN, GPIO_PIN_RESET);

	g.Mode  = GPIO_MODE_OUTPUT_PP;
	g.Pull  = GPIO_NOPULL;
	g.Speed = GPIO_SPEED_FREQ_LOW;

	/* Per-pin / per-port HAL_GPIO_Init calls, which are read-modify-write: PG6
	   (OCTOSPI1_CS) sits right next to ST_PCLK_PIN = PG7 and must not move. */
	g.Pin = ST_CS_PIN;
	HAL_GPIO_Init(ST_CS_PORT, &g);
	g.Pin = ST_SDA_PIN | ST_SCL_PIN;
	HAL_GPIO_Init(ST_SDA_PORT, &g);
	g.Pin = ST_PCLK_PIN;
	HAL_GPIO_Init(ST_PCLK_PORT, &g);
}

void st7789_rgb_init(void)
{
	/* No settle needed on entry.  The factory image inserted a 1 ms wait here,
	   but that covered the gap between parking the pins and the first frame --
	   and st7789_rgb_pins_init() now runs a whole reset pulse earlier.  What the
	   ST7789 actually requires, 5 ms between RESX going high and the first
	   command, is already provided four times over by ltdc_panel_reset(). */
	st_send_sequence();
	/* Pins are left as GPIO: the caller re-arms the LTDC alternate function once
	   it is ready to start scanning (see st7789_rgb.h). */
}
