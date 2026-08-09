/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    cmd_lcd.c
 * @brief   `lcd` shell command: LTDC display status + test patterns (issue #7).
 *
 *   lcd info            panel / clock / frame buffer / LTDC error flags
 *   lcd fill <color>    flood the screen (colour name or 0xRGB565 / decimal)
 *   lcd bar             eight colour bars along the long axis (RGB / bit-order check)
 *   lcd grad            horizontal black->white gradient (pixel-clock check)
 *   lcd clear           fill black
 *   lcd anim            bouncing rectangle (tear-free double-buffer demo)
 *   lcd blit            DMA2D M2M demo (copy the colour bars left->right half)
 *   lcd on | lcd off    whole display on/off (backlight + LTDC scanout)
 *   lcd reset           resend the ST7789 power-on sequence to a live panel
 *
 * The LTDC is brought up at boot (app/main.c) with a DMA2D-accelerated,
 * tear-free double buffer in the PSRAM.  Each drawing command paints the back
 * buffer (DMA2D fills/blits or, for the gradient, the CPU) then presents it with
 * ltdc_flip() -- a vertical-blanking reload confirmed via SRCR.VBR.
 *
 * `lcd bar` is the most informative single image: it proves the RGB channel
 * wiring and the RGB565 bit order in one shot.
 *
 * Coordinates here are the LANDSCAPE 320x240 drawing surface, not the panel's
 * 240x320 (issue #38) -- see the header of ltdc_display.h for the mapping and for
 * why the panel cannot do the rotation itself.
 *
 * `lcd off` is also the prerequisite for any `psram` subcommand that retunes
 * OCTOSPI1: scanout reads the frame buffer out of that bus continuously, so
 * app/psram.c refuses its guard while the display is live.
 *
 * Clean-room design; no third-party code reused.
 */
#include "cli.h"
#include "ltdc_display.h"
#include "psram.h"           /* psram_acquire(): OCTOSPI1 interlock (see lcd on) */
#if BSP_ENABLE_CAMERA
#include "camera.h"          /* camera_streaming(): name the right culprit in `lcd on` */
#endif
#include "stm32h7xx_hal.h"   /* __HAL_RCC_DMA2D_IS_CLK_ENABLED (AHB3ENR read-back) */

#include <stdint.h>
#include <string.h>

/* Colour name or numeric RGB565 (0x.... / decimal, 0..0xFFFF). */
static int parse_color(const char *s, uint16_t *out)
{
	static const struct {
		const char *name;
		uint16_t    rgb;
	} names[] = {
		{ "black",   LTDC_RGB565_BLACK   },
		{ "blue",    LTDC_RGB565_BLUE    },
		{ "green",   LTDC_RGB565_GREEN   },
		{ "cyan",    LTDC_RGB565_CYAN    },
		{ "red",     LTDC_RGB565_RED     },
		{ "magenta", LTDC_RGB565_MAGENTA },
		{ "yellow",  LTDC_RGB565_YELLOW  },
		{ "white",   LTDC_RGB565_WHITE   },
	};
	uint32_t v;

	for (unsigned i = 0; i < sizeof names / sizeof names[0]; i++) {
		if (strcmp(s, names[i].name) == 0) {
			*out = names[i].rgb;
			return 0;
		}
	}
	if (cli_parse_u32(s, &v) == 0 && v <= 0xFFFFu) {
		*out = (uint16_t)v;
		return 0;
	}
	return -1;
}

/* Shared guard for the drawing commands. */
static int lcd_ready(struct cli_instance *sh)
{
	if (!ltdc_is_up()) {
		cli_error(sh, "lcd: display not initialized\r\n");
		return 0;
	}
	if (ltdc_scanout_off()) {
		cli_error(sh, "lcd: scanout off (run 'lcd on')\r\n");
		return 0;
	}
	return 1;
}

static int cmd_lcd_info(struct cli_instance *sh, int argc, char **argv)
{
	uint32_t err = ltdc_errors(false);
	uint32_t hz  = ltdc_pixel_clock_hz();
	uint32_t chz = ltdc_refresh_chz();
	struct ltdc_panel p;

	(void)argc;
	(void)argv;
	ltdc_panel_get(&p);

	cli_print(sh, "panel:   ST7789 %ux%u RGB565 (FPC-40, LTDC layer 0)\r\n",
	          (unsigned)p.w, (unsigned)p.h);
	/* The two differ on purpose (issue #38): the panel is portrait, everything
	   drawn on it is addressed landscape, and the rotation is a fixed coordinate
	   transform in ltdc_display.c -- not anything the panel does. */
	cli_print(sh, "surface: %ux%u landscape (drawing coordinates; fb stride stays %u)\r\n",
	          (unsigned)ltdc_surface_w(), (unsigned)ltdc_surface_h(),
	          (unsigned)p.w);
	cli_print(sh, "timing:  hs=%u hbp=%u hfp=%u  vs=%u vbp=%u vfp=%u"
	              "  (total %ux%u)\r\n",
	          (unsigned)p.hs, (unsigned)p.hbp, (unsigned)p.hfp,
	          (unsigned)p.vs, (unsigned)p.vbp, (unsigned)p.vfp,
	          (unsigned)(p.w + p.hs + p.hbp + p.hfp),
	          (unsigned)(p.h + p.vs + p.vbp + p.vfp));
	cli_print(sh, "clock:   %lu.%02lu MHz = PLL3R (VCO %lu MHz / %lu), %lu.%02lu Hz\r\n",
	          (unsigned long)(hz / 1000000u),
	          (unsigned long)(hz % 1000000u / 10000u),
	          (unsigned long)(LTDC_PLL3_VCO_HZ / 1000000u),
	          (unsigned long)ltdc_clock_div(),
	          (unsigned long)(chz / 100u), (unsigned long)(chz % 100u));
	cli_print(sh, "fb:      0x%08lx (.psram_noinit, non-cacheable)\r\n",
	          (unsigned long)(uintptr_t)ltdc_framebuffer());
	cli_print(sh, "buffers: 2 (double, tear-free VBR)\r\n");
	cli_print(sh, "front:   %u\r\n", (unsigned)ltdc_active_buffer());
	/* DMA2D is an on-demand AHB master: it moves bytes only while a fill/blit is
	   in flight and costs no PSRAM bandwidth at idle, so it is not something to
	   "turn off" (the continuous PSRAM reader is LTDC scanout, stopped by
	   'lcd off').  Report the real AHB3ENR gate bit rather than a hardcoded
	   "on" so the line reflects hardware. */
	cli_print(sh, "DMA2D:   %s\r\n",
	          __HAL_RCC_DMA2D_IS_CLK_ENABLED()
	                  ? "on (Chrom-ART clocked; on-demand, idle = no PSRAM traffic)"
	                  : "off (clock gated)");
	cli_print(sh, "state:   %s\r\n",
	          ltdc_scanout_off() ? "up, scanout off (PSRAM free for `psram`)"
	                             : (ltdc_is_up() ? "up, scanout active"
	                                             : "DOWN (init failed)"));
	cli_print(sh, "errors:  underrun=%s transfer=%s\r\n",
	          (err & LTDC_ERRFLAG_FIFO_UNDERRUN) ? "YES" : "no",
	          (err & LTDC_ERRFLAG_TRANSFER_ERROR) ? "YES" : "no");
	return 0;
}

/* Draw-then-present helper: the caller drew into the back buffer; flip it and
   warn (but do not fail the command) if the tear-free swap did not land. */
static void lcd_present(struct cli_instance *sh)
{
	if (ltdc_flip() != LTDC_OK)
		cli_warn(sh, "lcd: present failed\r\n");
}

static int cmd_lcd_fill(struct cli_instance *sh, int argc, char **argv)
{
	uint16_t color;

	(void)argc;
	if (!lcd_ready(sh))
		return 1;
	if (parse_color(argv[1], &color) != 0) {
		cli_error(sh, "lcd: bad colour '%s' (name or 0xRGB565)\r\n", argv[1]);
		return 1;
	}
	ltdc_lock_frame();
	ltdc_fill(color);
	lcd_present(sh);
	ltdc_unlock_frame();
	return 0;
}

static int cmd_lcd_bar(struct cli_instance *sh, int argc, char **argv)
{
	(void)argc;
	(void)argv;
	if (!lcd_ready(sh))
		return 1;
	ltdc_lock_frame();
	ltdc_colorbar();
	lcd_present(sh);
	ltdc_unlock_frame();
	return 0;
}

static int cmd_lcd_grad(struct cli_instance *sh, int argc, char **argv)
{
	(void)argc;
	(void)argv;
	if (!lcd_ready(sh))
		return 1;
	ltdc_lock_frame();
	ltdc_gradient();
	lcd_present(sh);
	ltdc_unlock_frame();
	return 0;
}

static int cmd_lcd_clear(struct cli_instance *sh, int argc, char **argv)
{
	(void)argc;
	(void)argv;
	if (!lcd_ready(sh))
		return 1;
	ltdc_lock_frame();
	ltdc_fill(LTDC_RGB565_BLACK);
	lcd_present(sh);
	ltdc_unlock_frame();
	return 0;
}

static int cmd_lcd_anim(struct cli_instance *sh, int argc, char **argv)
{
	/* A 40x40 rectangle bouncing on a dark background.  ltdc_flip() blocks until
	   the vertical-blanking reload commits, so the loop is naturally paced to the
	   panel's frame rate -- no explicit sleep needed; Ctrl+C is polled once per
	   frame for prompt cancellation. */
	int x = 0, y = 0, dx = 5, dy = 3;
	const int w = 40, h = 40;
	int maxx, maxy;

	(void)argc;
	(void)argv;
	if (!lcd_ready(sh))
		return 1;
	/* The drawing surface, not ltdc_panel_get()'s panel geometry: the API is
	   landscape 320x240 since issue #38, so a rectangle bounced inside 240x320
	   would spend half its travel clipped off the long edge. */
	maxx = (int)ltdc_surface_w() - w;
	maxy = (int)ltdc_surface_h() - h;

	for (;;) {
		int rc;

		ltdc_lock_frame();
		ltdc_fill(0x0008u);                              /* dim background */
		ltdc_fill_rect((uint16_t)x, (uint16_t)y,
		               (uint16_t)w, (uint16_t)h, LTDC_RGB565_CYAN);
		rc = ltdc_flip();
		ltdc_unlock_frame();
		if (rc != LTDC_OK) {
			cli_error(sh, "lcd: present failed\r\n");
			return 1;
		}

		x += dx;
		y += dy;
		if (x <= 0 || x >= maxx) { dx = -dx; x += dx; }
		if (y <= 0 || y >= maxy) { dy = -dy; y += dy; }

		if (cli_cancel_requested(sh))
			break;
	}
	return 0;
}

static int cmd_lcd_blit(struct cli_instance *sh, int argc, char **argv)
{
	/* DMA2D M2M demo (single frame): draw the colour bars into the back buffer,
	   then copy its left half over its right half with a strided M2M blit, and
	   present once.  The right half should mirror the left's first four bars --
	   which is what verifies the M2M path. */
	int rc;

	(void)argc;
	(void)argv;
	if (!lcd_ready(sh))
		return 1;

	ltdc_lock_frame();
	ltdc_colorbar();      /* pattern into the back buffer */
	ltdc_blit_demo();     /* M2M copy left half -> right half (same buffer) */
	rc = ltdc_flip();
	ltdc_unlock_frame();
	if (rc != LTDC_OK) {
		cli_error(sh, "lcd: present failed\r\n");
		return 1;
	}
	return 0;
}

/* `lcd on` / `lcd off`: the whole display.  on = LTDC scanout (LTDCEN) +
   backlight; off = backlight off + scanout stopped, which is also the
   prerequisite for any `psram` subcommand that retunes OCTOSPI1 (app/psram.c
   refuses its guard while the display is live).  These do NOT go through
   lcd_ready() -- `lcd on` must work precisely when scanout is currently off. */
static int cmd_lcd_on(struct cli_instance *sh, int argc, char **argv)
{
	int rc;

	(void)argc;
	(void)argv;
	if (!ltdc_is_up()) {
		cli_error(sh, "lcd: display not initialized\r\n");
		return 1;
	}
	if (ltdc_scanout_active())
		return 0;                 /* already on; nothing to arbitrate */
	/*
	 * Scanout reads the frame buffer out of PSRAM continuously, so it must not
	 * start while a `psram`/`membench` command owns OCTOSPI1.  The guard is HELD
	 * ACROSS the start, not merely probed: releasing it first would leave a
	 * window in which a backgrounded `psram` command grabs OCTOSPI1 and then
	 * retunes it underneath a scanout that has just come up.  Once scanout is
	 * live, app/psram.c's own ltdc_scanout_active() gate refuses new
	 * acquisitions, so handing the guard back here is safe -- that gate is what
	 * keeps the two apart from this point on.
	 */
	/* The SHARED guard: scan-out only reads the frame buffer, so it has to be
	   kept away from a command that is reconfiguring OCTOSPI1 -- but not from a
	   camera stream, which merely writes a different part of the same memory.
	   That is what lets the preview exist at all (issue #8 phase 3c). */
	if (!psram_acquire_shared()) {
		cli_error(sh, "lcd: OCTOSPI1 busy (a psram/membench command or "
		              "`ai stream` holds it)\r\n");
		return 1;
	}
	rc = ltdc_set_scanout(true);
	psram_release();
	if (rc != LTDC_OK) {
		cli_error(sh, "lcd: cannot start scanout\r\n");
		return 1;
	}
	return 0;
}

static int cmd_lcd_off(struct cli_instance *sh, int argc, char **argv)
{
	(void)argc;
	(void)argv;
	if (!ltdc_is_up()) {
		cli_error(sh, "lcd: display not initialized\r\n");
		return 1;
	}
	(void)ltdc_set_scanout(false);
	return 0;
}

/*
 * `lcd reset`: resend the ST7789 power-on sequence to a live panel.
 *
 * The panel has its own supply, so a software reset (`crash`, `reboot`) leaves
 * it awake in a state the sequence used to be unable to recover from -- the
 * screen goes uniformly white and every counter keeps reporting health (issue
 * #43).  `lcd off` / `lcd on` is not that: it toggles LTDCEN and the backlight
 * and never speaks to the panel.  This is the one command that does.
 *
 * The OCTOSPI1 guard is TAKEN FIRST and held across the whole call, not probed:
 * ltdc_panel_recover() stops scanout and starts it again, and in that window a
 * `psram` subcommand could otherwise grab the bus and retune it underneath the
 * scanout we are about to bring back.  Same reasoning as `lcd on` above -- see
 * the time-of-check/time-of-use note on psram_acquire().
 */
static int cmd_lcd_reset(struct cli_instance *sh, int argc, char **argv)
{
	int rc;

	(void)argc;
	(void)argv;
	if (!ltdc_is_up()) {
		cli_error(sh, "lcd: display not initialized\r\n");
		return 1;
	}
	if (!psram_acquire_shared()) {
		cli_error(sh, "lcd: OCTOSPI1 busy (a psram/membench command or "
		              "`ai stream` holds it)\r\n");
		return 1;
	}
	cli_print(sh, "lcd: resetting the panel (~0.3 s)...\r\n");
	rc = ltdc_panel_recover();
	psram_release();
	if (rc != LTDC_OK) {
		cli_error(sh, "lcd: panel reset refused (display faulted)\r\n");
		return 1;
	}
	/* The ST7789 is write-only over this link, so there is nothing to read back:
	   the picture is the check. */
	cli_print(sh, "lcd: panel re-initialized%s -- run 'lcd bar' to check\r\n",
	          ltdc_scanout_off() ? " (scanout still off)" : "");
	return 0;
}

CLI_SUBCMD_SET_CREATE(lcd_subcmds,
	CLI_CMD(info, NULL, "panel / clock / frame buffer / LTDC error flags",
	        cmd_lcd_info),
	CLI_CMD_ARG(fill, NULL, "flood with a colour (name or 0xRGB565)",
	            cmd_lcd_fill, 2, 0),
	CLI_CMD(bar, NULL, "8 vertical colour bars (RGB wiring check)", cmd_lcd_bar),
	CLI_CMD(grad, NULL, "horizontal gradient (pixel-clock check)", cmd_lcd_grad),
	CLI_CMD(clear, NULL, "fill black", cmd_lcd_clear),
	CLI_CMD(anim, NULL, "bouncing rectangle (tear-free double-buffer demo)",
	        cmd_lcd_anim),
	CLI_CMD(blit, NULL, "DMA2D M2M demo (copy bars left->right half)",
	        cmd_lcd_blit),
	CLI_CMD(on, NULL, "display on (backlight + scanout)", cmd_lcd_on),
	CLI_CMD(off, NULL, "display off (frees OCTOSPI1 for `psram`)", cmd_lcd_off),
	CLI_CMD(reset, NULL, "resend the ST7789 power-on sequence (~0.3 s)",
	        cmd_lcd_reset),
	CLI_SUBCMD_SET_END);

CLI_CMD_REGISTER(lcd, lcd_subcmds,
                 "on-board 2.8\" LCD (ST7789 240x320, LTDC + DMA2D)", NULL, 1, 0);
