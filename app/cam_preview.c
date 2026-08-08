/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    cam_preview.c
 * @brief   Live camera preview on the LCD (issue #8 phase 3c, issue #35).
 *
 * The pipeline, end to end:
 *
 *     DCMI -> DMA2_Stream1 (double buffer, two FIXED 60-row bands in AXI-SRAM)
 *       -> camera producer thread: invalidate, then transpose the band into the
 *          LTDC back buffer (PSRAM)
 *       -> preview thread: ltdc_flip() once the fourth band is in
 *
 * WHY BANDS.  The camera is 320x240 landscape and the panel 240x320 portrait, and
 * nothing on this board can rotate for free: no GFXMMU, no GPU2D, and the panel's
 * own MADCTL/MV and RAMCTRL/RM do not apply to RGB-interface pixels (the issue
 * #35/#38 spike; port/ltdc/st7789_rgb.c).  So every displayed frame is transposed
 * in software, and a transpose reads one side with a stride.  Issue #38 shipped
 * the honest slow version -- whole frames pinned out of the PSRAM ring -- and it
 * measured 25.0 ms of CPU per frame, 35% of a core, plus ~10 DCMI FIFO errors per
 * 30 s from the DCMI and the LTDC both wanting OCTOSPI1.  Staging the DCMI in
 * AXI-SRAM instead puts the strided reads on internal RAM and takes the camera off
 * that bus altogether, which is why both numbers move together.
 *
 * WHY THE FLIP IS ON ITS OWN THREAD.  ltdc_flip() waits for vertical blanking --
 * up to ~16 ms.  The band transposes run on the CAMERA's producer thread, and
 * whatever runs there delays the next band, which has to be consumed inside one
 * band period (~18.5 ms at ~13.5 fps -- the derivation is on cam_band_service()
 * in port/camera/camera.c).  Spending 16 of those 18 on a blanking wait once a
 * frame would trade nearly the whole margin for nothing.  The producer therefore
 * asks for a flip and returns, and this thread waits.
 *
 * WHICH MAKES THE HANDSHAKE THE INTERESTING PART.  ltdc_flip() swaps front and
 * back, so a flip that lands while the producer is part way through the next
 * frame would split that frame across both buffers.  The producer will not begin a
 * frame while a flip it asked for is still outstanding -- it drops the frame
 * instead, counted as `dropped`.  At ~13.5 fps the flip has ~74 ms to complete, so
 * this is a guard rather than a regular event.
 *
 * WHAT IS NOT DEFENDED, on purpose: the display lock is taken per band rather than
 * held across the whole frame, so an `lcd` command from the other console can draw
 * between two bands.  Holding it across a frame would mean holding it ~95% of the
 * time at 13.5 fps -- any contention would then land squarely on the band deadline
 * and cost real pixels, whereas an interleaved draw costs one cosmetically mixed
 * frame that the next one replaces.  A drawing `lcd` command holds the lock for a
 * DMA2D fill plus a blanking wait, which is close enough to one band period that
 * `band late` / `band torn` can tick; `lcd reset` holds it for ~0.3 s and will.
 * Both recover on the next frame, which is what those counters are for.
 */
#include "cam_preview.h"

#include "camera.h"
#include "ltdc_display.h"
#include "stm32h7xx.h"   /* DWT->CYCCNT: time the rotating blit */
#include "tx_api.h"

#define LOG_TAG "campv"
#include "log.h"
#include "mem_sections.h"  /* DTCM_BSS: CPU-only data out of AXI-SRAM (issue #46) */

/* Below the camera producer (10) -- a late flip costs a frame of display, a late
   producer costs a band.  Above the shell (16) so a busy console cannot stall the
   display. */
#define PREVIEW_PRIO   12u
#define PREVIEW_STACK  1024u

/* The camera frame and the landscape drawing surface are the same size, so the
 * preview is a full-surface blit at the origin -- no crop, no offsets, and each
 * band is a full-width strip of it. */
_Static_assert(CAMERA_FRAME_WIDTH == 320u && CAMERA_FRAME_HEIGHT == 240u,
               "preview assumes the camera matches the 320x240 landscape surface");

static TX_THREAD    preview_thread;
static UCHAR        preview_stack[PREVIEW_STACK] DTCM_BSS __attribute__((aligned(8)));
static TX_SEMAPHORE preview_flip_sem;

static volatile int preview_on;
/* Set by the producer thread when it has handed over a finished back buffer,
   cleared by the preview thread once the flip is done (or has failed).  One
   writer each, and the producer only ever tests it. */
static volatile int preview_flip_pending;
/* Producer-thread only: we are part way through a frame we intend to present. */
static int          preview_in_frame;

static uint32_t     preview_shown;
static uint32_t     preview_dropped;
/* Cycles this frame's four band transposes have taken so far, and the completed
   total from the most recent frame (DWT, 550 MHz).  The total is the figure of
   merit for issue #35 -- 25.0 ms was the whole-frame-from-PSRAM number it had to
   beat. */
static uint32_t     preview_blit_acc;
static uint32_t     preview_blit_cyc;

/*
 * One band, on the camera's producer thread.
 *
 * @px points into the AXI-SRAM band the DMA just finished; the driver has already
 * invalidated it and guarantees bands arrive 0,1,2,3 with no gaps inside a frame,
 * so the only frame-level decision left here is whether to start one.
 */
static void preview_band(void *ctx, unsigned band, const uint16_t *px,
                         unsigned rows)
{
	uint32_t t0;

	(void)ctx;
	if (!preview_on)
		return;

	if (band == 0u) {
		if (preview_flip_pending) {
			/* The previous frame is still on its way to the panel.  Drawing now
			   would land in a buffer that is about to become the front one --
			   skip the whole frame rather than tear it across the swap. */
			preview_in_frame = 0;
			preview_dropped++;
			return;
		}
		preview_in_frame = 1;
		preview_blit_acc = 0u;
	}
	if (!preview_in_frame)
		return;                   /* mid-frame remainder of a frame we skipped */

	/* The band IS the surface here: full width, at its own landscape row offset.
	   ltdc_blit() takes the display lock, clips, and transposes (svc/gfx_rot). */
	t0 = DWT->CYCCNT;
	ltdc_blit(px, 0, (uint16_t)(band * rows),
	          (uint16_t)CAMERA_FRAME_WIDTH, (uint16_t)rows);
	preview_blit_acc += DWT->CYCCNT - t0;

	if (band + 1u >= CAMERA_BANDS_PER_FRAME) {
		preview_in_frame = 0;
		preview_blit_cyc = preview_blit_acc;
		/* Order matters: the flag is what the next band 0 tests, so it has to be
		   set before the thread that clears it can possibly run. */
		preview_flip_pending = 1;
		(void)tx_semaphore_put(&preview_flip_sem);
	}
}

static void preview_entry(ULONG arg)
{
	(void)arg;
	for (;;) {
		if (tx_semaphore_get(&preview_flip_sem, TX_WAIT_FOREVER) != TX_SUCCESS)
			continue;
		if (preview_on) {
			if (ltdc_flip() == LTDC_OK)
				preview_shown++;
			else
				preview_dropped++;
		}
		/* Unconditionally, including after a failed flip: leaving it set would
		   stop the producer from ever starting another frame. */
		preview_flip_pending = 0;
	}
}

int cam_preview_init(void)
{
	if (tx_semaphore_create(&preview_flip_sem, "campv_fl", 0) != TX_SUCCESS) {
		LOG_ERR("preview semaphore create failed");
		return -1;
	}
	if (tx_thread_create(&preview_thread, "cam_prev", preview_entry, 0,
	                     preview_stack, sizeof preview_stack,
	                     PREVIEW_PRIO, PREVIEW_PRIO,
	                     TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS) {
		LOG_ERR("preview thread create failed");
		return -1;
	}
	return 0;
}

int cam_preview_enable(int on, int colorbar)
{
	int rc;

	if (!on) {
		preview_on = 0;
		if (camera_band_streaming())
			(void)camera_stream_stop();
		/* After the stop: the producer is gone, so nothing can set this again. */
		preview_flip_pending = 0;
		preview_in_frame     = 0;
		return 0;
	}

	if (!ltdc_is_up() || ltdc_scanout_off())
		return -1;                 /* nothing would appear; say so instead */
	/* Re-issuing `preview on` restarts rather than no-ops, so that `preview on
	   test` switches an already-running preview to the colour bars.  It is nearly
	   free: the sensor is already configured, so the restart only rewrites COM7's
	   test-pattern bit (camera_configure_locked). */
	if (camera_band_streaming()) {
		preview_on = 0;
		(void)camera_stream_stop();
	}

	preview_shown        = 0u;
	preview_dropped      = 0u;
	preview_blit_acc     = 0u;
	preview_blit_cyc     = 0u;
	preview_flip_pending = 0;
	preview_in_frame     = 0;
	while (tx_semaphore_get(&preview_flip_sem, TX_NO_WAIT) == TX_SUCCESS)
		;
	/* Before the stream, not after: the first band can arrive the moment
	   camera_band_start() returns, and preview_band() drops what it gets while
	   this is clear. */
	preview_on = 1;

	rc = camera_band_start(colorbar, preview_band, NULL);
	if (rc != CAM_OK) {
		preview_on = 0;
		return -1;
	}
	return 0;
}

int cam_preview_enabled(void)
{
	return preview_on;
}

void cam_preview_stats(uint32_t *shown, uint32_t *dropped, uint32_t *blit_us)
{
	if (shown != NULL)
		*shown = preview_shown;
	if (dropped != NULL)
		*dropped = preview_dropped;
	/* DWT counts CPU cycles and the app inherits a 550 MHz core (SystemCoreClock),
	   which is where the divisor comes from -- not a hardcoded 550. */
	if (blit_us != NULL)
		*blit_us = preview_blit_cyc / (SystemCoreClock / 1000000u);
}
