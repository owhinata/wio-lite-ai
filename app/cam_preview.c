/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    cam_preview.c
 * @brief   Live camera preview on the LCD (issue #8 phase 3c).
 *
 * A thread that pulls the newest published frame out of the camera's ring and
 * puts the middle of it on the panel:
 *
 *     pin newest -> (new generation?) blit + flip -> unpin -> repeat
 *
 * WHY A PULL LOOP AND NOT A frame_pipeline SINK.  The pipeline's push sinks are
 * the obvious fit and are what the f746 firmware uses for its GUIX preview, but
 * a sink's consume() runs ON THE PRODUCER THREAD: a DMA2D blit is ~2 ms and
 * ltdc_flip() then waits for vertical blanking, so the producer would lose ~20 ms
 * of every frame.  That producer's promptness is not a nicety -- it is what keeps
 * the DBM repoint safe (port/camera/camera.c: the memory register may only be
 * written while it is the inactive one, and the guard against writing the active
 * one is that the producer runs immediately after the completion that woke it).
 * Slowing it down would erode the very margin that makes streaming correct.
 *
 * An asynchronous sink would avoid that but brings the detach-quiesce problem
 * instead.  Pulling has neither: camera_stream_pin_latest() / camera_stream_put()
 * carry their own lifetime rules, and the camera side blocks a stream restart
 * while an outside pin is held.
 *
 * WHY THE MIDDLE.  The camera is 320x240 landscape and the panel 240x320
 * portrait, and this SoC cannot rotate: it has no GFXMMU and no GPU2D, and both
 * DMA2D and MDMA can only transpose by turning one side of the copy into ~77k
 * scattered 2-byte accesses -- the worst possible shape for a serial PSRAM.  The
 * panel's own MADCTL row/column exchange was measured and does not apply to
 * RGB-interface pixels (see port/ltdc/st7789_rgb.c).  So the preview shows the
 * centre 240x240 with a black band above and below, and ltdc_blit()'s existing
 * clipping does the crop for free: passing the SOURCE width as w makes it copy
 * 240 columns per row and step the source by the full 320.
 */
#include "cam_preview.h"

#include "camera.h"
#include "ltdc_display.h"
#include "tx_api.h"

#define LOG_TAG "campv"
#include "log.h"
#include "mem_sections.h"  /* DTCM_BSS: CPU-only data out of AXI-SRAM (issue #46) */

/* Below the camera producer (10) -- a late preview frame costs a frame of
   display, a late producer costs the DBM repoint margin.  Above the shell (16)
   so a busy console cannot stall the display. */
#define PREVIEW_PRIO   12u
#define PREVIEW_STACK  1024u
/* Poll interval when the camera has produced nothing new.  The stream runs at
   ~13.5 fps (74 ms), so this adds at most 10 ms of latency and costs 100 cheap
   wake-ups a second when idle. */
#define PREVIEW_IDLE_MS 10u

/* Centre crop: 240 of the camera's 320 columns, placed 40 rows down the panel. */
#define PREVIEW_X_OFF  ((CAMERA_FRAME_WIDTH - 240u) / 2u)   /* 40 px */
#define PREVIEW_Y_POS  ((320u - 240u) / 2u)                 /* 40 rows */

static TX_THREAD preview_thread;
static UCHAR     preview_stack[PREVIEW_STACK] DTCM_BSS __attribute__((aligned(8)));

static volatile int preview_on;
static uint32_t     preview_shown;
static uint32_t     preview_dropped;

static void preview_entry(ULONG arg)
{
	uint32_t last_gen = 0u;
	int have_last = 0;

	(void)arg;
	for (;;) {
		const struct frame_desc *f;

		if (!preview_on) {
			tx_thread_sleep(PREVIEW_IDLE_MS);
			continue;
		}
		/*
		 * Take the display lock BEFORE pinning a frame, and hold it across
		 * blit+flip.
		 *
		 * Order matters twice over.  Holding it across both is what every `lcd`
		 * command does (see cmd_lcd.c) and is what stops another console's
		 * drawing from landing between our blit and our flip -- otherwise we
		 * present someone else's back buffer.  And taking it FIRST means that
		 * when the display lock is contended we wait empty-handed instead of
		 * sitting on a ring slot: a pin held here costs the camera one of its
		 * four buffers.
		 *
		 * The resulting lock order is ltdc -> cam, and nothing goes the other
		 * way (app/psram.c only asks the two predicates and takes neither lock),
		 * so it cannot invert.  ThreadX mutexes are recursive for their owner,
		 * which is why ltdc_blit()/ltdc_flip() taking it again is fine.
		 */
		ltdc_lock_frame();
		f = camera_stream_pin_latest();
		if (f == NULL) {              /* no stream, or nothing published yet */
			ltdc_unlock_frame();
			tx_thread_sleep(PREVIEW_IDLE_MS);
			continue;
		}
		if (!have_last || f->gen != last_gen) {
			/* w is the SOURCE width, not the destination's: ltdc_blit() clips to
			   the panel (240) and advances the source by w - 240 = 80 px per row,
			   which is exactly the centre crop.  See the header comment. */
			ltdc_blit((const uint16_t *)f->data + PREVIEW_X_OFF,
			          0, (uint16_t)PREVIEW_Y_POS,
			          (uint16_t)CAMERA_FRAME_WIDTH, (uint16_t)CAMERA_FRAME_HEIGHT);
			if (ltdc_flip() == LTDC_OK)
				preview_shown++;
			else
				preview_dropped++;
			last_gen  = f->gen;
			have_last = 1;
			camera_stream_put(f);
			ltdc_unlock_frame();
		} else {
			/* Same picture as last time: give the slot straight back -- holding
			   a pin to poll faster would cost the ring a buffer for nothing. */
			camera_stream_put(f);
			ltdc_unlock_frame();
			tx_thread_sleep(PREVIEW_IDLE_MS);
		}
	}
}

int cam_preview_init(void)
{
	if (tx_thread_create(&preview_thread, "cam_prev", preview_entry, 0,
	                     preview_stack, sizeof preview_stack,
	                     PREVIEW_PRIO, PREVIEW_PRIO,
	                     TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS) {
		LOG_ERR("preview thread create failed");
		return -1;
	}
	return 0;
}

int cam_preview_enable(int on)
{
	if (on) {
		if (!ltdc_is_up() || ltdc_scanout_off())
			return -1;         /* nothing would appear; say so instead */
		preview_shown   = 0u;
		preview_dropped = 0u;
	}
	preview_on = on ? 1 : 0;
	return 0;
}

int cam_preview_enabled(void)
{
	return preview_on;
}

void cam_preview_stats(uint32_t *shown, uint32_t *dropped)
{
	if (shown != NULL)
		*shown = preview_shown;
	if (dropped != NULL)
		*dropped = preview_dropped;
}
