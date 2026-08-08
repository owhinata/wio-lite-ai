/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    gfx_rot.c
 * @brief   90-degree rotating RGB565 blit.  See gfx_rot.h for why it exists and
 *          which side of the transpose pays.
 */
#include "gfx_rot.h"

#include <stddef.h>   /* NULL */
#include <stdint.h>   /* uintptr_t */

/*
 * Pixels staged between the strided gather and the contiguous store.
 *
 * 🔴 THE STAGING IS THE OPTIMISATION.  It looks like pointless copying -- read a
 * pixel, put it in a buffer, copy the buffer out -- and the obvious "simplification"
 * of storing each pixel straight to the destination was tried and MEASURED SLOWER
 * on hardware: 34.5 ms per 320x240 frame against 25.7 ms for the staged version
 * (board #2, camera preview out of a PSRAM ring slot, `camera info`).
 *
 * The reason is that both sides are usually the same external OCTOSPI PSRAM.
 * Interleaving read/write/read/write turns the bus around on every pixel and
 * serialises loads that have no dependency on each other; batching lets the
 * strided reads overlap and then hands the bus a single run of stores.  So the
 * buffer is not a copy, it is a burst separator -- do not "clean it up".
 *
 * Measured on board #2 via `camera info`'s per-frame blit time, 320x240 out of a
 * PSRAM ring slot:
 *
 *     staged, byte-loop stores    25.70 ms   (153,600 store transactions)
 *     unstaged, halfword stores   34.50 ms   (bus turnaround per pixel)
 *     staged, 32-bit pair stores  25.05 ms   ( 38,400 store transactions)
 *
 * So batching is what buys time (-26%), and store width buys almost none of it:
 * quartering the transactions freed 0.65 ms.  That is the useful conclusion --
 * essentially all of the ~25 ms is the strided READS, which is why issue #35
 * attacks the source (DCMI staged in AXI-SRAM) and not this loop.
 *
 * The pair stores are kept anyway, for the bus rather than the clock.  This
 * preview provokes DCMI FIFO errors by competing with the camera's DMA for
 * OCTOSPI1 -- 10 in 30 s, and zero when the preview is off -- and the lesson from
 * issue #8 phase 3c is that what saturates that bus is the arbitration COUNT, not
 * the byte rate.  A 5 s sample after this change showed none, which is short
 * enough to be luck (the same window had shown 4 before) but points the right way.
 *
 * 🔴 AND THE STORES BELOW ARE volatile FOR A REASON THAT COST A MEASUREMENT.
 * They were first written as a plain 32-bit copy loop, on the theory that a
 * quarter as many transactions would help.  GCC's loop-distribute-patterns
 * recognised the loop and turned it back into a call to memcpy() -- which in
 * newlib-nano is a BYTE loop -- so the "32-bit" build was bit-for-bit the
 * byte-store build, measured 25.66 ms, and was read as "store width does not
 * matter".  It was not an experiment; it was the same binary twice.  The objdump
 * check caught it.  A per-file -fno-tree-loop-distribute-patterns would not be
 * trustworthy either: this firmware links with LTO, where compile-only flags are
 * advisory (see CMakeLists.txt).  volatile is what actually holds.
 *
 * It is a local, and locals are in DTCM since issue #46: zero wait states, no
 * cache line to evict, and no static state to make this non-reentrant.  64 pixels
 * is 128 B, comfortably inside the smallest thread stack here (512 B).
 */
#define GFX_RUN_PIXELS  64u

void gfx_blit_rot(const uint16_t *src, uint32_t src_stride_px,
                  uint16_t *fb, uint32_t fb_stride_px, uint32_t fb_rows,
                  uint32_t x0, uint32_t y0, uint32_t w, uint32_t h)
{
	/* A union rather than a cast so the 32-bit view is properly aligned by
	   construction.  (The build is -fno-strict-aliasing anyway -- CMakeLists.txt
	   line ~579 -- but relying on that for something this easy to state exactly
	   would be sloppy.) */
	union {
		uint16_t px[GFX_RUN_PIXELS];
		uint32_t pair[GFX_RUN_PIXELS / 2u];
	} run;
	uint32_t sx;

	if (src == NULL || fb == NULL || w == 0u || h == 0u)
		return;

	for (sx = 0u; sx < w; sx++) {
		/* Landscape column x0+sx becomes frame-buffer row fb_rows-1-(x0+sx),
		   and the column's pixels run along that row from y0.  One contiguous
		   destination run per source column -- that is the whole trick. */
		uint16_t       *dst = fb + (fb_rows - 1u - (x0 + sx)) * fb_stride_px + y0;
		const uint16_t *col = src + sx;
		uint32_t        left = h;

		while (left != 0u) {
			uint32_t n = (left < GFX_RUN_PIXELS) ? left : GFX_RUN_PIXELS;
			uint32_t i;

			/* Gather down the source column: strided, and cheap only when the
			   source is AXI-SRAM or flash.  From PSRAM this is the whole cost of
			   the operation -- which is what issue #35 moves off this bus. */
			for (i = 0u; i < n; i++) {
				run.px[i] = *col;
				col += src_stride_px;
			}

			/* Store the run: 32-bit pairs when the destination is 4-byte
			   aligned, which quarters the transaction count on a non-cacheable
			   window.  A full run is 64 px = 128 B, so an aligned run start keeps
			   every later run in this column aligned and the test costs one
			   branch per run rather than per pixel; only the final short run can
			   have an odd length, hence the halfword tail.

			   volatile is load-bearing, not decoration -- see the header comment.
			   Without it the compiler rewrites these loops as memcpy() calls and
			   the store width silently reverts to bytes. */
			if ((((uintptr_t)dst) & 3u) == 0u) {
				volatile uint32_t *d32 = (volatile uint32_t *)(void *)dst;
				uint32_t pairs = n / 2u;

				for (i = 0u; i < pairs; i++)
					d32[i] = run.pair[i];
				if ((n & 1u) != 0u)
					dst[n - 1u] = run.px[n - 1u];
			} else {
				volatile uint16_t *d16 = (volatile uint16_t *)dst;

				for (i = 0u; i < n; i++)
					d16[i] = run.px[i];
			}

			dst  += n;
			left -= n;
		}
	}
}
