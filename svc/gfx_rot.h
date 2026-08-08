/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    gfx_rot.h
 * @brief   90-degree rotating RGB565 blit (issues #38, #35).
 *
 * The panel is 240x320 portrait and everything worth putting on it is landscape:
 * the camera is 320x240, and so was the board's factory firmware, which drew in a
 * 320x240 coordinate system and transposed per pixel on its way into a 240-stride
 * frame buffer.  This part cannot rotate for us -- no GFXMMU, no GPU2D, nothing in
 * the LTDC -- and the panel will not either: MADCTL/MV was measured dead on the
 * RGB interface in issue #8 phase 3a, and RAMCTRL's RM bit, the last untried knob,
 * came back negative too (see port/ltdc/st7789_rgb.c, and do not re-open it).
 *
 * So the transpose is ours to do, and this is the one place that does it.
 *
 * WHY IT LIVES IN svc/.  It touches no hardware -- no HAL, no CMSIS, no ThreadX,
 * just pixels and strides.  Both callers are above it in the dependency order
 * (port/ltdc's landscape drawing API, and the camera pipeline in issue #35), so a
 * shared, hardware-free service is the honest home; putting it in port/ltdc would
 * make the camera path depend on the display driver for arithmetic.
 *
 * ---- The one thing to understand before using it ---------------------------
 *
 * A transpose cannot make both sides sequential.  One of them walks with a stride,
 * and WHICH ONE lands where decides the cost far more than the pixel count does.
 * This implementation reads the source with a stride and writes the destination in
 * contiguous runs, which is the right way round for a portrait frame buffer in the
 * external PSRAM:
 *
 *   - destination: for a fixed source column, successive source rows land at
 *     successive frame-buffer addresses, so each column becomes ONE contiguous run
 *     of `h` pixels.  Serial PSRAM is happy with that.
 *   - source: read with `src_stride_px` between rows.  Free from AXI-SRAM or the
 *     internal flash; expensive from PSRAM.
 *
 * 🔴 So do not hand this a source in PSRAM and expect the camera to work.  Issue
 * #35 exists precisely to stage DCMI frames through an AXI-SRAM band first; a
 * PSRAM-to-PSRAM transpose is the shape issue #8 phase 3a called "the worst
 * possible for a serial PSRAM" and measured as such.
 */
#ifndef GFX_ROT_H
#define GFX_ROT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Copy a landscape RGB565 bitmap into a portrait frame buffer, rotated.
 *
 * Source pixel (sx, sy) is placed at landscape position (x0 + sx, y0 + sy), which
 * in the frame buffer is
 *
 *     fb[(fb_rows - 1 - (x0 + sx)) * fb_stride_px + (y0 + sy)]
 *
 * i.e. the mapping the board's factory firmware called rotation 2.  The caller is
 * responsible for clipping: every landscape pixel this writes must be inside the
 * surface, or it will write outside @p fb.
 *
 * @param src            top-left of the source bitmap, RGB565, row-major
 * @param src_stride_px  pixels from one source row to the next (>= @p w)
 * @param fb             frame-buffer base (pixel 0 of frame-buffer row 0)
 * @param fb_stride_px   pixels per frame-buffer row (the panel width, 240)
 * @param fb_rows        frame-buffer rows (the panel height, 320)
 * @param x0, y0         landscape position of the source's top-left pixel
 * @param w, h           source size in landscape pixels
 */
void gfx_blit_rot(const uint16_t *src, uint32_t src_stride_px,
                  uint16_t *fb, uint32_t fb_stride_px, uint32_t fb_rows,
                  uint32_t x0, uint32_t y0, uint32_t w, uint32_t h);

#ifdef __cplusplus
}
#endif

#endif /* GFX_ROT_H */
