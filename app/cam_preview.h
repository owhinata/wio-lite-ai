/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    cam_preview.h
 * @brief   Live camera preview on the LCD (issue #8 phase 3c, issue #35).
 *
 * Shows the camera on the panel, rotated into the landscape drawing surface
 * (issue #38).  Since issue #35 the preview OWNS its stream: it starts a band
 * stream (camera_band_start) in which the DCMI lands 60-row slices in AXI-SRAM,
 * transposes each band into the LTDC back buffer as it arrives, and presents once
 * the fourth one is in.
 *
 * That replaced pinning whole frames out of the PSRAM ring, where the transpose's
 * strided side was the external OCTOSPI1 -- 25 ms of CPU per frame and enough
 * extra bus traffic to make the DCMI's FIFO complain.  `camera stream start` still
 * exists unchanged for whole frames into the ring; the two are exclusive.
 *
 * This lives in app/ rather than in either driver because it is the one thing
 * that needs BOTH -- port/camera for the pixels and port/ltdc for the display --
 * and port/ modules do not reach sideways into each other.  It is the same shape
 * as app/nx_net.c gluing NetX Duo to the RTL8720 link.
 */
#ifndef CAM_PREVIEW_H
#define CAM_PREVIEW_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Create the preview thread (parked).  Call from tx_application_define(). */
int cam_preview_init(void);

/**
 * Turn the preview on or off, starting / stopping the camera's band stream with
 * it.  @p colorbar selects the sensor's test pattern (ignored when turning off) --
 * the same picture `camera stream start test` produces, through the same path, so
 * a suspect image can be split into "optics" and "everything after the sensor".
 *
 * @return 0, or <0 when the LCD is down / its scan-out is off (turning the
 *         preview on then would silently do nothing, which is worse than a
 *         refusal that says why), or when the camera refuses to stream -- which
 *         it does while a `camera stream` or a `camera capture` owns the DCMI.
 */
int cam_preview_enable(int on, int colorbar);
int cam_preview_enabled(void);

/** Frames actually presented, frames skipped because the previous one had not
 *  been presented yet, and how long the most recent frame's transpose took in
 *  microseconds -- summed over its four bands, so it stays directly comparable to
 *  the 25.0 ms per frame the whole-frame-from-PSRAM version measured before issue
 *  #35.  Reported by `camera info`.  Any argument may be NULL. */
void cam_preview_stats(uint32_t *shown, uint32_t *dropped, uint32_t *blit_us);

#ifdef __cplusplus
}
#endif

#endif /* CAM_PREVIEW_H */
