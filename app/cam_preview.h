/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    cam_preview.h
 * @brief   Live camera preview on the LCD (issue #8 phase 3c).
 *
 * Shows the running camera stream on the panel: it pins the newest published
 * frame, blits the centre of it into the LTDC back buffer and presents.
 *
 * This lives in app/ rather than in either driver because it is the one thing
 * that needs BOTH -- port/camera for the frames and port/ltdc for the display --
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
 * Turn the preview on or off.  Independent of the stream: enabling it without a
 * running stream is legal and simply shows nothing until frames arrive, so
 * `camera stream start` does not have to imply a display.
 *
 * @return 0, or <0 when the LCD is down / its scan-out is off (turning the
 *         preview on then would silently do nothing, which is worse than a
 *         refusal that says why).
 */
int cam_preview_enable(int on);
int cam_preview_enabled(void);

/** Frames actually presented, and frames skipped because the LTDC could not be
 *  handed a new buffer.  Reported by `camera info`. */
void cam_preview_stats(uint32_t *shown, uint32_t *dropped);

#ifdef __cplusplus
}
#endif

#endif /* CAM_PREVIEW_H */
