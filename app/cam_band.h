/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    cam_band.h
 * @brief   Refcounted fan-out for the camera's band stream (issue #9 phase 3).
 *
 * port/camera exposes ONE band callback (camera_band_start() overwrites a single
 * function pointer, and refuses outright while any stream is active).  Two consumers
 * now want 60-row bands: the LCD preview (app/cam_preview.c) and the NN ingest
 * (app/nn_camera.c).  Calling the NN from inside preview_band() would have worked
 * only while the preview was on -- and inference with the preview OFF is the fastest
 * configuration this board has, so that is exactly the case that must not be
 * second-class.  This module owns the one real callback and fans it out.
 *
 * Fan-out order is fixed: PREVIEW first, NN second, so the picture never waits on
 * inference bookkeeping.  Every client makes its own frame-level decision -- the
 * preview dropping a frame because a flip is still pending must not make the NN skip
 * one, and vice versa.
 *
 * Callbacks are supplied at claim time rather than looked up here, and that is a
 * layering requirement rather than a style choice: this file is built whenever
 * BSP_ENABLE_CAMERA is on, while cam_preview.c needs BSP_ENABLE_CAMERA *and*
 * BSP_ENABLE_LCD.  Including cam_preview.h here would drag the LCD into a
 * camera-only build.
 *
 * 🔴 A REFCOUNT ALONE CANNOT DESCRIBE WHAT HAPPENS AFTER A DCMI OVERRUN.  The camera
 * tears its band stream down from the producer side (cam_band_cb = NULL,
 * cam_stream_active = 0; port/camera/camera.c cam_stream_teardown()) and tells
 * nobody, which would leave clients "claimed" with nothing behind them -- claimed and
 * silently dead.  So cam_band_stream_lost() latches that state, the fan-out stops,
 * and NOTHING RE-ARMS ITSELF.  Recovery is re-issuing the command: cam_band_claim()
 * clears the latch and re-arms explicitly.  A stream that quietly restarts after an
 * overrun hides exactly the fault worth seeing.
 *
 * Threading: cam_band_claim() / cam_band_release() are thread context (they sleep).
 * The fan-out runs on the camera's producer thread, never an ISR -- but claim/release
 * and the fan-out synchronise through a brief PRIMASK critical section, the same
 * idiom as nn_session_try_acquire() and blob_busy_acquire().
 */
#ifndef CAM_BAND_H
#define CAM_BAND_H

#include <stdint.h>

/** Fan-out order is this enum's order: the preview is served first. */
enum cam_band_client {
	CAM_BAND_PREVIEW = 0,
	CAM_BAND_NN      = 1,
	CAM_BAND_CLIENTS
};

/**
 * One band, on the camera's producer thread.  @p px is CAMERA_BAND_ROWS rows of
 * RGB565, CAMERA_FRAME_WIDTH wide, in AXI-SRAM, valid ONLY for the duration of the
 * call and read-only (the DMA owns it).  Bands arrive 0..3 in order with no gaps
 * inside a frame.  Whatever runs here delays the next band, so it must finish well
 * inside a band period (~18.5 ms at ~13.5 fps).
 */
typedef void (*cam_band_client_fn)(unsigned band, const uint16_t *px, unsigned rows);

#define CAM_BAND_OK         0
#define CAM_BAND_ERR_ARG   (-1)  /**< bad client id or NULL callback              */
#define CAM_BAND_ERR_START (-2)  /**< the camera refused to start a band stream   */
#define CAM_BAND_ERR_BUSY  (-3)  /**< release: a callback never returned (see below) */
#define CAM_BAND_ERR_STOP  (-4)  /**< the camera did not stop within its timeout  */
#define CAM_BAND_ERR_LOCK  (-5)  /**< the other console held claim/release too long */

/**
 * Claim the band stream for @p c, starting it if it is not running.  Clears the
 * stream-lost latch and re-arms, so re-issuing a command is the documented recovery
 * from a DCMI overrun.  @p colorbar selects the sensor's internal test pattern and
 * only takes effect on the claim that actually starts the stream.
 */
int cam_band_claim(enum cam_band_client c, int colorbar, cam_band_client_fn fn);

/**
 * Drop @p c's claim, stopping the stream when the last client leaves.
 *
 * 🔴 ON CAM_BAND_ERR_BUSY THE CALLER MUST NOT FREE ANYTHING THE CALLBACK TOUCHES.
 * The drain waits for an in-flight callback to return; if it does not (a producer
 * killed mid-callback), the client's buffers may still be written.  For the NN that
 * means the model session and the OCTOSPI1 guard stay held -- handing the arena to
 * `ai bench` while a dead-but-not-returned callback can still write the input tensor
 * is not recoverable, whereas holding a session nobody can use is.
 */
int cam_band_release(enum cam_band_client c);

/** Is @p c currently claimed? */
int cam_band_claimed(enum cam_band_client c);

/** Is any client claimed? */
int cam_band_any_claimed(void);

/**
 * Has the stream died underneath its claimants?  Evaluated lazily by whoever asks --
 * no polling thread exists, and none is needed: the reporting surfaces
 * (`ai stream stats`, `camera info`) and the NN worker's bounded semaphore wait all
 * come through here.  Sticky until the last client releases or a claim re-arms.
 */
int cam_band_stream_lost(void);

/** The colorbar setting the running stream was started with. */
int cam_band_colorbar(void);

#endif /* CAM_BAND_H */
