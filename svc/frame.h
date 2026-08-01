/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    frame.h
 * @brief   Format-agnostic zero-copy camera frame descriptor (svc/ layer,
 *          issue #47 -- DESIGN PROPOSAL, no implementation yet).
 *
 * The camera frame pipeline distributes one producer (DCMI capture) to many
 * sinks (file / LTDC / Ethernet / VCP).  A captured frame lives in an SDRAM
 * ring slot (.sdram, MPU non-cacheable, #40); it is never copied between the
 * producer and a sink -- only this descriptor (a pointer + geometry + a
 * generation number) is handed across.  @ref frame_desc is the data contract
 * every sink sees; @ref frame_sink and the ring mechanics are in
 * frame_pipeline.h.
 *
 * This header is a clean-room IF proposal: it declares the descriptor and the
 * format enum only.  The ring/dispatch engine (frame_pipeline.c) is implemented
 * later (#46 producer / #45 formats); see docs/{ja,en}/architecture/
 * frame-pipeline.md for the full design and the ownership/back-pressure
 * contract.
 */
#ifndef FRAME_H
#define FRAME_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Pixel / payload format of a frame.  A sink declares which formats it accepts
 * in frame_sink.open(); a mismatch is rejected there.  Network sinks favour
 * JPEG (bandwidth); LTDC favours RGB565 (or converts via DMA2D).
 */
enum frame_format {
	FRAME_FMT_RGB565 = 0, /**< 16-bit little-endian, R5<<11|G6<<5|B5 (capture default) */
	FRAME_FMT_YUV422,     /**< packed YUV 4:2:2                                         */
	FRAME_FMT_Y8,         /**< 8-bit greyscale                                          */
	FRAME_FMT_JPEG,       /**< OV5640 hardware JPEG; @ref frame_desc.bytes is variable  */
};

/**
 * Zero-copy reference to one captured frame in an SDRAM ring slot.
 *
 * The producer fills @ref data (by DMA) and the geometry; the pipeline stamps
 * @ref gen on publish.  A sink receives `const struct frame_desc *` and must
 * treat @ref data as **read-only** -- only the producer writes the slot.  The
 * descriptor is valid while the slot's reference count is nonzero.  Each push
 * delivery is pre-pinned by the pipeline on the sink's behalf; the sink balances
 * it with exactly one frame_pipeline_put() (even on error) -- synchronously
 * inside consume(), or from its own thread for an async sink.  No extra get() is
 * needed merely to keep the frame past consume(); frame_pipeline_get() is only
 * for taking an ADDITIONAL pin (e.g. re-queueing).  See frame_pipeline.h for the
 * full pin/put and detach contract.
 */
struct frame_desc {
	void    *data;   /**< slot start in SDRAM (non-cacheable, #40); sink read-only  */
	uint32_t bytes;  /**< valid payload length; for JPEG the effective stream length
	                      (NOT the DMA transfer length -- DCMI zero-pads the last
	                      32-bit word, RM0385 17.3.9), set by the producer         */
	uint32_t gen;    /**< monotonic generation, bumped per published frame; a
	                      multi-call pull reader compares it to detect a frame that
	                      was replaced between reads (generalises cam_frame_gen)    */
	uint16_t width;  /**< pixels per row                                            */
	uint16_t height; /**< rows                                                      */
	uint16_t stride; /**< bytes per row (0 for JPEG)                                */
	uint8_t  format; /**< enum frame_format                                         */
	uint8_t  flags;  /**< reserved (e.g. end-of-stream marker)                      */
	void    *_slot;  /**< pipeline-internal slot handle; sinks must not touch it    */
};

#ifdef __cplusplus
}
#endif

#endif /* FRAME_H */
