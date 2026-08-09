/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    camera.h
 * @brief   OV2640 DVP camera over DCMI: XCLK, SCCB, snapshot and continuous
 *          capture (issue #8).
 *
 * The board's J7 FPC-24 connector carries a plain 8-bit DVP camera into the
 * STM32H725's DCMI.  Phase 1 brought up everything the sensor needs *before* a
 * single pixel can move -- the master clock, the SCCB (I2C) control bus, the
 * PWDN/RESETB lines and a chip-ID read (the part in hand is an OV2640, SCCB
 * 0x30, chip ID 0x2642).  Phase 2 adds the pixel path: the QVGA RGB565 register
 * sequence, and one frame captured over DCMI + DMA2_Stream1 into a PSRAM frame
 * buffer.  Phase 3b adds continuous capture: the DCMI runs free with the DMA in
 * double-buffer mode over a ring of PSRAM slots, and a producer thread publishes
 * each finished frame into svc/frame_pipeline.  Phase 3c puts those frames on the
 * LCD (app/cam_preview.c) and lets the DCMI and the LTDC share OCTOSPI1.
 *
 * Two board facts drive the whole design and both differ from the STM32F746
 * Discovery firmware this project is otherwise ported from
 * (../stm32f746g-disco port/camera, B-CAMS-OMV / OV5640):
 *
 *   - **The host must generate XCLK.**  Schematic sheet 7 leaves the 24 MHz
 *     oscillator OSC1 and its series R15 DNP; the only populated path to the
 *     module's XCLK pin is R11 (0R) from DCMI_XCLK = PA2.  The f746's module
 *     clocks itself from an on-board crystal, so that firmware has no XCLK code
 *     at all.  An OmniVision sensor derives its SCCB timing from XVCLK, so
 *     **the I2C bus stays dead until the clock runs** -- the single most
 *     confusing failure mode here.  PA2 is driven by TIM5_CH3 (AF2); TIM2, the
 *     other candidate, is already the ThreadX execution-profile time source
 *     (port/threadx/tx_glue.c).
 *
 *   - **The camera supply cannot be switched off.**  U8 (ME6216A28M3G, SOT23)
 *     feeds VDD_2V8 straight from SYS_5V with no enable pin.  f746's
 *     camera_power_off() cuts a GPIO-controlled rail; here camera_off() can only
 *     assert PWDN, hold RESETB low and stop XCLK.  Say so in help text rather
 *     than pretending the rail is gated.
 *
 * Layering: this module sits in port/ over HAL/CMSIS/ThreadX and pulls in
 * nothing from app/.  Concurrency: every public call serializes on an internal
 * TX_MUTEX and may block, so the API is **thread-context only** -- never from
 * an ISR, and never before camera_init() ran in tx_application_define().
 *
 * Clean-room implementation; RM0468, the board schematic and the OmniVision
 * SCCB protocol description were used as a register / wiring reference only.
 */
#ifndef CAMERA_H
#define CAMERA_H

#include <stdint.h>

#include "frame.h"   /* struct frame_desc: what camera_stream_pin_latest() hands out */

#ifdef __cplusplus
extern "C" {
#endif

/* Error returns (negative); 0 is success.  Same names/meanings as the f746
   driver so the shell layer reads identically across the two firmwares. */
#define CAM_OK             0
#define CAM_ERR_PARAM     -1   /* bad argument                                  */
#define CAM_ERR_HAL       -2   /* HAL / SCCB I/O reported an error              */
#define CAM_ERR_TIMEOUT   -3   /* bus never went idle                           */
#define CAM_ERR_STATE     -4   /* driver not initialized / sensor not powered   */
#define CAM_ERR_NO_SENSOR -5   /* nothing answered, or an unrecognised chip ID  */
#define CAM_ERR_NO_FRAME  -6   /* no captured frame available                   */
#define CAM_ERR_BUSY      -7   /* a capture or stream already owns the DCMI      */

/** Sensors this phase knows how to identify.  CAM_SENSOR_UNKNOWN means "some
 *  device ACKed on the SCCB bus but its ID did not match any candidate" -- a
 *  useful state, not an error to hide: `camera reg` can then poke at it. */
enum camera_sensor {
	CAM_SENSOR_NONE = 0,
	CAM_SENSOR_OV2640,
	CAM_SENSOR_OV5640,
	CAM_SENSOR_UNKNOWN,
};

/** Register-address width of the sensor's control bus, in bits. */
#define CAM_REG_WIDTH_8   8u
#define CAM_REG_WIDTH_16  16u

/** Capture geometry.  Phase 2 has exactly one mode; the struct exists so the
 *  shell never hardcodes the numbers and phase 3 can add modes without changing
 *  the command surface. */
#define CAMERA_FRAME_WIDTH   320u
#define CAMERA_FRAME_HEIGHT  240u
#define CAMERA_FRAME_BYTES   (CAMERA_FRAME_WIDTH * CAMERA_FRAME_HEIGHT * 2u)

/**
 * Band geometry for the staged stream (issue #35).
 *
 * A band is a horizontal slice of the frame that the DMA delivers on its own.
 * 60 rows divides 240 exactly, which is what makes band boundaries coincide with
 * frame boundaries: the DCMI transfers active pixels only (RM0468 sec 36.3.6 /
 * 36.3.7), so a frame is exactly CAMERA_BANDS_PER_FRAME transfers and the fourth
 * one always ends where the frame does.  60 rows is also 38,400 B -- small enough
 * that two of them fit in AXI-SRAM beside everything else, and a multiple of 32 so
 * a cache maintenance operation on one band cannot touch the other.
 */
#define CAMERA_BAND_ROWS        60u
#define CAMERA_BANDS_PER_FRAME  (CAMERA_FRAME_HEIGHT / CAMERA_BAND_ROWS)
#define CAMERA_BAND_BYTES       (CAMERA_FRAME_WIDTH * CAMERA_BAND_ROWS * 2u)

/** Frames a snapshot discards before the one it keeps, so the sensor's own
 *  exposure and gain loops have converged.  Measured, not guessed: see
 *  camera_capture_locked() in camera.c.  Public because it is what makes a
 *  capture take ~1 s, and the shell says so before it blocks. */
#define CAMERA_WARM_FRAMES   15u

struct camera_mode {
	uint16_t width;
	uint16_t height;
	uint32_t frame_bytes;
	const char *format;    /**< "RGB565" */
};

/** Snapshot of driver + sensor state (camera_get_info). */
struct camera_info {
	uint8_t  ready;        /**< camera_init() completed                        */
	uint8_t  powered;      /**< XCLK running, PWDN released, RESETB deasserted */
	uint8_t  sensor;       /**< enum camera_sensor                             */
	uint8_t  i2c_addr;     /**< 7-bit SCCB address that answered, 0 if none    */
	uint8_t  reg_width;    /**< CAM_REG_WIDTH_8 / _16 of the identified sensor */
	uint8_t  configured;   /**< the QVGA RGB565 register sequence is loaded    */
	uint8_t  frame_valid;  /**< the frame buffer holds a captured image        */
	uint16_t chip_id;      /**< product ID (0x2642 = OV2640, 0x5640 = OV5640)  */
	uint16_t manuf_id;     /**< manufacturer ID (0x7FA2 for OmniVision), or 0  */
	uint32_t frame_bytes;  /**< bytes valid in the frame buffer                */
	uint32_t xclk_hz;      /**< XCLK actually being emitted, 0 when stopped    */
	uint32_t xclk_src_hz;  /**< TIM5 kernel clock the divider works from       */
	uint32_t i2c_ker_hz;   /**< I2C4 kernel clock (rcc_pclk4)                  */
};

/**
 * Configure GPIO, TIM5 and I2C4; create the driver mutex.  Idempotent.
 *
 * Touches no sensor: XCLK stays stopped, PWDN stays asserted and RESETB stays
 * low, so the module is left exactly as power-on reset left it.  Enables no
 * interrupt and waits on nothing, which is what makes it safe to call from
 * tx_application_define() before the scheduler runs (issue #12's rule: an
 * interrupt source is only enabled after the ThreadX objects its ISR touches
 * exist -- phase 1 has no interrupt source at all).
 *
 * @return CAM_OK, or CAM_ERR_STATE / CAM_ERR_HAL.
 */
int camera_init(void);

/**
 * Run the power-up sequence: start XCLK, release PWDN, release RESETB, settle.
 * Blocks ~40 ms in tx_thread_sleep.  No SCCB traffic and no identification.
 */
int camera_on(void);

/**
 * Assert PWDN, hold RESETB low and stop XCLK.  Note this does NOT remove power
 * (see the file header) -- it is the deepest sleep this board can reach.
 * Clears the identification, so a later camera_reg_*() needs a fresh probe.
 */
int camera_off(void);

/**
 * Power-cycle the sensor (off, then on) and identify it over SCCB.
 *
 * On success @ref camera_info carries the sensor, its SCCB address, register
 * width and IDs.  On CAM_ERR_NO_SENSOR the sensor is deliberately left POWERED
 * so `camera scan` / `camera reg` can keep investigating -- unlike the f746
 * driver, which powers the module down on a failed probe because there the rail
 * is switchable and leaving it on costs current.
 */
int camera_probe(void);

/** Copy the current state out.  Never fails once camera_init() ran. */
int camera_get_info(struct camera_info *out);

/** Capture geometry / pixel format.  Constant in phase 2. */
int camera_get_mode(struct camera_mode *out);

/** Readout orientation as configured after ST's table, for `camera info`
 *  (issue #53).  Never NULL. */
const char *camera_get_orient(void);

/** Which shape of stream owns the DCMI (camera_stream_stats.mode). */
enum camera_stream_mode {
	CAM_STREAM_NONE = 0,   /**< nothing running                               */
	CAM_STREAM_FRAME,      /**< whole frames into the PSRAM ring + pipeline   */
	CAM_STREAM_BAND,       /**< 60-row bands into AXI-SRAM, pushed to a sink  */
};

/** Streaming counters (camera_stream_stats).
 *
 *  @p mode says which of the fields below mean anything: the ring, the repoint
 *  and the sink counters exist only in CAM_STREAM_FRAME.  A band-mode reader must
 *  not print them as zero -- zero would read as "no overruns" when the truth is
 *  "there is no ring here at all". */
struct camera_stream_stats {
	uint8_t  active;       /**< a stream is running right now                 */
	uint8_t  mode;         /**< enum camera_stream_mode; the LAST run's, once
	                        *   stopped, so a post-mortem `stats` still parses */
	uint32_t frames;       /**< frames published (frame mode) / completed (band) */
	uint32_t elapsed_ms;   /**< run time (frozen at teardown for a post-stop read) */
	uint32_t dcmi_ovr;     /**< DCMI overrun / sync errors (terminal)         */
	uint32_t ring_ovr;     /**< frame mode: completions with no free slot     */
	uint32_t dma_fe;       /**< DMA FIFO/direct-mode errors tolerated          */
	uint32_t repoint_skip; /**< frame mode: DBM repoints skipped (see .c)     */
	uint32_t delivered;    /**< frame mode, stat sink: frames consumed        */
	uint32_t dropped;      /**< frame mode, stat sink: frames dropped         */
	uint32_t slots;        /**< frame mode: ring depth                        */
	uint32_t band_late;    /**< band mode: bands the consumer did not reach   */
	uint32_t band_torn;    /**< band mode: bands the DMA caught up with       */
	uint32_t band_desync;  /**< band mode: frame ends off a band boundary     */
};

/**
 * Start / stop continuous capture.
 *
 * The DCMI runs in continuous mode with the DMA in double-buffer mode over a
 * ring of frame slots; a dedicated producer thread publishes each completed
 * frame into the pipeline.  @p frames / @p secs are optional self-stop limits
 * (0 = run until camera_stream_stop()).  @p colorbar selects the test pattern.
 *
 * Streaming and camera_capture() share the DCMI and are mutually exclusive.
 *
 * The frames land in the OCTOSPI1 PSRAM, so while a stream runs nothing else may
 * retune that bus -- psram_acquire() refuses on camera_streaming() for exactly
 * that reason.  The caller does NOT hold a guard for the stream's lifetime
 * (a stream is open-ended and can self-stop, so there would be no one left to
 * release it).
 */
int camera_stream_start(int colorbar, uint32_t frames, uint32_t secs);
int camera_stream_stop(void);

/**
 * Band consumer, called from the producer thread once per delivered band.
 *
 * @param band  0 .. CAMERA_BANDS_PER_FRAME-1, always in that order: the driver
 *              drops the remainder of a frame rather than hand out a gap, so a
 *              consumer that starts a frame at band 0 gets all of it or nothing.
 * @param px    CAMERA_BAND_ROWS rows of RGB565, CAMERA_FRAME_WIDTH wide, in
 *              AXI-SRAM.  Valid only for the duration of the call, and READ-ONLY:
 *              the DMA owns this memory and the driver has just invalidated it.
 *
 * Thread context (never an ISR), but on the DRIVER'S producer thread -- what runs
 * here delays the next band, so it must finish well inside a band period (~18 ms
 * at the sensor's ~13.5 fps).  Do not block on anything slow here; the display
 * flip in app/cam_preview.c is deliberately handed to another thread.
 */
typedef void (*camera_band_fn)(void *ctx, unsigned band, const uint16_t *px,
                               unsigned rows);

/**
 * Start a BAND stream: the DCMI lands in AXI-SRAM, not in the PSRAM ring.
 *
 * WHY THIS MODE EXISTS (issue #35).  The panel is portrait and the camera
 * landscape, so every displayed frame is transposed, and a transpose reads one
 * side with a stride.  With whole frames in PSRAM that strided side was the
 * external OCTOSPI1 -- 25 ms of CPU per frame, and enough extra bus arbitration
 * to make the DCMI's FIFO complain.  Staging 60-row bands in AXI-SRAM puts the
 * strided reads on internal RAM and takes the DCMI off OCTOSPI1 altogether.
 *
 * The DMA runs in double-buffer mode over two fixed band buffers and the memory
 * registers are never rewritten, so the whole on-the-fly repoint dance that frame
 * mode needs (RM0468 sec 15.3.11) does not exist here.
 *
 * Band mode publishes nothing into svc/frame_pipeline and fills no ring, so
 * camera_stream_pin_latest() finds nothing and `camera save` / `send` keep
 * showing the last camera_capture().  It is exclusive with camera_stream_start()
 * and with camera_capture(); stop it with camera_stream_stop() like any stream.
 *
 * @return CAM_OK, CAM_ERR_PARAM (@p fn NULL), CAM_ERR_BUSY, CAM_ERR_HAL, ...
 */
int camera_band_start(int colorbar, camera_band_fn fn, void *ctx);

/** Nonzero while a BAND stream specifically is running (camera_streaming() is
 *  true for either mode -- that one gates the OCTOSPI1 retune). */
int camera_band_streaming(void);

/** Nonzero while a stream owns the DCMI and is writing the PSRAM ring.
 *  Read by app/psram.c to refuse OCTOSPI1 retunes -- see psram_acquire(). */
int camera_streaming(void);

int camera_stream_stats(struct camera_stream_stats *out);

/**
 * Pin the newest streamed frame for read-only access, or NULL if none.
 *
 * Frame mode only: a band stream fills no ring, so this returns NULL throughout
 * one rather than handing back whatever the previous frame-mode run left behind.
 *
 * For a consumer that wants to look at live frames without going through a
 * push sink -- the LCD preview in app/cam_preview.c.  The returned pointer stays
 * valid, and the producer will not recycle that slot, until the caller balances
 * it with exactly one camera_stream_put().  Holding it costs the ring one slot,
 * so hold it for the work and no longer.
 *
 * Pinning takes the driver lock briefly; releasing does not.  That asymmetry is
 * deliberate: a stream start has to wait for outstanding pins to drain before it
 * re-initialises the pipeline, and it does that waiting with the driver lock
 * held -- so a put() that needed the same lock would deadlock against it, while
 * a pin() that needs it is exactly what must be kept out.
 *
 * The descriptor carries the slot address, length and geometry, plus a
 * generation counter -- an unchanged gen means the same picture, which is how a
 * preview skips redundant work.
 */
const struct frame_desc *camera_stream_pin_latest(void);
void camera_stream_put(const struct frame_desc *f);

/**
 * Configure the sensor if needed, then capture ONE frame into the driver's
 * frame buffer over DCMI + DMA.  Blocks until the frame lands or the transfer
 * times out (~1 s); the first call after a power cycle also spends ~250 ms
 * writing the QVGA RGB565 register sequence.
 *
 * @param colorbar  nonzero selects the sensor's internal test pattern instead
 *                  of the live image -- it proves the DCMI/DMA path without
 *                  involving optics or lighting.
 *
 * The frame buffer lives in the OCTOSPI1 PSRAM.  This call does NOT arbitrate
 * that bus: the caller must hold the psram guard (the shell does), because a
 * concurrent retune of OCTOSPI1 would stall the AXI, not merely spoil the
 * frame.
 *
 * @return CAM_OK, CAM_ERR_NO_SENSOR, CAM_ERR_HAL (sync/overrun/DMA error),
 *         CAM_ERR_TIMEOUT (no frame arrived), CAM_ERR_BUSY, CAM_ERR_STATE.
 */
int camera_capture(int colorbar);

/**
 * Copy @p len bytes at @p offset out of the captured frame.
 *
 * @param gen  (may be NULL) receives the frame generation counter.  A caller
 *             that reads the frame in several calls must check it does not
 *             change between them -- frame_valid alone cannot tell that a
 *             concurrent capture replaced the pixels underneath.
 * @return CAM_OK, CAM_ERR_NO_FRAME, CAM_ERR_PARAM, CAM_ERR_STATE.
 */
int camera_frame_read(uint32_t offset, void *dst, uint32_t len, uint32_t *gen);

/**
 * Scan 7-bit SCCB addresses 0x08..0x77 and store the ones that ACK.
 *
 * @param addrs  caller buffer receiving the ACKing addresses (may be NULL)
 * @param max    capacity of @p addrs
 * @param found  receives the number of devices found (may exceed @p max)
 * @return CAM_OK, or CAM_ERR_STATE when the sensor is not powered.
 */
int camera_scan(uint8_t *addrs, unsigned max, unsigned *found);

/**
 * Raw SCCB register access at an explicit address and width -- the tool for a
 * sensor camera_probe() could not identify, and the one used to debug register
 * sequences in phase 2.  @p reg_width is CAM_REG_WIDTH_8 or _16.
 */
int camera_reg_read(uint8_t i2c_addr, uint16_t reg, unsigned reg_width,
                    uint8_t *val);
int camera_reg_write(uint8_t i2c_addr, uint16_t reg, unsigned reg_width,
                     uint8_t val);

/** Human-readable forms for the shell. */
const char *cam_strerror(int rc);
const char *camera_sensor_name(unsigned sensor);

#ifdef __cplusplus
}
#endif

#endif /* CAMERA_H */
