/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    camera.h
 * @brief   OV2640 DVP camera over DCMI: XCLK, SCCB, snapshot capture (issue #8
 *          phases 1-2).
 *
 * The board's J7 FPC-24 connector carries a plain 8-bit DVP camera into the
 * STM32H725's DCMI.  Phase 1 brought up everything the sensor needs *before* a
 * single pixel can move -- the master clock, the SCCB (I2C) control bus, the
 * PWDN/RESETB lines and a chip-ID read (the part in hand is an OV2640, SCCB
 * 0x30, chip ID 0x2642).  Phase 2 adds the pixel path: the QVGA RGB565 register
 * sequence, and one frame captured over DCMI + DMA2_Stream1 into a PSRAM frame
 * buffer.  Continuous capture and the frame pipeline are phase 3.
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
#define CAM_ERR_BUSY      -7   /* a capture already owns the DCMI               */

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

/** SCCB read style (see struct camera_tuning).  OmniVision documents reads as
 *  two separate transactions; most parts also tolerate a repeated START, which
 *  is what HAL_I2C_Mem_Read emits.  Both are selectable because "the sensor
 *  ACKs its address but every read returns 0xFF" is otherwise a dead end. */
#define CAM_SCCB_SPLIT    0u   /* write reg + STOP, then a separate read (default) */
#define CAM_SCCB_RESTART  1u   /* single transfer with a repeated START            */

/** Capture geometry.  Phase 2 has exactly one mode; the struct exists so the
 *  shell never hardcodes the numbers and phase 3 can add modes without changing
 *  the command surface. */
#define CAMERA_FRAME_WIDTH   320u
#define CAMERA_FRAME_HEIGHT  240u
#define CAMERA_FRAME_BYTES   (CAMERA_FRAME_WIDTH * CAMERA_FRAME_HEIGHT * 2u)

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

/** Bring-up escape hatches.  Every one of these is a board fact this phase
 *  cannot prove from the schematic alone, so each is switchable at run time --
 *  the internal flash is only rated for ~10k erase cycles and sweeping an
 *  unknown by reflashing burns them (the lesson from issue #7's LCD bring-up). */
struct camera_tuning {
	uint8_t pwdn_active_high; /**< 1 = PWDN high powers the sensor down (default) */
	uint8_t rst_active_low;   /**< 1 = RESETB low holds the sensor in reset (default) */
	uint8_t i2c_pullup;       /**< 1 = also enable the MCU's internal pull-ups       */
	uint8_t sccb_style;       /**< CAM_SCCB_SPLIT / CAM_SCCB_RESTART                 */
	uint8_t dvp_swap;         /**< 1 = IMAGE_MODE bit0 set (ST's shipped default)    */
	uint32_t i2c_hz;          /**< nominal SCCB bit rate: 50000 / 100000 / 400000    */
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

/**
 * Frames discarded before the one camera_capture() keeps (default 15, max 31).
 *
 * The OV2640's exposure loop starts from the register table's defaults, so the
 * frames right after a power cycle are badly under-exposed -- measured here, the
 * first frame at 0 is seven times too dark and three back-to-back captures still
 * have not converged, while 15 discarded frames put the first kept frame at the
 * settled value.  0 restores the single-frame behaviour, which is what makes the
 * effect measurable instead of a matter of belief.
 */
int      camera_set_warm_frames(unsigned n);
unsigned camera_get_warm_frames(void);

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

/**
 * Retune XCLK.  @p hz is clamped to the range the TIM5 divider can express
 * (roughly 4..46 MHz off the 275 MHz kernel clock); @p actual (may be NULL)
 * receives the frequency really emitted.  Takes effect immediately when the
 * clock is already running.  The default is ~22.92 MHz = 275 MHz / 12, the
 * closest exact-50%-duty point to the 24 MHz an OmniVision sensor expects.
 */
int camera_set_xclk(uint32_t hz, uint32_t *actual);

/** Current tuning knobs / replace them.  A change to the I2C fields
 *  re-initialises I2C4; a change to a polarity takes effect on the next
 *  camera_on()/camera_off(). */
int camera_get_tuning(struct camera_tuning *out);
int camera_set_tuning(const struct camera_tuning *in);

/** Human-readable forms for the shell. */
const char *cam_strerror(int rc);
const char *camera_sensor_name(unsigned sensor);

#ifdef __cplusplus
}
#endif

#endif /* CAMERA_H */
