/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    cmd_camera.c
 * @brief   `camera` shell command: OV2640 bring-up, snapshot, continuous stream
 *          and LCD preview (issue #8).
 *
 *   camera probe          power-cycle the module and identify the sensor
 *   camera on | off       run / undo the power-up sequence (no ID read)
 *   camera info           driver, sensor, XCLK, SCCB and frame state
 *   camera capture [test] snapshot one frame; 'test' = the sensor's colour bars
 *   camera save <path>    write the frame raw to the microSD
 *   camera send [name]    stream the frame to the PC over YMODEM
 *   camera scan           list every 7-bit SCCB address that ACKs
 *   camera reg <a> <r> .. raw SCCB register read / write
 *
 * `camera probe` proves the sensor-control side: a chip ID means XCLK is at a
 * frequency the sensor accepts, PWDN and RESETB have the polarity assumed, and
 * I2C4 is talking at the right bit rate -- four things at once.  `camera capture`
 * proves the pixel side; read its channel statistics, not just its exit status.
 * Its `seam:` line reports the largest step between adjacent rows' channel means:
 * a settled frame is well under 1, and anything in the tens is a gain landing
 * mid-readout.
 *
 * READ THE RATIO BETWEEN THE THREE CHANNELS, NOT JUST dR (issue #44).  A gain
 * landing mid-readout lands once, at a varying size.  A DEAD DVP DATA LINE gives
 * the same step size in EVERY frame at a random row, and the ratio between the
 * channels names the line, because a DVP bit sits at a fixed weight in each
 * channel.  With the byte order this driver uses (IMAGE_MODE bit 0 set, so the
 * bytes land in arrival order and a pixel reads little-endian):
 *
 *     high byte  R4 R3 R2 R1 R0 G5 G4 G3      low byte  G2 G1 G0 B4 B3 B2 B1 B0
 *
 *     line   dR    dG    dB          line   dR    dG    dB
 *     D7     16     4     0          D3      1     0     8
 *     D6      8     2     0          D2      0    32     4
 *     D5      4     1     0          D1      0    16     2
 *     D4      2     0    16          D0      0     8     1
 *
 * Issue #44 was `dR 8.00  dG 2.05  dB 0.02` in every single capture: DCMI_D6
 * (PE5, ball C3 -> FPC-24 pin 11) was open inside the camera module, so that one
 * bit was sampled off a floating pin.  It read as a constant per row, drifting
 * between the rails every few rows, which is a horizontal stripe -- and since the
 * bit is only 8/31 of R and 2/63 of G, the picture still looked broadly right.
 *
 * NOTHING ELSE CAN SEE THIS.  `ovr dcmi`, `ovr ring`, `dma fe` and `repoint skip`
 * all stayed at zero throughout: neither the DCMI nor the DMA has any way to know
 * that one data bit is lying.  Every counter reading healthy is not evidence that
 * the pixels arrived intact -- this line is.  Swapping the camera module took
 * `dR` from 8.00 to 0.05.
 *
 * The frames live in the PSRAM, so `capture`, `save`, `send` and `stream start`
 * all take the SHARED OCTOSPI1 guard -- which keeps out a command that is
 * reconfiguring the bus, but not the LTDC scanning out of it.  That distinction
 * is what lets `camera preview` put a live image on the display; see
 * cam_psram_take() and app/psram.h.
 *
 * `xclk` and `tune` used to sit here: seven run-time knobs for board facts the
 * schematic could not settle (XCLK rate, PWDN/RESETB polarity, SCCB pull-ups and
 * bit rate, the read style, the DVP byte swap, the warm-up count).  They existed
 * because the internal flash is rated for ~10k erase cycles and sweeping an
 * unknown by reflashing burns them -- the lesson from issue #7's LCD bring-up.
 * The board answered every one of them and has not changed its mind since, so
 * phase 3c-2 folded the answers into constants and deleted the knobs.  `reg`
 * stays: poking the sensor's registers is still how you learn anything about it.
 *
 * Note `camera off` does NOT remove power: the camera's 2V8 rail comes off a
 * fixed LDO with no enable pin (schematic U8), so the deepest state reachable
 * is PWDN asserted + RESETB low + XCLK stopped.
 *
 * Clean-room design; no third-party code reused.
 */
#include "camera.h"
#include "cli.h"
#if BSP_ENABLE_PSRAM
#include "psram.h"        /* psram_acquire(): OCTOSPI1 interlock, see cam_psram_* */
#endif
#if BSP_ENABLE_SD
#include "fs_cmd_core.h"  /* fs_sd_device(): the microSD media + its ownership gates */
#endif
#if BSP_ENABLE_LCD
#include "cam_preview.h"  /* the LCD preview this command switches on and off */
#endif
#include "cmd_xfer.h"     /* xfer_send_source(): YMODEM over the console */

#include <stdint.h>
#include <string.h>

#define CAM_SCAN_MAX 16u
/* Frame I/O chunk.  Exactly one 320-pixel RGB565 row, so `capture` can compute
   per-row statistics without buffering more of the frame than that. */
#define CAM_IO_CHUNK (CAMERA_FRAME_WIDTH * 2u)

/*
 * OCTOSPI1 interlock for every command that touches the frame buffer.
 *
 * The frame lives in the PSRAM, and `psram`/`membench`/`devmem`/`wifi flash`
 * can RETUNE that bus.  Doing so while the DCMI's DMA is writing it -- or while
 * `save`/`send` is reading it -- is not a spoiled image: a memory-mapped access
 * to a half-configured OCTOSPI stalls the AXI indefinitely (issue #3).  There
 * are two consoles, so "the user would not do that" is not an argument.
 *
 * psram_acquire_shared() is the right guard for that: it refuses a command that
 * is RECONFIGURING OCTOSPI1, and nothing else.  It deliberately does NOT refuse
 * the LTDC scanning out of the same memory -- the display and the camera can
 * share the bus, and since phase 3c they must, or there is no live preview.
 */
static int cam_psram_take(struct cli_instance *sh)
{
#if BSP_ENABLE_PSRAM
	/* The SHARED guard: capturing and streaming only read and write the ring, so
	   they have to be kept away from a command that is RECONFIGURING OCTOSPI1 --
	   but not from the LTDC scanning out of the same memory.  Since phase 3c that
	   distinction is what lets the display show a live preview. */
	if (!psram_acquire_shared()) {
		cli_error(sh, "camera: OCTOSPI1 busy (wait for "
		              "psram/membench/devmem/wifi flash)\r\n");
		return 0;
	}
	return 1;
#else
	(void)sh;
	return 1;
#endif
}

static void cam_psram_give(void)
{
#if BSP_ENABLE_PSRAM
	psram_release();
#endif
}

static void report(struct cli_instance *sh, int rc)
{
	if (rc != CAM_OK)
		cli_error(sh, "camera: %s\r\n", cam_strerror(rc));
}

static int cmd_camera_probe(struct cli_instance *sh, int argc, char **argv)
{
	struct camera_info ci;
	int rc;

	(void)argc;
	(void)argv;
	cli_print(sh, "camera: powering up and probing (takes ~50 ms) ...\r\n");
	rc = camera_probe();
	if (camera_get_info(&ci) != CAM_OK) {
		report(sh, CAM_ERR_STATE);
		return 1;
	}
	if (rc != CAM_OK) {
		if (ci.i2c_addr != 0u)
			cli_error(sh, "camera: a device ACKs at SCCB 0x%02x but its ID "
			              "matched no known sensor -- try `camera reg 0x%02x <reg>`\r\n",
			          (unsigned)ci.i2c_addr, (unsigned)ci.i2c_addr);
		else
			cli_error(sh, "camera: nothing answered on SCCB (XCLK %lu.%02lu MHz) -- "
			              "module connected?\r\n",
			          (unsigned long)(ci.xclk_hz / 1000000u),
			          (unsigned long)(ci.xclk_hz % 1000000u / 10000u));
		return 1;
	}
	cli_print(sh, "camera: %s, chip ID 0x%04x", camera_sensor_name(ci.sensor),
	          (unsigned)ci.chip_id);
	if (ci.manuf_id != 0u)
		cli_print(sh, ", manufacturer 0x%04x", (unsigned)ci.manuf_id);
	cli_print(sh, " at SCCB 0x%02x (%u-bit registers)\r\n",
	          (unsigned)ci.i2c_addr, (unsigned)ci.reg_width);
	return 0;
}

static int cmd_camera_on(struct cli_instance *sh, int argc, char **argv)
{
	int rc;

	(void)argc;
	(void)argv;
	rc = camera_on();
	if (rc != CAM_OK) {
		report(sh, rc);
		return 1;
	}
	cli_print(sh, "camera: XCLK on, PWDN released, RESETB released\r\n");
	return 0;
}

static int cmd_camera_off(struct cli_instance *sh, int argc, char **argv)
{
	int rc;

	(void)argc;
	(void)argv;
	rc = camera_off();
	if (rc != CAM_OK) {
		report(sh, rc);
		return 1;
	}
	cli_print(sh, "camera: PWDN asserted, RESETB low, XCLK stopped "
	              "(the 2V8 rail stays on -- no enable pin)\r\n");
	return 0;
}

static int cmd_camera_info(struct cli_instance *sh, int argc, char **argv)
{
	struct camera_info ci;
	struct camera_mode m;
	int rc;

	(void)argc;
	(void)argv;
	rc = camera_get_info(&ci);
	if (rc != CAM_OK) {
		report(sh, rc);
		return 1;
	}

	cli_print(sh, "state:   %s\r\n", ci.powered ? "powered" : "powered down");
	cli_print(sh, "sensor:  %s", camera_sensor_name(ci.sensor));
	if (ci.sensor == CAM_SENSOR_OV2640 || ci.sensor == CAM_SENSOR_OV5640)
		cli_print(sh, " (chip ID 0x%04x, SCCB 0x%02x, %u-bit regs)",
		          (unsigned)ci.chip_id, (unsigned)ci.i2c_addr,
		          (unsigned)ci.reg_width);
	else if (ci.i2c_addr != 0u)
		cli_print(sh, " (unidentified device at SCCB 0x%02x)",
		          (unsigned)ci.i2c_addr);
	cli_print(sh, "\r\n");
	cli_print(sh, "xclk:    ");
	if (ci.xclk_hz != 0u)
		cli_print(sh, "%lu.%02lu MHz = %lu MHz / %lu (PA2, TIM5_CH3)\r\n",
		          (unsigned long)(ci.xclk_hz / 1000000u),
		          (unsigned long)(ci.xclk_hz % 1000000u / 10000u),
		          (unsigned long)(ci.xclk_src_hz / 1000000u),
		          (unsigned long)(ci.xclk_src_hz / ci.xclk_hz));
	else
		cli_print(sh, "stopped (PA2 driven low)\r\n");
	/* Everything below the kernel clock is a compile-time constant now (phase
	   3c-2 folded away the `tune` knobs), so it is spelled out rather than read
	   back.  The kernel clock is not: it is what the SCCB TIMINGR constants were
	   computed against, and 0.0 MHz here means the mux moved off rcc_pclk4. */
	cli_print(sh, "sccb:    I2C4 PF14/PF15, 100 kHz nominal, kernel %lu.%lu MHz, "
	              "split reads, board pull-ups\r\n",
	          (unsigned long)(ci.i2c_ker_hz / 1000000u),
	          (unsigned long)(ci.i2c_ker_hz % 1000000u / 100000u));
	cli_print(sh, "lines:   PWDN PE7 active high, RESETB PH12 active low\r\n");
	cli_print(sh, "dcmi:    8-bit, PCK rising, VS/HS low, DMA2_Stream1 -> PSRAM\r\n");
#if BSP_ENABLE_LCD
	{
		uint32_t shown, dropped;

		cam_preview_stats(&shown, &dropped);
		cli_print(sh, "preview: %s (centre 240x240 on the panel), "
		              "shown %lu dropped %lu\r\n",
		          cam_preview_enabled() ? "on" : "off",
		          (unsigned long)shown, (unsigned long)dropped);
	}
#endif
	cli_print(sh, "sensor cfg: %s, DVP byte swap on\r\n",
	          ci.configured ? "QVGA RGB565 loaded" : "not loaded (lazy)");
	if (camera_get_mode(&m) == CAM_OK)
		cli_print(sh, "frame:   %ux%u %s (%lu bytes) -- %s\r\n",
		          (unsigned)m.width, (unsigned)m.height, m.format,
		          (unsigned long)m.frame_bytes,
		          ci.frame_valid ? "captured" : "none yet");
	return 0;
}

static int cmd_camera_scan(struct cli_instance *sh, int argc, char **argv)
{
	uint8_t addrs[CAM_SCAN_MAX];
	unsigned found = 0u;
	int rc;

	(void)argc;
	(void)argv;
	rc = camera_scan(addrs, CAM_SCAN_MAX, &found);
	if (rc != CAM_OK) {
		if (rc == CAM_ERR_STATE)
			cli_error(sh, "camera: not powered -- run `camera on` first "
			              "(SCCB needs XCLK)\r\n");
		else
			report(sh, rc);
		return 1;
	}
	if (found == 0u) {
		cli_print(sh, "camera: no device ACKed on SCCB (0x08..0x77)\r\n");
		return 0;
	}
	cli_print(sh, "camera: %u device%s:", found, (found == 1u) ? "" : "s");
	for (unsigned i = 0u; i < found && i < CAM_SCAN_MAX; i++)
		cli_print(sh, " 0x%02x", (unsigned)addrs[i]);
	cli_print(sh, "\r\n");
	return 0;
}

/*
 * camera reg <i2c-addr> <reg> [value] [-16]
 *
 * The address and the width are explicit rather than taken from the last probe:
 * this command's whole reason to exist is poking at a sensor probe could NOT
 * identify, where neither is known.
 */
static int cmd_camera_reg(struct cli_instance *sh, int argc, char **argv)
{
	unsigned width = CAM_REG_WIDTH_8;
	uint32_t addr, reg, val = 0u;
	int have_val = 0;
	uint8_t v;
	int rc;

	if (cli_parse_u32(argv[1], &addr) != 0 || addr > 0x7Fu) {
		cli_error(sh, "camera: bad 7-bit SCCB address '%s'\r\n", argv[1]);
		return 1;
	}
	if (cli_parse_u32(argv[2], &reg) != 0 || reg > 0xFFFFu) {
		cli_error(sh, "camera: bad register '%s'\r\n", argv[2]);
		return 1;
	}
	for (int i = 3; i < argc; i++) {
		if (strcmp(argv[i], "-16") == 0) {
			width = CAM_REG_WIDTH_16;
		} else if (!have_val && cli_parse_u32(argv[i], &val) == 0 &&
		           val <= 0xFFu) {
			have_val = 1;
		} else {
			cli_error(sh, "camera: bad argument '%s' (value 0..255 | -16)\r\n",
			          argv[i]);
			return 1;
		}
	}
	if (width == CAM_REG_WIDTH_8 && reg > 0xFFu) {
		cli_error(sh, "camera: register 0x%lx needs -16\r\n",
		          (unsigned long)reg);
		return 1;
	}

	/* svc/fmt.c implements no '*' field width, so the two register widths need
	   two literal formats rather than one parameterised one. */
	if (have_val) {
		rc = camera_reg_write((uint8_t)addr, (uint16_t)reg, width, (uint8_t)val);
		if (rc != CAM_OK) {
			report(sh, rc);
			return 1;
		}
		if (width == CAM_REG_WIDTH_16)
			cli_print(sh, "0x%02x[0x%04x] <- 0x%02x\r\n", (unsigned)addr,
			          (unsigned)reg, (unsigned)val);
		else
			cli_print(sh, "0x%02x[0x%02x] <- 0x%02x\r\n", (unsigned)addr,
			          (unsigned)reg, (unsigned)val);
		return 0;
	}

	rc = camera_reg_read((uint8_t)addr, (uint16_t)reg, width, &v);
	if (rc != CAM_OK) {
		report(sh, rc);
		return 1;
	}
	if (width == CAM_REG_WIDTH_16)
		cli_print(sh, "0x%02x[0x%04x] = 0x%02x\r\n", (unsigned)addr,
		          (unsigned)reg, (unsigned)v);
	else
		cli_print(sh, "0x%02x[0x%02x] = 0x%02x\r\n", (unsigned)addr,
		          (unsigned)reg, (unsigned)v);
	return 0;
}

/* ---- capture ------------------------------------------------------------- */

/*
 * Per-channel min/max/mean over the whole frame.
 *
 * This is the instrument phase 2 exists to read.  A frame that never arrived is
 * all zeros; a saturated one pins every channel at max; a DCMI that is latching
 * on the wrong edge or losing sync gives min 0 / max full on all three with a
 * mid-grey mean.  Covering the lens and pointing at a light must move the means
 * in opposite directions -- that, and nothing cheaper, is what proves the pixels
 * are real.
 */
struct rgb_stats {
	uint32_t min[3], max[3], sum[3];
	uint32_t npx;
};

static void stats_init(struct rgb_stats *s)
{
	for (int i = 0; i < 3; i++) {
		s->min[i] = 0xFFFFFFFFu;
		s->max[i] = 0u;
		s->sum[i] = 0u;
	}
	s->npx = 0u;
}

static void stats_add_rgb565(struct rgb_stats *s, const uint8_t *buf, uint32_t n)
{
	for (uint32_t i = 0; i + 1u < n; i += 2u) {
		uint32_t p = (uint32_t)buf[i] | ((uint32_t)buf[i + 1u] << 8);
		uint32_t ch[3] = { (p >> 11) & 0x1Fu, (p >> 5) & 0x3Fu, p & 0x1Fu };

		for (int k = 0; k < 3; k++) {
			if (ch[k] < s->min[k]) s->min[k] = ch[k];
			if (ch[k] > s->max[k]) s->max[k] = ch[k];
			s->sum[k] += ch[k];
		}
		s->npx++;
	}
}

/* Per-row channel means, x100 so the seam metric keeps two decimals in integers. */
static void row_means_x100(const uint8_t *buf, uint32_t n, uint32_t out[3])
{
	uint32_t sum[3] = { 0u, 0u, 0u };
	uint32_t npx = 0u;

	for (uint32_t i = 0; i + 1u < n; i += 2u) {
		uint32_t p = (uint32_t)buf[i] | ((uint32_t)buf[i + 1u] << 8);

		sum[0] += (p >> 11) & 0x1Fu;
		sum[1] += (p >> 5) & 0x3Fu;
		sum[2] += p & 0x1Fu;
		npx++;
	}
	for (int k = 0; k < 3; k++)
		out[k] = npx ? (sum[k] * 100u) / npx : 0u;
}

/* Largest adjacent-row step, and where.  Ranked on the summed channel delta so
   one row is reported rather than three unrelated ones. */
struct seam_max {
	uint32_t row;
	uint32_t d[3];
	uint32_t total;
};

static void seam_track(struct seam_max *s, uint32_t row,
                       const uint32_t a[3], const uint32_t b[3])
{
	uint32_t d[3], total = 0u;

	for (int k = 0; k < 3; k++) {
		d[k] = (a[k] > b[k]) ? a[k] - b[k] : b[k] - a[k];
		total += d[k];
	}
	if (total > s->total) {
		s->total = total;
		s->row   = row;
		for (int k = 0; k < 3; k++)
			s->d[k] = d[k];
	}
}

static int cmd_camera_capture(struct cli_instance *sh, int argc, char **argv)
{
	static const char *const chan[3] = { "R(5)", "G(6)", "B(5)" };
	struct seam_max seam = { 0u, { 0u, 0u, 0u }, 0u };
	struct rgb_stats st;
	struct camera_mode m;
	uint8_t buf[CAM_IO_CHUNK];
	uint32_t prev[3] = { 0u, 0u, 0u };
	uint32_t gen0 = 0, gen, off, row;
	int colorbar = 0, have_psram = 0, ret = 1, rc;

	if (argc > 1) {
		if (strcmp(argv[1], "test") != 0) {
			cli_error(sh, "camera: unknown option '%s' (try: test)\r\n", argv[1]);
			return 1;
		}
		colorbar = 1;
	}
	(void)camera_get_mode(&m);

	if (!cam_psram_take(sh))
		return 1;
	have_psram = 1;

	/* Every capture pays for the warm-up frames (~1 s at 15).  The FIRST one
	   after a power cycle also pays for the sensor bring-up: 200 ms COM7 reset +
	   220 ms register table + 100 ms settle, plus 100 ms more if the colorbar
	   state has to change. */
	cli_print(sh, "camera: capturing %s frame (~1 s of warm-up; +0.6 s on the "
	              "first one after a power cycle) ...\r\n",
	          colorbar ? "colorbar test" : "live");
	rc = camera_capture(colorbar);
	if (rc != CAM_OK) {
		report(sh, rc);
		goto out;
	}

	stats_init(&st);
	for (off = 0, row = 0; off < m.frame_bytes; row++) {
		uint32_t n = (m.frame_bytes - off < CAM_IO_CHUNK)
		             ? m.frame_bytes - off : CAM_IO_CHUNK;
		uint32_t rmean[3];

		rc = camera_frame_read(off, buf, n, &gen);
		if (rc != CAM_OK) {
			report(sh, rc);
			goto out;
		}
		if (off == 0u)
			gen0 = gen;
		else if (gen != gen0) {
			cli_error(sh, "camera: frame changed mid-read "
			              "(concurrent capture)\r\n");
			goto out;
		}
		stats_add_rgb565(&st, buf, n);
		row_means_x100(buf, n, rmean);
		if (row != 0u)
			seam_track(&seam, row, prev, rmean);
		prev[0] = rmean[0]; prev[1] = rmean[1]; prev[2] = rmean[2];
		off += n;
	}

	cli_print(sh, "frame: %ux%u %s (%lu bytes, %u warm-up frames)\r\n",
	          (unsigned)m.width, (unsigned)m.height, m.format,
	          (unsigned long)m.frame_bytes, (unsigned)CAMERA_WARM_FRAMES);
	for (int k = 0; k < 3; k++)
		cli_print(sh, "%s: min %3lu  max %3lu  mean %3lu\r\n", chan[k],
		          (unsigned long)st.min[k], (unsigned long)st.max[k],
		          (unsigned long)(st.npx ? st.sum[k] / st.npx : 0u));
	/* The banding metric: the largest jump between two adjacent rows' channel
	   means (x100).  A settled frame is single digits; a gain change landing
	   mid-readout shows up as hundreds, at the row where it landed. */
	cli_print(sh, "seam: row %3lu  dR %lu.%02lu  dG %lu.%02lu  dB %lu.%02lu\r\n",
	          (unsigned long)seam.row,
	          (unsigned long)(seam.d[0] / 100u), (unsigned long)(seam.d[0] % 100u),
	          (unsigned long)(seam.d[1] / 100u), (unsigned long)(seam.d[1] % 100u),
	          (unsigned long)(seam.d[2] / 100u), (unsigned long)(seam.d[2] % 100u));
	ret = 0;
out:
	if (have_psram)
		cam_psram_give();
	return ret;
}

/* ---- save / send --------------------------------------------------------- */

#if BSP_ENABLE_SD
static int cmd_camera_save(struct cli_instance *sh, int argc, char **argv)
{
	const struct fs_device *dev = fs_sd_device();
	struct camera_mode m;
	uint8_t buf[CAM_IO_CHUNK];
	FX_MEDIA *media;
	FX_FILE file;
	uint32_t gen0 = 0, gen, off;
	int have_psram = 0, have_op = 0, have_file = 0, ret = 1, rc;
	UINT st;

	(void)argc;
	(void)camera_get_mode(&m);

	if (!cam_psram_take(sh))
		return 1;
	have_psram = 1;

	media = fs_core_mount(dev, sh);
	if (media == NULL)
		goto out;
	st = dev->op_begin();
	if (st != FX_SUCCESS) {
		cli_error(sh, "camera: %s: %s\r\n", dev->name, fs_strerror(st));
		goto out;
	}
	have_op = 1;

	(void)fx_file_delete(media, argv[1]);   /* overwrite semantics */
	st = fx_file_create(media, argv[1]);
	if (st != FX_SUCCESS && st != FX_ALREADY_CREATED) {
		cli_error(sh, "camera: create %s: %s\r\n", argv[1], fs_strerror(st));
		goto out;
	}
	st = fx_file_open(media, &file, argv[1], FX_OPEN_FOR_WRITE);
	if (st != FX_SUCCESS) {
		cli_error(sh, "camera: open %s: %s\r\n", argv[1], fs_strerror(st));
		goto out;
	}
	have_file = 1;

	for (off = 0; off < m.frame_bytes; ) {
		uint32_t n = (m.frame_bytes - off < CAM_IO_CHUNK)
		             ? m.frame_bytes - off : CAM_IO_CHUNK;

		rc = camera_frame_read(off, buf, n, &gen);
		if (rc != CAM_OK) {
			report(sh, rc);
			goto out;
		}
		if (off == 0u)
			gen0 = gen;
		else if (gen != gen0) {
			cli_error(sh, "camera: frame changed mid-save by a concurrent "
			              "capture (file left partial)\r\n");
			goto out;
		}
		st = fx_file_write(&file, buf, n);
		if (st != FX_SUCCESS) {
			cli_error(sh, "camera: write failed: %s\r\n", fs_strerror(st));
			goto out;
		}
		off += n;
	}
	cli_print(sh, "camera: wrote %s (%lu bytes, %ux%u %s)\r\n", argv[1],
	          (unsigned long)m.frame_bytes, (unsigned)m.width,
	          (unsigned)m.height, m.format);
	ret = 0;
out:
	if (have_file) {
		(void)fx_file_close(&file);
		(void)fx_media_flush(media);
	}
	if (have_op)
		dev->op_end();
	if (have_psram)
		cam_psram_give();
	return ret;
}
#endif /* BSP_ENABLE_SD */

/* YMODEM source over the captured frame.  The generation check makes a
   concurrent capture a read fault rather than a silently spliced file. */
struct cam_ym_ctx {
	uint32_t pos;
	uint32_t gen;
	int      have_gen;
};

static int cam_ym_read(void *ctx, uint8_t *dst, uint32_t want, uint32_t *got)
{
	struct cam_ym_ctx *c = ctx;
	struct camera_mode m;
	uint32_t gen, n;

	(void)camera_get_mode(&m);
	if (c->pos >= m.frame_bytes) {
		*got = 0u;
		return 0;
	}
	n = m.frame_bytes - c->pos;
	if (n > want)
		n = want;
	if (camera_frame_read(c->pos, dst, n, &gen) != CAM_OK)
		return -1;
	if (!c->have_gen) {
		c->gen = gen;
		c->have_gen = 1;
	} else if (gen != c->gen) {
		return -1;
	}
	c->pos += n;
	*got = n;
	return 0;
}

/*
 * On the PC side use `rz -y -b`, and `ffmpeg -y` to render.  Without those flags
 * neither tool overwrites an existing file -- YMODEM block 0 carries a name and a
 * size but no mtime, so rz has nothing to compare -- and you quietly keep looking
 * at the PREVIOUS capture, which is indistinguishable from a camera that has
 * stopped producing new frames.  That is exactly what happened during bring-up.
 */
static int cmd_camera_send(struct cli_instance *sh, int argc, char **argv)
{
	struct cam_ym_ctx ctx = { 0u, 0u, 0 };
	struct camera_mode m;
	struct camera_info ci;
	struct ym_source src;
	int have_psram = 0, ret = 1;

	(void)camera_get_mode(&m);
	if (camera_get_info(&ci) != CAM_OK || !ci.frame_valid) {
		report(sh, CAM_ERR_NO_FRAME);
		return 1;
	}
	if (!cam_psram_take(sh))
		return 1;
	have_psram = 1;

	src.ctx  = &ctx;
	src.name = (argc > 1) ? argv[1] : "frame.raw";
	src.size = m.frame_bytes;
	src.read = cam_ym_read;
	cli_print(sh, "camera: receive with `rz -y -b` -- without -y an existing "
	              "file is kept and you get the previous frame\r\n");
	ret = xfer_send_source(sh, &src);

	if (have_psram)
		cam_psram_give();
	return ret;
}

/* ---- streaming ----------------------------------------------------------- */

/*
 * `camera stream start` is the one command that does NOT hold the OCTOSPI1 guard
 * for its own duration: a stream is open-ended and can stop itself on
 * --frames/--secs, so there would be nobody left to release it.  It takes the
 * shared guard only across the arm -- long enough to be refused if another
 * command is reconfiguring the bus -- and hands over to camera_streaming(),
 * which app/psram.c consults from then on.  camera_stream_start() has set that
 * flag by the time it returns, so there is no gap between the two.
 */
static int cmd_stream_start(struct cli_instance *sh, int argc, char **argv)
{
	uint32_t frames = 0u, secs = 0u, v;
	int colorbar = 0, rc;

	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "test") == 0) {
			colorbar = 1;
		} else if (strcmp(argv[i], "--frames") == 0 && i + 1 < argc) {
			if (cli_parse_u32(argv[++i], &v) != 0 || v == 0u)
				goto bad;
			frames = v;
		} else if (strcmp(argv[i], "--secs") == 0 && i + 1 < argc) {
			if (cli_parse_u32(argv[++i], &v) != 0 || v == 0u)
				goto bad;
			secs = v;
		} else {
			goto bad;
		}
	}

	if (!cam_psram_take(sh))
		return 1;
	rc = camera_stream_start(colorbar, frames, secs);
	cam_psram_give();
	if (rc != CAM_OK) {
		report(sh, rc);
		return 1;
	}
	cli_print(sh, "camera: streaming %s", colorbar ? "colorbar" : "live");
	if (frames)
		cli_print(sh, ", stopping after %lu frames", (unsigned long)frames);
	if (secs)
		cli_print(sh, ", stopping after %lu s", (unsigned long)secs);
	cli_print(sh, " -- `camera stream stats` / `stop`\r\n");
	return 0;
bad:
	cli_error(sh, "camera: usage: stream start [test] [--frames N] [--secs S]\r\n");
	return 1;
}

static int cmd_stream_stop(struct cli_instance *sh, int argc, char **argv)
{
	int rc;

	(void)argc;
	(void)argv;
	rc = camera_stream_stop();
	if (rc != CAM_OK) {
		report(sh, rc);
		return 1;
	}
	cli_print(sh, "camera: stream stopped\r\n");
	return 0;
}

static int cmd_stream_stats(struct cli_instance *sh, int argc, char **argv)
{
	struct camera_stream_stats st;
	int rc;

	(void)argc;
	(void)argv;
	rc = camera_stream_stats(&st);
	if (rc != CAM_OK) {
		report(sh, rc);
		return 1;
	}
	cli_print(sh, "state:     %s\r\n", st.active ? "streaming" : "stopped");
	cli_print(sh, "frames:    %lu\r\n", (unsigned long)st.frames);
	cli_print(sh, "elapsed:   %lu ms\r\n", (unsigned long)st.elapsed_ms);
	if (st.elapsed_ms != 0u) {
		uint32_t f10 = (uint32_t)((uint64_t)st.frames * 10000u / st.elapsed_ms);

		cli_print(sh, "fps:       %lu.%lu\r\n", (unsigned long)(f10 / 10u),
		          (unsigned long)(f10 % 10u));
	}
	cli_print(sh, "ovr dcmi:  %lu\r\n", (unsigned long)st.dcmi_ovr);
	cli_print(sh, "ovr ring:  %lu\r\n", (unsigned long)st.ring_ovr);
	/* The figure of merit for the OCTOSPI1 contention work: DMA FIFO errors are
	   tolerated (they do not stop the stream) but their RATE says how close the
	   bus is to not keeping up -- which is what decides whether the LTDC can
	   scan out at the same time (phase 3c). */
	cli_print(sh, "dma fe:    %lu\r\n", (unsigned long)st.dma_fe);
	if (st.elapsed_ms != 0u) {
		uint32_t e10 = (uint32_t)((uint64_t)st.dma_fe * 10000u / st.elapsed_ms);

		cli_print(sh, "dma fe/s:  %lu.%lu\r\n", (unsigned long)(e10 / 10u),
		          (unsigned long)(e10 % 10u));
	}
	cli_print(sh, "repoint skip: %lu\r\n", (unsigned long)st.repoint_skip);
	cli_print(sh, "ring:      %lu slots x %lu B\r\n", (unsigned long)st.slots,
	          (unsigned long)CAMERA_FRAME_BYTES);
	cli_print(sh, "delivered: %lu  dropped: %lu\r\n",
	          (unsigned long)st.delivered, (unsigned long)st.dropped);
	return 0;
}

static int cmd_camera_stream(struct cli_instance *sh, int argc, char **argv)
{
	if (argc > 1)
		cli_error(sh, "camera: unknown stream subcommand '%s'\r\n", argv[1]);
	cli_print(sh, "usage: camera stream <start [test] [--frames N] [--secs S]"
	              " | stop | stats>\r\n");
	return argc > 1 ? 1 : 0;
}

CLI_SUBCMD_SET_CREATE(camera_stream_subcmds,
	CLI_CMD_ARG(start, NULL, "begin continuous capture (test, --frames N, --secs S)",
	            cmd_stream_start, 1, 5),
	CLI_CMD(stop,  NULL, "stop the running stream", cmd_stream_stop),
	CLI_CMD(stats, NULL, "frame / fps / overrun counters", cmd_stream_stats),
	CLI_SUBCMD_SET_END);

/* ---- preview ------------------------------------------------------------- */

#if BSP_ENABLE_LCD
static int cmd_camera_preview(struct cli_instance *sh, int argc, char **argv)
{
	int on;

	(void)argc;
	if (strcmp(argv[1], "on") == 0)
		on = 1;
	else if (strcmp(argv[1], "off") == 0)
		on = 0;
	else {
		cli_error(sh, "camera: preview takes on|off\r\n");
		return 1;
	}
	if (cam_preview_enable(on) != 0) {
		cli_error(sh, "camera: the display is down or its scanout is off "
		              "(run 'lcd on')\r\n");
		return 1;
	}
	cli_print(sh, "camera: preview %s%s\r\n", on ? "on" : "off",
	          (on && !camera_streaming())
	                  ? " -- nothing to show until `camera stream start`" : "");
	return 0;
}
#endif /* BSP_ENABLE_LCD */

CLI_SUBCMD_SET_CREATE(camera_subcmds,
	CLI_CMD(probe, NULL, "power-cycle the module and read the sensor chip ID",
	        cmd_camera_probe),
	CLI_CMD(on,   NULL, "XCLK on, release PWDN/RESETB (no ID read)", cmd_camera_on),
	CLI_CMD(off,  NULL, "PWDN + RESETB asserted, XCLK off (rail stays on)",
	        cmd_camera_off),
	CLI_CMD(info, NULL, "driver / sensor / XCLK / SCCB state", cmd_camera_info),
	CLI_CMD(scan, NULL, "list SCCB addresses that ACK (0x08..0x77)",
	        cmd_camera_scan),
	CLI_CMD_ARG(capture, NULL, "snapshot one frame + channel stats ('test' = colorbar)",
	            cmd_camera_capture, 1, 1),
#if BSP_ENABLE_SD
	CLI_CMD_ARG(save, NULL, "write the frame raw to the microSD <path>",
	            cmd_camera_save, 2, 0),
#endif
	CLI_CMD_ARG(send, NULL, "stream the frame to the PC over YMODEM [name]",
	            cmd_camera_send, 1, 1),
	CLI_CMD_ARG(stream, camera_stream_subcmds,
	            "continuous capture (start/stop/stats)", cmd_camera_stream, 1, 1),
#if BSP_ENABLE_LCD
	CLI_CMD_ARG(preview, NULL, "show the running stream on the LCD <on|off>",
	            cmd_camera_preview, 2, 0),
#endif
	CLI_CMD_ARG(reg, NULL, "raw SCCB access <addr> <reg> [value] [-16]",
	            cmd_camera_reg, 3, 2),
	CLI_SUBCMD_SET_END);

CLI_CMD_REGISTER(camera, camera_subcmds,
                 "FPC-24 DVP camera (OV2640 over DCMI, QVGA RGB565 snapshot)",
                 NULL, 1, 0);
