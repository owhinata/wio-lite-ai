/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    cmd_camera.c
 * @brief   `camera` shell command: DVP camera bring-up (issue #8 phase 1).
 *
 *   camera probe          power-cycle the module and identify the sensor
 *   camera on | off       run / undo the power-up sequence (no ID read)
 *   camera info           driver, sensor, XCLK and SCCB state
 *   camera scan           list every 7-bit SCCB address that ACKs
 *   camera reg <a> <r> .. raw SCCB register read / write
 *   camera xclk [hz]      show or retune the master clock
 *   camera tune ...       polarity / pull-up / bit-rate / read-style overrides
 *
 * `camera probe` is the one command this phase exists for: if it prints a chip
 * ID, then XCLK is at a frequency the sensor accepts, PWDN and RESETB have the
 * polarity assumed, and I2C4 is talking at the right bit rate -- all four proven
 * at once.  Until then, `scan` narrows it down to "does anything answer at all".
 *
 * `xclk` and `tune` exist because none of those four can be proven from the
 * schematic alone, and the internal flash is only rated for ~10k erase cycles:
 * sweeping an unknown by reflashing is what these knobs replace (the lesson
 * carried over from issue #7's LCD bring-up).  They are bring-up instruments,
 * not a permanent part of the command surface -- expect them to shrink once the
 * sensor is known.
 *
 * Note `camera off` does NOT remove power: the camera's 2V8 rail comes off a
 * fixed LDO with no enable pin (schematic U8), so the deepest state reachable
 * is PWDN asserted + RESETB low + XCLK stopped.
 *
 * Clean-room design; no third-party code reused.
 */
#include "camera.h"
#include "cli.h"

#include <stdint.h>
#include <string.h>

#define CAM_SCAN_MAX 16u

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
			              "module connected? try `camera tune` / `camera xclk`\r\n",
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
	struct camera_tuning t;
	struct camera_info ci;
	int rc;

	(void)argc;
	(void)argv;
	rc = camera_get_info(&ci);
	if (rc != CAM_OK) {
		report(sh, rc);
		return 1;
	}
	(void)camera_get_tuning(&t);

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
	cli_print(sh, "sccb:    I2C4 PF14/PF15, %lu kHz nominal, kernel %lu.%lu MHz, "
	              "%s reads\r\n",
	          (unsigned long)(t.i2c_hz / 1000u),
	          (unsigned long)(ci.i2c_ker_hz / 1000000u),
	          (unsigned long)(ci.i2c_ker_hz % 1000000u / 100000u),
	          (t.sccb_style == CAM_SCCB_RESTART) ? "repeated-START" : "split");
	cli_print(sh, "lines:   PWDN PE7 active %s, RESETB PH12 active %s, "
	              "pull-ups %s\r\n",
	          t.pwdn_active_high ? "high" : "low",
	          t.rst_active_low ? "low" : "high",
	          t.i2c_pullup ? "board + internal" : "board only");
	cli_print(sh, "dcmi:    not configured (issue #8 phase 2)\r\n");
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

static int cmd_camera_xclk(struct cli_instance *sh, int argc, char **argv)
{
	struct camera_info ci;
	uint32_t hz, actual;
	int rc;

	if (argc < 2) {
		if (camera_get_info(&ci) != CAM_OK) {
			report(sh, CAM_ERR_STATE);
			return 1;
		}
		cli_print(sh, "xclk: %lu Hz (source %lu Hz)\r\n",
		          (unsigned long)ci.xclk_hz, (unsigned long)ci.xclk_src_hz);
		return 0;
	}
	if (cli_parse_u32(argv[1], &hz) != 0 || hz == 0u) {
		cli_error(sh, "camera: bad frequency '%s' (Hz)\r\n", argv[1]);
		return 1;
	}
	rc = camera_set_xclk(hz, &actual);
	if (rc != CAM_OK) {
		report(sh, rc);
		return 1;
	}
	cli_print(sh, "xclk: %lu Hz requested, %lu Hz emitted%s\r\n",
	          (unsigned long)hz, (unsigned long)actual,
	          (camera_get_info(&ci) == CAM_OK && ci.xclk_hz == 0u)
	                  ? " on the next `camera on`" : "");
	return 0;
}

/* ---- tune ---------------------------------------------------------------- */

static int tune_apply(struct cli_instance *sh, const struct camera_tuning *t,
                      const char *name, const char *val)
{
	int rc = camera_set_tuning(t);

	if (rc != CAM_OK) {
		cli_error(sh, "camera: %s for %s\r\n", cam_strerror(rc), name);
		return 1;
	}
	cli_print(sh, "camera: %s = %s\r\n", name, val);
	return 0;
}

static int cmd_tune_pwdn(struct cli_instance *sh, int argc, char **argv)
{
	struct camera_tuning t;

	(void)argc;
	if (camera_get_tuning(&t) != CAM_OK)
		return 1;
	if (strcmp(argv[1], "high") == 0)
		t.pwdn_active_high = 1u;
	else if (strcmp(argv[1], "low") == 0)
		t.pwdn_active_high = 0u;
	else {
		cli_error(sh, "camera: pwdn takes high|low\r\n");
		return 1;
	}
	return tune_apply(sh, &t, "pwdn", argv[1]);
}

static int cmd_tune_rst(struct cli_instance *sh, int argc, char **argv)
{
	struct camera_tuning t;

	(void)argc;
	if (camera_get_tuning(&t) != CAM_OK)
		return 1;
	if (strcmp(argv[1], "low") == 0)
		t.rst_active_low = 1u;
	else if (strcmp(argv[1], "high") == 0)
		t.rst_active_low = 0u;
	else {
		cli_error(sh, "camera: rst takes low|high\r\n");
		return 1;
	}
	return tune_apply(sh, &t, "rst", argv[1]);
}

static int cmd_tune_pull(struct cli_instance *sh, int argc, char **argv)
{
	struct camera_tuning t;

	(void)argc;
	if (camera_get_tuning(&t) != CAM_OK)
		return 1;
	if (strcmp(argv[1], "on") == 0)
		t.i2c_pullup = 1u;
	else if (strcmp(argv[1], "off") == 0)
		t.i2c_pullup = 0u;
	else {
		cli_error(sh, "camera: pull takes on|off\r\n");
		return 1;
	}
	return tune_apply(sh, &t, "pull", argv[1]);
}

static int cmd_tune_i2c(struct cli_instance *sh, int argc, char **argv)
{
	struct camera_tuning t;
	uint32_t khz;

	(void)argc;
	if (camera_get_tuning(&t) != CAM_OK)
		return 1;
	if (cli_parse_u32(argv[1], &khz) != 0 ||
	    (khz != 50u && khz != 100u && khz != 400u)) {
		cli_error(sh, "camera: i2c takes 50|100|400 (kHz)\r\n");
		return 1;
	}
	t.i2c_hz = khz * 1000u;
	return tune_apply(sh, &t, "i2c", argv[1]);
}

static int cmd_tune_sccb(struct cli_instance *sh, int argc, char **argv)
{
	struct camera_tuning t;

	(void)argc;
	if (camera_get_tuning(&t) != CAM_OK)
		return 1;
	if (strcmp(argv[1], "split") == 0)
		t.sccb_style = CAM_SCCB_SPLIT;
	else if (strcmp(argv[1], "restart") == 0)
		t.sccb_style = CAM_SCCB_RESTART;
	else {
		cli_error(sh, "camera: sccb takes split|restart\r\n");
		return 1;
	}
	return tune_apply(sh, &t, "sccb", argv[1]);
}

static int cmd_camera_tune(struct cli_instance *sh, int argc, char **argv)
{
	struct camera_tuning t;

	if (argc > 1) {
		cli_error(sh, "camera: unknown tune setting '%s'\r\n", argv[1]);
		return 1;
	}
	if (camera_get_tuning(&t) != CAM_OK) {
		report(sh, CAM_ERR_STATE);
		return 1;
	}
	cli_print(sh, "pwdn: %s\r\n", t.pwdn_active_high ? "high" : "low");
	cli_print(sh, "rst:  %s\r\n", t.rst_active_low ? "low" : "high");
	cli_print(sh, "pull: %s\r\n", t.i2c_pullup ? "on" : "off");
	cli_print(sh, "i2c:  %lu\r\n", (unsigned long)(t.i2c_hz / 1000u));
	cli_print(sh, "sccb: %s\r\n",
	          (t.sccb_style == CAM_SCCB_RESTART) ? "restart" : "split");
	return 0;
}

CLI_SUBCMD_SET_CREATE(camera_tune_subcmds,
	CLI_CMD_ARG(pwdn, NULL, "PWDN assert level <high|low>", cmd_tune_pwdn, 2, 0),
	CLI_CMD_ARG(rst,  NULL, "RESETB assert level <low|high>", cmd_tune_rst, 2, 0),
	CLI_CMD_ARG(pull, NULL, "internal SCCB pull-ups <on|off>", cmd_tune_pull, 2, 0),
	CLI_CMD_ARG(i2c,  NULL, "SCCB bit rate <50|100|400> kHz", cmd_tune_i2c, 2, 0),
	CLI_CMD_ARG(sccb, NULL, "read style <split|restart>", cmd_tune_sccb, 2, 0),
	CLI_SUBCMD_SET_END);

CLI_SUBCMD_SET_CREATE(camera_subcmds,
	CLI_CMD(probe, NULL, "power-cycle the module and read the sensor chip ID",
	        cmd_camera_probe),
	CLI_CMD(on,   NULL, "XCLK on, release PWDN/RESETB (no ID read)", cmd_camera_on),
	CLI_CMD(off,  NULL, "PWDN + RESETB asserted, XCLK off (rail stays on)",
	        cmd_camera_off),
	CLI_CMD(info, NULL, "driver / sensor / XCLK / SCCB state", cmd_camera_info),
	CLI_CMD(scan, NULL, "list SCCB addresses that ACK (0x08..0x77)",
	        cmd_camera_scan),
	CLI_CMD_ARG(reg, NULL, "raw SCCB access <addr> <reg> [value] [-16]",
	            cmd_camera_reg, 3, 2),
	CLI_CMD_ARG(xclk, NULL, "show or set the master clock [hz]",
	            cmd_camera_xclk, 1, 1),
	CLI_CMD_ARG(tune, camera_tune_subcmds,
	            "bring-up overrides (no arg = show current)", cmd_camera_tune, 1, 1),
	CLI_SUBCMD_SET_END);

CLI_CMD_REGISTER(camera, camera_subcmds,
                 "FPC-24 DVP camera (DCMI, issue #8 phase 1: XCLK + SCCB)",
                 NULL, 1, 0);
