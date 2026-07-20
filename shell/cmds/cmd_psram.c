/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    cmd_psram.c
 * @brief   `psram` shell command (issue #3): OCTOSPI1 APS6408 PSRAM status + test.
 *
 *   psram info          show bring-up state, device clock, ID, mmap window
 *   psram test [bytes]  write/read/verify patterns over the mmap window (default
 *                       the whole 8 MB); reports the first mismatch address
 *
 * This is the primary functional check for the OCTOSPI1 octal wiring + latency:
 * a wrong data-line AF, latency code, or delay-block setting shows up as a
 * mismatch (or, if the mmap latency is badly off, an AXI/bus fault that the
 * fault handler records to dmesg).  Registered only in the `shell` executable
 * (like cmd_membench.c); PSRAM is scratch RAM so no dangerous-command gate.
 *
 * Clean-room design; no third-party code reused.
 */
#include "cli.h"
#include "psram.h"

#include <stdint.h>
#include <string.h>

/* Parse a 32-bit unsigned: 0x-hex or decimal.  Returns 0 on success. */
static int parse_u32(const char *s, uint32_t *out)
{
	uint32_t base = 10, val = 0;
	const char *p = s;

	if (p == NULL || *p == '\0')
		return -1;
	if (p[0] == '0' && (p[1] == 'x' || p[1] == 'X')) { base = 16; p += 2; }
	if (*p == '\0')
		return -1;
	for (; *p != '\0'; p++) {
		uint32_t d;
		char c = *p;
		if (c >= '0' && c <= '9')            d = (uint32_t)(c - '0');
		else if (base == 16 && c >= 'a' && c <= 'f') d = (uint32_t)(c - 'a' + 10);
		else if (base == 16 && c >= 'A' && c <= 'F') d = (uint32_t)(c - 'A' + 10);
		else return -1;
		if (val > (0xFFFFFFFFu - d) / base)
			return -1;
		val = val * base + d;
	}
	*out = val;
	return 0;
}

/* Take the OCTOSPI1 concurrency guard for a hardware subcommand; on contention
 * (a backgrounded `psram`/`membench` command still holds it) print and bail so
 * two commands never drive OCTOSPI1 at once.  Pair every success with
 * psram_release().  Returns 0 when acquired, -1 when busy. */
static int psram_guard(struct cli_instance *sh)
{
	if (!psram_acquire()) {
		cli_error(sh, "psram: busy (another psram/membench command holds "
		          "OCTOSPI1)\r\n");
		return -1;
	}
	return 0;
}

/* Bring-up stage names (PSRAM_STAGE_*). */
static const char *psram_stage_name(uint32_t s)
{
	switch (s) {
	case PSRAM_STAGE_GRST: return "global-reset (TCF timeout)";
	case PSRAM_STAGE_MR0:  return "MA0 pair read";
	case PSRAM_STAGE_MR2:  return "MA2 pair read";
	default:               return "OK";
	}
}

/* Decode one OCTOSPI SR sample: flags + FIFO level. */
static void psram_print_sr(struct cli_instance *sh, const char *tag, uint32_t sr)
{
	cli_print(sh, "  %-7s: 0x%08lX%s%s%s%s%s%s flevel=%lu\r\n",
	          tag, (unsigned long)sr,
	          (sr & 0x01u) ? " TEF"  : "",
	          (sr & 0x02u) ? " TCF"  : "",
	          (sr & 0x04u) ? " FTF"  : "",
	          (sr & 0x08u) ? " SMF"  : "",
	          (sr & 0x10u) ? " TOF"  : "",
	          (sr & 0x20u) ? " BUSY" : "",
	          (unsigned long)((sr >> 8) & 0x3Fu));
}

/* Print one transaction snapshot (init failure or probe result). */
static void psram_print_diag(struct cli_instance *sh, const struct psram_diag *d)
{
	cli_print(sh, "  stage  : %s -> %s, %u data byte(s)",
	          psram_stage_name(d->stage), d->ok ? "completed" : "FAILED",
	          d->ndata);
	if (d->ndata >= 1u)
		cli_print(sh, "  D0=0x%02X", d->data[0]);
	if (d->ndata >= 2u)
		cli_print(sh, " D1=0x%02X", d->data[1]);
	cli_print(sh, "\r\n");
	psram_print_sr(sh, "sr pre", d->sr_pre);
	psram_print_sr(sh, "sr data", d->sr_data);
	psram_print_sr(sh, "sr end", d->sr_end);
	cli_print(sh, "  cfg    : CR=0x%08lX CCR=0x%08lX TCR=0x%08lX\r\n",
	          (unsigned long)d->cr, (unsigned long)d->ccr, (unsigned long)d->tcr);
	cli_print(sh, "         : IR=0x%02lX AR=0x%08lX DLR=0x%lX\r\n",
	          (unsigned long)d->ir, (unsigned long)d->ar, (unsigned long)d->dlr);
	cli_print(sh, "  dcr    : 1=0x%08lX 2=0x%08lX 3=0x%08lX 4=0x%08lX\r\n",
	          (unsigned long)d->dcr1, (unsigned long)d->dcr2,
	          (unsigned long)d->dcr3, (unsigned long)d->dcr4);
	cli_print(sh, "  dlyb   : CR=0x%08lX CFGR=0x%08lX\r\n",
	          (unsigned long)d->dlyb_cr, (unsigned long)d->dlyb_cfgr);
}

static int cmd_psram_info(struct cli_instance *sh, int argc, char **argv)
{
	uint8_t id[2];
	uint32_t rd, wr, sel, unit;

	(void)argc; (void)argv;
	psram_read_id(id);
	psram_get_latency(&rd, &wr);
	psram_get_phase(&sel, &unit);

	cli_print(sh, "PSRAM (OCTOSPI1 / APS6408):\r\n");
	cli_print(sh, "  state   : %s\r\n", psram_ready() ? "ready (mmap)" : "NOT ready");
	if (!psram_ready())
		cli_print(sh, "  init    : failed at %s (see `psram snap`)\r\n",
		          psram_stage_name(psram_init_stage()));
	cli_print(sh, "  window  : 0x%08lX .. 0x%08lX (%lu MB)\r\n",
	          (unsigned long)PSRAM_BASE_ADDR,
	          (unsigned long)(PSRAM_BASE_ADDR + PSRAM_SIZE_BYTES),
	          (unsigned long)(PSRAM_SIZE_BYTES >> 20));
	cli_print(sh, "  clock   : %lu Hz (device)\r\n", (unsigned long)psram_clock_hz());
	cli_print(sh, "  id      : MR1=0x%02X MR2=0x%02X\r\n", id[0], id[1]);
	cli_print(sh, "  latency : read %lu / write %lu dummy cycles\r\n",
	          (unsigned long)rd, (unsigned long)wr);
	cli_print(sh, "  dlyb    : phase %lu / units %lu\r\n",
	          (unsigned long)sel, (unsigned long)unit);
	cli_print(sh, "  mr0     : 0x%02lX   read-flags 0x%lX (b0=SSHIFT b1=DHQC)\r\n",
	          (unsigned long)psram_get_mr0(), (unsigned long)psram_get_read_flags());
	cli_print(sh, "  ifmt    : %s instruction\r\n",
	          psram_get_instruction_dtr() ? "16-bit DTR doubled" : "8-bit SDR official");
	cli_print(sh, "  probe   : dqse %lu / refresh %lu (DCR4)\r\n",
	          (unsigned long)psram_get_dqse(),
	          (unsigned long)psram_get_refresh());
	return 0;
}

/* psram probe [ma]: one legal 2-byte MA-pair read with the current knobs,
 * ALLOWED while NOT ready, with the full SR/config snapshot.  Distinguishes
 * "no data ever arrives" (FTF timeout: sr data lacks FTF, flevel=0 -- the
 * device/DQS path returned nothing) from a completed-but-wrong read. */
static int cmd_psram_probe(struct cli_instance *sh, int argc, char **argv)
{
	uint32_t ma = 0u;
	struct psram_diag d;

	int rc;

	if (argc >= 2 && parse_u32(argv[1], &ma) != 0) {
		cli_error(sh, "psram: bad MA '%s'\r\n", argv[1]);
		return 1;
	}
	if (psram_guard(sh))
		return 1;
	rc = psram_probe_pair(ma, &d);
	psram_release();
	if (rc != 0) {
		cli_error(sh, "psram: MA must be 0, 2, 4 or 8 (even pairs)\r\n");
		return 1;
	}
	cli_print(sh, "psram probe MA%lu (expect MA0=09 0D, MA2=93 E0 at power-up):\r\n",
	          (unsigned long)ma);
	psram_print_diag(sh, &d);
	return 0;
}

/* psram init: re-run the full bring-up (Global Reset + MR pair reads + mmap
 * entry) with the LIVE knob values -- so `psram set`/`psram dqs`/... sweeps can
 * retry initialization without a reflash or power cycle. */
static int cmd_psram_init(struct cli_instance *sh, int argc, char **argv)
{
	int ok;

	(void)argc; (void)argv;
	if (psram_guard(sh))
		return 1;
	ok = psram_hw_init();
	psram_release();
	if (ok)
		cli_print(sh, "psram init: ready (mmap); run `psram test 4096`\r\n");
	else
		cli_error(sh, "psram init: FAILED at %s (see `psram snap`)\r\n",
		          psram_stage_name(psram_init_stage()));
	return psram_ready() ? 0 : 1;
}

/* psram grst: manually re-issue the 4-clock Global Reset (datasheet allows it
 * only as power-up init; mid-session use is a diagnostic). */
static int cmd_psram_grst(struct cli_instance *sh, int argc, char **argv)
{
	struct psram_diag d;
	int ok;

	(void)argc; (void)argv;
	if (psram_guard(sh))
		return 1;
	ok = psram_global_reset_cmd();
	psram_get_last_diag(&d);
	psram_release();
	cli_print(sh, "psram grst: %s\r\n", ok ? "TCF completed" : "TCF TIMEOUT");
	psram_print_diag(sh, &d);
	if (ok)
		cli_print(sh, "device registers reverted to power-up defaults; "
		          "now `psram probe 0`\r\n");
	return 0;
}

/* psram snap: init-failure snapshot + live controller registers. */
static int cmd_psram_snap(struct cli_instance *sh, int argc, char **argv)
{
	struct psram_regs r;

	(void)argc; (void)argv;
	if (!psram_ready()) {
		struct psram_diag d;

		psram_get_init_diag(&d);
		cli_print(sh, "init failure snapshot:\r\n");
		psram_print_diag(sh, &d);
	} else {
		cli_print(sh, "init: OK (no failure snapshot)\r\n");
	}
	psram_snap_regs(&r);
	cli_print(sh, "live OCTOSPI1:\r\n");
	cli_print(sh, "  CR=0x%08lX SR=0x%08lX\r\n",
	          (unsigned long)r.cr, (unsigned long)r.sr);
	cli_print(sh, "  DCR1=0x%08lX DCR2=0x%08lX DCR3=0x%08lX DCR4=0x%08lX\r\n",
	          (unsigned long)r.dcr1, (unsigned long)r.dcr2,
	          (unsigned long)r.dcr3, (unsigned long)r.dcr4);
	cli_print(sh, "  CCR=0x%08lX TCR=0x%08lX IR=0x%02lX AR=0x%08lX DLR=0x%lX\r\n",
	          (unsigned long)r.ccr, (unsigned long)r.tcr,
	          (unsigned long)r.ir, (unsigned long)r.ar, (unsigned long)r.dlr);
	cli_print(sh, "  WCCR=0x%08lX WTCR=0x%08lX WIR=0x%02lX\r\n",
	          (unsigned long)r.wccr, (unsigned long)r.wtcr,
	          (unsigned long)r.wir);
	cli_print(sh, "  OCTOSPIM CR=0x%08lX P1CR=0x%08lX P2CR=0x%08lX\r\n",
	          (unsigned long)r.om_cr, (unsigned long)r.om_p1cr,
	          (unsigned long)r.om_p2cr);
	cli_print(sh, "  DLYB CR=0x%08lX CFGR=0x%08lX\r\n",
	          (unsigned long)r.dlyb_cr, (unsigned long)r.dlyb_cfgr);
	return 0;
}

/* psram pins: GPIO mode/AF/pull/input of every PSRAM pin. */
static int cmd_psram_pins(struct cli_instance *sh, int argc, char **argv)
{
	struct psram_pin p[16];
	uint32_t i, n;
	static const char *const mode_name[4] = { "IN ", "OUT", "AF ", "ANA" };
	static const char *const pupd_name[4] = { "-", "up", "down", "?" };

	(void)argc; (void)argv;
	n = psram_snap_pins(p, 16u);
	for (i = 0u; i < n; i++)
		cli_print(sh, "  %-3s P%c%-2u %s AF%-2u pull=%-4s in=%u\r\n",
		          p[i].name, p[i].port, p[i].pin,
		          mode_name[p[i].mode & 3u], p[i].af,
		          pupd_name[p[i].pupd & 3u], p[i].idr);
	cli_print(sh, "expected: all AF mode; IO/DQS/NCS AF10, CLK AF9; DQS pull-down\r\n");
	return 0;
}

/* psram dqs <0|1>: DQS-gated capture on the register/probe read path.  0 =
 * sample with the internal clock instead -- if a probe then returns any
 * non-FF data, the device answers and the DQS input path is the fault. */
static int cmd_psram_dqs(struct cli_instance *sh, int argc, char **argv)
{
	uint32_t en;

	(void)argc;
	if (parse_u32(argv[1], &en) != 0 || en > 1u) {
		cli_error(sh, "psram: use 1 (DQS-gated) or 0 (internal clock)\r\n");
		return 1;
	}
	psram_set_dqse((int)en);
	cli_print(sh, "psram: probe/register reads %s\r\n",
	          en ? "DQS-gated" : "internal-clock sampled");
	return 0;
}

/* psram pscan [ma]: sweep ifmt x dlyb x dqse x refresh x dcyc over one MA-pair
 * probe each and list every combination that produced ANY data.  The handover
 * plan's step 4: answers "does any configuration make the device respond at
 * all" in one command, without a reflash.  Restores the knobs afterwards. */
static int cmd_psram_pscan(struct cli_instance *sh, int argc, char **argv)
{
	uint32_t ma = 0u;
	uint32_t rd0, wr0, dqse0, ref0, ifmt0;
	uint32_t ifmt, dly, dq, refi, dcyc, hits = 0u, done = 0;
	static const uint32_t refs[2] = { 320u, 0u };

	if (argc >= 2 && (parse_u32(argv[1], &ma) != 0 ||
	    (ma != 0u && ma != 2u && ma != 4u && ma != 8u))) {
		cli_error(sh, "psram: MA must be 0, 2, 4 or 8\r\n");
		return 1;
	}
	if (psram_guard(sh))               /* held across the whole get/sweep/restore */
		return 1;
	psram_get_latency(&rd0, &wr0);
	dqse0 = psram_get_dqse();
	ref0  = psram_get_refresh();
	ifmt0 = psram_get_instruction_dtr();

	cli_print(sh, "pscan MA%lu: ifmt{8sdr,16dtr} x dlyb{on,byp} x dqse{1,0} "
	          "x ref{320,0} x dcyc 0..10\r\n", (unsigned long)ma);
	for (ifmt = 0u; ifmt <= 1u && !done; ifmt++) {
		psram_set_instruction_dtr((int)ifmt);
		for (dly = 0u; dly <= 1u && !done; dly++) {
			psram_dlyb_bypass((int)dly);
			for (dq = 0u; dq <= 1u && !done; dq++) {
				psram_set_dqse((int)(1u - dq));
				for (refi = 0u; refi < 2u && !done; refi++) {
					psram_set_refresh(refs[refi]);
					for (dcyc = 0u; dcyc <= 10u; dcyc++) {
						struct psram_diag d;

						psram_set_latency(dcyc, wr0);
						if (psram_probe_pair(ma, &d) != 0)
							continue;
						if (d.ndata != 0u) {
							hits++;
							cli_print(sh,
							          "  HIT ifmt=%lu dlyb=%s dqse=%lu ref=%-3lu "
							          "dcyc=%-2lu %s D0=0x%02X D1=0x%02X\r\n",
							          (unsigned long)ifmt, dly ? "byp" : "on ",
							          (unsigned long)(1u - dq),
							          (unsigned long)refs[refi],
							          (unsigned long)dcyc,
							          d.ok ? "ok  " : "part",
							          d.data[0], d.data[1]);
						}
						if (cli_cancel_requested(sh)) {
							cli_print(sh, "\r\npscan: canceled\r\n");
							done = 1;
							break;
						}
					}
				}
			}
		}
	}
	/* Restore the pre-scan knobs (delay block re-engaged). */
	psram_set_instruction_dtr((int)ifmt0);
	psram_dlyb_bypass(0);
	psram_set_dqse((int)dqse0);
	psram_set_refresh(ref0);
	psram_set_latency(rd0, wr0);
	psram_release();
	cli_print(sh, "pscan: %lu responding combination(s)%s\r\n",
	          (unsigned long)hits,
	          hits ? "" : " -- device never drove data (init/power/HW level)");
	return 0;
}

/* psram clk <presc>: device clock = 266MHz/(presc+1).  2=88.7M, 7=33.3M. */
static int cmd_psram_clk(struct cli_instance *sh, int argc, char **argv)
{
	uint32_t p;

	(void)argc;
	if (parse_u32(argv[1], &p) != 0 || p > 255u) {
		cli_error(sh, "psram: bad prescaler (0..255)\r\n");
		return 1;
	}
	if (psram_guard(sh))
		return 1;
	psram_set_prescaler(p);
	psram_release();
	cli_print(sh, "psram: clock %lu Hz; run `psram test 4096`\r\n",
	          (unsigned long)psram_clock_hz());
	/* DCR4.REFRESH is not re-derived here (RM0468 sec 25.7.5: it counts kernel
	 * cycles, so its wall-clock window shifts with the prescaler).  Harmless at
	 * the shipped point -- CSBOUND=32B releases NCS long before the refresh
	 * limiter or tCEM -- but a diagnostic caveat for the `psram clk` sweep. */
	cli_print(sh, "  note: DCR4 refresh not re-tuned for this clock (diagnostic)\r\n");
	return 0;
}

/* psram dlyb <1|0>: enable (un-bypass, at current phase/unit) or bypass the
 * read delay block. */
static int cmd_psram_dlyb(struct cli_instance *sh, int argc, char **argv)
{
	uint32_t en;

	(void)argc;
	if (parse_u32(argv[1], &en) != 0 || en > 1u) {
		cli_error(sh, "psram: use 1 (enable) or 0 (bypass)\r\n");
		return 1;
	}
	if (psram_guard(sh))
		return 1;
	psram_dlyb_bypass(en == 0u);
	psram_release();
	cli_print(sh, "psram: delay block %s\r\n", en ? "enabled" : "bypassed");
	return 0;
}

/* psram phase <sel> [units]: override the DLYB read-sampling phase to sweep for
 * the value that aligns the read data eye (no reflash). */
static int cmd_psram_phase(struct cli_instance *sh, int argc, char **argv)
{
	uint32_t sel, unit, cur_sel, cur_unit;

	psram_get_phase(&cur_sel, &cur_unit);
	unit = cur_unit;
	if (parse_u32(argv[1], &sel) != 0 || sel > 12u ||
	    (argc >= 3 && (parse_u32(argv[2], &unit) != 0 || unit > 127u))) {
		cli_error(sh, "psram: bad phase (sel 0..12, units 0..127)\r\n");
		return 1;
	}
	if (psram_guard(sh))
		return 1;
	psram_set_phase(sel, unit);
	psram_release();
	cli_print(sh, "psram: dlyb phase %lu / units %lu; run `psram test 4096`\r\n",
	          (unsigned long)sel, (unsigned long)unit);
	return 0;
}

/* psram mr0 <hex>: write the APS6408 MR0 read-latency register (#16 diagnostic).
 * 0x09 = power-up default (Variable LC5); 0x24 = Fixed Latency LC8 (ST U585
 * reference -- pair with `psram set 8 <wr>`).  Verify with `psram probe 0`. */
static int cmd_psram_mr0(struct cli_instance *sh, int argc, char **argv)
{
	uint32_t v;

	(void)argc;
	if (!psram_ready()) {
		cli_error(sh, "psram: not ready\r\n");
		return 1;
	}
	if (parse_u32(argv[1], &v) != 0 || v > 0xFFu) {
		cli_error(sh, "psram: bad MR0 value (0..0xFF)\r\n");
		return 1;
	}
	if (psram_guard(sh))
		return 1;
	psram_set_mr0(v);
	psram_release();
	cli_print(sh, "psram: MR0<=0x%02lX; set matching read dummy (`psram set`), "
	          "verify `psram probe 0`, then `psram test 4096`\r\n",
	          (unsigned long)v);
	return 0;
}

/* psram set <read_dcyc> <write_dcyc>: re-enter mmap with new latencies to sweep
 * for values that pass `psram test` without a reflash. */
static int cmd_psram_set(struct cli_instance *sh, int argc, char **argv)
{
	uint32_t rd, wr;

	(void)argc;
	if (parse_u32(argv[1], &rd) != 0 || parse_u32(argv[2], &wr) != 0 ||
	    rd > 31u || wr > 31u) {
		cli_error(sh, "psram: bad dummy-cycle count (0..31)\r\n");
		return 1;
	}
	if (psram_guard(sh))
		return 1;
	psram_set_latency(rd, wr);
	psram_release();
	cli_print(sh, "psram: latency read %lu / write %lu; run `psram test 4096`\r\n",
	          (unsigned long)rd, (unsigned long)wr);
	return 0;
}

/* Write `pat` to every word in [base,base+len), then read back and compare.
 * `addr_pattern` != 0 stores each word's own address instead of `pat`.  Returns
 * 0 on match, 1 on the first mismatch (already reported).  Cancelable. */
static int test_pass(struct cli_instance *sh, uint32_t base, uint32_t words,
                     uint32_t pat, int addr_pattern, const char *label)
{
	volatile uint32_t *p = (volatile uint32_t *)(uintptr_t)base;
	uint32_t i;

	for (i = 0u; i < words; i++) {
		p[i] = addr_pattern ? (base + i * 4u) : pat;
		if ((i & 0x3FFFu) == 0u && cli_cancel_requested(sh)) {
			cli_print(sh, "\r\n%s: canceled (write @0x%08lX)\r\n",
			          label, (unsigned long)(base + i * 4u));
			return 1;
		}
	}
	for (i = 0u; i < words; i++) {
		uint32_t want = addr_pattern ? (base + i * 4u) : pat;
		uint32_t got  = p[i];
		if (got != want) {
			cli_error(sh, "%s: MISMATCH @0x%08lX  wrote 0x%08lX  read 0x%08lX\r\n",
			          label, (unsigned long)(base + i * 4u),
			          (unsigned long)want, (unsigned long)got);
			return 1;
		}
		if ((i & 0x3FFFu) == 0u && cli_cancel_requested(sh)) {
			cli_print(sh, "\r\n%s: canceled (verify @0x%08lX)\r\n",
			          label, (unsigned long)(base + i * 4u));
			return 1;
		}
	}
	cli_print(sh, "  %-12s OK\r\n", label);
	return 0;
}

static int cmd_psram_test(struct cli_instance *sh, int argc, char **argv)
{
	uint32_t len = PSRAM_SIZE_BYTES;
	uint32_t words;

	if (!psram_ready()) {
		cli_error(sh, "psram: not ready (bring-up failed; see `psram info`)\r\n");
		return 1;
	}
	if (argc >= 2 && parse_u32(argv[1], &len) != 0) {
		cli_error(sh, "psram: bad length '%s'\r\n", argv[1]);
		return 1;
	}
	if (len == 0u || len > PSRAM_SIZE_BYTES)
		len = PSRAM_SIZE_BYTES;
	len &= ~0x3u;                       /* whole 32-bit words */
	words = len / 4u;

	cli_print(sh, "psram test: 0x%08lX, %lu bytes\r\n",
	          (unsigned long)PSRAM_BASE_ADDR, (unsigned long)len);

	if (psram_guard(sh))               /* held across all three mmap passes */
		return 1;
	if (test_pass(sh, PSRAM_BASE_ADDR, words, 0x00000000u, 1, "addr-in-addr") ||
	    test_pass(sh, PSRAM_BASE_ADDR, words, 0x55555555u, 0, "0x55555555") ||
	    test_pass(sh, PSRAM_BASE_ADDR, words, 0xAAAAAAAAu, 0, "0xAAAAAAAA")) {
		psram_release();
		return 1;
	}
	psram_release();

	cli_print(sh, "psram test: PASS (%lu bytes verified)\r\n", (unsigned long)len);
	return 0;
}

/* psram mmapscan <start [phase]|show|stop>: reset-persistent DLYB sweep
 * validated against real memory-mapped access (issue #16).  `start` captures the
 * current clock/latency/MR0 and sweeps DLYB units 0..124 (step 4) on `phase`
 * (default 3), auto-rebooting per unit -- the console drops during the sweep and
 * returns when it completes; then `show` prints which units passed mmap. */
static int cmd_psram_mmapscan(struct cli_instance *sh, int argc, char **argv)
{
	struct psram_scan_state st;
	uint32_t phase = 3u, ulo, ustep, ncand, idx, ci;
	int state;

	if (argc < 2) {
		cli_error(sh, "psram: usage mmapscan <start [phase]|show|stop>\r\n");
		return 1;
	}
	if (strcmp(argv[1], "stop") == 0) {
		psram_mmapscan_stop();
		cli_print(sh, "psram mmapscan: stopped\r\n");
		return 0;
	}
	if (strcmp(argv[1], "show") == 0) {
		state = psram_mmapscan_get(&st);
		if (state == 0) {
			cli_print(sh, "psram mmapscan: no sweep recorded\r\n");
			return 0;
		}
		ulo   = st.rng & 0xFFu;
		ustep = (st.rng >> 8) & 0xFFu;
		ncand = (st.rng >> 16) & 0xFFu;
		idx   = (st.rng >> 24) & 0xFFu;
		cli_print(sh, "psram mmapscan: %s  phase %lu  presc %lu (%lu Hz)  "
		          "rd %lu wr %lu  mr0 0x%02lX\r\n",
		          (state == 2) ? "COMPLETE" : "running",
		          (unsigned long)((st.cfg >> 8) & 0xFFu),
		          (unsigned long)(st.cfg & 0xFFu),
		          (unsigned long)(266000000u / ((st.cfg & 0xFFu) + 1u)),
		          (unsigned long)((st.cfg >> 16) & 0xFFu),
		          (unsigned long)((st.cfg >> 24) & 0xFFu),
		          (unsigned long)st.mr0);
		cli_print(sh, "  mmap eye (P pass / x fail / . untested), "
		          "unit %lu..%lu step %lu:\r\n", (unsigned long)ulo,
		          (unsigned long)(ulo + (ncand - 1u) * ustep), (unsigned long)ustep);
		cli_print(sh, "  |");
		for (ci = 0u; ci < ncand; ci++)
			cli_print(sh, "%c", (st.passed & (1u << ci)) ? 'P'
			          : (st.tested & (1u << ci)) ? 'x' : '.');
		cli_print(sh, "|  (%lu/%lu tested)\r\n",
		          (unsigned long)(idx < ncand ? idx : ncand), (unsigned long)ncand);
		for (ci = 0u; ci < ncand; ci++)
			if (st.passed & (1u << ci))
				cli_print(sh, "  PASS unit %lu\r\n",
				          (unsigned long)(ulo + ci * ustep));
		return 0;
	}
	if (strcmp(argv[1], "start") == 0) {
#if !BSP_ENABLE_IWDG
		/* The sweep relies on the IWDG to recover a hanging mmap read at a bad
		 * DLYB point; without it a hang is unrecoverable (a permanent lock). */
		cli_error(sh, "psram: mmapscan needs the IWDG (built -DBSP_ENABLE_IWDG=OFF)\r\n");
		return 1;
#else
		if (!psram_ready()) {
			cli_error(sh, "psram: not ready\r\n");
			return 1;
		}
		if (argc >= 3 && (parse_u32(argv[2], &phase) != 0 ||
		    phase < 1u || phase > 12u)) {
			cli_error(sh, "psram: bad phase (1..12)\r\n");
			return 1;
		}
		/* Serialize the config snapshot against a backgrounded psram/membench
		 * command so we capture a coherent clock/latency/MR0 (start does not
		 * return on success, so no release needed then). */
		if (psram_guard(sh))
			return 1;
		cli_print(sh, "psram mmapscan: sweeping phase %lu, unit 0..124 step 4 at "
		          "the current clock/latency; the board will auto-reboot per unit "
		          "(console drops). Reconnect when it stops, then `psram mmapscan "
		          "show`.\r\n", (unsigned long)phase);
		/* Does not return on success (system reset into the boot loop). */
		if (!psram_mmapscan_start(phase, 0u, PSRAM_SCAN_STEP, PSRAM_SCAN_COLS)) {
			psram_release();
			cli_error(sh, "psram: mmapscan start rejected\r\n");
		}
		return 1;
#endif
	}
	cli_error(sh, "psram: usage mmapscan <start [phase]|show|stop>\r\n");
	return 1;
}

/* psram wtune [addr]: sweep the mmap write dummy 0..15 with write+readback
 * markers (needs a trusted read path -- run `psram eye` first). */
static int cmd_psram_wtune(struct cli_instance *sh, int argc, char **argv)
{
	uint32_t addr = PSRAM_BASE_ADDR + 0x200u;
	uint32_t mask, d;

	if (argc >= 2 && parse_u32(argv[1], &addr) != 0) {
		cli_error(sh, "psram: bad address '%s'\r\n", argv[1]);
		return 1;
	}
	if (!psram_ready() || addr < PSRAM_BASE_ADDR ||
	    addr > PSRAM_BASE_ADDR + PSRAM_SIZE_BYTES - 8u) {
		cli_error(sh, "psram: not ready / bad addr\r\n");
		return 1;
	}
	if (psram_guard(sh))
		return 1;
	mask = psram_wtune(addr - PSRAM_BASE_ADDR, 4u);
	psram_release();
	for (d = 0u; d <= 15u; d++)
		cli_print(sh, "  wr dummy %2lu: %s\r\n", (unsigned long)d,
		          (mask & (1u << d)) ? "PASS" : "fail");
	if (mask != 0u)
		cli_print(sh, "psram wtune: lowest passing dummy applied; "
		          "run `psram test 4096`\r\n");
	else
		cli_print(sh, "psram wtune: nothing round-tripped\r\n");
	return 0;
}

CLI_SUBCMD_SET_CREATE(psram_subcmds,
	CLI_CMD_ARG(info, NULL, "show PSRAM state/clock/id/latency", cmd_psram_info, 1, 0),
	CLI_CMD_ARG(init, NULL, "re-run bring-up with live knobs",     cmd_psram_init, 1, 0),
	CLI_CMD_ARG(probe, NULL, "MA-pair read + SR snapshot [0|2|4|8]", cmd_psram_probe, 1, 1),
	CLI_CMD_ARG(pscan, NULL, "sweep knobs, list responding combos [ma]", cmd_psram_pscan, 1, 1),
	CLI_CMD_ARG(grst, NULL, "re-issue Global Reset (diagnostic)",  cmd_psram_grst, 1, 0),
	CLI_CMD_ARG(snap, NULL, "init-failure + live register snapshot", cmd_psram_snap, 1, 0),
	CLI_CMD_ARG(pins, NULL, "GPIO mode/AF/pull of PSRAM pins",     cmd_psram_pins, 1, 0),
	CLI_CMD_ARG(dqs,  NULL, "probe-read DQS gating <1|0>",         cmd_psram_dqs,  2, 0),
	CLI_CMD_ARG(test, NULL, "write/verify patterns [bytes]",     cmd_psram_test, 1, 1),
	CLI_CMD_ARG(set,  NULL, "set mmap latency <rd> <wr> cycles", cmd_psram_set,  3, 0),
	CLI_CMD_ARG(mr0,  NULL, "write device MR0 latency reg <hex>", cmd_psram_mr0,  2, 0),
	CLI_CMD_ARG(phase, NULL, "set dlyb read phase <sel> [units]", cmd_psram_phase, 2, 1),
	CLI_CMD_ARG(mmapscan, NULL, "reset-persistent mmap DLYB sweep <start|show|stop>", cmd_psram_mmapscan, 2, 1),
	CLI_CMD_ARG(wtune, NULL, "sweep mmap write dummy 0..15 [addr]", cmd_psram_wtune, 1, 1),
	CLI_CMD_ARG(clk,  NULL, "set device clock prescaler <0..255>", cmd_psram_clk,   2, 0),
	CLI_CMD_ARG(dlyb, NULL, "delay block bypass off/on <1|0>",     cmd_psram_dlyb,  2, 0),
	CLI_SUBCMD_SET_END);

CLI_CMD_REGISTER(psram, psram_subcmds,
                 "OCTOSPI1 APS6408 PSRAM info/test", NULL, 1, 0);
