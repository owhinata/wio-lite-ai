/*
 * Wio Lite AI (STM32H725AEI6) -- OCTOSPI1 APS6408 PSRAM API (issue #3).
 */
#ifndef APP_PSRAM_H
#define APP_PSRAM_H

#include <stdint.h>

/* External OCTOSPI1 APS6408 memory-mapped window. */
#define PSRAM_BASE_ADDR   0x90000000u
#define PSRAM_SIZE_BYTES  0x00800000u   /* 8 MB (APS6408_RAM_SIZE) */

/* Bring-up failure stages (psram_init_stage): which init transaction failed.
 * OK means every stage passed and memory-mapped mode was entered. */
#define PSRAM_STAGE_OK    0u
#define PSRAM_STAGE_GRST  1u            /* Global Reset transaction (TCF timeout) */
#define PSRAM_STAGE_MR0   2u            /* MA0 2-byte register-pair read */
#define PSRAM_STAGE_MR2   3u            /* MA2 2-byte register-pair read */

/* Observability record of one indirect transaction (init failure snapshot and
 * `psram probe`).  SR is sampled at three points: before arming the transfer,
 * when the first data poll ended (FTF set or timed out; 0 for the data-less
 * Global Reset), and when the completion poll ended (TCF set or timed out).
 * SR bit0=TEF bit1=TCF bit2=FTF bit3=SMF bit4=TOF bit5=BUSY [13:8]=FLEVEL.
 * Config registers are read back AFTER arming, so `ar`/`dlr` show what the
 * ES0491 2.8.4 even-address/length clearing actually left in hardware. */
struct psram_diag {
	uint8_t  stage;                 /* PSRAM_STAGE_* this record describes */
	uint8_t  ok;                    /* 1 = transaction completed (TCF) */
	uint8_t  ndata;                 /* bytes that actually arrived (0..2) */
	uint8_t  data[2];               /* first two data bytes, if any */
	uint32_t sr_pre, sr_data, sr_end;
	uint32_t cr, ccr, tcr, ir, ar, dlr;
	uint32_t dcr1, dcr2, dcr3, dcr4;
	uint32_t dlyb_cr, dlyb_cfgr;
};

/* Live OCTOSPI1 + IO-manager + delay-block register snapshot (`psram snap`). */
struct psram_regs {
	uint32_t cr, sr, dcr1, dcr2, dcr3, dcr4;
	uint32_t ccr, tcr, ir, ar, dlr;
	uint32_t wccr, wtcr, wir;
	uint32_t om_cr, om_p1cr, om_p2cr;
	uint32_t dlyb_cr, dlyb_cfgr;
};

/* One PSRAM pin's GPIO state (`psram pins`): MODER field (2=AF), AF number,
 * pull (0=none 1=up 2=down), and the current input level. */
struct psram_pin {
	char        port;               /* 'B','D','E','F','G' */
	uint8_t     pin;                /* 0..15 */
	uint8_t     mode;
	uint8_t     af;
	uint8_t     pupd;
	uint8_t     idr;
	const char *name;               /* "IO0".."IO7","DQS","CLK","NCS" */
};

/*
 * Bring up OCTOSPI1 + the APS6408 Octal DDR PSRAM in memory-mapped mode.
 *
 * Bare-register, bounded poll-loops, fail-soft.  Touches ONLY OCTOSPI1 and GPIO
 * banks B/D/E -- never OCTOSPI2, the OCTOSPIM I/O manager, or the RCC
 * clock tree -- so it is safe to call while the app executes XIP from OCTOSPI2 at
 * 0x70000000 (OCTOSPI1 is a separate port + register block sharing only the
 * already-running PLL2R kernel clock).  Uses no HAL_GetTick or libc.
 *
 * Call after HAL_Init(), before tx_kernel_enter().  Returns 1 if the device
 * answered and memory-mapped mode was entered, 0 on failure (the shell still
 * runs; `psram info` reports "not ready").
 */
int psram_hw_init(void);

/* 1 once psram_hw_init() has entered memory-mapped mode. */
int psram_ready(void);

/* Serialize access to the single OCTOSPI1 resource across concurrent shell
 * commands (`cmd &` background workers).  psram_acquire() returns 1 on success,
 * 0 if another command already holds it (caller should reject and return).
 * Bracket every shell path that mutates OCTOSPI1 state or does memory-mapped
 * access at 0x90000000; release on every exit.  Thread context only. */
int psram_acquire(void);
void psram_release(void);

/* Which bring-up stage failed (PSRAM_STAGE_*); PSRAM_STAGE_OK when ready. */
uint32_t psram_init_stage(void);

/* Copy of the transaction snapshot recorded at the first init failure. */
void psram_get_init_diag(struct psram_diag *out);

/* Copy of the most recent instrumented transaction's snapshot (any register
 * read or Global Reset, success or failure). */
void psram_get_last_diag(struct psram_diag *out);

/* Issue one legal 2-byte register-pair read at even mode address 0/2/4/8 with
 * the CURRENT knob settings (latency/dqse/ifmt/dlyb/refresh), even when the
 * bring-up failed, and return its full transaction snapshot.  Leaves mmap and
 * restores it afterwards when the PSRAM is up.  -1 on a bad address. */
int psram_probe_pair(uint32_t ma, struct psram_diag *out);

/* Re-issue the four-clock Global Reset (datasheet Fig.4) at any time; returns
 * 1 if the transaction completed (TCF).  Device registers revert to power-up
 * defaults; per the datasheet this is only legal as power-up initialization,
 * so treat a mid-session use as a diagnostic. */
int psram_global_reset_cmd(void);

/* DQS-gated data capture for the indirect register-read path (default 1).
 * 0 samples with the internal clock instead: if a probe then returns ANY
 * non-0xFF data the device is answering and the DQS path is the problem. */
uint32_t psram_get_dqse(void);
void psram_set_dqse(int en);

/* DCR4.REFRESH transaction limiter (0 disables; nonzero must exceed one whole
 * minimum transaction).  Applied with the OCTOSPI disabled. */
uint32_t psram_get_refresh(void);
void psram_set_refresh(uint32_t r);

/* Live register + pin snapshots for `psram snap` / `psram pins`.
 * psram_snap_pins fills up to `max` entries, returns the count. */
void psram_snap_regs(struct psram_regs *out);
uint32_t psram_snap_pins(struct psram_pin *out, uint32_t max);

/* OCTOSPI1 device clock in Hz (kernel / prescaler). */
uint32_t psram_clock_hz(void);

/* Change the device-clock prescaler at runtime (clock = 266MHz/(presc+1); e.g.
 * 2=88.7MHz, 7=33.3MHz).  Diagnostic knob (`psram clk`). */
void psram_set_prescaler(uint32_t presc);

/* Bypass (1) / engage (0) the read delay block at the current phase/unit. */
void psram_dlyb_bypass(int bypass);

/* The two ID/mode-register bytes read at bring-up (vendor/density), for `psram
 * info`.  id[0]=MR1, id[1]=MR2 as read back (0xFF/0x00 => nothing answered). */
void psram_read_id(uint8_t id[2]);

/* Current memory-mapped read/write dummy-cycle (latency) counts. */
void psram_get_latency(uint32_t *rd_dcyc, uint32_t *wr_dcyc);

/* Re-enter memory-mapped mode with new read/write dummy-cycle counts, to sweep
 * the APS6408 latency on hardware without a reflash (`psram set` command). */
void psram_set_latency(uint32_t rd_dcyc, uint32_t wr_dcyc);

/* Select instruction framing while keeping address/data in DTR: 0 = official
 * APS6408 8-bit SDR instruction, 1 = doubled 16-bit DTR diagnostic form. */
uint32_t psram_get_instruction_dtr(void);
void psram_set_instruction_dtr(int dtr);

/* Delay-block (read data-eye) sampling phase: getter + runtime override, to
 * sweep the DLYB phase without a reflash (`psram phase` command). */
void psram_get_phase(uint32_t *sel, uint32_t *unit);
void psram_set_phase(uint32_t sel, uint32_t unit);

/* Read-path timing flags (bit0=SSHIFT, bit1=DHQC), for `psram info`. */
uint32_t psram_get_read_flags(void);

/* APS6408 read-latency register MR0 (device-side latency code), for `psram
 * info`.  Reflects the power-up default unless a diagnostic changed it. */
uint32_t psram_get_mr0(void);

/* Read-eye scan: write one 32-byte marker, then score all DLYB points through
 * repeated full-marker reads.  Fills counts[1..12], leaves the best phase/unit
 * applied (reverts to bypass when nothing scores), and returns its phase.
 * See the note in psram.c: an eye result must survive `psram test` before being
 * adopted as the operating point (issue #16). */
uint32_t psram_scan_eye(uint32_t counts[13], uint32_t trials);

/* Write-latency tuner: sweep mmap write dummy 0..15 with marker write+readback
 * (x`reps` each); returns a pass bitmask (bit d = dummy d round-tripped) and
 * leaves the lowest passing dummy applied (`psram wtune`). */
uint32_t psram_wtune(uint32_t off, uint32_t reps);

#endif /* APP_PSRAM_H */
