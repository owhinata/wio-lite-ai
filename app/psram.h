/*
 * Wio Lite AI (STM32H725AEI6) -- OCTOSPI1 APS6408 PSRAM API (issue #3).
 */
#ifndef APP_PSRAM_H
#define APP_PSRAM_H

#include <stdint.h>

/* External OCTOSPI1 APS6408 memory-mapped window. */
#define PSRAM_BASE_ADDR   0x90000000u
#define PSRAM_SIZE_BYTES  0x00800000u   /* 8 MB (APS6408_RAM_SIZE) */

/*
 * The window is not uniform since issue #9 phase 2a.  Its top 2 MB is carved out as
 * Normal cacheable write-back (app/mpu.c region 3) for the CPU-only working set of
 * the NN runtime -- activations, input staging and model slots -- because inference
 * through a non-cacheable window is bound by one bus transaction per access rather
 * than by bandwidth.  Everything below stays Normal non-cacheable, which is what
 * lets the LTDC, the DMA2D and the DCMI share their buffers with the CPU untouched.
 *
 * Two rules follow, and both are enforced after every link by
 * cmake/check_psram_ai_residency.py:
 *   - nothing a bus master touches may live in the carve-out (it would read stale
 *     cache lines, intermittently, under cache pressure);
 *   - the linker script's .psram_ai section must start exactly at PSRAM_AI_BASE_ADDR,
 *     since the MPU region and the section are declared in different files.
 *
 * 2 MB naturally aligned at 0x90600000, so the MPU needs no sub-region masking.
 */
#define PSRAM_AI_BASE_ADDR  0x90600000u
#define PSRAM_AI_BYTES      0x00200000u   /* 2 MB, ending at the top of the window */

/* The non-cacheable pool: everything below the carve-out.  `psram test` and the
 * membench PSRAM row bound themselves by this -- a write-back region cannot be used
 * to test the device, because a store followed by a load can be answered by the
 * cache and never reach the APS6408 at all. */
#define PSRAM_NC_BYTES    (PSRAM_AI_BASE_ADDR - PSRAM_BASE_ADDR)   /* 6 MB */

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

/*
 * Bring up OCTOSPI1 + the APS6408 Octal DDR PSRAM in memory-mapped mode.
 *
 * Bare-register, bounded poll-loops, fail-soft.  Touches ONLY OCTOSPI1 and its
 * Port-1 pins (GPIO banks B/D/E/F/G, per-pin RMW) -- never OCTOSPI2, the OCTOSPIM
 * routing registers, or the RCC clock tree; it runs on the PLL2R kernel clock the
 * bootloader already started.  Uses no HAL_GetTick or libc.
 *
 * Call after HAL_Init(), before tx_kernel_enter().  Returns 1 if the device
 * answered and memory-mapped mode was entered, 0 on failure (the shell still
 * runs; `psram info` reports "not ready").
 */
int psram_hw_init(void);

/* 1 once psram_hw_init() has entered memory-mapped mode. */
int psram_ready(void);

/*
 * Serialize access to the single OCTOSPI1 resource across concurrent shell
 * commands (`cmd &` background workers).  Both calls return 1 on success and 0
 * when refused (the caller rejects and returns); both are released with
 * psram_release().  Thread context only.
 *
 * The two differ in what they refuse, because the callers differ in what they
 * do to the bus:
 *
 *   psram_acquire()        -- for anything that RECONFIGURES OCTOSPI1: the
 *                             `psram` tuning subcommands, `membench`, `devmem`,
 *                             `wifi flash`.  Refused while a continuous user
 *                             (LTDC scan-out, a camera stream) is on the bus,
 *                             because retuning underneath one does not spoil a
 *                             frame -- it stalls the AXI indefinitely.
 *
 *   psram_acquire_shared() -- for callers that only READ OR WRITE the memory:
 *                             `lcd on`, `camera stream start`, `camera capture`
 *                             / `save` / `send`, `ai bench`, `ai stream`.
 *                             Refused only while a reconfiguring command holds
 *                             the bus.  These do not exclude each other, which is
 *                             what lets the display scan out while the camera
 *                             streams (issue #8 3c).
 *
 * A continuous user takes the shared guard just long enough to arm itself, then
 * releases it: from that point the gates above (ltdc_scanout_active(),
 * camera_streaming()) are what keep the reconfigurers away.
 *
 * 🔴 `ai stream` IS THE ONE EXCEPTION: it holds the shared guard for its whole
 * lifetime (issue #9 phase 3, app/nn_camera.h).  Arm-and-release does not work for
 * it, because the two gates above answer for the CAMERA and the DISPLAY, and the
 * thing that must not be retuned underneath is neither -- it is the NN worker
 * reading its arena out of the cacheable PSRAM carve-out.  The moment a DCMI
 * overrun tears the band stream down, camera_streaming() goes false while that
 * worker is still inside nn_run(), and a `psram clk` would become legal.  The
 * consequence is that a running `ai stream` refuses a NEW `lcd on` / `lcd reset` /
 * `camera capture` / `camera preview on`, so every site that reports this guard as
 * busy names `ai stream` as a possible holder.
 */
int psram_acquire(void);
int psram_acquire_shared(void);
void psram_release(void);

/* Which bring-up stage failed (PSRAM_STAGE_*); PSRAM_STAGE_OK when ready, for
 * the `psram info` "init: failed at ..." line. */
uint32_t psram_init_stage(void);

/* DQS-gate / DCR4.REFRESH values for `psram info` (set at bring-up). */
uint32_t psram_get_dqse(void);
uint32_t psram_get_refresh(void);

/* OCTOSPI1 device clock in Hz (kernel / prescaler). */
uint32_t psram_clock_hz(void);

/* Change the device-clock prescaler at runtime (clock = 266MHz/(presc+1); e.g.
 * 2=88.7MHz, 7=33.3MHz).  Diagnostic knob (`psram clk`). */
void psram_set_prescaler(uint32_t presc);

/* The two ID/mode-register bytes read at bring-up (vendor/density), for `psram
 * info`.  id[0]=MR1, id[1]=MR2 as read back (0xFF/0x00 => nothing answered). */
void psram_read_id(uint8_t id[2]);

/* Current memory-mapped read/write dummy-cycle (latency) counts. */
void psram_get_latency(uint32_t *rd_dcyc, uint32_t *wr_dcyc);

/* Re-enter memory-mapped mode with new read/write dummy-cycle counts, to sweep
 * the APS6408 latency on hardware without a reflash (`psram set` command). */
void psram_set_latency(uint32_t rd_dcyc, uint32_t wr_dcyc);

/* Instruction framing (0 = 8-bit SDR, the shipped form), for `psram info`. */
uint32_t psram_get_instruction_dtr(void);

/* Delay-block (read data-eye) sampling phase: getter + runtime override, to
 * sweep the DLYB phase without a reflash (`psram phase` command). */
void psram_get_phase(uint32_t *sel, uint32_t *unit);
void psram_set_phase(uint32_t sel, uint32_t unit);

/* Read-path timing flags (bit0=SSHIFT, bit1=DHQC), for `psram info`. */
uint32_t psram_get_read_flags(void);

/* APS6408 read-latency register MR0 (device-side latency code): getter for
 * `psram info`, and a diagnostic writer (`psram mr0`) to try e.g. Fixed Latency
 * LC8 (0x24) -- deterministic read timing to widen the eye at high clock (#16).
 * Verify with `psram probe 0` (MA0 D0 = MR0). */
uint32_t psram_get_mr0(void);
void psram_set_mr0(uint32_t val);

/* mmapscan (issue #16): reset-persistent DLYB sweep validated against real
 * memory-mapped access.  cfg/rng pack the plan and progress into two words so
 * the whole struct is a handful of DTCM words (read-back-persisted, #13):
 *   cfg = presc[7:0] | phase[15:8] | rd_dcyc[23:16] | wr_dcyc[31:24]
 *   rng = unit_lo[7:0] | unit_step[15:8] | ncand[23:16] | idx[31:24]
 * tested/passed are per-candidate bitmaps; mr0 is the device MR0 to write (0 =
 * keep the power-up default). */
struct psram_scan_state {
	uint32_t magic;
	uint32_t cfg;
	uint32_t rng;
	uint32_t tested;
	uint32_t passed;
	uint32_t mr0;
};

/* Boot hook: run one sweep step (called from main() after psram_hw_init(),
 * before tx_kernel_enter()).  Returns immediately unless a sweep is active. */
void psram_mmapscan_boot(void);

/* Start a sweep at the CURRENT clock/latency/MR0 over `ncand` DLYB units
 * [ulo .. ulo+(ncand-1)*ustep] on `phase`; persists and system-resets into the
 * boot loop (does not return on success).  Returns 0 on bad params. */
int psram_mmapscan_start(uint32_t phase, uint32_t ulo, uint32_t ustep,
                         uint32_t ncand);

/* Abort a sweep so the next boot comes up normally. */
void psram_mmapscan_stop(void);

/* Copy the sweep state for `psram mmapscan show`.  0 = no sweep recorded,
 * 1 = active, 2 = complete. */
int psram_mmapscan_get(struct psram_scan_state *out);

/* Read-eye scan granularity: DLYB unit is swept 0..(COLS-1)*STEP in STEP-sized
 * columns (issue #16).  32 columns of step 4 span the full 0..124 unit range at
 * 4-unit resolution -- fine enough to locate the passing window's centre. */
#define PSRAM_SCAN_STEP  4u
#define PSRAM_SCAN_COLS  32u

/* Write-latency tuner: sweep mmap write dummy 0..15 with marker write+readback
 * (x`reps` each); returns a pass bitmask (bit d = dummy d round-tripped) and
 * leaves the lowest passing dummy applied (`psram wtune`). */
uint32_t psram_wtune(uint32_t off, uint32_t reps);

#endif /* APP_PSRAM_H */
