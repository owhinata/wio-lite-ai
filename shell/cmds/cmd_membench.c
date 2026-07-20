/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    cmd_membench.c
 * @brief   `membench` shell command: memory bandwidth + latency at cycle precision.
 *
 * A self-contained micro-benchmark (ported from the stm32f746g-disco `membench`,
 * itself borrowing only the two core ideas of lmbench -- bw_mem sequential
 * read/write/copy bandwidth and lat_mem_rd pointer-chase latency) that measures
 * the physical memories of the Wio Lite AI with the Cortex-M7 DWT cycle counter.
 *
 * Wio adaptations vs the f746 donor:
 *  - Clock is SystemCoreClock (550 MHz, the CPU/DWT clock), NOT HAL_RCC_GetHCLKFreq()
 *    (which returns the D2/AHB clock 275 MHz on this part -- DWT CYCCNT counts at the
 *    core clock, so HCLK would halve every result).
 *  - No SDRAM (the board has none).  Both L1 caches are on (app/main.c): the tiny
 *    measurement loops are I-cached, and the D-cache (16 KB, 32 B line, 4-way = 512
 *    lines) is visible in the AXI-SRAM latency rows -- a 4 KB working set fits the
 *    cache (L1-hit rate) while a 64 KB set overflows it (AXI refill rate).  The
 *    refill set must exceed 512 lines: at the 64 B chase stride a 32 KB set touches
 *    exactly 512 lines and still fits, so 64 KB (1024 lines) is used.  Sequential
 *    *bandwidth* stays high even out-of-cache (burst line-refills), so the cache
 *    step shows mainly in the dependent-load *latency* rows.  DTCM is TCM (bypasses
 *    the D-cache, always the raw rate) and Flash is the read-only XIP window.
 *  - Regions: DTCM (4 KB, .dtcm_bench), AXI-SRAM (64 KB, malloc'd on demand),
 *    Flash int = embedded flash via AXIM 0x08000000, Flash ext = OCTOSPI2 XIP window
 *    0x70000000 (both read-only: measure the flash read rate; a read of the boot
 *    region is harmless -- no write/erase, so no brick risk), PSRAM = OCTOSPI1
 *    APS6408 mmap window 0x90000000 (writable scratch, MPU non-cacheable ->
 *    raw octal-DTR rate; skipped when the bring-up failed).
 *
 * Timing: DWT CYCCNT.  Each timed run is sized to ~0.3 ms (< one 1 kHz SysTick
 * period) via a calibration pass, run up to MEMBENCH_TRIALS times; runs during
 * which the millisecond tick advanced (a SysTick ISR fired) are rejected, and the
 * minimum over the tick-clean runs is reported.  Interrupts are never disabled.
 * Cancel with Ctrl+C between cells.
 *
 * DCE/line-reuse defeat: reads go through `const volatile`, a volatile sink ends
 * each loop, and the latency walk is a dependent load chain `idx = buf[idx]` over
 * word indices (each load address depends on the previous result).
 *
 * A singleton guard rejects a second concurrent run (`membench &` twice): the
 * shared static DTCM buffer / sink would otherwise be raced.  Linked into the
 * shell firmware only.  Clean-room glue.
 */
#include "cli.h"

#include "stm32h7xx_hal.h"   /* DWT/CoreDebug/SCB, SystemCoreClock, HAL_GetTick, __DSB/__ISB */

#if BSP_ENABLE_PSRAM
#include "psram.h"           /* PSRAM_BASE_ADDR + psram_ready() (issue #3) */
#endif

#include <stdint.h>
#include <stdlib.h>          /* malloc / free for the on-demand SRAM buffer */
#include <stdio.h>           /* snprintf for table cells */
#include <string.h>          /* strcpy */

/* Per-region benchmark buffers.  DTCM needs a dedicated NOLOAD section (the region
 * otherwise holds only the log ring); it is the only permanently-reserved bench
 * buffer (malloc returns AXI-SRAM, never DTCM, so a DTCM-resident buffer must be
 * static).  Kept to 4 KB (issue #14): DTCM is uncached TCM with no cache cliff, so a
 * small working set measures the same raw bandwidth/latency as a large one, and it
 * matches SRAM_CACHED_BYTES for an apples-to-apples DTCM-raw vs SRAM-L1-hit compare.
 * The SRAM buffer is malloc'd on demand and freed on exit rather than permanently
 * reserving .bss for a rarely-run command. */
#define DTCM_BENCH_BYTES   ( 4u * 1024u)
#define SRAM_BENCH_BYTES   (64u * 1024u)   /* > 512 D-cache lines @64B stride -> refill */
#define SRAM_CACHED_BYTES  ( 4u * 1024u)   /* fits in the 16 KB L1 D-cache */
#define FLASH_BENCH_BYTES  (64u * 1024u)
#define PSRAM_BENCH_BYTES  (64u * 1024u)   /* scratch at PSRAM base (non-cacheable mmap) */
#define EFLASH_BENCH_BASE  0x70000000u   /* external: OCTOSPI2 XIP window (read-only) */
#define IFLASH_BENCH_BASE  0x08000000u   /* internal: embedded flash via AXIM (read-only).
                                          * This is the DFU bootloader's region, but a READ
                                          * is harmless -- no write/erase, so no brick risk;
                                          * measured only for the int-vs-ext comparison. */

static uint32_t dtcm_bench_buf[DTCM_BENCH_BYTES / 4]
	__attribute__((aligned(32), section(".dtcm_bench")));

/* Volatile sink: every measured loop ends by storing into it, so -O2/-O3 cannot
 * eliminate the loop as dead code. */
static volatile uint32_t g_sink;

/* Harness tuning. */
#define MEMBENCH_TRIALS    16u    /* attempts to find tick-clean runs           */
#define MEMBENCH_CLEAN      3u    /* stop once this many clean runs are seen     */
#define MEMBENCH_MAX_ITERS 100000u
#define MEMBENCH_TARGET_DIV 3333u /* clk / 3333 ~= 0.3 ms of cycles per run      */

typedef void (*work_fn)(void *ctx);

/* ---- reentrancy guard ------------------------------------------------------- */

static volatile uint8_t membench_busy;

static int membench_try_acquire(void)
{
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	int ok = !membench_busy;
	if (ok)
		membench_busy = 1u;
	__set_PRIMASK(pm);
	return ok;
}

/* ---- DWT cycle counter ------------------------------------------------------ */

/* DWT Lock Access Register (0xE0001FB0) + unlock key.  On Cortex-M7 the DWT has a
 * software lock that, with no debugger attached, leaves CYCCNT frozen at 0 even
 * after TRCENA + CYCCNTENA are set; writing the key unlocks register access.  Not
 * all CMSIS core_cm7.h revisions expose DWT->LAR, so the absolute address is used. */
#define DWT_LAR_ADDR  0xE0001FB0u
#define DWT_LAR_KEY   0xC5ACCE55u

static int dwt_enable(void)
{
	int attempt;

	if (DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk)
		return -1;                  /* CYCCNT not implemented on this core */

	/* Enable, then self-test that the counter actually advances; retry a few
	 * times re-asserting trace-enable + the DWT unlock.  If it never advances,
	 * abort cleanly rather than let calibration see a zero delta -> a hang.
	 *
	 * Never write DWT->CYCCNT: it is the shared free-running timebase that
	 * svc/timebase.c's udelay() (the `usleep`/`sleep` busy-wait) reads, so
	 * zeroing it here would corrupt a concurrent `usleep N &`'s elapsed delta and
	 * end its wait early.  Test advancement with a wrap-safe delta from a sampled
	 * value instead (CYCCNTENA is idempotent; timebase_init already enabled it). */
	for (attempt = 0; attempt < 3; attempt++) {
		volatile uint32_t spin = 64u;
		uint32_t a;

		CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
		*((volatile uint32_t *)DWT_LAR_ADDR) = DWT_LAR_KEY;   /* unlock (M7) */
		DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

		a = DWT->CYCCNT;
		while (spin--)
			__NOP();
		if ((uint32_t)(DWT->CYCCNT - a) != 0u)
			return 0;           /* counting */
	}
	return -1;
}

/* ---- timed harness ---------------------------------------------------------- */

/* Pick an iteration count for `fn` so one run is ~target cycles (~0.3 ms). */
static void calibrate(work_fn fn, void *ctx, uint32_t *iter, uint32_t init,
                      uint32_t target)
{
	uint32_t c0, c1, total, per, it;

	*iter = init;
	__DSB(); __ISB();
	c0 = DWT->CYCCNT;
	fn(ctx);
	__DSB(); __ISB();
	c1 = DWT->CYCCNT;
	total = c1 - c0;
	if (total < init) {     /* counter not advancing -> keep reps at 1 (anti-hang) */
		*iter = 1u;
		return;
	}
	per = total / init;     /* now per >= 1 */
	it = target / per;
	if (it < 1u)
		it = 1u;
	if (it > MEMBENCH_MAX_ITERS)
		it = MEMBENCH_MAX_ITERS;
	*iter = it;
}

/* Run fn (warm-up discarded) and return the min cycle count over tick-clean runs;
 * falls back to min-of-all if no clean run is found. */
static uint32_t timed_min(work_fn fn, void *ctx)
{
	uint32_t best_clean = 0xFFFFFFFFu, best_any = 0xFFFFFFFFu;
	uint32_t clean = 0u, i;

	fn(ctx);                            /* warm-up -> discard */
	for (i = 0u; i < MEMBENCH_TRIALS && clean < MEMBENCH_CLEAN; i++) {
		uint32_t t0 = HAL_GetTick();
		uint32_t c0, c1, t1, dc;

		__DSB(); __ISB();
		c0 = DWT->CYCCNT;
		fn(ctx);
		__DSB(); __ISB();
		c1 = DWT->CYCCNT;
		t1 = HAL_GetTick();

		dc = c1 - c0;
		if (dc < best_any)
			best_any = dc;
		if (t1 == t0) {                 /* no SysTick fired across the run */
			clean++;
			if (dc < best_clean)
				best_clean = dc;
		}
	}
	return clean ? best_clean : best_any;
}

/* ---- bandwidth (bw_mem) ----------------------------------------------------- */

struct bw_ctx {
	const volatile uint32_t *src;
	volatile uint32_t       *dst;
	uint32_t                 words;   /* words touched per scan */
	uint32_t                 reps;    /* scans per timed run    */
};

static void bw_read_work(void *vctx)
{
	struct bw_ctx *c = (struct bw_ctx *)vctx;
	const volatile uint32_t *p = c->src;
	uint32_t words = c->words, reps = c->reps, r, i, acc = 0u;

	for (r = 0u; r < reps; r++) {
		for (i = 0u; i + 8u <= words; i += 8u) {
			acc += p[i + 0] + p[i + 1] + p[i + 2] + p[i + 3];
			acc += p[i + 4] + p[i + 5] + p[i + 6] + p[i + 7];
		}
		for (; i < words; i++)
			acc += p[i];
	}
	g_sink = acc;
}

static void bw_write_work(void *vctx)
{
	struct bw_ctx *c = (struct bw_ctx *)vctx;
	volatile uint32_t *p = c->dst;
	uint32_t words = c->words, reps = c->reps, r, i;
	uint32_t v = g_sink + 0x9E3779B9u;   /* derive from volatile -> not const-folded */

	for (r = 0u; r < reps; r++) {
		for (i = 0u; i + 8u <= words; i += 8u) {
			p[i + 0] = v; p[i + 1] = v; p[i + 2] = v; p[i + 3] = v;
			p[i + 4] = v; p[i + 5] = v; p[i + 6] = v; p[i + 7] = v;
		}
		for (; i < words; i++)
			p[i] = v;
	}
}

static void bw_copy_work(void *vctx)
{
	struct bw_ctx *c = (struct bw_ctx *)vctx;
	const volatile uint32_t *s = c->src;
	volatile uint32_t *d = c->dst;
	uint32_t words = c->words, reps = c->reps, r, i;

	for (r = 0u; r < reps; r++)
		for (i = 0u; i < words; i++)
			d[i] = s[i];
}

static uint32_t bw_mbps(uint32_t cycles, uint64_t bytes, uint32_t clk)
{
	if (cycles == 0u)
		return 0u;
	return (uint32_t)((bytes * (uint64_t)clk) / ((uint64_t)cycles * 1000000ULL));
}

/* ---- latency (lat_mem_rd) --------------------------------------------------- */

struct lat_ctx {
	const volatile uint32_t *buf;
	uint32_t                 k;    /* chase accesses per timed run */
};

/* Build a single-cycle chase over the working set as WORD INDICES: nodes are 64 B
 * = 16 words apart, linked i -> i+1 (mod n).  buf[node*16] holds the next node's
 * word index.  Deterministic, reproducible. */
static void build_chase(uint32_t *buf, uint32_t wss_bytes)
{
	uint32_t n = wss_bytes / 64u, i;

	if (n == 0u)
		n = 1u;
	for (i = 0u; i < n; i++)
		buf[i * 16u] = ((i + 1u) % n) * 16u;   /* next node's word index */
}

static void lat_work(void *vctx)
{
	struct lat_ctx *c = (struct lat_ctx *)vctx;
	const volatile uint32_t *b = c->buf;
	uint32_t idx = 0u, k = c->k;

	while (k--)                          /* each load address depends on the */
		idx = b[idx];               /* previous result -> serialized chain */
	g_sink = idx;
}

/* Tenths of a nanosecond per access: cycles/k accesses at clk. */
static uint32_t lat_ns_tenths(uint32_t cycles, uint32_t k, uint32_t clk)
{
	if (k == 0u || clk == 0u)
		return 0u;
	return (uint32_t)(((uint64_t)cycles * 10000000000ULL) / ((uint64_t)clk * k));
}

/* ---- formatting / rows ------------------------------------------------------ */

static void fmt_u(char *buf, size_t n, uint32_t v)
{
	snprintf(buf, n, "%lu", (unsigned long)v);
}

static void fmt_ns(char *buf, size_t n, uint32_t tenths)
{
	snprintf(buf, n, "%lu.%lu", (unsigned long)(tenths / 10u),
	         (unsigned long)(tenths % 10u));
}

/* One bandwidth row: read always; write/copy only when `writable` (Flash is RO).
 * read/write span `words`; copy moves the first half into the second half. */
static void bw_row(struct cli_instance *sh, const char *label, uint32_t *base,
                   uint32_t words, uint32_t clk, int writable)
{
	struct bw_ctx c;
	char rds[12], wrs[12], cps[12];
	uint32_t target = clk / MEMBENCH_TARGET_DIV;

	c.src = base; c.dst = base; c.words = words; c.reps = 1u;
	calibrate(bw_read_work, &c, &c.reps, 1u, target);
	fmt_u(rds, sizeof rds,
	      bw_mbps(timed_min(bw_read_work, &c), (uint64_t)words * 4u * c.reps, clk));

	if (writable) {
		uint32_t half = words / 2u;

		c.src = base; c.dst = base; c.words = words; c.reps = 1u;
		calibrate(bw_write_work, &c, &c.reps, 1u, target);
		fmt_u(wrs, sizeof wrs,
		      bw_mbps(timed_min(bw_write_work, &c), (uint64_t)words * 4u * c.reps, clk));

		c.src = base; c.dst = base + half; c.words = half; c.reps = 1u;
		calibrate(bw_copy_work, &c, &c.reps, 1u, target);
		fmt_u(cps, sizeof cps,
		      bw_mbps(timed_min(bw_copy_work, &c), (uint64_t)half * 4u * c.reps, clk));
	} else {
		strcpy(wrs, "--");
		strcpy(cps, "--");
	}

	cli_print(sh, "  %-22s %8s %8s %8s\r\n", label, rds, wrs, cps);
}

/* One latency cell over the requested working set (small set = L1-hit, large set
 * = refill, on the cacheable AXI-SRAM; DTCM/TCM is always the raw rate). */
static uint32_t lat_ns10(uint32_t *buf, uint32_t wss_bytes, uint32_t clk)
{
	struct lat_ctx c;
	uint32_t target = clk / MEMBENCH_TARGET_DIV;

	build_chase(buf, wss_bytes);
	c.buf = buf;
	c.k = 256u;
	calibrate(lat_work, &c, &c.k, 256u, target);
	return lat_ns_tenths(timed_min(lat_work, &c), c.k, clk);
}

/* ---- command ---------------------------------------------------------------- */

static int cmd_membench(struct cli_instance *sh, int argc, char **argv)
{
	int do_dtcm = 1, do_sram = 1, do_flash = 1, do_psram = 1;
	uint32_t clk;
	void     *sram_raw = NULL;         /* malloc base (freed on every exit via `done`) */
	uint32_t *sram_bench_buf = NULL;   /* 32-byte-aligned working pointer */
	int rc = 0;

	if (argc >= 2) {
		const char *r = argv[1];

		do_dtcm = do_sram = do_flash = do_psram = 0;
		if (!strcmp(r, "all"))        do_dtcm = do_sram = do_flash = do_psram = 1;
		else if (!strcmp(r, "dtcm"))  do_dtcm = 1;
		else if (!strcmp(r, "sram"))  do_sram = 1;
		else if (!strcmp(r, "flash")) do_flash = 1;
		else if (!strcmp(r, "psram")) do_psram = 1;
		else {
			cli_error(sh, "membench: unknown region '%s' (dtcm|sram|flash|psram|all)\r\n", r);
			return 1;
		}
	}

	if (!membench_try_acquire()) {
		cli_error(sh, "membench: already running\r\n");
		return 1;
	}

	if (dwt_enable() != 0) {
		cli_error(sh, "membench: DWT CYCCNT unavailable on this core\r\n");
		rc = 1;
		goto done;
	}
	clk = SystemCoreClock;             /* DWT CYCCNT counts at the CPU clock (550 MHz) */

	/* On-demand SRAM buffer: malloc'd from the heap (which lives in AXI-SRAM, the
	 * region we want to measure) instead of a permanent .bss array.  Over-allocate
	 * by 32 for a cache-line-aligned pointer.  On failure, skip the SRAM rows. */
	if (do_sram) {
		sram_raw = malloc(SRAM_BENCH_BYTES + 32u);
		if (sram_raw == NULL) {
			cli_warn(sh, "membench: no heap for the 64 KB SRAM buffer; "
			             "skipping SRAM rows\r\n");
			do_sram = 0;
		} else {
			sram_bench_buf = (uint32_t *)
				(((uintptr_t)sram_raw + 31u) & ~(uintptr_t)31u);
		}
	}

	cli_print(sh, "DWT CYCCNT @%luMHz; warm-up + tick-guarded min; I-cache + D-cache on "
	          "(DTCM=TCM uncached; SRAM via 16KB L1 D$; Flash int=embedded/AXIM, "
	          "ext=OCTOSPI2/XIP).\r\n\r\n", (unsigned long)(clk / 1000000u));

	/* bandwidth table */
	cli_print(sh, "%-24s %8s %8s %8s\r\n", "bandwidth (MB/s)", "read", "write", "copy");
	if (do_dtcm) {
		if (cli_cancel_requested(sh)) goto done;
		bw_row(sh, "DTCM   ( 4KB)", dtcm_bench_buf, DTCM_BENCH_BYTES / 4u, clk, 1);
	}
	if (do_sram) {
		if (cli_cancel_requested(sh)) goto done;
		bw_row(sh, "SRAM   ( 4KB, cached)", sram_bench_buf, SRAM_CACHED_BYTES / 4u, clk, 1);
		bw_row(sh, "SRAM   (64KB, refill)", sram_bench_buf, SRAM_BENCH_BYTES / 4u, clk, 1);
	}
#if BSP_ENABLE_PSRAM
	/* PSRAM is scratch RAM (writable) behind the MPU non-cacheable window, so
	 * both directions measure the raw OCTOSPI1 rate -- no cache step. */
	if (do_psram) {
		if (cli_cancel_requested(sh)) goto done;
		/* Hold the OCTOSPI1 guard for the duration of the mmap benchmark so a
		 * concurrent `psram` command cannot re-enter mmap / abort mid-access
		 * (would corrupt the numbers or stall the AXI bus). */
		if (!psram_ready())
			cli_warn(sh, "membench: PSRAM not ready; skipping\r\n");
		else if (!psram_acquire())
			cli_warn(sh, "membench: PSRAM busy (a psram command holds it); skipping\r\n");
		else {
			bw_row(sh, "PSRAM  (64KB)", (uint32_t *)PSRAM_BASE_ADDR,
			       PSRAM_BENCH_BYTES / 4u, clk, 1);
			psram_release();
		}
	}
#endif
	if (do_flash) {
		if (cli_cancel_requested(sh)) goto done;
		bw_row(sh, "Flash  int (64KB)", (uint32_t *)IFLASH_BENCH_BASE,
		       FLASH_BENCH_BYTES / 4u, clk, 0);
		if (cli_cancel_requested(sh)) goto done;
		bw_row(sh, "Flash  ext (64KB)", (uint32_t *)EFLASH_BENCH_BASE,
		       FLASH_BENCH_BYTES / 4u, clk, 0);
	}

	/* latency vs. working-set size (pointer-chase; Flash is read-only, no row).
	 * One row per size, one column per region -- the cache cliff shows as the
	 * SRAM column jumping from the L1-hit rate to the AXI-refill rate once the
	 * set overflows the 16 KB D-cache, while DTCM (TCM, uncached) and PSRAM
	 * (non-cacheable) stay flat.  DTCM stops at its 4 KB buffer. */
	{
		static const uint32_t sizes_kb[] = { 1u, 2u, 4u, 8u, 16u, 32u, 64u };
		size_t si;
		int lat_psram = 0;

#if BSP_ENABLE_PSRAM
		if (do_psram && psram_ready())
			lat_psram = psram_acquire();   /* held across the whole column */
#endif
		if (do_dtcm || do_sram || lat_psram) {
			cli_print(sh, "\r\nlatency (ns/access, pointer-chase, 64B stride)\r\n");
			cli_print(sh, "  %6s", "wss");
			if (do_dtcm)   cli_print(sh, " %8s", "DTCM");
			if (do_sram)   cli_print(sh, " %8s", "SRAM");
			if (lat_psram) cli_print(sh, " %8s", "PSRAM");
			cli_print(sh, "\r\n");

			for (si = 0u; si < sizeof sizes_kb / sizeof sizes_kb[0]; si++) {
				uint32_t wss = sizes_kb[si] * 1024u;
				char cell[12];

				if (cli_cancel_requested(sh)) {
#if BSP_ENABLE_PSRAM
					if (lat_psram) psram_release();
#endif
					goto done;
				}
				cli_print(sh, "  %4lu KB", (unsigned long)sizes_kb[si]);
				if (do_dtcm) {
					if (wss <= DTCM_BENCH_BYTES) {
						fmt_ns(cell, sizeof cell,
						       lat_ns10(dtcm_bench_buf, wss, clk));
						cli_print(sh, " %8s", cell);
					} else
						cli_print(sh, " %8s", "--");
				}
				if (do_sram) {
					fmt_ns(cell, sizeof cell,
					       lat_ns10(sram_bench_buf, wss, clk));
					cli_print(sh, " %8s", cell);
				}
#if BSP_ENABLE_PSRAM
				if (lat_psram) {
					fmt_ns(cell, sizeof cell,
					       lat_ns10((uint32_t *)PSRAM_BASE_ADDR, wss, clk));
					cli_print(sh, " %8s", cell);
				}
#endif
				cli_print(sh, "\r\n");
			}
		}
#if BSP_ENABLE_PSRAM
		if (lat_psram) psram_release();
#endif
	}

done:
	free(sram_raw);        /* NULL when SRAM was not benched (free(NULL) is a no-op) */
	membench_busy = 0u;    /* single cleanup point: guard cleared on every exit */
	return rc;
}

CLI_CMD_REGISTER(membench, NULL, "memory bandwidth + latency benchmark",
                 cmd_membench, 1, 1);
