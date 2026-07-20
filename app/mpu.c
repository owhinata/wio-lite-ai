/*
 * Wio Lite AI (STM32H725AEI6) -- MPU non-cacheable region setup (issue #3).
 *
 * The app enables the Cortex-M7 D-cache (app/main.c).  By the ARMv7-M default
 * memory map the external OCTOSPI1 PSRAM window at 0x90000000 (0x80000000-
 * 0x9FFFFFFF = "RAM", Normal, Write-Through cacheable) would be cached, so a
 * future bus master (SDMMC / DCMI camera / OCTOSPI DMA) writing PSRAM behind the
 * CPU's back would leave stale D-cache lines -- the coherency hole flagged in the
 * D-cache-enable comment in app/main.c.  This module carves the PSRAM window out
 * as Normal, non-cacheable, shareable so DMA buffers placed there stay coherent
 * with no per-transfer clean/invalidate.  It is the reusable foundation the SD /
 * LCD / camera work (issues #5-#7) builds their DMA buffers on: add a region to
 * mpu_regions[] rather than sprinkling cache maintenance through each driver.
 *
 * Ordering (hard invariant, PM0253 sec 4.6.8 -- a barrier is required after every
 * MPU update): mpu_config() MUST run *between* SCB_EnableICache() and
 * SCB_EnableDCache() (see app/main.c).  PSRAM is not accessed until it is brought
 * up later, so there are never cached PSRAM lines to maintain when the attribute
 * flips from cacheable-default to non-cacheable.
 *
 * PRIVDEFENA=1 keeps the ARMv7-M background default map for everything the table
 * does not cover, so XIP code at 0x70000000 (Normal WBWA cacheable, executable),
 * AXI-SRAM at 0x24000000 (cacheable), the DTCM log ring (TCM, bypasses the
 * D-cache) and the peripheral/PPB windows all keep their default attributes --
 * only the listed regions override.  Unused regions are explicitly cleared
 * (PM0253 sec 4.6.9: a stale region left from a prior configuration can otherwise
 * take effect).
 */
#include "stm32h7xx_hal.h"   /* CMSIS core (ARM_MPU_*, __MPU_PRESENT) + device */
#include "app.h"

/* One MPU region: base, size code (ARM_MPU_REGION_SIZE_*), and its RASR word.
 * Region numbers are the array index; keep the count <= the 16 M7 regions. */
struct mpu_region {
	uint32_t rbar;   /* ARM_MPU_RBAR(region, base) */
	uint32_t rasr;   /* ARM_MPU_RASR(...) attributes + enable */
};

/*
 * v1 region table.  Region 0: the whole 8 MB OCTOSPI1 PSRAM window at 0x90000000
 * as Normal, Non-cacheable, Shareable, full RW, execute-never (DMA/data buffers,
 * not code) -- TEX=001, C=0, B=0, S=1, AP=full, XN=1.  8 MB is a power of two and
 * naturally aligned, so it needs no sub-region masking.  Future DMA carve-outs
 * (camera framebuffer, SD scratch) are added as further rows here.
 */
static const struct mpu_region mpu_regions[] = {
	{
		.rbar = ARM_MPU_RBAR(0u, 0x90000000u),
		.rasr = ARM_MPU_RASR(/*DisableExec*/ 1u, ARM_MPU_AP_FULL,
		                     /*TEX*/ 1u, /*IsShareable*/ 1u,
		                     /*IsCacheable*/ 0u, /*IsBufferable*/ 0u,
		                     /*SubRegionDisable*/ 0u,
		                     ARM_MPU_REGION_SIZE_8MB),
	},
};

#define MPU_REGION_COUNT (sizeof mpu_regions / sizeof mpu_regions[0])

void mpu_config(void)
{
	uint32_t i;

	/* Disable the MPU (DMB inside) before reprogramming its regions. */
	ARM_MPU_Disable();

	/* Clear every hardware region first so no stale entry from a prior config
	 * survives (PM0253 sec 4.6.9), then install the table.  The STM32H7 M7 has
	 * 16 MPU regions. */
	for (i = 0u; i < 16u; i++)
		ARM_MPU_ClrRegion(i);

	for (i = 0u; i < MPU_REGION_COUNT; i++)
		ARM_MPU_SetRegion(mpu_regions[i].rbar, mpu_regions[i].rasr);

	/* Enable with PRIVDEFENA so the default background map covers everything not
	 * in the table; ARM_MPU_Enable() ORs in the ENABLE bit and issues DSB+ISB. */
	ARM_MPU_Enable(MPU_CTRL_PRIVDEFENA_Msk);
}
