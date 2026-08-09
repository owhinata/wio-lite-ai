/*
 * Wio Lite AI (STM32H725AEI6) -- MPU non-cacheable region setup (issue #3).
 *
 * The app enables the Cortex-M7 D-cache (app/main.c).  By the ARMv7-M default
 * memory map the external OCTOSPI1 PSRAM window at 0x90000000 (0x80000000-
 * 0x9FFFFFFF = "RAM", Normal, Write-Through cacheable) would be cached, so a bus
 * master (DCMI camera / OCTOSPI DMA) writing PSRAM behind the CPU's back would
 * leave stale D-cache lines.  This module carves the PSRAM window out as Normal,
 * non-cacheable, shareable so DMA buffers placed there stay coherent with no
 * per-transfer clean/invalidate.
 *
 * That premise is no longer hypothetical: issue #6 enabled the SDMMC1 IDMA, the
 * first bus master in this firmware.  It deliberately does NOT use a region here --
 * its 4 KB bounce buffer sits in ordinary cacheable AXI-SRAM and port/sd/sd_card.c
 * cleans/invalidates around each transfer, because a small scratch buffer does not
 * justify one of the 16 MPU regions.  Use this table for buffers that are large,
 * long-lived, or touched at a rate where per-transfer maintenance would hurt (an
 * LCD/camera framebuffer); use explicit maintenance for small scratch.
 *
 * Ordering (hard invariant, PM0253 sec 4.6.8 -- a barrier is required after every
 * MPU update): mpu_config() MUST run *between* SCB_EnableICache() and
 * SCB_EnableDCache() (see app/main.c).  PSRAM is not accessed until it is brought
 * up later, so there are never cached PSRAM lines to maintain when the attribute
 * flips from cacheable-default to non-cacheable.
 *
 * PRIVDEFENA=1 keeps the ARMv7-M background default map for everything the table
 * does not cover, so app code in the internal flash at 0x08020000 (Normal
 * cacheable, executable), AXI-SRAM at 0x24000000 (cacheable), the DTCM log ring
 * (TCM, bypasses the D-cache) and the peripheral/PPB windows all keep their
 * default attributes -- only the listed regions override.  Unused regions are
 * explicitly cleared (PM0253 sec 4.6.9: a stale region left from a prior
 * configuration can otherwise take effect).
 */
#include "stm32h7xx_hal.h"   /* CMSIS core (ARM_MPU_*, __MPU_PRESENT) + device */
#include "app.h"
#include "psram.h"           /* PSRAM_AI_BASE_ADDR / PSRAM_AI_BYTES (issue #9) */

/* One MPU region: base, size code (ARM_MPU_REGION_SIZE_*), and its RASR word.
 * Region numbers are the array index; keep the count <= the 16 M7 regions. */
struct mpu_region {
	uint32_t rbar;   /* ARM_MPU_RBAR(region, base) */
	uint32_t rasr;   /* ARM_MPU_RASR(...) attributes + enable */
};

/*
 * Region table.  Region 0: the whole 8 MB OCTOSPI1 PSRAM window at 0x90000000
 * as Normal, Non-cacheable, Shareable, full RW, execute-never (DMA/data buffers,
 * not code) -- TEX=001, C=0, B=0, S=1, AP=full, XN=1.  8 MB is a power of two and
 * naturally aligned, so it needs no sub-region masking.  Future DMA carve-outs
 * (camera framebuffer, SD scratch) are added as further rows here.
 *
 * Region 1: the 64 KB ITCM at 0x00000000, read-only but executable.  Since issue
 * #24 the interrupt paths live there (.itcm), and ITCM sits at address zero -- so
 * a stray write through a NULL pointer would overwrite the PendSV/SysTick/UART ISR
 * code and the failure would surface later as unexplained chaos.  Read-only turns
 * that into an immediate MemManage fault, which app/fault.c records to the
 * reset-persistent log so `dmesg` names the culprit PC.  Nothing writes ITCM after
 * boot: SystemInit() loads it before mpu_config() runs, and no code self-modifies.
 * The MPU is unified and applies the same region settings to instruction and data
 * accesses (PM0253 sec 4.6), and accesses to 0x00000000-0x1FFFFFFF go to the ITCM
 * interface (sec 2.3.3), so this covers ITCM fetches too: AP=read-only grants the
 * read permission an instruction fetch needs, and XN=0 leaves execution allowed.
 * The memory type (TEX=001/C=0/B=0, matching region 0's Normal non-cacheable
 * encoding) is effectively moot -- a TCM is never cached -- so this row exists
 * purely for the permission bits.
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
	{
		.rbar = ARM_MPU_RBAR(1u, 0x00000000u),
		.rasr = ARM_MPU_RASR(/*DisableExec*/ 0u, ARM_MPU_AP_RO,
		                     /*TEX*/ 1u, /*IsShareable*/ 0u,
		                     /*IsCacheable*/ 0u, /*IsBufferable*/ 0u,
		                     /*SubRegionDisable*/ 0u,
		                     ARM_MPU_REGION_SIZE_64KB),
	},
	/*
	 * Region 2: the external OCTOSPI2 window at 0x70000000, fenced off (issue #25).
	 *
	 * The app used to execute in place from here; it now runs from the internal
	 * flash, so nothing memory-maps this window.  In the ARMv7-M background map
	 * 0x60000000-0x7FFFFFFF is Normal memory, which the core is allowed to access
	 * speculatively -- and an access to an OCTOSPI that is not in memory-mapped
	 * mode either stalls the AXI read indefinitely or returns a slave error
	 * (RM0468 sec 25.4.16).  An indefinite stall is the bad one: it looks like a
	 * lockup with no evidence, and only the IWDG gets the board back.
	 *
	 * No-access + execute-never turns any such access -- speculative or a stale
	 * pointer someone forgot to update -- into an immediate MemManage fault, which
	 * app/fault.c records to the reset-persistent log so `dmesg` names the faulting
	 * PC.  256 MB is the whole 0x70000000-0x7FFFFFFF quarter and is naturally
	 * aligned, so it needs no sub-region masking.
	 *
	 * THIS ROW STAYS EVEN THOUGH OCTOSPI2 IS BROUGHT UP AGAIN (issue #37).  An
	 * earlier revision of this comment said to remove it in that case; that was
	 * wrong, and removing it is precisely the mistake it would cause.  port/nor
	 * drives the device entirely through indirect transactions and never enters the
	 * memory-mapped window, so there is nothing behind this address range to
	 * un-fence -- while opening it up would restore exactly the speculative-read
	 * hazard above, which no lock can arbitrate because the CPU never asked for the
	 * access.  A window here needs its own MPU region designed alongside it, which
	 * is the blob-storage work left on issue #10, not a line deleted from this one.
	 */
	{
		.rbar = ARM_MPU_RBAR(2u, 0x70000000u),
		.rasr = ARM_MPU_RASR(/*DisableExec*/ 1u, ARM_MPU_AP_NONE,
		                     /*TEX*/ 1u, /*IsShareable*/ 0u,
		                     /*IsCacheable*/ 0u, /*IsBufferable*/ 0u,
		                     /*SubRegionDisable*/ 0u,
		                     ARM_MPU_REGION_SIZE_256MB),
	},
	/*
	 * Region 3: the top 2 MB of the PSRAM window, Normal cacheable write-back
	 * write-allocate -- TEX=001, C=1, B=1, S=0, AP=full, XN=1 (PM0253 sec 4.6.6).
	 * This is the first carve-out the region 0 comment above anticipated, and it
	 * overrides region 0 in the overlap because the MPU resolves overlapping
	 * regions in favour of the HIGHEST region number (PM0253 sec 4.6).
	 *
	 * WHY.  Issue #9's inference runtime keeps its activations, input staging and
	 * model slots here, and they are far too large for AXI-SRAM.  Through the
	 * non-cacheable window that working set is bound by one bus transaction per
	 * access rather than by bandwidth: the phase 1 stub measured ~9.8 core cycles
	 * for every byte-wide access, against the 113 MB/s the same window streams.
	 * Cache line fills amortise 32 bytes per transaction, which is the whole point.
	 *
	 * WHY SHAREABLE IS 0 HERE AND 1 IN REGION 0, WHICH IS NOT A TYPO.  Region 0 is
	 * shared with the LTDC, the DMA2D and the DCMI, and being non-cacheable is what
	 * makes that safe.  Nothing but the CPU may touch THIS region -- that is the
	 * precondition for caching it at all -- and on a Cortex-M7 leaving S=1 would
	 * defeat the carve-out outright: Normal Cacheable Shared memory is not held in
	 * the data cache by default, and only CACR.SIWT changes how it is treated at all
	 * (PM0253 sec 4.9.3).  A shareable "cacheable" region would therefore be exactly
	 * as slow as what it replaced, while looking configured.
	 *
	 * WHY NO CACHE MAINTENANCE IS NEEDED WHEN THE ATTRIBUTE FLIPS.  Same argument as
	 * region 0's, and it depends on the ordering the file header states: mpu_config()
	 * runs before SCB_EnableDCache() and long before app/psram.c brings the PSRAM up,
	 * so at the moment this range becomes cacheable there is nothing cached from it.
	 *
	 * XN stays set.  Only data lives here -- issue #9 deliberately rejected the
	 * relocatable-network backend that would have executed from external memory --
	 * so the window keeps W^X.
	 *
	 * 2 MB at 0x90600000 is a power of two at a natural boundary and reaches exactly
	 * the top of the 8 MB window, so like the rows above it needs no sub-region
	 * masking.  ldscript/STM32H725AEIx_IROM.ld asserts .psram_ai starts at the same
	 * address, and cmake/check_psram_ai_residency.py asserts both that the NN buffers
	 * are inside it and that no bus-master buffer ever lands in it.
	 */
	{
		.rbar = ARM_MPU_RBAR(3u, PSRAM_AI_BASE_ADDR),
		.rasr = ARM_MPU_RASR(/*DisableExec*/ 1u, ARM_MPU_AP_FULL,
		                     /*TEX*/ 1u, /*IsShareable*/ 0u,
		                     /*IsCacheable*/ 1u, /*IsBufferable*/ 1u,
		                     /*SubRegionDisable*/ 0u,
		                     ARM_MPU_REGION_SIZE_2MB),
	},
};

_Static_assert(PSRAM_AI_BYTES == 0x00200000u,
               "MPU region 3 is coded ARM_MPU_REGION_SIZE_2MB; keep PSRAM_AI_BYTES in step");
_Static_assert((PSRAM_AI_BASE_ADDR & (PSRAM_AI_BYTES - 1u)) == 0u,
               "an ARMv7-M MPU region must be naturally aligned to its own size");
_Static_assert(PSRAM_AI_BASE_ADDR + PSRAM_AI_BYTES == PSRAM_BASE_ADDR + PSRAM_SIZE_BYTES,
               "the carve-out is meant to reach exactly the top of the PSRAM window");

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
