/*
 * CoreMark port for Wio Lite AI (STM32H725AEI6) -- run as the shell `coremark`
 * command.
 *  - Timing : 1 ms SysTick via HAL_GetTick() (EE_TICKS_PER_SEC == 1000); the
 *             SysTick that drives ThreadX also feeds HAL_IncTick (tx_glue.c).
 *  - Output : printf over USB CDC, retargeted by the shell backend's _write.
 * Derived from EEMBC's barebones core_portme.c (Apache-2.0).
 */
#include "coremark.h"
#include "stm32h7xx_hal.h"

#ifndef ITERATIONS
#define ITERATIONS 0   /* 0 -> CoreMark auto-calibrates to run 10..100 s */
#endif

/* HAL_GetTick() counts milliseconds. */
#define EE_TICKS_PER_SEC 1000

#if (SEED_METHOD == SEED_VOLATILE)
/* Performance run seeds (0, 0, 0x66) -> canonical CoreMark score. */
volatile ee_s32 seed1_volatile = 0x0;
volatile ee_s32 seed2_volatile = 0x0;
volatile ee_s32 seed3_volatile = 0x66;
volatile ee_s32 seed4_volatile = ITERATIONS;
volatile ee_s32 seed5_volatile = 0;
#endif

static CORETIMETYPE start_time_val, stop_time_val;

void start_time(void)
{
    start_time_val = HAL_GetTick();
}

void stop_time(void)
{
    stop_time_val = HAL_GetTick();
}

CORE_TICKS get_time(void)
{
    return (CORE_TICKS)(stop_time_val - start_time_val);
}

secs_ret time_in_secs(CORE_TICKS ticks)
{
    return ((secs_ret)ticks) / (secs_ret)EE_TICKS_PER_SEC;
}

ee_u32 default_num_contexts = 1;

void portable_init(core_portable *p, int *argc, char *argv[])
{
    (void)argc;
    (void)argv;

    /* The board (550 MHz clock inherited from the bootloader, I-cache, USB CDC,
     * printf) is already up: main() ran before the kernel started, so this port
     * must NOT re-init it (a second init would disturb the live shell / clock). */

    if (sizeof(ee_ptr_int) != sizeof(ee_u8 *))
    {
        ee_printf(
            "ERROR! Please define ee_ptr_int to a type that holds a "
            "pointer!\n");
    }
    if (sizeof(ee_u32) != 4)
    {
        ee_printf("ERROR! Please define ee_u32 to a 32b unsigned type!\n");
    }
    p->portable_id = 1;
}

void portable_fini(core_portable *p)
{
    p->portable_id = 0;
}
