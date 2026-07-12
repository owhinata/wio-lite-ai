/*
 * ThreadX low-level glue for Wio Lite AI (STM32H725 / Cortex-M7 / GNU).
 *
 * Adapted from the stm32f746g-disco reference, with the Execution Profile Kit
 * dropped for v1 (see tx_user.h).  Integration choices:
 *  - SysTick is set up by HAL_Init()/HAL_InitTick() (1 ms, reload =
 *    SystemCoreClock/1000 = 550000) and shared with ThreadX: this handler
 *    increments the HAL tick and, once ThreadX is initialized, calls
 *    _tx_timer_interrupt().  So HAL_GetTick() and ThreadX both work.  The app
 *    does NOT reprogram the RCC -- it inherits the bootloader's 550 MHz clock and
 *    OCTOSPI2 XIP map (reprogramming would stall the XIP instruction fetch).
 *  - PendSV (context switch) runs at the lowest priority; SysTick one step higher.
 *    SysTick MUST outrank PendSV: when no thread is ready ThreadX idles by spinning
 *    inside PendSV with interrupts enabled, and SysTick must be able to preempt that
 *    spin to advance the tick -- else sleeping threads never wake (deadlock).
 *  - Critical sections use PRIMASK (TX_PORT_USE_BASEPRI undefined), so an OTG_HS
 *    ISR calling tx_event_flags_set can never preempt a ThreadX critical section.
 *  - ThreadX's PendSV_Handler comes from the port asm, so the app ships no
 *    stm32h7xx_it.c; every other vector falls to the startup weak Default_Handler.
 */
#include "tx_api.h"
#include "stm32h7xx_hal.h"

#include "tx_glue.h"

extern VOID  _tx_timer_interrupt(VOID);
extern VOID *_tx_initialize_unused_memory;

/* The threads in this app own their stacks statically, so ThreadX never needs the
   "first unused memory" region; point it at a tiny valid buffer. */
static UCHAR tx_unused_memory[4];

/* Gate so the SysTick ISR does not poke ThreadX timer lists before they exist
   (SysTick is already ticking from HAL_Init() before tx_kernel_enter()). */
static volatile UINT tx_timer_active = 0u;

void _tx_initialize_low_level(void)
{
    NVIC_SetPriority(PendSV_IRQn, 15);
    NVIC_SetPriority(SysTick_IRQn, 14);

    _tx_initialize_unused_memory = (VOID *)tx_unused_memory;
}

/* Called at the end of tx_application_define(), once the timer lists are set up by
   _tx_initialize_high_level(), to let the SysTick ISR drive ThreadX. */
void tx_glue_timer_enable(void)
{
    tx_timer_active = 1u;
}

/* Number of 1 ms SysTicks per ThreadX tick.  Default 1 (1 kHz ThreadX tick).
   Override with -DTX_GLUE_TICK_DIV; keep in sync with TX_TIMER_TICKS_PER_SECOND. */
#ifndef TX_GLUE_TICK_DIV
#define TX_GLUE_TICK_DIV 1u
#endif

void SysTick_Handler(void)
{
    HAL_IncTick();   /* HAL timebase stays at 1 ms */

    if (tx_timer_active != 0u)
    {
        static UINT div = 0u;
        if (++div >= (UINT)TX_GLUE_TICK_DIV)
        {
            div = 0u;
            _tx_timer_interrupt();
        }
    }
}
