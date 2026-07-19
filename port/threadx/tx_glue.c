/*
 * ThreadX low-level glue for Wio Lite AI (STM32H725 / Cortex-M7 / GNU).
 *
 * Adapted from the stm32f746g-disco reference.  Integration choices:
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

#ifdef TX_EXECUTION_PROFILE_ENABLE
/* Gate for the Execution Profile Kit ISR hooks (issue #2).  Separate from
   tx_timer_active: the isr enter/exit calls must not run before the kit's state is
   set up by _tx_execution_initialize() (called after tx_application_define(), just
   before the scheduler starts).  Armed by tx_glue_profile_enable() from a thread
   entry, so it flips 0->1 only in thread context -- an ISR runs to completion
   without the thread resuming, so a single ISR invocation always sees one value and
   its enter/exit stay balanced. */
static volatile UINT profile_active = 0u;

/* Start TIM2 as the free-running 32-bit execution-profile time source (see
   tx_user.h TX_EXECUTION_TIME_SOURCE = TIM2->CNT).  APB1/D2, TIMCLK = 2*PCLK1 =
   275 MHz, PSC = 0 (full resolution).  __HAL_RCC_TIM2_CLK_ENABLE touches only
   APB1LENR.TIM2EN (the peripheral gate) and _SLEEP_ENABLE only APB1LLPENR.TIM2LPEN
   (keep counting in CSleep/WFI) -- neither reprograms the clock tree, so the
   OCTOSPI2 XIP clock inheritance is untouched.  No interrupt is used; the kit just
   reads TIM2->CNT. */
static void tx_glue_epk_timer_init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_SLEEP_ENABLE();
    TIM2->PSC = 0u;
    TIM2->ARR = 0xFFFFFFFFu;
    TIM2->CNT = 0u;
    TIM2->EGR = TIM_EGR_UG;      /* latch PSC/ARR */
    TIM2->CR1 = TIM_CR1_CEN;     /* up-count, no interrupt */
}
#endif /* TX_EXECUTION_PROFILE_ENABLE */

void _tx_initialize_low_level(void)
{
    NVIC_SetPriority(PendSV_IRQn, 15);
    NVIC_SetPriority(SysTick_IRQn, 14);

    _tx_initialize_unused_memory = (VOID *)tx_unused_memory;

#ifdef TX_EXECUTION_PROFILE_ENABLE
    /* Bring TIM2 up here (before _tx_execution_initialize() samples the source). */
    tx_glue_epk_timer_init();
#endif
}

/* Called at the end of tx_application_define(), once the timer lists are set up by
   _tx_initialize_high_level(), to let the SysTick ISR drive ThreadX. */
void tx_glue_timer_enable(void)
{
    tx_timer_active = 1u;
}

/* Execution Profile Kit ISR-time accounting (issue #2).  The kit's nest counter is a
   plain read-modify-write; OTG_HS (prio 6) can preempt SysTick (prio 14), so the
   enter/exit calls are made atomic with a PRIMASK critical section.  No-op until
   tx_glue_profile_enable(); compiled out entirely when EPK is disabled. */
void tx_glue_isr_enter(void)
{
#ifdef TX_EXECUTION_PROFILE_ENABLE
    if (profile_active != 0u)
    {
        TX_INTERRUPT_SAVE_AREA
        TX_DISABLE
        _tx_execution_isr_enter();
        TX_RESTORE
    }
#endif
}

void tx_glue_isr_exit(void)
{
#ifdef TX_EXECUTION_PROFILE_ENABLE
    if (profile_active != 0u)
    {
        TX_INTERRUPT_SAVE_AREA
        TX_DISABLE
        _tx_execution_isr_exit();
        TX_RESTORE
    }
#endif
}

void tx_glue_profile_enable(void)
{
#ifdef TX_EXECUTION_PROFILE_ENABLE
    profile_active = 1u;
#endif
}

/* Number of 1 ms SysTicks per ThreadX tick.  Default 1 (1 kHz ThreadX tick).
   Override with -DTX_GLUE_TICK_DIV; keep in sync with TX_TIMER_TICKS_PER_SECOND. */
#ifndef TX_GLUE_TICK_DIV
#define TX_GLUE_TICK_DIV 1u
#endif

void SysTick_Handler(void)
{
    tx_glue_isr_enter();   /* EPK: attribute this ISR's time to (isr), not the thread */

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

    tx_glue_isr_exit();
}
