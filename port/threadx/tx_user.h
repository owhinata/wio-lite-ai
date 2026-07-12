/*
 * tx_user.h - ThreadX build-time configuration for Wio Lite AI (STM32H725).
 *
 * Included by the Cortex-M7 GNU port assembly (unconditionally) and by the C
 * core + shell_obj when TX_INCLUDE_USER_DEFINE_FILE is defined (set in CMake), so
 * every translation unit that sees TX_THREAD agrees on its layout (ABI match).
 */
#ifndef TX_USER_H
#define TX_USER_H

/* ThreadX tick rate.  The SysTick handler (tx_glue.c) calls _tx_timer_interrupt()
   every TX_GLUE_TICK_DIV ms, so the effective tick rate is 1000 / TX_GLUE_TICK_DIV
   Hz.  Default: 1 kHz (1 tick = 1 ms).  The SysTick reload is derived from
   SystemCoreClock (=550 MHz -> 550000) by HAL_InitTick(); see port/threadx/tx_glue.c. */
#ifndef TX_TIMER_TICKS_PER_SECOND
#define TX_TIMER_TICKS_PER_SECOND  1000
#endif

/*
 * v1 keeps the port minimal: the Execution Profile Kit (TX_EXECUTION_PROFILE_ENABLE
 * / TX_CORTEX_M_EPK, giving `thread` cpu%) and TX_ENABLE_WFI (idle sleep) from the
 * stm32f746g-disco reference are intentionally NOT enabled here -- they need a
 * free-running time source (TIM2) and extra wake-path analysis.  They can be added
 * later.  TX_PORT_USE_BASEPRI is left undefined so the Cortex-M7 GNU port uses
 * PRIMASK critical sections: an OTG_HS ISR calling tx_event_flags_set can then never
 * preempt a ThreadX critical section, whatever its NVIC priority.
 */

#endif /* TX_USER_H */
