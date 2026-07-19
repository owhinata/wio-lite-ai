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
 * Execution Profile Kit -- the `thread` command's cpu% column (issue #2).
 *
 * Defined here (not via CMake -D) so every translation unit that sees TX_THREAD
 * agrees on its layout: the kit adds tx_thread_execution_time_total / _last_start
 * to TX_THREAD, and tx_api.h auto-includes tx_execution_profile.h -- both only when
 * this is defined.  tx_user.h is included by the port asm (tx_thread_schedule.S,
 * tx_thread_context_{save,restore}.S) and by the C core + shell, so all stay
 * ABI-consistent and the port asm emits the _tx_execution_thread_enter/exit hooks.
 */
#define TX_EXECUTION_PROFILE_ENABLE

/* Use the Cortex-M execution-profile path (nest counter) for ISR accounting.
 * Mandatory here: this port's TX_THREAD_GET_SYSTEM_STATE() ORs in the IPSR
 * (tx_port.h), so inside an ISR it is never == 1; the non-EPK "== 1" guard in
 * tx_execution_profile.c would drop all ISR time.  The EPK path guards on
 * "truthy && nest_counter == 1", which works once our plain-C ISRs (SysTick +
 * OTG_HS) call _tx_execution_isr_enter/exit (via tx_glue_isr_enter/exit). */
#define TX_CORTEX_M_EPK

/* Execution-profile time source = TIM2->CNT (0x40000024): APB1/D2, 32-bit,
 * free-running at TIM2CLK = 2*PCLK1 = 275 MHz (wrap ~15.6 s).  TIM2 is started in
 * port/threadx/tx_glue.c (_tx_initialize_low_level).  Chosen over the kit default
 * DWT->CYCCNT (0xE0001004) because DWT freezes when the core clock is gated by WFI
 * (issue #2): TIM2 keeps counting in CSleep (TIM2LPEN, RM0468 s8.7.53), so cpu%/idle
 * stay correct once WFI is enabled.  DWT stays the udelay/membench time base (those
 * busy-wait in the foreground and never run while the core is in WFI).  Each EPK
 * delta is bounded to <= 1 ms by the SysTick isr hook, far below the 32-bit wrap.
 * TX_EXECUTION_MAX_TIME_SOURCE keeps its 0xFFFFFFFF default (full 32-bit). */
#define TX_EXECUTION_TIME_SOURCE \
    ((EXECUTION_TIME_SOURCE_TYPE)(*(volatile ULONG *)0x40000024UL))

/* Idle power saving: when no thread is ready the Cortex-M7 port inserts DSB;WFI;ISB
 * (tx_thread_schedule.S __tx_ts_wait) instead of busy-spinning, so the core sleeps
 * until an interrupt.  Build-gated by BSP_ENABLE_WFI (CMake, default ON) so an
 * SWD-debug build can be made with -DBSP_ENABLE_WFI=OFF (a WFI-sleeping core is hard
 * to attach without connect-under-reset).  Safe: Cortex-M7 CSleep keeps HCLK/APB
 * running (only the CPU stops), so SysTick(14) still ticks, OTG_HS(6) RX wakes the
 * core (WFI wakes on any enabled IRQ regardless of PRIMASK), and TIM2/OCTOSPI2 keep
 * their clocks (TIM2LPEN / OCTO2LPEN) -- the app XIP-executes from OCTOSPI2. */
#if defined(BSP_ENABLE_WFI) && (BSP_ENABLE_WFI)
#define TX_ENABLE_WFI
#endif

/* TX_PORT_USE_BASEPRI is left undefined so the Cortex-M7 GNU port uses PRIMASK
 * critical sections: an OTG_HS ISR calling tx_event_flags_set can then never preempt
 * a ThreadX critical section, whatever its NVIC priority. */

#endif /* TX_USER_H */
