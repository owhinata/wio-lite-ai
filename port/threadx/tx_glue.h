/*
 * ThreadX low-level glue for Wio Lite AI -- public hooks (see tx_glue.c).
 */
#ifndef TX_GLUE_H
#define TX_GLUE_H

/* C++-callable: port/mlperf is compiled as C++ (the MLPerf harness headers it must
   match declare no linkage of their own), and tx_glue_us64() is its time source. */
#ifdef __cplusplus
extern "C" {
#endif

/* Enable the SysTick -> _tx_timer_interrupt() path.  Call once, at the very end of
   tx_application_define(), after all ThreadX objects/timer lists exist. */
void tx_glue_timer_enable(void);

/* Execution Profile Kit (issue #2) ISR-time accounting hooks.  Bracket the body of
   every plain-C ISR (SysTick internally, OTG_HS in app/usb_cdc.c) with these so the
   kit attributes the ISR's time to the (isr) row instead of the interrupted thread.
   They no-op until tx_glue_profile_enable(), and compile to nothing when EPK is off. */
void tx_glue_isr_enter(void);
void tx_glue_isr_exit(void);

/* Arm the ISR hooks above.  Call once from an application thread's entry (i.e. after
   the scheduler and _tx_execution_initialize() have run). */
void tx_glue_profile_enable(void);

#include <stdint.h>

/*
 * Monotonic microseconds since the timer came up (issue #55).
 *
 * Lives here because THIS FILE OWNS TIM2 -- it is the execution-profile time source
 * tx_glue_epk_timer_init() starts, and a module that merely wants a clock has no
 * business reading another layer's peripheral.
 *
 * WHY IT IS BUILT FROM TWO SOURCES.  None of the three clocks this board has is
 * sufficient alone over a benchmark-length interval:
 *
 *   DWT->CYCCNT   1.8 ns, but wraps every 7.81 s AND freezes under idle WFI
 *                 (issue #2), so it does not measure wall clock at all.
 *   TIM2->CNT     3.6 ns and WFI-safe (TIM2LPEN, RM0468 sec 8.7.53), but 32-bit at
 *                 the timer clock, so it wraps every ~15.6 s.
 *   tx_time_get() 1 kHz, wraps in 49.7 days, never freezes -- but 1 ms of resolution
 *                 is coarse for a per-inference figure.
 *
 * Composed, they cover each other: TIM2 supplies the sub-wrap phase, and the tick
 * counter says which wrap that phase belongs to.  The wrap estimate is correct as
 * long as the tick's idea of "now" is within half a TIM2 period -- 7.8 s -- of the
 * truth, and the tick is off by milliseconds.  Eight orders of magnitude of margin.
 *
 * Needs no lock: the two counter reads can straddle a millisecond, which is precisely
 * the error the round-to-nearest already absorbs.  It is therefore safe to call from
 * an ISR that runs after the scheduler has started -- but not from early startup,
 * because it asks the RCC for the timer clock and reads the ThreadX tick, neither of
 * which is meaningful before _tx_initialize_low_level() has run.
 *
 * The microsecond period of a TIM2 wrap is computed with integer division, so each
 * wrap loses under one microsecond.  Over a benchmark window of minutes that is tens
 * of microseconds against hundreds of seconds; it is not worth a rational
 * approximation, but it is why this is a good stopwatch and not a good calendar.
 *
 * NOT monotonic across the tick counter's own wrap -- tx_time_get() is 32-bit, so
 * after 49.7 days of uptime the coarse half restarts and this value jumps backwards.
 * Every caller so far measures intervals of seconds, which is what makes that
 * acceptable; a caller that needs an absolute epoch needs a different design, not a
 * bigger number here.
 */
uint64_t tx_glue_us64(void);

/* The timer clock tx_glue_us64() counts in, derived (not assumed) from the RCC.  Also
   exported because it is what makes a raw TIM2->CNT delta meaningful to a caller
   doing its own short-interval timing. */
uint32_t tx_glue_timer_hz(void);

#ifdef __cplusplus
}
#endif

#endif /* TX_GLUE_H */
