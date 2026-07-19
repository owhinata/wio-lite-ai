/*
 * ThreadX low-level glue for Wio Lite AI -- public hooks (see tx_glue.c).
 */
#ifndef TX_GLUE_H
#define TX_GLUE_H

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

#endif /* TX_GLUE_H */
