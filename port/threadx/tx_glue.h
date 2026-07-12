/*
 * ThreadX low-level glue for Wio Lite AI -- public hooks (see tx_glue.c).
 */
#ifndef TX_GLUE_H
#define TX_GLUE_H

/* Enable the SysTick -> _tx_timer_interrupt() path.  Call once, at the very end of
   tx_application_define(), after all ThreadX objects/timer lists exist. */
void tx_glue_timer_enable(void);

#endif /* TX_GLUE_H */
