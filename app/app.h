/*
 * Shared declarations for the Wio Lite AI ThreadX shell app.
 */
#ifndef APP_H
#define APP_H

#include "tx_api.h"   /* ULONG */

/* Bring up the OTG_HS pins/clock (PA11/PA12 AF10, FS internal PHY). */
void usb_hw_init(void);

/* ThreadX thread entry: sole caller of tud_task() / tud_cdc_* (see app/usb_cdc.c). */
void usb_thread_entry(ULONG arg);

/* Enable the classified Cortex-M7 faults (Mem/Bus/Usage + div0 trap) and install
 * the crash-recording fault handlers (see app/fault.c).  Call after log_init(). */
void fault_init(void);

/* Create the mutex that makes the newlib heap thread-safe (see app/malloc_lock.c).
 * Call once from tx_application_define(), before any thread runs. */
void malloc_lock_init(void);

#endif /* APP_H */
