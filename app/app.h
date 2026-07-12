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

#endif /* APP_H */
