/*
 * newlib retargeting for the Wio Lite AI ThreadX shell app.
 *
 * Phase 1: stdout/stderr -> the CDC TX FIFO, best-effort.  It does NOT pump
 * tud_task() (only the usb thread does, so re-entering it from an arbitrary thread
 * would race) and returns `len` even on a short write so a large printf can never
 * spin newlib against a full FIFO.  Phase 1 output is small (banner/tick from the
 * usb thread), so nothing is dropped in practice; Phase 2 routes this through the
 * shell's ring + output lock for real flow control.
 */
#include <stdio.h>
#include "tusb.h"

extern char end;                 /* heap start (from the linker script) */
extern char _estack;             /* top of AXI-SRAM / MSP base (0x24050000) */
void *_sbrk(int incr)
{
  static char *heap = 0;
  if (heap == 0) heap = &end;
  /* Keep the heap clear of the top-of-RAM MSP stack: fail instead of silently
     walking into the stack / static thread stacks (no malloc is used yet, so this
     is a safety net, not a hot path). */
  char *const limit = &_estack - 0x1000;   /* reserve 4 KB below _estack for MSP */
  if (incr > 0 && (heap + incr > limit)) return (void *) -1;   /* ENOMEM */
  char *prev = heap;
  heap += incr;
  return prev;
}

int _write(int fd, char *buf, int len)
{
  (void) fd;
  if (!tud_cdc_connected()) return len;       /* discard when no host attached */
  tud_cdc_write((uint8_t const *) buf, (uint32_t) len);  /* accepts up to FIFO free */
  tud_cdc_write_flush();
  return len;                                 /* best-effort: overflow dropped */
}
