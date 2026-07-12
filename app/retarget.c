/*
 * newlib heap retargeting for the Wio Lite AI ThreadX shell app.
 *
 * _write (stdout/stderr -> the CDC TX ring) lives in the USB CDC backend
 * (shell/backend/cli_backend_usbcdc.c) so it shares the shell's single TX owner.
 * This file provides only the heap (_sbrk), bounded so a stray allocation cannot
 * walk into the top-of-RAM MSP stack.
 */
#include <stdint.h>

extern char end;                 /* heap start (from the linker script) */
extern char _estack;             /* top of AXI-SRAM / MSP base (0x24050000) */
void *_sbrk(int incr)
{
  static char *heap = 0;
  if (heap == 0) heap = &end;
  /* Reserve 4 KB below _estack for the MSP stack; integer math avoids the
     -Warray-bounds a bare `&_estack - 0x1000` triggers on the linker symbol. */
  char *const limit = (char *)((uintptr_t)&_estack - 0x1000u);
  if (incr > 0 && (heap + incr > limit)) return (void *) -1;   /* ENOMEM */
  char *prev = heap;
  heap += incr;
  return prev;
}
