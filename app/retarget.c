/*
 * newlib heap retargeting for the Wio Lite AI ThreadX shell app.
 *
 * _write (stdout/stderr -> the CDC TX ring) lives in the USB CDC backend
 * (shell/backend/cli_backend_usbcdc.c) so it shares the shell's single TX owner.
 * This file provides only the heap (_sbrk), bounded so a stray allocation cannot
 * walk off the end of AXI-SRAM.
 */
#include <stdint.h>

extern char end;                 /* heap start (from the linker script) */
/*
 * End of AXI-SRAM.  This used to be _estack, back when the main stack sat at the
 * top of AXI-SRAM and growing the heap into it was the hazard worth 4 KB of
 * clearance.  Issue #46 moved the main stack to the top of DTCM, so _estack is now
 * 0x20020000 -- BELOW the heap.  Keeping it here would have made every comparison
 * below true and turned _sbrk into an unconditional ENOMEM: malloc failing
 * everywhere, in a firmware that links without a warning.  __ram_end is the linker
 * symbol that means what this line actually needs (see the linker script).
 */
extern char __ram_end;
void *_sbrk(int incr)
{
  static char *heap = 0;
  if (heap == 0) heap = &end;
  /* Nothing lives above the heap in AXI-SRAM any more, so the whole of the rest of
     the region is available.  Integer math avoids the -Warray-bounds a bare
     `&__ram_end` pointer comparison triggers on a linker symbol. */
  char *const limit = (char *)((uintptr_t)&__ram_end);
  if (incr > 0 && (heap + incr > limit)) return (void *) -1;   /* ENOMEM */
  char *prev = heap;
  heap += incr;
  return prev;
}
