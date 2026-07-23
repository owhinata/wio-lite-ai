/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/*
 * Staging buffer for an RTL8720DN firmware image received from the PC -- issue #19 M5.
 *
 * Why this exists: until now the board could only read the module's flash (M2/M4).  To
 * (re)write it, an image has to get ONTO the board first, and that path is also the only
 * way to restore the verified stock backup in _ref/ambd/board2-stock/ -- so it has to
 * work before anything is allowed to erase the module's boot sectors.
 *
 * The image lands in the external OCTOSPI1 PSRAM window at 0x90000000 (8 MB, brought up
 * in issue #3/#16).  AXI-SRAM could not hold it: a full chip image is 2 MB against 320 KB
 * of system RAM.  The MPU maps that window Normal / non-cacheable / shareable (app/mpu.c),
 * so a CPU write is visible to the following CPU read with no cache maintenance.
 *
 * Layering: this is app-level resource management (it owns a hardware-backed buffer), so
 * it sits next to psram.c, below the shell.  The shell wires rtl_img_sink() into the
 * YMODEM receiver (shell/cmds/cmd_xfer.c) and hands rtl_img_data() to
 * rtl_dl_flash_program().
 *
 * CONCURRENCY: every entry point here assumes the caller holds the OCTOSPI1 guard
 * (psram_acquire(), app/psram.c) for the whole operation, exactly as cmd_psram.c /
 * cmd_membench.c / cmd_devmem.c do.  Holding it across a multi-minute transfer is
 * deliberate: it is what stops a backgrounded `membench &` from overwriting the staged
 * image mid-flight.
 */
#ifndef APP_RTL8720_IMG_H
#define APP_RTL8720_IMG_H

#include <stdint.h>

#include "ymodem.h"   /* struct ym_sink -- the receive-side vtable this module fills */

/* Largest image that may be staged.  Matches the download protocol's conservative
 * destructive cap (RTL_DL_FLASH_WRITE_MAX): there is no point accepting something that
 * could never be programmed, and rejecting it up front is cheaper than failing after a
 * multi-megabyte transfer. */
#define RTL_IMG_MAX  0x00200000u

/* What is currently staged.  @padded_len / @digest are only meaningful once @valid. */
struct rtl_img {
	char     name[32];     /* filename from the sender's YMODEM block 0 (truncated) */
	uint32_t len;          /* bytes actually received */
	uint32_t padded_len;   /* @len rounded up to a 4 KB multiple, 0xFF filled */
	uint32_t digest;       /* the module's 0x27 algorithm over @padded_len bytes */
	int      valid;        /* 1 = a complete image is staged */
};

/*
 * Discard whatever is staged, then check that the PSRAM window actually stores and
 * returns data (a write/read/compare at the head and tail of the staging area).
 *
 * psram_ready() only says the controller reached memory-mapped mode during boot; it is
 * not a live health check, and a half-working PSRAM would silently corrupt an image that
 * then gets programmed into the module.  Call this BEFORE starting a transfer.
 * IT DESTROYS THE STAGED IMAGE -- that is why it invalidates first rather than pretending
 * to be side-effect free.  (It cannot defend against a totally unresponsive device: an
 * mmap read of one of those stalls the AXI bus until the IWDG resets the board.)
 *
 * Returns 0 when the window is usable, -1 if PSRAM never came up, -2 on a data mismatch.
 */
int rtl_img_probe(void);

/* Discard whatever is staged (leaves the PSRAM contents alone; only the record dies). */
void rtl_img_invalidate(void);

/*
 * The YMODEM sink vtable for this staging buffer, to hand to xfer_recv_sink_locked().
 * Its begin() rejects a zero or oversized file, which cancels the batch before any data
 * is transferred.  Its write() cannot overrun: begin() bounded the declared size and the
 * receiver never delivers more than that.
 *
 * On a successful transfer the caller MUST call rtl_img_finish(); on any failure it MUST
 * call rtl_img_invalidate(), because a batch that did not close is not a complete image
 * (see the ymodem_recv() contract).
 */
const struct ym_sink *rtl_img_sink(void);

/*
 * Close out a received image: pad up to the next 4 KB boundary with 0xFF (the erased
 * value, so the padding programs as a no-op) and compute the digest over the padded
 * range -- which is exactly the range rtl_dl_flash_program() will write and the module
 * will digest back.  Returns 0 and marks the image valid, or -1 if nothing was received.
 */
int rtl_img_finish(void);

/* The staged-image record (never NULL; check ->valid). */
const struct rtl_img *rtl_img_get(void);

/* Pointer to the staged bytes in the PSRAM window (valid for ->padded_len bytes). */
const uint8_t *rtl_img_data(void);

/*
 * Recompute the digest by re-reading PSRAM.  Commands are separate transactions, so
 * between `wifi imgload` and `wifi flashwrite` another command (`psram test`,
 * `membench`, `devmem`) could have overwritten the staging area.  Comparing this against
 * rtl_img_get()->digest catches that before anything is erased.  Returns 0 if no image
 * is staged.
 */
uint32_t rtl_img_verify(void);

#endif /* APP_RTL8720_IMG_H */
