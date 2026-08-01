/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    fx_sd_driver.h
 * @brief   FileX media driver on the SDMMC1 block API (issue #6).
 *
 * Maps FileX driver requests onto sd_card_read/write_blocks().  The SD card has
 * its own wear-levelling FTL, so there is no LevelX layer.  Because PC and camera
 * tools format microSD with an MBR + a FAT partition (the VBR is then not at LBA 0)
 * while FileX itself only ever reads logical sector 0 and never computes a
 * partition offset, this driver presents partition 0 as a superfloppy: INIT reads
 * LBA 0, works out the partition start and size, and every later request adds that
 * start so FileX sees a flat volume.
 */
#ifndef FX_SD_DRIVER_H
#define FX_SD_DRIVER_H

#include "fx_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/** FileX media driver entry (pass to fx_media_open / fx_media_format). */
VOID fx_sd_driver(FX_MEDIA *media_ptr);

/**
 * The SD_* result of the sd_card_probe() run by the most recent FX_DRIVER_INIT.
 *
 * fx_media_open() reports every driver INIT failure as a flat FX_IO_ERROR, which
 * would make "no card in the slot" indistinguishable from "card present but not
 * FAT".  The mount glue reads this afterwards to tell the two apart and report
 * FS_ERR_NO_CARD for the former.  Reset at the top of every INIT, so a stale
 * SD_ERR_NO_CARD can never leak into a later, unrelated failure.
 */
int fx_sd_last_probe_rc(void);

/**
 * Enable/disable "format mode" for the next driver INIT.  A blank card has no
 * valid VBR/MBR, so the normal INIT partition detection fails and fx_media_format
 * could never run.  With format mode on, INIT skips detection and treats the whole
 * card as a superfloppy (partition start 0, size = card block count).  The `sd`
 * command sets it under the exclusive ownership slot around fx_media_format and
 * clears it afterwards.
 */
void fx_sd_set_format_mode(int on);

#ifdef __cplusplus
}
#endif

#endif /* FX_SD_DRIVER_H */
