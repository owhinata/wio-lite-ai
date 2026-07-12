/*
 * DFU class callbacks for the Wio Lite AI STANDALONE DFU bootloader.
 *
 * Unlike the app-first boot/ firmware (which ran XIP from 0x70000000 and therefore
 * had to write to a scratch region to avoid self-destruction), this bootloader runs
 * from internal flash, so a DFU download targets the APP BASE directly: flash offset 0
 * == AXI 0x70000000, the address the board boots from.  Usage:
 *     dfu-util -a 0 -D app.bin      (program)
 *     dfu-util -a 0 -U readback.bin (read back for byte verification)
 *
 * DFU (1.1) carries the image over EP0 control transfers in blocks of wTransferSize
 * (CFG_TUD_DFU_XFER_BUFSIZE = 1024).  block_num counts 0,1,2,...; the flash offset is
 * block_num*wTransferSize.  Each 4 KB sector (= 4 blocks) is erased on its first block,
 * then each block is page-programmed.  Flashing is synchronous inside the callback;
 * get_timeout_cb paces the host's GET_STATUS poll.
 */

#include "tusb.h"
#include "class/dfu/dfu_device.h"
#include "octospi.h"

#define APP_FLASH_BASE_AXI   0x70000000u   // memory-mapped app base (for read-back)
#define APP_FLASH_SIZE       0x01000000u   // 16 MB (W25Q128), the whole device

// High-water mark (bytes) written this DFU session, so UPLOAD returns exactly what was
// downloaded.  Reset on each fresh download so a later UPLOAD reflects the new image.
static uint32_t g_dl_end;

// Invoked before tud_dfu_download_cb() (DFU_DNBUSY) or tud_dfu_manifest_cb()
// (DFU_MANIFEST): bwPollTimeout in ms the host waits before the next GET_STATUS.
// We flash synchronously inside the callbacks, so this only paces the host.
uint32_t tud_dfu_get_timeout_cb(uint8_t alt, uint8_t state)
{
  (void) alt;
  if (state == DFU_DNBUSY)   return 60;   // covers a 4 KB sector erase + program
  if (state == DFU_MANIFEST) return 0;
  return 0;
}

// Invoked on DFU_DNLOAD (wLength>0) + GET_STATUS (DFU_DNBUSY): flash one block to the
// app base.  block_num 0 starts a new image, so reset the high-water mark there.
void tud_dfu_download_cb(uint8_t alt, uint16_t block_num, uint8_t const *data, uint16_t length)
{
  (void) alt;

  if (block_num == 0u) g_dl_end = 0u;

  uint32_t off = (uint32_t) block_num * CFG_TUD_DFU_XFER_BUFSIZE;

  if ((off < APP_FLASH_SIZE) && (length > 0u))
  {
    uint32_t n = length;
    if (off + n > APP_FLASH_SIZE) n = APP_FLASH_SIZE - off;   // clamp to device

    // Erase the 4 KB sector when a block lands on its boundary (every 4th block).
    // 1024 divides 4096, so a block never straddles a sector boundary.
    if ((off & 0xFFFu) == 0u) octospi2_erase_sector(off);

    octospi2_program(off, data, n);

    if (off + n > g_dl_end) g_dl_end = off + n;
  }

  tud_dfu_finish_flashing(DFU_STATUS_OK);
}

// Invoked on DFU_DNLOAD (wLength=0) + GET_STATUS (DFU_MANIFEST): download complete.
// Milestone 3 has no boot flow yet, so just acknowledge (the board stays in DFU mode).
void tud_dfu_manifest_cb(uint8_t alt)
{
  (void) alt;
  tud_dfu_finish_flashing(DFU_STATUS_OK);
}

// Invoked on DFU_UPLOAD: return the app region read through the memory-mapped OCTOSPI2
// so the host can byte-verify (dfu-util -a 0 -U).  Bounded by this session's download
// high-water mark (or a 4 KB default) so upload terminates instead of streaming 16 MB.
uint16_t tud_dfu_upload_cb(uint8_t alt, uint16_t block_num, uint8_t *data, uint16_t length)
{
  (void) alt;
  uint32_t upload_len = g_dl_end ? g_dl_end : 0x1000u;

  uint32_t off = (uint32_t) block_num * length;
  if (off >= upload_len) return 0;                   // signal end of upload

  uint32_t n = upload_len - off;
  if (n > length) n = length;

  const volatile uint8_t *src = (const volatile uint8_t *)(APP_FLASH_BASE_AXI + off);
  for (uint32_t i = 0; i < n; i++) data[i] = src[i];
  return (uint16_t) n;
}
