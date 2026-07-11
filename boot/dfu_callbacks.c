/*
 * DFU class callbacks for the Wio Lite AI DFU bootloader.
 *
 * PHASE 1.4: DFU download (dfu-util -a 0 -D file) is written to the external
 * OCTOSPI2 flash via the RAM-resident driver (boot/ospi_ram.c).
 *
 * App-first target: this firmware itself runs XIP from 0x70000000, so a download
 * here writes to a SCRATCH region (flash offset 0x400000 = AXI 0x70400000), NOT the
 * app base -- overwriting 0x70000000 would destroy the running code.  That conflict
 * only disappears in Phase 2, when the bootloader runs from internal flash and the
 * app base becomes a legitimate target.  Verify with:  dfu-util -a 0 -U readback.bin
 *
 * DFU (1.1) carries the image over EP0 control transfers in blocks of wTransferSize
 * (CFG_TUD_DFU_XFER_BUFSIZE = 1024).  block_num counts 0,1,2,...; the flash offset
 * is block_num*wTransferSize.  Each 4 KB sector (= 4 blocks) is erased on its first
 * block; each block is then page-programmed.
 */

#include "tusb.h"
#include "class/dfu/dfu_device.h"
#include "ospi_ram.h"

#define DFU_SCRATCH_FLASH   0x00400000u   // flash offset (AXI 0x70400000)
#define DFU_SCRATCH_SIZE    0x00010000u   // 64 KB test window

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

// Invoked on DFU_DNLOAD (wLength>0) + GET_STATUS (DFU_DNBUSY): flash one block.
void tud_dfu_download_cb(uint8_t alt, uint16_t block_num, uint8_t const *data, uint16_t length)
{
  (void) alt;

  uint32_t off = (uint32_t) block_num * CFG_TUD_DFU_XFER_BUFSIZE;

  if ((off < DFU_SCRATCH_SIZE) && (length > 0u))
  {
    uint32_t faddr = DFU_SCRATCH_FLASH + off;

    // Erase the 4 KB sector when a block lands on its boundary (every 4th block).
    if ((faddr & 0xFFFu) == 0u) ospi_ram_erase_sector(faddr);

    uint32_t n = length;
    if (off + n > DFU_SCRATCH_SIZE) n = DFU_SCRATCH_SIZE - off;   // clamp to window
    ospi_ram_program(faddr, data, n);
  }

  tud_dfu_finish_flashing(DFU_STATUS_OK);
}

// Invoked on DFU_DNLOAD (wLength=0) + GET_STATUS (DFU_MANIFEST): download complete.
void tud_dfu_manifest_cb(uint8_t alt)
{
  (void) alt;
  tud_dfu_finish_flashing(DFU_STATUS_OK);
}

// Invoked on DFU_UPLOAD: return the scratch window read through the memory-mapped
// OCTOSPI2 so the host can byte-verify the downloaded image (dfu-util -a 0 -U).
uint16_t tud_dfu_upload_cb(uint8_t alt, uint16_t block_num, uint8_t *data, uint16_t length)
{
  (void) alt;
  const uint32_t scratch_axi = 0x70400000u;

  uint32_t off = (uint32_t) block_num * length;
  if (off >= DFU_SCRATCH_SIZE) return 0;              // signal end of upload

  uint32_t n = DFU_SCRATCH_SIZE - off;
  if (n > length) n = length;

  const volatile uint8_t *src = (const volatile uint8_t *)(scratch_axi + off);
  for (uint32_t i = 0; i < n; i++) data[i] = src[i];
  return (uint16_t) n;
}
