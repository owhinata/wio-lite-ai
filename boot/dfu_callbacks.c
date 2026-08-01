/*
 * DFU class callbacks for the Wio Lite AI standalone DFU bootloader.
 *
 * A download targets the APP PARTITION IN THE INTERNAL FLASH: offset 0 ==
 * 0x08020000 (sector 1), the address the board boots from since issue #25.
 * Usage is unchanged from when the app lived in the external flash:
 *     dfu-util -d 0483:df11 -a 0 -D app.bin      (program)
 *     dfu-util -d 0483:df11 -a 0 -U readback.bin (read back for verification)
 *
 * DFU (1.1) carries the image over EP0 control transfers in blocks of
 * wTransferSize (CFG_TUD_DFU_XFER_BUFSIZE = 1024).  block_num counts 0,1,2,...;
 * the partition offset is block_num*wTransferSize.  1024 divides the 128 KB
 * sector size, so a block never straddles a sector, and it is a multiple of the
 * 32-byte flash word, so every program call is naturally aligned.
 *
 * TWO PROPERTIES THIS FILE EXISTS TO GUARANTEE
 * --------------------------------------------
 *
 * 1. VECTOR-LAST COMMIT.  The first flash word (32 B: initial MSP + reset vector)
 *    is held back and written only when the whole image has arrived and passed
 *    every check.  Until then the word stays erased (0xFFFFFFFF), which
 *    app_valid() in main.c rejects -- so an interrupted, aborted or corrupted
 *    download leaves a board that comes up in DFU mode ready for another try,
 *    never one that jumps into a half-written image.  With the external flash
 *    gone there is no second app to fall back to, so this is THE mechanism that
 *    keeps a failed download recoverable.
 *
 * 2. NO PARTIAL SESSION CAN COMMIT.  The state machine below only ever commits a
 *    session that started at block 0 and received strictly contiguous blocks with
 *    no error.  TinyUSB calls tud_dfu_abort_cb() on DFU_ABORT but nothing at all
 *    on CLRSTATUS or SET_INTERFACE (it just resets its own state machine), so the
 *    bootloader cannot rely on being told; requiring a complete block-0-rooted
 *    chain covers those cases without needing a notification.
 *
 * The image's own reset vector is checked against the internal app window before
 * the commit, which is what stops an old external-XIP build (whose reset vector
 * points into 0x70000000) from being installed by mistake.
 */

#include <string.h>

#include "tusb.h"
#include "class/dfu/dfu_device.h"
#include "iflash.h"

/*
 * Poll timeouts (ms) handed to the host.  dfu-util sleeps for exactly this long
 * before its next GET_STATUS, so these values are both a correctness floor and
 * the download's wall-clock cost: a 208-block image spends 208*PROGRAM_POLL_MS
 * plus one ERASE_POLL_MS per 128 KB sector, sleeping.
 *
 * ERASE_POLL_MS is the one that MUST NOT be too small.  While a sector erase
 * runs, this bootloader -- which executes from the same single flash bank -- is
 * stalled completely: no USB interrupt, no SysTick (RM0468 sec 4.3.8).  The host
 * has to have been told to wait longer than the erase takes before it is started.
 * Measured on board #2: a 128 KB sector erase is 888 ms (app-side `iflash test`
 * during the issue #25 bring-up); the bootloader re-reports the live figure on
 * the CDC console after every erase.  2500 ms is ~2.8x that.
 *
 * PROGRAM_POLL_MS covers one 1024-byte block: 2.685 ms measured, including the
 * read-back verify.  10 ms is ~3.7x.  (The old external-flash path used 60 ms
 * because each block could also carry a 4 KB sector erase; internally that is
 * pure overhead -- 12.5 s of sleeping across a 208 KB image.)  Undershooting
 * here is benign anyway: programming happens synchronously before the reply the
 * host is waiting for, so an early poll simply waits.
 */
#define ERASE_POLL_MS      2500u
#define PROGRAM_POLL_MS      10u
#define MANIFEST_POLL_MS    100u   /* commits one 32-byte flash word */

/* On-chip RAM regions an image's initial MSP may point into (AXI-SRAM, TCM,
 * D2 and D3 SRAM) -- the same test main.c applies before jumping. */
static int msp_plausible(uint32_t msp)
{
	uint32_t hi = msp & 0xFF000000u;

	return (hi == 0x24000000u) || (hi == 0x20000000u) ||
	       (hi == 0x30000000u) || (hi == 0x38000000u);
}

/* A reset vector inside the internal app partition, with the Thumb bit set. */
static int reset_plausible(uint32_t rst)
{
	return (rst >= IFLASH_APP_BASE) &&
	       (rst < IFLASH_APP_BASE + IFLASH_APP_SIZE) && ((rst & 1u) != 0u);
}

/* ---- download session state ------------------------------------------- */
static struct {
	uint8_t  active;                     /* started at block 0 */
	uint8_t  err;                        /* something failed; refuse to commit */
	uint8_t  have_vec;                   /* vec[] holds the image's first word */
	uint32_t next_off;                   /* offset the next block must have */
	uint32_t total;                      /* bytes accepted so far */
	uint8_t  vec[IFLASH_WORD_SIZE];      /* held-back first flash word */
} dl;

/* Bytes of the last successfully committed image: bounds UPLOAD so a read-back
 * returns exactly what was written instead of streaming the whole partition. */
static uint32_t g_image_len;

static void dl_reset(void)
{
	memset(&dl, 0, sizeof dl);
}

void tud_dfu_abort_cb(uint8_t alt)
{
	(void) alt;
	dl_reset();
}

/*
 * Invoked before tud_dfu_download_cb() (DFU_DNBUSY) or tud_dfu_manifest_cb()
 * (DFU_MANIFEST): bwPollTimeout in ms the host waits before the next GET_STATUS.
 * TinyUSB asks for this while building the GET_STATUS reply and only runs the
 * callback after that reply is acknowledged, so the host is already waiting when
 * a long erase starts.
 *
 * THE QUOTE IS MADE BEFORE THE BLOCK NUMBER IS KNOWN, which is the awkward part:
 * the DFU protocol asks for the delay one control transfer earlier than it tells
 * us what is coming.  So the rule is conservative -- quote the erase delay unless
 * we can PROVE the next block is a mid-sector continuation of a healthy session:
 *
 *   - no session, or the last one ended (manifest / abort) -> the next block is a
 *     block 0, which erases sector 1.
 *   - a session that hit an error -> the host's natural response is to restart at
 *     block 0, which erases.  (This is the case a "use dl.next_off alone" version
 *     got wrong: after a failure, next_off is some mid-sector value, so a restart
 *     at block 0 would have been quoted 10 ms and then spent ~888 ms erasing.)
 *   - next_off on a 128 KB boundary -> the arriving block enters a new sector.
 *
 * Residual, accepted: a host that restarts at block 0 in the middle of a healthy
 * transfer (no error, mid-sector) is indistinguishable here from a continuation,
 * so it gets the short quote and then waits out the erase.  dfu-util does not do
 * this, and the wait is bounded by one erase (~888 ms measured), well inside
 * libusb's 5 s control-transfer timeout -- slow, not broken.
 */
uint32_t tud_dfu_get_timeout_cb(uint8_t alt, uint8_t state)
{
	(void) alt;

	if (state == DFU_DNBUSY) {
		if (!dl.active || dl.err ||
		    (dl.next_off % IFLASH_SECTOR_SIZE) == 0u)
			return ERASE_POLL_MS;
		return PROGRAM_POLL_MS;
	}
	if (state == DFU_MANIFEST)
		return MANIFEST_POLL_MS;
	return 0;
}

/*
 * Invoked on DFU_DNLOAD (wLength>0) + GET_STATUS (DFU_DNBUSY): take one block.
 * Block 0 starts a fresh session and its first flash word is held back; every
 * later block must continue exactly where the previous one ended.
 */
void tud_dfu_download_cb(uint8_t alt, uint16_t block_num,
                         uint8_t const *data, uint16_t length)
{
	uint32_t off = (uint32_t) block_num * CFG_TUD_DFU_XFER_BUFSIZE;

	(void) alt;

	if (block_num == 0u) {
		dl_reset();
		if (!iflash_available()) {
			/* Wrong device: refuse the whole path rather than erase
			 * something whose sector map we do not know. */
			tud_dfu_finish_flashing(DFU_STATUS_ERR_TARGET);
			return;
		}
		if (length < IFLASH_WORD_SIZE) {
			/* Too small to even hold a vector table. */
			tud_dfu_finish_flashing(DFU_STATUS_ERR_FILE);
			return;
		}
		dl.active = 1u;
	}

	if (!dl.active || dl.err) {
		/* Not rooted at block 0, or a previous block already failed. */
		dl.err = 1u;
		tud_dfu_finish_flashing(DFU_STATUS_ERR_UNKNOWN);
		return;
	}

	if (off != dl.next_off || length == 0u ||
	    length > CFG_TUD_DFU_XFER_BUFSIZE ||
	    off > IFLASH_APP_SIZE || length > IFLASH_APP_SIZE - off) {
		dl.err = 1u;
		tud_dfu_finish_flashing(DFU_STATUS_ERR_ADDRESS);
		return;
	}

	/* Erase on entering a sector.  off is a multiple of 1024 and the sector
	 * size is 128 KB, so this is exact. */
	if ((off % IFLASH_SECTOR_SIZE) == 0u) {
		uint32_t sector = IFLASH_APP_SECTOR_LO + off / IFLASH_SECTOR_SIZE;

		if (iflash_erase_sector(sector) != IFLASH_OK) {
			dl.err = 1u;
			tud_dfu_finish_flashing(DFU_STATUS_ERR_ERASE);
			return;
		}
	}

	if (off == 0u) {
		/* Hold back the first flash word; write the rest of the block. */
		memcpy(dl.vec, data, IFLASH_WORD_SIZE);
		dl.have_vec = 1u;
		if (iflash_program(IFLASH_WORD_SIZE, data + IFLASH_WORD_SIZE,
		                   length - IFLASH_WORD_SIZE) != IFLASH_OK) {
			dl.err = 1u;
			tud_dfu_finish_flashing(DFU_STATUS_ERR_PROG);
			return;
		}
	} else if (iflash_program(off, data, length) != IFLASH_OK) {
		dl.err = 1u;
		tud_dfu_finish_flashing(DFU_STATUS_ERR_PROG);
		return;
	}

	dl.next_off = off + length;
	dl.total    = dl.next_off;
	tud_dfu_finish_flashing(DFU_STATUS_OK);
}

/*
 * Invoked on DFU_DNLOAD (wLength=0) + GET_STATUS (DFU_MANIFEST): the image is
 * complete.  This is the commit point -- and the only place the first flash word
 * is ever written.  Reboot is requested ONLY if the commit itself read back
 * clean, so a failure leaves the vector unwritten (or detectably wrong) and the
 * next boot lands in DFU mode instead of jumping into a broken image.
 */
void boot_request_reboot(void);   /* boot/main.c */

void tud_dfu_manifest_cb(uint8_t alt)
{
	uint32_t msp, rst;

	(void) alt;

	if (!dl.active || dl.err || !dl.have_vec || dl.total < IFLASH_WORD_SIZE) {
		dl_reset();
		tud_dfu_finish_flashing(DFU_STATUS_ERR_NOTDONE);
		return;
	}

	memcpy(&msp, &dl.vec[0], sizeof msp);
	memcpy(&rst, &dl.vec[4], sizeof rst);

	/* Refuse an image that is not a valid app for THIS partition.  Notably an
	 * old external-XIP build has a reset vector in 0x70000000 and is rejected
	 * here rather than installed and jumped into. */
	if (!msp_plausible(msp) || !reset_plausible(rst)) {
		dl_reset();
		tud_dfu_finish_flashing(DFU_STATUS_ERR_FILE);
		return;
	}

	/* iflash_program() reads back what it wrote, so a successful return means
	 * the vector really is in the flash. */
	if (iflash_program(0u, dl.vec, IFLASH_WORD_SIZE) != IFLASH_OK) {
		dl_reset();
		tud_dfu_finish_flashing(DFU_STATUS_ERR_VERIFY);
		return;   /* no reboot: stay in DFU so the host can retry */
	}

	g_image_len = dl.total;
	dl_reset();
	tud_dfu_finish_flashing(DFU_STATUS_OK);
	boot_request_reboot();
}

/*
 * Invoked on DFU_UPLOAD: return the app partition so the host can byte-verify
 * (dfu-util -a 0 -U).  Bounded by the last committed image length (or 4 KB if
 * nothing has been downloaded this session) so the upload terminates instead of
 * streaming the whole 384 KB partition.
 */
uint16_t tud_dfu_upload_cb(uint8_t alt, uint16_t block_num,
                           uint8_t *data, uint16_t length)
{
	uint32_t upload_len = g_image_len ? g_image_len : 0x1000u;
	/* Block numbers count wTransferSize units (what the DFU functional
	 * descriptor advertises), not whatever wLength this particular request
	 * happens to carry. */
	uint32_t off = (uint32_t) block_num * CFG_TUD_DFU_XFER_BUFSIZE;
	uint32_t n;
	const volatile uint8_t *src;
	uint32_t i;

	(void) alt;

	if (off >= upload_len)
		return 0;                  /* signal end of upload */

	n = upload_len - off;
	if (n > length)
		n = length;

	src = (const volatile uint8_t *)(IFLASH_APP_BASE + off);
	for (i = 0; i < n; i++)
		data[i] = src[i];
	return (uint16_t) n;
}
