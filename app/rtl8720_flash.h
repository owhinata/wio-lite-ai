/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/*
 * Wio Lite AI (STM32H725AEI6) -- onboard RTL8720DN (Realtek AmebaD/RTL8721D) UART
 * firmware-download support (issue #19).  This is the STM32-side, on-device flasher
 * that speaks the AmebaD ROM download protocol directly over the RTL8720's LOG UART
 * (= our UART9, PD14/PD15), replacing the host-PC "image tool".
 *
 * MILESTONE 1 (this file, for now): prove UART download-mode ENTRY on this board via
 * a read-only handshake -- it performs the strap+reset sequence, then issues the
 * download protocol's read-word command and checks for the framed reply.  It does NOT
 * erase or write any flash (fully reversible; the RTL8720 mask-ROM download mode is
 * re-enterable, so this cannot brick it).  M2 adds the flashloader stub upload + flash
 * READ (still non-destructive -- SRAM write + flash read only); M3 adds erase/write/verify;
 * M4 adds the microSD image source.
 *
 * Layering: HAL/CMSIS <- rtl8720.c (basic UART/power) <- rtl8720_flash.c (download
 * protocol) <- cmd_wifi.c (shell).  It never touches the RCC clock tree -- only GPIO
 * reconfig + the #17 rtl8720 UART/power primitives -- so it is clock-safe.  cli-agnostic:
 * timing via ThreadX, cancellation via an optional abort hook (like app/erpc.c).
 *
 * REQUIRED CALLER DISCIPLINE (issue #21 increment 8).  Every entry point here drives
 * CHIP_EN and opens/closes UART9 at several baud rates internally, and reads the shared
 * SPSC RX ring itself.  The RTL8720 link now has other users (the resident eRPC service
 * thread, and the L2 bridge owner that holds the UART while the host stack is up), so a
 * caller MUST hold the coarse link mutex with no eRPC session live for the WHOLE session --
 * rtl_link_hw_claim(sh, false) .. rtl_link_hw_release(sh) (app/rtl_link.h), which
 * rejects the command outright while the eRPC UART is referenced.  Nothing here takes
 * that lock for you.
 *
 * Entry mechanism (RTL872xD datasheet Table 3-2/3-4, schematic sheet 5/8):
 *   - The boot ROM samples PA[7] (= UART_LOG_TXD) at reset; LOW selects UART download
 *     mode (active-low).  PA[7] is wired to STM32 PD14 (UART9_RX) via WIFI_DEBUG_TXD.
 *   - CHIP_EN = PC3 is the sole reset/enable.
 * So: power the module off (PC3 low) FIRST, drive PD14 low (strap), release reset
 * (PC3 high) so the ROM samples the strap, briefly hold, then return PD14 to UART9_RX
 * and talk the protocol at 115200.
 *
 * Protocol reference (clean-room; comments cite the OSS reference): pvvx
 * SharpRTL872xTool Program.cs.  read-word = 0x31 + addr(u32 LE) -> 0x31 + data(u32 LE)
 * + 0x15 status (Program.cs ReadRegs).  This is the module's first real command in the
 * reference tool, so a valid framed reply proves download-mode entry.
 */
#ifndef APP_RTL8720_FLASH_H
#define APP_RTL8720_FLASH_H

#include <stdint.h>

/*
 * The module's own flash digest (its 0x27 command), computed incrementally on our
 * side so a host-supplied image can be verified WITHOUT reading the flash back.
 *
 * ALGORITHM (established on hardware, board #2, M4): a plain sum of the range read
 * as 32-bit LITTLE-ENDIAN words.  A 4 KB dump the module digested as 0xC9AB910F
 * matched the LE word sum of the received bytes exactly, while a byte sum, CRC-32
 * and Adler-32 all differed.  This single definition is shared by the backup
 * streamer, the staged-image checker and rtl_dl_flash_program() -- do not
 * re-implement it.  Ranges are always 4 KB multiples here, so the tail is exact.
 *
 * NOTE it is only a 32-bit sum: it detects corruption, not adversarial collision.
 * Where BYTE equality has to be proven (a backup, a restored image) compare the
 * bytes themselves on the host rather than trusting this alone.
 */
struct rtl_dl_digest {
	uint32_t sum;    /* completed words */
	uint32_t acc;    /* partial word being assembled */
	uint8_t  nacc;   /* bytes already in acc (0..3) */
};

void     rtl_dl_digest_init(struct rtl_dl_digest *d);
void     rtl_dl_digest_add(struct rtl_dl_digest *d, const uint8_t *p, uint32_t n);
uint32_t rtl_dl_digest_value(const struct rtl_dl_digest *d);

/* Result of an M1 download-mode probe (all diagnostic; the `wifi flashprobe` command
 * that printed it was retired by issue #28 -- the probe stays as the protocol layer's
 * entry self-test). */
struct rtl_dl_result {
	int      entered;      /* 1 if a valid read-word reply frame (0x31..0x15) was seen */
	int      slip;         /* 1 if SLIP framing was used for the reply, 0 if raw */
	uint32_t word;         /* read-word value at 0x00082000 (valid only if entered);
	                        * 0x00082021 means the flashloader stub is already resident */
	uint8_t  raw[32];      /* raw bytes received during the probe (for diagnosis) */
	int      raw_len;      /* number of valid bytes in raw[] */
	uint32_t overflows;    /* UART9 RX ring overflows observed (rtl8720_uart_overflows) */
};

/*
 * Enter UART download mode: power-cycle the RTL8720 with PD14 held low across the
 * CHIP_EN rising edge so the ROM samples the download strap, then open UART9 @115200.
 * @hold_us is how long PD14 is held low AFTER CHIP_EN goes high (bounded; kept small
 * to minimise any PD14/PA[7] overlap -- see the .c).  @should_abort (may be NULL) is
 * polled during the waits so Ctrl+C can cancel.
 *
 * Returns 0 with UART9 open (caller must eventually rtl8720_uart_close()), -1 if UART9
 * did not come ready, or -4 if aborted.  GUARANTEE: on every return path (success,
 * failure, abort) PD14 is left as an input / AF -- NEVER as a driven-low output -- so
 * it cannot contend with the RTL8720 driving PA[7] as its LOG-UART TX.
 */
int rtl_dl_enter(uint32_t hold_us, int (*should_abort)(void *ctx), void *abort_ctx);

/*
 * Probe for download-mode entry on the already-open UART9 (call after rtl_dl_enter):
 * send the read-word command for 0x00082000 and wait up to @timeout_ms for the framed
 * reply.  @use_slip selects the framing: 0 = raw (matches the pvvx reference tool),
 * non-zero = SLIP (0xC0-delimited, 0xC0->DB DC / 0xDB->DB DD) -- an EXPLORATORY probe
 * for whether the ROM/Realtek image-tool framing is SLIP (pvvx has no SLIP path).
 * Fills @r (entered?, framing, word, raw bytes, overflow count).  Read-only: it issues
 * no erase/write.  Returns 0 when the probe ran (inspect @r->entered), -4 if aborted.
 */
int rtl_dl_probe(int use_slip, uint32_t timeout_ms,
                 int (*should_abort)(void *ctx), void *abort_ctx,
                 struct rtl_dl_result *r);

/* ---- M2: flashloader stub + flash read (non-destructive: SRAM write + flash READ) ---- */

/*
 * Raise the download-link baud.  ONLY 115200 or 1500000 are accepted (the only-board
 * policy -- not the full AmebaD baud table).  Sends the set-baud command (its ACK
 * arrives at the OLD baud, so it is read before switching) then reopens UART9 at @baud.
 * Returns 0 on success, negative on a bad baud / no ACK / reopen failure.
 */
int rtl_dl_set_baud(uint32_t baud);

/*
 * Ensure the AmebaD flashloader stub is resident in the module SRAM at 0x00082000 and
 * the link runs at @target_baud (115200 or 1500000).  Raises the baud; if the stub is
 * not already resident (read-word @0x00082000 != 0x00082021) it uploads it via the ROM
 * SRAM block-transfer (writes SRAM only -- NEVER flash), drops to 115200 for the stub's
 * reboot, re-raises the baud, and re-verifies residency.  Returns 0 on a verified-
 * resident stub, negative on failure.  @should_abort (may be NULL) is polled to cancel.
 */
int rtl_dl_load_flashloader(uint32_t target_baud,
                            int (*should_abort)(void *ctx), void *abort_ctx);

/*
 * Read @nsectors 4 KB flash sectors from @offset (4 KB-aligned, 24-bit) using the
 * flashloader block-read (call rtl_dl_load_flashloader first).  The whole stream is
 * consumed off the wire; the leading min(nsectors*4096, @buf_cap) bytes are copied into
 * @buf.  READ-ONLY -- it never erases or writes flash.  Returns the number of bytes
 * copied into @buf (>=0) on success, or negative on failure / abort.
 */
int rtl_dl_read_flash(uint32_t offset, uint32_t nsectors, uint8_t *buf, uint32_t buf_cap,
                      int (*should_abort)(void *ctx), void *abort_ctx);

/* ---- M4: capacity detection + device checksum + SPI status/ID (READ-ONLY) ---- */

/* Bytes compared at each candidate offset by rtl_dl_detect_size.  One sector, which is
 * the floor: rtl_dl_read_flash() reads whole sectors and the probe derives its sector
 * count as RTL_DL_SIZE_PROBE_LEN / 4096.
 *
 * It was 2 sectors.  The comparison is a full memcmp rather than a hash precisely so
 * there is no collision argument to make, and halving the span does not weaken that --
 * 4096 bytes of km0_boot still either wrap or they do not.  What it does buy is 8 KB of
 * AXI-SRAM (the probe holds two of these buffers, live only during `wifi flash`), and
 * AXI-SRAM is the only memory a bus master can reach (issue #46). */
#define RTL_DL_SIZE_PROBE_LEN  4096u

/* Highest flash offset the download protocol can express: its read / erase / checksum
 * commands all carry a 24-bit offset.  Exposed so callers can reject an out-of-range
 * request up front instead of failing part-way through a long transfer. */
#define RTL_DL_FLASH_LIMIT     0x01000000u

/* Result of rtl_dl_detect_size(). */
struct rtl_dl_size {
	uint32_t size;    /* detected capacity in bytes, or 0 = not determined */
	uint32_t probed;  /* how many candidate offsets were tried (diagnostic) */
	int      blank;   /* 1 = the reference span read all 0xFF, so wrap detection is impossible */
};

/* Result of rtl_dl_flash_jedec() -- EXPERIMENTAL, see that function. */
struct rtl_dl_jedec {
	uint8_t  id[3];       /* manufacturer, memory type, capacity (SPI 0x9F) */
	int      ok;          /* 1 = the RDID probe returned a self-consistent 3-byte ID */
	uint32_t size;        /* capacity decoded from id[2], or 0 if implausible */
};

/*
 * Determine the real flash capacity by ADDRESS WRAP, using only the proven block-read
 * path: a serial flash ignores address bits above its capacity, so a read at `cap`
 * returns offset 0 again.  Reads RTL_DL_SIZE_PROBE_LEN at offset 0 and compares it
 * byte-for-byte against the same span at 1/2/4/8 MB, smallest first; the first full
 * match is the capacity.  Read-only.
 *
 * This is the AUTHORITATIVE capacity source -- rtl_dl_flash_jedec()'s JEDEC ID is only a
 * cross-check, because the 0x21 command shape it relies on is not established by the
 * reference tool.  Returns 0 when the probe completed (@s->size == 0 means no wrap was
 * seen up to 8 MB, i.e. unknown), -2 on a read failure, -3 if the reference span is blank
 * (nothing to match against; @s->blank set).  Call after rtl_dl_load_flashloader().
 */
int rtl_dl_detect_size(struct rtl_dl_size *s, int (*should_abort)(void *ctx), void *abort_ctx);

/*
 * Read the three flash status registers into @sr (SR1/SR2/SR3 = SPI 0x05/0x35/0x15).
 * Read-only, and the ONLY 0x21 command shape the reference tool actually uses, so this
 * is safe to call mid-session.  Returns 0 on success, negative on failure.
 */
int rtl_dl_flash_status(uint8_t sr[3], int (*should_abort)(void *ctx), void *abort_ctx);

/*
 * EXPERIMENTAL: probe the JEDEC ID (SPI RDID 0x9F) through the same 0x21 command.
 * Read-only, but it ASSUMES the command's second byte is a raw SPI opcode and its third
 * byte a read length -- neither is established by the reference tool, which only ever
 * reads status registers one byte at a time.  It asks for one byte first and only then
 * for three, discarding the result unless the two agree, so a wrong guess yields
 * @j->ok == 0 rather than a plausible-looking lie.
 *
 * Because a mis-framed reply can leave the link desynchronised, THE CALLER MUST END THE
 * SESSION afterwards (close the UART and reset the module) rather than issue further
 * commands -- give this its own re-entered session.  Never size a flash operation from
 * the result; that is rtl_dl_detect_size()'s job.  Returns 0 when the probe ran
 * (inspect @j->ok), negative if the link did not answer at all.
 */
int rtl_dl_flash_jedec(struct rtl_dl_jedec *j, int (*should_abort)(void *ctx), void *abort_ctx);

/*
 * Ask the module to checksum @len bytes of flash from @off (both 4 KB-aligned, within
 * the 16 MB protocol limit) and return its 32-bit result in @out.  Read-only.
 *
 * ALGORITHM (established on hardware, board #2): the digest is a plain sum of the range
 * read as 32-bit LITTLE-ENDIAN words.  A 4 KB dump that the module digested as
 * 0xC9AB910F matched the LE word sum of the received bytes exactly, while a byte sum,
 * CRC-32 and Adler-32 all differed.  The host can therefore compute the same value while
 * streaming and verify a backup (or, in M5, a freshly written image) without a re-read.
 *
 * The module reads the whole range before answering, so @timeout_ms must cover it (a
 * full-chip digest takes seconds).  IMPORTANT: a non-zero return POISONS THE SESSION --
 * a late reply would desynchronise the next command, so the caller must stop issuing
 * commands and reset the module.  Make this the LAST operation of a session.
 * Returns 0 on success, -1 on bad arguments, -2 if aborted, -1 on timeout.
 */
int rtl_dl_flash_chksum(uint32_t off, uint32_t len, uint32_t timeout_ms, uint32_t *out,
                        int (*should_abort)(void *ctx), void *abort_ctx);

/* ---- M5: program a host-supplied firmware image (DESTRUCTIVE, the real thing) ---- *
 *
 * M3's single-sector erase/write/verify self-test (`wifi flashtest`, rtl_dl_flash_selftest)
 * lived here until issue #28.  It proved the erase/write/verify path on one gated sector
 * before there was a real writer; rtl_dl_flash_program below has exercised that same path
 * on every real flash since M5, so the test window and its bespoke gates went.  The erase
 * and block-write primitives it shared with the programmer stay private and range-checked.
 */

/* First 8 bytes of an AmebaD km0_boot image -- checked before anything may be written
 * at offset 0.  Confirmed twice: fw/rtl8720/vendor/firmware/km0_boot_all.bin, and the M2 read
 * of this board's own flash. */
#define RTL_DL_KM0_MAGIC_LEN  8u
extern const uint8_t rtl_dl_km0_magic[RTL_DL_KM0_MAGIC_LEN];

/* Per-step result of rtl_dl_flash_program (printed by `wifi flash write`). */
struct rtl_dl_program {
	uint32_t sectors;      /* sectors the range covers */
	uint32_t erased;       /* sectors successfully erased */
	uint32_t written;      /* bytes handed to the block transfer */
	uint32_t host_sum;     /* our digest of the source bytes */
	uint32_t dev_sum;      /* the module's 0x27 digest of the written range */
	uint32_t cap;          /* capacity detected before erasing (0 = unknown) */
	int      cap_known;
	int      erase_ok;
	int      write_ok;
	int      verify_ok;    /* dev_sum == host_sum */
};

/*
 * DESTRUCTIVE: erase [offset, offset+len) and program @data into it, then verify.
 * This is the ONLY public API that writes flash at all -- it is what actually
 * (re)flashes the RTL8720DN's firmware, so read the gates below before changing
 * anything here.
 *
 * Owns the whole session: the caller only needs the RTL link claim
 * (rtl_link_hw_claim(sh, false), see the header note above) + a final rtl8720_reset;
 * do NOT rtl_dl_enter/load first.
 * Two phases, because a flash program leaves the flashloader unresponsive until a
 * power-cycle (established in M3):
 *   Phase 1  enter -> load flashloader -> detect capacity -> erase -> block-transfer
 *   Phase 2  power-cycle, re-enter, re-load -> 0x27 digest over the same range,
 *            compared against our digest of @data.  Verifying by digest instead of
 *            by read-back is what makes a multi-megabyte image practical.
 *
 * GATES -- all of these must hold or nothing is powered up, let alone erased:
 *   1. @offset and @len are 4 KB-aligned, @len != 0, and offset+len <= 2 MB
 *      (RTL_DL_FLASH_WRITE_MAX -- the conservative destructive cap, unchanged
 *      since M3; the wider READ cap deliberately does not apply here).
 *   2. Writing at offset 0 requires @data to start with rtl_dl_km0_magic: the one
 *      thing that would make this module unbootable is a garbage boot image, and
 *      the check costs nothing.
 *   3. The capacity probe runs BEFORE the erase (while there is still data to
 *      detect the address wrap against).  If it reports a capacity, offset+len must
 *      fit in it; if the probe itself FAILS to read, nothing is erased.  A probe
 *      that simply cannot determine a size (a blank chip -- the state a failed
 *      program leaves behind, and the one that has to stay repairable) proceeds
 *      under gate 1's 2 MB bound.
 * The shell adds two more: an explicit `confirm` token, and a re-check of the
 * staged image's digest immediately before the call.
 *
 * Returns 0 only on a full pass (erased, written, and the digests agree).  Negative
 * otherwise: -1 bad arguments / gate 1, -2 session setup, -3 gate 2 (magic),
 * -10 gate 3 could not read the flash to size it, -4 gate 3 says the range is past
 * the detected capacity, -5 erase, -6 block transfer, -7 phase-2 session,
 * -8 digest unavailable, -9 digest MISMATCH.  Nothing has been erased for any code
 * above -5, so those are all safe failures.  On any failure from -5 on,
 * the range is in an indeterminate state and the operation must be REPEATED (erase
 * + write is idempotent as a whole) -- do not treat a failed verify as recoverable
 * by rebooting the module.  @should_abort (may be NULL) is polled to cancel.
 */
int rtl_dl_flash_program(uint32_t offset, const uint8_t *data, uint32_t len,
                         uint32_t hold_us, struct rtl_dl_program *r,
                         int (*should_abort)(void *ctx), void *abort_ctx);

#endif /* APP_RTL8720_FLASH_H */
