/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/*
 * RTL8720DN (AmebaD) on-device UART firmware-download support -- issue #19.
 * See rtl8720_flash.h for the entry mechanism, wiring and protocol references.
 *
 * Milestones, in the order they were proven on board #2: M1 download-mode entry,
 * M2 flashloader stub + flash read (non-destructive), M3 erase/write/verify on one
 * gated test sector, M4 capacity detection + full-chip backup (read-only), M5
 * rtl_dl_flash_program() -- the real thing, which erases and rewrites firmware
 * images including the boot sectors.  Everything destructive still funnels through
 * the two private primitives dl_erase_sectors() / dl_send_flash() and the single
 * range validator dl_flash_range_ok().
 *
 * M1 scope: prove UART download-mode ENTRY via a read-only handshake.  It performs
 * the strap+reset sequence (module OFF first, PD14 held low across the CHIP_EN rising
 * edge so the ROM samples the download strap, brief bounded hold, then PD14 released
 * to UART9_RX) and issues the download read-word command, checking for the framed
 * reply.  NO erase / NO write -- fully reversible, cannot brick (mask-ROM download is
 * re-enterable).  Register-agnostic beyond GPIO + the #17 rtl8720 primitives (never
 * touches the RCC clock tree) -> XIP-safe.  cli-agnostic (ThreadX timing + abort hook).
 */
#include "stm32h7xx_hal.h"   /* GPIO reconfig (PD14 strap: AF11 UART9_RX <-> output low) */
#include "tx_api.h"          /* tx_time_get / tx_thread_sleep (1 tick = 1 ms here) */
#include "rtl8720.h"         /* rtl8720_power/reset + UART9 open/close/read/write (#17) */
#include "timebase.h"        /* udelay: DWT busy-wait for the bounded strap hold */
#include "rtl8720_flash.h"

#include <string.h>

/* PD14 = UART9_RX = RTL8720 PA[7]/UART_LOG_TXD = the UART_DOWNLOAD strap (active-low).
 * Held as a GPIO output low across the CHIP_EN edge, then returned to UART9_RX (AF11). */
#define RTL_DL_STRAP_PORT   GPIOD
#define RTL_DL_STRAP_PIN    GPIO_PIN_14

/* read-word (Program.cs ReadRegs): 0x31 + addr(u32 LE); reply 0x31 + data(u32 LE) + 0x15.
 * 0x00082000 is the flashloader load address; a framed reply here proves download entry
 * (and its data == 0x00082021 would mean the stub is already resident).  Read-only. */
#define RTL_DL_READ_WORD    0x31u
#define RTL_DL_STATUS_OK    0x15u
#define RTL_DL_PROBE_ADDR   0x00082000u

/* ------------------------------------------------------------------ *
 *  strap GPIO (PD14) + cancellable waits
 * ------------------------------------------------------------------ */

/* Drive PD14 as a push-pull output LOW (assert the download strap).  Only called while
 * the RTL8720 is powered off (PC3 low), so there is no contention with PA[7]. */
static void dl_strap_drive_low(void)
{
	GPIO_InitTypeDef io = {0};

	__HAL_RCC_GPIOD_CLK_ENABLE();
	HAL_GPIO_WritePin(RTL_DL_STRAP_PORT, RTL_DL_STRAP_PIN, GPIO_PIN_RESET); /* preset low */
	io.Pin   = RTL_DL_STRAP_PIN;
	io.Mode  = GPIO_MODE_OUTPUT_PP;
	io.Pull  = GPIO_NOPULL;
	io.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(RTL_DL_STRAP_PORT, &io);
	HAL_GPIO_WritePin(RTL_DL_STRAP_PORT, RTL_DL_STRAP_PIN, GPIO_PIN_RESET);
}

/* Return PD14 to a benign INPUT (pulled high, like idle UART RX).  This is the cleanup
 * guarantee: PD14 must never be left driven low, so it cannot fight the RTL8720 driving
 * PA[7] as its LOG-UART TX.  rtl8720_uart_open() later reconfigures it to AF11. */
static void dl_strap_release_input(void)
{
	GPIO_InitTypeDef io = {0};

	__HAL_RCC_GPIOD_CLK_ENABLE();
	io.Pin   = RTL_DL_STRAP_PIN;
	io.Mode  = GPIO_MODE_INPUT;
	io.Pull  = GPIO_PULLUP;
	io.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(RTL_DL_STRAP_PORT, &io);
}

/* Sleep @ms milliseconds, yielding; returns -1 if @should_abort fired, else 0. */
static int dl_wait_ms(uint32_t ms, int (*should_abort)(void *), void *ctx)
{
	ULONG deadline = tx_time_get() + (ULONG)ms;   /* 1 tick = 1 ms */

	while ((int32_t)(tx_time_get() - deadline) < 0) {
		if (should_abort != NULL && should_abort(ctx))
			return -1;
		tx_thread_sleep(1);
	}
	return 0;
}

/* Read whatever arrives (up to @cap) within @timeout_ms; returns the byte count, or
 * -2 if aborted.  A short read (timeout) is normal -- the caller parses what it got. */
static int dl_recv(uint8_t *buf, int cap, uint32_t timeout_ms,
                   int (*should_abort)(void *), void *ctx)
{
	ULONG deadline = tx_time_get() + (ULONG)timeout_ms;
	int n = 0;

	while (n < cap) {
		n += (int)rtl8720_uart_read(buf + n, (size_t)(cap - n));
		if (n >= cap)
			break;
		if (should_abort != NULL && should_abort(ctx))
			return -2;
		if ((int32_t)(tx_time_get() - deadline) >= 0)
			break;                        /* timeout: return what we have */
		tx_thread_sleep(1);
	}
	return n;
}

/* ------------------------------------------------------------------ *
 *  minimal SLIP (exploratory framing probe only -- pvvx uses raw)
 * ------------------------------------------------------------------ */

/* SLIP-encode @n bytes into @out (0xC0-delimited; 0xC0->DB DC, 0xDB->DB DD).
 * Returns the encoded length, or -1 if it would not fit @cap. */
static int slip_encode(const uint8_t *in, int n, uint8_t *out, int cap)
{
	int o = 0;
	int i;

	if (o >= cap) return -1;
	out[o++] = 0xC0u;
	for (i = 0; i < n; i++) {
		uint8_t b = in[i];
		if (b == 0xC0u) {
			if (o + 2 > cap) return -1;
			out[o++] = 0xDBu; out[o++] = 0xDCu;
		} else if (b == 0xDBu) {
			if (o + 2 > cap) return -1;
			out[o++] = 0xDBu; out[o++] = 0xDDu;
		} else {
			if (o >= cap) return -1;
			out[o++] = b;
		}
	}
	if (o >= cap) return -1;
	out[o++] = 0xC0u;
	return o;
}

/* SLIP-decode one frame out of @in into @out; returns the decoded length. */
static int slip_decode(const uint8_t *in, int n, uint8_t *out, int cap)
{
	int o = 0, i, started = 0, esc = 0;

	for (i = 0; i < n; i++) {
		uint8_t b = in[i];
		if (b == 0xC0u) {
			if (started && o > 0) break;  /* closing delimiter of a non-empty frame */
			started = 1;                  /* (re)start on the opening delimiter */
			continue;
		}
		if (!started)
			continue;
		if (esc) {
			b = (b == 0xDCu) ? 0xC0u : (b == 0xDDu) ? 0xDBu : b;
			esc = 0;
		} else if (b == 0xDBu) {
			esc = 1;
			continue;
		}
		if (o < cap)
			out[o++] = b;
	}
	return o;
}

/* ------------------------------------------------------------------ *
 *  public API
 * ------------------------------------------------------------------ */

int rtl_dl_enter(uint32_t hold_us, int (*should_abort)(void *), void *ctx)
{
	/* Close any open UART9 first so the ISR cannot run against the strap reconfig, then
	 * put PD14 in a known-safe INPUT state up front.  This makes the cleanup contract
	 * (PD14 is NEVER returned as a driven-low output) hold on EVERY path -- including the
	 * first abort below -- regardless of PD14's state on entry.  An input / pull-up does
	 * not contend even if the RTL8720 is currently driving PA[7] as its LOG-UART TX. */
	rtl8720_uart_close();
	dl_strap_release_input();

	/* Power the module OFF before driving the strap: if it were running, its PA[7]
	 * (LOG-UART TX) would be a live output and driving PD14 low against it would be a
	 * push-pull contention.  Off first eliminates that. */
	rtl8720_power(false);
	if (dl_wait_ms(100u, should_abort, ctx))
		return -4;                            /* aborted; PD14 is input (released above) */

	dl_strap_drive_low();                         /* PD14 low = download strap asserted */
	if (dl_wait_ms(400u, should_abort, ctx)) {    /* total off >= 500 ms */
		dl_strap_release_input();             /* never leave PD14 driven low */
		return -4;
	}

	/* Release reset: the ROM samples PA[7] (= PD14, low) and enters download mode.
	 * Hold PD14 low a brief, bounded time across/after the edge, then release it to an
	 * input BEFORE the ROM can drive PA[7] as its LOG-UART TX (Wio Terminal holds this
	 * strap low for 500 ms on the same module; our tens of ms is well within that).
	 * M1 on board #2: 2 ms is too short (strap not latched -> normal boot), ~20 ms
	 * latches -> download mode; the caller (`wifi flashprobe`) defaults to 30 ms. */
	rtl8720_power(true);
	if (hold_us)
		udelay(hold_us);
	dl_strap_release_input();

	if (rtl8720_uart_open(RTL8720_UART_LOG, 115200u) != 0)
		return -1;                            /* PD14 already released to input above */

	if (dl_wait_ms(20u, should_abort, ctx)) {     /* let RE/TE + the ROM settle */
		rtl8720_uart_close();
		return -4;
	}
	return 0;
}

int rtl_dl_probe(int use_slip, uint32_t timeout_ms,
                 int (*should_abort)(void *), void *ctx,
                 struct rtl_dl_result *r)
{
	/* read-word 0x00082000: 0x31 + addr(u32 LE). */
	uint8_t cmd[5] = { RTL_DL_READ_WORD,
	                   (uint8_t)RTL_DL_PROBE_ADDR,
	                   (uint8_t)(RTL_DL_PROBE_ADDR >> 8),
	                   (uint8_t)(RTL_DL_PROBE_ADDR >> 16),
	                   (uint8_t)(RTL_DL_PROBE_ADDR >> 24) };
	uint8_t tx[16], dec[32], junk[64];
	const uint8_t *p;
	int txn, n, pn, i;

	memset(r, 0, sizeof(*r));
	r->slip = use_slip ? 1 : 0;

	/* Drop any stale RX (a normal-boot banner, if the strap did not latch) so the parse
	 * starts clean.  Bounded by the ring size (SPSC, no infinite spin). */
	while (rtl8720_uart_read(junk, sizeof(junk)) > 0u)
		;

	if (use_slip) {
		txn = slip_encode(cmd, (int)sizeof(cmd), tx, (int)sizeof(tx));
		if (txn < 0)
			return -1;
		rtl8720_uart_write(tx, (size_t)txn);
	} else {
		rtl8720_uart_write(cmd, sizeof(cmd));
	}

	n = dl_recv(r->raw, (int)sizeof(r->raw), timeout_ms, should_abort, ctx);
	r->overflows = rtl8720_uart_overflows();
	if (n < 0)
		return -4;                            /* aborted */
	r->raw_len = n;

	/* Parse for the framed reply 0x31 <4 data> 0x15 (after SLIP-decode if used). */
	if (use_slip) {
		pn = slip_decode(r->raw, n, dec, (int)sizeof(dec));
		p = dec;
	} else {
		pn = n;
		p = r->raw;
	}
	for (i = 0; i + 6 <= pn; i++) {
		if (p[i] == RTL_DL_READ_WORD && p[i + 5] == RTL_DL_STATUS_OK) {
			r->entered = 1;
			r->word = (uint32_t)p[i + 1] | ((uint32_t)p[i + 2] << 8) |
			          ((uint32_t)p[i + 3] << 16) | ((uint32_t)p[i + 4] << 24);
			break;
		}
	}
	return 0;
}

/* ================================================================== *
 *  M2: flashloader stub upload + flash READ (issue #19)
 *
 *  NON-DESTRUCTIVE: this section writes only the module SRAM (the flashloader stub at
 *  0x00082000) and READS flash (0x20).  It sends NO flash-erase (0x17) and NO flash
 *  block-write.  The block-transfer helper (dl_send_sram) is private and refuses any
 *  destination outside the flashloader SRAM window, so no code path here can write flash.
 *  Protocol reference: pvvx SharpRTL872xTool Program.cs (cited per function).
 * ================================================================== */

/* Flashloader stub, embedded at build time from _ref/ambd/imgtool_flashloader_amebad.bin
 * (see CMakeLists.txt; the Realtek blob is not committed). */
extern const uint8_t  rtl8720_flashloader[];
extern const uint32_t rtl8720_flashloader_len;

#define RTL_DL_ACK          0x06u
#define RTL_DL_CAN          0x18u
#define RTL_DL_SOH          0x01u   /* 128-byte block */
#define RTL_DL_STX          0x02u   /* 1024-byte block */
#define RTL_DL_EOT          0x04u
#define RTL_DL_XMD_START    0x07u
#define RTL_DL_SETBAUD      0x05u
#define RTL_DL_READFLASH    0x20u
#define RTL_DL_GETSTATUS    0x21u   /* M4: SPI status/ID read -- see dl_spi_read */
#define RTL_DL_CHKSUM       0x27u   /* M4: device-side checksum -- see rtl_dl_flash_chksum */
#define RTL_DL_STUB_ADDR    0x00082000u
#define RTL_DL_STUB_ENTRY   0x00082021u   /* read-word @STUB_ADDR when the stub is resident */
#define RTL_DL_BAUD_IDX_115200   0x0Du
#define RTL_DL_BAUD_IDX_1500000  0x18u

/* M3: flash erase/write bounds (destructive -- see rtl_dl_flash_selftest). */
#define RTL_DL_ERASE             0x17u
#define RTL_DL_FLASH_BASE        0x08000000u   /* a block's addr carries this base for flash */
#define RTL_DL_SELFTEST_MIN      0x00100000u   /* flashtest sector must sit past the app (~0xE2000) */

/*
 * Flash range caps.  READ and WRITE are deliberately DIFFERENT (issue #19 M4):
 *
 *  - WRITE_MAX stays at the original conservative 2 MB.  Every destructive path
 *    (dl_erase_sectors / dl_send_flash / rtl_dl_flash_selftest) keeps exactly the
 *    bounds it had before M4 -- the chip may be smaller than we think, and a write
 *    that wrapped past the die onto the boot image would be unrecoverable.
 *  - READ_MAX is the protocol's own ceiling: the read/erase/checksum commands carry
 *    a 24-bit offset, so 16 MB is everything that is addressable at all.  Reads are
 *    harmless (a read past the die just wraps), and M4 has to read past 2 MB both to
 *    detect the real capacity by wrap and to back the whole chip up.
 */
#define RTL_DL_FLASH_READ_MAX    0x01000000u
#define RTL_DL_FLASH_WRITE_MAX   0x00200000u

/* Per-block ACK budgets for the block transfer (see dl_send_blocks). */
#define RTL_DL_ACK_MS_SRAM       1000u
#define RTL_DL_ACK_MS_FLASH      3000u

/* M5: an AmebaD km0_boot image starts with this signature -- gate 2 of
 * rtl_dl_flash_program(), so a non-firmware blob can never land at offset 0. */
const uint8_t rtl_dl_km0_magic[RTL_DL_KM0_MAGIC_LEN] = {
	0x99u, 0x99u, 0x96u, 0x96u, 0x3Fu, 0xCCu, 0x66u, 0xFCu
};

/* ---- the module's 0x27 digest, computed on our side (see rtl8720_flash.h) ---- */

void rtl_dl_digest_init(struct rtl_dl_digest *d)
{
	d->sum = 0u;
	d->acc = 0u;
	d->nacc = 0u;
}

void rtl_dl_digest_add(struct rtl_dl_digest *d, const uint8_t *p, uint32_t n)
{
	uint32_t i;

	/* Byte-wise because callers feed arbitrarily-sized slices (a YMODEM block, a
	 * flash read chunk); the word boundary is carried in acc/nacc across calls. */
	for (i = 0u; i < n; i++) {
		d->acc |= (uint32_t)p[i] << (8u * d->nacc);
		if (++d->nacc == 4u) {
			d->sum += d->acc;
			d->acc = 0u;
			d->nacc = 0u;
		}
	}
}

uint32_t rtl_dl_digest_value(const struct rtl_dl_digest *d)
{
	/* A trailing partial word (only possible on a non-multiple-of-4 range, which the
	 * 4 KB-aligned callers never produce) is folded in as the module would see it. */
	return (d->nacc == 0u) ? d->sum : d->sum + d->acc;
}

/* Read bytes, skipping any that are not @want, until @want is seen (WaitResp).
 * 0 = found, -1 = timeout, -2 = aborted. */
static int dl_wait_byte(uint8_t want, uint32_t timeout_ms, int (*ab)(void *), void *ctx)
{
	ULONG deadline = tx_time_get() + (ULONG)timeout_ms;
	uint8_t b;

	for (;;) {
		if (rtl8720_uart_read(&b, 1u) == 1u) {
			if (b == want)
				return 0;
			continue;
		}
		if (ab != NULL && ab(ctx))
			return -2;
		if ((int32_t)(tx_time_get() - deadline) >= 0)
			return -1;
		tx_thread_sleep(1);
	}
}

/* Read exactly @n bytes (ReadBytes).  0 ok, -1 timeout, -2 aborted. */
static int dl_read_exact(uint8_t *buf, uint32_t n, uint32_t timeout_ms,
                         int (*ab)(void *), void *ctx)
{
	ULONG deadline = tx_time_get() + (ULONG)timeout_ms;
	uint32_t got = 0u;

	while (got < n) {
		got += (uint32_t)rtl8720_uart_read(buf + got, (size_t)(n - got));
		if (got >= n)
			break;
		if (ab != NULL && ab(ctx))
			return -2;
		if ((int32_t)(tx_time_get() - deadline) >= 0)
			return -1;
		tx_thread_sleep(1);
	}
	return 0;
}

/* Send @cmd then wait for the ACK 0x06 (WriteCmd).  0 ok, -1 timeout, -2 aborted. */
static int dl_send_wait_ack(const uint8_t *cmd, uint32_t n, uint32_t timeout_ms,
                            int (*ab)(void *), void *ctx)
{
	rtl8720_uart_write(cmd, (size_t)n);
	return dl_wait_byte(RTL_DL_ACK, timeout_ms, ab, ctx);
}

/* Drain RX until it has been quiet for @quiet_ms (bounded by @max_ms).  Unlike an
 * instantaneous drain this also absorbs late chatter -- e.g. a flashloader still emitting
 * bytes just after a flash program completes -- and gives the flash a moment to settle,
 * so the NEXT command starts from a truly clean boundary.  Call it BEFORE sending a
 * command (never after, or it would eat the reply). */
static void dl_drain_quiet(uint32_t quiet_ms, uint32_t max_ms)
{
	uint8_t junk[64];
	ULONG start = tx_time_get();
	ULONG last  = start;

	for (;;) {
		if (rtl8720_uart_read(junk, sizeof(junk)) > 0u) {
			last = tx_time_get();
		} else {
			if ((int32_t)(tx_time_get() - last) >= (int32_t)quiet_ms)
				return;
			if ((int32_t)(tx_time_get() - start) >= (int32_t)max_ms)
				return;
			tx_thread_sleep(1);
		}
	}
}

/* read-word @addr (ReadRegs, Program.cs:603-641): 0x31 + addr(u32 LE) -> 0x31 + data + 0x15.
 * Reads exactly the framed reply (skip to the 0x31 echo, then 5 bytes).  Read-only.
 * 0 ok (*out set), negative on failure. */
static int dl_read_word(uint32_t addr, uint32_t *out, int (*ab)(void *), void *ctx)
{
	uint8_t cmd[5], rp[5];
	int rr;

	dl_drain_quiet(10u, 200u);              /* start from a clean, quiet boundary */
	cmd[0] = 0x31u;
	cmd[1] = (uint8_t)addr;        cmd[2] = (uint8_t)(addr >> 8);
	cmd[3] = (uint8_t)(addr >> 16); cmd[4] = (uint8_t)(addr >> 24);
	rtl8720_uart_write(cmd, sizeof(cmd));

	rr = dl_wait_byte(0x31u, 500u, ab, ctx);
	if (rr)
		return rr;                          /* -1 timeout / -2 aborted (propagate) */
	rr = dl_read_exact(rp, 5u, 500u, ab, ctx);
	if (rr)
		return rr;
	if (rp[4] != 0x15u)
		return -1;
	*out = (uint32_t)rp[0] | ((uint32_t)rp[1] << 8) |
	       ((uint32_t)rp[2] << 16) | ((uint32_t)rp[3] << 24);
	return 0;
}

/*
 * PRIVATE block-transfer core (SendXmodem, Program.cs:783-868): 0x07 -> blocks -> 0x04.
 * block = [cmd][seq][~seq][addr:4LE][data:N(0xFF pad)][csum], csum = (addr4+data) & 0xFF.
 * @addr32 is the full destination address of the FIRST byte (SRAM as-is, or flash with the
 * 0x08000000 base); each block uses addr32 + off.  Callers (dl_send_sram / dl_send_flash)
 * own the destination validation -- this core does none.  0 ok, -2 aborted, -3 no ACK.
 *
 * @ack_ms is the per-block ACK budget.  It is a parameter rather than a constant only so
 * the flash path can wait longer than the SRAM path for a slow page program (M5): the
 * block/ACK SEQUENCE ITSELF IS UNCHANGED from the M2/M3-proven code.
 *
 * DELIBERATELY NO RETRY, unlike the reference tool (Program.cs:399 passes retry=3).  A
 * resent block would be harmless to the flash -- every block carries its own destination
 * address, so re-programming identical bytes changes no cells -- but it is NOT harmless to
 * the protocol: dl_wait_byte() returns on the first ACK it sees, so an ACK that merely
 * arrived late would satisfy the retry, and every subsequent block would then be matched
 * against the previous block's ACK.  A whole program run is idempotent, so the recovery
 * for a lost ACK is to repeat the operation, not to patch it up mid-stream.  If real
 * hardware ever shows ACK timeouts at scale, add "dl_drain_quiet() then resend" as its
 * own reviewed change -- do not sneak it in here.
 */
static int dl_send_blocks(const uint8_t *data, uint32_t len, uint32_t addr32,
                          uint32_t ack_ms, int (*ab)(void *), void *ctx)
{
	static uint8_t pkt[3 + 4 + 1024 + 1];   /* function-static: off the 4 KB shell stack */
	uint8_t start = RTL_DL_XMD_START, eot = RTL_DL_EOT;
	uint32_t off = 0u;
	uint8_t seq = 1u;

	if (dl_send_wait_ack(&start, 1u, 500u, ab, ctx))
		return -2;

	while (off < len) {
		uint32_t rem  = len - off;
		uint32_t N    = (rem > 128u) ? 1024u : 128u;
		uint32_t take = (rem < N) ? rem : N;
		uint32_t addr = addr32 + off;
		uint32_t i, sum = 0u;
		int rr;

		pkt[0] = (rem > 128u) ? RTL_DL_STX : RTL_DL_SOH;
		pkt[1] = seq;
		pkt[2] = (uint8_t)(0xFFu - seq);
		pkt[3] = (uint8_t)addr;         pkt[4] = (uint8_t)(addr >> 8);
		pkt[5] = (uint8_t)(addr >> 16); pkt[6] = (uint8_t)(addr >> 24);
		for (i = 0u; i < N; i++)
			pkt[7 + i] = (i < take) ? data[off + i] : 0xFFu;
		for (i = 3u; i < 7u + N; i++)   /* csum over addr(4) + data(N) */
			sum += pkt[i];
		pkt[7 + N] = (uint8_t)(sum & 0xFFu);

		rr = dl_send_wait_ack(pkt, 7u + N + 1u, ack_ms, ab, ctx);
		if (rr)
			return (rr == -2) ? -2 : -3;
		seq = (uint8_t)(seq + 1u);
		off += take;
	}

	if (dl_send_wait_ack(&eot, 1u, 1000u, ab, ctx))
		return -2;
	return 0;
}

/* PRIVATE, SRAM-ONLY (M2): refuse any address but the flashloader SRAM window -- so this
 * path can never write flash.  0 ok, -1 bad args, else dl_send_blocks code. */
static int dl_send_sram(const uint8_t *data, uint32_t len, uint32_t sram_addr,
                        int (*ab)(void *), void *ctx)
{
	if (sram_addr != RTL_DL_STUB_ADDR)          /* only the flashloader SRAM window */
		return -1;
	if (len == 0u || len > 0x10000u)            /* the stub is ~4.7 KB */
		return -1;
	/* 1000 ms per block: the M2-proven budget, kept exactly as it was. */
	return dl_send_blocks(data, len, sram_addr, RTL_DL_ACK_MS_SRAM, ab, ctx);
}

/*
 * Central, overflow-safe range validator for every flash op (read / erase / write).
 * Requires @off and @len 4 KB-aligned, non-zero @len, and off + len <= @max (the last
 * check written as `len <= max - off` so it cannot wrap).  Returns 1 if OK.
 *
 * @max is passed in rather than hard-coded so the destructive callers keep the
 * conservative RTL_DL_FLASH_WRITE_MAX while reads may use RTL_DL_FLASH_READ_MAX --
 * every call site names its own ceiling, and a read cap can never widen a write.
 */
static int dl_flash_range_ok(uint32_t off, uint32_t len, uint32_t max)
{
	if ((off & 0xFFFu) != 0u || (len & 0xFFFu) != 0u || len == 0u)
		return 0;
	if (off >= max)
		return 0;
	if (len > max - off)                        /* overflow-safe off + len <= max */
		return 0;
	return 1;
}

/* PRIVATE flash block-write (M3): the block-transfer with the flash base OR'd into the
 * address.  Bounded by dl_flash_range_ok.  No public general flash-write API exists -- the
 * only caller is rtl_dl_flash_selftest, which additionally gates on an erasable sector. */
static int dl_send_flash(const uint8_t *data, uint32_t len, uint32_t flash_off,
                         int (*ab)(void *), void *ctx)
{
	if (!dl_flash_range_ok(flash_off, len, RTL_DL_FLASH_WRITE_MAX))
		return -1;
	/* 3000 ms per block rather than the SRAM path's 1000 ms: M3 only ever programmed
	 * 4 KB (4 blocks), so a slow page program part-way through a megabyte-scale image
	 * is untested territory and there is no retry to fall back on. */
	return dl_send_blocks(data, len, (flash_off & 0x00FFFFFFu) | RTL_DL_FLASH_BASE,
	                      RTL_DL_ACK_MS_FLASH, ab, ctx);
}

/* PRIVATE flash erase (M3, EraseSectorsFlash Program.cs:371-398): one 0x17 command per 4 KB
 * sector.  Bounded by dl_flash_range_ok.  0 ok, -1 bad args, -2 aborted / no ACK. */
static int dl_erase_sectors(uint32_t offset, uint32_t nsectors, int (*ab)(void *), void *ctx)
{
	uint32_t i;

	if (nsectors == 0u || nsectors > 0x10000u)          /* bound the multiply below */
		return -1;
	if (!dl_flash_range_ok(offset, nsectors * 4096u, RTL_DL_FLASH_WRITE_MAX))
		return -1;
	for (i = 0u; i < nsectors; i++) {
		uint32_t off = offset + i * 4096u;
		uint8_t pkt[6];

		pkt[0] = RTL_DL_ERASE;
		pkt[1] = (uint8_t)off; pkt[2] = (uint8_t)(off >> 8); pkt[3] = (uint8_t)(off >> 16);
		pkt[4] = 0x01u; pkt[5] = 0x00u;                 /* count = 1 (u16 LE) */
		if (dl_send_wait_ack(pkt, sizeof(pkt), 2000u, ab, ctx))
			return -2;
	}
	return 0;
}

int rtl_dl_set_baud(uint32_t baud)
{
	uint8_t pkt[2];

	pkt[0] = RTL_DL_SETBAUD;
	if (baud == 115200u)
		pkt[1] = RTL_DL_BAUD_IDX_115200;
	else if (baud == 1500000u)
		pkt[1] = RTL_DL_BAUD_IDX_1500000;
	else
		return -1;                          /* only-board policy: 115200 / 1500000 only */

	/* The ACK comes back at the OLD baud, so wait for it before reopening at the new baud. */
	if (dl_send_wait_ack(pkt, 2u, 500u, NULL, NULL))
		return -2;
	if (rtl8720_uart_open(RTL8720_UART_LOG, baud) != 0)
		return -3;
	return 0;
}

int rtl_dl_load_flashloader(uint32_t target_baud, int (*ab)(void *), void *ctx)
{
	uint32_t word = 0u;
	int rc;

	if (target_baud != 115200u && target_baud != 1500000u)
		return -1;

	/* Raise to the working baud.  Skip when the target is 115200: we entered download at
	 * 115200 and the ROM does not ACK a same-baud set (pvvx omits it too). */
	if (target_baud != 115200u && rtl_dl_set_baud(target_baud))
		return -2;

	rc = dl_read_word(RTL_DL_STUB_ADDR, &word, ab, ctx);
	if (rc == -2)
		return -7;                              /* aborted */
	if (rc == 0 && word == RTL_DL_STUB_ENTRY)
		return 0;                               /* stub already resident */

	/* Upload the stub to SRAM (writes SRAM only), then it reboots to 115200. */
	if (dl_send_sram(rtl8720_flashloader, rtl8720_flashloader_len, RTL_DL_STUB_ADDR, ab, ctx))
		return -3;
	if (rtl8720_uart_open(RTL8720_UART_LOG, 115200u) != 0)
		return -4;
	tx_thread_sleep(50);                            /* let the stub boot + re-init its UART */
	if (target_baud != 115200u && rtl_dl_set_baud(target_baud))
		return -5;

	rc = dl_read_word(RTL_DL_STUB_ADDR, &word, ab, ctx);
	if (rc == -2)
		return -7;
	if (rc != 0 || word != RTL_DL_STUB_ENTRY)
		return -6;                              /* stub did not come up resident */
	return 0;
}

/* ================================================================== *
 *  M4: capacity detection + device checksum + SPI status/ID (issue #19)
 *
 *  READ-ONLY.  Nothing here erases or writes flash: the only commands issued are
 *  0x20 (block read, already proven in M2), 0x21 (SPI status/ID read) and 0x27
 *  (device-side checksum).  The 0x21 SPI opcodes used (0x05/0x35/0x15 RDSR1..3 and
 *  0x9F RDID) are all read-only opcodes, so even a mis-framed 0x21 cannot modify
 *  flash -- the worst case is a garbage reply.
 * ================================================================== */

/*
 * SPI status/ID read (0x21, GetFlashStatus Program.cs:426-455): [0x21][opcode][n]
 * -> ACK byte 0x21 -> @n bytes.
 *
 * CONFIDENCE: the reference tool only ever issues this with n == 1 and opcode
 * 0x05/0x35/0x15 (RDSR1/2/3), so ONLY that shape is confirmed.  Whether byte 2 is a
 * raw SPI opcode (making 0x9F/RDID work) and whether byte 3 is really a length are
 * NOT established by the reference -- callers must treat anything else as an
 * experimental probe whose failure ends the session (see rtl_dl_flash_jedec).
 * 0 ok, -1 timeout / no ACK, -2 aborted.
 */
static int dl_spi_read(uint8_t opcode, uint8_t *out, uint32_t n,
                       int (*ab)(void *), void *ctx)
{
	uint8_t pkt[3];
	int rr;

	if (n == 0u || n > 8u)
		return -1;
	dl_drain_quiet(10u, 200u);              /* start from a clean, quiet boundary */
	pkt[0] = RTL_DL_GETSTATUS;
	pkt[1] = opcode;
	pkt[2] = (uint8_t)n;
	rtl8720_uart_write(pkt, sizeof(pkt));

	rr = dl_wait_byte(RTL_DL_GETSTATUS, 500u, ab, ctx);   /* the command byte is the ACK */
	if (rr)
		return rr;
	return dl_read_exact(out, n, 500u, ab, ctx);
}

int rtl_dl_flash_chksum(uint32_t off, uint32_t len, uint32_t timeout_ms, uint32_t *out,
                        int (*ab)(void *), void *ctx)
{
	uint8_t pkt[7], rp[4];
	int rr;

	if (out == NULL)
		return -1;
	/* Same 4 KB-aligned, overflow-safe bounds as a read: this only reads flash. */
	if (!dl_flash_range_ok(off, len, RTL_DL_FLASH_READ_MAX))
		return -1;

	dl_drain_quiet(10u, 200u);
	pkt[0] = RTL_DL_CHKSUM;
	pkt[1] = (uint8_t)off;  pkt[2] = (uint8_t)(off >> 8);  pkt[3] = (uint8_t)(off >> 16);
	pkt[4] = (uint8_t)len;  pkt[5] = (uint8_t)(len >> 8);  pkt[6] = (uint8_t)(len >> 16);
	rtl8720_uart_write(pkt, sizeof(pkt));

	/* The module reads the whole range before replying, so @timeout_ms must cover a
	 * full-chip digest.  A timeout here POISONS THE SESSION: the reply may still land
	 * later and would desynchronise the next command (dl_drain_quiet only absorbs
	 * 200 ms of late chatter), so callers must treat a non-zero return as "stop using
	 * this flashloader session" -- close the UART and reset the module. */
	rr = dl_wait_byte(RTL_DL_CHKSUM, timeout_ms, ab, ctx);
	if (rr)
		return rr;
	rr = dl_read_exact(rp, sizeof(rp), 1000u, ab, ctx);
	if (rr)
		return rr;
	*out = (uint32_t)rp[0] | ((uint32_t)rp[1] << 8) |
	       ((uint32_t)rp[2] << 16) | ((uint32_t)rp[3] << 24);
	return 0;
}

int rtl_dl_flash_status(uint8_t sr[3], int (*ab)(void *), void *ctx)
{
	/* The reference tool's exact usage: one status register per command, one byte back
	 * (Program.cs:426-455).  This shape is confirmed, so it is safe mid-session -- it is
	 * deliberately kept SEPARATE from the experimental RDID probe below, so a caller can
	 * read the status registers without risking a desynchronising guess. */
	static const uint8_t sr_op[3] = { 0x05u, 0x35u, 0x15u };
	int i;

	if (sr == NULL)
		return -1;
	for (i = 0; i < 3; i++)
		if (dl_spi_read(sr_op[i], &sr[i], 1u, ab, ctx) != 0)
			return -2;
	return 0;
}

int rtl_dl_flash_jedec(struct rtl_dl_jedec *j, int (*ab)(void *), void *ctx)
{
	uint8_t b;

	if (j == NULL)
		return -1;
	memset(j, 0, sizeof(*j));

	/*
	 * EXPERIMENTAL (see the header): RDID assumes byte 2 of the 0x21 command is a raw
	 * SPI opcode and byte 3 a read length -- neither is established by the reference
	 * tool.  Ask for one byte (the manufacturer ID) first; only if that looks real ask
	 * for the full three, and keep the answer only when the two agree, so a wrong guess
	 * about the command shape leaves j->ok == 0 instead of fabricating an ID.  Any exit
	 * from here may leave the link desynchronised, which is why the caller must give
	 * this probe a session of its own and end that session afterwards.
	 */
	if (dl_spi_read(0x9Fu, &b, 1u, ab, ctx) != 0)
		return -2;                      /* the link did not answer at all */
	if (b == 0x00u || b == 0xFFu)
		return 0;                       /* not a plausible manufacturer ID */
	if (dl_spi_read(0x9Fu, j->id, 3u, ab, ctx) != 0)
		return 0;
	if (j->id[0] != b)
		return 0;                       /* n==3 not honoured / inconsistent -> discard */
	j->ok = 1;
	/* Capacity byte: JEDEC encodes it as log2(bytes), e.g. 0x15 = 2 MB, 0x16 = 4 MB. */
	if (j->id[2] >= 0x10u && j->id[2] <= 0x19u)
		j->size = 1u << j->id[2];
	return 0;
}

int rtl_dl_detect_size(struct rtl_dl_size *s, int (*ab)(void *), void *ctx)
{
	/* Two reference sectors (8 KB) read in ONE command, plus the candidate's 8 KB.
	 * Function-static: off the 4 KB shell stack. */
	static uint8_t ref[RTL_DL_SIZE_PROBE_LEN];
	static uint8_t cand[RTL_DL_SIZE_PROBE_LEN];
	static const uint32_t cands[] = {
		0x00100000u, 0x00200000u, 0x00400000u, 0x00800000u,
	};
	uint32_t i;
	int rc, all_ff;

	if (s == NULL)
		return -1;
	memset(s, 0, sizeof(s[0]));

	/* Reference: the first 8 KB.  It must NOT be blank, or every candidate on a blank
	 * chip would "match" and we would report the smallest size.  The module's km0_boot
	 * lives here (verified on hardware in M2), so this is data in practice. */
	rc = rtl_dl_read_flash(0u, RTL_DL_SIZE_PROBE_LEN / 4096u, ref, sizeof(ref), ab, ctx);
	if (rc < (int)sizeof(ref))
		return -2;
	all_ff = 1;
	for (i = 0u; i < sizeof(ref); i++)
		if (ref[i] != 0xFFu) { all_ff = 0; break; }
	if (all_ff) {
		s->blank = 1;
		return -3;                      /* indeterminate: nothing to match against */
	}

	/*
	 * Wrap detection: a serial flash ignores the address bits above its capacity, so
	 * reading at `cap` returns offset 0 again.  Walk the candidates smallest-first and
	 * take the first full 8 KB match -- full compare, not a hash, so there is no
	 * collision argument to make.  Reading a candidate beyond the die is harmless.
	 */
	for (i = 0u; i < sizeof(cands) / sizeof(cands[0]); i++) {
		rc = rtl_dl_read_flash(cands[i], RTL_DL_SIZE_PROBE_LEN / 4096u,
		                       cand, sizeof(cand), ab, ctx);
		if (rc < (int)sizeof(cand))
			return -2;
		s->probed = i + 1u;
		if (memcmp(ref, cand, sizeof(ref)) == 0) {
			s->size = cands[i];
			return 0;
		}
	}
	return 0;                               /* no wrap seen: size stays 0 = unknown */
}

int rtl_dl_read_flash(uint32_t offset, uint32_t nsectors, uint8_t *buf, uint32_t buf_cap,
                      int (*ab)(void *), void *ctx)
{
	static uint8_t blk[2 + 1024 + 1];   /* seq + ~seq + 1024 data + csum (off the stack) */
	uint8_t hdr[6];
	uint32_t total_blocks, i, copied = 0u;

	if (nsectors == 0u || nsectors > 0x10000u)             /* bound the multiply */
		return -1;
	/* READ_MAX (16 MB) rather than the destructive cap: reads are harmless, and M4's
	 * wrap-based capacity detection + full-chip backup both need to read past 2 MB.
	 * The range check also keeps nsectors <= 0x1000, so it always fits the u16 count
	 * field in the command header below. */
	if (!dl_flash_range_ok(offset, nsectors * 4096u, RTL_DL_FLASH_READ_MAX))
		return -1;
	total_blocks = nsectors * 4u;                          /* module streams 4 x 1024 / sector */

	dl_drain_quiet(10u, 200u);                             /* clean, quiet boundary */
	hdr[0] = RTL_DL_READFLASH;
	hdr[1] = (uint8_t)offset;  hdr[2] = (uint8_t)(offset >> 8);  hdr[3] = (uint8_t)(offset >> 16);
	hdr[4] = (uint8_t)nsectors; hdr[5] = (uint8_t)(nsectors >> 8);
	rtl8720_uart_write(hdr, sizeof(hdr));

	for (i = 0u; i < total_blocks; i++) {
		uint32_t j, sum = 0u, chunk;

		if (dl_wait_byte(RTL_DL_STX, 2000u, ab, ctx))          /* 0x02 block header */
			return -2;
		if (dl_read_exact(blk, 2u + 1024u + 1u, 2000u, ab, ctx))  /* seq,~seq,1024,csum */
			return -2;
		if (blk[0] != (uint8_t)((i + 1u) & 0xFFu) ||
		    blk[1] != (uint8_t)(blk[0] ^ 0xFFu))
			return -3;                                     /* bad block sequence */
		for (j = 0u; j < 1024u; j++)
			sum += blk[2 + j];
		if ((uint8_t)(sum & 0xFFu) != blk[2 + 1024u]) {
			uint8_t can = RTL_DL_CAN;
			rtl8720_uart_write(&can, 1u);                  /* abort the stream */
			return -3;
		}

		if (copied < buf_cap && buf != NULL) {                 /* keep leading buf_cap bytes */
			chunk = buf_cap - copied;
			if (chunk > 1024u)
				chunk = 1024u;
			memcpy(buf + copied, blk + 2, chunk);
			copied += chunk;
		}

		if (i + 1u < total_blocks) {
			uint8_t ack = RTL_DL_ACK;
			rtl8720_uart_write(&ack, 1u);                  /* continue: bare ACK, no wait */
		} else {
			uint8_t can = RTL_DL_CAN;                       /* last block: CAN, wait its ACK */
			if (dl_send_wait_ack(&can, 1u, 1000u, ab, ctx))
				return -2;
		}
	}
	return (int)copied;
}

int rtl_dl_flash_selftest(uint32_t offset, uint32_t hold_us, struct rtl_dl_selftest *r,
                          int (*ab)(void *), void *ctx)
{
	static uint8_t buf[4096];   /* one sector; function-static, off the 4 KB shell stack */
	uint32_t i;
	int rc, all_ff, all_pat, attempt;

	memset(r, 0, sizeof(*r));

	/* Range gate: only a 4 KB-aligned sector in [SELFTEST_MIN, WRITE_MAX) -- past the app,
	 * within the conservative 2 MB destructive cap (unchanged by M4's wider READ cap).
	 * `offset > MAX - 4096` is overflow-safe. */
	if ((offset & 0xFFFu) != 0u ||
	    offset < RTL_DL_SELFTEST_MIN ||
	    offset > RTL_DL_FLASH_WRITE_MAX - 4096u)
		return -1;

	/* Phase 1: enter download + load the flashloader (this routine owns the session). */
	if (rtl_dl_enter(hold_us, ab, ctx) != 0 ||
	    rtl_dl_load_flashloader(1500000u, ab, ctx) != 0)
		return -2;

	/* Content gate: read the whole sector; only proceed if it is erasable -- all 0xFF
	 * (unused) or exactly our own test pattern (a previous DIRTY run then self-heals).
	 * Any foreign data => refuse to erase/write (protects boot/app/other data). */
	rc = rtl_dl_read_flash(offset, 1u, buf, sizeof(buf), ab, ctx);
	if (rc < (int)sizeof(buf))
		return -2;
	memcpy(r->found, buf, sizeof(r->found));
	all_ff = 1; all_pat = 1;
	for (i = 0u; i < sizeof(buf); i++) {
		if (buf[i] != 0xFFu)               all_ff = 0;
		if (buf[i] != (uint8_t)(i & 0xFFu)) all_pat = 0;
	}
	if (!all_ff && !all_pat)
		return -3;                          /* foreign data */
	r->gate_ok = 1;
	r->gate_was_ff = all_ff;

	/* Erase -> verify all 0xFF. */
	if (dl_erase_sectors(offset, 1u, ab, ctx))
		return -4;
	rc = rtl_dl_read_flash(offset, 1u, buf, sizeof(buf), ab, ctx);
	if (rc < (int)sizeof(buf))
		return -4;
	for (i = 0u; i < sizeof(buf); i++)
		if (buf[i] != 0xFFu)
			return -4;
	r->erase_ok = 1;

	/* Write the i&0xFF test pattern -> read back -> verify.  From here the sector holds
	 * data, so any failure marks it dirty (re-running flashtest self-heals via the pattern
	 * gate).  Distinct codes split the sub-steps: -5 send, -7 read-back, -8 mismatch. */
	for (i = 0u; i < sizeof(buf); i++)
		buf[i] = (uint8_t)(i & 0xFFu);
	if (dl_send_flash(buf, sizeof(buf), offset, ab, ctx)) {
		r->dirty = 1;
		return -5;
	}
	r->dirty = 1;                               /* pattern now on flash */

	/* Phase 2: the flashloader goes unresponsive after a flash program (only a power-cycle
	 * revives it), so re-enter download + reload the stub.  The flash content persists
	 * across the reset; then read it back to verify. */
	if (rtl_dl_enter(hold_us, ab, ctx) != 0 ||
	    rtl_dl_load_flashloader(1500000u, ab, ctx) != 0)
		return -7;
	rc = rtl_dl_read_flash(offset, 1u, buf, sizeof(buf), ab, ctx);
	r->rc_detail = rc;
	r->read_attempts = 1;
	if (rc < (int)sizeof(buf)) {
		r->dirty = 1;
		return -7;
	}
	for (i = 0u; i < sizeof(buf); i++)
		if (buf[i] != (uint8_t)(i & 0xFFu)) {
			r->dirty = 1;
			return -8;
		}
	r->write_ok = 1;

	/* Restore: re-erase back to all 0xFF (retry once). */
	for (attempt = 0; attempt < 2; attempt++) {
		if (dl_erase_sectors(offset, 1u, ab, ctx))
			continue;
		rc = rtl_dl_read_flash(offset, 1u, buf, sizeof(buf), ab, ctx);
		if (rc < (int)sizeof(buf))
			continue;
		all_ff = 1;
		for (i = 0u; i < sizeof(buf); i++)
			if (buf[i] != 0xFFu) { all_ff = 0; break; }
		if (all_ff) {
			r->restore_ok = 1;
			break;
		}
	}
	if (!r->restore_ok)
		return -6;                          /* left with data; re-running flashtest self-heals */
	r->dirty = 0;                               /* sector restored to 0xFF */
	return 0;
}

/* ------------------------------------------------------------------ *
 *  M5: program a host-supplied firmware image (the real write path)
 * ------------------------------------------------------------------ */

/* How long to allow the module for its 0x27 digest over @len bytes.  It reads the
 * whole range before answering (M4 measured ~30 s for the full 2 MB), so scale with
 * the size and keep a floor for small ranges. */
static uint32_t dl_chksum_budget_ms(uint32_t len)
{
	uint32_t ms = 10000u + (len >> 6);      /* 2 MB -> ~42 s */

	return (ms > 120000u) ? 120000u : ms;
}

int rtl_dl_flash_program(uint32_t offset, const uint8_t *data, uint32_t len,
                         uint32_t hold_us, struct rtl_dl_program *r,
                         int (*ab)(void *), void *ctx)
{
	struct rtl_dl_digest dg;
	struct rtl_dl_size   sz;
	int                  rc;

	memset(r, 0, sizeof(*r));
	if (data == NULL)
		return -1;

	/* Gate 1: the same central, overflow-safe validator every destructive path uses,
	 * against the conservative 2 MB write cap (NOT the wider read cap). */
	if (!dl_flash_range_ok(offset, len, RTL_DL_FLASH_WRITE_MAX))
		return -1;
	r->sectors = len / 4096u;

	/* Gate 2: refuse to put anything but an AmebaD boot image at offset 0. */
	if (offset == 0u &&
	    memcmp(data, rtl_dl_km0_magic, RTL_DL_KM0_MAGIC_LEN) != 0)
		return -3;

	/* Our digest of exactly the bytes we are about to write -- compared in phase 2
	 * against the module's own, so the verify needs no read-back. */
	rtl_dl_digest_init(&dg);
	rtl_dl_digest_add(&dg, data, len);
	r->host_sum = rtl_dl_digest_value(&dg);

	/* --- Phase 1: session, capacity, erase, write --- */
	if (rtl_dl_enter(hold_us, ab, ctx) != 0 ||
	    rtl_dl_load_flashloader(1500000u, ab, ctx) != 0)
		return -2;

	/*
	 * Gate 3: capacity, BEFORE erasing -- the wrap probe needs real data at offset 0 to
	 * match against, so it cannot run on a chip we have already blanked.
	 *
	 * A READ FAILURE HERE IS FATAL (rc == -2, which also covers a Ctrl+C abort): the
	 * link just failed to answer a read, and the very next thing this function would do
	 * is erase.  Never erase on the strength of a probe that did not work.
	 *
	 * The two "capacity not determined" outcomes are allowed through, because gate 1 has
	 * already bounded the write to 2 MB -- the capacity M4 measured on this board -- and
	 * refusing them would break the recovery path:
	 *   rc == -3  the first 8 KB is blank, so there is nothing to detect a wrap against.
	 *             THIS IS EXACTLY THE STATE A FAILED PROGRAM LEAVES BEHIND, and re-running
	 *             this function is how it is repaired.  Rejecting it would mean a botched
	 *             write could never be fixed.
	 *   size == 0 no wrap seen up to 8 MB, i.e. the chip is larger than anything we would
	 *             write anyway.
	 */
	rc = rtl_dl_detect_size(&sz, ab, ctx);
	if (rc == -2)
		return -10;                     /* probe failed; nothing erased */
	if (rc == 0 && sz.size != 0u) {
		r->cap_known = 1;
		r->cap = sz.size;
		if (offset >= sz.size || len > sz.size - offset)
			return -4;
	}

	if (dl_erase_sectors(offset, r->sectors, ab, ctx))
		return -5;
	r->erased = r->sectors;
	r->erase_ok = 1;

	if (dl_send_flash(data, len, offset, ab, ctx))
		return -6;
	r->written = len;
	r->write_ok = 1;

	/* --- Phase 2: the flashloader is wedged by the program (M3), so power-cycle,
	 * re-enter and ask the module to digest what it now holds. --- */
	if (rtl_dl_enter(hold_us, ab, ctx) != 0 ||
	    rtl_dl_load_flashloader(1500000u, ab, ctx) != 0)
		return -7;

	/* LAST operation of the session: a timeout here may still be answered later and
	 * would desynchronise anything that followed, so nothing does. */
	if (rtl_dl_flash_chksum(offset, len, dl_chksum_budget_ms(len), &r->dev_sum,
	                        ab, ctx) != 0)
		return -8;
	if (r->dev_sum != r->host_sum)
		return -9;
	r->verify_ok = 1;
	return 0;
}
