/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/*
 * RTL8720DN (AmebaD) on-device UART firmware-download support -- issue #19, M1.
 * See rtl8720_flash.h for the entry mechanism, wiring and protocol references.
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
#define RTL_DL_STUB_ADDR    0x00082000u
#define RTL_DL_STUB_ENTRY   0x00082021u   /* read-word @STUB_ADDR when the stub is resident */
#define RTL_DL_BAUD_IDX_115200   0x0Du
#define RTL_DL_BAUD_IDX_1500000  0x18u

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

/* read-word @addr (ReadRegs, Program.cs:603-641): 0x31 + addr(u32 LE) -> 0x31 + data + 0x15.
 * Reads exactly the framed reply (skip to the 0x31 echo, then 5 bytes).  Read-only.
 * 0 ok (*out set), negative on failure. */
static int dl_read_word(uint32_t addr, uint32_t *out, int (*ab)(void *), void *ctx)
{
	uint8_t cmd[5], rp[5], junk[64];
	int rr;

	while (rtl8720_uart_read(junk, sizeof(junk)) > 0u)   /* start from a clean boundary */
		;
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
 * PRIVATE, SRAM-ONLY block transfer (SendXmodem, Program.cs:783-868): 0x07 -> blocks -> 0x04.
 * block = [cmd][seq][~seq][addr:4LE][data:N(0xFF pad)][csum], csum = (addr4+data) & 0xFF.
 *
 * SAFETY (M2 non-destructive): @sram_addr is validated to be exactly the flashloader SRAM
 * window and MUST NOT carry the flash base (0x08000000).  This function therefore cannot
 * write flash; the flash-destination block-write is deliberately NOT implemented in M2.
 * 0 ok, -1 bad args, -2 aborted, -3 no ACK.
 */
static int dl_send_sram(const uint8_t *data, uint32_t len, uint32_t sram_addr,
                        int (*ab)(void *), void *ctx)
{
	static uint8_t pkt[3 + 4 + 1024 + 1];   /* function-static: off the 4 KB shell stack */
	uint8_t start = RTL_DL_XMD_START, eot = RTL_DL_EOT;
	uint32_t off = 0u;
	uint8_t seq = 1u;

	if (sram_addr != RTL_DL_STUB_ADDR)          /* only the flashloader SRAM window */
		return -1;
	if ((sram_addr & 0x08000000u) != 0u)        /* never a flash address */
		return -1;
	if (len == 0u || len > 0x10000u)            /* the stub is ~4.7 KB */
		return -1;

	if (dl_send_wait_ack(&start, 1u, 500u, ab, ctx))
		return -2;

	while (off < len) {
		uint32_t rem  = len - off;
		uint32_t N    = (rem > 128u) ? 1024u : 128u;
		uint32_t take = (rem < N) ? rem : N;
		uint32_t addr = sram_addr + off;
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

		rr = dl_send_wait_ack(pkt, 7u + N + 1u, 1000u, ab, ctx);
		if (rr)
			return (rr == -2) ? -2 : -3;
		seq = (uint8_t)(seq + 1u);
		off += take;
	}

	if (dl_send_wait_ack(&eot, 1u, 1000u, ab, ctx))
		return -2;
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

int rtl_dl_read_flash(uint32_t offset, uint32_t nsectors, uint8_t *buf, uint32_t buf_cap,
                      int (*ab)(void *), void *ctx)
{
	static uint8_t blk[2 + 1024 + 1];   /* seq + ~seq + 1024 data + csum (off the stack) */
	uint8_t hdr[6], junk[64];
	uint32_t total_blocks, i, copied = 0u;

	if ((offset & 0xFFFu) != 0u || offset > 0x00FFFFFFu)   /* 4 KB-aligned, 24-bit */
		return -1;
	if (nsectors == 0u || nsectors > 0x10000u)
		return -1;
	total_blocks = nsectors * 4u;                          /* module streams 4 x 1024 / sector */

	while (rtl8720_uart_read(junk, sizeof(junk)) > 0u)      /* clean boundary */
		;
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
