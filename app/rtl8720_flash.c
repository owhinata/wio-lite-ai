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
