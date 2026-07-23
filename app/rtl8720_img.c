/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/*
 * RTL8720DN firmware-image staging in PSRAM -- issue #19 M5.  See rtl8720_img.h for
 * why the image lives at 0x90000000 and what the caller owes this module (the OCTOSPI1
 * guard, and finish()-or-invalidate() after every transfer).
 *
 * Deliberately dumb: it is a bounded byte buffer plus the module's own checksum
 * algorithm.  Nothing here talks to the RTL8720 -- rtl8720_flash.c does that -- and
 * nothing here touches the RCC (the PSRAM controller was configured at boot by
 * psram.c), so it is XIP-safe like the rest of the app.
 */
#include "rtl8720_img.h"

#include "psram.h"           /* PSRAM_BASE_ADDR / PSRAM_SIZE_BYTES / psram_ready() */
#include "rtl8720_flash.h"   /* rtl_dl_digest_* -- the module's 0x27 algorithm */

#include <string.h>

_Static_assert(RTL_IMG_MAX <= PSRAM_SIZE_BYTES,
               "the staging window must fit inside the PSRAM device");

/* The staging area is the bottom of the PSRAM window.  Nothing else claims a fixed
 * address there; the diagnostic commands that use PSRAM (psram test / membench /
 * devmem) are transient and excluded by the OCTOSPI1 guard while a transfer runs. */
#define IMG_BASE ((uint8_t *)(uintptr_t)PSRAM_BASE_ADDR)

static struct rtl_img s_img;

void rtl_img_invalidate(void)
{
	memset(&s_img, 0, sizeof(s_img));
}

int rtl_img_probe(void)
{
	/* A probe writes to the staging area, so nothing staged can survive it. */
	rtl_img_invalidate();

	if (!psram_ready())
		return -1;

	/* Head and tail of the staging window: a controller that only answers at low
	 * addresses (bad address-bit wiring, a half-applied latency setting) would pass a
	 * head-only check and then quietly corrupt the far end of a 2 MB image. */
	{
		static const uint32_t pat[4] = {
			0xA5A5F00Fu, 0x0F0FC3C3u, 0xDEADBEEFu, 0x12345678u
		};
		volatile uint32_t *head = (volatile uint32_t *)(uintptr_t)IMG_BASE;
		volatile uint32_t *tail =
			(volatile uint32_t *)(uintptr_t)(IMG_BASE + RTL_IMG_MAX - sizeof(pat));
		int i;

		for (i = 0; i < 4; i++) {
			head[i] = pat[i];
			tail[i] = ~pat[i];
		}
		for (i = 0; i < 4; i++) {
			if (head[i] != pat[i])
				return -2;
			if (tail[i] != ~pat[i])
				return -2;
		}
	}
	return 0;
}

/* ---- YMODEM sink ---------------------------------------------------------- */

static int img_begin(void *ctx, const char *name, uint32_t size)
{
	size_t i;

	(void)ctx;
	rtl_img_invalidate();

	if (size == 0u || size > RTL_IMG_MAX)
		return -1;                  /* cancels the batch before any data moves */
	if (!psram_ready())
		return -1;

	for (i = 0u; i + 1u < sizeof(s_img.name) && name[i] != '\0'; i++)
		s_img.name[i] = name[i];
	s_img.name[i] = '\0';
	s_img.len = 0u;
	/* padded_len records the accepted ceiling until finish() narrows it; write()
	 * bounds against it so a receiver bug cannot walk past the staging window. */
	s_img.padded_len = size;
	return 0;
}

static int img_write(void *ctx, const uint8_t *data, uint32_t len)
{
	(void)ctx;
	/* The first test also makes the subtraction below safe: without a successful
	 * begin() padded_len is 0, and no receiver bug can turn that into a huge bound. */
	if (s_img.len > s_img.padded_len || len > s_img.padded_len - s_img.len)
		return -1;                  /* more than was declared: refuse, do not clamp */
	memcpy(IMG_BASE + s_img.len, data, len);
	s_img.len += len;
	return 0;
}

static const struct ym_sink s_sink = { NULL, img_begin, img_write };

const struct ym_sink *rtl_img_sink(void)
{
	return &s_sink;
}

int rtl_img_finish(void)
{
	struct rtl_dl_digest dg;
	uint32_t             padded;

	if (s_img.len == 0u) {
		rtl_img_invalidate();
		return -1;
	}

	/* Pad to the 4 KB granularity every flash operation works in.  0xFF is the erased
	 * value, so the padding programs as a no-op and reads back identically -- which is
	 * what lets the digest cover the padded range and still match the module's. */
	padded = (s_img.len + 4095u) & ~4095u;
	if (padded > RTL_IMG_MAX) {             /* only reachable if len itself was at the cap */
		rtl_img_invalidate();
		return -1;
	}
	memset(IMG_BASE + s_img.len, 0xFF, padded - s_img.len);
	s_img.padded_len = padded;

	rtl_dl_digest_init(&dg);
	rtl_dl_digest_add(&dg, IMG_BASE, padded);
	s_img.digest = rtl_dl_digest_value(&dg);
	s_img.valid = 1;
	return 0;
}

const struct rtl_img *rtl_img_get(void)
{
	return &s_img;
}

const uint8_t *rtl_img_data(void)
{
	return IMG_BASE;
}

uint32_t rtl_img_verify(void)
{
	struct rtl_dl_digest dg;

	if (!s_img.valid)
		return 0u;
	rtl_dl_digest_init(&dg);
	rtl_dl_digest_add(&dg, IMG_BASE, s_img.padded_len);
	return rtl_dl_digest_value(&dg);
}
