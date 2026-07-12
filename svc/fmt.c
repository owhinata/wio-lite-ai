/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    fmt.c
 * @brief   Clean-room minimal printf formatter (callback / fixed-buffer sinks).
 *
 * The formatter that used to live inline in cli_printf.c (issue #5), lifted here
 * (issue #28) behind a putc callback so the shell output API, the RAM log and
 * the fault dump share one implementation.  It streams characters into the sink
 * one at a time (no large intermediate buffer; honours the §8 "flush when full"
 * model when the sink is the shell's 32 B staging).  Supported conversions:
 * %% %c %s %d %i %u %x %X %p, length modifiers l / ll / z, field width with
 * '0' / '-' flags.  No precision, no + / space / # flags.  Clean-room -- neither
 * newlib's nor Zephyr's printf code is reused.  Depends only on the freestanding
 * headers below, never on cli_instance or any tx_* API.
 */
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "fmt.h"

/* ---- low-level helpers ------------------------------------------------- */

static void out_str(fmt_putc_t put, void *ctx, const char *s)
{
	while (*s)
		put(ctx, *s++);
}

/* Emit a body (digits/text) of @p blen chars with optional sign / "0x" prefix,
 * padded to @p width using spaces, or zeros when @p zero (zeros go after the
 * sign/prefix), left-justified when @p left. */
void fmt_padded(fmt_putc_t put, void *ctx, const char *prefix,
                const char *body, int blen, int width, int zero, int left)
{
	int plen = prefix ? (int)strlen(prefix) : 0;
	int total = plen + blen;
	int pad = width > total ? width - total : 0;

	if (left) {
		if (prefix) out_str(put, ctx, prefix);
		for (int i = 0; i < blen; i++) put(ctx, body[i]);
		while (pad-- > 0) put(ctx, ' ');
	} else if (zero) {
		if (prefix) out_str(put, ctx, prefix);
		while (pad-- > 0) put(ctx, '0');
		for (int i = 0; i < blen; i++) put(ctx, body[i]);
	} else {
		while (pad-- > 0) put(ctx, ' ');
		if (prefix) out_str(put, ctx, prefix);
		for (int i = 0; i < blen; i++) put(ctx, body[i]);
	}
}

/* Render @p mag in @p base (10/16) into @p buf (reversed-safe), return length.
 * @p upper selects A-F vs a-f.  buf must hold >= 20 chars. */
int fmt_utoa(unsigned long long mag, unsigned base, int upper, char *buf)
{
	const char *digits = upper ? "0123456789ABCDEF" : "0123456789abcdef";
	char tmp[20];
	int n = 0;
	do {
		tmp[n++] = digits[mag % base];
		mag /= base;
	} while (mag != 0);
	for (int i = 0; i < n; i++)
		buf[i] = tmp[n - 1 - i];     /* un-reverse */
	return n;
}

/* ---- minimal formatter ------------------------------------------------- */

enum len_mod { LM_INT, LM_LONG, LM_LLONG, LM_SIZE };

/* Read the integer arg inline.  va_list is an array type on some ABIs (so it
 * decays to a pointer as a function parameter); reading directly off `ap` here
 * keeps the single sequential va_arg cursor correct -- passing &ap to a helper
 * would mistype it.  Magnitude is unsigned so LLONG_MIN negates safely. */
#define GET_UNSIGNED(lm, ap)                                                  \
	((lm) == LM_LONG  ? (unsigned long long)va_arg((ap), unsigned long)  : \
	 (lm) == LM_LLONG ? va_arg((ap), unsigned long long)                 : \
	 (lm) == LM_SIZE  ? (unsigned long long)va_arg((ap), size_t)         : \
	                    (unsigned long long)va_arg((ap), unsigned int))

/* For %zd the argument is the *signed type corresponding to size_t*, which has
 * no portable name -- read it as whichever standard signed type matches size_t's
 * width (int on targets where size_t is unsigned int, e.g. Cortex-M; long on
 * LP64 hosts).  Reading a fixed `long` would be the wrong va_arg type on M7. */
#define GET_SIGNED(lm, ap)                                                    \
	((lm) == LM_LONG  ? (long long)va_arg((ap), long)      :              \
	 (lm) == LM_LLONG ? va_arg((ap), long long)            :              \
	 (lm) == LM_SIZE                                                       \
		? (sizeof(size_t) == sizeof(int)  ? (long long)va_arg((ap), int)  : \
		   sizeof(size_t) == sizeof(long) ? (long long)va_arg((ap), long) : \
		                                    va_arg((ap), long long))      : \
	                    (long long)va_arg((ap), int))

void fmt_vformat(fmt_putc_t put, void *ctx, const char *fmt, va_list ap)
{
	for (const char *p = fmt; *p; p++) {
		if (*p != '%') {
			put(ctx, *p);
			continue;
		}
		p++;                            /* consume '%' */

		int left = 0, zero = 0;
		for (;; p++) {                  /* flags */
			if (*p == '-')      left = 1;
			else if (*p == '0') zero = 1;
			else break;
		}

		int width = 0;                  /* field width (capped at 4096) */
		while (*p >= '0' && *p <= '9') {
			width = width * 10 + (*p - '0');
			if (width > 4096)
				width = 4096;
			p++;
		}

		enum len_mod lm = LM_INT;       /* length modifier */
		if (*p == 'l') {
			p++;
			if (*p == 'l') { lm = LM_LLONG; p++; }
			else             lm = LM_LONG;
		} else if (*p == 'z') {
			lm = LM_SIZE;
			p++;
		}

		char body[20];
		int  blen;
		switch (*p) {
		case '%':
			put(ctx, '%');
			break;
		case 'c': {
			char ch = (char)va_arg(ap, int);
			fmt_padded(put, ctx, NULL, &ch, 1, width, 0, left);
			break;
		}
		case 's': {
			const char *s = va_arg(ap, const char *);
			if (s == NULL) s = "(null)";
			fmt_padded(put, ctx, NULL, s, (int)strlen(s), width, 0, left);
			break;
		}
		case 'd':
		case 'i': {
			long long v = GET_SIGNED(lm, ap);
			unsigned long long mag = (v < 0)
				? (unsigned long long)(-(v + 1)) + 1ULL   /* safe for LLONG_MIN */
				: (unsigned long long)v;
			blen = fmt_utoa(mag, 10, 0, body);
			fmt_padded(put, ctx, v < 0 ? "-" : NULL, body, blen, width, zero, left);
			break;
		}
		case 'u':
			blen = fmt_utoa(GET_UNSIGNED(lm, ap), 10, 0, body);
			fmt_padded(put, ctx, NULL, body, blen, width, zero, left);
			break;
		case 'x':
			blen = fmt_utoa(GET_UNSIGNED(lm, ap), 16, 0, body);
			fmt_padded(put, ctx, NULL, body, blen, width, zero, left);
			break;
		case 'X':
			blen = fmt_utoa(GET_UNSIGNED(lm, ap), 16, 1, body);
			fmt_padded(put, ctx, NULL, body, blen, width, zero, left);
			break;
		case 'p': {
			uintptr_t v = (uintptr_t)va_arg(ap, void *);
			blen = fmt_utoa((unsigned long long)v, 16, 0, body);
			fmt_padded(put, ctx, "0x", body, blen, width, zero, left);
			break;
		}
		case '\0':                      /* trailing '%': emit literally, stop */
			put(ctx, '%');
			return;
		default:                        /* unknown spec: emit verbatim */
			put(ctx, '%');
			put(ctx, *p);
			break;
		}
	}
}

/* ---- fixed-buffer sink ------------------------------------------------- */

struct fmt_snbuf { char *buf; size_t size; size_t pos; };

/* Store while a byte plus the trailing NUL still fits; pos tracks bytes stored
 * (it never advances past size-1, so it is the truncated length, not the
 * would-have-been length of a full vsnprintf). */
static void snbuf_putc(void *ctx, char c)
{
	struct fmt_snbuf *s = (struct fmt_snbuf *)ctx;
	if (s->pos + 1 < s->size)
		s->buf[s->pos++] = c;
}

int fmt_vsnformat(char *buf, size_t size, const char *fmt, va_list ap)
{
	struct fmt_snbuf s = { buf, size, 0 };
	if (size == 0)
		return 0;
	fmt_vformat(snbuf_putc, &s, fmt, ap);
	buf[s.pos] = '\0';
	return (int)s.pos;
}
