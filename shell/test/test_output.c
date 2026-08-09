/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 *
 * Host unit test for the Shell output API (issue #5): the minimal formatter
 * (conversions, length modifiers, width/flags and their boundaries), 32 B
 * staging + autoflush across a >32 B write, VT100 colour for error/warn/info,
 * hexdump layout, and TX-failure handling (drop + tx_failed + negative return).
 * Output and the tx_* glue go through the shared dummy backend (cli_backend_dummy)
 * + host_glue, so the staged bytes actually traverse tr->api->write().  The
 * tx_dropped *count* (TX-timeout path) is asserted in test_integration.c.  Built
 * with colour ON (the default) so the SGR escapes are asserted.
 */
#include <assert.h>
#include <limits.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "cli_instance.h"
#include "cli_internal.h"
#include "cli_vt100.h"
#include "cli_backend_dummy.h"
#include "host_glue.h"

CLI_BACKEND_DUMMY_DEFINE(tr);
static struct cli_instance sh;

static void setup(void)
{
	memset(&sh, 0, sizeof sh);
	sh.tr = &tr;
	tr.sh = &sh;
	cli_dummy_clear_output(&tr);
	cli_dummy_clear_rx(&tr);
	cli_dummy_reset_stats(&tr);
	cli_dummy_set_tx_fail(&tr, 0);
	cli_dummy_set_tx_cap(&tr, 0);          /* unlimited */
	cli_test_set_tx_wait_hook(NULL, NULL);
}

#define EXPECT(want, ...) do {                                                \
	setup();                                                             \
	cli_print(&sh, __VA_ARGS__);                                        \
	const char *got = cli_dummy_output_str(&tr);                        \
	if (strcmp(got, (want)) != 0) {                                     \
		printf("FAIL line %d: got [%s] want [%s]\n",                \
		       __LINE__, got, (want));                              \
		assert(0);                                                  \
	}                                                                   \
} while (0)

/* Unattributed alias: deliberately-malformed formats (%q, trailing %, NULL %s)
 * are valid runtime inputs to exercise, but the format(printf) attribute would
 * warn on the literals.  Calls through this pointer are not format-checked. */
static int (*print_raw)(struct cli_instance *, const char *, ...) = cli_print;

static void test_formatter(void)
{
	EXPECT("hello", "hello");
	EXPECT("a%b", "a%%b");
	EXPECT("A", "%c", 'A');
	EXPECT("x=42", "x=%d", 42);
	EXPECT("-7", "%d", -7);
	EXPECT("4000000000", "%u", 4000000000U);
	EXPECT("ab AB", "%x %X", 0xab, 0xab);
	setup();
	print_raw(&sh, "%s", (char *)NULL);      /* NULL %s -> "(null)" */
	assert(strcmp(cli_dummy_output_str(&tr), "(null)") == 0);
	EXPECT("hi there", "%s %s", "hi", "there");

	/* length modifiers */
	EXPECT("1234567890", "%lu", 1234567890UL);
	EXPECT("9223372036854775807", "%lld", (long long)9223372036854775807LL);
	EXPECT("deadbeef", "%llx", 0xdeadbeefULL);

	/* width / flags */
	EXPECT("   42", "%5d", 42);
	EXPECT("42   |", "%-5d|", 42);
	EXPECT("0000001f", "%08x", 0x1fu);
	EXPECT("  abc", "%5s", "abc");
	EXPECT("abc  |", "%-5s|", "abc");

	/* Width on %lu, which is what `thread`'s columns are built from (issue #26 widened
	 * the runs column from 6 to 8 after a 7-digit value pushed every column right).
	 * The interesting cases are the two ends: a value narrower than the field must be
	 * padded, and one wider must OVERFLOW rather than truncate -- a truncated run count
	 * would be a wrong number reported as if it were right. */
	EXPECT(" 5143002", "%8lu", 5143002UL);
	EXPECT("99999999", "%8lu", 99999999UL);
	EXPECT("4294967295", "%8lu", 4294967295UL);   /* ULONG max: wider than the field */
}

static void test_formatter_boundaries(void)
{
	EXPECT("-2147483648", "%d", INT_MIN);
	EXPECT("-9223372036854775808", "%lld", LLONG_MIN);

	setup();
	print_raw(&sh, "%q");                     /* unknown spec: verbatim */
	assert(strcmp(cli_dummy_output_str(&tr), "%q") == 0);

	setup();
	print_raw(&sh, "end%");                   /* trailing '%' */
	assert(strcmp(cli_dummy_output_str(&tr), "end%") == 0);

	/* %p prints 0x-prefixed hex of the pointer value */
	setup();
	cli_print(&sh, "%p", (void *)0x1234);
	assert(strcmp(cli_dummy_output_str(&tr), "0x1234") == 0);

	/* %zu reads size_t; %zd its signed counterpart (width-matched per ABI) */
	setup();
	cli_print(&sh, "%zu", (size_t)4096);
	assert(strcmp(cli_dummy_output_str(&tr), "4096") == 0);

	setup();
	print_raw(&sh, "%zd", (long)-42);   /* signed size (long matches size_t on host) */
	assert(strcmp(cli_dummy_output_str(&tr), "-42") == 0);

	/* absurd width is capped, not overflowed (no crash, bounded output) */
	setup();
	cli_print(&sh, "%999999999d", 1);
	size_t n;
	cli_dummy_output(&tr, &n);
	assert(n <= 4096);
}

static void test_autoflush(void)
{
	char big[100];
	for (int i = 0; i < 99; i++) big[i] = 'A' + (i % 26);
	big[99] = '\0';

	setup();
	cli_print(&sh, "%s", big);               /* > 32 B staging: flushes ~4x */
	size_t n;
	const char *o = cli_dummy_output(&tr, &n);
	assert(n == 99);
	assert(memcmp(o, big, 99) == 0);         /* every byte arrived, in order */
}

static void test_color(void)
{
	setup();
	cli_error(&sh, "oops");
	assert(strcmp(cli_dummy_output_str(&tr), "\x1b[31moops\x1b[0m") == 0);

	setup();
	cli_warn(&sh, "careful");
	assert(strcmp(cli_dummy_output_str(&tr), "\x1b[33mcareful\x1b[0m") == 0);

	setup();
	cli_info(&sh, "ok");
	assert(strcmp(cli_dummy_output_str(&tr), "\x1b[32mok\x1b[0m") == 0);

	setup();
	cli_print(&sh, "plain");                 /* no colour on cli_print */
	assert(strcmp(cli_dummy_output_str(&tr), "plain") == 0);
}

static void test_write_and_hexdump(void)
{
	setup();
	assert(cli_write(&sh, "raw\x00\x01", 5) == 0);
	size_t n;
	const char *o = cli_dummy_output(&tr, &n);
	assert(n == 5 && memcmp(o, "raw\x00\x01", 5) == 0);

	setup();
	cli_hexdump(&sh, "Hi\x01!", 4);
	assert(strstr(cli_dummy_output_str(&tr), "00000000  48 69 01 21 ") != NULL);
	assert(strstr(cli_dummy_output_str(&tr), "Hi.!\r\n") != NULL);   /* 0x01 -> '.' */

	/* cli_hexdump_base: the offset column counts from `base`, everything else is
	 * identical -- so cli_hexdump is exactly the base == 0 case. */
	setup();
	cli_hexdump_base(&sh, "Hi\x01!", 4, 0x20000000ull);
	assert(strstr(cli_dummy_output_str(&tr), "20000000  48 69 01 21 ") != NULL);
	assert(strstr(cli_dummy_output_str(&tr), "Hi.!\r\n") != NULL);

	/* base advances 16 per row; a second row shows base + 0x10. */
	setup();
	cli_hexdump_base(&sh, "0123456789abcdefG", 17, 0x08000000ull);
	assert(strstr(cli_dummy_output_str(&tr), "08000000  ") != NULL);
	assert(strstr(cli_dummy_output_str(&tr), "08000010  47 ") != NULL);  /* 'G' */
}

/* Immediate transport failure: the call returns <0, tx_failed sticks and drops
 * further output, and recovery resumes once the flag is cleared.  Per the real
 * cli_core.c contract a write()<0 sets tx_failed but does NOT count tx_dropped
 * (that is the TX-timeout path, covered in test_integration.c). */
static void test_tx_failure(void)
{
	setup();
	cli_dummy_set_tx_fail(&tr, 1);
	int r = cli_print(&sh, "dropme");
	size_t n;
	cli_dummy_output(&tr, &n);
	assert(r < 0 && sh.tx_failed == 1 && n == 0);   /* nothing emitted */

	/* sticky: further output stays dropped until tx_failed is cleared */
	r = cli_print(&sh, "again");
	cli_dummy_output(&tr, &n);
	assert(r < 0 && n == 0);

	/* recovery: clear the flag, stop failing -> output flows again */
	sh.tx_failed = 0;
	cli_dummy_set_tx_fail(&tr, 0);
	r = cli_print(&sh, "back");
	assert(r == 0 && strcmp(cli_dummy_output_str(&tr), "back") == 0);
}

/*
 * End-of-unit signal (issue #49).  A backend that TRANSMITS needs to know when an output
 * call is complete, or it sends a third of a line-editor redraw and pays a network round
 * trip for it -- which is exactly what made the telnet console sluggish.  Two properties
 * the telnet backend's coalescing rests on:
 *
 *   1. exactly ONE flush per output call, however many staged chunks it took;
 *   2. a flush on EVERY release, nested ones included -- cli_console_claim() holds the
 *      output lock across a whole YMODEM transfer, so if only the outermost release
 *      signalled, every block inside it would sit unsent and the protocol would stall.
 */
static void test_flush_signal(void)
{
	struct cli_dummy *d = (struct cli_dummy *)tr.ctx;
	char big[CLI_PRINTF_BUFFER_SIZE * 3 + 7];

	setup();
	d->flushes = 0;
	cli_print(&sh, "hi");
	assert(d->flushes == 1);                /* short: one staged chunk, one flush */

	/* Long enough to auto-flush the staging buffer several times: the backend must
	 * still be told "done" exactly once, or one redraw becomes several packets. */
	memset(big, 'x', sizeof big - 1);
	big[sizeof big - 1] = '\0';
	cli_dummy_clear_output(&tr);
	d->flushes = 0;
	cli_print(&sh, "%s", big);
	assert(d->flushes == 1);
	assert(strlen(cli_dummy_output_str(&tr)) == sizeof big - 1);

	/* Nested: the inner release signals too (the YMODEM case above). */
	d->flushes = 0;
	assert(cli_lock(&sh) == 0);             /* outer holder, as a transfer would */
	cli_print(&sh, "block");                /* inner lock/unlock */
	assert(d->flushes == 1);                /* signalled WITHOUT the outer release */
	cli_unlock(&sh);
	assert(d->flushes == 2);
}

int main(void)
{
	test_formatter();
	test_formatter_boundaries();
	test_autoflush();
	test_color();
	test_write_and_hexdump();
	test_tx_failure();
	test_flush_signal();
	printf("OK: formatter / boundaries / autoflush / colour / hexdump / tx-fail / "
	       "flush pass\n");
	return 0;
}
