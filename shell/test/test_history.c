/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 *
 * Host unit test for the command history fixed ring (issue #10): add + recall
 * (↑/↓, Ctrl+p/n), consecutive-duplicate suppression, non-consecutive duplicates
 * kept, FIFO eviction at the byte cap, empty lines skipped, the key wiring
 * through cli_input_byte, navigation-state reset on submit / Ctrl+C / the blank
 * re-submit regression, the MVP "no draft restore" behaviour, and per-instance
 * isolation.
 *
 * Built with -DCLI_HISTORY_BUFFER_SIZE=32 so a few short entries force eviction,
 * and colour OFF so any echoed bytes compare plainly.  Drives the ThreadX-free
 * core directly (cli_history_* + cli_input_byte) and asserts the model; output /
 * tx_* glue go through the shared dummy backend + host_glue like test_edit.c.
 */
#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "cli_instance.h"
#include "cli_internal.h"
#include "cli_backend_dummy.h"
#include "host_glue.h"

/* ---- harness ----------------------------------------------------------- */

CLI_BACKEND_DUMMY_DEFINE(tr);
CLI_BACKEND_DUMMY_DEFINE(tr2);
static struct cli_instance sh;
static struct cli_instance sh2;

static void reset(struct cli_instance *s, struct cli_transport *t)
{
	memset(s, 0, sizeof *s);
	s->tr = t;
	t->sh = s;
	strcpy(s->prompt, "> ");
	s->term_width = 80;                    /* deterministic width (no CPR here) */
	cli_dummy_clear_output(t);
	cli_dummy_clear_rx(t);
	cli_dummy_reset_stats(t);
	cli_dummy_set_tx_fail(t, 0);
	cli_dummy_set_tx_cap(t, 0);            /* unlimited */
	cli_test_set_tx_wait_hook(NULL, NULL);
}

static void reset_sh(void) { reset(&sh, &tr); }

/* Drive bytes through the RX/edit state machine of the global instance. */
static void feed_byte(uint8_t b) { cli_input_byte(&sh, b); }
static void feed(const char *s) { for (const char *p = s; *p; p++) feed_byte((uint8_t)*p); }

/* ESC[ <s> -- a CSI sequence (s carries any params + the final byte). */
static void csi(const char *s) { feed_byte(0x1B); feed_byte('['); feed(s); }
static void key_up(void)   { csi("A"); }
static void key_down(void) { csi("B"); }

/* ---- tests ------------------------------------------------------------- */

/* add + recall round-trip, including the oldest/newest end stops. */
static void test_add_recall(void)
{
	reset_sh();
	cli_history_add(&sh, "one");
	cli_history_add(&sh, "two");
	cli_history_add(&sh, "three");

	cli_history_prev(&sh); assert(strcmp(sh.line, "three") == 0 && sh.cur == 5);
	assert(sh.hist_nav_on == 1);
	cli_history_prev(&sh); assert(strcmp(sh.line, "two") == 0);
	cli_history_prev(&sh); assert(strcmp(sh.line, "one") == 0);
	cli_history_prev(&sh); assert(strcmp(sh.line, "one") == 0);   /* oldest: stay */

	cli_history_next(&sh); assert(strcmp(sh.line, "two") == 0);
	cli_history_next(&sh); assert(strcmp(sh.line, "three") == 0);
	cli_history_next(&sh);                                        /* past newest */
	assert(sh.len == 0 && sh.cur == 0 && sh.line[0] == '\0' && sh.hist_nav_on == 0);
	cli_history_next(&sh); assert(sh.hist_nav_on == 0);           /* on live: no-op */
}

/* Consecutive duplicates are not recorded twice. */
static void test_dup_suppress(void)
{
	reset_sh();
	cli_history_add(&sh, "ls");
	cli_history_add(&sh, "ls");
	assert(sh.hist_used == 3);                                    /* "ls\0" once */

	cli_history_prev(&sh); assert(strcmp(sh.line, "ls") == 0);
	cli_history_prev(&sh); assert(strcmp(sh.line, "ls") == 0);    /* only entry */
}

/* A duplicate that is not the most recent entry is kept. */
static void test_nonconsec_dup(void)
{
	reset_sh();
	cli_history_add(&sh, "ls");
	cli_history_add(&sh, "cd");
	cli_history_add(&sh, "ls");
	assert(sh.hist_used == 9);                                    /* 3 * "xx\0" */

	cli_history_prev(&sh); assert(strcmp(sh.line, "ls") == 0);
	cli_history_prev(&sh); assert(strcmp(sh.line, "cd") == 0);
	cli_history_prev(&sh); assert(strcmp(sh.line, "ls") == 0);
}

/* FIFO eviction at the byte cap (CLI_HISTORY_BUFFER_SIZE == 32).  Five 7-char
 * entries (8 B each) overflow 32 B: the oldest is dropped, leaving four. */
static void test_fifo_evict(void)
{
	reset_sh();
	cli_history_add(&sh, "AAAAAAA");
	cli_history_add(&sh, "BBBBBBB");
	cli_history_add(&sh, "CCCCCCC");
	cli_history_add(&sh, "DDDDDDD");       /* used == 32 exactly, no eviction yet */
	assert(sh.hist_used == 32);
	cli_history_add(&sh, "EEEEEEE");       /* evicts A, used back to 32 */
	assert(sh.hist_used == 32);

	cli_history_prev(&sh); assert(strcmp(sh.line, "EEEEEEE") == 0);
	cli_history_prev(&sh); assert(strcmp(sh.line, "DDDDDDD") == 0);
	cli_history_prev(&sh); assert(strcmp(sh.line, "CCCCCCC") == 0);
	cli_history_prev(&sh); assert(strcmp(sh.line, "BBBBBBB") == 0);
	cli_history_prev(&sh); assert(strcmp(sh.line, "BBBBBBB") == 0);  /* A is gone */
}

/* Eviction boundaries: an entry that exactly fills the ring, evicting that
 * ring-filling entry, and one add that drops more than one older entry. */
static void test_evict_boundaries(void)
{
	char big[40];

	/* cap-1 chars (31) fill the 32 B ring exactly and stay recallable. */
	reset_sh();
	memset(big, 'Z', 31); big[31] = '\0';
	cli_history_add(&sh, big);
	assert(sh.hist_used == 32);
	cli_history_prev(&sh); assert(strcmp(sh.line, big) == 0);

	/* Even a 1-char line must evict the ring-filling entry. */
	reset_sh();
	cli_history_add(&sh, big);             /* used == 32 */
	cli_history_add(&sh, "x");             /* evicts big, used == 2 */
	assert(sh.hist_used == 2);
	cli_history_prev(&sh); assert(strcmp(sh.line, "x") == 0);
	cli_history_prev(&sh); assert(strcmp(sh.line, "x") == 0);   /* big is gone */

	/* One add evicting two older entries at once. */
	reset_sh();
	cli_history_add(&sh, "aa");
	cli_history_add(&sh, "bb");
	cli_history_add(&sh, "cc");
	cli_history_add(&sh, "dd");            /* used == 12 (3 B each) */
	memset(big, 'Y', 25); big[25] = '\0'; /* 26 B: evicts aa + bb */
	cli_history_add(&sh, big);
	assert(sh.hist_used == 32);            /* cc + dd + big = 3 + 3 + 26 */
	cli_history_prev(&sh); assert(strcmp(sh.line, big) == 0);
	cli_history_prev(&sh); assert(strcmp(sh.line, "dd") == 0);
	cli_history_prev(&sh); assert(strcmp(sh.line, "cc") == 0);
	cli_history_prev(&sh); assert(strcmp(sh.line, "cc") == 0);  /* aa, bb evicted */
}

/* Empty lines are never recorded; recall on an empty ring is a no-op. */
static void test_empty_skipped(void)
{
	reset_sh();
	cli_history_add(&sh, "");
	assert(sh.hist_used == 0);
	strcpy(sh.line, "draft"); sh.len = 5; sh.cur = 5;
	cli_history_prev(&sh);                                        /* nothing to recall */
	assert(strcmp(sh.line, "draft") == 0 && sh.hist_nav_on == 0);
}

/* The key wiring: arrows and Ctrl+p/n recall, a full line submits + records. */
static void test_key_path(void)
{
	reset_sh();
	feed("abc\r");                          /* dispatch records "abc" */
	assert(sh.len == 0 && sh.hist_nav_on == 0);

	key_up();   assert(strcmp(sh.line, "abc") == 0 && sh.cur == 3 && sh.hist_nav_on == 1);
	key_down(); assert(sh.len == 0 && sh.hist_nav_on == 0);   /* past newest -> empty */

	feed("xyz\r");                          /* history: "abc", "xyz" */
	feed_byte(0x10); assert(strcmp(sh.line, "xyz") == 0);    /* Ctrl+p */
	feed_byte(0x10); assert(strcmp(sh.line, "abc") == 0);    /* Ctrl+p */
	feed_byte(0x0E); assert(strcmp(sh.line, "xyz") == 0);    /* Ctrl+n */
}

/* Submitting a recalled line leaves navigation and re-records as newest. */
static void test_submit_resets_nav(void)
{
	reset_sh();
	feed("foo\r");
	key_up(); assert(strcmp(sh.line, "foo") == 0 && sh.hist_nav_on == 1);
	feed_byte('\r');                        /* submit the recalled "foo" */
	assert(sh.hist_nav_on == 0 && sh.hist_nav == 0);
	key_up(); assert(strcmp(sh.line, "foo") == 0);   /* starts from newest again */
}

/* Ctrl+C leaves history navigation. */
static void test_ctrl_c_resets_nav(void)
{
	reset_sh();
	feed("foo\r");
	key_up(); assert(sh.hist_nav_on == 1);
	feed_byte(0x03);                        /* Ctrl+C */
	assert(sh.hist_nav_on == 0 && sh.len == 0);
}

/* Regression for the BLOCKING: clearing a recalled line and submitting it blank
 * must still reset navigation (cli_history_add is skipped for an empty line). */
static void test_blank_resubmit_resets_nav(void)
{
	reset_sh();
	feed("foo\r");
	key_up(); assert(strcmp(sh.line, "foo") == 0 && sh.hist_nav_on == 1);
	feed_byte(0x08); feed_byte(0x08); feed_byte(0x08);   /* erase "foo" */
	assert(sh.len == 0 && sh.hist_nav_on == 1);          /* still navigating */
	feed_byte('\r');                        /* blank submit: add() not called */
	assert(sh.hist_nav_on == 0 && sh.hist_nav == 0);
	key_up(); assert(strcmp(sh.line, "foo") == 0);       /* not a stale offset */
}

/* MVP "no draft restore": a typed draft and edits to a recalled line are lost
 * when navigation moves; past-newest gives an empty line, not the draft. */
static void test_draft_and_edits_discarded(void)
{
	reset_sh();
	feed("foo\r");
	feed("bar\r");

	feed("dra");                            /* a live draft */
	key_up(); assert(strcmp(sh.line, "bar") == 0);       /* draft lost */
	feed_byte('X'); assert(strcmp(sh.line, "barX") == 0);/* edit a recalled line */
	key_up(); assert(strcmp(sh.line, "foo") == 0);       /* edit discarded */
	key_down(); assert(strcmp(sh.line, "bar") == 0);
	key_down(); assert(sh.len == 0 && sh.hist_nav_on == 0);   /* empty, not "dra" */
}

/* Histories are per-instance: two shells never share entries (req §10). */
static void test_instance_isolation(void)
{
	reset(&sh, &tr);
	reset(&sh2, &tr2);

	cli_history_add(&sh, "alpha");
	cli_history_add(&sh2, "beta");

	cli_history_prev(&sh);  assert(strcmp(sh.line, "alpha") == 0);
	cli_history_prev(&sh2); assert(strcmp(sh2.line, "beta") == 0);
	cli_history_prev(&sh2); assert(strcmp(sh2.line, "beta") == 0);   /* only its own */
	assert(strcmp(sh.line, "alpha") == 0);                           /* sh untouched */
}

int main(void)
{
	test_add_recall();
	test_dup_suppress();
	test_nonconsec_dup();
	test_fifo_evict();
	test_evict_boundaries();
	test_empty_skipped();
	test_key_path();
	test_submit_resets_nav();
	test_ctrl_c_resets_nav();
	test_blank_resubmit_resets_nav();
	test_draft_and_edits_discarded();
	test_instance_isolation();
	printf("OK: history ring / recall / dedup / FIFO evict / nav reset / isolation pass\n");
	return 0;
}
