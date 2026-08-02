/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    fal_nor.c
 * @brief   FAL flash device over the external NOR, plus FlashDB's log/assert hooks
 *          (issue #37).
 *
 * This is the whole of FlashDB's contact with the hardware: four operations
 * forwarded to port/nor, which owns the controller, the device mutex and the
 * bounded waiting.  Nothing about OCTOSPI appears here.
 *
 * Layering: port/flashdb calls port/nor.  Both are port/, so this is a same-layer
 * call, matching how port/filex/fx_sd_driver.c sits on port/sd/sd_card.c.
 *
 * Locking: these callbacks do NOT take the device mutex themselves -- port/nor's
 * primitives take it (recursively) on every call, and FlashDB has already taken it
 * through its own lock callback by the time it gets here.  The recursion is what
 * makes the layering safe: app/kv.c can hold the lock across a whole multi-step
 * operation and every level below still just works.
 */
#include <fal.h>
#include <fdb_cfg.h>

#include "nor_flash.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define LOG_TAG "kv"
#include "log.h"

/*
 * FAL's write_gran field exists but fal_partition_write() passes it straight
 * through to ops.write() without checking anything against it -- so a device that
 * disagreed with FDB_WRITE_GRAN would not be caught anywhere at run time.  It gets
 * caught here instead, at compile time.  If these two ever diverge, FlashDB would
 * lay out status bytes for one granularity while the flash enforced another, and
 * the damage would show up as unexplained corruption long after the change.
 */
#define NOR_FAL_WRITE_GRAN  8
_Static_assert(FDB_WRITE_GRAN == NOR_FAL_WRITE_GRAN,
               "FlashDB write granularity must match the FAL device's write_gran");

static int nor_fal_init(void)
{
	/* The device was brought up in main(); this only reports whether it answered.
	 * Nothing here talks to the flash, so fal_init() stays cheap and side-effect
	 * free even when the hardware is missing. */
	return nor_flash_ready() ? 0 : -1;
}

static int nor_fal_read(long offset, uint8_t *buf, size_t size)
{
	if (offset < 0)
		return -1;
	if (nor_read((uint32_t)offset, buf, (uint32_t)size) != NOR_OK)
		return -1;
	return (int)size;
}

static int nor_fal_write(long offset, const uint8_t *buf, size_t size)
{
	if (offset < 0)
		return -1;
	if (nor_write((uint32_t)offset, buf, (uint32_t)size) != NOR_OK)
		return -1;
	return (int)size;
}

static int nor_fal_erase(long offset, size_t size)
{
	uint32_t addr, end;

	if (offset < 0 || size == 0u)
		return -1;
	/*
	 * Round out to whole sectors.  FlashDB always asks for sector-aligned ranges,
	 * but an erase that quietly did less than asked would leave bits set where the
	 * caller is entitled to assume 0xFF -- so widen rather than trust, and let
	 * port/nor reject anything that then runs off the end of the device.
	 */
	addr = (uint32_t)offset - ((uint32_t)offset % NOR_SECTOR_SIZE);
	end = (uint32_t)offset + (uint32_t)size;
	end = (end + NOR_SECTOR_SIZE - 1u) & ~(NOR_SECTOR_SIZE - 1u);
	if (nor_erase(addr, end - addr) != NOR_OK)
		return -1;
	return (int)size;
}

const struct fal_flash_dev nor_flash_dev = {
	.name = NOR_FLASH_DEV_NAME,
	.addr = 0,                      /* offsets, not addresses: nothing is mapped */
	.len = NOR_SIZE_BYTES,
	.blk_size = NOR_SECTOR_SIZE,
	.ops = {
		.init  = nor_fal_init,
		.read  = nor_fal_read,
		.write = nor_fal_write,
		.erase = nor_fal_erase,
	},
	.write_gran = NOR_FAL_WRITE_GRAN,
};

/* ------------------------------------------------------------------ *
 *  Diagnostics: FDB_PRINT / FAL_PRINTF -> the reset-persistent log ring
 * ------------------------------------------------------------------ */

/*
 * FlashDB and FAL both emit one logical message as SEVERAL printf fragments -- a
 * prefix, the body, a colour reset -- so forwarding each call to LOG_INF() would
 * shred every message across three records with the formatting split between
 * them.  svc/log.c's line assembler puts them back together and emits one record
 * per newline (and drops the ANSI colour escapes FAL's log_e/log_i wrap them in).
 *
 * The assembler is static and unguarded, which is safe for one specific reason:
 * every FlashDB and FAL code path that logs runs with the NOR device mutex held
 * (app/kv.c takes it around the whole operation, and FlashDB's own lock callback
 * takes it again inside), so there is never a second thread inside this function.
 * If a caller is ever added that logs outside that lock, this needs a lock too.
 */
static struct log_line kv_log;

void kv_fdb_print(const char *fmt, ...)
{
	char frag[LOG_LINE_ASM_MAX];
	va_list ap;
	int n;

	va_start(ap, fmt);
	n = vsnprintf(frag, sizeof frag, fmt, ap);
	va_end(ap);
	if (n < 0)
		return;
	if (n > (int)sizeof frag - 1)
		n = (int)sizeof frag - 1;   /* vsnprintf reports the untruncated length */
	log_line_feed(&kv_log, LOG_TAG, frag, (size_t)n);
}

/* ------------------------------------------------------------------ *
 *  Assertion hook
 * ------------------------------------------------------------------ */

/* Sticky: once FlashDB has asserted, its internal state is not to be trusted, so
 * app/kv.c latches the database out of service rather than continuing to answer
 * queries from it.  See fdb_cfg.h for why this does not spin forever. */
static volatile int kv_fdb_asserted;

void kv_fdb_assert_failed(const char *expr, const char *func)
{
	kv_fdb_asserted = 1;
	LOG_ERR("FlashDB assert: %s in %s()", expr, func);
}

int kv_fdb_assert_tripped(void)
{
	return kv_fdb_asserted;
}
