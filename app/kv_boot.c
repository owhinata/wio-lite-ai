/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    kv_boot.c
 * @brief   The configuration thread: opens the KV store at boot (issue #37).
 *
 * A thread exists here for one reason: kv_init() can erase.  A first boot on a
 * blank partition lays down a fresh database, and an erase waits by sleeping --
 * which needs a scheduler.  tx_application_define() runs before there is one, so
 * everything in it must be pure object creation (see the comments there); the work
 * that has to wait on hardware belongs on a thread, and this is that thread.
 *
 * It is also where the boot sequence that reads those settings will live: once the
 * store is open, `wifi.enable` decides whether the radio is powered, then
 * `wifi.autoconnect`, then `net.mode`.  Each step is independent and fail-soft --
 * the shell comes up either way and every one of those actions can be typed by
 * hand, so a store that will not open costs convenience, not function.
 *
 * Priority 15: above the shell (16) so the configuration is settled before a user
 * can reasonably type, but below the USB pump (8) and the IWDG petter (5), neither
 * of which may ever wait behind a flash erase.
 */
#include "kv.h"

#include "tx_api.h"

#define LOG_TAG "kv"
#include "log.h"

#define KV_BOOT_PRIORITY    15u
#define KV_BOOT_STACK_SIZE  2048u

static TX_THREAD kv_boot_thread;
static UCHAR     kv_boot_stack[KV_BOOT_STACK_SIZE] __attribute__((aligned(8)));

static void kv_boot_entry(ULONG arg)
{
	(void)arg;

	if (kv_init() != KV_OK)
		LOG_WRN("configuration store unavailable: %s", kv_state_str());

	/* The store is open (or known not to be).  The wifi/net boot sequence that
	 * reads it is the next increment of issue #37; until then this thread has
	 * nothing left to do and returns, which leaves it TERMINATED rather than
	 * burning a stack slot forever. */
}

int kv_boot_init(void)
{
	UINT rc = tx_thread_create(&kv_boot_thread, "cfg", kv_boot_entry, 0,
	                           kv_boot_stack, sizeof kv_boot_stack,
	                           KV_BOOT_PRIORITY, KV_BOOT_PRIORITY,
	                           TX_NO_TIME_SLICE, TX_AUTO_START);

	return (rc == TX_SUCCESS) ? 0 : -1;
}
