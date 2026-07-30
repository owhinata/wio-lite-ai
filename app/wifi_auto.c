/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/*
 * Host-side automatic re-association (issue #32).  See app/wifi_auto.h for why the
 * module's own wifi_set_autoreconnect() cannot be used, and for the threading rules.
 */
#include "wifi_auto.h"

#include <string.h>

#include "tx_api.h"
#include "stm32h7xx_hal.h"   /* __get_PRIMASK / __disable_irq / __set_PRIMASK */

#include "cli.h"
#define LOG_TAG "wauto"
#include "log.h"
#include "rtl_link.h"        /* rtl_link_forget_gen: the module identity this is armed against */

/* ---- state ------------------------------------------------------------------- */

/*
 * `armed` is read from erpc_call_ex()'s abort hook about once a millisecond, so it is the
 * one field with a lock-free reader and therefore the one that must be volatile.  Every
 * writer below still updates it inside the critical section with the rest, so a reader
 * that sees it false is guaranteed to be looking at a fully-disarmed state.
 */
static volatile bool wa_armed;

/*
 * True from the moment wifi_auto_attempt() commits to the call until it has an answer.
 * Only the console uses it, and only to tell the operator whether disarming actually cut
 * something short -- saying so unconditionally would claim a module-side join that in the
 * usual case never existed.
 */
static volatile bool wa_in_flight;

/*
 * ON at boot.  Re-associating is what the operator wants in almost every case, and having
 * to remember a preference before the outage -- which is the only time it can be set,
 * since arming happens at `wifi connect` -- made the feature miss precisely the situation
 * it exists for.  The cost is stated at wifi_auto.h: with this on, every successful
 * `wifi connect` leaves its passphrase in host RAM.  `wifi autoreconnect off` opts out for
 * the session; there is no persistent setting, so a reset comes back on.
 */
static bool     wa_enabled = true;
static char     wa_ssid[WIFI_AUTO_SSID_MAX + 1u];
static char     wa_pass[WIFI_AUTO_PASS_MAX + 1u];
static bool     wa_open;                 /* no passphrase (open network) */
static uint32_t wa_security;

/* The module identity the credentials belong to.  rtl_link_forget_module() advances this
 * counter on every path that changes what is in front of us -- a CHIP_EN move, a flash
 * session, a fresh power-on -- so a mismatch means these credentials are for a module that
 * is no longer there.  The shell disarms explicitly on all of those paths; this is the
 * backstop for one that is ever missed. */
static uint32_t wa_forget_gen;

/* Backoff and tally. */
static ULONG    wa_next_try;             /* tx_time_get() deadline */
static uint32_t wa_backoff_ms;
static uint32_t wa_attempts, wa_ok, wa_failed;
static ULONG    wa_last_try;
static bool     wa_last_try_valid;
static int      wa_last_rc;
static int32_t  wa_last_result;
static const char *wa_why = "waiting for the next `wifi connect`";

/* ---- the critical section ---------------------------------------------------- */

/*
 * Short PRIMASK sections, not a mutex: every one of these is a handful of stores, and the
 * state has to be consistent for a reader that cannot block (the abort hook) and for one
 * that runs on the owner thread while a CLI thread is disarming.  Same instrument, and the
 * same reason, as nx_net.c's nxn_mod_accumulate().
 */
static uint32_t wa_lock(void)
{
	uint32_t prim = __get_PRIMASK();
	__disable_irq();
	return prim;
}

static void wa_unlock(uint32_t prim)
{
	__set_PRIMASK(prim);
}

/* Wipe the credentials.  Call with the section held. */
static void wa_clear_locked(const char *why)
{
	memset(wa_ssid, 0, sizeof wa_ssid);
	memset(wa_pass, 0, sizeof wa_pass);
	wa_open      = false;
	wa_security  = 0u;
	wa_armed     = false;
	wa_backoff_ms = 0u;
	if (why != NULL)
		wa_why = why;
}

/* ---- enable / arm / disarm --------------------------------------------------- */

void wifi_auto_set_enabled(bool on)
{
	uint32_t prim = wa_lock();
	bool was = wa_armed;

	wa_enabled = on;
	if (!on)
		wa_clear_locked("turned off");
	else if (!wa_armed)
		wa_why = "waiting for the next `wifi connect`";
	wa_unlock(prim);
	/* Logged for the same reason wifi_auto_disarm() logs: dmesg is where "why did it stop
	 * re-associating" gets answered after the fact, and turning it off has to leave the
	 * same trace there as `wifi reset` and `wifi disconnect` do. */
	if (!on && was)
		LOG_INF("disarmed (turned off)");
}

bool wifi_auto_enabled(void)
{
	return wa_enabled;
}

void wifi_auto_arm(const char *ssid, const char *password, uint32_t security)
{
	uint32_t prim;

	if (ssid == NULL || strlen(ssid) > WIFI_AUTO_SSID_MAX ||
	    (password != NULL && strlen(password) > WIFI_AUTO_PASS_MAX))
		return;                          /* `wifi connect` bounds these too */

	prim = wa_lock();
	if (!wa_enabled) {
		wa_unlock(prim);
		return;                          /* opt-in: no enable, no passphrase in RAM */
	}
	wa_clear_locked(NULL);
	memcpy(wa_ssid, ssid, strlen(ssid));
	if (password != NULL)
		memcpy(wa_pass, password, strlen(password));
	else
		wa_open = true;
	wa_security   = security;
	wa_forget_gen = rtl_link_forget_gen();
	wa_next_try   = tx_time_get();
	wa_backoff_ms = 0u;
	wa_why        = "";
	wa_armed      = true;
	wa_unlock(prim);
	LOG_INF("armed for \"%s\"", ssid);
}

void wifi_auto_disarm(const char *why)
{
	uint32_t prim = wa_lock();
	bool was = wa_armed;

	wa_clear_locked((why != NULL) ? why : "disarmed");
	wa_unlock(prim);
	if (was)
		LOG_INF("disarmed (%s)", (why != NULL) ? why : "disarmed");
}

bool wifi_auto_armed(void)
{
	return wa_armed;
}

bool wifi_auto_attempt_in_flight(void)
{
	return wa_in_flight;
}

/*
 * The forget-generation check, on the paths that may take a lock and print -- never on
 * wifi_auto_armed().  Returns true if the credentials are still for the module in front
 * of us; disarms and returns false if they are not.
 */
static bool wa_check_gen(void)
{
	uint32_t prim;
	bool stale;

	if (!wa_armed)
		return false;
	prim  = wa_lock();
	stale = (wa_armed && wa_forget_gen != rtl_link_forget_gen());
	if (stale)
		wa_clear_locked("the module was power-cycled or reflashed");
	wa_unlock(prim);
	if (stale)
		LOG_INF("disarmed (the module was power-cycled or reflashed)");
	return !stale && wa_armed;
}

bool wifi_auto_should_try(void)
{
	uint32_t prim;
	bool go;

	if (!wa_check_gen())
		return false;
	prim = wa_lock();
	go = wa_armed && (int32_t)(tx_time_get() - wa_next_try) >= 0;
	wa_unlock(prim);
	return go;
}

/* ---- the attempt ------------------------------------------------------------- */

int wifi_auto_attempt(const struct wifi_rpc_opts *o)
{
	char ssid[WIFI_AUTO_SSID_MAX + 1u], pass[WIFI_AUTO_PASS_MAX + 1u];
	uint32_t security, prim;
	int32_t result = -1;
	bool open;
	int rc;

	/* Take a private copy under the section: the call below blocks for many seconds and
	 * a CLI thread may zero the originals while it does (that IS the abort). */
	prim = wa_lock();
	if (!wa_armed) {
		wa_unlock(prim);
		return -1;
	}
	memcpy(ssid, wa_ssid, sizeof ssid);
	memcpy(pass, wa_pass, sizeof pass);
	open     = wa_open;
	security = wa_security;
	wa_attempts++;
	wa_last_try       = tx_time_get();
	wa_last_try_valid = true;
	wa_in_flight      = true;
	wa_unlock(prim);

	rc = wifi_rpc_connect(o, ssid, open ? NULL : pass, security, &result);
	memset(pass, 0, sizeof pass);        /* no passphrase left on the owner's stack */

	prim = wa_lock();
	wa_in_flight   = false;
	wa_last_rc     = rc;
	wa_last_result = result;
	if (rc == -4) {
		/* Aborted, which means somebody disarmed us: not a network failure, so it
		 * neither counts nor backs off.  The caller must now leave the link alone --
		 * the module is still inside wifi_connect(). */
		wa_unlock(prim);
		return -4;
	}
	if (rc == 0 && result == WIFI_RPC_OK) {
		wa_ok++;
		wa_backoff_ms = 0u;
		wa_next_try   = tx_time_get();
		wa_why        = "";
		wa_unlock(prim);
		LOG_INF("re-associated with \"%s\"", ssid);
		return 0;
	}
	wa_failed++;
	wa_backoff_ms = (wa_backoff_ms == 0u) ? WIFI_AUTO_BACKOFF_MIN_MS
	                                      : (wa_backoff_ms * 2u);
	if (wa_backoff_ms > WIFI_AUTO_BACKOFF_MAX_MS)
		wa_backoff_ms = WIFI_AUTO_BACKOFF_MAX_MS;
	wa_next_try = tx_time_get() + (ULONG)wa_backoff_ms;
	wa_unlock(prim);
	LOG_WRN("attempt failed (rc %d, result %ld), retrying in %u ms",
	        rc, (long)result, (unsigned)wa_backoff_ms);
	return (rc != 0) ? rc : -1;
}

/* ---- the report -------------------------------------------------------------- */

void wifi_auto_print(struct cli_instance *sh)
{
	uint32_t prim;
	char ssid[WIFI_AUTO_SSID_MAX + 1u];
	bool enabled, armed, open;
	uint32_t security, backoff, attempts, ok, failed;
	ULONG now, next, last;
	bool last_valid;
	int last_rc;
	int32_t last_result;
	const char *why;

	(void)wa_check_gen();

	prim = wa_lock();
	enabled     = wa_enabled;
	armed       = wa_armed;
	open        = wa_open;
	security    = wa_security;
	backoff     = wa_backoff_ms;
	attempts    = wa_attempts;
	ok          = wa_ok;
	failed      = wa_failed;
	next        = wa_next_try;
	last        = wa_last_try;
	last_valid  = wa_last_try_valid;
	last_rc     = wa_last_rc;
	last_result = wa_last_result;
	why         = wa_why;
	memcpy(ssid, wa_ssid, sizeof ssid);
	wa_unlock(prim);
	now = tx_time_get();

	cli_print(sh, "wifi: autoreconnect %s\r\n", enabled ? "on" : "off");
	if (armed)
		cli_print(sh, "  armed for \"%s\" (%s, sec 0x%08lX)\r\n", ssid,
		          open ? "open" : "passphrase held in RAM", (unsigned long)security);
	else
		cli_print(sh, "  not armed%s%s\r\n", (why && why[0]) ? " -- " : "",
		          (why != NULL) ? why : "");

	cli_print(sh, "  attempts %lu, ok %lu, failed %lu\r\n",
	          (unsigned long)attempts, (unsigned long)ok, (unsigned long)failed);
	if (last_valid)
		cli_print(sh, "  last attempt %lu s ago (rc %d, result %ld)\r\n",
		          (unsigned long)((now - last) / 1000u), last_rc, (long)last_result);
	if (armed && backoff != 0u) {
		int32_t left = (int32_t)(next - now);
		cli_print(sh, "  backoff %u s, next try in %ld s\r\n",
		          (unsigned)(backoff / 1000u), (long)((left > 0) ? left / 1000 : 0));
	}
	if (!enabled)
		cli_print(sh, "  `wifi autoreconnect on`, then `wifi connect <ssid> <pw>` to "
		          "arm it\r\n");
}
