/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    kv_boot.c
 * @brief   The configuration thread: opens the KV store and applies it (issue #37).
 *
 * A thread exists here for one reason: this work waits on hardware.  Opening the
 * store can erase (a first boot formats a blank partition), associating blocks on
 * the module for up to 22 s, and a DHCP lease takes seconds -- all of which need a
 * scheduler.  tx_application_define() runs before there is one, so everything in
 * it must be pure object creation; the work that has to wait belongs on a thread,
 * and this is that thread.
 *
 * WHAT IT DOES, and what each step is allowed to assume:
 *
 *   kv_init()                  open the store (may format a blank partition)
 *   wifi.enable      ? power the radio               : leave it off
 *   wifi.autoconnect ? associate with wifi.ssid      : leave it to the operator
 *   net.mode           dhcp / static / manual
 *   net.shell.autoarm? arm the telnet console        : leave it off
 *
 * EVERY STEP IS INDEPENDENT AND FAIL-SOFT.  A missing key means "do not do this",
 * never "guess"; a step that fails is logged and the next one still runs; and none
 * of them can stop the shell coming up, because every one of these actions can be
 * typed by hand afterwards.  A configuration store that turns a mistyped setting
 * into a board you cannot talk to would be worse than no store at all.
 *
 * WHY THE OUTPUT GOES THROUGH A SHELL INSTANCE: the wifi/net modules this drives
 * report through `struct cli_instance`, and there is no console attached at boot.
 * Rather than make four modules tolerate a NULL instance, the thread owns an
 * instance on the log transport (shell/backend/cli_backend_log.h), so the whole
 * account of what the sequence did lands in `dmesg` -- which is the only place it
 * could be read afterwards anyway.  cli_init() without cli_start(): there is
 * nothing to read, so the instance needs no thread of its own.
 *
 * One known wart in that arrangement: the association claims the console through
 * rtl_link_begin(), and cli_console_claim() raises a GLOBAL cli_xfer_active, not a
 * per-instance one.  So for the length of a boot association every console's
 * TX-wait path stops draining RX -- a Ctrl+C typed on USB CDC during those seconds
 * is noticed late.  It is bounded and it only happens when autoconnect is on, so it
 * is accepted here rather than fixed by changing what that flag means; if a second
 * unattended claimant ever appears, that is the point to make the flag per-console.
 *
 * Priority 15: above the shell (16) so the configuration is settled before a user
 * can reasonably type, but below the USB pump (8) and the IWDG petter (5), neither
 * of which may ever wait behind a flash erase or a 22 s association.
 */
/* Before anything else: cli_backend_log.h pulls in log.h, which defaults LOG_TAG
 * when it is not already set -- so this has to come first to win. */
#define LOG_TAG "cfg"
#include "log.h"

#include "kv.h"

#include "cli.h"
#include "cli_backend_log.h"
#include "nx_net.h"
#include "net_shell.h"
#include "rtl8720.h"
#include "wifi_auto.h"
#include "wifi_connect.h"
#include "wifi_rpc.h"

#include "tx_api.h"

#include <string.h>
#include "mem_sections.h"  /* DTCM_BSS: CPU-only data out of AXI-SRAM (issue #46) */

#define KV_BOOT_PRIORITY    15u

/* 4 KB.  A kv_get_str() costs about 700 bytes of stack on its own -- a whole record
 * buffer plus the decoded copy -- and this thread then calls the association
 * sequence, which nests several modules' frames underneath it.  The shell threads
 * that run the same sequence get 4 KB; there is no reason for this one to have
 * less, and a stack overflow here would be a boot-time fault with no console up. */
#define KV_BOOT_STACK_SIZE  4096u

/* How long to wait for a DHCP lease before giving up and moving on. */
#define KV_BOOT_DHCP_WAIT_MS  30000u
#define KV_BOOT_POLL_MS       250u

static TX_THREAD kv_boot_thread;
static UCHAR     kv_boot_stack[KV_BOOT_STACK_SIZE] DTCM_BSS __attribute__((aligned(8)));

/* The console this sequence reports through: every line ends up in `dmesg`. */
CLI_BACKEND_LOG_DEFINE(kv_boot_tr, "cfg");
static struct cli_instance kv_boot_sh = {
	.tr     = &kv_boot_tr,
	.prompt = "cfg> ",
	/* No stack: cli_start() is never called (see the file header), so the
	 * instance thread that would use one does not exist. */
};

/* ------------------------------------------------------------------ *
 *  Setting readers
 * ------------------------------------------------------------------ *
 * Each returns the configured value or the supplied default.  A key that is absent
 * and a key that is present but of the wrong type are treated the same way -- as
 * "not configured" -- but only the second is logged, because a missing key is the
 * normal case and a mistyped one is a mistake somebody wants to hear about.
 */
static int cfg_bool(const char *key, int dflt)
{
	int v;
	int rc = kv_get_bool(key, &v);

	if (rc == KV_OK)
		return v;
	if (rc != KV_ERR_NOTFOUND)
		LOG_WRN("%s: ignored (%s)", key,
		        (rc == KV_ERR_TYPE) ? "not a bool" : "unreadable");
	return dflt;
}

static uint32_t cfg_u32(const char *key, uint32_t dflt)
{
	uint32_t v;
	int rc = kv_get_u32(key, &v);

	if (rc == KV_OK)
		return v;
	if (rc != KV_ERR_NOTFOUND)
		LOG_WRN("%s: ignored (%s)", key,
		        (rc == KV_ERR_TYPE) ? "not a u32" : "unreadable");
	return dflt;
}

/* Returns 1 when @p buf holds a configured string. */
static int cfg_str(const char *key, char *buf, uint32_t buflen)
{
	int rc = kv_get_str(key, buf, buflen);

	if (rc == KV_OK)
		return 1;
	if (rc != KV_ERR_NOTFOUND)
		LOG_WRN("%s: ignored (%s)", key,
		        (rc == KV_ERR_TYPE) ? "not a string" :
		        (rc == KV_ERR_TRUNC) ? "too long" : "unreadable");
	buf[0] = '\0';
	return 0;
}

/* ------------------------------------------------------------------ *
 *  Steps
 * ------------------------------------------------------------------ */

static void cfg_wifi(void)
{
	char ssid[WIFI_AUTO_SSID_MAX + 1];
	char psk[WIFI_AUTO_PASS_MAX + 1];
	uint32_t security;
	int have_psk, security_explicit;

	if (!cfg_bool("wifi.enable", 0)) {
		/* Nothing to do, and deliberately so: rtl8720_init() has already driven
		 * CHIP_EN low from tx_application_define(), so the module is held OFF
		 * rather than left floating.  That call is unconditional -- it is what
		 * makes the module's state defined at all -- and only powering it ON is
		 * what this setting gates. */
		LOG_INF("wifi.enable is not set -- radio stays off");
		return;
	}

	if (!cfg_bool("wifi.autoconnect", 0)) {
		/*
		 * Power-only.  The boot wait is ours to do here because the paths that
		 * would otherwise do it -- rtl_link_begin() and `wifi ver` -- only wait
		 * when THEY moved CHIP_EN, and both see a module that is already
		 * powered.  Nothing else is racing us to the UART, but leaving the
		 * module mid-boot with the host believing it is ready is the kind of
		 * state that fails once in twenty and is never reproduced.
		 */
		rtl8720_power(true);
		tx_thread_sleep(1500u);
		LOG_INF("radio powered (no autoconnect)");
		return;
	}

	/*
	 * Autoconnect: do NOT power the module here.  rtl_link_begin(sh, true) inside
	 * the sequence powers it AND waits ~1.5 s for it to boot -- but only when it
	 * is the one that moved CHIP_EN (app/rtl_link.c).  Powering it first would
	 * therefore skip the boot wait entirely and put the first eRPC frame on the
	 * wire while the module was still starting.
	 */
	if (!cfg_str("wifi.ssid", ssid, sizeof ssid)) {
		LOG_WRN("wifi.autoconnect is set but wifi.ssid is not -- not associating");
		return;
	}
	have_psk = cfg_str("wifi.psk", psk, sizeof psk);
	security_explicit = (kv_get_u32("wifi.security", &security) == KV_OK);
	if (!security_explicit)
		security = have_psk ? WIFI_RPC_SEC_WPA2_AES_PSK : WIFI_RPC_SEC_OPEN;

	/*
	 * Arm the re-association policy BEFORE associating, not after (issue #32).
	 * wifi_connect_run() captures the credentials on success by calling
	 * wifi_auto_arm(), and that call is a no-op while the policy is disabled --
	 * so enabling it afterwards would leave it armed with nothing, and the first
	 * AP outage would find no credentials to retry with.
	 */
	if (cfg_bool("wifi.autoreconnect", 0))
		wifi_auto_set_enabled(true);

	/*
	 * This used to fail every single time, and issue #40 explains why: the radio's
	 * associations fail intermittently (about half of them, against the 5 GHz AP it
	 * was measured on), and this path is the only one that never got a second go.  An
	 * operator retypes `wifi connect` without thinking of it as a retry; here there is
	 * nobody to do that.  wifi_connect_run() now retries by itself, which is what makes
	 * automatic association work -- there is nothing left for this file to wait for.
	 *
	 * In particular it is NOT a delay problem, so do not "fix" a future recurrence by
	 * growing rtl_link_begin()'s 1.5 s boot wait: a join issued 20 s after CHIP_EN goes
	 * high fails just as readily as one at 1.7 s (measured).
	 */
	LOG_INF("associating with \"%s\"", ssid);
	if (wifi_connect_run(&kv_boot_sh, ssid, have_psk ? psk : NULL, security,
	                     security_explicit, /* earn_gen */ true) != 0)
		LOG_WRN("automatic association failed -- `wifi connect` by hand");

	/* The passphrase is on the stack here and in host RAM inside wifi_auto after
	 * this; wipe our copy on the way out so it is not left in a thread stack that
	 * `devmem` can read.  This is hygiene, not protection -- the value is stored
	 * in the clear on the external flash regardless (see kv.h). */
	memset(psk, 0, sizeof psk);
}

/* Wait for the host stack to hold an address.  Returns 1 when it does. */
static int cfg_wait_ip(unsigned budget_ms)
{
	struct nx_net_info ni;
	unsigned waited = 0u;

	for (;;) {
		if (nx_net_info_get(&ni) == NXN_OK && ni.ip_valid)
			return 1;
		if (waited >= budget_ms)
			return 0;
		tx_thread_sleep(KV_BOOT_POLL_MS);
		waited += KV_BOOT_POLL_MS;
	}
}

static void cfg_net(void)
{
	char mode[16];
	char ip[16], mask[16], gw[16];
	uint32_t a, m, g;

	if (!cfg_str("net.mode", mode, sizeof mode))
		return;

	if (strcmp(mode, "manual") == 0)
		return;

	if (!nx_net_is_up()) {
		/* Not a failure of this step: without an association there is no host
		 * stack to address, and saying so is more useful than a DHCP timeout. */
		LOG_WRN("net.mode=%s ignored -- the host stack is not up", mode);
		return;
	}

	if (strcmp(mode, "dhcp") == 0) {
		if (nx_net_dhcp_renew() != NXN_OK) {
			LOG_WRN("the DHCP client would not start");
			return;
		}
		if (!cfg_wait_ip(KV_BOOT_DHCP_WAIT_MS)) {
			LOG_WRN("no DHCP lease within %u s", KV_BOOT_DHCP_WAIT_MS / 1000u);
			return;
		}
		LOG_INF("DHCP lease acquired");
	} else if (strcmp(mode, "static") == 0) {
		/* All three or none: a static address with a guessed netmask is a
		 * quietly broken network, so an incomplete configuration is refused
		 * rather than completed with defaults. */
		if (!cfg_str("net.ip", ip, sizeof ip) ||
		    !cfg_str("net.mask", mask, sizeof mask) ||
		    !cfg_str("net.gw", gw, sizeof gw)) {
			LOG_WRN("net.mode=static needs net.ip, net.mask and net.gw");
			return;
		}
		if (cli_parse_ipv4(ip, &a) != 0 || cli_parse_ipv4(mask, &m) != 0 ||
		    cli_parse_ipv4(gw, &g) != 0) {
			LOG_WRN("net.ip/mask/gw: not all are dotted quads");
			return;
		}
		if (nx_net_set_static(a, m, g) != NXN_OK) {
			LOG_WRN("the host stack refused the static address");
			return;
		}
		LOG_INF("static address applied");
	} else {
		LOG_WRN("net.mode=%s is not one of dhcp/static/manual", mode);
		return;
	}

	/* An address exists: this is where the telnet console belongs, matching what
	 * `net dhcp` does on the interactive path. */
	if (cfg_bool("net.shell.autoarm", 0)) {
		uint32_t port = cfg_u32("net.shell.port", NET_SHELL_PORT_DEFAULT);
		const char *why = "";

		if (port == 0u || port > 0xFFFFu) {
			LOG_WRN("net.shell.port=%lu is not a port", (unsigned long)port);
		} else if (net_shell_start((uint16_t)port, &why) != 0) {
			LOG_WRN("telnet console not armed: %s", why);
		} else {
			LOG_INF("telnet console arming");
		}
	}
}

/* ------------------------------------------------------------------ *
 *  Thread
 * ------------------------------------------------------------------ */
static void kv_boot_entry(ULONG arg)
{
	(void)arg;

	if (kv_init() != KV_OK) {
		LOG_WRN("configuration store unavailable: %s", kv_state_str());
		return;   /* nothing to apply; every command still works by hand */
	}

	cfg_wifi();
	cfg_net();

	/* The sequence may have ended mid-line (a module that printed without a
	 * newline); flush it so nothing is left sitting in the assembler. */
	cli_backend_log_flush(&kv_boot_tr);

	/* Done.  Returning leaves the thread TERMINATED rather than parked on a
	 * sleep it would never wake from; its stack is not reclaimed either way, but
	 * `thread` then shows what is true instead of an idle thread that will never
	 * run again. */
}

int kv_boot_init(void)
{
	UINT rc;

	/* The reporting instance: objects only (output mutex + event flags), no
	 * thread.  A failure here is not fatal -- but the sequence reports through
	 * this instance, so without it the wifi/net steps have nowhere to write and
	 * are skipped rather than run blind. */
	if (cli_init(&kv_boot_sh) != 0)
		return -1;

	rc = tx_thread_create(&kv_boot_thread, "cfg", kv_boot_entry, 0,
	                      kv_boot_stack, sizeof kv_boot_stack,
	                      KV_BOOT_PRIORITY, KV_BOOT_PRIORITY,
	                      TX_NO_TIME_SLICE, TX_AUTO_START);

	return (rc == TX_SUCCESS) ? 0 : -1;
}
