/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/*
 * Wio Lite AI (STM32H725AEI6) -- host-side automatic re-association (issue #32).
 *
 * When the AP disappears, is switched off or deauthenticates us, the association goes and
 * nothing brings it back.  This file is the policy that does: the credentials of the last
 * successful `wifi connect`, an enable flag, a backoff, and the one eRPC call that
 * re-associates.  The DETECTION and the SEQUENCING live in app/nx_net.c, which already
 * samples the association every NXN_REFRESH_MS with the coarse link mutex in hand; this
 * file never claims anything and never decides when it is safe to talk to the module.
 *
 * ---- why not the module's own autoreconnect ------------------------------------
 *
 * `wifi_set_autoreconnect()` installs a handler that runs wifi_connect() and then
 * LwIP_DHCP(0, START), falling back to LwIP_AUTOIP when nothing answers (disassembly of
 * lib_arduino.a:wifi_conf.o, issue #31).  While the L2 bridge is in, not one byte reaches
 * the module's lwIP, so that DHCP always times out and AutoIP puts 169.254.x.x on
 * xnetif[0] -- and the WLAN driver's netif_is_valid_IP() returns 1 unconditionally ONLY
 * while the netif address is zero.  The moment it is not, host-bound unicast IP is dropped
 * before it ever reaches netif_rx().  Enabling the module's feature therefore kills the
 * bridge silently, which is why the host re-issues rpc_wifi_connect itself: the module's
 * DHCP never runs, the netif address stays zero, and the tap is untouched.
 *
 * (wifi_connect()'s own success path only calls netif_set_link_up() and
 * restore_wifi_info_to_flash() -- and the latter early-returns while
 * p_write_reconnect_ptr is NULL, which it is precisely because we never call
 * wifi_set_autoreconnect.  So a re-association writes no module flash either.)
 *
 * ---- the credentials -----------------------------------------------------------
 *
 * Re-associating needs the SSID and the passphrase, so they are kept in host RAM.
 *
 * THIS IS ON BY DEFAULT, so an ordinary `wifi connect` leaves its passphrase there.  That
 * is a real exposure -- `devmem` and the telnet console can both read RAM -- and it was an
 * opt-in when the feature landed for exactly that reason.  It is a default now because the
 * preference could only ever be set BEFORE the outage it protects against, so requiring it
 * meant the feature was off in most of the cases it exists for.  `wifi autoreconnect off`
 * opts out and wipes what is held; nothing is persisted, so a reset comes back on.
 *
 * The credentials are captured by a successful `wifi connect` rather than dug out of the
 * module, and they are zeroed on every path that ends the association or changes the
 * module underneath us (see wifi_auto_disarm() and the forget-generation backstop below).
 *
 * ---- threading -----------------------------------------------------------------
 *
 * Two threads touch this state: a CLI thread running the `wifi` commands, and the nx_net
 * owner thread running an attempt.  Updates are made under a short PRIMASK critical
 * section so that no reader can catch the credentials, the armed flag and the backoff
 * half-updated (the same reason nx_net.c's nxn_mod_accumulate() masks).  The one
 * exception is wifi_auto_armed(), which is polled roughly every millisecond from
 * erpc_call_ex()'s abort hook and therefore only ever reads a single volatile bool.
 *
 * No clock/RCC/register work of its own (clock-safe).  Clean-room design.
 */
#ifndef APP_WIFI_AUTO_H
#define APP_WIFI_AUTO_H

#include <stdbool.h>
#include <stdint.h>

#include "wifi_rpc.h"    /* struct wifi_rpc_opts */

struct cli_instance;

/* Module-imposed bounds (rpc_wifi_connect / wifi_connect take no longer strings). */
#define WIFI_AUTO_SSID_MAX 32u
#define WIFI_AUTO_PASS_MAX 64u

/*
 * Backoff after a failed attempt, in milliseconds.  The first loss is retried at once --
 * an AP that bounced is back before the second sample -- and only a run of failures backs
 * off, so a genuinely absent AP costs one attempt a minute rather than one every 8 s.
 */
#define WIFI_AUTO_BACKOFF_MIN_MS 8000u
#define WIFI_AUTO_BACKOFF_MAX_MS 60000u

/* `wifi autoreconnect on|off`.  Turning it off also disarms (the credentials go). */
void wifi_auto_set_enabled(bool on);
bool wifi_auto_enabled(void);

/*
 * Remember what `wifi connect` just associated with, so an attempt can repeat it.  A no-op
 * unless autoreconnect is enabled -- that is what makes keeping the passphrase opt-in.
 * @password may be NULL (open network).
 */
void wifi_auto_arm(const char *ssid, const char *password, uint32_t security);

/*
 * Forget the credentials and stop attempting.  @why is remembered for `wifi autoreconnect`.
 *
 * This is ALSO the abort: an attempt in flight sees wifi_auto_armed() go false through its
 * should_abort hook and returns within about a millisecond.  Making disarm and abort the
 * same act is what removes the ordering hazard a separate cancel flag would have -- there
 * is no window in which a command has asked to stop and an attempt starts anyway.
 *
 * Callable from any thread; takes no lock a command could be holding.
 */
void wifi_auto_disarm(const char *why);

/* Polled from erpc_call_ex()'s abort hook: a single volatile read, no generation check. */
bool wifi_auto_armed(void);

/*
 * Is an attempt blocked on the module right now?  Only for the console, and only so that
 * disarming can say whether it actually cut a join short -- claiming it unconditionally
 * would describe a module-side state that usually does not exist.  Inherently a snapshot:
 * the attempt may finish between this call and the next line.
 */
bool wifi_auto_attempt_in_flight(void);

/* Armed, the module still the one we armed against, and the backoff expired. */
bool wifi_auto_should_try(void);

/*
 * Re-issue rpc_wifi_connect with the stored credentials and fold the outcome into the
 * statistics and the backoff.  The caller owns the coarse link mutex and has already
 * extended the module's bridge hold; @o carries the timeout, the abort hook and the diag
 * block.  Returns 0 when the module reported success, -4 when the attempt was aborted
 * (the caller must then issue NO further eRPC and release the link at once -- the module
 * is still inside wifi_connect() and holding its serial mutex), or another negative value
 * for a transport failure or a module error.
 */
int wifi_auto_attempt(const struct wifi_rpc_opts *o);

/* The `wifi autoreconnect` report: state, credentials in force, tally, backoff. */
void wifi_auto_print(struct cli_instance *sh);

#endif /* APP_WIFI_AUTO_H */
