/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    wifi_connect.c
 * @brief   The association sequence (issue #37 step 4).
 *
 * See wifi_connect.h for why this is a module of its own.  Everything below was
 * moved verbatim out of shell/cmds/cmd_wifi.c, where it grew across issues #5,
 * #30 and #32; the only change is that the sequence now takes its parameters
 * instead of reading argv, and can earn the firmware generation itself when the
 * caller has no operator to ask.
 */
#include "wifi_connect.h"

#include "cli.h"
#include "rtl8720.h"
#include "erpc.h"
#include "wifi_rpc.h"
#include "rtl_link.h"
#include "wifi_auto.h"   /* issue #32: the host's own re-association policy */
#include "net_shell.h"
#include "nx_net.h"      /* the host stack owns the module while it is up (#23 U3) */
#include "stm32h7xx_hal.h"   /* HAL_GetTick: the retry budget in issue #40 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/*
 * Extract N from a "2.1.3+wio-nN" build id; 0 if the string is not one of ours.
 *
 * The generation is what gates every firmware-dependent capability (see
 * erpc_set_module_gen), so it has to be read as an ordered number: a string compare
 * against the newest id withdraws capabilities as soon as the next one is flashed, and a
 * compare against an old one keeps claiming them after a downgrade.
 */
uint8_t wifi_fw_gen_of(const char *ver)
{
	const char *p = strstr(ver, "wio-n");
	unsigned n = 0u;

	if (p == NULL)
		return 0u;
	p += 5;                                     /* past "wio-n" */
	if (*p < '0' || *p > '9')
		return 0u;
	while (*p >= '0' && *p <= '9' && n < 255u)
		n = n * 10u + (unsigned)(*p++ - '0');
	return (uint8_t)((n > 255u) ? 255u : n);
}

/*
 * Coordinate a long eRPC flow with the L2 bridge (issue #30 B2b).
 *
 * These flows used to be REFUSED while the host stack was up (`nx_net_guard`), which is
 * why recovering from an AP outage meant tearing the whole interface down.  Two things
 * made the refusal obsolete: issue #30 B1 stopped `wifi connect` touching the module's
 * lwIP address and DHCP client, and the SDK shows the tap survives the off->on(STA)
 * cycle (`netif_add()` runs only inside LwIP_Init(); wifi_on/off only move netif
 * up/down flags, never netif->input).
 *
 * What DOES still conflict is time: the owner refreshes the module's bridge watchdog by
 * taking the coarse mutex, and these flows hold it far longer than the hold.  So instead
 * of refusing, hold the watchdog open first -- and if that cannot be done, refuse after
 * all, because starting the flow would drop the bridge somewhere in the middle of it.
 *
 * Call with the session open (coarse mutex held).  Returns 0 to proceed.
 */
int wifi_hold_bridge(struct cli_instance *sh, const char *what)
{
	if (nx_net_hold_extend() == 0)
		return 0;
	cli_error(sh, "%s: refused -- the module did not renew the L2 bridge, and starting "
	          "now would drop it mid-flow; retry, then `wifi reset`\r\n", what);
	return 1;
}

/*
 * Bring the L2 bridge up after a successful association (issue #30 B2b).  MUST be called
 * with the eRPC session already closed -- see the note at the call site.  Prints what
 * happened and returns the command's exit status.
 */
#define WIFI_ARM_POLL_MS   100u
#define WIFI_ARM_WAIT_MS   20000u

static int wifi_arm_bridge(struct cli_instance *sh)
{
	const char *why = "";
	unsigned waited = 0u;

	if (nx_net_state() != NX_NET_OFF) {
		/* Already bridged: re-associating did not disturb the tap (it survives the
		 * off->on(STA) cycle), so there is nothing to do.  The link flag follows on
		 * the owner's next association poll, within NXN_REFRESH_MS. */
		cli_print(sh, "  the host stack is already up (link follows within ~8s)\r\n");
		return 0;
	}
	if (nx_net_up(&why) != 0) {
		cli_error(sh, "  the host stack did not come up -- %s\r\n", why);
		return 1;
	}
	cli_print(sh, "  bringing the host stack up...\r\n");
	while (nx_net_state() == NX_NET_ARMING && waited < WIFI_ARM_WAIT_MS) {
		/* Ctrl+C stops WAITING, not the owner: a half-installed bridge must be
		 * unwound by the owner, never by a second thread (app/nx_net.h). */
		if (cli_cancel_requested(sh)) {
			cli_print(sh, "  still arming in the background; `net info` to check\r\n");
			return 1;
		}
		cli_sleep(sh, WIFI_ARM_POLL_MS);
		waited += WIFI_ARM_POLL_MS;
	}
	if (!nx_net_is_up()) {
		nx_net_print_status(sh);
		return 1;
	}
	cli_print(sh, "  host stack up -- run `net dhcp` for an address\r\n");
	return 0;
}

/* The two eRPC calls wifi_enter_sta() makes, and its settle -- named so the retry
 * budget below can account for the whole attempt rather than just the join. */
#define WIFI_STA_RPC_TMO_MS   5000u
#define WIFI_STA_SETTLE_MS    50u
#define WIFI_ENTER_STA_MAX_MS (2u * WIFI_STA_RPC_TMO_MS + WIFI_STA_SETTLE_MS)

/*
 * How many times to attempt an association, and the wall-clock ceiling across all of
 * them (issue #40).  The measured per-attempt failure rate was about one in two on the
 * 5 GHz AP, so three attempts take that to roughly one in eight, and the observed
 * failures came back in 4-7.5 s (a boot-path attempt took the longest) -- fast enough
 * for all three to run.
 *
 * BUDGET_MS is a REAL ceiling, not merely a gate on starting: a further attempt is begun
 * only when its ENTIRE worst case -- wifi_enter_sta()'s two 5 s calls plus a full-length
 * join -- would still land inside it.  Both halves have to be counted.  Bounding just
 * the join lets an attempt admitted at 22 s run to 54 s, and bounding nothing but
 * `elapsed < budget` lets one admitted at 29 s do the same; either way the constant
 * would not describe the behaviour, which is the only thing that makes it useful.
 *
 * The cost of stating it honestly is that BUDGET_MS has to be roomy enough to hold that
 * worst case, so it is far larger than the time this normally takes.  Where it lands:
 * three attempts that fail at the measured speed finish in ~22 s and all three run,
 * while a run of full-length timeouts stops after the first -- and a join that spends
 * its whole 22 s is the module saying it never heard the AP, which repeating cannot fix.
 */
#define WIFI_JOIN_ATTEMPTS    3u
#define WIFI_JOIN_TIMEOUT_MS  22000u   /* the module gives ITSELF 20 s -- see the join */
#define WIFI_JOIN_BUDGET_MS   60000u

/* The Ctrl+C-abortable option block every eRPC call in this file uses. */
void wifi_opts(struct wifi_rpc_opts *o, struct cli_instance *sh,
                      struct erpc_diag *diag)
{
	o->should_abort = rtl_abort_cb;
	o->abort_ctx    = sh;
	o->diag         = diag;
}

/* The failure epilogue the eRPC subcommands share: what the link saw, and the way out. */
void wifi_fail_diag(struct cli_instance *sh, const struct erpc_diag *diag)
{
	cli_print(sh, "  diag: crc_fail %u oversize %u timeout %u skipped %u unsupported %u\r\n",
	          diag->crc_fail, diag->oversize, diag->timeout, diag->skipped_reply,
	          diag->unsupported_invocation);
	cli_print(sh, "  note: if the module seems stuck, `wifi reset` power-cycles it\r\n");
}

/* Land the radio in STA mode the way rpcWiFi's mode() does.  Boot leaves WiFi running
 * in RTW_MODE_NONE and wifi_on() then early-returns "already running" (1) WITHOUT
 * switching mode, so cycle off then on(STA).  wifi_on returns 0 (fresh) or 1 (already
 * on); only RTW_ERROR (<0) is a real failure.  Returns 0, or non-zero with the reason
 * already printed (a Ctrl+C during the settle sleep returns silently, like the
 * callers' other abort paths). */
int wifi_enter_sta(struct cli_instance *sh, struct wifi_rpc_opts *o)
{
	int32_t result = -1;
	int rc;

	o->timeout_ms = WIFI_STA_RPC_TMO_MS;
	(void)wifi_rpc_off(o, &result);             /* best effort: ignore off's result */
	if (cli_sleep(sh, WIFI_STA_SETTLE_MS))      /* let the driver settle (Ctrl+C ok) */
		return -4;
	o->timeout_ms = WIFI_STA_RPC_TMO_MS;
	rc = wifi_rpc_on(o, WIFI_RPC_MODE_STA, &result);
	if (rc == -4) { cli_print(sh, "wifi: aborted\r\n"); return -4; }
	if (rc || result < 0) {
		cli_error(sh, "wifi: set STA mode failed (rc %d, result %ld)\r\n",
		          rc, (long)result);
		return -1;
	}
	return 0;
}
/* The association sequence (issue #5 inc 3): put the module in STA mode, associate
 * with the AP, and arm the L2 bridge the host stack rides on (issue #30 B2b) -- L3
 * is `net`'s job, see the note at the end of this function.  The steps are
 * synchronous eRPC calls on the USART1 link (at whatever rate it runs now);
 * associating blocks on the module up to 22 s (abortable with Ctrl+C). */
int wifi_connect_run(struct cli_instance *sh, const char *ssid, const char *pass,
                     uint32_t security, bool security_explicit, bool earn_gen)
{
	struct wifi_rpc_opts o;
	struct erpc_diag diag;
	uint32_t t_start = HAL_GetTick();
	unsigned attempt;
	int32_t result = -1;
	int rc;

	/* Not from the telnet console: dropping the association takes the very session
	 * running this command away.  Run it from USB CDC. */
	if (net_shell_guard(sh, "wifi connect"))
		return 1;
	/*
	 * Disarm before claiming (issue #32).  An operator typing `wifi connect` during an
	 * outage is the very moment an automatic attempt is likely to be holding the coarse
	 * mutex, and rtl_link_begin() below would give up on it after RTL_LINK_CLAIM_WAIT_MS
	 * and report a busy link.  The credentials are about to be replaced by this command
	 * anyway, so cutting the attempt short costs nothing: success re-arms below.
	 */
	wifi_auto_disarm("superseded by `wifi connect`");
	if (rtl_link_begin(sh, true) != RTL_LINK_READY)
		return 1;
	/*
	 * Check the firmware proof BEFORE spending 15 s associating (issue #30 B2b).
	 *
	 * Since associating is what brings the interface up, a connect that cannot bridge
	 * leaves the module joined to an AP with no way to carry a packet -- which is worse
	 * than not connecting, because it looks like it worked.  The generation is dropped
	 * by every CHIP_EN move and every flash session (rtl_link_forget_module), so this
	 * fires exactly when the host genuinely does not know what is on the module.
	 *
	 * It cannot be earned automatically: rpc_system_version corrupts the heap of
	 * pre-N2 firmware, which is why `wifi ver` is a deliberate act (issue #20).
	 */
	if (erpc_module_gen() < WIFI_BRIDGE_MIN_GEN && earn_gen) {
		/*
		 * Earn it here (issue #37 step 4).  An unattended sequence has nobody to
		 * ask, and it can never inherit the proof either: rtl8720_init() drives
		 * CHIP_EN low at every boot, so the module in front of us has just been
		 * power-cycled and the generation is genuinely unknown again.
		 *
		 * The link session is already open here, which is the only reason this is
		 * one call rather than a repeat of `wifi ver`.  It is still the same act
		 * with the same caveat -- erpc_system_version corrupts the heap of pre-N2
		 * firmware -- which is why only a caller that asked for it gets it.
		 */
		char ver[64];
		struct erpc_diag vdiag = {0};

		if (erpc_system_version(ver, (uint16_t)sizeof ver, &vdiag) >= 0) {
			erpc_set_module_gen(wifi_fw_gen_of(ver));
			cli_print(sh, "wifi: module firmware %s (gen n%u)\r\n", ver,
			          (unsigned)erpc_module_gen());
		} else {
			cli_error(sh, "wifi: the module did not answer a version query\r\n");
		}
	}
	if (erpc_module_gen() < WIFI_BRIDGE_MIN_GEN) {
		rtl_link_end(sh);
		cli_error(sh, "wifi: the host cannot bridge this module -- it does not know "
		          "which firmware is loaded\r\n");
		cli_print(sh, "  run `wifi ver` first.  Any `wifi reset` or flash session "
		          "drops that proof, so it has to be re-earned after one.\r\n");
		return 1;
	}
	if (wifi_hold_bridge(sh, "wifi connect")) {
		rtl_link_end(sh);
		return 1;
	}

	wifi_opts(&o, sh, &diag);

	/* 0) Bring up the module's lwIP stack (tcpip_adapter_init = LwIP_Init) BEFORE the
	 * WiFi driver (re)binds in step 1: the factory firmware never inits lwIP at boot
	 * (setup() only calls wifi_init()), so without this the STA netif never exists and
	 * LwIP_DHCP(DHCP_START) blocks forever.  Once per power-on (state survives off/on). */
	if (!rtl_tcpip_inited()) {
		o.timeout_ms = 5000u;
		rc = wifi_rpc_tcpip_init(&o, &result);
		if (rc || result != WIFI_RPC_OK) {
			cli_error(sh, "wifi: tcpip/lwIP init failed (rc %d, result %ld)\r\n",
			          rc, (long)result);
			goto fail;
		}
		rtl_tcpip_set_inited(true);
	}

	/*
	 * 1+2) STA mode, then associate -- and do it again if the radio simply had a bad
	 * draw (issue #40).
	 *
	 * Measured on board #2 against a 5 GHz AP at -63 dBm: 4 of 8 associations failed
	 * with the module reporting RTW_NONE_NETWORK or RTW_4WAY_HANDSHAKE_TIMEOUT, while
	 * `wifi scan` listed that AP every time and 11 of 11 associations to the same
	 * router's 2.4 GHz SSID (20 dB WEAKER) succeeded.  It is not the credentials
	 * (RTW_WRONG_PASSWORD never appeared), not elapsed time since power-on (a join 20 s
	 * after CHIP_EN fails as readily as one at 1.5 s), not the first-call-only
	 * LwIP_Init above (an attempt right after a SUCCESSFUL one fails too), and not the
	 * module's channel plan (it ships as RT_CHANNEL_DOMAIN_REALTEK_DEFINE; installing
	 * FCC1_FCC1 did not stop the failures).  Both reported reasons are transient radio
	 * outcomes, and a repeat attempt is what has always recovered them.
	 *
	 * That is why this looks like "typing `wifi connect` twice works": the operator was
	 * the retry loop.  The boot configuration (issue #37) has no operator, gets exactly
	 * one attempt, and so was left failing every time -- which is the bug reported.
	 *
	 * Both steps repeat, not just the join: every manual recovery went through
	 * wifi_enter_sta() again, so that is the sequence with evidence behind it.
	 */
	for (attempt = 1; ; attempt++) {
		int32_t why = WIFI_RPC_ERR_UNKNOWN;
		uint32_t elapsed;

		if (wifi_enter_sta(sh, &o) != 0)
			goto fail;

		/*
		 * WIFI_JOIN_TIMEOUT_MS is 22 s, because the module gives ITSELF 20
		 * (rtw_down_timeout_sema(join_sema, 20000) inside lib_arduino.a's
		 * wifi_connect).  The 15 s this used to wait expired while the module was
		 * still trying, so its answer came back as a stale frame and its serial mutex
		 * stayed held for another five seconds -- long enough for the next command to
		 * queue behind a call that had already been given up on (issue #32).  Do not
		 * shorten it to fit more retries in: that reintroduces the stale reply.
		 */
		cli_print(sh, "wifi: connecting to \"%s\"%s...\r\n",
		          ssid, pass ? "" : " (open)");
		o.timeout_ms = WIFI_JOIN_TIMEOUT_MS;
		rc = wifi_rpc_connect(&o, ssid, pass, security, &result);
		if (rc == -4) { cli_print(sh, "wifi: aborted\r\n"); goto fail; }
		if (rc == 0 && result == WIFI_RPC_OK)
			break;                       /* associated */

		cli_error(sh, "wifi: connect failed (rc %d, result %ld)\r\n", rc, (long)result);
		/*
		 * Ask the module WHY.  wifi_connect() collapses every reason into one
		 * RTW_ERROR, and the reason it does record is cleared when the next attempt
		 * starts -- so this has to run before the retry below, not after it.  Only
		 * after a module-side failure: a transport error (rc != 0) means the -1 is
		 * ours, and the flag would describe some earlier attempt instead of this one.
		 */
		if (rc == 0) {
			int qrc;

			o.timeout_ms = 3000u;
			qrc = wifi_rpc_get_last_error(&o, &why);
			/* Ctrl+C HERE still means stop.  Without this the abort would only be
			 * noticed by the settle sleep inside the next attempt's
			 * wifi_enter_sta() -- after that attempt had already sent the module a
			 * wifi_off, which is a command the operator asked us not to send. */
			if (qrc == -4) { cli_print(sh, "wifi: aborted\r\n"); goto fail; }
			if (qrc == 0)
				cli_print(sh, "  module reason: %s (%ld)\r\n",
				          wifi_rpc_err_name(why), (long)why);
		}

		/*
		 * Repeating cannot improve a key the module rejected outright, so that answer
		 * ends it.  Everything else retries, INCLUDING a reason we could not read:
		 * the observed failures are transient, and treating "unknown" as fatal would
		 * put the reported bug back for any of them.
		 *
		 * Do NOT read this as "a wrong passphrase costs one attempt".  It does not,
		 * and the reason codes cannot be made to say so.  In the SDK's disconnect
		 * handler (wifi_conf.c, the WPA2_AES_PSK branch) a join that got all the way
		 * to JOIN_LINK_READY is classified by the AP's OWN disconnect reason:
		 * REASON_4WAY_HNDSHK_TIMEOUT yields RTW_4WAY_HANDSHAKE_TIMEOUT and anything
		 * else yields RTW_WRONG_PASSWORD.  A wrong 16-character passphrase was
		 * measured reporting the TIMEOUT, and a correct one reports it too whenever
		 * the radio has a bad draw -- the same code for both.  RTW_WRONG_PASSWORD
		 * shows up for keys refused before the join (a length outside 8..63), which
		 * is a real case worth short-circuiting but not the common one.
		 *
		 * So a genuinely wrong passphrase spends all WIFI_JOIN_ATTEMPTS against the
		 * AP.  That is accepted rather than fixed: the count is small and bounded,
		 * and the alternative -- refusing to retry a handshake timeout -- would
		 * restore the failure this whole loop exists to remove.
		 */
		if (why == WIFI_RPC_ERR_WRONG_PASSWORD)
			goto no_retry;
		if (attempt >= WIFI_JOIN_ATTEMPTS)
			goto no_retry;
		/*
		 * Bound the wall-clock too, not just the count -- this loop holds the coarse
		 * link mutex and the console for its whole duration.  The test is whether
		 * a WHOLE further attempt still fits -- wifi_enter_sta() included, not just
		 * the join -- and not merely whether we are under the budget now; see
		 * WIFI_JOIN_BUDGET_MS.  The subtraction is modulo 2^32, so a HAL_GetTick()
		 * wrap mid-sequence still yields the true elapsed time.
		 */
		elapsed = HAL_GetTick() - t_start;
		if (elapsed + WIFI_ENTER_STA_MAX_MS + WIFI_JOIN_TIMEOUT_MS >
		    WIFI_JOIN_BUDGET_MS) {
			cli_print(sh, "  giving up after %lu ms -- another attempt would not "
			          "finish inside the %u ms budget\r\n",
			          (unsigned long)elapsed, WIFI_JOIN_BUDGET_MS);
			goto no_retry;
		}
		cli_print(sh, "  retrying (attempt %u of %u)...\r\n",
		          attempt + 1u, WIFI_JOIN_ATTEMPTS);
		continue;

no_retry:
		/* Only when the caller took the default: someone who already named a
		 * security value does not need to be told to name one. */
		if (!security_explicit)
			cli_print(sh, "  hint: try an explicit security, e.g. "
			          "`wifi connect \"%s\" <pw> 0x00600006`\r\n", ssid);
		goto fail;
	}
	if (attempt > 1u)
		cli_print(sh, "  (associated on attempt %u)\r\n", attempt);

	/*
	 * ...and that is the whole command (issue #30 B1).  It used to run the MODULE's DHCP
	 * client here and print the lease, which is why `wifi` (L2) ended up owning an L3
	 * step; the address it produced belongs to the module's lwIP, and the host stack --
	 * the only thing that carries traffic since issue #23 U4 -- throws it away
	 * immediately (arming the bridge stops that DHCP client and zeroes the netif address, because
	 * the WLAN driver filters received IP against it).  So association is all that
	 * happens here, and L3 lives entirely in `net`.
	 */
	rtl_link_end(sh);
	cli_print(sh, "wifi: connected\r\n");
	/*
	 * Capture the credentials for issue #32.  A no-op unless `wifi autoreconnect on` has
	 * been run, which is what keeps holding a passphrase in RAM an opt-in rather than
	 * something every association does behind the operator's back.
	 */
	wifi_auto_arm(ssid, pass, security);
	if (wifi_auto_enabled())
		cli_print(sh, "  autoreconnect armed for \"%s\"\r\n", ssid);
	/*
	 * Bring the bridge up as part of associating (issue #30 B2b), so there is no
	 * user-visible "which stack owns the network" mode left.
	 *
	 * ORDER IS LOAD-BEARING: rtl_link_end() above released the coarse mutex and our
	 * UART reference FIRST.  nxn_arm() runs on the owner thread and takes exactly those
	 * two, in that order -- so arming while we still held them could never succeed; it
	 * would spin until NXN_ARM_CLAIM_MS and report "the link stayed busy".
	 */
	return wifi_arm_bridge(sh);

fail:
	rtl_link_end(sh);
	wifi_fail_diag(sh, &diag);
	return 1;
}
