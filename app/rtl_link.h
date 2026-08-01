/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    rtl_link.h
 * @brief   Ownership of the onboard RTL8720DN link (issue #5, issue #21 increment 8).
 *
 * The module is a single physical resource shared by several unrelated users:
 *
 *   - the `wifi` (L2) and `net` (L3) shell commands, which run multi-RPC flows
 *     (`wifi connect` = tcpip_init -> off -> on(STA) -> connect -> arm the bridge)
 *     that must not interleave with each other,
 *   - the eRPC link itself (USART1 @2 Mbaud), whose frames are multiplexed by the
 *     resident service thread in app/erpc.c,
 *   - the `wifi log` console bridge and the issue-#19 flash download
 *     path, which re-open the SAME UART peripheral (UART9, other baud rates) and read
 *     the SAME strict-SPSC RX ring themselves.
 *
 * So this file owns two things: a COARSE MUTEX that serialises whole command flows,
 * and a REFERENCE COUNT on the eRPC UART so a resident user (since issue #30 B2b that is
 * the L2 bridge owner in app/nx_net.c) can hold it open across many commands.  Together
 * with the service thread's "touch nothing while idle" rule they keep the invariant that
 * the RX ring
 * has exactly ONE consumer at any instant: the eRPC service thread while the UART is
 * referenced, or the command thread while a bridge / flash session owns it.
 *
 * Two API tiers:
 *   (a) thread-agnostic core -- rtl_link_claim/_unclaim, rtl_link_uart_ref/_unref,
 *       rtl_link_force_quiesce, the module lifecycle state.  Usable from any thread
 *       (the bridge owner thread in app/nx_net.c uses these directly).
 *   (b) shell adapters -- rtl_link_begin/_end and rtl_link_hw_claim/_hw_release, which
 *       add the console claim (foreground only, single owner of the console RX) and
 *       print the reason on failure.
 *
 * Lock order when both are needed: THIS coarse mutex first, then app/erpc.c's internal
 * mutex (via rtl_link_uart_ref / erpc_link_lock).  The eRPC service thread takes only
 * the latter, so the order cannot cycle.
 *
 * No clock/RCC/register work of its own -- it only claims the console, powers the
 * module via app/rtl8720.c and opens the eRPC UART (clock-safe).  Clean-room design.
 */
#ifndef APP_RTL_LINK_H
#define APP_RTL_LINK_H

#include <stdbool.h>
#include <stdint.h>

#include "rtl8720.h"     /* enum rtl8720_uart */

struct cli_instance;

/* rtl_link_begin() result. */
enum { RTL_LINK_READY = 0, RTL_LINK_ERR = 1, RTL_LINK_OFF = 2 };

/* How long a claim waits for the coarse mutex before giving up.  Bounded on purpose:
 * a command may hold the link for a long time (`net echo` until Ctrl+C, DHCP for 30 s),
 * and a second console must get a "busy" message rather than hang silently. */
#define RTL_LINK_CLAIM_WAIT_MS 2000u

/* ---- (a) thread-agnostic core ------------------------------------------------- */

/* Create the coarse mutex.  Call ONCE from tx_application_define(); every function
 * below needs it.  Returns 0 on success, -1 if the mutex could not be created (then
 * claims fail and the `wifi`/`net` commands report an error -- fail-soft). */
int  rtl_link_core_init(void);

/* Take / release the coarse link mutex.  Owner-reentrant (ThreadX counts ownership).
 * rtl_link_claim() returns 0 on success, -1 on timeout / no mutex. */
int  rtl_link_claim(uint32_t timeout_ms);
void rtl_link_unclaim(void);

/*
 * Reference-counted open of one RTL8720 UART.  The first reference opens it (@which,
 * @baud); the last closes it.  A reference whose @which / @baud disagrees with the
 * already-open one is REFUSED (-1) instead of silently re-opening: that is what stops
 * `wifi log` (UART9 @115200) from stealing the link from under a resident eRPC user.
 * Bracketed by erpc_link_lock() so the open/close cannot race the service thread.
 * Returns 0 on success, -1 on conflict / failure to bring the UART up.
 *
 * CALLER MUST HOLD THE COARSE MUTEX (rtl_link_claim, or rtl_link_begin/_hw_claim which
 * take it): the reference count protects against another *referencing* user, not against
 * a session that drives the UART directly without referencing it -- which is exactly
 * what the issue-#19 flash download path does (see app/rtl8720_flash.h).  The coarse
 * mutex is what excludes those two.
 */
int  rtl_link_uart_ref(enum rtl8720_uart which, uint32_t baud);
void rtl_link_uart_unref(void);

/* True while someone holds a UART reference (an eRPC session is live). */
bool rtl_link_uart_busy(void);

/*
 * How many references are outstanding.  rtl_link_uart_busy() cannot answer "is anybody
 * ELSE holding it?", because a command that took its own reference (rtl_link_begin) always
 * sees true -- so `wifi link baud`, which must exclude the RESIDENT holder (the L2 bridge
 * owner) and only itself, tests for exactly 1 while holding the coarse mutex.  That is
 * race-free because every reference is taken and dropped under that same mutex.
 */
unsigned rtl_link_uart_refs(void);

/*
 * ---- the link's baud rate is MODULE state (issue #23 U0-3) ---------------------
 *
 * The eRPC link ran at a hard-coded 2 Mbaud everywhere until `wifi link baud` made it
 * changeable.  The rate then has to be remembered somewhere that outlives a single UART
 * open -- otherwise the next rtl_link_begin() re-opens USART1 at 2 Mbaud while the module
 * is still at 6, and the link is dead until a power cycle.  It belongs with the module
 * generation latch (erpc_set_module_gen) because it describes the same thing: the module
 * in front of us.  Both are reset together, at every point that identity can change:
 *   - rtl_link_force_quiesce()   (`wifi on/off/reset` -- CHIP_EN moves)
 *   - rtl_dl_enter()             (any flash session, including an aborted one)
 *   - rtl_link_begin()'s fresh power-on path (drives CHIP_EN directly, NOT via
 *     force_quiesce -- easy to miss)
 * The module always boots at 2 Mbaud, so the reset value is the default.
 */
uint32_t rtl_link_erpc_baud(void);

/*
 * Drop EVERYTHING the host believes about the module in front of it -- link baud back to
 * 2 Mbaud, firmware generation back to unknown (which also restores the conservative
 * wire budget), lwIP-inited and IP-mode back to unknown.
 *
 * Call it from any path that moves CHIP_EN or rewrites the module's flash.  It exists as
 * one function precisely so that set cannot drift apart: rtl_link_force_quiesce() and the
 * fresh power-on paths all go through here, and anything module-scoped added later gets
 * reset everywhere by being added once.
 */
void rtl_link_forget_module(void);

/*
 * Re-open the eRPC UART at @baud, KEEPING the reference count and both generation
 * counters (an open that is really the same session continuing at a different rate).
 *
 * That is only safe with no RESIDENT reference holder: app/net_shell.c detects that its
 * reference was revoked by comparing rtl_link_uart_gen(), and holding the generation
 * still here would hide a close/open from it.  `wifi link baud` enforces
 * rtl_link_uart_refs() == 1 for exactly this reason -- it is a precondition of the
 * design, not a convenience.  Caller must hold the coarse mutex.  Returns 0, or -1 with
 * the link left CLOSED (the caller must then recover, e.g. by re-opening at the old rate).
 */
int rtl_link_uart_rebaud(uint32_t baud);

/* Return codes of rtl_link_set_rate(). */
#define RTL_RATE_OK        0    /* now at the requested rate                          */
#define RTL_RATE_UNCHANGED (-1) /* the module never switched; the link is still healthy */
#define RTL_RATE_DEAD      (-2) /* no answer at either rate -- `wifi reset` required   */

/*
 * Change the rate of the link, both ends.
 *
 * The caller must already hold the coarse mutex and a UART reference, and must have
 * established that the module firmware speaks LINK CTRL (erpc_module_gen() >= 5).
 *
 * The sequence is the delicate part and there is exactly ONE implementation of it, here,
 * because two would eventually disagree:
 *
 *   - LINK_PING first.  The module requires LINK_SETBAUD to carry the sequence byte
 *     following the last CTRL frame it accepted, so pinging makes that true by
 *     construction rather than by hoping nothing has been lost; it also proves the link
 *     is alive at the current rate immediately before we change it.
 *   - the module ACKs on the OLD rate and only then switches, so a lost ACK leaves both
 *     ends where they were and this returns RTL_RATE_UNCHANGED having changed nothing.
 *   - after switching, the host verifies with LINK_PING and, if that fails, tries to put
 *     both ends back.  That fallback is BEST EFFORT and nothing more: the "come back"
 *     message is sent at the rate that just failed, so if the rate failed for signal
 *     reasons it will not arrive either.  RTL_RATE_DEAD says that happened.
 *
 * The guaranteed recovery is `wifi reset` -- a CHIP_EN power cycle, after which the module
 * is at 2 Mbaud and rtl_link_forget_module() has put the host there too.
 *
 * Blocks for up to ~350 ms.  Uses tx_thread_sleep(), never cli_sleep(), so it is callable
 * from a service thread that has no console.
 */
int rtl_link_set_rate(uint32_t baud);

/* Is @baud one of the rates both ends support? */
bool rtl_link_rate_supported(uint32_t baud);

/*
 * ---- generation counters, for a RESIDENT reference holder (issue #21 increment 9) ------
 *
 * An ordinary command takes and drops its reference inside one coarse-mutex section, so its
 * reference can never be revoked underneath it.  A RESIDENT holder is different (issue #21
 * increment 9 introduced one; since issue #30 B2b it is the L2 bridge owner in
 * app/nx_net.c):
 * it holds a reference across many commands, and rtl_link_force_quiesce() (`wifi on/off/
 * reset`) deliberately drops the count to zero while it is holding one.  A plain
 * rtl_link_uart_unref() afterwards would decrement SOMEBODY ELSE'S reference -- the next
 * user's -- and close the UART under them.
 *
 * rtl_link_uart_gen() therefore identifies the current "open": it advances on every 0->1
 * open AND on force-quiesce.  A resident holder records it when it takes the reference and
 * releases with rtl_link_uart_unref_gen(), which is a NO-OP once the generation has moved
 * (its reference no longer exists).  Comparing the current value against the recorded one
 * is also how the holder notices its link was taken away.
 *
 * rtl_link_quiesce_gen() advances ONLY in rtl_link_force_quiesce(), i.e. only on the three
 * recovery commands, all of which drive CHIP_EN.  It is the "the module was power-cycled"
 * ticket a caller can wait for after leaking module-side state (see app/net_shell.c's dirty
 * latch); an ordinary ref/unref does not move it.
 */
uint32_t rtl_link_uart_gen(void);
void     rtl_link_uart_unref_gen(uint32_t gen);
uint32_t rtl_link_quiesce_gen(void);

/*
 * rtl_link_forget_gen() advances on every rtl_link_forget_module(), which is the strictly
 * wider set: force-quiesce AND the flash session's own recovery AND rtl_link_begin()'s
 * fresh power-on path, which drives CHIP_EN without going through force_quiesce and so
 * does not move rtl_link_quiesce_gen().
 *
 * It exists for a holder of state that is only meaningful for ONE module -- issue #32's
 * stored WiFi credentials (app/wifi_auto.c) -- to notice, on its own, that the module in
 * front of us has changed.  Having the holder latch and watch this counter is what keeps
 * rtl_link_forget_module() free of calls up into the layers that own such state.
 */
uint32_t rtl_link_forget_gen(void);

/*
 * Take the link away from whoever holds it, unconditionally: abandon every in-flight
 * eRPC token (their waiters wake up with -2), drop the reference count to zero and
 * close the UART.  This is the recovery primitive behind `wifi off` / `wifi reset` --
 * a power-cycle invalidates all module state anyway, and it must never be blocked by
 * "the link is in use", because that is exactly when it is needed.  Call it with the
 * coarse mutex held, BEFORE driving CHIP_EN.
 */
void rtl_link_force_quiesce(void);

/*
 * Whether the module's lwIP stack (tcpip_adapter_init = LwIP_Init) has been brought
 * up since its last power-on.  The factory firmware does NOT init lwIP at boot, so
 * the host must -- once per CHIP_EN power cycle (lwIP state survives wifi off/on but
 * not a power cycle).  Reset on every power off / reset / fresh power-on; set after a
 * successful tcpip init.
 */
bool rtl_tcpip_inited(void);
void rtl_tcpip_set_inited(bool v);

/*
 * (The host-side memo of how the address was obtained -- rtl_ip_mode() -- lived here
 * until issue #30 B1.  The module no longer takes an address at all, and the host
 * stack keeps its own dhcp-vs-static flag in struct nx_net_info.)
 */

/* ---- (b) shell adapters ------------------------------------------------------- */

/*
 * Open an eRPC session for a shell command: claim the console (foreground only), take
 * the coarse mutex, ensure the module is powered and reference the eRPC UART
 * (USART1 @2 Mbaud).
 *   @power_on true : power the module and wait for boot if it is off (for `connect`).
 *   @power_on false: if the module is off, release and return RTL_LINK_OFF so the
 *                    caller can report "powered off" without powering it (status /
 *                    net queries are pure reads).
 * Returns RTL_LINK_READY with everything held (caller must rtl_link_end()),
 * RTL_LINK_OFF (nothing held), or RTL_LINK_ERR (nothing held, message already
 * printed).  A fresh power-on here resets the host-tracked lwIP / IP-mode state.
 */
int  rtl_link_begin(struct cli_instance *sh, bool power_on);

/* Tear down an rtl_link_begin() session (drop the UART reference, release the coarse
 * mutex, release the console) -- in that order. */
void rtl_link_end(struct cli_instance *sh);

/*
 * Claim the module for a command that drives the hardware WITHOUT the eRPC link: the
 * power/reset commands and the `wifi log` bridge + flash download paths, which open the
 * UART themselves.  Console claim + coarse mutex; no UART reference is taken.
 *   @allow_busy false: refuse (with a message) while an eRPC session holds the UART --
 *                      a bridge/flasher must not re-open the peripheral under it.
 *   @allow_busy true : proceed anyway; for the RECOVERY commands (`wifi on/off/reset`),
 *                      which call rtl_link_force_quiesce() before touching CHIP_EN.
 * Returns 0 with both held (caller must rtl_link_hw_release()), non-zero otherwise
 * (nothing held, message already printed).
 */
int  rtl_link_hw_claim(struct cli_instance *sh, bool allow_busy);
void rtl_link_hw_release(struct cli_instance *sh);

/* Abort thunk for wifi_rpc_opts.should_abort: non-zero once Ctrl+C was pressed. */
int  rtl_abort_cb(void *ctx);

#endif /* APP_RTL_LINK_H */
