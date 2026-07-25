/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/*
 * Ownership of the onboard RTL8720DN link (issue #5; promoted from shell/cmds/ and
 * given the coarse mutex + UART reference count in issue #21 increment 8).
 * See rtl_link.h for the model and the lock order.  No clock/RCC/register access of
 * its own (XIP-safe).
 */
#include "rtl_link.h"

#include "tx_api.h"
#include "cli.h"
#include "erpc.h"
#include "rtl8720.h"

/* Coarse link mutex: serialises whole command flows against each other.  The eRPC
 * service thread deliberately does NOT take it -- that is what lets another thread's
 * RPCs slip in between the RPCs of a running command (issue #21 increment 9). */
static TX_MUTEX g_link_mutex;
static uint8_t  g_mutex_ready;

/* Reference count on the eRPC/LOG UART, and which configuration it is open with. */
static unsigned          g_uart_refs;
static enum rtl8720_uart g_uart_which;
static uint32_t          g_uart_baud;

/* Generation counters for resident reference holders (see rtl_link.h).  Both are only ever
 * touched under erpc_link_lock(), the same section that mutates g_uart_refs. */
static uint32_t g_uart_gen = 1u;     /* ++ on every 0->1 open and on force-quiesce */
static uint32_t g_quiesce_gen;       /* ++ on force-quiesce only (= CHIP_EN moved)  */

/* The rate the eRPC link currently runs at.  MODULE state, not link state -- see
 * rtl_link_erpc_baud() in rtl_link.h for why, and for the four places that reset it. */
#define RTL_ERPC_BAUD_DEFAULT 2000000u
static uint32_t g_erpc_baud = RTL_ERPC_BAUD_DEFAULT;

/* Whether the module's lwIP stack has been brought up since its last power-on.  See
 * rtl_link.h; reset on every power off / reset / fresh power-on, set after tcpip init. */
static bool g_tcpip_inited;

/* Host-side memo of DHCP-vs-static for `net info` (see rtl_link.h). */
static enum rtl_ipmode g_ip_mode;

/*
 * Everything the host believes about the module in front of it, dropped in one place.
 * Called from every point where that identity can change; keeping it a single function
 * is what stops the set from drifting apart as more module-scoped state appears.
 */
void rtl_link_forget_module(void)
{
	g_erpc_baud = RTL_ERPC_BAUD_DEFAULT;   /* the module always boots at 2 Mbaud */
	erpc_set_module_gen(0u);               /* ... and its firmware must be re-proved */
	g_tcpip_inited = false;
	g_ip_mode      = RTL_IP_UNKNOWN;
}

/* ------------------------------------------------------------------ *
 *  core
 * ------------------------------------------------------------------ */
int rtl_link_core_init(void)
{
	if (g_mutex_ready)
		return 0;
	if (tx_mutex_create(&g_link_mutex, "rtl_link", TX_INHERIT) != TX_SUCCESS)
		return -1;
	g_mutex_ready = 1u;
	return 0;
}

int rtl_link_claim(uint32_t timeout_ms)
{
	if (!g_mutex_ready)
		return -1;
	return (tx_mutex_get(&g_link_mutex, (ULONG)timeout_ms) == TX_SUCCESS) ? 0 : -1;
}

void rtl_link_unclaim(void)
{
	if (g_mutex_ready)
		tx_mutex_put(&g_link_mutex);
}

int rtl_link_uart_ref(enum rtl8720_uart which, uint32_t baud)
{
	int rc = 0;

	/* Under the eRPC service thread's mutex: rtl8720_uart_open() closes whatever is
	 * open and resets the shared RX ring, which must not race that thread. */
	erpc_link_lock();
	if (g_uart_refs != 0u) {
		/* Already open: only the very same configuration may join.  Refusing here
		 * is what protects a resident eRPC user from a bridge/flasher re-opening
		 * the peripheral underneath it. */
		if (which != g_uart_which || baud != g_uart_baud)
			rc = -1;
		else
			g_uart_refs++;
		erpc_link_unlock();
		return rc;
	}
	if (rtl8720_uart_open(which, baud) != 0) {
		erpc_link_unlock();
		return -1;
	}
	g_uart_which = which;
	g_uart_baud  = baud;
	g_uart_refs  = 1u;
	g_uart_gen++;                        /* a new "open": stale resident refs are void */
	/* Fresh stream: reset the reader + wire ledger.  Only the AT UART carries eRPC --
	 * the LOG UART is the console bridge, so requests stay refused for it. */
	erpc_link_opened(which == RTL8720_UART_AT);
	erpc_link_unlock();
	return 0;
}

void rtl_link_uart_unref(void)
{
	erpc_link_lock();
	if (g_uart_refs != 0u && --g_uart_refs == 0u) {
		erpc_link_closed();          /* abandon stragglers, reset the reader */
		rtl8720_uart_close();
	}
	erpc_link_unlock();
}

bool rtl_link_uart_busy(void)
{
	return g_uart_refs != 0u;
}

unsigned rtl_link_uart_refs(void)
{
	return g_uart_refs;
}

uint32_t rtl_link_erpc_baud(void)
{
	return g_erpc_baud;
}

int rtl_link_uart_rebaud(uint32_t baud)
{
	int rc = -1;

	/* Under the service thread's mutex, exactly like ref/unref: holding it proves that
	 * thread is touching neither USART1 nor the RX ring while we close and re-open. */
	erpc_link_lock();
	if (g_uart_refs != 0u) {
		rtl8720_uart_close();
		if (rtl8720_uart_open(g_uart_which, baud) == 0) {
			g_uart_baud = baud;
			g_erpc_baud = baud;
			/* Deliberately NOT advancing g_uart_gen: this is the SAME session
			 * continuing at a new rate, and the caller has established there is
			 * no resident holder whose reference could be invalidated.  The
			 * stream and the wire ledger do start over, though -- bytes in
			 * flight across a rate change are meaningless. */
			erpc_link_opened(g_uart_which == RTL8720_UART_AT);
			rc = 0;
		} else {
			/* Left closed on purpose: pretending the old rate is still up would
			 * be a lie, and every send would silently go nowhere.  The caller
			 * recovers (or reports) from here. */
			erpc_link_closed();
		}
	}
	erpc_link_unlock();
	return rc;
}

uint32_t rtl_link_uart_gen(void)
{
	return g_uart_gen;
}

void rtl_link_uart_unref_gen(uint32_t gen)
{
	/* Same section as the count itself, so the compare and the decrement cannot be split
	 * by another ref/unref/force-quiesce.  A mismatch means our open is long gone (someone
	 * force-quiesced, and possibly re-opened): the count we would decrement belongs to
	 * somebody else, so do nothing. */
	erpc_link_lock();
	if (gen == g_uart_gen && g_uart_refs != 0u && --g_uart_refs == 0u) {
		erpc_link_closed();
		rtl8720_uart_close();
	}
	erpc_link_unlock();
}

uint32_t rtl_link_quiesce_gen(void)
{
	return g_quiesce_gen;
}

void rtl_link_force_quiesce(void)
{
	erpc_link_lock();
	g_uart_refs = 0u;                    /* whoever held it does not any more */
	g_uart_gen++;                        /* ... and their recorded generation is now stale */
	g_quiesce_gen++;                     /* only here: "CHIP_EN is about to move" */
	/* This is the recovery path (`wifi reset` and friends).  If the link went dirty
	 * BECAUSE the host was wrong about the module -- its firmware, or the rate it is
	 * listening at -- the recovery must undo those assumptions too, so everything
	 * module-scoped goes back to what a freshly powered module actually is and `wifi rpc
	 * ver` has to earn it again.  Costs one command; a stale belief costs a dead link. */
	rtl_link_forget_module();
	erpc_link_closed();                  /* tokens abandoned; their waiters get -2 */
	rtl8720_uart_close();
	erpc_link_unlock();
}

bool rtl_tcpip_inited(void)          { return g_tcpip_inited; }
void rtl_tcpip_set_inited(bool v)    { g_tcpip_inited = v; }

enum rtl_ipmode rtl_ip_mode(void)         { return g_ip_mode; }
void            rtl_set_ip_mode(enum rtl_ipmode m) { g_ip_mode = m; }

/* ------------------------------------------------------------------ *
 *  shell adapters
 * ------------------------------------------------------------------ */
int rtl_abort_cb(void *ctx)
{
	return cli_cancel_requested((struct cli_instance *)ctx) ? 1 : 0;
}

/* Console + coarse mutex, in that order (released in the reverse).  Returns 0 with
 * both held; on failure nothing is held and the reason has been printed. */
static int link_claim_common(struct cli_instance *sh)
{
	if (cli_console_claim(sh) != 0) {               /* bg-reject / single owner */
		cli_error(sh, "wifi: run in the foreground (not `... &`)\r\n");
		return -1;
	}
	if (rtl_link_claim(RTL_LINK_CLAIM_WAIT_MS) != 0) {
		cli_console_release(sh);
		cli_error(sh, "wifi: the RTL8720 link is busy (another command owns it)\r\n");
		return -1;
	}
	return 0;
}

int rtl_link_begin(struct cli_instance *sh, bool power_on)
{
	if (link_claim_common(sh) != 0)
		return RTL_LINK_ERR;
	if (!rtl8720_powered()) {
		if (!power_on) {
			rtl_link_unclaim();
			cli_console_release(sh);
			return RTL_LINK_OFF;
		}
		cli_print(sh, "wifi: powering on RTL8720DN, waiting ~1.5s for boot...\r\n");
		rtl8720_power(true);
		/* CHIP_EN moved, but NOT through rtl_link_force_quiesce() -- so this path has
		 * to forget the module itself.  It is the easiest of the four reset points to
		 * overlook, and forgetting the baud here is what stops a `link baud 6000000`
		 * session from being followed by a 2 Mbaud open against a 2 Mbaud module. */
		rtl_link_forget_module();
		if (cli_sleep(sh, 1500u)) {            /* cancellable boot wait */
			rtl_link_unclaim();
			cli_console_release(sh);
			return RTL_LINK_ERR;
		}
	}
	if (rtl_link_uart_ref(RTL8720_UART_AT, rtl_link_erpc_baud()) != 0) {
		rtl_link_unclaim();
		cli_console_release(sh);
		cli_error(sh, "wifi: USART1 @%lu did not come ready\r\n",
		          (unsigned long)rtl_link_erpc_baud());
		return RTL_LINK_ERR;
	}
	return RTL_LINK_READY;
}

void rtl_link_end(struct cli_instance *sh)
{
	rtl_link_uart_unref();
	rtl_link_unclaim();
	cli_console_release(sh);
}

int rtl_link_hw_claim(struct cli_instance *sh, bool allow_busy)
{
	if (link_claim_common(sh) != 0)
		return 1;
	if (!allow_busy && rtl_link_uart_busy()) {
		rtl_link_unclaim();
		cli_console_release(sh);
		cli_error(sh, "wifi: the eRPC link is in use -- stop it first "
		          "(this command needs the UART to itself; `net shell stop` if the "
		          "telnet console is armed)\r\n");
		return 1;
	}
	return 0;
}

void rtl_link_hw_release(struct cli_instance *sh)
{
	rtl_link_unclaim();
	cli_console_release(sh);
}
