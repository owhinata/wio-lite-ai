/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/*
 * Wio Lite AI (STM32H725AEI6) -- onboard RTL8720DN (Realtek AmebaD/RTL8721D) UART
 * firmware-download support (issue #19).  This is the STM32-side, on-device flasher
 * that speaks the AmebaD ROM download protocol directly over the RTL8720's LOG UART
 * (= our UART9, PD14/PD15), replacing the host-PC "image tool".
 *
 * MILESTONE 1 (this file, for now): prove UART download-mode ENTRY on this board via
 * a read-only handshake -- it performs the strap+reset sequence, then issues the
 * download protocol's read-word command and checks for the framed reply.  It does NOT
 * erase or write any flash (fully reversible; the RTL8720 mask-ROM download mode is
 * re-enterable, so this cannot brick it).  M2 (protocol body: flashloader upload +
 * erase/write/verify) and M3 (image from microSD) will extend this file.
 *
 * Layering: HAL/CMSIS <- rtl8720.c (basic UART/power) <- rtl8720_flash.c (download
 * protocol) <- cmd_wifi.c (shell).  It never touches the RCC clock tree -- only GPIO
 * reconfig + the #17 rtl8720 UART/power primitives -- so it is XIP-safe.  cli-agnostic:
 * timing via ThreadX, cancellation via an optional abort hook (like app/erpc.c).
 *
 * Entry mechanism (RTL872xD datasheet Table 3-2/3-4, schematic sheet 5/8):
 *   - The boot ROM samples PA[7] (= UART_LOG_TXD) at reset; LOW selects UART download
 *     mode (active-low).  PA[7] is wired to STM32 PD14 (UART9_RX) via WIFI_DEBUG_TXD.
 *   - CHIP_EN = PC3 is the sole reset/enable.
 * So: power the module off (PC3 low) FIRST, drive PD14 low (strap), release reset
 * (PC3 high) so the ROM samples the strap, briefly hold, then return PD14 to UART9_RX
 * and talk the protocol at 115200.
 *
 * Protocol reference (clean-room; comments cite the OSS reference): pvvx
 * SharpRTL872xTool Program.cs.  read-word = 0x31 + addr(u32 LE) -> 0x31 + data(u32 LE)
 * + 0x15 status (Program.cs ReadRegs).  This is the module's first real command in the
 * reference tool, so a valid framed reply proves download-mode entry.
 */
#ifndef APP_RTL8720_FLASH_H
#define APP_RTL8720_FLASH_H

#include <stdint.h>

/* Result of an M1 download-mode probe (all diagnostic; printed by `wifi flashprobe`). */
struct rtl_dl_result {
	int      entered;      /* 1 if a valid read-word reply frame (0x31..0x15) was seen */
	int      slip;         /* 1 if SLIP framing was used for the reply, 0 if raw */
	uint32_t word;         /* read-word value at 0x00082000 (valid only if entered);
	                        * 0x00082021 means the flashloader stub is already resident */
	uint8_t  raw[32];      /* raw bytes received during the probe (for diagnosis) */
	int      raw_len;      /* number of valid bytes in raw[] */
	uint32_t overflows;    /* UART9 RX ring overflows observed (rtl8720_uart_overflows) */
};

/*
 * Enter UART download mode: power-cycle the RTL8720 with PD14 held low across the
 * CHIP_EN rising edge so the ROM samples the download strap, then open UART9 @115200.
 * @hold_us is how long PD14 is held low AFTER CHIP_EN goes high (bounded; kept small
 * to minimise any PD14/PA[7] overlap -- see the .c).  @should_abort (may be NULL) is
 * polled during the waits so Ctrl+C can cancel.
 *
 * Returns 0 with UART9 open (caller must eventually rtl8720_uart_close()), -1 if UART9
 * did not come ready, or -4 if aborted.  GUARANTEE: on every return path (success,
 * failure, abort) PD14 is left as an input / AF -- NEVER as a driven-low output -- so
 * it cannot contend with the RTL8720 driving PA[7] as its LOG-UART TX.
 */
int rtl_dl_enter(uint32_t hold_us, int (*should_abort)(void *ctx), void *abort_ctx);

/*
 * Probe for download-mode entry on the already-open UART9 (call after rtl_dl_enter):
 * send the read-word command for 0x00082000 and wait up to @timeout_ms for the framed
 * reply.  @use_slip selects the framing: 0 = raw (matches the pvvx reference tool),
 * non-zero = SLIP (0xC0-delimited, 0xC0->DB DC / 0xDB->DB DD) -- an EXPLORATORY probe
 * for whether the ROM/Realtek image-tool framing is SLIP (pvvx has no SLIP path).
 * Fills @r (entered?, framing, word, raw bytes, overflow count).  Read-only: it issues
 * no erase/write.  Returns 0 when the probe ran (inspect @r->entered), -4 if aborted.
 */
int rtl_dl_probe(int use_slip, uint32_t timeout_ms,
                 int (*should_abort)(void *ctx), void *abort_ctx,
                 struct rtl_dl_result *r);

#endif /* APP_RTL8720_FLASH_H */
