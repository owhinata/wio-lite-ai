/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    cmd_devmem.c
 * @brief   `devmem` built-in shell command (issue #14): peek / poke / dump.
 *
 * A developer/debug memory-access command, registered only in the `shell`
 * executable (never the host test harness, like cmd_system.c / cmd_thread.c).
 * It reads board state through the standard buffered output API and touches only
 * the shell instance passed to it, so it stays reentrant across instances (§10).
 *
 *   devmem peek <addr> [8|16|32]        read  one 8/16/32-bit word (default 32)
 *   devmem poke <addr> <val> [8|16|32]  write one word, then read it back
 *   devmem dump <addr> [len]            canonical hex+ASCII over [addr, addr+len)
 *
 * devmem is a *dangerous* command (spec §12): the whole file compiles in only
 * when CLI_ENABLE_DANGEROUS_CMDS is set (default ON for the demo, forwarded from
 * the CMake option).  With it off, the handlers and their registration vanish, so
 * devmem leaves .shell_root_cmds and disappears from help and completion.
 *
 * Address-range gate (spec §12): rather than a single [min,max], accesses are
 * checked against a compile-time region allow-list (devmem_map[]).  An access
 * must lie wholly inside one region, be permitted for its direction, and use a
 * width that region allows; otherwise it is rejected with an error instead of
 * being attempted.  This keeps a typo off the Reserved holes in the H725 memory
 * map (RM0468 §2.3.1) -- a read/write there faults to the weak Default_Handler,
 * whose infinite loop would hang the shell.  The default map permits the real
 * on-chip RAM, the internal + external XIP flash (read-only) and the PPB; the
 * general peripheral windows are omitted until a fault handler lands (Phase 3b).
 * The PPB is word-access only (many registers fault or misbehave on sub-word
 * access), so dump -- which is byte-granular -- is allowed on RAM/Flash only.
 * Production hardening removes devmem wholesale via CLI_ENABLE_DANGEROUS_CMDS=0.
 *
 * Clean-room design; no third-party code reused.
 */
#include "cli.h"

#include <stdint.h>
#include <string.h>

#if BSP_ENABLE_PSRAM
#include "psram.h"           /* psram_ready() gate for the 0x90000000 window (#3) */
#endif

#if CLI_ENABLE_DANGEROUS_CMDS

/* Allowed access widths, as a bitmask carried per region. */
#define W8   0x1u
#define W16  0x2u
#define W32  0x4u
#define WALL (W8 | W16 | W32)

struct devmem_region {
	uint32_t    base;       /* region start */
	uint32_t    size;       /* region length in bytes */
	uint8_t     read;       /* 1 = peek/dump allowed */
	uint8_t     write;      /* 1 = poke allowed */
	uint8_t     widths;     /* bitmask of permitted access widths (W8/W16/W32) */
	uint8_t     psram;      /* 1 = gate on psram_ready() (unbacked window hangs) */
	const char *name;       /* shown in range/width error messages */
};

/*
 * Default region allow-list for the Wio Lite AI (STM32H725) memory map (RM0468
 * §2.3.1).  Only real on-chip RAM, the internal + external XIP flash (read-only),
 * and the PPB are listed; reserved holes and the general peripheral bus windows
 * are absent, so a typo there is rejected instead of faulting.  This matters
 * because this build has no fault handler yet (Phase 3b), so a stray access would
 * spin in the weak Default_Handler and hang the shell.  The PPB is word-only.
 * (Peripheral windows can be added once the fault handler lands.)
 */
static const struct devmem_region devmem_map[] = {
	{ 0x00000000u, 0x00010000u, 1, 1, WALL, 0, "ITCM"      }, /* 64 KB               */
	{ 0x08000000u, 0x00080000u, 1, 0, WALL, 0, "IntFlash"  }, /* 512 KB int, RO (boot) */
	{ 0x20000000u, 0x00020000u, 1, 1, WALL, 0, "DTCM"      }, /* 128 KB              */
	{ 0x24000000u, 0x00050000u, 1, 1, WALL, 0, "AXI-SRAM"  }, /* 320 KB (D1)         */
	{ 0x70000000u, 0x00800000u, 1, 0, WALL, 0, "XIP-Flash" }, /* ext OCTOSPI2 XIP, RO */
#if BSP_ENABLE_PSRAM
	{ 0x90000000u, 0x00800000u, 1, 1, WALL, 1, "PSRAM"     }, /* ext OCTOSPI1 APS6408 (#3); gated on psram_ready() */
#endif
	{ 0xE0000000u, 0x00100000u, 1, 1, W32,  0, "PPB"       }, /* SCB/NVIC/SysTick/DWT */
};

/*
 * Parse a 32-bit unsigned value: 0x/0X-prefixed hex or plain decimal.  Strict --
 * an empty string, an invalid digit, trailing garbage, or 32-bit overflow all
 * fail.  No newlib strtoul dependency (this firmware ships its own parsers).
 */
static int parse_u32(const char *s, uint32_t *out)
{
	uint32_t base = 10;
	uint32_t val = 0;
	const char *p = s;

	if (p == NULL || *p == '\0')
		return -1;
	if (p[0] == '0' && (p[1] == 'x' || p[1] == 'X')) {
		base = 16;
		p += 2;
		if (*p == '\0')                 /* bare "0x" */
			return -1;
	}
	for (; *p != '\0'; p++) {
		uint32_t digit;
		char c = *p;

		if (c >= '0' && c <= '9')
			digit = (uint32_t)(c - '0');
		else if (base == 16 && c >= 'a' && c <= 'f')
			digit = (uint32_t)(c - 'a' + 10);
		else if (base == 16 && c >= 'A' && c <= 'F')
			digit = (uint32_t)(c - 'A' + 10);
		else
			return -1;              /* invalid / trailing character */

		if (val > (0xFFFFFFFFu - digit) / base)
			return -1;              /* would overflow 32-bit */
		val = val * base + digit;
	}
	*out = val;
	return 0;
}

/* "8"/"16"/"32" -> access width in bytes (1/2/4). */
static int parse_width(const char *s, uint32_t *bytes)
{
	if (strcmp(s, "8") == 0)  { *bytes = 1; return 0; }
	if (strcmp(s, "16") == 0) { *bytes = 2; return 0; }
	if (strcmp(s, "32") == 0) { *bytes = 4; return 0; }
	return -1;
}

/*
 * Gate one access of `span` bytes starting at `addr`, made up of `elem_bytes`
 * (1/2/4) wide elements, against the region table.  The access must lie wholly
 * inside a single region (64-bit arithmetic avoids wrap at the 4 GiB top), be
 * permitted for its direction, and use a width that region allows.  Returns 0 if
 * allowed; otherwise prints the reason and returns -1.
 */
static int devmem_check(struct cli_instance *sh, uint32_t addr, uint32_t span,
                        uint32_t elem_bytes, int want_write)
{
	uint8_t want_w = (elem_bytes == 1) ? W8 : (elem_bytes == 2) ? W16 : W32;
	uint64_t alo = addr;
	uint64_t ahi = (uint64_t)addr + span;           /* exclusive end */

	/* span > 0 is assumed: peek/poke pass width (>=1) and dump returns early on
	 * len == 0, so the exclusive-end test never degenerates to a zero-width span. */
	size_t i;

	for (i = 0; i < sizeof devmem_map / sizeof devmem_map[0]; i++) {
		const struct devmem_region *r = &devmem_map[i];
		uint64_t rlo = r->base;
		uint64_t rhi = (uint64_t)r->base + r->size; /* exclusive end */

		if (alo < rlo || ahi > rhi)
			continue;                       /* not wholly within this region */

		if (want_write ? !r->write : !r->read) {
			cli_error(sh, "devmem: %s not allowed in %s\r\n",
			          want_write ? "write" : "read", r->name);
			return -1;
		}
		if (!(r->widths & want_w)) {
			cli_error(sh, "devmem: %lu-bit access not allowed in %s\r\n",
			          (unsigned long)(elem_bytes * 8u), r->name);
			return -1;
		}
#if BSP_ENABLE_PSRAM
		/* The PSRAM window is memory-mapped only after psram_hw_init() succeeds.
		 * Before that (or after a diagnostic wedged the device) a CPU access at
		 * 0x90000000 is an unbounded DQS-gated OCTOSPI1 transaction that stalls
		 * the AXI bus until the IWDG resets -- devmem must not attempt it. */
		if (r->psram && !psram_ready()) {
			cli_error(sh, "devmem: PSRAM not ready; use `psram info`/`psram probe`\r\n");
			return -1;
		}
#endif
		return 0;
	}
	cli_error(sh, "devmem: 0x%08lx (%lu bytes) not in an allowed region\r\n",
	          (unsigned long)addr, (unsigned long)span);
	return -1;
}

/* Read `width` bytes at `addr` (already gated and aligned). */
static uint32_t mem_read(uint32_t addr, uint32_t width)
{
	uintptr_t a = (uintptr_t)addr;

	switch (width) {
	case 1:  return *(const volatile uint8_t  *)a;
	case 2:  return *(const volatile uint16_t *)a;
	default: return *(const volatile uint32_t *)a;
	}
}

/* Print "0x<addr>: 0x<value>" with the value zero-padded to the access width. */
static void print_cell(struct cli_instance *sh, uint32_t addr, uint32_t width,
                       uint32_t val)
{
	switch (width) {
	case 1:
		cli_print(sh, "0x%08lx: 0x%02lx\r\n",
		          (unsigned long)addr, (unsigned long)val);
		break;
	case 2:
		cli_print(sh, "0x%08lx: 0x%04lx\r\n",
		          (unsigned long)addr, (unsigned long)val);
		break;
	default:
		cli_print(sh, "0x%08lx: 0x%08lx\r\n",
		          (unsigned long)addr, (unsigned long)val);
		break;
	}
}

static int parse_addr_width(struct cli_instance *sh, const char *addr_s,
                            const char *width_s, uint32_t *addr, uint32_t *width)
{
	*width = 4;                                      /* default 32-bit */
	if (parse_u32(addr_s, addr) != 0) {
		cli_error(sh, "devmem: bad address '%s'\r\n", addr_s);
		return -1;
	}
	if (width_s != NULL && parse_width(width_s, width) != 0) {
		cli_error(sh, "devmem: bad width '%s' (use 8/16/32)\r\n", width_s);
		return -1;
	}
	if (*addr % *width != 0) {
		cli_error(sh, "devmem: 0x%08lx not %lu-bit aligned\r\n",
		          (unsigned long)*addr, (unsigned long)(*width * 8u));
		return -1;
	}
	return 0;
}

/*
 * PSRAM accesses share the single OCTOSPI1 controller with the `psram`/`membench`
 * commands, so a raw devmem poke/peek/dump at 0x90000000 must hold the same guard
 * or a concurrent `psram` command (run via `cmd &`) can ABORT / leave mmap /
 * change DCR mid-access and corrupt the data or stall the AXI bus.  Returns 1 if
 * the guard was taken (release with psram_release()), 0 if the access does not
 * touch PSRAM, -1 if PSRAM but the guard is busy (reject the command).
 */
#if BSP_ENABLE_PSRAM
static int devmem_psram_lock(struct cli_instance *sh, uint32_t addr, uint32_t span)
{
	if (addr < PSRAM_BASE_ADDR ||
	    (uint64_t)addr + span > (uint64_t)PSRAM_BASE_ADDR + PSRAM_SIZE_BYTES)
		return 0;                               /* not the PSRAM window */
	if (!psram_acquire()) {
		cli_error(sh, "devmem: PSRAM busy (a psram command holds OCTOSPI1)\r\n");
		return -1;
	}
	return 1;
}
#define devmem_psram_unlock()  psram_release()
#else
#define devmem_psram_lock(sh, addr, span)  0
#define devmem_psram_unlock()              ((void)0)
#endif

static int cmd_devmem_peek(struct cli_instance *sh, int argc, char **argv)
{
	uint32_t addr, width;
	int plk;

	if (parse_addr_width(sh, argv[1], argc >= 3 ? argv[2] : NULL,
	                     &addr, &width) != 0)
		return 1;
	if (devmem_check(sh, addr, width, width, 0) != 0)
		return 1;

	plk = devmem_psram_lock(sh, addr, width);
	if (plk < 0)
		return 1;
	print_cell(sh, addr, width, mem_read(addr, width));
	if (plk > 0)
		devmem_psram_unlock();
	return 0;
}

static int cmd_devmem_poke(struct cli_instance *sh, int argc, char **argv)
{
	uint32_t addr, width, value;
	uintptr_t a;
	int plk;

	if (parse_u32(argv[2], &value) != 0) {
		cli_error(sh, "devmem: bad value '%s'\r\n", argv[2]);
		return 1;
	}
	if (parse_addr_width(sh, argv[1], argc >= 4 ? argv[3] : NULL,
	                     &addr, &width) != 0)
		return 1;
	if (width < 4 && value > ((1u << (8u * width)) - 1u)) {
		cli_error(sh, "devmem: value 0x%lx does not fit %lu-bit\r\n",
		          (unsigned long)value, (unsigned long)(width * 8u));
		return 1;
	}
	if (devmem_check(sh, addr, width, width, 1) != 0)
		return 1;

	plk = devmem_psram_lock(sh, addr, width);
	if (plk < 0)
		return 1;
	a = (uintptr_t)addr;
	switch (width) {
	case 1:  *(volatile uint8_t  *)a = (uint8_t)value;  break;
	case 2:  *(volatile uint16_t *)a = (uint16_t)value; break;
	default: *(volatile uint32_t *)a = value;           break;
	}

	print_cell(sh, addr, width, mem_read(addr, width));     /* read-back */
	if (plk > 0)
		devmem_psram_unlock();
	return 0;
}

static int cmd_devmem_dump(struct cli_instance *sh, int argc, char **argv)
{
	uint32_t addr, len = 64;                        /* default 64 bytes */

	if (parse_u32(argv[1], &addr) != 0) {
		cli_error(sh, "devmem: bad address '%s'\r\n", argv[1]);
		return 1;
	}
	if (argc >= 3 && parse_u32(argv[2], &len) != 0) {
		cli_error(sh, "devmem: bad length '%s'\r\n", argv[2]);
		return 1;
	}
	if (len == 0)
		return 0;                               /* nothing to dump */
	if (len > CLI_DEVMEM_DUMP_MAX_LEN) {
		cli_error(sh, "devmem: length %lu exceeds max %u\r\n",
		          (unsigned long)len, (unsigned)CLI_DEVMEM_DUMP_MAX_LEN);
		return 1;
	}
	/* dump is byte-granular, so it needs an 8-bit-capable region (RAM/Flash). */
	if (devmem_check(sh, addr, len, 1, 0) != 0)
		return 1;

	{
		int plk = devmem_psram_lock(sh, addr, len);
		int rc;

		if (plk < 0)
			return 1;
		rc = cli_hexdump_base(sh, (const void *)(uintptr_t)addr, len, addr);
		if (plk > 0)
			devmem_psram_unlock();
		return rc == 0 ? 0 : 1;
	}
}

CLI_SUBCMD_SET_CREATE(devmem_subcmds,
	CLI_CMD_ARG(peek, NULL, "read  <addr> [8|16|32]",       cmd_devmem_peek, 2, 1),
	CLI_CMD_ARG(poke, NULL, "write <addr> <val> [8|16|32]", cmd_devmem_poke, 3, 1),
	CLI_CMD_ARG(dump, NULL, "hexdump <addr> [len]",         cmd_devmem_dump, 2, 1),
	CLI_SUBCMD_SET_END);

CLI_CMD_REGISTER(devmem, devmem_subcmds,
                 "read/write memory (peek/poke/dump)", NULL, 1, 0);

#endif /* CLI_ENABLE_DANGEROUS_CMDS */
