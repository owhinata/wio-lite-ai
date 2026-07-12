/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    cmd_crash.c
 * @brief   `crash` shell command: force a fault to test the dump.
 *
 *   crash bus     read a Reserved address  -> precise BusFault (BFAR check)
 *   crash undef   execute an undefined insn -> UsageFault (UNDEFINSTR)
 *   crash div0    integer divide by zero    -> UsageFault (DIVBYZERO)
 *
 * Each subcommand deliberately triggers a fault and never returns: the fault
 * handler (app/fault.c) records the crash to the RAM log and resets the board
 * (or halts under a debugger).  This is a *dangerous* command (spec §12): the
 * whole file compiles in only when CLI_ENABLE_DANGEROUS_CMDS is set, exactly like
 * reboot / devmem.  devmem cannot stand in for it -- its region allow-list
 * rejects the Reserved address before the access happens, so a decisive fault
 * needs this dedicated command.
 *
 * Clean-room design; no third-party code reused.
 */
#include "cli.h"

#if CLI_ENABLE_DANGEROUS_CMDS

#include <stdint.h>

static int cmd_crash_bus(struct cli_instance *sh, int argc, char **argv)
{
	(void)argc;
	(void)argv;
	cli_warn(sh, "crash: read 0x20080000 (Reserved) -> BusFault, resetting\r\n");
	/* 0x20080000 sits in the reserved gap above the 128 KB DTCM (ends
	 * 0x20020000) and below AXI-SRAM (0x24000000), RM0468 §2.3.2 Table 6: a
	 * read there is a precise BusFault with BFAR = the address. */
	volatile uint32_t v = *(volatile uint32_t *)0x20080000u;
	(void)v;
	return 0;                       /* unreachable: the fault handler resets */
}

static int cmd_crash_undef(struct cli_instance *sh, int argc, char **argv)
{
	(void)argc;
	(void)argv;
	cli_warn(sh, "crash: undefined instruction -> UsageFault, resetting\r\n");
	__asm volatile(".inst.n 0xde00");       /* permanently UNDEFINED (UDF #0) */
	return 0;                       /* unreachable */
}

static int cmd_crash_div0(struct cli_instance *sh, int argc, char **argv)
{
	(void)argc;
	(void)argv;
	cli_warn(sh, "crash: divide by zero -> UsageFault, resetting\r\n");
	/* volatile so the divide is actually emitted; traps via CCR.DIV_0_TRP. */
	volatile int a = 1, b = 0;
	volatile int c = a / b;
	(void)c;
	return 0;                       /* unreachable */
}

CLI_SUBCMD_SET_CREATE(crash_subcmds,
	CLI_CMD(bus,   NULL, "read Reserved addr -> BusFault",      cmd_crash_bus),
	CLI_CMD(undef, NULL, "undefined instr -> UsageFault",       cmd_crash_undef),
	CLI_CMD(div0,  NULL, "divide by zero -> UsageFault",        cmd_crash_div0),
	CLI_SUBCMD_SET_END);

CLI_CMD_REGISTER(crash, crash_subcmds,
                 "force a fault to test the crash dump (RESETS the board)",
                 NULL, 1, 0);

#endif /* CLI_ENABLE_DANGEROUS_CMDS */
