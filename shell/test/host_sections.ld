/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 *
 * Host-only linker augmentation for the Shell registration smoke test.
 *
 * The target firmware defines `.shell_root_cmds` and its boundary symbols in
 * ldscript/STM32H725AEIx_XIP.ld.  On the host we need the same section and
 * symbols, but the GNU ld default script does not provide auto __start_/__stop_
 * symbols for a section whose name contains a dot, so we supply them here.
 *
 * The `INSERT AFTER` form augments (does not replace) the host default script
 * when passed via -T, mirroring the target placement just after .rodata.
 */
SECTIONS
{
  .shell_root_cmds (READONLY) :
  {
    . = ALIGN(4);
    __cli_root_cmds_start = .;
    KEEP (*(SORT_BY_NAME(.shell_root_cmds.*)))
    KEEP (*(.shell_root_cmds))
    __cli_root_cmds_end = .;
    . = ALIGN(4);
  }
} INSERT AFTER .rodata;
