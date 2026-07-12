/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    cli_vt100.h
 * @brief   VT100 SGR colour escapes + cursor/erase control for the Shell.
 *
 * cli_error/warn/info wrap their text in the SGR colour codes (error=red,
 * warn=yellow, info=green) and reset afterwards.  When CLI_USE_COLOR is 0 every
 * *colour* macro is the empty string, so a monochrome terminal or a log sink
 * sees plain text and the output path stays byte-for-byte identical minus the
 * escapes.
 *
 * The cursor/erase control sequences (issue #9 line editor) are a *separate*
 * group that is NOT gated by CLI_USE_COLOR -- they carry editing semantics, not
 * decoration, so they are always emitted.  Numeric cursor moves (ESC[<n>A/B/C)
 * are built at run time in cli_edit.c; only the parameter-less sequences live
 * here.  Only a handful of well-known sequences -- this is original, not copied.
 */
#ifndef CLI_VT100_H
#define CLI_VT100_H

#include "cli_config.h"   /* CLI_USE_COLOR */

#if CLI_USE_COLOR
#define CLI_VT100_RED     "\x1b[31m"
#define CLI_VT100_YELLOW  "\x1b[33m"
#define CLI_VT100_GREEN   "\x1b[32m"
#define CLI_VT100_RESET   "\x1b[0m"
#else
#define CLI_VT100_RED     ""
#define CLI_VT100_YELLOW  ""
#define CLI_VT100_GREEN   ""
#define CLI_VT100_RESET   ""
#endif

/* Cursor / erase control (always emitted, issue #9). */
#define CLI_VT100_CLR_LINE   "\x1b[2K"          /**< erase the whole current line */
#define CLI_VT100_CLR_EOS    "\x1b[0J"          /**< erase from cursor to end of screen */
#define CLI_VT100_CLR_SCREEN "\x1b[2J\x1b[H"    /**< clear screen + home (Ctrl+l) */
/* Width probe: jump far right, then request a cursor-position report (CPR).  The
 * reply ESC[<rows>;<cols>R reports cols = terminal width (issue #9, §2 wrap). */
#define CLI_VT100_CPR_PROBE  "\x1b[999C\x1b[6n"

#endif /* CLI_VT100_H */
