/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    fx_user.h
 * @brief   FileX build configuration for the QSPI NOR filesystem (issue #30).
 *
 * Pulled in by fx_port.h via FX_INCLUDE_USER_DEFINE_FILE (CMakeLists.txt).
 *
 * Deliberately NOT defined:
 *   - FX_SINGLE_THREAD: keep the per-media ThreadX mutex so shell foreground
 *     and background jobs can hit the same media concurrently.
 *   - FX_ENABLE_FAULT_TOLERANT: out of the MVP scope (#27) -- the acceptance
 *     bar is persistence after a clean flush, not power-loss-during-write
 *     integrity (explicit follow-up issue).
 */
#ifndef FX_USER_H
#define FX_USER_H

/* The fs commands use absolute paths only; drop the per-thread local-path
 * machinery (and its TX_THREAD extension slot). */
#define FX_NO_LOCAL_PATH

#endif /* FX_USER_H */
