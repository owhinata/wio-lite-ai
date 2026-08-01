/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    ov2640_regs.h
 * @brief   OV2640 register sequences vendored from ST's component driver.
 *
 * The tables themselves are BSD-3-Clause (see ov2640_regs.c and NOTICE); this
 * header is just the declaration.  Each row is { register, value } written in
 * order over SCCB with the sensor's 8-bit register addressing.  Register 0xFF
 * (RA_DLMT) appears inside the sequence and switches banks, so the rows are
 * ORDER-DEPENDENT and must not be sorted, deduplicated or reordered -- 0xDA in
 * particular is written twice on purpose.
 */
#ifndef OV2640_REGS_H
#define OV2640_REGS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** QVGA 320x240 RGB565 over DVP. */
extern const uint8_t ov2640_qvga_rgb565[][2];
extern const unsigned ov2640_qvga_rgb565_len;

#ifdef __cplusplus
}
#endif

#endif /* OV2640_REGS_H */
