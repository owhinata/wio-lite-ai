/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    ymodem.h
 * @brief   Clean-room YMODEM-CRC batch sender (svc/ layer).
 *
 * Sends a single file over a byte stream using the YMODEM protocol (CRC-16
 * variant, 1024-byte STX blocks with a 128-byte SOH block 0 carrying the
 * filename + exact size).  The transport and the data source are both injected
 * as vtables, so this core has NO dependency on the shell, ThreadX or HAL -- it
 * sits in the freestanding svc/ layer next to fmt.c/log.c and is unit-tested on
 * the host (shell/test/test_ymodem.c) against mock IO/source.
 *
 * Ported verbatim from ../stm32f746g-disco (its issue #50) for issue #19 M4: the
 * RTL8720DN full-chip flash backup (`wifi flashbackup`) needs to stream megabytes
 * off the board, and this is the shell's designed binary-transfer path.  The shell
 * wires @ref ym_io to cli_read_byte()/cli_write() (see shell/cmds/cmd_xfer.c) and
 * @ref ym_source to the RTL8720 download-protocol flash reader.
 *
 * Clean-room: written from the YMODEM/XMODEM-CRC protocol description, NOT
 * derived from lrzsz (GPL) or any other implementation.  Interoperates with the
 * PC-side lrzsz `rb`/`rz` receivers.
 */
#ifndef YMODEM_H
#define YMODEM_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @ref ym_io getc() sentinels (distinct from any 0..255 byte value). */
#define YM_IO_TIMEOUT (-1)   /**< no byte within the requested timeout */
#define YM_IO_ABORT   (-2)   /**< local abort: Ctrl+C / thread kill */

/**
 * Byte transport, injected by the caller.  Pure: no knowledge of cli_instance.
 */
struct ym_io {
	void *ctx;
	/** Blocking-with-timeout single-byte read.  Returns 0..255 on a byte, or
	 *  YM_IO_TIMEOUT / YM_IO_ABORT.  timeout_ms==0 means "poll once". */
	int  (*getc)(void *ctx, unsigned timeout_ms);
	/** Send @p len bytes, blocking until queued/sent.  0 on success, <0 fatal. */
	int  (*put)(void *ctx, const uint8_t *buf, size_t len);
};

/**
 * Byte source (the file or camera frame), injected by the caller.
 */
struct ym_source {
	void       *ctx;
	const char *name;   /**< basename for block 0 (no '/'), e.g. "frame.raw" */
	uint32_t    size;   /**< exact byte count for block 0 */
	/** Read up to @p want bytes at the current position into @p dst; set
	 *  *@p got to the number produced (0 == EOF).  Returns 0 on success, <0 on
	 *  a read fault (e.g. a concurrent capture changed the frame, or an FX
	 *  error).  Sequential; ymodem advances the position.  A short read
	 *  (*got < want) that is NOT EOF is allowed -- the core re-reads to fill a
	 *  full block, so only *got == 0 ends the stream. */
	int  (*read)(void *ctx, uint8_t *dst, uint32_t want, uint32_t *got);
};

/** ymodem_send() outcome. */
enum ym_result {
	YM_OK = 0,       /**< file transferred and the batch was closed */
	YM_ERR_CANCEL,   /**< receiver sent CAN, or the IO returned ABORT */
	YM_ERR_TIMEOUT,  /**< exhausted retries waiting for a handshake / ACK */
	YM_ERR_SOURCE,   /**< source->read() returned <0 (frame changed / FX error) */
	YM_ERR_IO,       /**< put() returned <0 (transport failure) */
};

/**
 * Drive one full YMODEM batch send of a single file.  Blocking; runs to
 * completion or a terminal error.  On ANY non-success result a CAN sequence is
 * emitted so the PC side (rz/rb) tears down cleanly.  At most src->size bytes are
 * sent regardless of what the source returns (block 0's size is authoritative).
 * Not reentrant (uses a single static block buffer): the caller must serialise
 * transfers -- the shell does so by holding the output lock.
 */
enum ym_result ymodem_send(const struct ym_io *io, const struct ym_source *src);

/**
 * CRC-16/CCITT (XMODEM variant: poly 0x1021, init 0x0000).  Exposed for the
 * host unit test.  ymodem_crc16("123456789", 9) == 0x31C3.
 */
uint16_t ymodem_crc16(const uint8_t *p, size_t n);

#ifdef __cplusplus
}
#endif

#endif /* YMODEM_H */
