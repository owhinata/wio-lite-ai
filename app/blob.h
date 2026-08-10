/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    blob.h
 * @brief   Read-only asset region on the external NOR (issue #10 / issue #9 P2b).
 *
 * Six fixed 512 KB slots above the `kv` partition, each holding one file that
 * arrived from the PC over YMODEM.  This exists because issue #9 decided that the
 * TFLM model is supplied from the external NOR: a 189 KB `.tflite` does not fit in
 * the internal flash's spare ~115 KB, and linking it in would spend an internal
 * flash erase cycle (~10k lifetime) on every model change.  Here a model is data,
 * and replacing one costs nothing but a transfer.
 *
 * THIS IS THE THIRD SIBLING ON THE SAME DEVICE, not a layer under or over them:
 * port/nor is the device, app/kv.c is the configuration store, and this is the
 * asset store.  It calls the nor_* primitives directly and knows nothing of
 * FlashDB or FAL -- the one thing it borrows from that side is fdb_calc_crc32().
 *
 *   0x000000 .. 0x100000   kv partition (FlashDB, issue #37)
 *   0x100000 .. 0x400000   THIS: 6 slots x 512 KB
 *   0x400000 .. 0x1000000  unallocated (12 MB, still with no FAL entry)
 *
 * Slot layout.  The first 4 KB sector holds the header and the remaining 508 KB is
 * the payload.  Giving the header a whole sector costs 0.8% of a slot and buys two
 * things: the payload stays 4 KB-aligned, and the header can be erased on its own
 * (one ~45 ms sector erase) to retire a blob without erasing 512 KB.
 *
 * WHY 512 KB x 6 AND NOT 256 KB x 8 (issue #55).  MLPerf Tiny v1.4 needs five models
 * on the device at once, and four of them -- IC (260,648 B), IC02 (512,024), VWW
 * (333,288) and AD (276,976) -- do not fit the old 258,048 B payload.  512 KB is the
 * smallest power-of-two slot that swallows the largest of them, and six of those hold
 * the five benchmarks plus one non-MLPerf model (BlazeFace, issue #9, lives there).
 * The region had to grow past 0x300000 to fit them, which is why the `nor test`
 * default address moved with it (shell/cmds/cmd_nor.c) -- that is the one constant
 * outside this file that is tied to the region END rather than to a slot.
 *
 * 🔴 THE GEOMETRY CHANGE IS WHY BLOB_FMT_VER IS NOW 2.  The new slot bases coincide
 * with the old EVEN slots' header sectors, so a version-1 header would still decode
 * and still verify -- under a different slot number.  See app/blob.c.
 *
 * THE HEADER IS WRITTEN LAST, AND IN TWO STEPS.  ymodem.h is explicit that YM_OK
 * requires reaching the sender's closing block and that anything else means the
 * caller must DISCARD what the sink accumulated -- so the header is only written
 * once ymodem_recv() has returned YM_OK, and an interrupted transfer leaves the
 * header sector erased and the slot reading `empty`.  This is the same trick the
 * DFU bootloader uses when it programs the app's first 32 bytes last so that an
 * interrupted download always falls back into DFU.
 *
 * The two steps matter as much as the ordering: the body (version, name, length,
 * payload CRC) is programmed, read back and compared, and only then is the magic
 * programmed by a SEPARATE call.  Written in one go, a page program that failed or
 * lost power part-way could land the magic and leave the rest half-formed -- a slot
 * that looks valid and is not.  Split, every outcome is either "no magic, so not
 * valid" or "magic, so the body was read back and matched".  Both programs touch
 * bytes that are still erased in the same page, which is exactly the partial-page
 * programming `nor test` accepts on this device (W25Q128JV sec 8.2.13).
 *
 * THE PAYLOAD CRC COMES FROM THE RECEIVED STREAM, NOT FROM A READ-BACK.  Deriving
 * it from the flash would make `blob verify` compare the flash against itself and
 * pass forever, however badly the write went.  Computed over the bytes the PC sent,
 * the stored CRC is a claim about the transfer that a later read-back can falsify.
 *
 * LOCKING -- deliberately NOT held across a transfer.  A `blob write` holds the
 * device only inside each nor_* primitive (they take the recursive device mutex
 * themselves), never across the whole YMODEM session.  Holding it across would stop
 * the configuration store and boot-time config application for as long as a transfer
 * takes while buying nothing: the two ranges are disjoint (FAL range-checks within a
 * partition and the only partition is `kv` at [0, 1 MB), see port/flashdb/fal_cfg.h),
 * and so is the device STATE -- port/nor never writes a status register, never uses
 * erase suspend/resume and never enters memory-mapped mode, and each WREN ->
 * program/erase -> WIP wait completes inside one lock.  There is no shared state
 * left for the other user to disturb between two of our operations.
 *
 * The honest cost in the other direction: a KV operation running while a transfer is
 * in flight delays a block ACK by however long it holds the device.  An ordinary
 * `kv set` is one ~45 ms sector erase, but `kv format` erases a megabyte -- ~2.4 s
 * typical and tens of seconds at the driver's timeout budget.  That still fits
 * inside lrzsz's patience, but "do not run `kv format` during a transfer" is the
 * real rule.
 *
 * What IS serialised is blob-mutating operations against each other, with the busy
 * flag below: YMODEM only runs on the USB CDC console, but a telnet session can
 * still type `blob erase` into the middle of a `blob write`.
 *
 * THREAD CONTEXT ONLY, after the scheduler is running: erases and programs sleep.
 */
#ifndef APP_BLOB_H
#define APP_BLOB_H

#include <stdint.h>

#include "nor_flash.h"   /* NOR_SECTOR_SIZE -- the header reservation is one sector */
#include "ymodem.h"      /* struct ym_sink */

#ifdef __cplusplus
extern "C" {
#endif

/* ---- geometry ------------------------------------------------------------ */

#define BLOB_SLOT_COUNT    6u
#define BLOB_REGION_BASE   0x00100000u
#define BLOB_SLOT_SIZE     0x00080000u                 /* 512 KB               */
#define BLOB_HDR_SIZE      NOR_SECTOR_SIZE             /* 4 KB, header only    */
#define BLOB_PAYLOAD_MAX   (BLOB_SLOT_SIZE - BLOB_HDR_SIZE)   /* 520,192 B     */
#define BLOB_REGION_SIZE   (BLOB_SLOT_COUNT * BLOB_SLOT_SIZE) /* 3 MB          */
#define BLOB_REGION_END    (BLOB_REGION_BASE + BLOB_REGION_SIZE)  /* 0x400000  */

/* Longest stored name, excluding the terminator.  A YMODEM block 0 can carry more;
 * a longer name is truncated rather than refused, because the name is a label for
 * the operator and refusing would fail a transfer for a cosmetic reason -- after
 * the slot has already been erased. */
#define BLOB_NAME_MAX      63u

/* ---- returns ------------------------------------------------------------- */

#define BLOB_OK            0
#define BLOB_ERR_PARAM    -1   /* bad slot index / offset / length             */
#define BLOB_ERR_IO       -2   /* the NOR refused a read, program or erase     */
#define BLOB_ERR_STATE    -3   /* device down, or the sink was not armed       */
#define BLOB_ERR_BUSY     -4   /* another blob-mutating operation is running   */
#define BLOB_ERR_EMPTY    -5   /* no valid blob in that slot                   */
#define BLOB_ERR_CRC      -6   /* read-back does not match the stored CRC32    */
#define BLOB_ERR_SHORT    -7   /* fewer bytes arrived than the sender declared */

/** What the header sector says about a slot. */
enum blob_state {
	BLOB_EMPTY = 0,   /**< header sector is entirely erased                    */
	BLOB_VALID,       /**< magic + a self-consistent body                      */
	BLOB_INCOMPLETE,  /**< body written, magic not: a commit that was cut short */
	BLOB_INVALID,     /**< magic present but the body does not decode          */
};

struct blob_info {
	enum blob_state state;
	char            name[BLOB_NAME_MAX + 1u];  /**< printable ASCII, NUL-terminated */
	uint32_t        length;                    /**< payload bytes                */
	uint32_t        crc32;                     /**< CRC-32/ISO-HDLC of the payload */
};

/* ---- read side (no exclusion: see the locking note above) ----------------- */

/** Device offset of a slot's header sector / of its payload.  Slot must be valid. */
uint32_t blob_slot_addr(unsigned slot);
uint32_t blob_payload_addr(unsigned slot);

/**
 * Decode @p slot's header into @p out.  Returns BLOB_OK whenever the header sector
 * could be READ -- @p out->state then says what was there, including "nothing".
 * Only a device fault gives BLOB_ERR_IO, so a listing can tell "slot is empty"
 * apart from "the flash did not answer".
 */
int blob_stat(unsigned slot, struct blob_info *out);

/**
 * Read @p len payload bytes from @p off within @p slot.  Bounded by the payload
 * AREA, not by the stored length: a debugging hexdump may legitimately look past
 * the end of a blob.  A consumer that wants the blob itself takes the length from
 * blob_stat() first.
 *
 * No yield: 189 KB is ~17 ms on the wire (about 90 ns/byte), which is a reasonable
 * amount of time for the calling thread to keep, and every higher-priority thread
 * (the IWDG petter included) still preempts it.  If a future consumer streams
 * megabytes this is where chunking with a yield belongs -- see nor_flash.h.
 */
int blob_read(unsigned slot, uint32_t off, void *buf, uint32_t len);

/**
 * Re-read the payload and compare it against the stored CRC32.  BLOB_OK on a match,
 * BLOB_ERR_CRC on a mismatch (with the computed value in *@p crc_out when non-NULL),
 * BLOB_ERR_EMPTY when the slot holds no valid blob.
 */
int blob_verify(unsigned slot, uint32_t *crc_out);

/* ---- mutating side (all of it under blob_busy_acquire) -------------------- */

/**
 * Serialise blob-mutating operations against each other.  A plain test-and-set under
 * a brief PRIMASK critical section -- interrupt-safe, thread-agnostic and needing no
 * initialisation, the same idiom as the NN session guard (port/nn/nn.c).  Returns
 * BLOB_OK when acquired, BLOB_ERR_BUSY otherwise.
 */
int  blob_busy_acquire(void);
void blob_busy_release(void);

/** Erase a whole slot, header included.  ~0.6 s (four 64 KB block erases). */
int  blob_erase(unsigned slot);

/**
 * Arm the YMODEM sink for @p slot: erase the slot, then accept a transfer into it.
 *
 * THE ERASE HAPPENS HERE, BEFORE THE PROTOCOL STARTS, and that is the point.  Done
 * inside the sink's begin() it would sit between block 0 and its ACK, adding a
 * second of silence exactly where the sender is counting retries.  Nobody is waiting
 * yet at this point, so a slow erase costs nothing.
 *
 * The price is that a failed transfer does not leave the old contents behind.  That
 * is what the other seven slots are for: write to a different one to keep the blob
 * you have.
 */
int  blob_recv_arm(unsigned slot);

/** The sink to hand to xfer_recv_sink_locked(), valid between arm and commit. */
const struct ym_sink *blob_sink(void);

/**
 * Commit the armed slot: write the header body, read it back, then write the magic.
 * Call ONLY after ymodem_recv() reported YM_OK (see the file header).  Fills @p out
 * with what was committed when non-NULL.
 *
 * BLOB_ERR_SHORT if fewer bytes arrived than block 0 declared: the batch closed
 * cleanly but the file was truncated on the way, and a blob whose length we are not
 * sure of must not be given a magic.
 */
int  blob_recv_commit(struct blob_info *out);

/** Drop the armed state without writing a header (the failure path). */
void blob_recv_disarm(void);

/** Bytes accepted by the sink so far -- for reporting a failure position. */
uint32_t blob_recv_pos(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_BLOB_H */
