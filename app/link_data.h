/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/*
 * Wio Lite AI (STM32H725AEI6) -- the link's DATA channel (issue #23 U1).
 *
 * The RTL8720DN link carries three kinds of frame on one UART, all demultiplexed by
 * the single link service thread in app/erpc.c:
 *
 *   eRPC  u16 size | u16 crc | body            request/reply, `wifi` / `net` / telnet
 *   CTRL  u16 0xFFFF | u16 len | u16 crc | body  link management (issue #23 U0-3)
 *   DATA  u16 0xFFFE | u16 len | u16 crc | body  THIS FILE
 *
 * DATA is what the other two are not: unsolicited, bidirectional and unacknowledged.
 * It exists to carry raw Ethernet frames once the module is turned into an L2 bridge
 * (issue #23 U2) so the host's own TCP/IP stack can run on top (U3).  In U1 the far end
 * is a sink/source bench instead, which is why this file knows nothing about Ethernet.
 *
 *   body = u8 chan | u8 flags | payload[len - 2]
 *
 * Why 0xFFFE is unambiguous is the same argument as CTRL's 0xFFFF, and it is about the
 * SENDER, not the receiver: an eRPC frame's leading u16 is its message size, the host
 * can only produce 8 + ERPC_REQ_MAX and the module only its 4096-byte MessageBuffer, so
 * neither end can emit 65534 there.  A desynchronised stream can still land on it by
 * chance, so the length is bounded before it is believed and the CRC is checked.
 *
 * The two-byte body header is not free, and it is not decoration: with the buffer
 * 8-byte aligned, an Ethernet frame starting at +2 puts its IP header at +16, i.e.
 * 4-byte aligned, which is what the host stack wants.
 *
 * ---- ownership ---------------------------------------------------------------
 *
 * Frames are staged in static pools here, never on a caller's stack:
 *   TX  link_data_send() copies the payload into a pool buffer, computes the CRC ON
 *       THE CALLER'S THREAD (a 1500-byte CRC is ~11 us that the service thread does
 *       not have to spend) and queues it.  The service thread writes it out.
 *   RX  the service thread claims a pool buffer, the frame reader fills it straight
 *       from the UART ring (no copy through a scratch buffer), and the completed frame
 *       is handed to the registered callback.
 *
 * link_data_send() deliberately does NOT take app/erpc.c's mutex.  That is what keeps
 * the lock order acyclic once a real IP stack is attached: the receive callback runs
 * with no link lock held and may take the stack's lock, while a transmit from inside
 * the stack takes only the short interrupt-masked region in this file.
 */
#ifndef APP_LINK_DATA_H
#define APP_LINK_DATA_H

#include <stdint.h>

/* Largest payload one DATA frame may carry.  1536 leaves room above the 1514-byte
 * (1518 with a VLAN tag) Ethernet frame that U2 will put in here. */
#define LINK_DATA_PAYLOAD_MAX 1536u

/* body = chan + flags + payload */
#define LINK_DATA_HDR         2u
#define LINK_DATA_BODY_MAX    (LINK_DATA_HDR + LINK_DATA_PAYLOAD_MAX)

/* Leading u16 of a DATA frame, and the total frame overhead (magic, length, CRC). */
#define LINK_DATA_MAGIC       0xFFFEu
#define LINK_DATA_FRAME_HDR   6u

/* Logical channels multiplexed inside DATA.  Only one consumer is attached at a time;
 * the channel byte says what a frame is so a bench can be told from real traffic while
 * both exist (U1 uses only BENCH, U2+ only ETH). */
enum {
	LINK_DATA_CHAN_ETH   = 0,        /* raw Ethernet frame (issue #23 U2 onwards) */
	LINK_DATA_CHAN_BENCH = 1         /* U1 sink/source measurement traffic */
};

/*
 * Pool depths, 1544 B per buffer of AXI-SRAM BSS.
 *
 * RECEIVE is a staging depth only: the service thread dispatches a completed frame to the
 * consumer, which returns having already copied it, so buffers come straight back.  8 is
 * plenty.
 *
 * TRANSMIT is different, and issue #23 U4 is why it grew from 8 to 16.  The producer above
 * it is now a real TCP/IP stack, which can hand down a burst -- several sockets' queued
 * segments plus a whole ARP waiting list flushed the moment an address resolves -- before
 * the service thread has written a single one of them out.  A full pool here is a silent
 * drop, which is exactly right for Ethernet and exactly wrong for a stream: TCP pays for
 * it with a retransmit timeout measured in hundreds of milliseconds.  The bound this
 * number has to satisfy is written down in app/nx_net.h ("the transmit budget") and
 * checked by a _Static_assert in app/nx_echo.c.
 */
#define LINK_DATA_RX_BUFS     8u
#define LINK_DATA_TX_BUFS     16u

struct link_data_stats {
	uint32_t tx_frames;
	uint32_t tx_bytes;               /* payload bytes, framing excluded */
	uint32_t tx_drops;               /* send() with no free buffer -- Ethernet-legal */
	uint32_t rx_frames;
	uint32_t rx_bytes;
	uint32_t rx_drops;               /* no free buffer: the body was drained instead */
	uint32_t rx_crc_err;             /* body arrived but the CRC did not match */
	uint32_t rx_oversize;            /* length outside 2..LINK_DATA_BODY_MAX */
	uint32_t rx_no_consumer;         /* frame complete but nobody is attached */
	uint32_t tx_queued;              /* frames waiting for the service thread now */
	uint32_t rx_inuse;               /* RX buffers currently out of the pool */
};

/*
 * Attach the one consumer of received DATA frames.  Returns 0, or -1 if another
 * consumer is already attached (the channel has exactly one owner, like the console).
 *
 * @rx runs ON THE LINK SERVICE THREAD (priority 10) with NO link lock held, once per
 * received frame.  Its return value transfers ownership of the buffer:
 *      0  done with it -- freed as soon as it returns
 *      1  kept -- the consumer must call link_data_rx_free(@p) later
 * CONTRACT: return quickly.  While it runs, nothing is draining the UART's 16 kB RX
 * ring, so a slow consumer shows up as ring drops rather than as latency.
 */
int  link_data_attach(int (*rx)(void *ctx, uint8_t chan, uint8_t *p, uint16_t n),
                      void *ctx);

/*
 * Detach the consumer.
 *
 * ORDERING RULE (this is load-bearing -- see app/erpc.c's send step): while a consumer
 * is attached, the service thread stops flushing stale RX bytes before a request,
 * because those "stale" bytes may be the middle of a DATA frame.  Detaching restores
 * the flush, so it must not happen while the far end may still be sending: stop the
 * module's source and confirm BOTH ends are quiet first (LINK_DATA_CFG(off) -> its
 * acknowledgement means the module has drained its own DATA queue -> LINK_DATA_STATS
 * shows nothing in flight -> link_data_tx_pending() == 0 and the reader is at a frame
 * boundary).  The exception is a teardown that takes the module down with it
 * (`wifi reset`, force-quiesce, UART close), where nothing survives to be desynchronised.
 */
void link_data_detach(void);

/* Is a consumer attached (i.e. is the DATA channel live)? */
int  link_data_attached(void);

/*
 * Queue one DATA frame for transmission.  Copies @p (@n bytes, <= LINK_DATA_PAYLOAD_MAX)
 * and wakes the link service thread.  Returns 0, or -1 if the link is down, the
 * arguments are bad, or no pool buffer is free -- the last is a DROP, counted in
 * @tx_drops, which is the correct behaviour for a channel carrying Ethernet.
 */
int  link_data_send(uint8_t chan, const uint8_t *p, uint16_t n);

/* Return a buffer the receive callback kept (returned 1).  Ignores NULL and any
 * pointer that is not a live RX payload. */
void link_data_rx_free(uint8_t *p);

/* Frames queued for transmit but not yet written to the UART. */
int  link_data_tx_pending(void);

/* Snapshot of the counters.  Monotonic since the last link_data_reset(). */
void link_data_stats(struct link_data_stats *out);

/*
 * Drop everything: queued transmits, completed receives, buffers claimed by the reader.
 * Called from app/erpc.c whenever the link is opened, closed or abandoned -- all points
 * where the byte stream restarts and anything half-received is meaningless.
 *
 * It does NOT detach the consumer: a resident owner (the U3 network interface) keeps
 * existing across a link bounce, and tearing down somebody else's registration from a
 * recovery path is exactly the class of bug rtl_link_uart_gen() exists to prevent.
 */
void link_data_reset(void);

/* ---- internals used by the link service thread (app/erpc.c) -------------------- */

/* Claim a buffer for an incoming frame of @body_len bytes; NULL when the pool is empty
 * (the caller must then DRAIN those bytes -- never lose stream synchronisation). */
uint8_t *link_data_rx_claim(uint16_t body_len);

/* Give back a claimed buffer without a frame in it (reader reset / resync / close). */
void link_data_rx_abort(uint8_t *body);

/* A claimed buffer is now full: check the CRC and queue it for dispatch. */
void link_data_rx_commit(uint8_t *body, uint16_t body_len, uint16_t crc);

/* Hand every completed frame to the consumer.  MUST be called with no link lock held. */
void link_data_rx_dispatch(void);

/* Frames waiting for link_data_rx_dispatch(). */
int  link_data_rx_ready(void);

/* Look at the frame at the head of the transmit queue (NULL if none), then drop it
 * once its bytes have been handed to the UART. */
const uint8_t *link_data_tx_peek(uint16_t *body_len, uint16_t *crc);
void           link_data_tx_pop(void);

#endif /* APP_LINK_DATA_H */
