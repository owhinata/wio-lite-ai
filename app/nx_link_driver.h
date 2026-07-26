/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/*
 * Wio Lite AI (STM32H725AEI6) -- the NetX Duo link driver (issue #23 U3).
 *
 * NetX Duo needs an Ethernet MAC.  This board does not have one: it has an RTL8720DN
 * that was turned into a relay of raw Ethernet frames in U2, reachable over the link's
 * DATA channel (app/link_data.h).  So this file is what nx_eth_driver.c is in the sibling
 * f746 project -- the NX_LINK_* dispatch -- with the MAC, its DMA descriptors, the
 * zero-copy allocate/link callbacks and the PHY all replaced by two calls:
 *
 *      transmit   link_data_send(LINK_DATA_CHAN_ETH, frame, len)
 *      receive    the link_data_attach() callback, nx_link_driver_rx()
 *
 * It knows nothing about the RTL8720, the eRPC link, or how the bridge is turned on and
 * kept alive.  That is app/nx_net.c, which owns the session and calls the four setters
 * below.  Keeping the split here means this file can be read as "a NetX driver" and that
 * one as "an interface that comes and goes".
 *
 * ---- context ------------------------------------------------------------------
 *
 *   nx_link_driver()      the NetX IP thread (and, at create time, whoever calls
 *                         nx_ip_create)
 *   nx_link_driver_rx()   THE LINK SERVICE THREAD, priority 10, no link lock held,
 *                         and contractually obliged to return quickly -- nothing is
 *                         draining the UART's receive ring while it runs
 *   the setters           the nx_net owner thread
 */
#ifndef APP_NX_LINK_DRIVER_H
#define APP_NX_LINK_DRIVER_H

#include <stdint.h>

#include "nx_api.h"

/* NetX driver entry point, passed to nx_ip_create(). */
VOID nx_link_driver(NX_IP_DRIVER *driver_req_ptr);

/*
 * Give the driver the pool it allocates received frames from.  MUST be called before
 * nx_ip_create(), which runs NX_LINK_INITIALIZE on the IP thread.
 */
VOID nx_link_driver_set_pool(NX_PACKET_POOL *pool);

/*
 * The MAC to use as the source address.  Not known at boot -- it belongs to the module
 * and only LINK_ETH_INFO reports it -- so the interface starts with 00:00:00:00:00:00 and
 * this is called once the bridge is up.  Goes through nx_ip_interface_physical_address_set()
 * so the interface and the driver can never disagree.
 */
int nx_link_driver_set_mac(const uint8_t mac[6]);

/* The link's real bit rate, reported verbatim by NX_LINK_GET_SPEED.  It is a property of
 * the UART, which `wifi link baud` can change, so it is told to the driver rather than baked
 * in as an Ethernet-shaped 10 or 100. */
void nx_link_driver_set_speed(uint32_t bits_per_sec);

/*
 * Raise or drop the link.  Down means every transmit is dropped and counted; NetX is told
 * through _nx_ip_driver_link_status_event(), which wakes the IP thread to ask
 * NX_LINK_GET_STATUS and run the status-change callback -- which updates the interface
 * flag and nothing more (app/nx_net.c explains why it must not start DHCP from there).
 */
void nx_link_driver_set_link(int up);
int  nx_link_driver_link_up(void);

/*
 * The link_data consumer callback.  Registered by nx_net.c with link_data_attach(), never
 * called from here.  Always returns 0: the frame is copied into an NX_PACKET, so the DATA
 * pool buffer is finished with by the time it returns.
 */
int nx_link_driver_rx(void *ctx, uint8_t chan, uint8_t *p, uint16_t n);

/*
 * Counters.  Deliberately split finely: "a frame did not make it" has half a dozen very
 * different causes here and each one points at a different subsystem.
 */
struct nx_link_stats {
	uint32_t rx_frames;
	uint32_t rx_bytes;
	uint32_t rx_no_buf;        /* the NetX packet pool was empty                    */
	uint32_t rx_oversize;      /* longer than an Ethernet frame can be              */
	uint32_t rx_undersize;     /* shorter than a 14-byte header                     */
	uint32_t rx_unknown_type;  /* not IPv4 / ARP / RARP -- released, not an error   */
	uint32_t rx_link_down;     /* arrived before the interface was up               */
	uint32_t tx_frames;
	uint32_t tx_bytes;
	uint32_t tx_no_buf;        /* the link's DATA transmit pool was full (a drop)   */
	uint32_t tx_oversize;      /* NetX handed us more than 1514 bytes               */
	uint32_t tx_coalesced;     /* a chained NX_PACKET had to be flattened           */
	uint32_t tx_link_down;
};

void nx_link_driver_get_stats(struct nx_link_stats *out);
void nx_link_driver_reset_stats(void);

#endif /* APP_NX_LINK_DRIVER_H */
