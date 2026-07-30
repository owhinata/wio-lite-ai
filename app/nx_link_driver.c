/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/*
 * NetX Duo link driver over the RTL8720 link's DATA channel (issue #23 U3).
 * See app/nx_link_driver.h for what this is and which thread runs what.
 */
#include "nx_link_driver.h"

#include <string.h>

#include "nx_ip.h"
#include "nx_arp.h"
#include "nx_rarp.h"
#include "nx_packet.h"

#include "link_data.h"
#include "nx_net.h"      /* the transmit budget asserted just below */

/*
 * The transmit budget from app/nx_net.h, checked HERE (issue #27) because this is the
 * translation unit that owns the transmit path -- it lived in nx_echo.c until then,
 * where deleting a diagnostic command would have silently taken the production driver's
 * sizing guarantee with it.  If a future socket or a deeper queue breaks it, the build
 * stops rather than the console stalling on a TCP retransmit timer.
 */
_Static_assert(NXN_TCP_SOCKETS_MAX * NXN_TCP_TX_DEPTH + NX_ARP_MAX_QUEUE_DEPTH +
                       NXN_TX_SPARE <= LINK_DATA_TX_BUFS,
               "LINK_DATA_TX_BUFS is too shallow for the NetX transmit budget "
               "(see the transmit budget note in app/nx_net.h)");

/* ---- constants ------------------------------------------------------------- */

#define ETH_HDR_SIZE     14u
#define ETH_FRAME_MAX    1514u                 /* 14 + 1500, no VLAN tag         */
#define ETH_MTU          1500u

#define ETHTYPE_IP       0x0800u
#define ETHTYPE_ARP      0x0806u
#define ETHTYPE_RARP     0x8035u

/*
 * Where a received frame is placed inside the packet payload.  nx_packet_allocate() with
 * NX_RECEIVE_PACKET (which is 0) points prepend_ptr at the start of the payload, and the
 * payload itself is NX_PACKET_ALIGNMENT (8) aligned.  Put the frame two bytes in and the
 * IPv4 header, 14 bytes further along, lands at +16 -- aligned.  This is the same
 * arithmetic the f746 driver applies for its DMA buffers, for the same reason and with
 * the same magic number.
 */
#define RX_FRAME_PAD     2u

/* ---- state ----------------------------------------------------------------- */

static struct {
	NX_IP           *ip;
	NX_INTERFACE    *iface;
	NX_PACKET_POOL  *pool;
	UINT             iface_index;
	ULONG            mac_msw;
	ULONG            mac_lsw;
	volatile uint8_t link_up;
	uint8_t          started;
	ULONG            speed_bps;      /* the link's real bit rate, set by nx_net.c    */
	struct nx_link_stats st;
} g;

/*
 * Staging buffer for a chained NX_PACKET.  With a 1600-byte pool payload and a 1514-byte
 * maximum frame, NetX has no reason to chain, so this should never be used -- which is
 * exactly why it exists: an unexpected chain becomes a counted, correct transmit rather
 * than a truncated frame or a walk off the end of a fragment.  Only ever touched on the
 * NetX IP thread, which is the only caller of the transmit path.
 */
static uint8_t tx_coalesce[ETH_FRAME_MAX] __attribute__((aligned(8)));

/* ---- transmit -------------------------------------------------------------- */

/*
 * Undo the 14-byte Ethernet header this driver prepended.
 *
 * This is not tidiness.  _nx_packet_transmit_release() rewinds only NetX's OWN headers,
 * so a packet it retains (a TCP segment awaiting acknowledgement) comes back with
 * prepend_ptr 14 bytes -- i.e. 2 mod 4 -- below where NetX left it, and the next
 * _nx_ip_header_add() does an unaligned 32-bit store into it.  The f746 port took a
 * UsageFault to learn this (its issue #79).  Every release path below calls this first.
 */
static void link_tx_unprepend(NX_PACKET *pkt)
{
	pkt->nx_packet_prepend_ptr += ETH_HDR_SIZE;
	pkt->nx_packet_length      -= ETH_HDR_SIZE;
}

static void link_tx(NX_IP_DRIVER *req)
{
	NX_PACKET *pkt = req->nx_ip_driver_packet;
	UINT       cmd = req->nx_ip_driver_command;
	USHORT     etype;
	ULONG      dmsw, dlsw;
	uint8_t   *eh;
	const uint8_t *frame;
	ULONG      len;

	if (cmd == NX_LINK_ARP_SEND || cmd == NX_LINK_ARP_RESPONSE_SEND)
		etype = ETHTYPE_ARP;
	else if (cmd == NX_LINK_RARP_SEND)
		etype = ETHTYPE_RARP;
	else
		etype = ETHTYPE_IP;

	if (!g.link_up) {
		g.st.tx_link_down++;
		_nx_packet_transmit_release(pkt);
		return;
	}

	/* Prepend the Ethernet header into the room NX_PHYSICAL_HEADER reserved. */
	pkt->nx_packet_prepend_ptr -= ETH_HDR_SIZE;
	pkt->nx_packet_length      += ETH_HDR_SIZE;
	eh   = pkt->nx_packet_prepend_ptr;
	dmsw = req->nx_ip_driver_physical_address_msw;
	dlsw = req->nx_ip_driver_physical_address_lsw;
	eh[0]  = (uint8_t)(dmsw >> 8);       eh[1]  = (uint8_t)dmsw;
	eh[2]  = (uint8_t)(dlsw >> 24);      eh[3]  = (uint8_t)(dlsw >> 16);
	eh[4]  = (uint8_t)(dlsw >> 8);       eh[5]  = (uint8_t)dlsw;
	eh[6]  = (uint8_t)(g.mac_msw >> 8);  eh[7]  = (uint8_t)g.mac_msw;
	eh[8]  = (uint8_t)(g.mac_lsw >> 24); eh[9]  = (uint8_t)(g.mac_lsw >> 16);
	eh[10] = (uint8_t)(g.mac_lsw >> 8);  eh[11] = (uint8_t)g.mac_lsw;
	eh[12] = (uint8_t)(etype >> 8);      eh[13] = (uint8_t)etype;

	len = pkt->nx_packet_length;
	if (len > ETH_FRAME_MAX) {
		g.st.tx_oversize++;
		goto release;
	}

	if (pkt->nx_packet_next == NX_NULL) {
		frame = pkt->nx_packet_prepend_ptr;
	} else {
		NX_PACKET *f;
		uint8_t   *d = tx_coalesce;

		for (f = pkt; f != NX_NULL; f = f->nx_packet_next) {
			ULONG l = (ULONG)(f->nx_packet_append_ptr - f->nx_packet_prepend_ptr);

			if ((ULONG)(d - tx_coalesce) + l > sizeof tx_coalesce) {
				g.st.tx_oversize++;
				goto release;
			}
			memcpy(d, f->nx_packet_prepend_ptr, l);
			d += l;
		}
		g.st.tx_coalesced++;
		frame = tx_coalesce;
		len   = (ULONG)(d - tx_coalesce);
	}

	if (link_data_send(LINK_DATA_CHAN_ETH, frame, (uint16_t)len) != 0) {
		/* The DATA transmit pool was full.  Dropping is what an Ethernet device
		 * does when its ring is full, and the protocols above expect it. */
		g.st.tx_no_buf++;
	} else {
		g.st.tx_frames++;
		g.st.tx_bytes += len;
	}

release:
	/* The frame has been copied, so the packet is finished with either way -- there is
	 * no transmit-complete callback here to release it later. */
	link_tx_unprepend(pkt);
	_nx_packet_transmit_release(pkt);
}

/* ---- receive --------------------------------------------------------------- */

int nx_link_driver_rx(void *ctx, uint8_t chan, uint8_t *p, uint16_t n)
{
	NX_PACKET *pkt;
	USHORT     etype;

	(void)ctx;

	if (chan != LINK_DATA_CHAN_ETH)
		return 0;                        /* a stray bench frame; not ours       */

	if (!g.link_up || g.ip == NX_NULL || g.pool == NX_NULL) {
		g.st.rx_link_down++;
		return 0;
	}
	if (n < ETH_HDR_SIZE) {
		g.st.rx_undersize++;
		return 0;
	}
	if (n > ETH_FRAME_MAX) {
		g.st.rx_oversize++;
		return 0;
	}

	/* NX_NO_WAIT is mandatory, not an optimisation: this runs on the thread that
	 * drains the UART, and _nx_packet_allocate() with a wait option would suspend it.
	 * Allocation itself only masks interrupts briefly, so it is safe here. */
	if (_nx_packet_allocate(g.pool, &pkt, NX_RECEIVE_PACKET, NX_NO_WAIT) != NX_SUCCESS) {
		g.st.rx_no_buf++;
		return 0;
	}

	memcpy(pkt->nx_packet_prepend_ptr + RX_FRAME_PAD, p, n);
	pkt->nx_packet_prepend_ptr += RX_FRAME_PAD;
	pkt->nx_packet_append_ptr   = pkt->nx_packet_prepend_ptr + n;
	pkt->nx_packet_length       = n;
	pkt->nx_packet_ip_interface = g.iface;

	g.st.rx_frames++;
	g.st.rx_bytes += n;

	etype = (USHORT)(((USHORT)pkt->nx_packet_prepend_ptr[12] << 8) |
	                 pkt->nx_packet_prepend_ptr[13]);

	/* Hand the payload over with the L2 header stripped, exactly as a MAC driver would. */
	pkt->nx_packet_prepend_ptr += ETH_HDR_SIZE;
	pkt->nx_packet_length      -= ETH_HDR_SIZE;

	/*
	 * The DEFERRED variants, unlike the f746 driver which is already on the IP helper
	 * thread when it dispatches.  These only queue the packet and set an event flag, so
	 * the protocol work happens on the IP thread and this callback keeps its promise to
	 * return quickly -- while it runs, nothing is emptying the UART's 16 kB ring.
	 */
	switch (etype) {
	case ETHTYPE_IP:
		_nx_ip_packet_deferred_receive(g.ip, pkt);
		break;
	case ETHTYPE_ARP:
		_nx_arp_packet_deferred_receive(g.ip, pkt);
		break;
	case ETHTYPE_RARP:
		_nx_rarp_packet_deferred_receive(g.ip, pkt);
		break;
	default:
		g.st.rx_unknown_type++;
		_nx_packet_release(pkt);
		break;
	}

	return 0;                                /* the DATA buffer is finished with */
}

/* ---- driver entry ---------------------------------------------------------- */

VOID nx_link_driver(NX_IP_DRIVER *req)
{
	NX_IP        *ip    = req->nx_ip_driver_ptr;
	NX_INTERFACE *iface = req->nx_ip_driver_interface;

	req->nx_ip_driver_status = NX_SUCCESS;

	switch (req->nx_ip_driver_command) {
	case NX_LINK_INTERFACE_ATTACH:
		g.iface = iface;
		break;

	case NX_LINK_INITIALIZE:
		g.ip          = ip;
		g.iface       = iface;
		g.iface_index = iface->nx_interface_index;

		nx_ip_interface_mtu_set(ip, g.iface_index, ETH_MTU);
		nx_ip_interface_address_mapping_configure(ip, g.iface_index, NX_TRUE);

		/*
		 * The MAC belongs to the RTL8720 and is not known until a bridge session
		 * reads it with LINK_ETH_INFO, so start from all-zero and let
		 * nx_link_driver_set_mac() fill it in.  Nothing can be transmitted before
		 * then anyway: the link starts down.
		 */
		g.mac_msw = 0;
		g.mac_lsw = 0;
		nx_ip_interface_physical_address_set(ip, g.iface_index, 0, 0, NX_FALSE);

		/* NetX sets this optimistically true; the link is genuinely down until a
		 * `net up` brings the bridge in. */
		iface->nx_interface_link_up = NX_FALSE;
		g.link_up = 0;
		break;

	case NX_LINK_SET_PHYSICAL_ADDRESS:
		/*
		 * Reached from nx_ip_interface_physical_address_set(..., NX_TRUE).  Taking
		 * the address from here rather than caching it separately is what keeps the
		 * interface's idea of the MAC and the one this driver writes into every
		 * frame from ever drifting apart.
		 */
		g.mac_msw = req->nx_ip_driver_physical_address_msw;
		g.mac_lsw = req->nx_ip_driver_physical_address_lsw;
		break;

	case NX_LINK_ENABLE:
		g.started = 1;
		break;

	case NX_LINK_DISABLE:
	case NX_LINK_UNINITIALIZE:
		g.started = 0;
		break;

	case NX_LINK_PACKET_SEND:
	case NX_LINK_PACKET_BROADCAST:
	case NX_LINK_ARP_SEND:
	case NX_LINK_ARP_RESPONSE_SEND:
	case NX_LINK_RARP_SEND:
		link_tx(req);
		break;

	case NX_LINK_GET_STATUS:
		*req->nx_ip_driver_return_ptr = (ULONG)(g.link_up ? NX_TRUE : NX_FALSE);
		break;
	case NX_LINK_GET_SPEED:
		/*
		 * The link's actual bit rate (6,000,000 at the rate `wifi link baud` currently
		 * uses), not a made-up Ethernet 10/100.  NetX does not act on this; it is a
		 * status query, so the honest number is the useful one.
		 */
		*req->nx_ip_driver_return_ptr = g.speed_bps;
		break;
	case NX_LINK_GET_DUPLEX_TYPE:
		/* A UART with separate TX and RX lines really is full duplex, and U1
		 * measured both directions running at once at the wire rate. */
		*req->nx_ip_driver_return_ptr = (ULONG)NX_TRUE;
		break;

	case NX_LINK_MULTICAST_JOIN:
	case NX_LINK_MULTICAST_LEAVE:
		/* Nothing filters multicast on this path: the module's WLAN driver hands up
		 * whatever it received and this driver forwards every frame it is given.
		 * Accepting the request truthfully describes the result. */
		break;

	default:
		req->nx_ip_driver_status = NX_UNHANDLED_COMMAND;
		break;
	}
}

/* ---- setters used by app/nx_net.c ------------------------------------------ */

VOID nx_link_driver_set_pool(NX_PACKET_POOL *pool)
{
	g.pool = pool;
}

void nx_link_driver_set_speed(uint32_t bits_per_sec)
{
	g.speed_bps = (ULONG)bits_per_sec;
}

int nx_link_driver_set_mac(const uint8_t mac[6])
{
	ULONG msw, lsw;

	if (g.ip == NX_NULL)
		return -1;

	msw = ((ULONG)mac[0] << 8) | mac[1];
	lsw = ((ULONG)mac[2] << 24) | ((ULONG)mac[3] << 16) |
	      ((ULONG)mac[4] << 8) | mac[5];

	/* NX_TRUE routes it through NX_LINK_SET_PHYSICAL_ADDRESS above, so g.mac_* and the
	 * interface are written from the same call. */
	if (nx_ip_interface_physical_address_set(g.ip, g.iface_index, msw, lsw,
	                                         NX_TRUE) != NX_SUCCESS)
		return -1;
	return 0;
}

void nx_link_driver_set_link(int up)
{
	if (g.ip == NX_NULL)
		return;

	g.link_up = up ? 1u : 0u;
	/*
	 * Only flag the change; the IP thread asks NX_LINK_GET_STATUS for the real state and
	 * runs the status-change callback, which updates the interface flag and nothing else
	 * (nx_net.c records why starting DHCP from there would be a lock-order cycle).  Note
	 * that _nx_ip_deferred_link_status_process() returns immediately when no callback is
	 * registered, so nx_ip_link_status_change_notify_set() is not optional.
	 */
	_nx_ip_driver_link_status_event(g.ip, g.iface_index);
}

int nx_link_driver_link_up(void)
{
	return g.link_up ? 1 : 0;
}

void nx_link_driver_get_stats(struct nx_link_stats *out)
{
	*out = g.st;
}

void nx_link_driver_reset_stats(void)
{
	memset(&g.st, 0, sizeof g.st);
}
