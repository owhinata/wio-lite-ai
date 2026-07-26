/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/*
 * NetX Duo build configuration for Wio Lite AI (issue #23 U3).
 *
 * Pulled in by every NetX Duo translation unit through the Cortex-M7/GNU port:
 * ports/cortex_m7/gnu/inc/nx_port.h includes this file when NX_INCLUDE_USER_DEFINE_FILE
 * is defined, BEFORE applying its own #ifndef defaults, so every value here wins.  It is
 * the same mechanism as port/threadx/tx_user.h.
 *
 * This is the ONLY file under port/netxduo/.  The link driver lives in app/, because the
 * layer below it here is the RTL8720 link's DATA channel (app/link_data.h) rather than a
 * MAC on this side of the fence -- see app/nx_link_driver.c.
 *
 * DHCP-only knobs (NX_DHCP_THREAD_PRIORITY, NX_DHCP_CLIENT_USER_CREATE_PACKET_POOL) are
 * passed as -D on the build target instead; they affect the addon, not the core.
 */
#ifndef NX_USER_H
#define NX_USER_H

/*
 * CRITICAL.  The Cortex-M7/GNU nx_port.h hard-defaults NX_IP_PERIODIC_RATE to 100, but
 * ThreadX here runs at 1000 Hz (port/threadx/tx_user.h, TX_TIMER_TICKS_PER_SECOND 1000).
 * NetX derives every time base it has -- ARP retransmit and aging, TCP retransmit, ICMP,
 * DHCP lease and renewal -- from this, so leaving it at 100 makes all of them run ten
 * times slow.  It must match the tick rate.  After this override, every wait_option in
 * the NetX API is a millisecond count.
 */
#define NX_IP_PERIODIC_RATE          1000

/* IPv4 only.  Every *ipv6* / *icmpv6* / *_nd_* source is wrapped in #ifdef
 * FEATURE_NX_IPV6, which nx_api.h only sets when this is unset, so they compile to empty
 * objects and --gc-sections drops them. */
#define NX_DISABLE_IPV6

/* Room reserved in every packet for L2 framing: the 14-byte Ethernet header padded to 16
 * so that the IPv4 header behind it stays 32-bit aligned, plus a 4-byte trailer.  The
 * driver's receive path depends on this exact value (app/nx_link_driver.c). */
#define NX_PHYSICAL_HEADER           16
#define NX_PHYSICAL_TRAILER          4

/*
 * 8, not the 32 the f746 port uses.  There, packet payloads are handed straight to the
 * Ethernet MAC's DMA out of non-cacheable SDRAM and 32 is the cache-line size.  Here the
 * only bus master that ever touches a payload is the CPU -- the frame is memcpy'd to and
 * from the link's DATA pool -- so no cache maintenance and no DMA alignment is involved.
 * 8 keeps the 64-bit copies the M7 emits aligned and wastes nothing.
 */
#define NX_PACKET_ALIGNMENT          8

/*
 * Enables nx_tcp_socket_queue_depth_notify_set.  Nothing in U3 creates a TCP socket, but
 * this define changes NX_TCP_SOCKET, and U4's telnet backend needs the notification to
 * resume output after back-pressure instead of timing out (the f746 nx_shell.c does
 * exactly this).  Defining it now means U4 adds a socket rather than an ABI change.
 */
#define NX_ENABLE_TCP_QUEUE_DEPTH_UPDATE_NOTIFY

/*
 * DHCP client thread stack.  nxd_dhcp_client.h declares nx_dhcp_thread_stack[] as a
 * member of the NX_DHCP control block behind a #ifndef-guarded 4096 default; the f746
 * port measured a 436 B high-water, so 2048 keeps a wide margin.  Overriding here shrinks
 * the member without editing the read-only submodule.
 */
#define NX_DHCP_THREAD_STACK_SIZE    2048

/*
 * Deliberately NOT defined here, and each for a reason:
 *
 *   NX_ENABLE_INTERFACE_CAPABILITY  There is no hardware checksum unit anywhere on this
 *                                   path -- the module relays raw frames and computes
 *                                   nothing -- so NetX must calculate every checksum
 *                                   itself.  Claiming an offload we do not have would put
 *                                   zeroes on the wire.
 *   NX_DRIVER_DEFERRED_PROCESSING   Nothing in this driver runs in an ISR, so there is no
 *                                   bottom half to schedule.  Received frames are handed
 *                                   to _nx_*_packet_deferred_receive() directly from the
 *                                   link service thread; those functions are not gated by
 *                                   this define.
 *   NX_ENABLE_TCPIP_OFFLOAD         That is road A of issue #23 -- leaving TCP/IP in the
 *                                   module -- which this whole series exists to replace.
 */

#endif /* NX_USER_H */
