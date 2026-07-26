# N2..N7 patches land here (see ../README.md).  Applied in filename order to a
# pristine export of the pinned upstream commit -- the reference checkout in
# _ref/seeed-ambd-firmware is never modified.
#
# N2 -- bounded handlers (issue #20), wire format unchanged:
#
#   0001-n2-bounded-socket-handlers.patch
#       src/wifi/wifi_api.c.  Stops a stalled peer from wedging the single-threaded
#       eRPC server:
#         - rpc_lwip_recv/read/recvfrom honour the IDL `timeout` (ms) by saving,
#           setting and restoring SO_RCVTIMEO (an int of ms here,
#           LWIP_SO_SNDRCVTIMEO_NONSTANDARD=1).  timeout==0 keeps the socket's own
#           SO_RCVTIMEO, so existing callers (the host sets it itself) are unchanged.
#           If a non-zero bound cannot be armed the handler returns -1 instead of
#           running an unbounded receive.
#         - rpc_lwip_connect/accept (no IDL timeout) get an internal cap via
#           O_NONBLOCK + lwip_select, restoring the original socket flags exactly.
#
#   0002-n2-system-version-build-id.patch
#       src/erpc_setup.cpp.  rpc_system_version() returned a string literal that the
#       generated shim erpc_free()s -- freeing .rodata corrupts the module heap, so the
#       STM32 never called it.  Return an erpc_malloc copy of the build id
#       "2.1.3+wio-n2" instead, so the host can read back which firmware is loaded.
#       (The generated shim rpc_system_server.cpp is left untouched.)
#
# N3 -- worker dispatch (issue #20), wire format unchanged (reply order is not):
#
#   0003-n3-worker-dispatch.patch
#       src/erpc_setup.cpp only (the vendored eRPC runtime is not modified).  Replaces
#       the single poll()-loop server task with a receive task + a small worker pool via
#       a thin SimpleServer subclass that exposes runInternalBegin (receive+parse) and
#       runInternalEnd (handler+reply).  So a blocking socket receive no longer stalls
#       every other RPC.  Only the per-socket blocking receives (lwip accept/connect/
#       recv/read/recvfrom) plus rpc_system_ack run in parallel; every other handler
#       takes a single serial mutex and stays effectively serial.  Queue-full is
#       portMAX_DELAY back-pressure (degrades to the stock serial rate, never leaks).
#       Needs the N3 host client (app/erpc.c erpc_begin/erpc_wait/erpc_cancel), which
#       routes now-out-of-order replies by sequence.  Verify with `net conc`.
#
# N4 -- the link UART is ours (issue #23 U0-2), wire format unchanged:
#
#   0004-n4-usi-link-driver.patch
#       New src/link/wio_usi_uart.{c,h} + src/link/wio_uart_transport.h, and
#       src/erpc_setup.cpp switches the transport over (and the build id to
#       "2.1.3+wio-n4").  Removes the three things that made a big REQUEST unreliable
#       and capped the link near 40 kB/s, all in the Arduino layer under the eRPC
#       transport: RxFifoTrigLevel 1 (one interrupt per byte, one byte read out of a
#       64-deep FIFO), a 127-byte RingBuffer whose store_char() drops silently when
#       full, and a transport that read one byte per call.  The driver owns USI0
#       (PB_20/PB_21), drains the whole FIFO on a threshold of 32, buffers 8 kB and
#       reads in bulk memcpys.  `Serial3` is simply never begun.
#       Verify with `net echo 2323 256` -- a 280-byte request frame, which is the size
#       that gets no reply on N3 and earlier.
#
# N5 -- LINK-CTRL channel (issue #23 U0-3), eRPC wire format unchanged:
#
#   0005-n5-link-ctrl.patch
#       src/link/wio_uart_transport.h gets a receive() override that multiplexes a
#       second frame type onto the same wire (u16 0xFFFF | u16 len | u16 crc | body),
#       handled in the link layer and never passed up to eRPC; src/link/wio_usi_uart.c
#       gains wio_usi_set_baud()/wio_usi_baud(); the build id becomes "2.1.3+wio-n5".
#       Commands: LINK_PING, LINK_STATS (the N4 counters, readable at last WHILE traffic
#       flows -- the LOG UART cannot be attached while the host holds the link),
#       LINK_SETBAUD, LINK_BENCH.  0xFFFF is safe because no sender can produce a
#       message size that big, but a DESYNCHRONISED stream can align on it, so the
#       length is bounded before it is believed, the CRC is checked, and LINK_SETBAUD
#       additionally needs a magic word and a rate from an allow-list.  It ACKs on the
#       old rate then reprograms, so a lost ACK changes nothing.  Recovery from a rate
#       mismatch is `wifi reset` -- the fallback the host attempts is best effort only.
#       Verify with `link info`, `link sweep`, and a `link baud` round trip.
#
# N6 -- DATA channel (issue #23 U1), eRPC wire format unchanged:
#
#   0006-n6-link-data.patch
#       New src/link/wio_link_data.{c,h}: a THIRD frame type
#       (u16 0xFFFE | u16 len | u16 crc | body, body = u8 chan | u8 flags | payload)
#       that is unsolicited, bidirectional and unacknowledged -- what a raw Ethernet
#       frame needs.  src/link/wio_uart_transport.h gains handleData() and a public
#       writeFramed() (the DATA task must take the SAME m_sendLock the N3 workers take,
#       or two writers interleave bytes inside one frame); the receive ring doubles to
#       16 kB; the build id becomes "2.1.3+wio-n6".  In U1 the endpoint is a sink/source
#       bench, so the plumbing is measured before anything depends on it.
#       The transmit queue is an index ring under the module's own mutex, NOT a FreeRTOS
#       queue: LINK_DATA_CFG(off) can only promise "no further frame can be on the way"
#       if taking the next frame and marking the writer busy happen in ONE locked step.
#       Verify with `link dbench 1500 3 rx | tx | both` -- zero loss on either end.
#
# N7 -- L2 bridge (issue #23 U2), eRPC wire format unchanged:
#
#   0007-n7-l2-bridge.patch
#       New src/link/wio_link_eth.{c,h}: the module stops handing received Ethernet
#       frames to its own lwIP and sends them over DATA instead, and DATA frames on
#       WIO_DATA_CHAN_ETH go out over the air.  It is a TAP, not a rewrite: LwIP_Init()
#       still runs and the bridge swaps xnetif[0].input, so switching it off restores
#       the module's own stack (and therefore `net ping`, `net echo` and telnet) intact.
#       Two things that are not obvious and cost a board if got wrong:
#         - the WLAN driver FILTERS received IP packets against the netif's address
#           (netif_is_valid_IP, an undefined reference in lib_wlan.a), so the bridge
#           must zero xnetif[0]'s address and restore it afterwards;
#         - lwIP here is NO_SYS=0 with LWIP_TCPIP_CORE_LOCKING=0, so BOTH mutations go
#           through tcpip_callback_with_block() -- which only blocks until the message
#           is posted, hence the completion semaphore -- and the host stops the DHCP
#           client over the existing eRPC path first.
#       BRIDGE is a wio_link_data_cfg() MODE (0x04), not a command of its own, so N6's
#       CFG(off) contract covers the bridge's producer too; wio_link_data_send() is
#       re-expressed as tx_claim + copy + tx_commit so the WiFi receive path can copy a
#       pbuf chain straight into the pool buffer with no staging buffer anywhere.  The
#       CFG @ms field doubles as a watchdog: the module takes the bridge down itself if
#       the host dies mid-session.  New CTRL command LINK_ETH_INFO (7) returns the MAC,
#       whether the radio is up, and the bridge counters.  Build id "2.1.3+wio-n7".
#       Verify with `link arp <gateway>` -- an `is-at` reply is the whole path proved.
