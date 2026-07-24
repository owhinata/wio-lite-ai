# N2/N3 patches land here (see ../README.md).  Applied in filename order to a
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
