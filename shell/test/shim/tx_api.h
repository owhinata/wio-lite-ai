/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 *
 * Host-only ThreadX type shim for the Shell core unit test.
 *
 * The core splits its ThreadX use into cli_core.c (calls tx_* APIs, firmware
 * only) and cli_session.c (line/dispatch logic, no tx_* calls).  To compile and
 * test cli_session.c on the host we still need `struct cli_instance` -- which
 * embeds ThreadX objects by value -- to be a complete type.  This shim provides
 * just the type names as small dummy structs and the base typedefs; it is found
 * ahead of the real tx_api.h via the test include path.  No ThreadX functions
 * are declared, because the host-tested code never calls any.
 */
#ifndef TX_API_H
#define TX_API_H

#include <stdint.h>

typedef unsigned long  ULONG;
typedef unsigned int   UINT;
typedef unsigned char  UCHAR;
typedef char           CHAR;
#define VOID void

/* Opaque control blocks, sized only so they can be embedded by value. */
typedef struct { void *opaque; } TX_THREAD;
typedef struct { void *opaque; } TX_EVENT_FLAGS_GROUP;
typedef struct { void *opaque; } TX_MUTEX;

#define TX_SUCCESS ((UINT)0)

#endif /* TX_API_H */
