/*
 * CoreMark port for Wio Lite AI (STM32H725AEI6, Cortex-M7 @ 550 MHz, bare-metal
 * ThreadX shell app).  Timing uses the 1 ms SysTick (HAL_GetTick); output uses
 * printf over the USB CDC console (retargeted by the shell _write).
 * Derived from EEMBC's barebones core_portme.h (Apache-2.0).
 */
#ifndef CORE_PORTME_H
#define CORE_PORTME_H

#include <stddef.h>

/* Platform capabilities */
#define HAS_FLOAT   1   /* secs_ret is double; needs float-enabled printf  */
#define HAS_TIME_H  0   /* no <time.h>                                     */
#define USE_CLOCK   0   /* custom timing in core_portme.c                  */
#define HAS_STDIO   1   /* <stdio.h> available                            */
#define HAS_PRINTF  1   /* maps ee_printf -> printf (retargeted to the CDC) */

/* Report strings */
#ifndef COMPILER_VERSION
#ifdef __GNUC__
#define COMPILER_VERSION "GCC" __VERSION__
#else
#define COMPILER_VERSION "unknown"
#endif
#endif
#ifndef FLAGS_STR
#define FLAGS_STR "-O3 -funroll-loops -mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard"
#endif
#ifndef COMPILER_FLAGS
#define COMPILER_FLAGS FLAGS_STR
#endif
#ifndef MEM_LOCATION
#define MEM_LOCATION "STACK"
#endif

/* Data types (ee_ptr_int must hold a pointer) */
typedef signed short   ee_s16;
typedef unsigned short ee_u16;
typedef signed int     ee_s32;
typedef double         ee_f32;
typedef unsigned char  ee_u8;
typedef unsigned int   ee_u32;
typedef ee_u32         ee_ptr_int;
typedef size_t         ee_size_t;
#ifndef NULL
#define NULL ((void *)0)
#endif

#define align_mem(x) (void *)(4 + (((ee_ptr_int)(x)-1) & ~3))

/* Timing return type */
#define CORETIMETYPE ee_u32
typedef ee_u32 CORE_TICKS;

#ifndef SEED_METHOD
#define SEED_METHOD SEED_VOLATILE
#endif

#ifndef MEM_METHOD
#define MEM_METHOD MEM_STACK
#endif

#ifndef MULTITHREAD
#define MULTITHREAD 1
#define USE_PTHREAD 0
#define USE_FORK    0
#define USE_SOCKET  0
#endif

#ifndef MAIN_HAS_NOARGC
#define MAIN_HAS_NOARGC 1   /* no argc/argv on bare metal */
#endif

#ifndef MAIN_HAS_NORETURN
#define MAIN_HAS_NORETURN 0
#endif

/* Must be 1 for this simple single-context port */
extern ee_u32 default_num_contexts;

typedef struct CORE_PORTABLE_S
{
    ee_u8 portable_id;
} core_portable;

void portable_init(core_portable *p, int *argc, char *argv[]);
void portable_fini(core_portable *p);

#if !defined(PROFILE_RUN) && !defined(PERFORMANCE_RUN) \
    && !defined(VALIDATION_RUN)
#if (TOTAL_DATA_SIZE == 1200)
#define PROFILE_RUN 1
#elif (TOTAL_DATA_SIZE == 2000)
#define PERFORMANCE_RUN 1
#else
#define VALIDATION_RUN 1
#endif
#endif

int ee_printf(const char *fmt, ...);

#endif /* CORE_PORTME_H */
