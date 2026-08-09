/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    cxx_runtime.cc
 * @brief   The whole C++ runtime this firmware has (issue #9 phase 2c).
 *
 * Compiled only into the `tflm` static library, so the default `null` build never sees
 * a line of it.  Its job is to keep the C++ runtime surface of a 384 KB image down to
 * the few functions on this page.
 *
 * 🔴 WHY OUR OWN operator new EXISTS, WHEN NOTHING SHOULD CALL IT.  With
 * TF_LITE_STATIC_MEMORY the interpreter serves every allocation from the arena with a
 * bump allocator and never reaches the heap.  But flatbuffers and the odd STL fragment
 * still REFERENCE operator new, and a reference is all the linker needs.  Left
 * undefined, it would be resolved from libstdc++_nano -- which is a prebuilt archive,
 * compiled WITH exceptions no matter what flags we use.  Its operator new throws
 * std::bad_alloc, so extracting it drags in __cxa_throw, the _Unwind_* personality
 * machinery and bad_alloc's type-info: tens of KB of code that -fno-exceptions
 * guarantees can never execute, in the one memory this board cannot spare it from.
 * Defining our own here means the linker resolves the symbol from this object and
 * never opens that archive member.  cmake/check_cxx_runtime.py verifies the outcome
 * after every link rather than trusting the reasoning.
 *
 * The same argument covers the nothrow forms below: libstdc++'s
 * `operator new(size_t, nothrow_t)` is implemented as a try/catch around the throwing
 * one, so referencing IT pulls the unwinder just as surely.
 *
 * Not covered, deliberately: the C++17 over-aligned forms
 * (`operator new(size_t, align_val_t)`).  Nothing in TFLM new's an over-aligned type
 * today, and if that ever changes the link resolves it from libstdc++_nano and
 * check_cxx_runtime.py fails the build with the exception symbols named.  A loud
 * failure at the moment the assumption breaks is worth more than two more definitions
 * whose correctness nobody would be in a position to test.
 *
 * The other three pieces of the runtime are removed by flags rather than replaced:
 * __cxa_guard_* by -fno-threadsafe-statics (function-local statics in this backend are
 * initialised once, from a single thread, under the nn session guard),
 * __cxa_atexit/__dso_handle by -fno-use-cxa-atexit (nothing here ever returns from
 * main, so a destructor registry would be RAM spent on code that cannot run), and the
 * unwind tables by -fno-unwind-tables.  check_cxx_runtime.py names all of them.
 */
#include "nn_tflm_priv.h"

#include <cstddef>
#include <cstdlib>
#include <new>

/* See nn_tflm_priv.h: this is evidence for `ai info`, not a statistic. */
extern "C" volatile uint32_t nn_tflm_cxx_new_calls;
volatile uint32_t nn_tflm_cxx_new_calls = 0u;

/*
 * Routed to malloc rather than returning nullptr: with TF_LITE_STATIC_MEMORY nothing
 * should reach these at all, and if something does, the useful outcome is that it
 * works while `ai info` reports a non-zero counter.  The counter is the claim; the
 * allocator behind it is a fallback.
 *
 * These deliberately do NOT run the new_handler loop the standard describes.  There is
 * no handler to install here, and the loop would only turn an out-of-memory into a
 * hang.  On this board malloc failing means the AXI-SRAM heap is exhausted, which
 * `free` reports and which no retry will fix.
 *
 * 🔴 THE THROWING FORMS MUST NOT RETURN NULL, so they spin instead.  The standard says
 * the non-nothrow operator new either returns storage or throws ([basic.stc.dynamic]);
 * with -fno-exceptions it cannot throw, and returning null would hand a null pointer to
 * a caller that -- correctly, by the contract -- never checks it.  The fault would then
 * land at some later dereference inside flatbuffers or a kernel, with nothing left to
 * say the heap was the cause.  Spinning keeps the program counter AT the allocation, so
 * SWD shows the answer immediately, and the IWDG turns it into a recorded reset within
 * ~3 s rather than a silent wedge.  Neither outcome is good; this one is diagnosable.
 *
 * The nothrow forms DO return null, because there the contract says they may and the
 * caller is required to check.
 */
static void *nn_tflm_alloc_or_spin(std::size_t n)
{
	void *p = malloc(n);

	if (!p) {
		/* Out of AXI-SRAM heap inside inference -- see above. */
		for (;;) { }
	}
	return p;
}

void *operator new(std::size_t n)
{
	nn_tflm_cxx_new_calls++;
	return nn_tflm_alloc_or_spin(n);
}

void *operator new[](std::size_t n)
{
	nn_tflm_cxx_new_calls++;
	return nn_tflm_alloc_or_spin(n);
}

void *operator new(std::size_t n, const std::nothrow_t &) noexcept
{
	nn_tflm_cxx_new_calls++;
	return malloc(n);
}

void *operator new[](std::size_t n, const std::nothrow_t &) noexcept
{
	nn_tflm_cxx_new_calls++;
	return malloc(n);
}

void operator delete(void *p) noexcept                       { free(p); }
void operator delete[](void *p) noexcept                     { free(p); }
void operator delete(void *p, std::size_t) noexcept          { free(p); }   /* C++14 sized */
void operator delete[](void *p, std::size_t) noexcept        { free(p); }
void operator delete(void *p, const std::nothrow_t &) noexcept   { free(p); }
void operator delete[](void *p, const std::nothrow_t &) noexcept { free(p); }

/*
 * Reached only if a pure virtual is called during construction or destruction of an
 * abstract base -- i.e. never, in code that works.  It is defined here so the link
 * never falls back to libsupc++ for it, and it spins rather than calling abort()
 * because abort() would pull newlib's exit path (and, through -specs=nosys, a _kill
 * that cannot work).  A spin is also the more diagnosable outcome on this board: the
 * IWDG resets it within ~3 s and `dmesg` shows the watchdog reset, whereas a silent
 * exit would look like a spontaneous reboot.
 */
extern "C" void __cxa_pure_virtual(void)
{
	for (;;) { }
}
