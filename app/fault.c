/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    fault.c
 * @brief   Cortex-M7 fault handlers + crash record (Wio Lite AI / STM32H725).
 *
 * Strong HardFault/MemManage/BusFault/UsageFault handlers override the CMSIS
 * startup weak aliases (which only spin in Default_Handler).  On a fault the
 * crash is recorded to the reset-persistent RAM log (svc/log.c) -- two LOG_ERR
 * lines that survive the reset -- so `dmesg` after the board comes back shows
 * what happened.
 *
 * Unlike the stm32f746g-disco sibling this port does NOT dump registers / stack /
 * backtrace over a live UART: the only console here is USB CDC, which cannot be
 * driven from fault context (TinyUSB + ThreadX are dead).  Instead the handler
 * records to RAM and then RESETS the board (NVIC_SystemReset) so USB
 * re-enumerates and the log can be replayed.  If a debugger owns the core it
 * spin-halts instead, so SWD post-mortem stays possible (the old ST-Link cannot
 * attach to a sleeping core, so this is a busy loop, not WFI).
 *
 * The exception entry is captured by a small naked stub shared by all four
 * vectors (the type is read back from SCB->ICSR), which hands the C handler the
 * stacked frame and EXC_RETURN.  Clean-room: concept from NuttX armv7-m fault
 * handlers / Zephyr log_panic; no code reused.
 */
#define LOG_TAG "fault"
#include "log.h"

#include <stdint.h>

#include "stm32h7xx_hal.h"
#include "iwdg.h"        /* BSP_ENABLE_IWDG gate for the debugger-halt IWDG1 pet */

/* ---- init -------------------------------------------------------------- */

void fault_init(void)
{
	/* Route MemManage/Bus/Usage to their own handlers instead of escalating to
	 * HardFault, so the record can classify the cause precisely (PM0253 4.3.9). */
	SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk |
	              SCB_SHCSR_USGFAULTENA_Msk;
	/* Trap integer divide-by-zero as a UsageFault (cheap, catches a real bug).
	 * UNALIGN_TRP is left off: HAL/memcpy do intentional unaligned accesses. */
	SCB->CCR |= SCB_CCR_DIV_0_TRP_Msk;
	__DSB();
	__ISB();
}

/* ---- helpers ----------------------------------------------------------- */

/* True for addresses in on-chip RAM where a live stack can legitimately sit:
 * DTCM (128 KB) or AXI-SRAM (320 KB, holds the ThreadX thread + MSP stacks).
 * Used to validate the stacked frame before dereferencing it. */
static int addr_in_ram(uint32_t a)
{
	return (a >= 0x20000000u && a < 0x20020000u) ||     /* DTCM 128 KB */
	       (a >= 0x24000000u && a < 0x24050000u);        /* AXI-SRAM 320 KB */
}

/* Final resting state: spin only while a debugger owns the core (DHCSR
 * C_DEBUGEN), so SWD post-mortem is possible; otherwise reset the board so USB
 * re-enumerates and `dmesg` can replay the crash record. */
static void fault_rest(void)
{
	if (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) {
		for (;;) {                  /* busy loop (not WFI): old ST-Link can attach */
#if BSP_ENABLE_IWDG
			/* Pet the IWDG1 directly (no HAL handle -- fault context, IRQs off) so
			 * the board survives at the fault until a human attaches over SWD.  Once
			 * they halt the core the pet stops and the IWDG resets (~timeout), but
			 * the crash is already in the reset-persistent RAM log (dmesg). */
			IWDG1->KR = 0x0000AAAAu;
#endif
		}
	}
	NVIC_SystemReset();             /* does not return */
	for (;;)
		;                           /* belt-and-suspenders */
}

/* ---- C fault handler --------------------------------------------------- */

void fault_handler_c(uint32_t *frame, uint32_t exc_return)
{
	static volatile uint32_t in_fault;

	__disable_irq();
	if (in_fault)
		fault_rest();                   /* secondary fault while recording: rest */
	in_fault = 1u;

	/* The stacked frame may itself be unreadable on a stacking fault
	 * (MSTKERR/STKERR) or a wild SP (PM0253 2.5.1 / 4.3.10): validate the 8-word
	 * basic-frame span is in RAM before dereferencing it, else fall back to a
	 * registers-only record so SCB status + EXC_RETURN still come out. */
	int frame_ok = addr_in_ram((uint32_t)frame) &&
	               addr_in_ram((uint32_t)frame + 31u);

	/* Basic exception frame R0-R3, R12, LR, PC, xPSR is always the lowest 8
	 * words of the stacked frame -- on an FPU-extended frame the S0-S15/FPSCR
	 * context is stacked ABOVE it, so frame[0..7] are correct either way
	 * (PM0253 2.4.7); only the frame SIZE differs, handled in the SP calc. */
	uint32_t lr = 0, pc = 0, xpsr = 0, sp = 0;
	if (frame_ok) {
		lr = frame[5]; pc = frame[6]; xpsr = frame[7];

		/* SP at the fault = frame + frame size (basic 8 words, or 26-word
		 * FPU-extended frame when EXC_RETURN bit4 is clear) + 4 if the stacked
		 * xPSR bit9 flags STKALIGN padding. */
		sp = (uint32_t)frame + ((exc_return & 0x10u) ? 8u : 26u) * 4u;
		if (xpsr & (1u << 9))
			sp += 4u;
	}

	uint32_t cfsr  = SCB->CFSR;
	uint32_t hfsr  = SCB->HFSR;
	uint32_t mmfar = SCB->MMFAR;
	uint32_t bfar  = SCB->BFAR;

	uint32_t vect = SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk;
	const char *name = (vect == 3u) ? "HardFault"  :
	                   (vect == 4u) ? "MemManage"  :
	                   (vect == 5u) ? "BusFault"   :
	                   (vect == 6u) ? "UsageFault" : "Fault";

	/* Record to the reset-persistent RAM log -- two lines, replayed by `dmesg`. */
	LOG_ERR("%s cfsr=%08lx hfsr=%08lx mmfar=%08lx bfar=%08lx",
	        name, (unsigned long)cfsr, (unsigned long)hfsr,
	        (unsigned long)mmfar, (unsigned long)bfar);
	if (frame_ok)
		LOG_ERR("pc=%08lx lr=%08lx psr=%08lx sp=%08lx exc=%08lx",
		        (unsigned long)pc, (unsigned long)lr, (unsigned long)xpsr,
		        (unsigned long)sp, (unsigned long)exc_return);
	else
		LOG_ERR("frame lost (stacking fault?) frame=%08lx exc=%08lx",
		        (unsigned long)(uintptr_t)frame, (unsigned long)exc_return);

	fault_rest();                       /* reset (or halt under a debugger) */
}

/* ---- naked entry stubs ------------------------------------------------- */

/* One stub for all four fault vectors: select MSP/PSP from EXC_RETURN bit2 and
 * pass the frame (r0) + EXC_RETURN (r1) to the C handler.  The C handler reads
 * the precise fault type from SCB->ICSR, so the stubs need not differ. */
__attribute__((naked)) void HardFault_Handler(void)
{
	__asm volatile(
		"tst   lr, #4            \n"
		"ite   eq                \n"
		"mrseq r0, msp           \n"
		"mrsne r0, psp           \n"
		"mov   r1, lr            \n"
		"b     fault_handler_c   \n");
}
void MemManage_Handler(void)  __attribute__((alias("HardFault_Handler")));
void BusFault_Handler(void)   __attribute__((alias("HardFault_Handler")));
void UsageFault_Handler(void) __attribute__((alias("HardFault_Handler")));
