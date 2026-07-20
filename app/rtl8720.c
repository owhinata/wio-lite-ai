/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/*
 * Wio Lite AI (STM32H725AEI6) -- onboard RTL8720DN WiFi/BLE companion driver
 * (issue #17).  See rtl8720.h for the wiring/summary.
 *
 * Design (plan codex-review LGTM):
 *  - Bare-register UART bring-up; GPIO/clock via HAL macros (house style, cf.
 *    app/psram.c, app/usb_cdc.c).  NEVER touches the RCC clock tree -- only the
 *    peripheral clock GATES -- so it is safe while the CPU runs XIP from OCTOSPI2.
 *  - Baud is derived from the inherited PCLK2 = 137.5 MHz (UART9/USART1 are on APB2;
 *    the bootloader leaves RCC_D2CCIP2R USART clock-select = 0 = PCLK2, and
 *    HCLK=SYSCLK/2=275 MHz, APB2 /2 -> PCLK2 = 137.5 MHz).
 *  - RX is interrupt-driven into a single SPSC ring (one UART active at a time).
 *    The ISR (prio 6, wrapped by the EPK hooks like OTG_HS) only writes `head`;
 *    rtl8720_uart_read() only writes `tail`.  Overflow drops the NEWEST byte to keep
 *    the ring strictly single-producer/single-consumer (the ISR never writes tail).
 *    A __DMB() orders the data store/load against the head/tail publish.
 *  - No DMA: the ISR/foreground both touch the ring with the CPU, so it is
 *    self-coherent under the D-cache (no MPU / clean / invalidate needed).
 */
#include "stm32h7xx_hal.h"
#include "tx_glue.h"      /* tx_glue_isr_enter/exit: EPK (issue #2) ISR accounting */
#include "timebase.h"     /* udelay: DWT busy-wait for the CHIP_EN reset pulse */
#include "rtl8720.h"

/* CHIP_EN = PC3 (schematic: WIFI_CHIP_EN, no board pull -> host drives it). */
#define RTL_EN_PORT   GPIOC
#define RTL_EN_PIN    GPIO_PIN_3

/* UART9/USART1 kernel clock = PCLK2 (see file header for the derivation). */
#define RTL_UART_PCLK2   137500000u

/* RX ring: power-of-two so head/tail wrap with a mask.  4 KB in AXI-SRAM BSS holds
 * ~350 ms of 115200-baud inflow -- ample against a brief USB CDC TX stall. */
#define RTL_RING_SIZE    4096u
#define RTL_RING_MASK    (RTL_RING_SIZE - 1u)

static volatile uint8_t  rtl_ring[RTL_RING_SIZE];
static volatile uint32_t rtl_ring_head;    /* producer: RX ISR only */
static volatile uint32_t rtl_ring_tail;    /* consumer: rtl8720_uart_read only */
static volatile uint32_t rtl_ring_drops;   /* RX bytes lost to overflow */

/* The currently-open UART, or NULL when closed.  Read by the shared ISR. */
static USART_TypeDef *volatile rtl_uart;
static IRQn_Type              rtl_irqn;

/* ------------------------------------------------------------------ *
 *  CHIP_EN (power/enable)
 * ------------------------------------------------------------------ */
void rtl8720_init(void)
{
	GPIO_InitTypeDef io = {0};

	__HAL_RCC_GPIOC_CLK_ENABLE();
	/* Preset ODR Low before switching the pad to output so the pin drives Low
	 * from the first instant (no High glitch): the module stays powered off. */
	HAL_GPIO_WritePin(RTL_EN_PORT, RTL_EN_PIN, GPIO_PIN_RESET);
	io.Pin   = RTL_EN_PIN;
	io.Mode  = GPIO_MODE_OUTPUT_PP;
	io.Pull  = GPIO_NOPULL;
	io.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(RTL_EN_PORT, &io);
	HAL_GPIO_WritePin(RTL_EN_PORT, RTL_EN_PIN, GPIO_PIN_RESET);
}

void rtl8720_power(bool on)
{
	HAL_GPIO_WritePin(RTL_EN_PORT, RTL_EN_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

bool rtl8720_powered(void)
{
	return (RTL_EN_PORT->ODR & RTL_EN_PIN) != 0u;
}

void rtl8720_reset(void)
{
	uint32_t i;

	HAL_GPIO_WritePin(RTL_EN_PORT, RTL_EN_PIN, GPIO_PIN_RESET);
	for (i = 0u; i < 80u; i++)          /* >=80 ms Low (conservative power-off) */
		udelay(1000u);
	HAL_GPIO_WritePin(RTL_EN_PORT, RTL_EN_PIN, GPIO_PIN_SET);
}

/* ------------------------------------------------------------------ *
 *  RX ring (SPSC)
 * ------------------------------------------------------------------ */
static void rtl_ring_reset(void)
{
	rtl_ring_head = 0u;
	rtl_ring_tail = 0u;
	rtl_ring_drops = 0u;
}

size_t rtl8720_uart_read(uint8_t *buf, size_t n)
{
	size_t got = 0u;
	uint32_t head = rtl_ring_head;      /* snapshot the producer's publish */
	uint32_t tail;

	__DMB();                             /* order head-load before the data reads */
	tail = rtl_ring_tail;
	while (got < n && tail != head) {
		buf[got++] = rtl_ring[tail];
		tail = (tail + 1u) & RTL_RING_MASK;
	}
	__DMB();                             /* data reads retire before tail publish */
	rtl_ring_tail = tail;
	return got;
}

uint32_t rtl8720_uart_overflows(void)
{
	return rtl_ring_drops;
}

/* Shared RX ISR body: drain the whole RX FIFO into the ring. */
static void rtl_uart_isr(void)
{
	USART_TypeDef *u = rtl_uart;
	uint32_t isr;

	if (u == NULL)
		return;
	isr = u->ISR;
	/* Clear sticky error/idle flags so they do not wedge the FIFO. */
	if (isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_IDLE))
		u->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF |
		         USART_ICR_IDLECF;
	/* RXFNE (RXFIFO-not-empty): drain every buffered byte, not just to a
	 * threshold, so tail bytes never stick (RM0468 sec 53.5.4). */
	while (u->ISR & USART_ISR_RXNE_RXFNE) {
		uint8_t b = (uint8_t)(u->RDR & 0xFFu);   /* read clears RXFNE for this byte */
		uint32_t head = rtl_ring_head;
		uint32_t next = (head + 1u) & RTL_RING_MASK;

		if (next == rtl_ring_tail) {              /* full: drop newest, keep SPSC */
			rtl_ring_drops++;
			continue;
		}
		rtl_ring[head] = b;
		__DMB();                                  /* data store before head publish */
		rtl_ring_head = next;
	}
}

void UART9_IRQHandler(void)
{
	tx_glue_isr_enter();
	rtl_uart_isr();
	tx_glue_isr_exit();
}

void USART1_IRQHandler(void)
{
	tx_glue_isr_enter();
	rtl_uart_isr();
	tx_glue_isr_exit();
}

/* ------------------------------------------------------------------ *
 *  UART bring-up / teardown
 * ------------------------------------------------------------------ */
void rtl8720_uart_close(void)
{
	if (rtl_uart == NULL)
		return;
	NVIC_DisableIRQ(rtl_irqn);
	__DSB();
	__ISB();                             /* the disable takes effect before we go on */
	rtl_uart->CR1 = 0u;                  /* UE=0: quiesce RE/TE/IRQ */
	rtl_uart = NULL;
}

int rtl8720_uart_open(enum rtl8720_uart which, uint32_t baud)
{
	GPIO_InitTypeDef io = {0};
	USART_TypeDef *u;
	IRQn_Type irqn;
	uint32_t spin, brr;

	if (which != RTL8720_UART_LOG && which != RTL8720_UART_AT)
		return -1;
	/* OVER8=0: BRR = USARTDIV, a 16-bit divider.  Reject a baud whose divider
	 * would not fit (min ~2098 = PCLK2/65535) rather than silently truncate. */
	if (baud == 0u)
		return -1;
	brr = (RTL_UART_PCLK2 + baud / 2u) / baud;
	if (brr < 16u || brr > 0xFFFFu)
		return -1;

	if (rtl_uart != NULL)
		rtl8720_uart_close();
	rtl_ring_reset();

	io.Mode  = GPIO_MODE_AF_PP;
	io.Pull  = GPIO_PULLUP;              /* idle-high RX when the module is not driving */
	io.Speed = GPIO_SPEED_FREQ_HIGH;

	if (which == RTL8720_UART_LOG) {
		/* UART9: PD14 RX / PD15 TX, both AF11. */
		__HAL_RCC_GPIOD_CLK_ENABLE();
		io.Alternate = GPIO_AF11_UART9;
		io.Pin = GPIO_PIN_14 | GPIO_PIN_15;
		HAL_GPIO_Init(GPIOD, &io);
		__HAL_RCC_UART9_CLK_ENABLE();
		u = UART9;
		irqn = UART9_IRQn;
	} else {
		/* USART1: PA10 RX (AF7) / PB14 TX (AF4). */
		__HAL_RCC_GPIOA_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();
		io.Alternate = GPIO_AF7_USART1;
		io.Pin = GPIO_PIN_10;
		HAL_GPIO_Init(GPIOA, &io);
		io.Alternate = GPIO_AF4_USART1;
		io.Pin = GPIO_PIN_14;
		HAL_GPIO_Init(GPIOB, &io);
		__HAL_RCC_USART1_CLK_ENABLE();
		u = USART1;
		irqn = USART1_IRQn;
	}

	/* Bare-register init (RM0468 sec 53.8): configure with UE=0, then enable.
	 * Order: NVIC off -> CR1=0 -> clear flags -> flush RX FIFO -> PRESC/BRR ->
	 * CR2/CR3 -> CR1(FIFOEN|RE|TE|RXFNEIE|UE) -> wait RE/TE ack. */
	NVIC_DisableIRQ(irqn);
	u->CR1 = 0u;
	u->CR2 = 0u;
	u->CR3 = 0u;
	u->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_IDLECF;
	u->RQR = USART_RQR_RXFRQ;            /* flush any stale RX FIFO content */
	u->PRESC = 0u;
	u->BRR = brr;                       /* OVER8=0: BRR=USARTDIV, rounded (validated above) */
	u->CR1 = USART_CR1_FIFOEN | USART_CR1_RE | USART_CR1_TE |
	         USART_CR1_RXNEIE_RXFNEIE | USART_CR1_UE;

	/* Publish the active UART, then enable its IRQ (NVIC was off during config so
	 * the ISR could not have run against a half-set-up peripheral). */
	rtl_uart = u;
	rtl_irqn = irqn;

	spin = 0u;                           /* bounded wait for RE/TE to come ready */
	while ((u->ISR & (USART_ISR_REACK | USART_ISR_TEACK)) !=
	       (USART_ISR_REACK | USART_ISR_TEACK)) {
		if (++spin > 200000u) {
			rtl8720_uart_close();
			return -1;
		}
	}

	NVIC_SetPriority(irqn, 6);           /* like OTG_HS; PRIMASK critical sections
	                                      * mask it regardless of priority */
	NVIC_ClearPendingIRQ(irqn);
	NVIC_EnableIRQ(irqn);
	return 0;
}

void rtl8720_uart_write(const uint8_t *buf, size_t n)
{
	USART_TypeDef *u = rtl_uart;
	size_t i;

	if (u == NULL)
		return;
	for (i = 0u; i < n; i++) {
		uint32_t spin = 0u;
		while (!(u->ISR & USART_ISR_TXE_TXFNF)) {
			if (++spin > 2000000u)       /* bounded: never hang the shell thread */
				return;
		}
		u->TDR = (uint32_t)buf[i];
	}
}
