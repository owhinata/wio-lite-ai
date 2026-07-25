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
 *    The ISR only writes `head`; rtl8720_uart_read() only writes `tail`.  Overflow
 *    drops the NEWEST byte to keep the ring strictly single-producer/single-consumer
 *    (the ISR never writes tail).  A __DMB() orders the data stores against the head
 *    publish -- ONCE per interrupt, after the whole drain loop (issue #23 U0-1).
 *  - No DMA: the ISR/foreground both touch the ring with the CPU, so it is
 *    self-coherent under the D-cache (no MPU / clean / invalidate needed).
 *
 * ---- RX interrupt scheme (issue #23 U0-1) -------------------------------------
 *
 * The link is being taken from 2 Mbaud towards 6 Mbaud (600 kB/s), where one
 * interrupt per byte would mean ~600 k IRQ/s.  So the RXFIFO is drained on a
 * THRESHOLD interrupt (CR3.RXFTIE + RXFTCFG) with CR1.IDLEIE recovering the tail of
 * a burst, instead of CR1.RXNEIE firing per byte.
 *
 * Picking the threshold is a latency budget, not a throughput one.  RM0468 sec 53.5.4
 * and the USART_ISR RXFT bit description: RXFT asserts once T data are held in total
 * (one in USART_RDR plus T-1 in the RXFIFO), and the RXFTCFG=101 note states that at
 * the full threshold RXFT asserts on the 16th datum, the 17th does NOT overrun and
 * the overrun happens on the 18th.  So the grace between RXFT asserting and losing a
 * byte is (18 - T) byte times:
 *
 *      RXFTCFG          T    IRQ/s @6Mbaud   grace      @6Mbaud    @2Mbaud
 *      011  (3/4)      12       50 k          6 B       10.0 us    30.0 us
 *      010  (1/2)       8       75 k         10 B       16.7 us    50.0 us   <- used
 *      001  (1/4)       4      150 k         14 B       23.4 us    70.0 us
 *
 * (For reference the old RXNEIE scheme is T=1 = 17 byte times = 85 us at 2 Mbaud, so
 * this is ~1/5 of the margin we used to run on.  That is the one real risk here, which
 * is why the ISR measures how much of the grace it actually consumes -- see
 * rtl8720_uart_stats().)
 *
 * TWO separate conditions have to hold, and it is easy to conflate them:
 *
 *  (a) ENTRY LATENCY.  What eats the grace is the delay from RXFT asserting to the
 *      first RDR read -- NOT how long the ISR then runs, because the drain loop keeps
 *      consuming bytes that land while it is running.  The delay comes from ThreadX
 *      PRIMASK critical sections (TX_PORT_USE_BASEPRI is not defined, so raising the
 *      NVIC priority does NOT exempt this ISR from them), the USB CDC / net-shell /
 *      cli-core PRIMASK byte regions, and any ISR at an equal or higher priority.
 *      OTG_HS runs at priority 6 (app/usb_cdc.c) and equal priorities do not pre-empt
 *      each other, so a dwc2 interrupt used to delay this one by its whole duration.
 *      Hence this ISR now runs at priority 5, ABOVE OTG_HS: a dropped UART byte is
 *      data loss, whereas USB has its own peripheral FIFO and NAKs.  Nothing here
 *      calls into TinyUSB, and tx_glue_isr_enter/exit guards its EPK nesting with
 *      PRIMASK, so a priority-5 ISR pre-empting the priority-6 one is safe.
 *
 *  (b) DRAIN RATE.  Per byte, the loop must be faster than the wire delivers, or one
 *      interrupt never catches up.  Measured on board #2 at 2 Mbaud: 3.3 us worst case
 *      for 8 bytes = ~230 cycles/byte, against 917 cycles/byte of arrival at 6 Mbaud
 *      (1.67 us at 550 MHz) -- about 4x headroom, so the loop converges.
 *
 * isr_max_bytes covers BOTH: everything the FIFO accumulated through the entry delay
 * and through the drain shows up in it, so "max stays at the threshold" is the single
 * number that says the scheme is holding.  Measured at 2 Mbaud it stayed at exactly 8
 * over both a short ack exchange and a ~2 KB scan reply (133 interrupts).
 *
 * BUT that measurement cannot clear a HIGHER baud, and issue #24 tracks why: this whole
 * path is fetched from OCTOSPI2 XIP, and the drain cost measured 8.7 us cold (7
 * interrupts) against 3.3 us warm (133) -- so the external-flash fetch is worth
 * microseconds.  At 2 Mbaud one byte time is 5 us, which is coarse enough to hide all
 * of it; at 6 Mbaud it is 1.67 us and it would show.  The entry path (vector read,
 * exception prologue, tx_glue_isr_enter) is XIP too and is not measured here at all.
 * Before raising the baud, move this path into ITCM / .RamFunc (issue #24) so the
 * argument becomes structural -- no XIP fetch in the RX path -- rather than a
 * workload-dependent measurement.
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

/* RX ring: power-of-two so head/tail wrap with a mask.  16 KB in AXI-SRAM BSS is
 * 27 ms of inflow at 6 Mbaud (600 kB/s) -- the consumer is the eRPC service thread,
 * which polls on 1 ms slices, so this absorbs a long scheduling hole. */
#define RTL_RING_SIZE    16384u
#define RTL_RING_MASK    (RTL_RING_SIZE - 1u)

/* RXFIFO threshold interrupt: 1/2 of the 16-deep FIFO (RM0468 sec 53.8.4,
 * USART_CR3 RXFTCFG[2:0] = 010).  See the interrupt-scheme note in the file header
 * for why 1/2 and not 3/4.  NOTE: the LPUART register table in RM0468 prints 110 for
 * "1/2"; that is a typo and does not apply to USART1. */
#define RTL_RXFTCFG      USART_CR3_RXFTCFG_1

/* Bytes the FIFO can still take after RXFT asserts, before an overrun: 18 - T with
 * T = 8.  The ISR high-water is compared against this to show how much of the
 * interrupt-latency budget is actually being used. */
#define RTL_ISR_GRACE    10u

static volatile uint8_t  rtl_ring[RTL_RING_SIZE];
static volatile uint32_t rtl_ring_head;    /* producer: RX ISR only */
static volatile uint32_t rtl_ring_tail;    /* consumer: rtl8720_uart_read only */
static volatile uint32_t rtl_ring_drops;   /* RX bytes lost to overflow */

/* ISR high-water marks (issue #23 U0-1).  Written by the ISR only; read without a
 * lock because each is a single word and they are diagnostics, not control flow. */
static volatile uint32_t rtl_isr_max_bytes;   /* bytes pulled from RDR in one interrupt */
static volatile uint32_t rtl_isr_max_cycles;  /* DWT cycles spent inside the ISR */
static volatile uint32_t rtl_isr_count;       /* interrupts taken since open */

/* Hardware receive errors, distinct from rtl_ring_drops (which is OUR ring
 * overflowing).  ORE means the USART's own RXFIFO overran -- the byte never reached
 * software at all, so it is the true failure the threshold scheme has to avoid.  FE/NE
 * (framing / noise) are the signal that a baud rate is wrong or marginal, which is
 * exactly what the U0-3 sweep needs to see per baud. */
static volatile uint32_t rtl_uart_ore;        /* USART overrun events */
static volatile uint32_t rtl_uart_ferr;       /* framing / noise events */

/* The currently-open UART, or NULL when closed.  Read by the shared ISR. */
static USART_TypeDef *volatile rtl_uart;
static IRQn_Type              rtl_irqn;

/*
 * Optional "bytes have landed" callback, run from the RX ISR right after the ring's
 * head is published (issue #23 U0-3).  Its only user is the eRPC service thread, which
 * otherwise notices new bytes on its 1 ms poll: for a 1500-byte frame at 6 Mbaud
 * (2.5 ms on the wire) that adds up to 1 ms per frame, enough to hide the difference
 * between 4 and 6 Mbaud in the U0-3 measurements.  The poll is deliberately KEPT as the
 * backstop, so this is a wake-up latency optimisation and nothing depends on it for
 * correctness -- a dropped notification costs at most one poll period.
 *
 * Cleared by open/close, so a callback can never outlive the session that installed it.
 */
static void (*volatile rtl_rx_notify)(void);

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
	rtl_isr_max_bytes  = 0u;
	rtl_isr_max_cycles = 0u;
	rtl_isr_count      = 0u;
	rtl_uart_ore       = 0u;
	rtl_uart_ferr      = 0u;
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

void rtl8720_uart_stats(struct rtl8720_uart_stats *out)
{
	if (out == NULL)
		return;
	out->isr_count      = rtl_isr_count;
	out->isr_max_bytes  = rtl_isr_max_bytes;
	out->isr_max_cycles = rtl_isr_max_cycles;
	out->isr_grace      = RTL_ISR_GRACE;
	out->drops          = rtl_ring_drops;
	out->ore            = rtl_uart_ore;
	out->ferr           = rtl_uart_ferr;
	out->ring_size      = RTL_RING_SIZE;
}

/* Shared RX ISR body: drain the whole RX FIFO into the ring.
 *
 * Entered on RXFT (FIFO at the threshold) or IDLE (the burst ended below it); an
 * IDLE with nothing buffered is normal and simply drains zero bytes.  Either way the
 * loop runs until RXFNE clears, so no byte is ever left stranded in the FIFO. */
static void rtl_uart_isr(void)
{
	USART_TypeDef *u = rtl_uart;
	uint32_t isr, head, tail, n = 0u, got = 0u;
	uint32_t t0 = DWT->CYCCNT, dt;

	if (u == NULL)
		return;
	isr = u->ISR;
	/* Count the hardware receive errors BEFORE clearing them: ORE means the USART's
	 * own FIFO overran and a byte never reached software, which our ring's drop
	 * counter cannot see; FE/NE flag a wrong or marginal baud rate. */
	if (isr & USART_ISR_ORE)
		rtl_uart_ore++;
	if (isr & (USART_ISR_FE | USART_ISR_NE))
		rtl_uart_ferr++;
	/* Clear sticky error/idle flags so they do not wedge the FIFO.  IDLE must be
	 * cleared or IDLEIE re-fires forever; RM0468 sec 53.8 says it cannot set again
	 * until RXFNE has set, so clearing it before the drain cannot lose a burst. */
	if (isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_IDLE))
		u->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF |
		         USART_ICR_IDLECF;

	/* Snapshot head/tail once and publish head once at the end: at 6 Mbaud a __DMB()
	 * per byte would cost more than the drain itself.  The consumer only ever
	 * ADVANCES tail, so a stale snapshot can make the ring look full when it is not
	 * -- re-read tail before believing it, never the other way round. */
	head = rtl_ring_head;
	tail = rtl_ring_tail;
	while (u->ISR & USART_ISR_RXNE_RXFNE) {
		uint8_t b = (uint8_t)(u->RDR & 0xFFu);   /* read clears RXFNE for this byte */
		uint32_t next = (head + 1u) & RTL_RING_MASK;

		got++;                                    /* pulled from the FIFO either way */
		if (next == tail) {
			tail = rtl_ring_tail;             /* the consumer may have moved on */
			if (next == tail) {               /* full: drop newest, keep SPSC */
				rtl_ring_drops++;
				continue;
			}
		}
		rtl_ring[head] = b;
		head = next;
		n++;
	}
	if (n != 0u) {
		void (*notify)(void) = rtl_rx_notify;

		__DMB();                                  /* data stores before head publish */
		rtl_ring_head = head;
		/* AFTER the publish: the woken reader must see the bytes this notification
		 * is announcing.  Inside the DWT window below on purpose -- isr_max_cycles
		 * must report what the ISR really costs now, including this. */
		if (notify != NULL)
			notify();
	}
	/* High-water of what the FIFO had accumulated by the time we got here -- count
	 * every byte pulled from RDR, including any the ring had to drop, because it is
	 * the FIFO occupancy that measures the consumed interrupt-latency grace. */
	if (got > rtl_isr_max_bytes)
		rtl_isr_max_bytes = got;

	/* Drain cost, for condition (b) in the file header.  (An inter-entry gap was
	 * tried and dropped: between bursts it just measures how long the module stayed
	 * quiet -- 85 ms across a scan -- and says nothing about latency, which
	 * isr_max_bytes already captures exactly.) */
	rtl_isr_count++;
	dt = DWT->CYCCNT - t0;
	if (dt > rtl_isr_max_cycles)
		rtl_isr_max_cycles = dt;
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
void rtl8720_uart_set_rx_notify(void (*cb)(void))
{
	rtl_rx_notify = cb;
}

void rtl8720_uart_close(void)
{
	if (rtl_uart == NULL)
		return;
	/* Drop the callback BEFORE the interrupt goes away, so no ISR already in flight
	 * can call into a session that is being torn down. */
	rtl_rx_notify = NULL;
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
	/* A fresh open owes nothing to the previous session: whoever wants notifications
	 * re-arms them afterwards (app/erpc.c does, from erpc_link_opened()). */
	rtl_rx_notify = NULL;

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
	 * CR2/CR3 -> CR1(FIFOEN|RE|TE|IDLEIE|UE) -> wait RE/TE ack.
	 * FIFOEN and OVER8 require UE=0 (sec 53.8.1); RXFTIE/RXFTCFG carry no such
	 * restriction (sec 53.8.4) but are programmed here anyway, before UE. */
	NVIC_DisableIRQ(irqn);
	u->CR1 = 0u;
	u->CR2 = 0u;
	u->CR3 = USART_CR3_RXFTIE | RTL_RXFTCFG;   /* threshold IRQ instead of per-byte */
	u->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_IDLECF;
	u->RQR = USART_RQR_RXFRQ;            /* flush any stale RX FIFO content */
	u->PRESC = 0u;
	u->BRR = brr;                       /* OVER8=0: BRR=USARTDIV, rounded (validated above) */
	/* IDLEIE, not RXNEIE: the threshold interrupt carries the bulk and IDLE picks up
	 * the tail of a burst that stopped below it. */
	u->CR1 = USART_CR1_FIFOEN | USART_CR1_RE | USART_CR1_TE |
	         USART_CR1_IDLEIE | USART_CR1_UE;

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

	/* Priority 5 = ABOVE OTG_HS (6), so a dwc2 interrupt cannot eat the RXFIFO
	 * grace (see the interrupt-scheme note in the file header).  ThreadX PRIMASK
	 * critical sections mask this regardless of priority. */
	NVIC_SetPriority(irqn, 5);
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
