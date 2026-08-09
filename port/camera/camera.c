/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    camera.c
 * @brief   OV2640 over DCMI: XCLK (TIM5), SCCB (I2C4), sensor ID, snapshot and
 *          continuous capture (issue #8).  See camera.h for the API contract and
 *          the two board facts that shape it (host-generated XCLK, ungated
 *          camera supply).
 *
 * Hardware setup, all cross-checked between the board schematic, ST's published
 * pin data for this exact package (_ref/stm32_open_pin_data/STM32H725AEIx.xml --
 * the UFBGA169 alternate-function map is NOT the H743's) and the factory Arduino
 * firmware left in the board's external flash, whose HAL_DCMI_MspInit programs
 * the same pins at AF13:
 *
 *   - XCLK: PA2 = TIM5_CH3, AF2.  PWM mode 1, PSC = 0, the duty pinned to
 *     ARR+1 over 2.  The kernel clock is rcc_tim5_ker_ck: with RCC_CFGR.TIMPRE
 *     at its reset 0 and D2PPRE1 = /2 (set by the bootloader), RM0468 sec 8.7.6 /
 *     8.7.8 make it 2 x rcc_pclk1 = 275 MHz.  cam_tim_ker_hz() derives that at
 *     run time from the HAL rather than hardcoding it, so a bootloader that ever
 *     changes the tree cannot silently skew XCLK.
 *
 *     275 MHz cannot produce exactly 24 MHz.  The default divider is 12 ->
 *     22.917 MHz, which keeps an exact 50% duty; 11 would give 25.0 MHz but a
 *     45.5% duty.  Both are inside the 6..27 MHz an OV2640 accepts.
 *
 *   - SCCB: I2C4, PF14 = SCL / PF15 = SDA, AF4, open-drain.  The board carries
 *     4.7k pull-ups (R33/R34), which the sensor answered on immediately, so the
 *     MCU's internal ones stay off (CAM_I2C_INT_PULLUP).
 *     I2C4 lives in D3 and is muxed by RCC_D3CCIPR.I2C4SEL, whose RESET value
 *     selects rcc_pclk4 = 137.5 MHz.  The __HAL_RCC_I2C4_CONFIG below re-states
 *     that default rather than changing anything -- the same idiom, and the same
 *     justification, as port/sd/sd_card.c's __HAL_RCC_SDMMC_CONFIG.  No PLL, no
 *     divider and no FLASH_ACR is written anywhere in this file (CLAUDE.md: the
 *     app inherits the bootloader's clock tree and never reconfigures it).
 *
 *   - PWDN = PE7 and RESETB = PH12, plain GPIO outputs.  Both are driven to
 *     their asserted level while still inputs, so switching them to outputs
 *     cannot glitch the module awake.
 *
 * Why the SCCB reads are two transactions: OmniVision's SCCB is I2C-like but
 * documents a read as "write the sub-address and STOP, then a separate read",
 * not as a repeated START.  HAL_I2C_Mem_Read emits a repeated START, which most
 * parts tolerate but not all.  Both forms were selectable during bring-up
 * because "the address ACKs but every read returns 0xFF" is otherwise an
 * unfalsifiable dead end; the split form worked on the first try and is now the
 * only one (CAM_SCCB_READ_SPLIT).
 *
 * Two shapes of continuous capture share all of that hardware and differ only in
 * where the DMA puts the pixels:
 *
 *   - FRAME mode (issue #8 phase 3b, camera_stream_start): whole 150 KB frames
 *     into a ring of PSRAM slots, published through svc/frame_pipeline.  The DMA's
 *     memory registers are repointed on the fly as slots recycle.
 *   - BAND mode (issue #35, camera_band_start): 60-row slices into two FIXED
 *     buffers in AXI-SRAM, pushed straight to one consumer.  Nothing is repointed
 *     and nothing lands in the PSRAM, which is what makes the display's rotating
 *     blit cheap and takes the DCMI off OCTOSPI1 entirely.
 *
 * Concurrency: one TX_MUTEX serializes every public call for the whole
 * operation; the work lives in *_locked() helpers so a public entry never
 * re-takes the mutex it already holds.  No interrupt is enabled by this phase.
 *
 * Clean-room implementation; RM0468, the schematic and the OmniVision SCCB
 * description used as a register / wiring reference only.
 */
#include "camera.h"

#include "frame_pipeline.h"
#include "ov2640_regs.h"
#include "stm32h7xx_hal.h"
#include "timebase.h"   /* udelay: DWT-based, used by the bit-banged bus clear */
#include "tx_api.h"
#include "tx_glue.h"    /* tx_glue_isr_enter/exit: EPK (issue #2) accounting  */

#define LOG_TAG "cam"
#include "log.h"

#include <string.h>
#include "mem_sections.h"  /* DTCM_BSS: CPU-only data out of AXI-SRAM (issue #46) */

/* ---- pins ---------------------------------------------------------------- */

#define CAM_XCLK_PORT     GPIOA
#define CAM_XCLK_PIN      GPIO_PIN_2
#define CAM_XCLK_AF       GPIO_AF2_TIM5

#define CAM_PWDN_PORT     GPIOE
#define CAM_PWDN_PIN      GPIO_PIN_7

#define CAM_RST_PORT      GPIOH
#define CAM_RST_PIN       GPIO_PIN_12

#define CAM_SCL_PORT      GPIOF
#define CAM_SCL_PIN       GPIO_PIN_14
#define CAM_SDA_PORT      GPIOF
#define CAM_SDA_PIN       GPIO_PIN_15
#define CAM_I2C_AF        GPIO_AF4_I2C4

/*
 * DCMI, 8-bit parallel (issue #8 phase 2).  All eleven lines are AF13 on this
 * UFBGA169 package -- checked against the schematic, against ST's published pin
 * data for this exact part, and against the factory Arduino firmware's
 * HAL_DCMI_MspInit, which programs the same set.  Grouped by port because
 * HAL_GPIO_Init takes a pin mask.
 */
#define CAM_DCMI_AF       GPIO_AF13_DCMI
#define CAM_DCMI_A_PINS   (GPIO_PIN_4 | GPIO_PIN_6 | GPIO_PIN_9)  /* HSYNC PIXCLK D0 */
#define CAM_DCMI_D_PINS   (GPIO_PIN_3)                            /* D5              */
#define CAM_DCMI_E_PINS   (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | \
                           GPIO_PIN_5 | GPIO_PIN_6)               /* D2 D3 D4 D6 D7  */
#define CAM_DCMI_G_PINS   (GPIO_PIN_9)                            /* VSYNC           */
#define CAM_DCMI_H_PINS   (GPIO_PIN_10)                           /* D1              */

/* ---- XCLK ---------------------------------------------------------------- */

#define CAM_TIM           TIM5
/* 275 MHz / 12 = 22.917 MHz at an exact 50% duty.  `camera xclk <hz>` used to
   pick this divider at run time (bounds 6 -> 45.8 MHz, 69 -> 3.99 MHz, which
   bracket every DVP sensor's XVCLK window); the OV2640 was happy here from the
   first probe, so phase 3c-2 fixed it. */
#define CAM_XCLK_DIV_DEF  12u

/* ---- SCCB ---------------------------------------------------------------- */

#define CAM_I2C           I2C4
#define CAM_I2C_TIMEOUT   100u   /* ms per HAL_I2C transfer */

/*
 * TIMINGR values for a 137.5 MHz kernel clock, PRESC = 15 (tPRESC = 116.36 ns):
 *
 *   fields: PRESC[31:28] SCLDEL[23:20] SDADEL[19:16] SCLH[15:8] SCLL[7:0]
 *
 *   100 kHz: SCLL=40, SCLH=44 -> tLOW 4.77 us + tHIGH 5.24 us = 99.93 kHz
 *
 * 50 kHz (0xF0425951) and 400 kHz (0xF031080C) were derived the same way and
 * selectable through `camera tune i2c` during bring-up; the sensor was happy at
 * 100 kHz from the first probe, so only that one survives.  The values are
 * recorded here in case a future module needs them.
 *
 * Those are the register-derived periods; the bus runs slightly slower because
 * the peripheral adds its own synchronisation delay to each edge.
 *
 * cam_i2c_init_locked() re-checks the kernel clock at run time and warns if it
 * is not 137.5 MHz, because these constants would then be silently wrong.
 */
#define CAM_I2C_KER_EXPECTED  137500000u
#define CAM_I2C_TIMING_100K   0xF0422C28u

/* Power-up delays (ms).  OmniVision's module application notes ask for ~10 ms
   between applying XCLK and releasing PWDN, and again before releasing RESETB;
   the final settle covers the sensor's internal reset before SCCB is usable.
   Generous on purpose: probe is a human-typed command, so tens of milliseconds
   cost nothing, whereas being short here produces "SCCB does not answer", the
   hardest symptom in this whole bring-up to attribute. */
#define CAM_T_XCLK_TO_PWDN  10u
#define CAM_T_PWDN_TO_RST   10u
#define CAM_T_RST_TO_SCCB   20u

/* ---- sensor identification ----------------------------------------------- */

/* OV2640: 7-bit SCCB address 0x30, 8-bit registers, banked.  Register 0xFF
   (RA_DLMT) selects the bank; bank 1 holds the ID registers. */
#define OV2640_ADDR        0x30u
#define OV2640_REG_BANK    0xFFu
#define OV2640_BANK_SENSOR 0x01u
#define OV2640_REG_PIDH    0x0Au
#define OV2640_REG_PIDL    0x0Bu
#define OV2640_REG_MIDH    0x1Cu
#define OV2640_REG_MIDL    0x1Du
#define OV2640_PIDH        0x26u
#define OV2640_VER_A       0x41u
#define OV2640_VER_B       0x42u
#define OV2640_MID         0x7FA2u

/* OV5640: 7-bit address 0x3C, 16-bit registers, chip ID 0x5640 at 0x300A. */
#define OV5640_ADDR        0x3Cu
#define OV5640_REG_IDH     0x300Au
#define OV5640_REG_IDL     0x300Bu
#define OV5640_ID          0x5640u

/* OV2640 registers this driver writes outside the vendored table. */
#define OV2640_REG_COM7        0x12u   /* sensor bank */
#define OV2640_COM7_COLORBAR   0x02u
#define OV2640_REG_IMAGE_MODE  0xDAu   /* DSP bank    */
#define OV2640_IMAGE_MODE_RGB565 0x08u
#define OV2640_IMAGE_MODE_SWAP   0x01u

/*
 * REG04, sensor bank: readout orientation.  ST's table ships 0xA8, which is the
 * 0x28 base (VREF/HREF enables) with bit 7 -- horizontal mirror -- SET, so the
 * live image comes out left-right reversed.  Split into base + bit so the value
 * in ov2640_apply_orient_locked() reads as an intent rather than a magic number.
 */
#define OV2640_REG_REG04       0x04u   /* sensor bank */
#define OV2640_REG04_BASE      0x28u
#define OV2640_REG04_HMIRROR   0x80u
#define OV2640_REG04_VFLIP     0x40u
#define OV2640_BANK_DSP    0x00u

/* ---- capture ------------------------------------------------------------- */

/* One frame's DMA budget in 32-bit words.  38400 < 65535, so HAL_DCMI_Start_DMA
   programs a single stream transfer -- the intra-frame banding (DBM) path that
   a >64 K-word frame would take, and the DMA FIFO-error handling it drags in,
   is NOT exercised here.  Adding VGA in a later phase changes that. */
#define CAM_FRAME_WORDS   (CAMERA_FRAME_BYTES / 4u)

/* A frame is ~1/15 s; a second is "the sensor is not producing frames at all". */
#define CAM_XFER_TIMEOUT_TICKS  1000u

/* Frames thrown away before the one we keep live in camera.h (the shell says how
   long a capture will take).  15 is not a guess -- see camera_capture_locked(). */

/* Streaming producer thread.  Priority 10 sits below the IWDG petter (5) and the
   RTL8720 UART (which has a hard byte deadline); it only has to keep up with the
   frame rate.  1024 B is ~2x the 508 B high-water the same producer measured on
   the f746 firmware -- it is a semaphore-wait loop, not a deep call chain. */
#define CAM_PRODUCER_PRIO   10u
#define CAM_PRODUCER_STACK  1024u
/* Bounded wait so --frames/--secs and `stream stop` still fire when no frame is
   arriving at all (sync lost), ms. */
#define CAM_PRODUCER_TICK   10u

/* Settle after the register sequence before the first frame is trusted. */
#define CAM_SETTLE_CONFIG_MS    100u
/* ST's driver spaces the sequence out: 200 ms after the COM7 soft reset and
   1 ms between rows.  Kept as-is -- shortening it turns a failure into "is the
   table wrong or is the pacing wrong?", which is the expensive question. */
#define CAM_T_SOFT_RESET_MS     200u

/* ---- state --------------------------------------------------------------- */

static TX_MUTEX cam_lock;
static TX_SEMAPHORE cam_done;
static I2C_HandleTypeDef hcam_i2c;
static DCMI_HandleTypeDef hdcmi;
static DMA_HandleTypeDef hdma_dcmi;

/*
 * The frame buffer.  AXI-SRAM has ~113 KB free and a QVGA RGB565 frame is 150 KB,
 * so the PSRAM is not a preference here, it is the only place it fits.
 *
 * CACHE CONTRACT: app/mpu.c maps the whole OCTOSPI1 window Normal non-cacheable,
 * so the CPU and the DCMI's DMA see the same bytes with no clean/invalidate.
 * That holds ONLY while this buffer stays in .psram_noinit -- move it to
 * AXI-SRAM and every read below needs cache maintenance (and the .axi_dma
 * treatment port/sd/sd_card.c uses for its bounce buffer).
 */
static uint8_t cam_frame[CAMERA_FRAME_BYTES]
	__attribute__((aligned(32), section(".psram_noinit.camera")));

static volatile int cam_xfer_active;   /* a capture owns the DCMI right now   */
static volatile int cam_xfer_err;      /* set by HAL_DCMI_ErrorCallback       */
static uint32_t cam_frame_gen;         /* bumped on every successful capture  */
static int cam_colorbar = -1;          /* live/colorbar state, -1 = unknown   */

/* ---- streaming state ----------------------------------------------------- */

/*
 * The ring the DMA writes into while streaming.  Separate from cam_frame: the
 * snapshot buffer stays a stable thing the shell can save/send from, and the
 * teardown copies the last streamed frame into it so those commands keep working
 * after a stream.
 *
 * Four slots is the smallest comfortable ring: the DMA's double buffer always
 * owns two of them, one holds the latest published frame (frame_pipeline_acquire
 * refuses to recycle that one), and one is free to hand the producer.  Three
 * would technically run but would drop a frame every time a sink held a pin.
 */
#define CAM_RING_SLOTS  4u

static uint8_t cam_ring[CAM_RING_SLOTS][CAMERA_FRAME_BYTES]
	__attribute__((aligned(32), section(".psram_noinit.camera")));

/*
 * The two band buffers of the staged stream (issue #35).
 *
 * 🔴 THIS IS THE SECOND BUFFER IN THIS FIRMWARE THAT A BUS MASTER WRITES AND THE
 * CPU READS -- port/sd/sd_card.c's sd_bounce was the first, and this follows it
 * exactly:
 *
 *   - AXI-SRAM (.axi_dma), because DMA2 cannot see either TCM at all (RM0468 sec
 *     2.1.2 / 2.1.5 / 2.1.6).  In DTCM the transfer would not fault, it would
 *     simply move nothing -- see include/mem_sections.h.  cmake/check_dtcm_
 *     residency.py holds the line; cam_band is in its REQUIRED_AXI list.
 *   - Its own 32 B-aligned section at both ends, so no neighbouring variable
 *     shares a cache line with it and an invalidate here cannot discard someone
 *     else's dirty data.
 *   - Explicit maintenance: the driver invalidates both bands before arming and
 *     the finished band before reading it, and the CPU never writes either.
 *     CAMERA_BAND_BYTES is a multiple of 32, so band 1 starts on a line boundary
 *     and maintaining one band cannot reach the other.
 *
 * 76,800 B is the whole cost of issue #35 in the scarce memory, and it was
 * budgeted in issue #46: AXI-SRAM had 193,664 B free, leaving ~117 KB for the
 * on-device AI work in issue #9.
 */
static uint8_t cam_band[2][CAMERA_BAND_BYTES]
	__attribute__((aligned(32), section(".axi_dma.cam_band")));

/* Everything about band mode rests on these three, so state them where they can
   fail the build rather than the picture. */
_Static_assert(CAMERA_FRAME_HEIGHT % CAMERA_BAND_ROWS == 0u,
               "band rows must divide the frame, or bands drift across frames");
_Static_assert(CAMERA_BAND_BYTES % 32u == 0u,
               "a band must be whole cache lines, or maintaining one touches the other");
_Static_assert(CAMERA_BAND_BYTES / 4u <= 0xFFFFu,
               "a band must fit one DMA transfer (NDTR is 16-bit)");

static TX_MUTEX     cam_pipe_lock;      /* the pipeline's injected mutex       */
static TX_SEMAPHORE cam_stream_sem;     /* DMA TC ISR -> producer              */
static TX_SEMAPHORE cam_start_sem;      /* start -> producer idle wakeup       */
static TX_THREAD    cam_producer;
static UCHAR        cam_producer_stack[CAM_PRODUCER_STACK] DTCM_BSS
                        __attribute__((aligned(8)));

static struct frame_pipeline cam_pipe;
static struct frame_sink     cam_stat_sink;

/* The two slots the DMA's M0AR/M1AR currently point at. */
static struct frame_desc *cam_m0;
static struct frame_desc *cam_m1;
static uint32_t cam_last_ct;

/* ---- band-mode state (issue #35) ----------------------------------------- */

/*
 * Band mode needs no ring, no pipeline and no repoint.  What it does need is a
 * band INDEX, and the honest source for that is a counter kept by the DMA's own
 * completion ISR: cam_band_seq counts transfers since the stream armed, so the
 * band that finished with transfer number s is s-1, its index in the frame is
 * (s-1) % CAMERA_BANDS_PER_FRAME and the buffer holding it is (s-1) % 2.
 *
 * Deriving the buffer from the sequence rather than re-reading the DMA's CT bit
 * per completion is what makes the pairing race-free: CT and the counter are
 * updated by different agents (hardware / this ISR) and a thread that sampled them
 * separately could catch them disagreeing.  CT is read exactly once, right after
 * the stream arms, into cam_band_ct0 -- because HAL_DMAEx_MultiBufferStart_IT
 * leaves CT wherever the previous run put it, which is the same trap the frame
 * path documents at its own cam_last_ct seeding.
 */
static volatile uint32_t cam_band_seq;       /* DMA transfers completed          */
static uint32_t cam_band_next;               /* sequence number expected next    */
static int      cam_band_sync;               /* mid-frame delivery is coherent   */
static camera_band_fn cam_band_cb;
static void          *cam_band_ctx;
static uint32_t  cam_band_ct0;               /* buffer the first transfer fills  */
static uint32_t  cam_band_frames;            /* frames delivered whole           */
static uint32_t  cam_band_late;              /* bands the consumer never saw     */
static uint32_t  cam_band_torn;              /* bands the DMA overtook mid-read  */
static volatile uint32_t cam_band_desync;    /* frame end off a band boundary    */
static volatile int      cam_band_mode;      /* the running stream is band mode  */
static uint8_t   cam_last_mode = CAM_STREAM_NONE;  /* survives the stop, for `stats` */

static volatile int      cam_stream_active;  /* owns the DCMI and the PSRAM ring */
static volatile int      cam_stream_err;     /* terminal: TE / DCMI OVR          */
static volatile int      cam_stop_req;
static volatile uint32_t cam_stream_fe;      /* tolerated FIFO/direct-mode errors */
static volatile uint32_t cam_dcmi_ovr;
static uint32_t cam_ring_ovr;
static uint32_t cam_repoint_skip;
static uint32_t cam_start_tick;
static uint32_t cam_elapsed_ms;              /* frozen at teardown for `stats`   */
static uint32_t cam_target_frames;
static uint32_t cam_target_secs;

/* Slots pinned by camera_stream_pin_latest() outside this driver (the preview).
   A stream start waits for this to reach zero before it re-initialises the
   pipeline -- see cam_stream_start_locked(). */
static volatile uint32_t cam_ext_pins;
#define CAM_EXT_PIN_DRAIN_MS  200u

static struct camera_info info;
/*
 * These six were run-time knobs through phases 1-2, because none of them could
 * be proven from the schematic and reflashing to sweep an unknown eats the
 * internal flash's ~10k erase cycles.  The board answered all of them on the
 * first try and has not contradicted itself since, so they are constants now --
 * a knob that has served its purpose is a liability, not a feature.  The values
 * and how they were established are in the file header and in issue #8.
 */
#define CAM_PWDN_ACTIVE_HIGH  1     /* PWDN high powers the sensor down          */
#define CAM_RST_ACTIVE_LOW    1     /* RESETB low holds it in reset              */
#define CAM_I2C_INT_PULLUP    0     /* the board's 4.7k pull-ups are enough      */
#define CAM_SCCB_READ_SPLIT   1     /* OmniVision's two-transaction read         */
#define CAM_DVP_BYTE_SWAP     1     /* IMAGE_MODE bit0, as ST's table ships it   */
/*
 * Readout orientation (issue #53).  ST's table ships the horizontal mirror ON,
 * which puts the live image out left-right reversed; this board wants it off.
 * Stated here, next to the byte-swap decision, because both are the same kind of
 * choice: what is "right" depends on what looks at the frame, not on the sensor.
 */
#define CAM_SENSOR_HMIRROR    0
#define CAM_SENSOR_VFLIP      0
#define CAM_I2C_TIMING        CAM_I2C_TIMING_100K

/* ---- locking ------------------------------------------------------------- */

static int op_lock(void)
{
	if (!info.ready)
		return CAM_ERR_STATE;
	if (tx_mutex_get(&cam_lock, TX_WAIT_FOREVER) != TX_SUCCESS)
		return CAM_ERR_STATE;
	return CAM_OK;
}

static void op_unlock(void)
{
	(void)tx_mutex_put(&cam_lock);
}

/* ---- XCLK ---------------------------------------------------------------- */

/*
 * rcc_tim5_ker_ck.  RM0468 sec 8.7.6 (RCC_CFGR.TIMPRE) / sec 8.7.8 (D2CFGR.D2PPRE1):
 * with an APB prescaler of 1 the timer clock equals PCLK; otherwise it is
 * 2 x PCLK (TIMPRE = 0) or 4 x PCLK (TIMPRE = 1), in both cases capped at HCLK.
 * The bootloader leaves TIMPRE = 0 and D2PPRE1 = /2, so this returns
 * 2 x 137.5 MHz = 275 MHz -- but it is computed, not assumed.
 */
static uint32_t cam_tim_ker_hz(void)
{
	uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
	uint32_t hclk  = HAL_RCC_GetHCLKFreq();
	uint32_t mul   = ((RCC->CFGR & RCC_CFGR_TIMPRE) != 0u) ? 4u : 2u;

	if (pclk1 == hclk)
		return hclk;
	return (pclk1 * mul > hclk) ? hclk : pclk1 * mul;
}

static uint32_t xclk_hz_for(uint32_t div)
{
	return (div != 0u) ? cam_tim_ker_hz() / div : 0u;
}

/*
 * Program the divider, stopping the counter across the update.
 *
 * Only xclk_start_locked() calls this now, where the counter is already stopped,
 * but the sequence is kept because it is what makes the write safe in general:
 * ARR has no preload here (CR1.ARPE = 0), so shrinking it while the counter is
 * above the new value would let a 32-bit CNT run all the way to 0xFFFFFFFF
 * before wrapping -- a ~15 second stuck output level, not a glitch.
 */
static void xclk_apply_div(uint32_t div)
{
	uint32_t cr1 = CAM_TIM->CR1;

	CAM_TIM->CR1  = cr1 & ~TIM_CR1_CEN;
	CAM_TIM->ARR  = div - 1u;
	CAM_TIM->CCR3 = div / 2u;
	CAM_TIM->CNT  = 0u;
	CAM_TIM->EGR  = TIM_EGR_UG;
	CAM_TIM->CR1  = cr1;
}

static void xclk_start_locked(void)
{
	GPIO_InitTypeDef g = {0};

	__HAL_RCC_TIM5_CLK_ENABLE();
	/* Keep the timer clocked in CSleep so idle WFI (issue #2) does not stop the
	   sensor's master clock.  TIM5LPEN resets to 1 (RM0468 sec 8.7.53); setting it
	   explicitly documents the dependency instead of relying on a default. */
	__HAL_RCC_TIM5_CLK_SLEEP_ENABLE();

	CAM_TIM->CR1   = 0u;
	CAM_TIM->PSC   = 0u;
	/* PWM mode 1 on channel 3 with output preload.  TIM5 is a general-purpose
	   timer: there is no BDTR/MOE to release. */
	CAM_TIM->CCMR2 = (6u << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;
	CAM_TIM->CCER  = TIM_CCER_CC3E;
	CAM_TIM->CNT   = 0u;
	xclk_apply_div(CAM_XCLK_DIV_DEF);
	CAM_TIM->CR1   = TIM_CR1_CEN;

	g.Pin       = CAM_XCLK_PIN;
	g.Mode      = GPIO_MODE_AF_PP;
	g.Pull      = GPIO_NOPULL;
	g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
	g.Alternate = CAM_XCLK_AF;
	HAL_GPIO_Init(CAM_XCLK_PORT, &g);

	info.xclk_hz = xclk_hz_for(CAM_XCLK_DIV_DEF);
}

static void xclk_stop_locked(void)
{
	GPIO_InitTypeDef g = {0};

	/* Park the pin as a driven low BEFORE stopping the timer: disabling the
	   channel output would otherwise leave PA2 floating, and a floating clock
	   input on the module is not a defined state. */
	HAL_GPIO_WritePin(CAM_XCLK_PORT, CAM_XCLK_PIN, GPIO_PIN_RESET);
	g.Pin   = CAM_XCLK_PIN;
	g.Mode  = GPIO_MODE_OUTPUT_PP;
	g.Pull  = GPIO_NOPULL;
	g.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(CAM_XCLK_PORT, &g);

	/* A write to a peripheral whose APB gate is off is silently dropped, so
	   check before touching TIM5: camera_off() / camera_probe() both run this
	   path before the timer has ever been started. */
	if (__HAL_RCC_TIM5_IS_CLK_ENABLED()) {
		CAM_TIM->CR1  = 0u;
		CAM_TIM->CCER = 0u;
	}
	info.xclk_hz = 0u;
}

/* ---- power / reset lines ------------------------------------------------- */

static GPIO_PinState pwdn_level(int assert_powerdown)
{
	int high = CAM_PWDN_ACTIVE_HIGH ? assert_powerdown : !assert_powerdown;

	return high ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

static GPIO_PinState rst_level(int assert_reset)
{
	int high = CAM_RST_ACTIVE_LOW ? !assert_reset : assert_reset;

	return high ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

static void power_off_locked(void)
{
	HAL_GPIO_WritePin(CAM_PWDN_PORT, CAM_PWDN_PIN, pwdn_level(1));
	HAL_GPIO_WritePin(CAM_RST_PORT, CAM_RST_PIN, rst_level(1));
	xclk_stop_locked();

	info.powered   = 0u;
	info.sensor    = CAM_SENSOR_NONE;
	info.i2c_addr  = 0u;
	info.reg_width = 0u;
	info.chip_id   = 0u;
	info.manuf_id  = 0u;
	/* The sensor's registers do not survive PWDN/reset, so the loaded sequence
	   is gone and the next capture must re-run it.  The FRAME, however, is still
	   in PSRAM and still readable -- `camera off` then `camera send` is a
	   reasonable thing to do, so frame_valid deliberately stays set. */
	info.configured = 0u;
	cam_colorbar    = -1;
}

static void power_on_locked(void)
{
	xclk_start_locked();
	tx_thread_sleep(CAM_T_XCLK_TO_PWDN);
	HAL_GPIO_WritePin(CAM_PWDN_PORT, CAM_PWDN_PIN, pwdn_level(0));
	tx_thread_sleep(CAM_T_PWDN_TO_RST);
	HAL_GPIO_WritePin(CAM_RST_PORT, CAM_RST_PIN, rst_level(0));
	tx_thread_sleep(CAM_T_RST_TO_SCCB);
	info.powered = 1u;
}

/* ---- SCCB ---------------------------------------------------------------- */

/*
 * The I2C4 kernel clock, resolved from the mux we just wrote.
 *
 * NOT HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_I2C4): that function has no I2C4
 * branch on this family -- it falls through its switch and returns 0.  So the
 * check below has been firing on every boot since phase 1, reporting "kernel
 * clock 0 Hz", and `camera info` printed `kernel 0.0 MHz`.  Nothing was actually
 * wrong; the question was simply unsupported.  A self-check that cannot tell a
 * wrong value from an unanswerable question is worse than no self-check, because
 * it trains you to ignore it.
 *
 * RM0468 s8.7.21 (RCC_D3CCIPR): I2C4SEL selects rcc_pclk4 / pll3_r_ck /
 * hsi_ker_ck / csi_ker_ck.  Only the first is resolvable here without pulling in
 * the PLL3 plumbing, and it is the one this driver sets and the reset default;
 * anything else means someone moved the mux, which is exactly what the warning
 * should shout about, so 0 ("unknown") is the right answer for those.
 */
static uint32_t cam_i2c_kernel_hz(void)
{
	if (__HAL_RCC_GET_I2C4_SOURCE() != RCC_I2C4CLKSOURCE_D3PCLK1)
		return 0u;
	return HAL_RCCEx_GetD3PCLK1Freq();
}

static int cam_i2c_init_locked(void)
{
	GPIO_InitTypeDef g = {0};

	__HAL_RCC_GPIOF_CLK_ENABLE();
	/* Re-state the RESET value of RCC_D3CCIPR.I2C4SEL (rcc_pclk4).  This changes
	   nothing -- it documents which kernel clock the TIMINGR constants above were
	   derived for.  Same idiom as port/sd/sd_card.c. */
	__HAL_RCC_I2C4_CONFIG(RCC_I2C4CLKSOURCE_D3PCLK1);
	__HAL_RCC_I2C4_CLK_ENABLE();

	info.i2c_ker_hz = cam_i2c_kernel_hz();
	if (info.i2c_ker_hz != CAM_I2C_KER_EXPECTED)
		LOG_WRN("I2C4 kernel clock %lu Hz, not %lu -- SCCB bit rate will be off",
		        (unsigned long)info.i2c_ker_hz,
		        (unsigned long)CAM_I2C_KER_EXPECTED);

	g.Pin       = CAM_SCL_PIN | CAM_SDA_PIN;
	g.Mode      = GPIO_MODE_AF_OD;
	g.Pull      = CAM_I2C_INT_PULLUP ? GPIO_PULLUP : GPIO_NOPULL;
	g.Speed     = GPIO_SPEED_FREQ_HIGH;
	g.Alternate = CAM_I2C_AF;
	HAL_GPIO_Init(CAM_SCL_PORT, &g);

	if (hcam_i2c.Instance != NULL)
		(void)HAL_I2C_DeInit(&hcam_i2c);

	hcam_i2c.Instance              = CAM_I2C;
	hcam_i2c.Init.Timing           = CAM_I2C_TIMING;
	hcam_i2c.Init.OwnAddress1      = 0u;
	hcam_i2c.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
	hcam_i2c.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
	hcam_i2c.Init.OwnAddress2      = 0u;
	hcam_i2c.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hcam_i2c.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
	hcam_i2c.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hcam_i2c) != HAL_OK) {
		LOG_ERR("I2C4 init failed");
		return CAM_ERR_HAL;
	}
	return CAM_OK;
}

/*
 * Bit-bang the bus out of a stuck state: a slave interrupted mid-byte holds SDA
 * low forever, and no amount of re-initialising the peripheral makes it let go.
 * Nine SCL pulses walk it to the end of its byte, then a manual STOP (SDA rising
 * while SCL is high) resynchronises it.  The I2C peripheral has to be released
 * from the pins first, so the caller must re-init it afterwards.
 */
#define CAM_I2C_CLEAR_HALF_US 5u   /* ~100 kHz while bit-banging */

static void i2c_bus_clear_locked(void)
{
	GPIO_InitTypeDef g = {0};

	if (hcam_i2c.Instance != NULL) {
		(void)HAL_I2C_DeInit(&hcam_i2c);
		hcam_i2c.Instance = NULL;
	}

	HAL_GPIO_WritePin(CAM_SCL_PORT, CAM_SCL_PIN | CAM_SDA_PIN, GPIO_PIN_SET);
	g.Pin   = CAM_SCL_PIN | CAM_SDA_PIN;
	g.Mode  = GPIO_MODE_OUTPUT_OD;
	g.Pull  = CAM_I2C_INT_PULLUP ? GPIO_PULLUP : GPIO_NOPULL;
	g.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(CAM_SCL_PORT, &g);

	for (unsigned i = 0u; i < 9u; i++) {
		if (HAL_GPIO_ReadPin(CAM_SDA_PORT, CAM_SDA_PIN) == GPIO_PIN_SET)
			break;
		HAL_GPIO_WritePin(CAM_SCL_PORT, CAM_SCL_PIN, GPIO_PIN_RESET);
		udelay(CAM_I2C_CLEAR_HALF_US);
		HAL_GPIO_WritePin(CAM_SCL_PORT, CAM_SCL_PIN, GPIO_PIN_SET);
		udelay(CAM_I2C_CLEAR_HALF_US);
	}
	/* STOP: SDA low with SCL low, raise SCL, then release SDA. */
	HAL_GPIO_WritePin(CAM_SCL_PORT, CAM_SCL_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(CAM_SDA_PORT, CAM_SDA_PIN, GPIO_PIN_RESET);
	udelay(CAM_I2C_CLEAR_HALF_US);
	HAL_GPIO_WritePin(CAM_SCL_PORT, CAM_SCL_PIN, GPIO_PIN_SET);
	udelay(CAM_I2C_CLEAR_HALF_US);
	HAL_GPIO_WritePin(CAM_SDA_PORT, CAM_SDA_PIN, GPIO_PIN_SET);
	udelay(CAM_I2C_CLEAR_HALF_US);
}

/*
 * A stuck-low SDA -- exactly what a wedged sensor or a missing pull-up produces --
 * leaves the peripheral BUSY, after which every transfer fails instantly and the
 * bring-up just looks like "the bus does nothing".  Every entry point that talks
 * to the bus passes through here first.
 *
 * Both conditions are checked: the HAL's software state (which a timed-out
 * transfer leaves non-READY) AND ISR.BUSY, which reflects the wire and can be set
 * while the HAL still believes it is idle -- checking only the former is what
 * makes a stuck bus look unrecoverable.
 */
static void i2c_ensure_ready_locked(void)
{
	int hal_busy = (HAL_I2C_GetState(&hcam_i2c) != HAL_I2C_STATE_READY);
	int wire_busy = (hcam_i2c.Instance != NULL) &&
	                ((hcam_i2c.Instance->ISR & I2C_ISR_BUSY) != 0u);

	if (!hal_busy && !wire_busy)
		return;

	LOG_WRN("I2C4 stuck (state 0x%02x, err 0x%08lx, ISR 0x%08lx) -- clearing bus",
	        (unsigned)HAL_I2C_GetState(&hcam_i2c),
	        (unsigned long)HAL_I2C_GetError(&hcam_i2c),
	        (unsigned long)((hcam_i2c.Instance != NULL) ? hcam_i2c.Instance->ISR
	                                                    : 0u));
	i2c_bus_clear_locked();
	(void)cam_i2c_init_locked();
}

/* One address probe, shared by camera_scan() and the post-probe fallback. */
static int addr_acks_locked(uint8_t addr7)
{
	return HAL_I2C_IsDeviceReady(&hcam_i2c, (uint16_t)(addr7 << 1), 1u, 5u)
	       == HAL_OK;
}

/* Register address bytes, MSB first for the 16-bit form. */
static unsigned reg_bytes(uint16_t reg, unsigned width, uint8_t *buf)
{
	if (width == CAM_REG_WIDTH_16) {
		buf[0] = (uint8_t)(reg >> 8);
		buf[1] = (uint8_t)reg;
		return 2u;
	}
	buf[0] = (uint8_t)reg;
	return 1u;
}

static int sccb_write_locked(uint8_t addr7, uint16_t reg, unsigned width,
                             uint8_t val)
{
	uint8_t buf[3];
	unsigned n = reg_bytes(reg, width, buf);

	buf[n++] = val;
	if (HAL_I2C_Master_Transmit(&hcam_i2c, (uint16_t)(addr7 << 1), buf,
	                            (uint16_t)n, CAM_I2C_TIMEOUT) != HAL_OK)
		return CAM_ERR_HAL;
	return CAM_OK;
}

static int sccb_read_locked(uint8_t addr7, uint16_t reg, unsigned width,
                            uint8_t *val)
{
	uint8_t buf[2];
	unsigned n = reg_bytes(reg, width, buf);

	if (!CAM_SCCB_READ_SPLIT) {
		if (HAL_I2C_Mem_Read(&hcam_i2c, (uint16_t)(addr7 << 1), reg,
		                     (width == CAM_REG_WIDTH_16)
		                             ? I2C_MEMADD_SIZE_16BIT
		                             : I2C_MEMADD_SIZE_8BIT,
		                     val, 1u, CAM_I2C_TIMEOUT) != HAL_OK)
			return CAM_ERR_HAL;
		return CAM_OK;
	}

	/* SCCB two-phase read: sub-address write terminated by a STOP, then an
	   independent read transaction. */
	if (HAL_I2C_Master_Transmit(&hcam_i2c, (uint16_t)(addr7 << 1), buf,
	                            (uint16_t)n, CAM_I2C_TIMEOUT) != HAL_OK)
		return CAM_ERR_HAL;
	if (HAL_I2C_Master_Receive(&hcam_i2c, (uint16_t)(addr7 << 1), val, 1u,
	                           CAM_I2C_TIMEOUT) != HAL_OK)
		return CAM_ERR_HAL;
	return CAM_OK;
}

/* ---- identification ------------------------------------------------------ */

static int ident_ov2640_locked(void)
{
	uint8_t pidh, pidl, midh, midl;

	if (sccb_write_locked(OV2640_ADDR, OV2640_REG_BANK, CAM_REG_WIDTH_8,
	                      OV2640_BANK_SENSOR) != CAM_OK)
		return CAM_ERR_NO_SENSOR;
	if (sccb_read_locked(OV2640_ADDR, OV2640_REG_PIDH, CAM_REG_WIDTH_8, &pidh) != CAM_OK ||
	    sccb_read_locked(OV2640_ADDR, OV2640_REG_PIDL, CAM_REG_WIDTH_8, &pidl) != CAM_OK ||
	    sccb_read_locked(OV2640_ADDR, OV2640_REG_MIDH, CAM_REG_WIDTH_8, &midh) != CAM_OK ||
	    sccb_read_locked(OV2640_ADDR, OV2640_REG_MIDL, CAM_REG_WIDTH_8, &midl) != CAM_OK)
		return CAM_ERR_NO_SENSOR;

	/*
	 * PIDH alone would let any device that happens to answer 0x26 at register
	 * 0x0A pass as an OV2640, so a second, independent field has to agree: either
	 * the OmniVision manufacturer ID or one of the two documented version bytes.
	 * Requiring BOTH would be stricter than the parts warrant -- VER is 0x41 on
	 * some lots and 0x42 on others, and clones are inconsistent about MID.
	 */
	if (pidh != OV2640_PIDH ||
	    (((uint16_t)midh << 8 | midl) != OV2640_MID &&
	     pidl != OV2640_VER_A && pidl != OV2640_VER_B)) {
		LOG_WRN("0x%02x answered but PID 0x%02x%02x / MID 0x%02x%02x is not OV2640",
		        OV2640_ADDR, pidh, pidl, midh, midl);
		return CAM_ERR_NO_SENSOR;
	}
	info.sensor    = CAM_SENSOR_OV2640;
	info.i2c_addr  = OV2640_ADDR;
	info.reg_width = CAM_REG_WIDTH_8;
	info.chip_id   = (uint16_t)((uint16_t)pidh << 8 | pidl);
	info.manuf_id  = (uint16_t)((uint16_t)midh << 8 | midl);
	if (info.manuf_id != OV2640_MID)
		LOG_WRN("OV2640 PID matched but MID is 0x%04x, not 0x%04x",
		        info.manuf_id, OV2640_MID);
	if (pidl != OV2640_VER_A && pidl != OV2640_VER_B)
		LOG_WRN("OV2640 MID matched but VER is 0x%02x, not 0x%02x/0x%02x",
		        pidl, OV2640_VER_A, OV2640_VER_B);
	return CAM_OK;
}

static int ident_ov5640_locked(void)
{
	uint8_t idh, idl;
	uint16_t id;

	if (sccb_read_locked(OV5640_ADDR, OV5640_REG_IDH, CAM_REG_WIDTH_16, &idh) != CAM_OK ||
	    sccb_read_locked(OV5640_ADDR, OV5640_REG_IDL, CAM_REG_WIDTH_16, &idl) != CAM_OK)
		return CAM_ERR_NO_SENSOR;

	id = (uint16_t)((uint16_t)idh << 8 | idl);
	if (id != OV5640_ID) {
		LOG_WRN("0x%02x answered but chip ID 0x%04x is not OV5640",
		        OV5640_ADDR, id);
		return CAM_ERR_NO_SENSOR;
	}
	info.sensor    = CAM_SENSOR_OV5640;
	info.i2c_addr  = OV5640_ADDR;
	info.reg_width = CAM_REG_WIDTH_16;
	info.chip_id   = id;
	info.manuf_id  = 0u;
	return CAM_OK;
}

/* Any device on the bus, so an unidentified module still reports *something*
   actionable rather than a bare "no sensor". */
static uint8_t first_acking_addr_locked(void)
{
	for (uint8_t a = 0x08u; a <= 0x77u; a++) {
		if (addr_acks_locked(a))
			return a;
	}
	return 0u;
}

/* Power-cycle and identify.  Split out of camera_probe() so camera_capture()
   can bring an unpowered sensor up without releasing and re-taking the lock. */
static int camera_probe_locked(void)
{
	int rc;

	power_off_locked();
	power_on_locked();
	i2c_ensure_ready_locked();

	rc = ident_ov2640_locked();
	if (rc != CAM_OK)
		rc = ident_ov5640_locked();

	if (rc != CAM_OK) {
		/* Leave the sensor powered: the supply cannot be cut anyway, and a live
		   bus is what lets `camera scan` / `camera reg` continue the hunt. */
		uint8_t a = first_acking_addr_locked();

		info.sensor    = (a != 0u) ? CAM_SENSOR_UNKNOWN : CAM_SENSOR_NONE;
		info.i2c_addr  = a;
		info.reg_width = 0u;
		info.chip_id   = 0u;
		info.manuf_id  = 0u;
		if (a != 0u)
			LOG_WRN("unidentified device at SCCB 0x%02x", a);
		else
			LOG_WRN("no SCCB response (module connected? XCLK %lu Hz)",
			        (unsigned long)info.xclk_hz);
		return CAM_ERR_NO_SENSOR;
	}

	LOG_INF("%s up: chip ID 0x%04x at SCCB 0x%02x",
	        camera_sensor_name(info.sensor), info.chip_id, info.i2c_addr);
	return CAM_OK;
}

/* ---- sensor configuration (OV2640 QVGA RGB565) --------------------------- */

/*
 * Walk the vendored ST table.  ST's own driver returns void, so a sensor that
 * NAKs half way through leaves it happily reporting success and produces a
 * garbage frame later; going through sccb_write_locked() means a bad row stops
 * the sequence at the row that failed, and says which one.
 */
static int ov2640_write_table_locked(void)
{
	for (unsigned i = 0u; i < ov2640_qvga_rgb565_len; i++) {
		if (sccb_write_locked(OV2640_ADDR, ov2640_qvga_rgb565[i][0],
		                      CAM_REG_WIDTH_8,
		                      ov2640_qvga_rgb565[i][1]) != CAM_OK) {
			LOG_ERR("QVGA sequence failed at row %u (reg 0x%02x)", i,
			        ov2640_qvga_rgb565[i][0]);
			return CAM_ERR_HAL;
		}
		tx_thread_sleep(1u);   /* ST paces the sequence at 1 ms per row */
	}
	return CAM_OK;
}

/*
 * Re-write IMAGE_MODE after the table so the DVP byte-swap bit is set from
 * CAM_DVP_BYTE_SWAP rather than buried in the vendored sequence.  The table
 * itself keeps ST's shipped 0x09; this is the one register we are prepared to
 * disagree with them about, because which order is "right" depends on what reads
 * the frame, not on the sensor -- so it is stated where it can be found.
 */
static int ov2640_apply_swap_locked(void)
{
	uint8_t v = OV2640_IMAGE_MODE_RGB565 |
	            (CAM_DVP_BYTE_SWAP ? OV2640_IMAGE_MODE_SWAP : 0u);

	if (sccb_write_locked(OV2640_ADDR, OV2640_REG_BANK, CAM_REG_WIDTH_8,
	                      OV2640_BANK_DSP) != CAM_OK ||
	    sccb_write_locked(OV2640_ADDR, OV2640_REG_IMAGE_MODE, CAM_REG_WIDTH_8,
	                      v) != CAM_OK)
		return CAM_ERR_HAL;
	return CAM_OK;
}

/*
 * Re-write REG04 after the table so the readout orientation comes from
 * CAM_SENSOR_HMIRROR / CAM_SENSOR_VFLIP rather than from ST's 0xA8.  Exactly the
 * same treatment as the byte swap above, and for the same reason: the table stays
 * byte-identical to upstream because "known-good" is the only property it has
 * that we cannot re-derive, so every register we disagree with them about is
 * re-written here where it can be found.
 *
 * Verify with `camera reg` (read it back) AND with the picture -- an orientation
 * bug shows up in no counter anywhere, which is the recurring lesson of issues
 * #7, #43 and #8 phase 3a.
 *
 * 🔴 THE READ-BACK DOES NOT EQUAL WHAT WAS WRITTEN, and that is not a fault.
 * REG04 bits [1:0] are AEC[1:0] -- the bottom two bits of the automatic exposure
 * value -- and the sensor's own AEC loop rewrites them continuously while it is
 * streaming.  Measured on board #2: this writes 0x28 and `camera reg 0x30 0x04`
 * then reads 0x2A.  Check bit 7, not the whole byte.  (Zeroing AEC[1:0] here is
 * harmless and is exactly what ST's table does with its 0xA8: it happens once at
 * configure time, before streaming, and the AEC overwrites it immediately.)
 *
 * The bank matters for that read: this leaves SENSOR selected, and so does
 * ov2640_set_colorbar_locked() which runs after it, so a bare `camera reg` after
 * configure lands in the right bank.  Before the table has been applied (`camera
 * on` alone, sensor cfg "not loaded") the read returns the power-on default and
 * says nothing about this function.
 */
static int ov2640_apply_orient_locked(void)
{
	uint8_t v = OV2640_REG04_BASE |
	            (CAM_SENSOR_HMIRROR ? OV2640_REG04_HMIRROR : 0u) |
	            (CAM_SENSOR_VFLIP ? OV2640_REG04_VFLIP : 0u);

	if (sccb_write_locked(OV2640_ADDR, OV2640_REG_BANK, CAM_REG_WIDTH_8,
	                      OV2640_BANK_SENSOR) != CAM_OK ||
	    sccb_write_locked(OV2640_ADDR, OV2640_REG_REG04, CAM_REG_WIDTH_8,
	                      v) != CAM_OK)
		return CAM_ERR_HAL;
	return CAM_OK;
}

/* COM7 bit1 swaps the live image for the sensor's internal colour-bar pattern.
   Worth having: it exercises DVP -> DCMI -> DMA -> PSRAM with a signal that owes
   nothing to the lens, the light or the exposure loop. */
static int ov2640_set_colorbar_locked(int on)
{
	uint8_t com7;

	if (sccb_write_locked(OV2640_ADDR, OV2640_REG_BANK, CAM_REG_WIDTH_8,
	                      OV2640_BANK_SENSOR) != CAM_OK)
		return CAM_ERR_HAL;
	if (sccb_read_locked(OV2640_ADDR, OV2640_REG_COM7, CAM_REG_WIDTH_8,
	                     &com7) != CAM_OK)
		return CAM_ERR_HAL;
	com7 = on ? (uint8_t)(com7 | OV2640_COM7_COLORBAR)
	          : (uint8_t)(com7 & (uint8_t)~OV2640_COM7_COLORBAR);
	if (sccb_write_locked(OV2640_ADDR, OV2640_REG_COM7, CAM_REG_WIDTH_8,
	                      com7) != CAM_OK)
		return CAM_ERR_HAL;
	return CAM_OK;
}

/* Lazy: the ~250 ms sequence runs on the first capture after a power cycle. */
static int camera_configure_locked(int colorbar)
{
	int rc;

	if (!info.configured) {
		/* COM7 soft reset first, exactly as ST's ov2640_Init does -- the table
		   assumes it starts from reset defaults. */
		if (sccb_write_locked(OV2640_ADDR, OV2640_REG_BANK, CAM_REG_WIDTH_8,
		                      OV2640_BANK_SENSOR) != CAM_OK ||
		    sccb_write_locked(OV2640_ADDR, OV2640_REG_COM7, CAM_REG_WIDTH_8,
		                      0x80u) != CAM_OK)
			return CAM_ERR_HAL;
		tx_thread_sleep(CAM_T_SOFT_RESET_MS);

		rc = ov2640_write_table_locked();
		if (rc != CAM_OK)
			return rc;
		rc = ov2640_apply_swap_locked();
		if (rc != CAM_OK)
			return rc;
		rc = ov2640_apply_orient_locked();
		if (rc != CAM_OK)
			return rc;

		info.configured = 1u;
		cam_colorbar    = -1;          /* force the block below to run */
		tx_thread_sleep(CAM_SETTLE_CONFIG_MS);
	}

	if (cam_colorbar != colorbar) {
		rc = ov2640_set_colorbar_locked(colorbar);
		if (rc != CAM_OK)
			return rc;
		/* Committed only on success, so a failed switch is retried next time
		   instead of being remembered as done. */
		cam_colorbar = colorbar;
		tx_thread_sleep(CAM_SETTLE_CONFIG_MS);
	}
	return CAM_OK;
}

/* ---- DCMI + DMA ---------------------------------------------------------- */

/*
 * Values recovered by disassembling the board's factory Arduino firmware
 * (its HAL_DCMI_MspInit / HAL_DCMI_Init), which is the only description of this
 * camera connector that exists: hardware sync (no embedded codes), pixel clock
 * latched on the RISING edge, VSYNC and HSYNC active LOW, every frame captured,
 * 8-bit parallel data.
 */
static int dcmi_init_locked(void)
{
	GPIO_InitTypeDef g = {0};

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_DCMI_CLK_ENABLE();
	__HAL_RCC_DMA2_CLK_ENABLE();

	g.Mode      = GPIO_MODE_AF_PP;
	g.Pull      = GPIO_NOPULL;
	g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
	g.Alternate = CAM_DCMI_AF;
	g.Pin = CAM_DCMI_A_PINS; HAL_GPIO_Init(GPIOA, &g);
	g.Pin = CAM_DCMI_D_PINS; HAL_GPIO_Init(GPIOD, &g);
	g.Pin = CAM_DCMI_E_PINS; HAL_GPIO_Init(GPIOE, &g);
	g.Pin = CAM_DCMI_G_PINS; HAL_GPIO_Init(GPIOG, &g);
	g.Pin = CAM_DCMI_H_PINS; HAL_GPIO_Init(GPIOH, &g);

	hdcmi.Instance              = DCMI;
	hdcmi.Init.SynchroMode      = DCMI_SYNCHRO_HARDWARE;
	hdcmi.Init.PCKPolarity      = DCMI_PCKPOLARITY_RISING;
	hdcmi.Init.VSPolarity       = DCMI_VSPOLARITY_LOW;
	hdcmi.Init.HSPolarity       = DCMI_HSPOLARITY_LOW;
	hdcmi.Init.CaptureRate      = DCMI_CR_ALL_FRAME;
	hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
	hdcmi.Init.JPEGMode         = DCMI_JPEG_DISABLE;
	hdcmi.Init.ByteSelectMode   = DCMI_BSM_ALL;
	hdcmi.Init.ByteSelectStart  = DCMI_OEBS_ODD;
	hdcmi.Init.LineSelectMode   = DCMI_LSM_ALL;
	hdcmi.Init.LineSelectStart  = DCMI_OELS_ODD;
	if (HAL_DCMI_Init(&hdcmi) != HAL_OK) {
		LOG_ERR("DCMI init failed");
		return CAM_ERR_HAL;
	}

	/* DMA2_Stream1 with DMAMUX request 75.  Word-wide on both sides: the DCMI
	   packs four received bytes into its 32-bit DR least-significant byte first
	   and the core is little-endian, so the bytes land in memory in arrival
	   order -- no swap is introduced here (the sensor's IMAGE_MODE bit 0 is a
	   separate, deliberate one). */
	hdma_dcmi.Instance                 = DMA2_Stream1;
	hdma_dcmi.Init.Request             = DMA_REQUEST_DCMI_PSSI;
	hdma_dcmi.Init.Direction           = DMA_PERIPH_TO_MEMORY;
	hdma_dcmi.Init.PeriphInc           = DMA_PINC_DISABLE;
	hdma_dcmi.Init.MemInc              = DMA_MINC_ENABLE;
	hdma_dcmi.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
	hdma_dcmi.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
	hdma_dcmi.Init.Mode                = DMA_NORMAL;
	hdma_dcmi.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
	hdma_dcmi.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
	hdma_dcmi.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
	/*
	 * Burst the memory side.  The factory firmware used SINGLE and phase 2 copied
	 * it, which is fine on a bus nobody else wants -- but it means one AXI write
	 * per 32-bit word, 38400 arbitrations per frame.  Put the LTDC on the same
	 * OCTOSPI and that falls apart: the DCMI FIFO is 8 words (RM0468 sec 36.3.10)
	 * and an active line arrives at PCLK x 2 B/s, so the DMA has roughly 1.3 us to
	 * win each round.  Measured with scan-out live and SINGLE bursts: 333 FIFO
	 * errors in 320 ms, then a DCMI overrun killed the stream.  INC4 cuts the
	 * arbitration count by four.
	 *
	 * INC4 is the ceiling here, not a choice: with MSIZE = word and the FIFO at
	 * the full threshold the HAL rejects INC8/INC16 (DMA_CheckFifoParam refuses
	 * any MemBurst with MBURST_1 set), because a burst may not exceed the 4-word
	 * FIFO.  The peripheral side stays SINGLE -- it reads one fixed register
	 * (PINC off), which cannot be bursted.
	 */
	hdma_dcmi.Init.MemBurst            = DMA_MBURST_INC4;
	hdma_dcmi.Init.PeriphBurst         = DMA_PBURST_SINGLE;
	if (HAL_DMA_Init(&hdma_dcmi) != HAL_OK) {
		LOG_ERR("DCMI DMA init failed");
		return CAM_ERR_HAL;
	}
	__HAL_LINKDMA(&hdcmi, DMA_Handle, hdma_dcmi);

	/* Priority 6 alongside SDMMC1 and OTG_HS, below the RTL8720 UART's hard
	   byte deadline (5).  It only shapes latency: ThreadX masks with PRIMASK,
	   and an overrun is decided by the DMA's own priority and the 8-word DCMI
	   FIFO long before any ISR could run.  NOT enabled here -- see
	   camera_capture(). */
	HAL_NVIC_SetPriority(DCMI_IRQn, 6, 0);
	HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 6, 0);
	return CAM_OK;
}

/*
 * NOTE THE NAME.  On this part the vector is DCMI_PSSI_IRQHandler -- the DCMI
 * shares its interrupt line with the PSSI, and CMSIS spells the handler
 * accordingly (lib/cmsis_device_h7/Source/Templates/gcc/startup_stm32h725xx.s).
 * Only the *IRQn* has a compatibility alias (`#define DCMI_IRQn DCMI_PSSI_IRQn`
 * in stm32h725xx.h), so writing DCMI_IRQHandler compiles, links, and silently
 * does nothing: the weak DCMI_PSSI_IRQHandler stays bound to Default_Handler and
 * the function is dropped by --gc-sections.  The symptom would have been every
 * capture timing out, i.e. an afternoon spent re-checking the FPC wiring.
 */
void DCMI_PSSI_IRQHandler(void)
{
	tx_glue_isr_enter();
	HAL_DCMI_IRQHandler(&hdcmi);
	tx_glue_isr_exit();
}

void DMA2_Stream1_IRQHandler(void)
{
	tx_glue_isr_enter();
	HAL_DMA_IRQHandler(&hdma_dcmi);
	tx_glue_isr_exit();
}

void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *h)
{
	(void)h;
	/*
	 * Band mode: an integrity check, not a synchronisation source.
	 *
	 * What actually aligns bands with frames is arithmetic -- the DCMI transfers
	 * active pixels only (RM0468 sec 36.3.6 / 36.3.7) and 240 = 4 x 60, so the
	 * fourth band of every frame ends exactly where the frame does.  This
	 * interrupt just says out loud when that stops being true, which is the sort
	 * of assumption that fails silently into a rolling picture otherwise.
	 *
	 * Re-arming is mandatory, not tidiness: HAL_DCMI_IRQHandler disables
	 * DCMI_IT_FRAME unconditionally on its way here (stm32h7xx_hal_dcmi.c, the
	 * disable sits OUTSIDE the snapshot-only branch above it), so without this the
	 * check would run once per stream and then go quiet.
	 *
	 * A count that is off by one band is tolerated on purpose: this interrupt and
	 * the DMA's transfer-complete for the last band are two sources at the same
	 * NVIC priority, so which one is taken first is not ordered.  Only a drift of
	 * two or more means the band grid has really slipped.
	 */
	if (cam_band_mode) {
		uint32_t phase = cam_band_seq % CAMERA_BANDS_PER_FRAME;

		if (phase != 0u && phase != CAMERA_BANDS_PER_FRAME - 1u)
			cam_band_desync++;
		__HAL_DCMI_ENABLE_IT(h, DCMI_IT_FRAME);
		return;
	}
	if (!cam_xfer_active)
		return;                        /* stale completion after an abort */
	(void)tx_semaphore_put(&cam_done);
}

void HAL_DCMI_ErrorCallback(DCMI_HandleTypeDef *h)
{
	(void)h;
	/* Streaming: a continuous-mode overrun is terminal -- RM0468 sec 36.3.10 has
	   the DCMI reset its FIFO and wait for a new SOF, and the HAL IRQ has already
	   aborted the DMA.  Flag it and wake the producer to tear down cleanly.
	   Mode-exclusive with the snapshot gate below. */
	if (cam_stream_active) {
		cam_dcmi_ovr++;
		cam_stream_err = 1;
		(void)tx_semaphore_put(&cam_stream_sem);
		return;
	}
	if (!cam_xfer_active)
		return;
	cam_xfer_err = 1;
	(void)tx_semaphore_put(&cam_done);
}

/* Drop any completion left over from a timed-out or aborted transfer, so the
   next capture cannot mistake it for its own. */
static void drain_done(void)
{
	while (tx_semaphore_get(&cam_done, TX_NO_WAIT) == TX_SUCCESS)
		;
}

/* One snapshot into cam_frame.  Arms, waits, stops -- no sensor state changes,
   so the warm-up loop can call it repeatedly. */
static int dcmi_snapshot_locked(void)
{
	drain_done();
	cam_xfer_err    = 0;
	cam_xfer_active = 1;
	/* HAL only clears these in Init/DeInit -- Stop() ORs in HAL_DCMI_ERROR_NONE,
	   which changes nothing -- so without this the log after a failure can carry
	   bits from an earlier one. */
	hdcmi.ErrorCode    = HAL_DCMI_ERROR_NONE;
	hdma_dcmi.ErrorCode = HAL_DMA_ERROR_NONE;

	/*
	 * Re-arm the error interrupts on EVERY capture.  HAL_DCMI_Init enables
	 * LINE/VSYNC/ERR/OVR once (stm32h7xx_hal_dcmi.c), but the snapshot FRAME
	 * handling in HAL_DCMI_IRQHandler disables all of them again, and
	 * HAL_DCMI_Start_DMA does not put them back.  Without this, an overrun on
	 * the second and later captures surfaces as a TIMEOUT instead of an error --
	 * which points the investigation at the wiring rather than at bandwidth.
	 * Clear the stale flags first so a latched one does not fire the ISR the
	 * instant the enables go in.  LINE/VSYNC stay off: nothing consumes them and
	 * they fire per line.
	 */
	__HAL_DCMI_CLEAR_FLAG(&hdcmi, DCMI_FLAG_ERRRI | DCMI_FLAG_OVRRI |
	                              DCMI_FLAG_FRAMERI | DCMI_FLAG_LINERI |
	                              DCMI_FLAG_VSYNCRI);
	__HAL_DCMI_ENABLE_IT(&hdcmi, DCMI_IT_ERR | DCMI_IT_OVR);

	HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
	HAL_NVIC_EnableIRQ(DCMI_IRQn);

	if (HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)(uintptr_t)cam_frame,
	                       CAM_FRAME_WORDS) != HAL_OK) {
		cam_xfer_active = 0;
		(void)HAL_DCMI_Stop(&hdcmi);
		drain_done();
		LOG_ERR("DCMI start failed (err 0x%lx)",
		        (unsigned long)HAL_DCMI_GetError(&hdcmi));
		return CAM_ERR_HAL;
	}

	if (tx_semaphore_get(&cam_done, CAM_XFER_TIMEOUT_TICKS) != TX_SUCCESS) {
		cam_xfer_active = 0;
		(void)HAL_DCMI_Stop(&hdcmi);
		drain_done();
		LOG_ERR("frame timed out -- no VSYNC/PCLK? (XCLK %lu Hz)",
		        (unsigned long)info.xclk_hz);
		return CAM_ERR_TIMEOUT;
	}
	cam_xfer_active = 0;

	if (cam_xfer_err) {
		(void)HAL_DCMI_Stop(&hdcmi);
		drain_done();
		LOG_ERR("capture error (DCMI err 0x%lx, DMA err 0x%lx)",
		        (unsigned long)HAL_DCMI_GetError(&hdcmi),
		        (unsigned long)HAL_DMA_GetError(&hdma_dcmi));
		return CAM_ERR_HAL;
	}

	/* Snapshot mode auto-clears CAPTURE; Stop also disables the DCMI and aborts
	   the DMA, leaving the HAL READY for the next call. */
	(void)HAL_DCMI_Stop(&hdcmi);

	info.frame_bytes = CAMERA_FRAME_BYTES;
	cam_frame_gen++;
	return CAM_OK;
}

/*
 * Capture, discarding the first CAMERA_WARM_FRAMES frames.
 *
 * The OV2640 owns its exposure and gain loops and they start from the register
 * table's defaults, so the frames right after a power cycle are badly
 * under-exposed.  Measured on this board, capturing back to back immediately
 * after `camera off; camera probe`:
 *
 *     warm = 0   frame 1 mean R2 G11 B0    frame 2 R4 G15 B0   frame 3 R6 G22 B3
 *     warm = 15  frame 1 mean R14 G28 B12  frame 2 R15 G31 B13 frame 3 R15 G31 B13
 *
 * i.e. without warm-up the first frame is seven times too dark and three
 * consecutive captures still have not converged; with 15 discarded frames the
 * first kept frame is already at the settled value.  Convergence therefore needs
 * somewhere between 4 and 16 frames -- 15 is the number actually shown to work,
 * so it is the default rather than a smaller guess.  The cost is ~1 s per
 * capture, which is invisible on a human-typed command.
 *
 * (An adaptive version -- keep discarding until the frame mean stops moving --
 * would be tighter, but it is a phase 3 concern: continuous capture will not use
 * this path at all.)
 *
 * NOT the cause of the horizontal banding that was blamed on it for a while.
 * Phase 2 saw a step at row 27 -- R -37% / G -8% / B ~0, the scene continuous
 * across it at 0.996 correlation -- and guessed at a gain landing mid-readout.
 * Issue #44 settled it: that was DCMI_D6 open inside the camera module, sampled
 * off a floating pin.  The channel ratio was the tell and it was in the numbers
 * from the start -- R and G hit, B untouched, because D6 carries R bit 3 and G
 * bit 1 and never touches B.  See the DVP data-line table in
 * shell/cmds/cmd_camera.c.
 *
 * The warm-up stays: it is justified by the exposure measurements above, which
 * are a separate and still-valid finding.  Do not re-derive one from the other.
 */
static int camera_capture_locked(int colorbar)
{
	int rc;

	/* Streaming owns hdcmi/hdma_dcmi; the two paths cannot coexist. */
	if (cam_xfer_active || cam_stream_active)
		return CAM_ERR_BUSY;
	if (!info.powered || info.sensor != CAM_SENSOR_OV2640) {
		rc = camera_probe_locked();
		if (rc != CAM_OK)
			return rc;
	}
	rc = camera_configure_locked(colorbar);
	if (rc != CAM_OK)
		return rc;

	info.frame_valid = 0u;
	for (unsigned i = 0; i <= CAMERA_WARM_FRAMES; i++) {
		rc = dcmi_snapshot_locked();
		if (rc != CAM_OK)
			return rc;
	}
	info.frame_valid = 1u;
	return CAM_OK;
}

/* ---- streaming producer -------------------------------------------------- */

/* The pipeline core's injected mutual exclusion -- a different mutex from
   cam_lock, and only ever taken from thread context. */
static void cam_pipe_os_lock(void *ctx)
{
	(void)ctx;
	(void)tx_mutex_get(&cam_pipe_lock, TX_WAIT_FOREVER);
}

static void cam_pipe_os_unlock(void *ctx)
{
	(void)ctx;
	(void)tx_mutex_put(&cam_pipe_lock);
}

static const struct frame_os cam_pipe_os = {
	NULL, cam_pipe_os_lock, cam_pipe_os_unlock
};

/* Counting sink: the display-independent throughput consumer.  DROP policy and
   it returns the pin immediately, so it can never stall the producer. */
static int cam_stat_open(void *ctx, enum frame_format fmt, uint16_t w, uint16_t h)
{
	(void)ctx; (void)fmt; (void)w; (void)h;
	return 0;
}

static int cam_stat_consume(void *ctx, const struct frame_desc *f)
{
	(void)ctx;
	frame_pipeline_put(&cam_pipe, &cam_stat_sink, f);
	return 0;
}

/* DMA transfer-complete, ISR context: one ring slot -- or, in band mode, one
   band -- just filled.  Wakes the producer and nothing else: it touches no ring,
   no pipeline and no CT.  The band counter is the single exception, and it is
   here rather than in the producer because only the ISR sees every completion;
   a producer that fell a band behind would otherwise lose count silently. */
static void cam_stream_dma_cb(DMA_HandleTypeDef *h)
{
	(void)h;
	if (!cam_stream_active)
		return;
	if (cam_band_mode)
		cam_band_seq++;
	(void)tx_semaphore_put(&cam_stream_sem);
}

/*
 * DMA error, ISR context.  HAL_DMAEx_MultiBufferStart_IT enables the FIFO-error
 * interrupt that the snapshot path (HAL_DMA_Start_IT) leaves off, so FE starts
 * being reported the moment we stream.  Per RM0468 sec 15.3.16 a FIFO or
 * direct-mode error does NOT disable the stream -- only a transfer error does --
 * so FE/DME are counted and ignored, and only TE is terminal.
 *
 * The bits have to be cleared out of h->ErrorCode by hand: HAL_DMA_IRQHandler
 * calls the error callback whenever ErrorCode is nonzero, including after a
 * plain transfer-complete, so leaving them set re-enters this on every frame.
 */
static void cam_stream_dma_err_cb(DMA_HandleTypeDef *h)
{
	if (!cam_stream_active)
		return;
	if (!(h->ErrorCode & HAL_DMA_ERROR_TE)) {
		cam_stream_fe++;
		h->ErrorCode &= ~(uint32_t)(HAL_DMA_ERROR_FE | HAL_DMA_ERROR_DME);
		return;
	}
	cam_stream_err = 1;
	(void)tx_semaphore_put(&cam_stream_sem);
}

static void drain_stream_sem(void)
{
	while (tx_semaphore_get(&cam_stream_sem, TX_NO_WAIT) == TX_SUCCESS)
		;
	/* A start kick that arrived while the producer was already running would
	   otherwise make its next idle wait return immediately. */
	while (tx_semaphore_get(&cam_start_sem, TX_NO_WAIT) == TX_SUCCESS)
		;
}

/*
 * Tear down a stream.  ORDER MATTERS, and it is deliberately not the order the
 * f746 firmware uses.
 *
 * app/psram.c refuses to retune OCTOSPI1 while camera_streaming() is true, so
 * cam_stream_active is what keeps another console off the bus.  It therefore has
 * to stay set until the DCMI is stopped, the DMA is aborted AND the final copy
 * out of the ring is done -- clearing it first (which is what f746 does, because
 * it has no such gate) would open a window where `psram clk` could retune the
 * bus mid-copy.  Interrupts go first so no late completion can disturb the
 * teardown; HAL_DCMI_Stop() and HAL_DMA_Abort() are synchronous, so nothing is
 * lost by not servicing that last IRQ.
 */
static void cam_stream_teardown(void)
{
	HAL_NVIC_DisableIRQ(DMA2_Stream1_IRQn);
	HAL_NVIC_DisableIRQ(DCMI_IRQn);

	(void)HAL_DCMI_Stop(&hdcmi);
	(void)HAL_DMA_Abort(&hdma_dcmi);
	/* Leave the stream the way the snapshot path expects to find it: DBM off and
	   CT back at buffer 0.  The abort above already cleared EN, which is what
	   makes these writable. */
	DMA2_Stream1->CR &= ~(uint32_t)(DMA_SxCR_DBM | DMA_SxCR_CT);

	cam_elapsed_ms = HAL_GetTick() - cam_start_tick;
	drain_stream_sem();

	if (cam_band_mode) {
		/* Nothing to hand over: a band stream never filled cam_ring, and copying
		   a stale frame out of it into cam_frame would make `camera save` claim a
		   picture the preview never showed.  The band buffers are the DMA's, and
		   it has stopped. */
		cam_band_mode = 0;
		cam_band_cb   = NULL;
		cam_band_ctx  = NULL;
		cam_stream_active = 0;      /* last: releases the OCTOSPI1 gate */
		LOG_INF("band stop: %lu frames, %lu ms, ovr %lu, fe %lu, late %lu, "
		        "torn %lu, desync %lu",
		        (unsigned long)cam_band_frames, (unsigned long)cam_elapsed_ms,
		        (unsigned long)cam_dcmi_ovr, (unsigned long)cam_stream_fe,
		        (unsigned long)cam_band_late, (unsigned long)cam_band_torn,
		        (unsigned long)cam_band_desync);
		return;
	}

	/* Hand the last streamed frame to the snapshot buffer so `camera save` /
	   `send` work on what was just captured.  Reading the ring is safe now (the
	   DMA is stopped), but cam_frame and info are shell-visible state, so this
	   takes cam_lock -- a `camera send` on the other console could be walking
	   cam_frame right now.  Lock order is cam_lock -> cam_pipe_lock here, the
	   same way round as cam_stream_start_locked(), so it cannot deadlock. */
	if (tx_mutex_get(&cam_lock, TX_WAIT_FOREVER) == TX_SUCCESS) {
		if (frame_pipeline_read_latest(&cam_pipe, 0u, cam_frame,
		                               CAMERA_FRAME_BYTES, NULL) == 0) {
			info.frame_bytes = CAMERA_FRAME_BYTES;
			info.frame_valid = 1u;
			cam_frame_gen++;
		}
		(void)tx_mutex_put(&cam_lock);
	}

	frame_pipeline_detach(&cam_pipe, &cam_stat_sink);
	cam_m0 = NULL;
	cam_m1 = NULL;

	cam_stream_active = 0;          /* last: releases the OCTOSPI1 gate */
	LOG_INF("stream stop: %lu frames, %lu ms, ovr %lu/%lu, fe %lu",
	        (unsigned long)cam_pipe.stats.published,
	        (unsigned long)cam_elapsed_ms, (unsigned long)cam_dcmi_ovr,
	        (unsigned long)cam_ring_ovr, (unsigned long)cam_stream_fe);
}

/*
 * Service one completed BAND (issue #35).
 *
 * Simpler than the frame path by construction: the memory registers are fixed for
 * the life of the stream, so there is no repoint, no CT race and no ring.  What is
 * left is bookkeeping -- which band did we just get, is it still intact, and does
 * it continue the frame the consumer is already drawing?
 *
 * THE DEADLINE, derived once here because it is easy to get wrong by a factor of
 * two.  cam_band_seq counts COMPLETIONS, so a sample of S means transfers 0..S-1
 * are done and the DMA is part way through transfer S.  The band we hand out is
 * S-1, in buffer (ct0 + S-1) % 2; the DMA is filling the other one.  It comes back
 * to ours at transfer S+1 -- which starts the instant transfer S completes, i.e.
 * the instant the ISR makes the counter S+1.  So the consumer has ONE band period
 * from the completion that woke us, ~18.5 ms at the sensor's ~13.5 fps, not two.
 * A transpose is ~0.4 ms, so the margin is large, but it is a single band period
 * and contention on the display lock spends it.
 *
 * That makes the check after the callback exact rather than conservative: any
 * advance of the counter at all means the DMA is already writing the buffer that
 * was just read.  The consumer only ever gets whole frames, so a torn band --
 * like a missed one -- abandons the rest of that frame instead of delivering a
 * hole, which would leave one stale stripe on the panel that no later frame
 * explains.
 */
static void cam_band_service(void)
{
	uint32_t seq = cam_band_seq;   /* completions so far; ~2.5 years to wrap */
	uint32_t newest, band, buf;

	if (seq == 0u)
		return;                    /* armed, nothing has completed yet */
	newest = seq - 1u;
	if (newest < cam_band_next)
		return;                    /* already delivered -- a bounded-wait tick */
	if (newest > cam_band_next) {
		cam_band_late += newest - cam_band_next;
		cam_band_sync = 0;         /* the frame in progress now has a hole */
	}
	cam_band_next = newest + 1u;

	band = newest % CAMERA_BANDS_PER_FRAME;
	if (!cam_band_sync) {
		if (band != 0u)
			return;                /* resume only on a frame boundary */
		cam_band_sync = 1;
	}

	buf = (cam_band_ct0 + newest) % 2u;
	/* Discard any line the CPU still holds over this band before reading what the
	   DMA wrote -- the same contract port/sd/sd_card.c has with the SDMMC IDMA.
	   Cheap despite the size: 38,400 B is 1,200 lines, and the strided read that
	   follows re-fetches each of them exactly once (a 640 B row stride over 60
	   rows is 1,920 B of working set, which stays resident across all 320
	   columns). */
	SCB_InvalidateDCache_by_Addr(cam_band[buf], (int32_t)CAMERA_BAND_BYTES);
	if (cam_band_cb != NULL)
		cam_band_cb(cam_band_ctx, (unsigned)band,
		            (const uint16_t *)(const void *)cam_band[buf],
		            (unsigned)CAMERA_BAND_ROWS);

	if (cam_band_seq != seq) {     /* exact, not conservative -- see the header */
		cam_band_torn++;
		cam_band_sync = 0;
	} else if (band == CAMERA_BANDS_PER_FRAME - 1u) {
		cam_band_frames++;
	}
}

/*
 * Service one completed frame.
 *
 * The tear-free ordering: find the slot the DMA just finished with, take a free
 * one, point the DMA's now-idle memory register at the free slot, and only then
 * publish the finished one.  A published slot is therefore never a live DMA
 * target.  No free slot -> do not publish; the memory register keeps pointing at
 * the finished slot and the DMA simply refills it (a dropped frame, counted).
 *
 * The repoint is the delicate part.  RM0468 sec 15.3.11 allows M0AR/M1AR to be
 * written on the fly ONLY for the buffer that is not currently the target:
 * M1AR while CT=0, M0AR while CT=1.  Writing the active one raises TEIF and the
 * hardware disables the stream.  HAL_DMAEx_ChangeMemory() checks none of this --
 * it writes the register and returns HAL_OK regardless.  CT is flipped by the
 * DMA itself, so masking interrupts does not freeze it; the only defence is to
 * re-read CT immediately before the write and abandon the repoint if it moved.
 * That is nearly always a non-event (the producer runs microseconds after the
 * completion that woke it, with a whole frame period before the next flip) and
 * matters only if the producer was delayed by a full frame.  If one does slip
 * through, TE stops the stream and it shows up in `camera stream stats` rather
 * than silently corrupting a frame.
 */
static void cam_stream_service(int had_sem)
{
	uint32_t ct, ct2, seen, extra = 0;
	struct frame_desc *done, *freed;
	int published = 0;

	if (cam_stop_req || cam_stream_err ||
	    (cam_target_frames && cam_pipe.stats.published >= cam_target_frames) ||
	    (cam_target_secs &&
	     (HAL_GetTick() - cam_start_tick) >= cam_target_secs * 1000u)) {
		cam_stream_teardown();
		return;
	}
	if (cam_band_mode) {
		cam_band_service();
		return;
	}

	/* Completions observed this pass.  Only the most recent buffer flip can be
	   published; anything older was already overwritten -> ring overrun. */
	while (tx_semaphore_get(&cam_stream_sem, TX_NO_WAIT) == TX_SUCCESS)
		extra++;
	seen = (had_sem ? 1u : 0u) + extra;

	ct = (DMA2_Stream1->CR & DMA_SxCR_CT) ? 1u : 0u;
	if (ct != cam_last_ct) {
		cam_last_ct = ct;
		/* CT==1 -> M0 just completed, CT==0 -> M1 just completed. */
		done  = ct ? cam_m0 : cam_m1;
		freed = frame_pipeline_acquire(&cam_pipe);
		if (freed != NULL) {
			ct2 = (DMA2_Stream1->CR & DMA_SxCR_CT) ? 1u : 0u;
			if (ct2 != ct) {
				/* CT moved while we were acquiring: the register we were about
				   to write is now the live one.  Writing it would raise TEIF and
				   kill the stream, so skip this frame entirely. */
				cam_repoint_skip++;
				cam_last_ct = ct2;
			} else {
				(void)HAL_DMAEx_ChangeMemory(&hdma_dcmi,
				        (uint32_t)(uintptr_t)freed->data,
				        ct ? MEMORY0 : MEMORY1);
				if (ct)
					cam_m0 = freed;
				else
					cam_m1 = freed;
				frame_pipeline_publish(&cam_pipe, done, CAMERA_FRAME_BYTES,
				                       FRAME_FMT_RGB565, CAMERA_FRAME_WIDTH,
				                       CAMERA_FRAME_HEIGHT,
				                       (uint16_t)(CAMERA_FRAME_WIDTH * 2u));
				published = 1;
			}
		}
	}
	if (seen > (uint32_t)published)
		cam_ring_ovr += seen - (uint32_t)published;
}

static void cam_producer_entry(ULONG arg)
{
	(void)arg;
	for (;;) {
		UINT got;

		if (!cam_stream_active) {
			(void)tx_semaphore_get(&cam_start_sem, TX_WAIT_FOREVER);
			continue;
		}
		got = tx_semaphore_get(&cam_stream_sem, CAM_PRODUCER_TICK);
		cam_stream_service(got == TX_SUCCESS);
	}
}

static int cam_stream_start_locked(int colorbar, uint32_t frames, uint32_t secs)
{
	int rc;

	if (cam_stream_active || cam_xfer_active)
		return CAM_ERR_BUSY;
	if (!info.powered || info.sensor != CAM_SENSOR_OV2640) {
		rc = camera_probe_locked();
		if (rc != CAM_OK)
			return rc;
	}
	rc = camera_configure_locked(colorbar);
	if (rc != CAM_OK)
		return rc;

	/*
	 * Re-initialise the pipeline for every run.
	 *
	 * frame_pipeline_acquire() marks a slot FILLING and only publish() gives it
	 * back, so the two slots the DMA was pointing at when the stream stopped stay
	 * FILLING forever -- after one start/stop cycle the ring would be two slots
	 * short and the second start would fail to get its DBM pair.  The counters
	 * would carry over too, which would make `--frames N` stop instantly on the
	 * second run.  init() memsets the whole struct and re-seeds the slots, so it
	 * fixes both at once and needs no change to the (byte-identical) core.
	 */
	/*
	 * Wait for external pins (the preview) to drain before re-initialising.
	 *
	 * init() memsets the pipeline, so a slot pinned by camera_stream_pin_latest()
	 * would come back with its refcount zeroed and the holder's later put() would
	 * corrupt a slot that now belongs to this new run.  Pinning takes cam_lock,
	 * which this function already holds, so nothing new can be pinned from here
	 * on -- waiting is enough to close the window, and it only has to outlast one
	 * blit+flip.  put() deliberately does NOT take cam_lock, or this wait would
	 * deadlock against the very pin it is waiting for.
	 */
	for (unsigned waited = 0u; cam_ext_pins != 0u; waited += 5u) {
		if (waited >= CAM_EXT_PIN_DRAIN_MS) {
			LOG_ERR("stream start: %lu external frame pin(s) never released",
			        (unsigned long)cam_ext_pins);
			return CAM_ERR_BUSY;
		}
		tx_thread_sleep(5u);
	}

	/* Under cam_pipe_lock, taken directly rather than through the pipeline's own
	   os vtable: init() memsets the struct -- p->os included -- so a concurrent
	   `camera stream stats` on the other console must not be inside the core
	   while it happens.  Same lock order as everywhere else (cam_lock is already
	   held here, then cam_pipe_lock). */
	(void)tx_mutex_get(&cam_pipe_lock, TX_WAIT_FOREVER);
	rc = frame_pipeline_init(&cam_pipe, &cam_pipe_os, cam_ring,
	                         CAMERA_FRAME_BYTES, CAM_RING_SLOTS);
	(void)tx_mutex_put(&cam_pipe_lock);
	if (rc != 0)
		return CAM_ERR_STATE;
	frame_pipeline_set_format(&cam_pipe, FRAME_FMT_RGB565, CAMERA_FRAME_WIDTH,
	                          CAMERA_FRAME_HEIGHT);
	if (frame_pipeline_attach(&cam_pipe, &cam_stat_sink) != 0)
		return CAM_ERR_STATE;

	cam_m0 = frame_pipeline_acquire(&cam_pipe);
	cam_m1 = frame_pipeline_acquire(&cam_pipe);
	if (cam_m0 == NULL || cam_m1 == NULL) {
		frame_pipeline_detach(&cam_pipe, &cam_stat_sink);
		return CAM_ERR_STATE;
	}

	cam_stream_err = 0;
	cam_stop_req   = 0;
	cam_stream_fe  = 0;
	cam_dcmi_ovr   = 0;
	cam_ring_ovr   = 0;
	cam_repoint_skip = 0;
	cam_elapsed_ms = 0;
	cam_target_frames = frames;
	cam_target_secs   = secs;
	drain_stream_sem();

	hdma_dcmi.XferCpltCallback   = cam_stream_dma_cb;
	hdma_dcmi.XferM1CpltCallback = cam_stream_dma_cb;
	hdma_dcmi.XferErrorCallback  = cam_stream_dma_err_cb;
	hdcmi.ErrorCode              = HAL_DCMI_ERROR_NONE;
	hdma_dcmi.ErrorCode          = HAL_DMA_ERROR_NONE;
	__HAL_DCMI_CLEAR_FLAG(&hdcmi, DCMI_FLAG_ERRRI | DCMI_FLAG_OVRRI |
	                              DCMI_FLAG_FRAMERI | DCMI_FLAG_LINERI |
	                              DCMI_FLAG_VSYNCRI);

	/* The interrupts are enabled here, not in camera_init(): the ThreadX objects
	   the ISRs post to exist, and nothing can fire before CAPTURE is set below.
	   The snapshot path does its own enable, so a stream started on a board that
	   never ran `camera capture` would otherwise never see a completion. */
	HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
	HAL_NVIC_EnableIRQ(DCMI_IRQn);

	cam_start_tick    = HAL_GetTick();
	cam_last_mode     = CAM_STREAM_FRAME;   /* before active: another console's
	                                           `stats` must not see the two disagree */
	cam_stream_active = 1;

	/* Manual double buffering: HAL's own >64 KB path bands a single frame
	   INTO one buffer, which is not what we want -- we want consecutive frames
	   landing in alternating slots. */
	if (HAL_DMAEx_MultiBufferStart_IT(&hdma_dcmi,
	        (uint32_t)(uintptr_t)&hdcmi.Instance->DR,
	        (uint32_t)(uintptr_t)cam_m0->data,
	        (uint32_t)(uintptr_t)cam_m1->data,
	        CAM_FRAME_WORDS) != HAL_OK) {
		cam_stream_active = 0;
		HAL_NVIC_DisableIRQ(DMA2_Stream1_IRQn);
		HAL_NVIC_DisableIRQ(DCMI_IRQn);
		frame_pipeline_detach(&cam_pipe, &cam_stat_sink);
		LOG_ERR("stream DMA start failed");
		return CAM_ERR_HAL;
	}
	/* Seed the CT tracker from the hardware rather than assuming 0.  Only
	   HAL_DMA_Init() clears CT; HAL_DMAEx_MultiBufferStart_IT() sets DBM, the
	   memory registers and NDTR but leaves CT wherever the previous run left it,
	   so a second stream could otherwise start with the tracker inverted and
	   repoint the live buffer on its first frame. */
	cam_last_ct = (DMA2_Stream1->CR & DMA_SxCR_CT) ? 1u : 0u;

	hdcmi.Instance->CR &= ~DCMI_CR_CM;          /* continuous, not snapshot */
	__HAL_DCMI_ENABLE(&hdcmi);
	__HAL_DCMI_ENABLE_IT(&hdcmi, DCMI_IT_ERR | DCMI_IT_OVR);
	hdcmi.Instance->CR |= DCMI_CR_CAPTURE;
	hdcmi.State = HAL_DCMI_STATE_BUSY;

	(void)tx_semaphore_put(&cam_start_sem);     /* wake the idle producer */
	LOG_INF("stream start (frames=%lu secs=%lu)", (unsigned long)frames,
	        (unsigned long)secs);
	return CAM_OK;
}

/*
 * Arm the band stream (issue #35).
 *
 * Deliberately parallel to cam_stream_start_locked() above, and deliberately
 * shorter: there is no pipeline to re-initialise, no ring slots to acquire and no
 * external pin to drain, because nothing outside the driver ever holds a band.
 * The pieces that DO carry over are the ones that were paid for in bugs -- the
 * error-code reset, the stale-flag clear, and enabling the interrupts here rather
 * than in camera_init() (issue #12: an interrupt source is armed only once the
 * ThreadX objects its ISR posts to exist).
 */
static int cam_band_start_locked(int colorbar, camera_band_fn fn, void *ctx)
{
	int rc;

	if (fn == NULL)
		return CAM_ERR_PARAM;
	if (cam_stream_active || cam_xfer_active)
		return CAM_ERR_BUSY;
	if (!info.powered || info.sensor != CAM_SENSOR_OV2640) {
		rc = camera_probe_locked();
		if (rc != CAM_OK)
			return rc;
	}
	rc = camera_configure_locked(colorbar);
	if (rc != CAM_OK)
		return rc;

	cam_stream_err   = 0;
	cam_stop_req     = 0;
	cam_stream_fe    = 0;
	cam_dcmi_ovr     = 0;
	cam_ring_ovr     = 0;
	cam_repoint_skip = 0;
	cam_elapsed_ms   = 0;
	/* A band stream runs until it is told to stop: there is no --frames/--secs
	   here, because the consumer is a display and it stops when the user says. */
	cam_target_frames = 0u;
	cam_target_secs   = 0u;
	cam_band_seq     = 0u;
	cam_band_next    = 0u;
	cam_band_sync    = 0;
	cam_band_frames  = 0u;
	cam_band_late    = 0u;
	cam_band_torn    = 0u;
	cam_band_desync  = 0u;
	cam_band_cb      = fn;
	cam_band_ctx     = ctx;
	drain_stream_sem();

	hdma_dcmi.XferCpltCallback   = cam_stream_dma_cb;
	hdma_dcmi.XferM1CpltCallback = cam_stream_dma_cb;
	hdma_dcmi.XferErrorCallback  = cam_stream_dma_err_cb;
	hdcmi.ErrorCode              = HAL_DCMI_ERROR_NONE;
	hdma_dcmi.ErrorCode          = HAL_DMA_ERROR_NONE;
	__HAL_DCMI_CLEAR_FLAG(&hdcmi, DCMI_FLAG_ERRRI | DCMI_FLAG_OVRRI |
	                              DCMI_FLAG_FRAMERI | DCMI_FLAG_LINERI |
	                              DCMI_FLAG_VSYNCRI);

	/* Hand both bands to the DMA with no line of ours left over them.  Nothing
	   here writes them again: the CPU is a reader for the life of the stream. */
	SCB_InvalidateDCache_by_Addr(cam_band, (int32_t)sizeof cam_band);

	HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
	HAL_NVIC_EnableIRQ(DCMI_IRQn);

	cam_start_tick    = HAL_GetTick();
	cam_band_mode     = 1;          /* before active: the ISR reads it */
	cam_last_mode     = CAM_STREAM_BAND;
	cam_stream_active = 1;

	if (HAL_DMAEx_MultiBufferStart_IT(&hdma_dcmi,
	        (uint32_t)(uintptr_t)&hdcmi.Instance->DR,
	        (uint32_t)(uintptr_t)cam_band[0],
	        (uint32_t)(uintptr_t)cam_band[1],
	        CAMERA_BAND_BYTES / 4u) != HAL_OK) {
		cam_stream_active = 0;
		cam_band_mode     = 0;
		cam_band_cb       = NULL;
		cam_band_ctx      = NULL;
		HAL_NVIC_DisableIRQ(DMA2_Stream1_IRQn);
		HAL_NVIC_DisableIRQ(DCMI_IRQn);
		LOG_ERR("band DMA start failed");
		return CAM_ERR_HAL;
	}
	/* Which band buffer the first transfer fills -- read, not assumed.  See the
	   comment on cam_band_seq: everything downstream pairs a completion with a
	   buffer from this one sample. */
	cam_band_ct0 = (DMA2_Stream1->CR & DMA_SxCR_CT) ? 1u : 0u;

	hdcmi.Instance->CR &= ~DCMI_CR_CM;          /* continuous, not snapshot */
	__HAL_DCMI_ENABLE(&hdcmi);
	/* FRAME as well as ERR/OVR: it is the integrity check on the band grid.  See
	   HAL_DCMI_FrameEventCallback() for why it has to be re-armed there. */
	__HAL_DCMI_ENABLE_IT(&hdcmi, DCMI_IT_ERR | DCMI_IT_OVR | DCMI_IT_FRAME);
	hdcmi.Instance->CR |= DCMI_CR_CAPTURE;
	hdcmi.State = HAL_DCMI_STATE_BUSY;

	(void)tx_semaphore_put(&cam_start_sem);     /* wake the idle producer */
	LOG_INF("band stream start (%u rows x %u bands, %lu B x2 in AXI-SRAM)",
	        (unsigned)CAMERA_BAND_ROWS, (unsigned)CAMERA_BANDS_PER_FRAME,
	        (unsigned long)CAMERA_BAND_BYTES);
	return CAM_OK;
}

/* ---- public API ---------------------------------------------------------- */

int camera_init(void)
{
	GPIO_InitTypeDef g = {0};
	int rc;

	if (info.ready)
		return CAM_OK;

	if (tx_mutex_create(&cam_lock, "camera", TX_INHERIT) != TX_SUCCESS)
		return CAM_ERR_STATE;
	/* The DCMI/DMA ISRs post these, so they exist before any interrupt can be
	   enabled -- which camera_capture() / camera_stream_start(), not this
	   function, do (issue #12). */
	if (tx_semaphore_create(&cam_done, "cam_done", 0) != TX_SUCCESS) {
		(void)tx_mutex_delete(&cam_lock);
		return CAM_ERR_STATE;
	}
	if (tx_mutex_create(&cam_pipe_lock, "campipe", TX_INHERIT) != TX_SUCCESS ||
	    tx_semaphore_create(&cam_stream_sem, "cam_strm", 0) != TX_SUCCESS ||
	    tx_semaphore_create(&cam_start_sem, "cam_strt", 0) != TX_SUCCESS)
		return CAM_ERR_STATE;
	if (frame_pipeline_init(&cam_pipe, &cam_pipe_os, cam_ring,
	                        CAMERA_FRAME_BYTES, CAM_RING_SLOTS) != 0)
		return CAM_ERR_STATE;
	cam_stat_sink.name    = "stats";
	cam_stat_sink.ctx     = NULL;
	cam_stat_sink.policy  = FRAME_POLICY_DROP;
	cam_stat_sink.open    = cam_stat_open;
	cam_stat_sink.consume = cam_stat_consume;
	cam_stat_sink.close   = NULL;

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();

	/* Drive the asserted level into ODR while the pins are still inputs, then
	   switch them to outputs: the module can never see a glitch that wakes it. */
	HAL_GPIO_WritePin(CAM_PWDN_PORT, CAM_PWDN_PIN, pwdn_level(1));
	HAL_GPIO_WritePin(CAM_RST_PORT, CAM_RST_PIN, rst_level(1));
	g.Mode  = GPIO_MODE_OUTPUT_PP;
	g.Pull  = GPIO_NOPULL;
	g.Speed = GPIO_SPEED_FREQ_LOW;
	g.Pin   = CAM_PWDN_PIN;
	HAL_GPIO_Init(CAM_PWDN_PORT, &g);
	g.Pin   = CAM_RST_PIN;
	HAL_GPIO_Init(CAM_RST_PORT, &g);

	/* XCLK parked low (timer stopped) until camera_on(). */
	HAL_GPIO_WritePin(CAM_XCLK_PORT, CAM_XCLK_PIN, GPIO_PIN_RESET);
	g.Pin = CAM_XCLK_PIN;
	HAL_GPIO_Init(CAM_XCLK_PORT, &g);

	/* The _locked helpers are called here without holding cam_lock, which is
	   correct exactly once: camera_init() runs from tx_application_define()
	   before the scheduler exists, so no other thread can be inside them. */
	rc = cam_i2c_init_locked();
	if (rc == CAM_OK)
		rc = dcmi_init_locked();
	if (rc != CAM_OK) {
		(void)tx_semaphore_delete(&cam_done);
		(void)tx_mutex_delete(&cam_lock);
		return rc;
	}

	/* The producer parks on cam_start_sem until a stream arms it.  Creating it
	   here is safe pre-scheduler: a ThreadX thread made in tx_application_define
	   is merely READY, and this one's first act is to block. */
	if (tx_thread_create(&cam_producer, "cam_prod", cam_producer_entry, 0,
	                     cam_producer_stack, sizeof cam_producer_stack,
	                     CAM_PRODUCER_PRIO, CAM_PRODUCER_PRIO,
	                     TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS) {
		LOG_ERR("producer thread create failed");
		return CAM_ERR_STATE;
	}

	info.xclk_src_hz = cam_tim_ker_hz();
	info.ready       = 1u;
	LOG_INF("ready: XCLK src %lu Hz, I2C4 ker %lu Hz",
	        (unsigned long)info.xclk_src_hz, (unsigned long)info.i2c_ker_hz);
	return CAM_OK;
}

int camera_on(void)
{
	int rc = op_lock();

	if (rc != CAM_OK)
		return rc;
	if (cam_stream_active) {
		op_unlock();
		return CAM_ERR_BUSY;
	}
	if (!info.powered)
		power_on_locked();
	op_unlock();
	return CAM_OK;
}

int camera_off(void)
{
	int rc = op_lock();

	if (rc != CAM_OK)
		return rc;
	/* Cutting XCLK and asserting PWDN under a live DMA leaves the DCMI waiting
	   for a VSYNC that will never come.  Two consoles make this reachable. */
	if (cam_stream_active) {
		op_unlock();
		return CAM_ERR_BUSY;
	}
	power_off_locked();
	op_unlock();
	return CAM_OK;
}

int camera_probe(void)
{
	int rc = op_lock();

	if (rc != CAM_OK)
		return rc;
	if (cam_stream_active) {
		op_unlock();
		return CAM_ERR_BUSY;      /* probe power-cycles the sensor */
	}
	rc = camera_probe_locked();
	op_unlock();
	return rc;
}

int camera_capture(int colorbar)
{
	int rc = op_lock();

	if (rc != CAM_OK)
		return rc;
	rc = camera_capture_locked(colorbar);
	op_unlock();
	return rc;
}

int camera_stream_start(int colorbar, uint32_t frames, uint32_t secs)
{
	int rc = op_lock();

	if (rc != CAM_OK)
		return rc;
	rc = cam_stream_start_locked(colorbar, frames, secs);
	op_unlock();
	return rc;
}

int camera_band_start(int colorbar, camera_band_fn fn, void *ctx)
{
	int rc = op_lock();

	if (rc != CAM_OK)
		return rc;
	rc = cam_band_start_locked(colorbar, fn, ctx);
	op_unlock();
	return rc;
}

int camera_band_streaming(void)
{
	return cam_stream_active && cam_band_mode;
}

/*
 * Ask the producer to stop and wait for it to finish the teardown.  The stop is
 * not done here: the producer owns the DCMI/DMA state machine, and having two
 * threads tear it down is how that state machine gets corrupted.  Waiting is
 * what makes `camera stream stop` mean "the bus is free now" to the caller --
 * which matters because psram_acquire() keys off camera_streaming().
 */
int camera_stream_stop(void)
{
	unsigned wait_ms = 0u;

	if (!info.ready)
		return CAM_ERR_STATE;
	if (!cam_stream_active)
		return CAM_OK;
	cam_stop_req = 1;
	(void)tx_semaphore_put(&cam_stream_sem);   /* poke it out of its bounded wait */
	while (cam_stream_active && wait_ms < 1000u) {
		tx_thread_sleep(5u);
		wait_ms += 5u;
	}
	return cam_stream_active ? CAM_ERR_TIMEOUT : CAM_OK;
}

int camera_streaming(void)
{
	return cam_stream_active;
}

int camera_stream_stats(struct camera_stream_stats *out)
{
	struct frame_stats fs;

	if (out == NULL)
		return CAM_ERR_PARAM;
	if (!info.ready)
		return CAM_ERR_STATE;
	memset(out, 0, sizeof *out);
	out->active     = (uint8_t)(cam_stream_active ? 1 : 0);
	out->mode       = cam_last_mode;
	out->elapsed_ms = cam_stream_active ? (HAL_GetTick() - cam_start_tick)
	                                    : cam_elapsed_ms;
	out->dcmi_ovr   = cam_dcmi_ovr;
	out->dma_fe     = cam_stream_fe;
	if (cam_last_mode == CAM_STREAM_BAND) {
		/* Nothing from the pipeline here -- the ring is not merely empty, it is
		   not part of this path, and reporting its zeroes beside real counters
		   would read as "no overruns" rather than "no ring". */
		out->frames     = cam_band_frames;
		out->band_late  = cam_band_late;
		out->band_torn  = cam_band_torn;
		out->band_desync = cam_band_desync;
		return CAM_OK;
	}
	frame_pipeline_stats(&cam_pipe, &fs);
	out->frames     = fs.published;
	out->ring_ovr   = cam_ring_ovr + fs.overruns;
	out->repoint_skip = cam_repoint_skip;
	out->delivered  = cam_stat_sink.delivered;
	out->dropped    = cam_stat_sink.dropped;
	out->slots      = CAM_RING_SLOTS;
	return CAM_OK;
}

/*
 * Pin the newest streamed frame for an outside reader (app/cam_preview.c).
 *
 * The count goes up BEFORE the pin and comes down AFTER the unpin, so a stream
 * start that samples it can only ever be too cautious, never too permissive.
 * Taking cam_lock here is what actually closes the window: a start holds that
 * lock across both its drain wait and the frame_pipeline_init() that follows, so
 * no pin can be created in between.
 */
const struct frame_desc *camera_stream_pin_latest(void)
{
	const struct frame_desc *d;

	if (!info.ready)
		return NULL;
	if (tx_mutex_get(&cam_lock, TX_WAIT_FOREVER) != TX_SUCCESS)
		return NULL;
	/* Band mode fills no ring: the pipeline still holds whatever the last frame
	   stream published, and handing that out would show a consumer a stale
	   picture it has no way to recognise as stale. */
	if (!cam_stream_active || cam_band_mode) {
		(void)tx_mutex_put(&cam_lock);
		return NULL;
	}
	cam_ext_pins++;
	d = frame_pipeline_pin_latest(&cam_pipe);
	if (d == NULL) {
		/* Nothing published yet -- a start has armed the DMA but no frame has
		   completed.  Give the count back, or the next start spends its whole
		   drain timeout waiting for a pin that was never taken. */
		cam_ext_pins--;
		(void)tx_mutex_put(&cam_lock);
		return NULL;
	}
	(void)tx_mutex_put(&cam_lock);
	return d;
}

/* Release a pin.  Deliberately does NOT take cam_lock: a stream start waits for
   the pin count with that lock held, so needing it here would deadlock. */
void camera_stream_put(const struct frame_desc *f)
{
	if (f == NULL)
		return;
	frame_pipeline_put(&cam_pipe, NULL, f);
	cam_ext_pins--;
}

int camera_get_mode(struct camera_mode *out)
{
	if (out == NULL)
		return CAM_ERR_PARAM;
	out->width       = CAMERA_FRAME_WIDTH;
	out->height      = CAMERA_FRAME_HEIGHT;
	out->frame_bytes = CAMERA_FRAME_BYTES;
	out->format      = "RGB565";
	return CAM_OK;
}

/*
 * Reported by `camera info` beside the byte swap, because it is the same kind of
 * fact: a build-time decision to disagree with ST's register table, invisible in
 * any counter, and detectable only by looking at the picture.  Derived from the
 * CAM_SENSOR_* constants so the console cannot drift from what was written.
 */
const char *camera_get_orient(void)
{
	if (CAM_SENSOR_HMIRROR && CAM_SENSOR_VFLIP)
		return "h-mirror + v-flip";
	if (CAM_SENSOR_HMIRROR)
		return "h-mirror";
	if (CAM_SENSOR_VFLIP)
		return "v-flip";
	return "no mirror/flip";
}

int camera_frame_read(uint32_t offset, void *dst, uint32_t len, uint32_t *gen)
{
	int rc;

	if (dst == NULL || len == 0u)
		return CAM_ERR_PARAM;
	rc = op_lock();
	if (rc != CAM_OK)
		return rc;
	if (!info.frame_valid) {
		op_unlock();
		return CAM_ERR_NO_FRAME;
	}
	if (offset > info.frame_bytes || len > info.frame_bytes - offset) {
		op_unlock();
		return CAM_ERR_PARAM;
	}
	/* Straight memcpy: the frame is in the MPU's non-cacheable PSRAM window, so
	   there is nothing to invalidate before reading what the DMA wrote. */
	memcpy(dst, cam_frame + offset, len);
	if (gen != NULL)
		*gen = cam_frame_gen;
	op_unlock();
	return CAM_OK;
}

int camera_get_info(struct camera_info *out)
{
	if (out == NULL)
		return CAM_ERR_PARAM;
	if (!info.ready) {
		memset(out, 0, sizeof *out);
		return CAM_ERR_STATE;
	}
	if (tx_mutex_get(&cam_lock, TX_WAIT_FOREVER) != TX_SUCCESS)
		return CAM_ERR_STATE;
	*out = info;
	op_unlock();
	return CAM_OK;
}

int camera_scan(uint8_t *addrs, unsigned max, unsigned *found)
{
	unsigned n = 0u;
	int rc = op_lock();

	if (rc != CAM_OK)
		return rc;
	if (!info.powered) {
		op_unlock();
		return CAM_ERR_STATE;
	}
	i2c_ensure_ready_locked();
	for (uint8_t a = 0x08u; a <= 0x77u; a++) {
		if (!addr_acks_locked(a))
			continue;
		if (addrs != NULL && n < max)
			addrs[n] = a;
		n++;
	}
	op_unlock();
	if (found != NULL)
		*found = n;
	return CAM_OK;
}

int camera_reg_read(uint8_t i2c_addr, uint16_t reg, unsigned reg_width,
                    uint8_t *val)
{
	int rc;

	if (val == NULL || i2c_addr > 0x7Fu ||
	    (reg_width != CAM_REG_WIDTH_8 && reg_width != CAM_REG_WIDTH_16))
		return CAM_ERR_PARAM;
	rc = op_lock();
	if (rc != CAM_OK)
		return rc;
	if (!info.powered) {
		op_unlock();
		return CAM_ERR_STATE;
	}
	i2c_ensure_ready_locked();
	rc = sccb_read_locked(i2c_addr, reg, reg_width, val);
	op_unlock();
	return rc;
}

int camera_reg_write(uint8_t i2c_addr, uint16_t reg, unsigned reg_width,
                     uint8_t val)
{
	int rc;

	if (i2c_addr > 0x7Fu ||
	    (reg_width != CAM_REG_WIDTH_8 && reg_width != CAM_REG_WIDTH_16))
		return CAM_ERR_PARAM;
	rc = op_lock();
	if (rc != CAM_OK)
		return rc;
	if (!info.powered) {
		op_unlock();
		return CAM_ERR_STATE;
	}
	/* Reads stay allowed while streaming (they are a diagnosis tool); a WRITE can
	   reconfigure the sensor out from under the DMA. */
	if (cam_stream_active) {
		op_unlock();
		return CAM_ERR_BUSY;
	}
	i2c_ensure_ready_locked();
	rc = sccb_write_locked(i2c_addr, reg, reg_width, val);
	op_unlock();
	return rc;
}

const char *cam_strerror(int rc)
{
	switch (rc) {
	case CAM_OK:             return "ok";
	case CAM_ERR_PARAM:      return "bad argument";
	case CAM_ERR_HAL:        return "SCCB transfer failed";
	case CAM_ERR_TIMEOUT:    return "timeout";
	case CAM_ERR_STATE:      return "camera not initialized or not powered";
	case CAM_ERR_NO_SENSOR:  return "no sensor identified";
	case CAM_ERR_NO_FRAME:   return "no frame captured yet";
	case CAM_ERR_BUSY:       return "a capture or stream already owns the DCMI";
	default:                 return "unknown error";
	}
}

const char *camera_sensor_name(unsigned sensor)
{
	switch (sensor) {
	case CAM_SENSOR_OV2640:  return "OV2640";
	case CAM_SENSOR_OV5640:  return "OV5640";
	case CAM_SENSOR_UNKNOWN: return "unknown";
	default:                 return "none";
	}
}
