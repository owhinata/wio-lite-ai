/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    ltdc_display.h
 * @brief   FPC-40 RGB panel bring-up via LTDC + DMA2D (issue #7).
 *
 * Drives the LCD on the board's 40-pin FPC connector through the STM32H725's
 * LTDC, with a DMA2D (Chrom-ART) accelerated, tear-free double buffer living in
 * the external OCTOSPI1 PSRAM.  Ported from ../stm32f746g-disco
 * (port/ltdc/ltdc_display.c) with three substantial changes forced by this
 * board; each is spelled out below because getting any of them wrong is either a
 * dead USB console or a hung AXI read.
 *
 * ---- 1. Pixel clock: pll3_r_ck, and PLL3 must be STOPPED to retune it -------
 *
 * RM0468 Figure 55: ltdc_ker_ck is wired straight to pll3_r_ck -- there is no
 * kernel-clock mux for the LTDC.  And RM0468 sec 8.7.16: DIVR3 "can be written
 * only when the PLL3 is disabled (PLL3ON = 0 and PLL3RDY = 0)"; the same holds
 * for DIVR3EN in RCC_PLLCFGR (sec 8.7.11).
 *
 * The DFU bootloader leaves PLL3 running as M=5 N=48 -> VCO 240 MHz, Q=5 ->
 * 48 MHz for USB (boot/clock.c).  **That 48 MHz is the USB CDC console, i.e. the
 * only way to see this board.**  So retuning the LTDC clock necessarily stops the
 * console's clock source for the duration.  Two rules follow, and both are
 * load-bearing:
 *
 *   a) Do NOT use HAL_RCCEx_PeriphCLKConfig(RCC_PERIPHCLK_LTDC).  Its
 *      RCCEx_PLL3_Config() (stm32h7xx_hal_rcc_ex.c) disables PLL3 and then
 *      rewrites M/N/P/Q/R *from the caller's struct*.  That turns the
 *      bootloader's values into something this file would have to hardcode, and
 *      any divergence permanently breaks USB.  ltdc_clock_init() instead does a
 *      bare read-modify-write that touches the DIVR3 field and DIVR3EN only --
 *      M/N/P/Q/FRACN/RGE/VCOSEL are written back bit-for-bit as read, so they
 *      cannot diverge from whatever the bootloader chose.
 *
 *   b) Call ltdc_clock_init() from main() BEFORE usb_hw_init(), while TinyUSB has
 *      not enumerated and OTG_HS is not even clocked.  The PLL3 outage is then a
 *      few tens of microseconds that nothing observes.  (Beware: in app/main.c
 *      usb_hw_init() runs BEFORE psram_hw_init(), so "next to the PSRAM bring-up"
 *      would be too late.)
 *
 * It follows that the pixel clock cannot be retuned at run time at all: there is
 * no safe moment to stop PLL3 once the console is up.  LTDC_CLK_DIV is therefore
 * a build-time constant, and changing it means a reflash.
 *
 * Divider arithmetic, which is off-by-one in the hardware: RM0468 sec 8.7.16
 * encodes DIVR3=0 as /1 and DIVR3=1 as /2.  LTDC_CLK_DIV is the *divide ratio*;
 * ltdc_clock_init() subtracts one on its way into the register.
 *
 * ---- 2. Frame buffers live in PSRAM, and that makes OCTOSPI1 shared ---------
 *
 * AXI-SRAM is full (this firmware's .bss alone is ~268 KB of 320 KB), so the two
 * frame buffers go to the external OCTOSPI1 APS6408 PSRAM at 0x90000000, in the
 * .psram_noinit section.  app/mpu.c region 0 already maps that whole window as
 * Normal **non-cacheable**, shareable, XN, so the CPU, the DMA2D and the LTDC's
 * read DMA are coherent by construction with no cache maintenance.
 *
 * The consequence is that the LTDC becomes a *continuous reader* of OCTOSPI1.
 * Anything that puts OCTOSPI1 back into indirect mode or retunes it (the `psram`
 * diagnostics: clk / set / mr0 / phase / wtune / mmapscan) while scanout is
 * running will starve the LTDC FIFO -- and a memory-mapped read of a
 * mis-configured OCTOSPI can stall the AXI indefinitely, which on this board is
 * only recoverable by the IWDG.  app/psram.c therefore refuses to hand out its
 * OCTOSPI1 guard while ltdc_scanout_active(), and tells the user to run
 * `lcd off`.  Keep that dependency one-way: **this module must not include
 * psram.h** (the caller checks psram_ready() before calling ltdc_init()).
 *
 * ---- 3. The panel needs a serial init before it accepts any pixel -----------
 *
 * The kit panel is labelled BL28005-B / JS28019H, a 2.8" 240x320 unit with no
 * datasheet in circulation.  It is NOT a dumb RGB panel: it carries an ST7789
 * controller that boots asleep with its RGB interface disabled, and its
 * CS/SDA/SCL are multiplexed onto LTDC_R2/R0/R1 -- which is why the FPC appears
 * to have no serial pins.  Until that sequence runs, the panel ignores every
 * pixel and stays backlit and blank *while the LTDC reports a perfectly healthy
 * scanout with zero FIFO underruns*.  See port/ltdc/st7789_rgb.h; ltdc_init()
 * runs it between the reset pulse and the alternate-function handover.
 *
 * And the panel does not lose that state when the MCU does: it has its own
 * supply, so a software reset hands the sequence an already-awake controller.
 * That is issue #43 -- the sequence needs a SWRESET prefix to be replayable, and
 * ltdc_panel_recover() exists so it can be replayed at all without a power
 * cycle.
 *
 * The same multiplexing has a second, nastier consequence: with RGB565 the LTDC
 * pads red by *replicating* its top bits into R[2:0] (RM0468 sec 38.3.2), so
 * scan-out drives CS/SCL/SDA with the top three bits of every pixel's red -- and
 * a live camera frame clocks real commands into the panel that way, blanking it.
 * PA1 (R2 = CS) is therefore kept out of the LTDC and parked high; see
 * ltdc_pin_cs_park() in ltdc_display.c.
 *
 * The timing, polarity and 6.00 MHz pixel clock were likewise recovered from the
 * board's factory firmware rather than guessed, so they are constants here.
 *
 * ---- Carried over from the f746 port ---------------------------------------
 *
 * Tear-free present: ltdc_flip() stages the back buffer with
 * HAL_LTDC_SetAddress_NoReload(), asks for a vertical-blanking reload, and
 * commits the swap only once the hardware has actually reloaded (SRCR.VBR reads
 * back 0 -- it self-clears after the HW reload).  A reload-ready IRQ wakes the
 * waiter promptly; the VBR poll is the authoritative truth.  A reload that never
 * lands latches the display faulted (fail-closed): the front buffer is left
 * alone and ltdc_is_up() goes false.
 *
 * DMA2D: register-to-memory fills and memory-to-memory blits.  Small transfers
 * poll; large ones (>= LTDC_DMA2D_IT_MIN_PIXELS) block on a completion semaphore
 * posted by DMA2D_IRQHandler so the waiting thread yields instead of spinning.
 *
 * GUIX ownership from the f746 version is deliberately NOT ported -- this board
 * uses the panel for camera preview plus overlays only, so there is no second
 * owner to arbitrate with.
 *
 * Clean-room implementation; RM0468 sec 8 (RCC) / sec 38 (LTDC) and the board
 * schematic were used as a register/pin reference only.
 */
#ifndef LTDC_DISPLAY_H
#define LTDC_DISPLAY_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Error returns (negative); 0 is success. */
#define LTDC_OK          0
#define LTDC_ERR_HAL    -1   /* a HAL init step (LTDC/layer) failed            */
#define LTDC_ERR_STATE  -2   /* wrong state: clock down, init failed, faulted  */

/* Frame-buffer capacity, in pixels, per buffer.  Two are allocated:
 * 2 * 76800 * 2 B = 300 KB of PSRAM. */
#define LTDC_FB_PIXELS   (240u * 320u)

/* ---- Pixel clock ---------------------------------------------------------- */

/* PLL3 VCO as the bootloader leaves it: HSE 25 MHz / M5 * N48 = 240 MHz
 * (boot/clock.c).  Reported for `lcd info`; the code derives the actual rate
 * from the register, it does not trust this constant. */
#define LTDC_PLL3_VCO_HZ  240000000u

/* Divide ratio for pll3_r_ck (NOT the DIVR3 register value -- see the file
 * header): 240 MHz / 40 = 6.00 MHz, the rate the factory firmware used. */
#define LTDC_CLK_DIV      40u

/**
 * Point pll3_r_ck at the LTDC and enable it.
 *
 * **Call from main() before usb_hw_init(), and from nowhere else.**  It stops
 * PLL3 for the duration, which is also the USB kernel clock; doing it any later
 * takes the CDC console down with it.  Interrupts are masked across the
 * stop/retune/restart window and every wait is a bounded spin (no HAL_GetTick,
 * no scheduler).
 *
 * Idempotent; later calls return the first result.  Returns LTDC_OK,
 * LTDC_ERR_STATE if PLL3 was not already running (the bootloader contract is
 * broken -- nothing is touched), or LTDC_ERR_HAL if PLL3 failed to stop or
 * relock within the spin budget.
 */
int ltdc_clock_init(void);

/** Pixel clock actually programmed, in Hz, read back from RCC_PLL3DIVR; 0 before
 *  a successful ltdc_clock_init(). */
uint32_t ltdc_pixel_clock_hz(void);

/** The divide ratio in effect, read back from the hardware (not the DIVR3
 *  register field); 0 before a successful ltdc_clock_init(). */
uint32_t ltdc_clock_div(void);

/* ---- Panel geometry / timing ---------------------------------------------- */

/**
 * Active size and blanking of the panel, in pixels / lines.  These are the
 * *spec* numbers; the LTDC's "minus one" register encoding (RM0468 sec 38.4.1)
 * is applied internally, so callers never subtract anything.  Fixed at build
 * time from the values the factory firmware used (see the file header).
 */
struct ltdc_panel {
	uint16_t w, h;          /* active width / height                        */
	uint16_t hs, hbp, hfp;  /* HSYNC width, horizontal back / front porch   */
	uint16_t vs, vbp, vfp;  /* VSYNC width, vertical back / front porch     */
};

/** Copy the programmed panel timing, for `lcd info` and the drawing helpers. */
void ltdc_panel_get(struct ltdc_panel *out);

/** Refresh rate in centi-Hz (5992 = 59.92 Hz) for the programmed timing and
 *  pixel clock; 0 when down.  Centi-Hz keeps `lcd info` integer-only. */
uint32_t ltdc_refresh_chz(void);

/* ---- Bring-up / state ----------------------------------------------------- */

/* LTDC error-flag bits returned by ltdc_errors() (RM0468 sec 38.5). */
#define LTDC_ERRFLAG_FIFO_UNDERRUN  0x1u   /* FUIF   */
#define LTDC_ERRFLAG_TRANSFER_ERROR 0x2u   /* TERRIF */

/**
 * One-time bring-up: ThreadX objects, LTDC + DMA2D clock gates, GPIO (AF14
 * everywhere except PA15/LTDC_R3 which is AF9 and PA1/LTDC_R2 which stays a
 * GPIO held high -- see ltdc_pin_cs_park() -- plus the LCD_RST/LCD_BL outputs),
 * the panel reset pulse, one RGB565 layer pointed at frame buffer 0, the
 * reload-ready and DMA2D IRQs, then backlight on.  BOTH buffers are cleared to
 * black by the CPU before the layer comes up (.psram_noinit is NOLOAD, i.e.
 * undefined at reset -- and a DMA2D transfer is not allowed here, see below).
 *
 * Requires ltdc_clock_init() to have succeeded and the caller to have confirmed
 * psram_ready() -- this module deliberately does not include psram.h, to keep
 * the OCTOSPI1 arbitration a one-way app/psram.c -> port/ltdc dependency.
 *
 * **Safe to call from tx_application_define(), and constrained accordingly**: it
 * only *creates* ThreadX objects and never waits on one, and it runs no DMA2D
 * transfer (both frame buffers are cleared by a CPU loop).  It does spend about
 * 255 ms in HAL_Delay() -- the reset pulse plus the ST7789's two mandatory
 * 120 ms settles, the software-reset one and the sleep-out one (issue #43; see
 * st7789_rgb.h) -- which is fine there: SysTick already feeds HAL_IncTick()
 * (issue #12), and the IWDG is armed later in the same function.  The IRQs it
 * enables stay dormant until the first ltdc_flip(), which cannot happen before
 * the scheduler starts.
 *
 * Idempotent; on failure it cleans up (display off, HAL_LTDC_DeInit, objects
 * deleted) and leaves ltdc_is_up() false.
 *
 * 🔴 ON SUCCESS THE PANEL IS INITIALIZED BUT DARK (issue #53): the ST7789 has
 * been woken and the controller configured, but LTDCEN and the backlight are
 * off, so ltdc_is_up() is true while ltdc_scanout_active() is false until
 * ltdc_set_scanout(true) (`lcd on`).  Drawing entry points still write the back
 * buffer; ltdc_flip() refuses, because no VBR reload would ever arrive.
 * Deferring the WAKE-UP instead of the presentation would be a different and
 * much worse thing -- see the note at the top of this file about "LTDC healthy,
 * screen uniformly white".
 */
int ltdc_init(void);

/** Nonzero once ltdc_init() succeeded AND the display has not latched a reload
 *  fault (see ltdc_flip()).  False means drawing/flip are no-ops. */
bool ltdc_is_up(void);

/** True when the LTDC is up AND scanout is running, i.e. the controller is
 *  reading the frame buffer out of OCTOSPI1 right now.  **This is the condition
 *  app/psram.c uses to refuse its OCTOSPI1 guard** -- retuning the octal bus
 *  under a live scanout starves the LTDC FIFO and can stall the AXI.  Both
 *  false when the LTDC never came up, which is the correct "not scanning" case. */
bool ltdc_scanout_active(void);

/** True when the LTDC is up but scanout is currently stopped (`lcd off`). */
bool ltdc_scanout_off(void);

/**
 * Stop (false) / start (true) scanout: clears/sets LTDC_GCR.LTDCEN so the
 * controller stops/resumes fetching from PSRAM, and parks/raises the backlight.
 * `lcd off` is the prerequisite for any `psram` command that retunes OCTOSPI1.
 * While disabled ltdc_flip() refuses (no VBR reload would ever come, so it would
 * otherwise latch a spurious fault); direct draws still write the back buffer,
 * nothing is presented.  Returns LTDC_OK or LTDC_ERR_STATE.
 */
int ltdc_set_scanout(bool on);

/** Drive LCD_BL (PF5): the backlight, on/off.  LCD_RST (PG5) is released by
 *  ltdc_init() and not touched again during normal operation -- resetting the
 *  panel puts the ST7789 back to sleep, and its serial pins have become LTDC
 *  outputs by then, so the wake-up sequence cannot be resent piecemeal.
 *  ltdc_panel_recover() below is the one path that does the whole thing. */
void ltdc_backlight(bool on);

/**
 * Re-run the panel bring-up on a live system: stop scanout, take CS/SDA/SCL back
 * from the LTDC, pulse LCD_RST, resend the ST7789 sequence, hand the pins back,
 * and restore whatever scanout state was in effect (an `lcd off` stays off).
 *
 * This exists because the panel has its own supply: a software reset restarts
 * the MCU but leaves the ST7789 awake, and until issue #43 there was no way back
 * from a panel that had stopped accepting pixels except unplugging the board.
 * `lcd off` / `lcd on` cannot do it -- they only toggle LTDCEN and the backlight.
 *
 * **Thread context only, and it blocks for ~245 ms** (the ST7789's two mandatory
 * 120 ms settles), holding the frame lock throughout -- so drawing commands and
 * the camera preview thread wait behind it.
 *
 * The caller is responsible for the OCTOSPI1 guard: scanout stops and restarts
 * inside this call, so hold psram_acquire_shared() across it exactly as `lcd on`
 * does.  (This module does not include psram.h -- see the file header.)
 *
 * Returns LTDC_OK, or LTDC_ERR_STATE if the LTDC never came up or has latched a
 * reload fault (that one is still a reset).  There is no failure return from the
 * sequence itself: the ST7789 is write-only over this link, so the check is the
 * picture -- run `lcd bar` afterwards.
 */
int ltdc_panel_recover(void);

/** Read the sticky FIFO-underrun / transfer-error flags (RM0468 sec 38.5).  The
 *  hardware sets these regardless of the interrupt enables, so they work with no
 *  LTDC IRQ: a non-zero underrun bit is direct evidence that OCTOSPI1 could not
 *  feed scanout.  Returns a mask of LTDC_ERRFLAG_*; cleared afterwards when
 *  @p clear. */
uint32_t ltdc_errors(bool clear);

/** Base of the currently displayed (front) RGB565 frame buffer, or NULL when
 *  down.  Row-major, one u16 per pixel, no padding, stride = panel width.
 *  READ-ONLY -- the LTDC is reading it; draw into ltdc_back_buffer(). */
uint16_t *ltdc_framebuffer(void);

/** Base of the off-screen (back) buffer to draw into, or NULL when down.  Only
 *  valid while the caller holds ltdc_lock_frame() -- a flip from another thread
 *  swaps which buffer is "back". */
uint16_t *ltdc_back_buffer(void);

/** Index (0/1) of the currently displayed front buffer -- diagnostic only. */
uint8_t ltdc_active_buffer(void);

/*
 * Frame lock (a ThreadX mutex; recursive within one thread).  Hold it around a
 * draw-then-flip sequence so the back buffer cannot be swapped out underneath
 * and concurrent drawers do not tear each other.  The draw helpers and
 * ltdc_flip() take it internally too, so a handler can wrap several of them in
 * one lock for atomicity.  Thread-context only; never from an ISR.
 */
void ltdc_lock_frame(void);
void ltdc_unlock_frame(void);

/**
 * Tear-free present: stage the back buffer's address, request a
 * vertical-blanking reload, and commit the swap only once the hardware has
 * reloaded (SRCR.VBR reads 0).  Returns LTDC_OK; LTDC_ERR_STATE if the display
 * is down or scanout is off; LTDC_ERR_HAL if the reload never landed within the
 * timeout -- in which case the display is latched faulted (front unchanged,
 * ltdc_is_up() false; recovery is a reset).  Thread-context only.
 */
int ltdc_flip(void);

/* ---- DMA2D completion: interrupt-driven for large transfers ----------------
 * The engine is single and every transfer here runs under ltdc_lock, so at most
 * one is ever in flight.  Small transfers poll (the IT handoff would dominate
 * their few-microsecond run time); LARGE ones block the calling thread on a
 * completion semaphore posted by DMA2D_IRQHandler.
 */

/* Destination pixel count (w*h) at/above which a transfer goes interrupt-driven.
 * ~32 KB of RGB565 (a 128x128 block): full-screen ops and a QVGA camera blit
 * clear it, small rect fills do not. */
#define LTDC_DMA2D_IT_MIN_PIXELS  16384u

/* ---- Drawing into the back buffer (DMA2D-accelerated where noted) ----------
 * These draw into the back buffer only; they do NOT present.  Call ltdc_flip()
 * afterwards.  All are no-ops when the display is down.  Coordinates are clipped
 * to the surface.
 *
 * 🔴 COORDINATES ARE LANDSCAPE 320x240, NOT THE PANEL'S 240x320 (issue #38).
 * x is the long axis (0..319), y the short one (0..239).  The frame buffer keeps
 * a 240-pixel stride and the rotation happens in ltdc_display.c, using the
 * mapping the board's factory firmware used:
 *
 *     landscape (x, y)  ->  fb[240 * (319 - x) + y]
 *
 * There is deliberately no runtime rotation control: the angle is fixed at build
 * time, so ltdc_blit()'s source coordinates always mean one thing.  The panel
 * cannot rotate for us either -- MADCTL/MV and RAMCTRL/RM were both measured dead
 * on the RGB interface (st7789_rgb.c); do not go looking again.
 *
 * Rotation is FREE for every fill: a 90-degree rotation maps a rectangle to a
 * rectangle, so the DMA2D work is unchanged and only the arguments are permuted.
 * ltdc_blit() is the one primitive that transposes pixels (svc/gfx_rot).
 */

/* The drawing surface, i.e. the panel with its axes swapped. */
#define LTDC_SURFACE_W  LTDC_DEF_H   /* 320, the long axis */
#define LTDC_SURFACE_H  LTDC_DEF_W   /* 240 */

/** Drawing-surface size (landscape).  ltdc_panel_get() still reports the panel's
 *  own 240x320 geometry and timings -- these two answer different questions. */
uint16_t ltdc_surface_w(void);
uint16_t ltdc_surface_h(void);

/** Fill the whole back buffer with one RGB565 colour (DMA2D R2M). */
void ltdc_fill(uint16_t rgb565);

/** Fill an axis-aligned rectangle with one RGB565 colour (DMA2D R2M). */
void ltdc_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                    uint16_t rgb565);

/** Copy a tightly-packed RGB565 source bitmap (w*h u16) to (x,y) (DMA2D M2M). */
void ltdc_blit(const uint16_t *src, uint16_t x, uint16_t y,
               uint16_t w, uint16_t h);

/** Eight colour bars across the long axis -- verifies the RGB channel wiring and
 *  the RGB565
 *  bit order, which is the single most useful first image on a new panel. */
void ltdc_colorbar(void);

/** DMA2D M2M self-copy demo: left half over right half, within one buffer (a
 *  strided memory-to-memory blit).  Draw a pattern first, then flip. */
void ltdc_blit_demo(void);

/** Horizontal black-to-white gradient (CPU-drawn, per column) -- banding or
 *  shimmer here means a pixel-clock or porch problem. */
void ltdc_gradient(void);

/* Handy RGB565 constants for the `lcd` command. */
#define LTDC_RGB565_BLACK    0x0000u
#define LTDC_RGB565_BLUE     0x001Fu
#define LTDC_RGB565_GREEN    0x07E0u
#define LTDC_RGB565_CYAN     0x07FFu
#define LTDC_RGB565_RED      0xF800u
#define LTDC_RGB565_MAGENTA  0xF81Fu
#define LTDC_RGB565_YELLOW   0xFFE0u
#define LTDC_RGB565_WHITE    0xFFFFu

#ifdef __cplusplus
}
#endif

#endif /* LTDC_DISPLAY_H */
