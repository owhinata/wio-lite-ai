/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    st7789_rgb.h
 * @brief   ST7789 panel controller bring-up over a bit-banged 3-wire SPI that is
 *          multiplexed onto the LTDC's own red data lines (issue #7).
 *
 * The kit panel (BL28005-B / JS28019H, 2.8" 240x320) is NOT a dumb RGB panel: it
 * carries an ST7789 controller that powers up asleep, with its RGB interface
 * disabled, and it will not accept a single pixel until it has been configured
 * over a serial link.  With the LTDC alone the panel simply stays backlit and
 * blank -- which is exactly what it did, with the controller reporting a
 * perfectly healthy scanout and zero FIFO underruns, because the LTDC was
 * faithfully driving a panel that was not listening.
 *
 * The FPC has no dedicated serial pins.  Instead the ST7789's CS/SDA/SCL are
 * multiplexed onto the three lowest red data lines, so the same wires are a
 * configuration bus before the LTDC takes them over and RGB data afterwards:
 *
 *     CS  = PA1  (LTDC_R2)
 *     SDA = PH2  (LTDC_R0)
 *     SCL = PH3  (LTDC_R1)
 *     (PG7 = LTDC_CLK is also parked low as a GPIO during the sequence)
 *
 * The schematic only names these LCD_R0/R1/R2; that they are also the ST7789's
 * serial bus comes from the factory firmware's Arduino pin table (below).
 *
 * SDA and SCL do go back to the LTDC afterwards.  **CS does not** -- PA1 stays a
 * GPIO driven high for good, because leaving it under the LTDC lets scanned-out
 * red bits clock commands into the panel.  That is issue #43's second half; the
 * mechanism and the evidence are in ltdc_pin_cs_park() (ltdc_display.c).
 *
 * Frames are 9 bits, MSB first: a leading data/command bit (0 = command,
 * 1 = parameter) followed by the byte, clocked in on SCL's rising edge with CS
 * low for the frame and high between frames.
 *
 * Everything here -- the pin assignment, the frame format, the register values
 * and the 6.00 MHz pixel clock the LTDC pairs with it -- was recovered by
 * disassembling the board's factory Arduino firmware
 * (`_ref/wio_APP_0x70000000_*.bin`, which links Seeed's `RGBLCD.cpp`) and
 * cross-referencing Seeed's Arduino variant pin table for this board.  No
 * datasheet for this panel is in circulation; the factory image is the
 * specification.  Notable values from that image, in case they ever need
 * revisiting: RAMCTRL(0xB0) = 0x11,0xF0 puts the controller in RGB-interface
 * mode, RGBCTRL(0xB1) = 0xC0,0x02,0x14 selects DE mode with active-low sync,
 * COLMOD(0x3A) = 0x55 is 16 bit/pixel, and MADCTL(0x36) = 0x00 is the default
 * scan order.
 *
 * The sequence is CPU/GPIO only: no DMA, no interrupts, no ThreadX call.  It
 * takes ~245 ms, almost all of it two mandatory 120 ms settles (see below), and
 * is safe to run both from tx_application_define() alongside the rest of
 * ltdc_init() and from a thread, where HAL_Delay() merely spins.
 *
 * ---- The panel is NOT necessarily cold when this runs (issue #43) -----------
 *
 * The ST7789 has its own supply, so a software reset -- `crash`, `reboot`, a DFU
 * reboot -- restarts the MCU and leaves the panel awake: Sleep Out, DISPON, RGB
 * interface live.  The factory transcription was never written against that
 * state (an Arduino sketch only ever starts from power-on) and does not recover
 * from it: the panel went uniformly white and stayed white until the board was
 * unplugged, while the LTDC reported a perfectly healthy scanout.
 *
 * Two things follow, and both are in st7789_rgb.c:
 *
 *   - the sequence is prefixed with SWRESET + 120 ms, which puts the controller
 *     back into the state the transcription assumes; and
 *   - the serial pins are parked BEFORE the caller's RESX pulse, which is why
 *     st7789_rgb_pins_init() is separate from st7789_rgb_init().  Between the
 *     MCU reset and ltdc_init() those three lines are floating inputs while the
 *     panel is awake and listening, so a stray edge on SCL can leave its 9-bit
 *     shift register misaligned and every later command garbage.  Driving them
 *     to their idle levels first closes that window.
 */
#ifndef ST7789_RGB_H
#define ST7789_RGB_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Take CS/SDA/SCL (plus LTDC_CLK, parked low) away from the LTDC and drive them
 * as plain GPIO outputs at their idle levels -- CS and SCL high, SDA and the
 * pixel clock low -- so the panel sees a quiet, driven bus.
 *
 * **Call this BEFORE pulsing the panel reset**, not after: the point is that the
 * three serial lines are never floating while the ST7789 is powered and
 * listening (see the file header).  The GPIOA/GPIOG/GPIOH clocks must already
 * be enabled.  Every HAL_GPIO_Init here is per-port and read-modify-write, so
 * PG6 (OCTOSPI1_CS), which neighbours LTDC_CLK on PG7, is not disturbed.
 *
 * Idempotent, and safe to call while the LTDC is disabled but still owns the
 * alternate function: writing MODER hands the pins to the GPIO block.
 */
void st7789_rgb_pins_init(void);

/**
 * Run the ST7789 power-on sequence -- SWRESET, then the factory transcription --
 * and leave the controller awake with its RGB interface enabled and the display
 * on.  Takes ~245 ms, almost all of it the two mandatory 120 ms settles.
 *
 * Call order is fixed and every step matters:
 *
 *     st7789_rgb_pins_init();   // park the serial bus, still driven
 *     <pulse LCD_RST>           // caller's job
 *     st7789_rgb_init();        // this function
 *     <restore the LTDC AF>     // caller's job -- SDA/SCL only, NOT CS
 *
 * The pins are left as GPIO on return: this function deliberately does not hand
 * them back, so the caller decides when the LTDC takes over.  CS (PA1) is left
 * high by the last frame and must simply stay that way (see the file header).
 */
void st7789_rgb_init(void);

#ifdef __cplusplus
}
#endif

#endif /* ST7789_RGB_H */
