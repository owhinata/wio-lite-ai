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
 * takes ~135 ms, almost all of it the mandatory sleep-out settle, and is safe to
 * run from tx_application_define() alongside the rest of ltdc_init().
 */
#ifndef ST7789_RGB_H
#define ST7789_RGB_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Configure CS/SDA/SCL (plus the parked LTDC_CLK) as plain GPIO outputs, run the
 * ST7789 power-on sequence, and leave the controller awake with its RGB
 * interface enabled and the display on.
 *
 * The caller MUST have pulsed the panel reset first, and MUST reconfigure these
 * pins back to their LTDC alternate function afterwards -- this function
 * deliberately leaves them as GPIO so the caller decides when the handover
 * happens.  The GPIOA/GPIOG/GPIOH clocks must already be enabled.
 */
void st7789_rgb_init(void);

#ifdef __cplusplus
}
#endif

#endif /* ST7789_RGB_H */
