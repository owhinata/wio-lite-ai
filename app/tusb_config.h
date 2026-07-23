/*
 * TinyUSB configuration for the Wio Lite AI ThreadX shell app (CDC-only console).
 *
 * Same single USB peripheral as the bootloader: USB1_OTG_HS in Full-Speed on its
 * internal PHY (roothub port 0; on H72x/H73x rhport 0 aliases to OTG_HS base +
 * OTG_HS_IRQn).  CFG_TUSB_MCU is supplied by the build
 * (-DCFG_TUSB_MCU=OPT_MCU_STM32H7).  Unlike boot/tusb_config.h this drops DFU --
 * the app only exposes a CDC serial console.
 */

#ifndef TUSB_CONFIG_H_
#define TUSB_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

//--------------------------------------------------------------------
// COMMON
//--------------------------------------------------------------------
#ifndef CFG_TUSB_MCU
#error "CFG_TUSB_MCU must be defined (expected OPT_MCU_STM32H7)"
#endif

// v1 pumps tud_task() from a dedicated ThreadX thread (the sole caller of the
// TinyUSB device API); the OSAL stays OPT_OS_NONE.  OPT_OS_THREADX is a later
// optimization (lib/tinyusb/src/osal/osal_threadx.h).
#define CFG_TUSB_OS            OPT_OS_NONE

#ifndef CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG        0
#endif

// USB DMA: the FS internal PHY uses the dedicated FIFO (no system-memory DMA),
// so no cache-coherency alignment section is needed.  Pin dwc2 to slave/FIFO mode
// explicitly (not just the TinyUSB default): the app runs with the D-cache ENABLED
// and no MPU, so its no-DMA-into-SRAM safety rests on this -- a future TinyUSB bump
// flipping the default must not silently enable USB DMA (that would need cache
// maintenance / MPU non-cacheable buffers).  See SCB_EnableDCache() in app/main.c.
#ifndef CFG_TUD_DWC2_DMA_ENABLE
#define CFG_TUD_DWC2_DMA_ENABLE   0
#endif
#ifndef CFG_TUD_DWC2_SLAVE_ENABLE
#define CFG_TUD_DWC2_SLAVE_ENABLE 1
#endif
#ifndef CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_SECTION
#endif
#ifndef CFG_TUSB_MEM_ALIGN
#define CFG_TUSB_MEM_ALIGN    __attribute__ ((aligned(4)))
#endif

//--------------------------------------------------------------------
// DEVICE
//--------------------------------------------------------------------
#define CFG_TUD_ENABLED       1

// Roothub port 0 -> USB1_OTG_HS (internal FS PHY).
#ifndef BOARD_TUD_RHPORT
#define BOARD_TUD_RHPORT      0
#endif

// CRITICAL for H72x/H73x: the OTG_HS core advertises a HS PHY in GHWCFG2, but the
// board only has the internal FS PHY.  Forcing FULL speed makes TinyUSB take the
// phy_fs_init() path (GCCFG.PWRDWN) instead of bringing up a HS PHY.
#define CFG_TUD_MAX_SPEED     OPT_MODE_FULL_SPEED

#define CFG_TUD_ENDPOINT0_SIZE   64

//------------- CLASS -------------//
#define CFG_TUD_CDC          1        // serial console (shell / printf)
#define CFG_TUD_MSC          0
#define CFG_TUD_HID          0
#define CFG_TUD_MIDI         0
#define CFG_TUD_VENDOR       0
#define CFG_TUD_DFU          0        // DFU lives in the bootloader, not the app
#define CFG_TUD_DFU_RUNTIME  0

// CDC FIFO sizes (TX generous so bursty output isn't dropped).
// RX raised from 64 for issue #19 M5: `wifi imgload` receives a multi-megabyte
// firmware image over YMODEM, and this FIFO is what absorbs OUT packets between
// runs of the backend pump that moves them into the shell's RX ring.  At 64 B
// (one packet) the host is NAKed after every single packet; 512 B lets a block
// stream in while the pump catches up.  It does not replace the ring itself --
// see CLI_USBCDC_RX_BUFFER_SIZE in CMakeLists.txt.
#define CFG_TUD_CDC_RX_BUFSIZE   512
#define CFG_TUD_CDC_TX_BUFSIZE   512
#define CFG_TUD_CDC_EP_BUFSIZE   64

#ifdef __cplusplus
}
#endif

#endif /* TUSB_CONFIG_H_ */
