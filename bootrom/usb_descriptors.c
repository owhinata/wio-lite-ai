/*
 * USB descriptors for the Wio Lite AI STANDALONE DFU bootloader.
 *
 * Composite DFU + CDC (serial console).  A single DFU-mode interface with one
 * alternate (alt 0) mapped to the external OCTOSPI2 flash at 0x70000000 -- the app
 * base, which the bootloader boots from.  Unlike the app-first boot/ firmware (which
 * targeted a scratch region to avoid self-destruction), this runs from internal flash
 * so alt 0 legitimately targets offset 0.  Target usage:  dfu-util -a 0 -D app.bin
 *
 * Adapted from lib/tinyusb .../examples/device/dfu, self-contained (no bsp/board_api)
 * with a serial derived from the STM32 96-bit unique ID.
 */

#include "tusb.h"
#include "class/dfu/dfu_device.h"

//--------------------------------------------------------------------+
// Device Descriptor
//--------------------------------------------------------------------+
// 0xCafe is TinyUSB's development VID; pair it with a DFU-only PID.  Fine for
// bench bring-up (dfu-util enumerates by VID/PID); revisit before any release.
#define USB_VID   0xCafe
#define USB_PID   0x4000
#define USB_BCD   0x0200

tusb_desc_device_t const desc_device =
{
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = USB_BCD,
    // Composite (DFU + CDC): CDC uses an IAD, so the device must declare the
    // Miscellaneous / Common / IAD class triple.
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = USB_VID,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0100,

    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,

    .bNumConfigurations = 0x01
};

// Invoked on GET DEVICE DESCRIPTOR
uint8_t const * tud_descriptor_device_cb(void)
{
  return (uint8_t const *) &desc_device;
}

//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------+

// One alternate per flash partition.  We expose a single partition (OCTOSPI2).
#define ALT_COUNT   1

// Interfaces: CDC (control + data, grouped by an IAD) then DFU.
enum
{
  ITF_NUM_CDC = 0,
  ITF_NUM_CDC_DATA,
  ITF_NUM_DFU,
  ITF_NUM_TOTAL
};

// Endpoints (OTG_HS FS internal PHY): CDC notification IN, CDC data OUT/IN.  DFU
// uses only the shared control endpoint (EP0).
#define EPNUM_CDC_NOTIF   0x81
#define EPNUM_CDC_OUT     0x02
#define EPNUM_CDC_IN      0x82

#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_DFU_DESC_LEN(ALT_COUNT))

// DFU functional attributes: accept downloads, allow uploads (host read-back
// verification), tolerate manifestation.
#define FUNC_ATTRS  (DFU_ATTR_CAN_DOWNLOAD | DFU_ATTR_CAN_UPLOAD | DFU_ATTR_MANIFESTATION_TOLERANT)

// String descriptor indices.
enum {
  STRID_LANGID = 0,
  STRID_MANUFACTURER,
  STRID_PRODUCT,
  STRID_SERIAL,
  STRID_CDC,        // 4: CDC interface name
  STRID_DFU_ALT0,   // 5: DFU alt 0 (runtime: JEDEC + self-test verdict)
};

uint8_t const desc_configuration[] =
{
  TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),
  // CDC: itf number, string index, notif EP, notif size, data OUT EP, data IN EP, data EP size
  TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, STRID_CDC, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),
  // DFU: itf number, alt count, starting string index, attributes, detach timeout (ms), transfer size
  TUD_DFU_DESCRIPTOR(ITF_NUM_DFU, ALT_COUNT, STRID_DFU_ALT0, FUNC_ATTRS, 1000, CFG_TUD_DFU_XFER_BUFSIZE),
};

// Invoked on GET CONFIGURATION DESCRIPTOR
uint8_t const * tud_descriptor_configuration_cb(uint8_t index)
{
  (void) index;
  return desc_configuration;
}

//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+

// The alt-0 interface name is filled in at runtime (JEDEC id + self-test verdict).
extern char g_dfu_alt0_str[];

static char const *string_desc_arr[] =
{
  (const char[]) { 0x09, 0x04 },      // 0: supported language = English (0x0409)
  "Seeed Studio",                     // 1: Manufacturer
  "Wio Lite AI DFU bootloader",       // 2: Product
  NULL,                               // 3: Serial (filled from the STM32 unique ID)
  "Wio Lite AI console",              // 4: CDC interface (STRID_CDC)
  NULL,                               // 5: DFU alt 0 -> g_dfu_alt0_str (runtime)
};

// STM32 96-bit unique device ID (RM0468 "Unique device ID register").
#define STM32_UUID_ADDR   ((uint32_t const *) 0x1FF1E800UL)

static uint16_t _desc_str[32 + 1];

// Render the 96-bit UUID as 24 hex UTF-16 chars into out; returns char count.
static uint8_t usb_serial_hex(uint16_t *out, size_t max_chars)
{
  static const char hex[] = "0123456789ABCDEF";
  uint8_t n = 0;
  for (int w = 0; w < 3 && (size_t)(n + 8) <= max_chars; w++)
  {
    uint32_t v = STM32_UUID_ADDR[w];
    for (int nib = 7; nib >= 0; nib--)
      out[n++] = (uint16_t) hex[(v >> (nib * 4)) & 0xF];
  }
  return n;
}

// Invoked on GET STRING DESCRIPTOR
uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
  (void) langid;
  size_t chr_count;

  switch (index)
  {
    case STRID_LANGID:
      memcpy(&_desc_str[1], string_desc_arr[0], 2);
      chr_count = 1;
      break;

    case STRID_SERIAL:
      chr_count = usb_serial_hex(_desc_str + 1, 32);
      break;

    default:
      if (index >= TU_ARRAY_SIZE(string_desc_arr)) return NULL;

      // alt-0 name is a runtime buffer (JEDEC id + self-test verdict).
      const char *str = (index == STRID_DFU_ALT0) ? g_dfu_alt0_str : string_desc_arr[index];
      if (str == NULL) return NULL;
      chr_count = strlen(str);
      size_t const max_count = sizeof(_desc_str) / sizeof(_desc_str[0]) - 1;
      if (chr_count > max_count) chr_count = max_count;

      for (size_t i = 0; i < chr_count; i++)
        _desc_str[1 + i] = str[i];
      break;
  }

  // first byte length (incl. header), second byte string type
  _desc_str[0] = (uint16_t) ((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
  return _desc_str;
}
