/*
 * USB descriptors for the Wio Lite AI ThreadX shell app -- a single CDC (ACM)
 * serial console.  Adapted from boot/usb_descriptors.c with the DFU interface
 * removed.  Self-contained (no bsp/board_api); the serial string is derived from
 * the STM32 96-bit unique ID.
 */

#include "tusb.h"

/* --- Device Descriptor -------------------------------------------------- */
/*
 * 0483:5740 is STMicroelectronics' standard Virtual COM Port (CDC ACM) ID -- the
 * natural identity for an STM32 CDC console, distinct from the bootloader's DFU
 * id (0483:DF11).  CDC uses an IAD, so the device declares the Miscellaneous /
 * Common / IAD class triple.
 */
#define USB_VID   0x0483
#define USB_PID   0x5740
#define USB_BCD   0x0200

tusb_desc_device_t const desc_device =
{
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = USB_BCD,
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

/* Invoked on GET DEVICE DESCRIPTOR */
uint8_t const *tud_descriptor_device_cb(void)
{
  return (uint8_t const *) &desc_device;
}

/* --- Configuration Descriptor ------------------------------------------- */

/* Interfaces: CDC control + data (grouped by an IAD). */
enum
{
  ITF_NUM_CDC = 0,
  ITF_NUM_CDC_DATA,
  ITF_NUM_TOTAL
};

/* Endpoints (OTG_HS FS internal PHY): CDC notification IN, CDC data OUT/IN. */
#define EPNUM_CDC_NOTIF   0x81
#define EPNUM_CDC_OUT     0x02
#define EPNUM_CDC_IN      0x82

#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)

/* String descriptor indices. */
enum {
  STRID_LANGID = 0,
  STRID_MANUFACTURER,
  STRID_PRODUCT,
  STRID_SERIAL,
  STRID_CDC,        /* 4: CDC interface name */
};

uint8_t const desc_configuration[] =
{
  TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),
  /* CDC: itf, str, notif EP, notif size, data OUT EP, data IN EP, data size */
  TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, STRID_CDC, EPNUM_CDC_NOTIF, 8,
                     EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),
};

/* Invoked on GET CONFIGURATION DESCRIPTOR */
uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
  (void) index;
  return desc_configuration;
}

/* --- String Descriptors ------------------------------------------------- */

static char const *string_desc_arr[] =
{
  (const char[]) { 0x09, 0x04 },      /* 0: language = English (0x0409) */
  "Seeed Studio",                     /* 1: Manufacturer */
  "Wio Lite AI ThreadX Shell",        /* 2: Product */
  NULL,                               /* 3: Serial (from the STM32 unique ID) */
  "Wio Lite AI console",              /* 4: CDC interface (STRID_CDC) */
};

/* STM32 96-bit unique device ID (RM0468 "Unique device ID register"). */
#define STM32_UUID_ADDR   ((uint32_t const *) 0x1FF1E800UL)

static uint16_t _desc_str[32 + 1];

/* Render the 96-bit UUID as 24 hex UTF-16 chars into out; returns the count. */
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

/* Invoked on GET STRING DESCRIPTOR */
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
      const char *str = string_desc_arr[index];
      if (str == NULL) return NULL;
      chr_count = strlen(str);
      size_t const max_count = sizeof(_desc_str) / sizeof(_desc_str[0]) - 1;
      if (chr_count > max_count) chr_count = max_count;

      for (size_t i = 0; i < chr_count; i++)
        _desc_str[1 + i] = str[i];
      break;
  }

  /* first byte length (incl. header), second byte string type */
  _desc_str[0] = (uint16_t) ((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
  return _desc_str;
}
