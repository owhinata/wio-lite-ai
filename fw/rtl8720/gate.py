#!/usr/bin/env python3
"""Static gates that must pass before an RTL8720DN image is flashed -- issue #20, N1.

Board #2 is the only surviving board, so nothing gets programmed on the strength of
"the build succeeded".  Every check here answers a specific way the flash could be left
in a state we cannot talk to:

  1. header      the two sub-images really are AmebaD IMG2 images with the run addresses
                 the boot ROM jumps to (0x0c000020 KM0 / 0x0e000020 KM4)
  2. size        the write stays below 0x105000, the factory WiFi-settings sector that
                 holds the SSID and the plaintext PSK
  3. boot        the core's prebuilt km0/km4 boot images are byte-identical to what is
                 on the chip -- if they are, the boot sectors do not need reflashing and
                 the write can be limited to 0x6000, which is the low-risk path
  4. diff        how far the build drifted from the image currently on the chip (report
                 only -- a byte-identical rebuild is not expected)

Exit status is non-zero if any BLOCKING gate fails.  In particular gate 3 failing means
STOP: reflashing the boot sectors is the single most dangerous operation available to us
and must not happen as an automatic fallback (see README.md and issue #20).
"""

import argparse
import hashlib
import struct
import sys

# AmebaD flash layout as programmed by Seeed (confirmed against board #2 in issue #19).
KM0_BOOT_OFF = 0x0000
KM4_BOOT_OFF = 0x4000
IMAGE2_OFF = 0x6000
WIFI_SETTINGS_OFF = 0x105000  # factory SSID/PSK sector -- must stay out of every write

IMG2_MAGIC = b"81958711"  # ASCII, first 8 bytes of an AmebaD IMG2 header
KM0_RUN_ADDR = 0x0C000020
KM4_RUN_ADDR = 0x0E000020
KM0_IMAGE_LEN = 102400  # km0_image2_all.bin is a fixed-size prebuilt from the core

fails = []
warns = []


def ok(msg):
    print(f"  \033[1;32mPASS\033[0m  {msg}")


def bad(msg):
    print(f"  \033[1;31mFAIL\033[0m  {msg}")
    fails.append(msg)


def warn(msg):
    print(f"  \033[1;33mWARN\033[0m  {msg}")
    warns.append(msg)


def info(msg):
    print(f"        {msg}")


def read(path):
    with open(path, "rb") as f:
        return f.read()


def device_digest(blob):
    """The module's own checksum: sum of 32-bit little-endian words, mod 2^32.

    This is the `0x27` algorithm identified on hardware in issue #19 and reimplemented
    on the STM32 in `app/rtl8720_flash.c` (`rtl_dl_digest_*`).  Printing it here is what
    ties this gate report to the bytes that actually get programmed: `wifi imginfo`
    reports the digest of the staged image and `wifi flashwrite` prints it again as the
    host digest, so an operator who staged the wrong file sees a mismatch before
    confirming.  The staging buffer pads to 4 KB with 0xFF and digests the padded range,
    so pad the same way here.
    """
    pad = (-len(blob)) % 4096
    b = blob + b"\xff" * pad
    return sum(int.from_bytes(b[i:i + 4], "little")
               for i in range(0, len(b), 4)) & 0xFFFFFFFF


def check_header(blob, off, name, run_addr):
    """An IMG2 header is: magic[8], length u32, run address u32."""
    if len(blob) < off + 16:
        bad(f"{name}: truncated before its header at 0x{off:x}")
        return None
    magic = blob[off:off + 8]
    seg_len, seg_addr = struct.unpack_from("<II", blob, off + 8)
    if magic != IMG2_MAGIC:
        bad(f"{name}: magic {magic!r} at 0x{off:x}, expected {IMG2_MAGIC!r}")
    else:
        ok(f"{name}: IMG2 magic at 0x{off:x}")
    if seg_addr != run_addr:
        bad(f"{name}: run address 0x{seg_addr:08x}, expected 0x{run_addr:08x}")
    else:
        ok(f"{name}: run address 0x{seg_addr:08x}")
    # The payload plus the 16-byte header must fit in what we actually have.
    if off + 16 + seg_len > len(blob):
        bad(f"{name}: header claims 0x{seg_len:x} bytes but only "
            f"0x{len(blob) - off - 16:x} are present")
    else:
        ok(f"{name}: payload 0x{seg_len:x} bytes fits")
    return seg_len


def check_boot(name, prebuilt, stock, part_off, part_end):
    """Gate 3.  Compare on the boot image's own length, not the partition length.

    The boot images are far shorter than their 0x4000/0x2000 partitions and the rest of
    the partition is erased flash, so comparing a whole partition against a short file
    is meaningless.  Compare the real bytes, then require the remainder to be 0xFF.
    """
    n = len(prebuilt)
    on_chip = stock[part_off:part_off + n]
    if on_chip != prebuilt:
        diffs = [i for i in range(n) if on_chip[i] != prebuilt[i]]
        bad(f"{name}: core prebuilt ({n} B) DIFFERS from the chip at 0x{part_off:x} "
            f"({len(diffs)} bytes, first at +0x{diffs[0]:x})")
        return
    ok(f"{name}: core prebuilt ({n} B) is byte-identical to the chip at 0x{part_off:x}")
    tail = stock[part_off + n:part_end]
    if tail.count(0xFF) != len(tail):
        first = next(i for i, b in enumerate(tail) if b != 0xFF)
        bad(f"{name}: chip 0x{part_off + n:x}..0x{part_end:x} is not erased "
            f"(first non-0xFF at 0x{part_off + n + first:x})")
    else:
        ok(f"{name}: chip 0x{part_off + n:x}..0x{part_end:x} is erased (all 0xFF)")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--image", required=True, help="built km0_km4_image2.bin")
    ap.add_argument("--km0-boot", required=True, help="core prebuilt km0_boot_all.bin")
    ap.add_argument("--km4-boot", required=True, help="core prebuilt km4_boot_all.bin")
    ap.add_argument("--stock", required=True,
                    help="full-chip backup of board #2 (2 MB, from `wifi flashbackup`)")
    ap.add_argument("--offset", type=lambda s: int(s, 0), default=IMAGE2_OFF,
                    help="flash offset the image will be written to (default 0x6000)")
    args = ap.parse_args()

    img = read(args.image)

    print("\n\033[1mgate 1: IMG2 headers\033[0m")
    check_header(img, 0, "km0_image2", KM0_RUN_ADDR)
    check_header(img, KM0_IMAGE_LEN, "km4_image2", KM4_RUN_ADDR)

    print("\n\033[1mgate 2: write range\033[0m")
    end = args.offset + len(img)
    info(f"image {len(img)} B (0x{len(img):x}) -> flash 0x{args.offset:x}..0x{end - 1:x}")
    if len(img) % 4096:
        bad(f"length 0x{len(img):x} is not a multiple of the 4 KB sector size")
    else:
        ok("length is 4 KB sector aligned")
    if args.offset % 4096:
        bad(f"offset 0x{args.offset:x} is not 4 KB aligned")
    else:
        ok(f"offset 0x{args.offset:x} is 4 KB aligned")
    if args.offset < IMAGE2_OFF:
        bad(f"offset 0x{args.offset:x} reaches into the boot sectors (< 0x{IMAGE2_OFF:x})")
    else:
        ok(f"offset stays above the boot sectors (>= 0x{IMAGE2_OFF:x})")
    if end > WIFI_SETTINGS_OFF:
        bad(f"write ends at 0x{end:x}, past the factory WiFi settings sector "
            f"0x{WIFI_SETTINGS_OFF:x} (SSID + plaintext PSK)")
    else:
        ok(f"write ends at 0x{end:x}, clear of the WiFi settings sector "
           f"0x{WIFI_SETTINGS_OFF:x} ({WIFI_SETTINGS_OFF - end} B margin)")

    print("\n\033[1mgate 3: boot sectors vs the chip (BLOCKING)\033[0m")
    try:
        stock = read(args.stock)
    except OSError as e:
        bad(f"cannot read the full-chip backup: {e}")
        stock = None
    if stock is not None:
        if len(stock) < IMAGE2_OFF:
            bad(f"backup is only {len(stock)} B -- need at least the boot partitions")
            stock = None
        else:
            check_boot("km0_boot", read(args.km0_boot), stock, KM0_BOOT_OFF, KM4_BOOT_OFF)
            check_boot("km4_boot", read(args.km4_boot), stock, KM4_BOOT_OFF, IMAGE2_OFF)

    print("\n\033[1mgate 4: drift from the image on the chip (report only)\033[0m")
    if stock is not None and len(stock) >= args.offset + len(img):
        on_chip = stock[args.offset:args.offset + len(img)]
        diffs = [i for i in range(len(img)) if img[i] != on_chip[i]]
        if not diffs:
            info("byte-identical to the image currently on the chip")
        else:
            km0 = sum(1 for i in diffs if i < KM0_IMAGE_LEN)
            info(f"{len(diffs)} of {len(img)} bytes differ "
                 f"(km0 part {km0}, km4 part {len(diffs) - km0}); "
                 f"first at 0x{diffs[0]:x}, last at 0x{diffs[-1]:x}")
            if km0:
                warn(f"the km0 part differs in {km0} bytes -- it is a prebuilt from the "
                     "core, so a difference means the core version changed")
    else:
        info("no full-chip backup covering the image range -- skipped")

    print("\n\033[1msummary\033[0m")
    info(f"md5    {hashlib.md5(img).hexdigest()}")
    info(f"sha256 {hashlib.sha256(img).hexdigest()}")
    info(f"device digest 0x{device_digest(img):08X}  <- must match `wifi imginfo`")
    if fails:
        print(f"\n\033[1;31m{len(fails)} gate(s) FAILED -- do not flash\033[0m")
        for m in fails:
            print(f"  - {m}")
        print("\nIf the failure is gate 3 (boot sectors differ from the chip): STOP.\n"
              "Do NOT fall back to writing all three images at 0x0 automatically.\n"
              "Record the core version, the boot headers and the stock restore procedure\n"
              "as an explicit approval on issue #20 first -- see README.md.")
        return 1
    if warns:
        print(f"\n\033[1;33mall gates passed with {len(warns)} warning(s)\033[0m")
    else:
        print("\n\033[1;32mall gates passed\033[0m")
    print(f"\nflash with:  wifi imgload   (send {args.image} with `sb -k`)\n"
          f"             wifi imginfo   (check digest 0x{device_digest(img):08X} "
          f"and {len(img)} bytes)\n"
          f"             wifi flashwrite 0x{args.offset:x} confirm")
    print("\nThe gates above ran on that file only.  `wifi imgload` will stage whatever\n"
          "you send it, so compare the digest before typing `confirm`.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
