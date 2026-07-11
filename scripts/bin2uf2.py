#!/usr/bin/env python3
"""
Minimal raw-binary -> UF2 converter for the Wio Lite AI (TinyUF2 bootloader).

The TinyUF2 bootloader on this board writes .uf2 payloads to the external
OCTOSPI2 flash and validates them against family id 0x6db66082 (STM32H7) and the
0x70000000 + 8 MB address window (both confirmed by disassembly of the bootloader).

Usage:
    bin2uf2.py <in.bin> <out.uf2> [base_addr=0x70000000] [family_id=0x6db66082]
"""
import sys
import struct

UF2_MAGIC_START0 = 0x0A324655
UF2_MAGIC_START1 = 0x9E5D5157
UF2_MAGIC_END    = 0x0AB16F30
UF2_FLAG_FAMILY  = 0x00002000   # familyID present in bytes[28:32]

PAYLOAD = 256                   # bytes of real data per 512-byte UF2 block


def convert(data: bytes, base: int, family: int) -> bytes:
    if len(data) % PAYLOAD:
        data += b"\x00" * (PAYLOAD - (len(data) % PAYLOAD))
    numblocks = len(data) // PAYLOAD
    out = bytearray()
    for i in range(numblocks):
        chunk = data[i * PAYLOAD:(i + 1) * PAYLOAD]
        header = struct.pack(
            "<8I",
            UF2_MAGIC_START0, UF2_MAGIC_START1, UF2_FLAG_FAMILY,
            base + i * PAYLOAD,       # target address
            PAYLOAD,                  # payload size
            i,                        # block number
            numblocks,                # total blocks
            family)                   # family id
        block = header + chunk + b"\x00" * (476 - PAYLOAD) + struct.pack("<I", UF2_MAGIC_END)
        assert len(block) == 512
        out += block
    return bytes(out)


def main() -> int:
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    infile, outfile = sys.argv[1], sys.argv[2]
    base   = int(sys.argv[3], 0) if len(sys.argv) > 3 else 0x70000000
    family = int(sys.argv[4], 0) if len(sys.argv) > 4 else 0x6db66082
    with open(infile, "rb") as f:
        data = f.read()
    uf2 = convert(data, base, family)
    with open(outfile, "wb") as f:
        f.write(uf2)
    print(f"{outfile}: {len(uf2)} bytes, {len(uf2)//512} blocks, "
          f"base=0x{base:08X}, family=0x{family:08X}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
