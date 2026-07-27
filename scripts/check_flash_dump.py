#!/usr/bin/env python3
"""Check that the SD card flash dump holds one continuous ADC sample stream.

The testbench ADC is a free-running sawtooth (tb_croc_soc.sv: 14-bit counter,
+1 per sample), and adc_acquisition_top.sv packs two samples per 32-bit word as
{2'b00, hi[13:0], 2'b00, lo[13:0]} -- low half is the *earlier* sample. So a
correct capture reads back as sample[n+1] == sample[n] + 1 (mod 2^14) straight
through, across block boundaries included. Any repeat, gap, or jump means
samples were dropped, duplicated, or written to the wrong LBA.

Usage:
    python3 scripts/check_flash_dump.py [verilator/sdcard/flash_dump.hex]

Exit status is 0 if the stream is continuous, 1 otherwise.
"""

import re
import sys

SAMPLE_BITS = 14
SAMPLE_MOD = 1 << SAMPLE_BITS
BLOCK_BYTES = 512

LINE_RE = re.compile(r"^([0-9a-fA-F]+):\s*([0-9a-fA-F]{8})$")


def parse(path):
    """[(byte_addr, word)] in file order."""
    words = []
    with open(path) as f:
        for lineno, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            m = LINE_RE.match(line)
            if not m:
                sys.exit(f"{path}:{lineno}: cannot parse {line!r}")
            words.append((int(m.group(1), 16), int(m.group(2), 16)))
    if not words:
        sys.exit(f"{path}: no data")
    return words


def written_extent(words):
    """Drop the all-zero tail -- flash the run never touched.

    sdModel.v always dumps a fixed 8 blocks regardless of how many were
    written, so without this every short capture reports thousands of bogus
    discontinuities in blank flash.
    """
    end = len(words)
    while end > 0 and words[end - 1][1] == 0:
        end -= 1
    return end


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "verilator/sdcard/flash_dump.hex"
    words = parse(path)
    n = written_extent(words)

    print(f"{path}: {len(words)} words dumped, {n} up to the last non-zero "
          f"({n * 4} B = {n * 4 / BLOCK_BYTES:.2f} blocks)")
    if n == 0:
        print("FAIL: dump is entirely zero -- nothing was written")
        return 1

    # Unpack into a flat sample stream, remembering where each came from.
    stream = []  # (byte_addr, half, sample)
    padded = []
    for addr, word in words[:n]:
        for half, shift in (("lo", 0), ("hi", 16)):
            field = (word >> shift) & 0xFFFF
            if field >> SAMPLE_BITS:
                padded.append((addr, half, field))
            stream.append((addr, half, field & (SAMPLE_MOD - 1)))

    # Zero words are worth calling out separately: a hole inside the written
    # region is a different failure from a plain miscount.
    zeros = [addr for addr, word in words[:n] if word == 0]

    def discontinuities(seq):
        out = []
        for i in range(1, len(seq)):
            prev, cur = seq[i - 1][2], seq[i][2]
            if cur != (prev + 1) % SAMPLE_MOD:
                out.append((seq[i][0], seq[i][1], prev, cur))
        return out

    breaks = discontinuities(stream)
    # Same check with the all-zero words taken out. If this passes while the
    # full check fails, no ADC sample was lost -- the zero words were *inserted*
    # into an otherwise intact stream, which is a very different bug.
    zero_addrs = set(zeros)
    real_breaks = discontinuities([s for s in stream if s[0] not in zero_addrs])

    first, last = stream[0][2], stream[-1][2]
    print(f"samples: {len(stream)}, first=0x{first:04x}, last=0x{last:04x}")

    ok = True

    if padded:
        ok = False
        print(f"\nFAIL: {len(padded)} sample(s) have non-zero pad bits "
              f"(expected {{2'b00, sample[13:0]}}):")
        for addr, half, field in padded[:8]:
            print(f"  0x{addr:06x} {half}: 0x{field:04x}")

    if zeros:
        ok = False
        print(f"\nFAIL: {len(zeros)} all-zero word(s) inside the written region:")
        for addr in zeros[:8]:
            off = addr % BLOCK_BYTES
            where = "last word of the block" if off == BLOCK_BYTES - 4 else f"offset 0x{off:03x}"
            print(f"  0x{addr:06x}  (block {addr // BLOCK_BYTES}, {where})")
        if len(zeros) > 8:
            print(f"  ... and {len(zeros) - 8} more")
        per_block = {a % BLOCK_BYTES for a in zeros}
        if len(per_block) == 1:
            print(f"  -> all at the same block offset 0x{per_block.pop():03x}: "
                  f"systematic, not random corruption")

    if not breaks:
        print("\nsample stream is continuous (+1 throughout, block boundaries "
              "included)")
    elif zeros and not real_breaks:
        # Every break is attributable to a zero word; the surrounding samples
        # still run +1 into each other.
        real_per_block = (BLOCK_BYTES // 4) - len(
            {a % BLOCK_BYTES for a in zeros})
        print("\nno ADC samples were lost: ignoring the zero words, the stream "
              "is continuous")
        print(f"  the zero words are *inserted*, not overwriting data -- each "
              f"{BLOCK_BYTES} B block")
        print(f"  carries {real_per_block} real words + "
              f"{len({a % BLOCK_BYTES for a in zeros})} filler, so the payload "
              f"is offset by 4 B more per block.")
        print("  Points at the fill side ending a frame one word early rather "
              "than at the copy engine.")
    else:
        ok = False
        print(f"\nFAIL: {len(real_breaks)} discontinuit(y/ies) in the sample "
              f"stream (zero words excluded) -- samples were lost, duplicated, "
              f"or written to the wrong LBA:")
        for addr, half, prev, cur in real_breaks[:16]:
            delta = (cur - prev) % SAMPLE_MOD
            note = ""
            if addr % BLOCK_BYTES == 0:
                note = f"  <- first word of block {addr // BLOCK_BYTES}"
            print(f"  0x{addr:06x} {half}: 0x{prev:04x} -> 0x{cur:04x} "
                  f"(+{delta}){note}")
        if len(real_breaks) > 16:
            print(f"  ... and {len(real_breaks) - 16} more")

    if ok:
        print("\nPASS")
        return 0
    print("\nFAIL")
    return 1


if __name__ == "__main__":
    sys.exit(main())
