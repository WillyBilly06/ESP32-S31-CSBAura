#!/usr/bin/env python3
"""Pack speaker prompt tones into the S31 'tones' flash partition image.

Input: a directory with the tone files (MP3, plus raw 16 kHz s16 mono .pcm),
e.g. the extracted re/local_tones/sdfs_v26.40.10 (local only, never committed).
Output: a partition image to write at the 'tones' partition offset:

    python tools/tones_pack.py re/local_tones/sdfs_v26.40.10 re/local_tones/tones.bin
    esptool --chip esp32s31 -p COM5 write-flash 0x810000 re/local_tones/tones.bin

Format (little-endian): "S31TONE1", u32 count, u32 sample rate, then count x
{char name[12], u32 offset, u32 samples}, then 16-bit mono PCM (4-aligned).
Needs ffmpeg on PATH.
"""
import os
import struct
import subprocess
import sys

RATE = 24000
PART_SIZE = 0x200000
SKIP = {'offdelay'}                     # pure silence


def decode(path):
    name, ext = os.path.splitext(os.path.basename(path))
    src = ['-f', 's16le', '-ar', '16000', '-ac', '1'] if ext.lower() == '.pcm' else []
    out = subprocess.run(['ffmpeg', '-v', 'error', *src, '-i', path, '-ac', '1', '-ar', str(RATE),
                          '-f', 's16le', '-'], check=True, capture_output=True).stdout
    return name, out


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    src, dst = sys.argv[1], sys.argv[2]
    files = sorted(f for f in os.listdir(src) if f.lower().endswith(('.mp3', '.pcm')))
    tones = [decode(os.path.join(src, f)) for f in files]
    tones = [(n, d) for n, d in tones if n not in SKIP and len(n) < 12]
    head = 16 + 20 * len(tones)
    off = (head + 3) & ~3
    index, blob = b'', b''
    for name, pcm in tones:
        index += struct.pack('<12sII', name.encode(), off + len(blob), len(pcm) // 2)
        blob += pcm + b'\0' * (-len(pcm) % 4)
    img = b'S31TONE1' + struct.pack('<II', len(tones), RATE) + index
    img += b'\0' * (off - len(img)) + blob
    if len(img) > PART_SIZE:
        sys.exit(f'{len(img)} B does not fit the {PART_SIZE} B partition')
    open(dst, 'wb').write(img)
    total = sum(len(p) for _, p in tones) / 2 / RATE
    print(f'{len(tones)} tones, {total:.1f} s at {RATE} Hz, {len(img)} B -> {dst}')
    for name, pcm in tones:
        print(f'  {name:10s} {len(pcm) / 2 / RATE:5.2f} s')


if __name__ == '__main__':
    main()
