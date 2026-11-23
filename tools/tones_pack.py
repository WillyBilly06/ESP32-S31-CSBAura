#!/usr/bin/env python3
"""Pack speaker prompt tones into the S31 'tones' flash partition image.

Input: a directory with tone files named after the prompt they replace (poweron.wav,
bt_pair.mp3, ...): WAV, MP3, FLAC, OGG, M4A/AAC, or raw 16 kHz s16 mono .pcm. Any rate,
mono or stereo; they are converted to 24 kHz mono.

    python tools/tones_pack.py <dir> tones.bin                    # a new set from <dir>
    python tools/tones_pack.py <dir> tones.bin --base tones.bin   # keep the set, replace
                                                                  # only the tones in <dir>

The S31 Flasher writes tones.bin from the project folder. By hand:
    esptool --chip esp32s31 -p COM5 write-flash 0x810000 tones.bin

Format (little-endian): "S31TONE1", u32 count, u32 sample rate, then count x
{char name[12], u32 offset, u32 samples}, then 16-bit mono PCM (4-aligned).
Needs ffmpeg on PATH.
"""
import argparse
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


EXTS = ('.wav', '.mp3', '.flac', '.ogg', '.m4a', '.aac', '.pcm')


def read_pack(path):
    """the tones of an existing tones.bin, as [(name, pcm)]"""
    d = open(path, 'rb').read()
    if d[:8] != b'S31TONE1':
        sys.exit(f'{path} is not a tones image')
    n, rate = struct.unpack_from('<II', d, 8)
    if rate != RATE:
        sys.exit(f'{path} is at {rate} Hz, not {RATE}')
    out = []
    for i in range(n):
        name, off, samples = struct.unpack_from('<12sII', d, 16 + 20 * i)
        out.append((name.split(b'\0')[0].decode(), d[off:off + samples * 2]))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('src', help='directory with tone files')
    ap.add_argument('dst', help='tones image to write')
    ap.add_argument('--base', help='existing tones image: keep its tones, replace those in src')
    a = ap.parse_args()
    src, dst = a.src, a.dst
    files = sorted(f for f in os.listdir(src) if f.lower().endswith(EXTS))
    new = [decode(os.path.join(src, f)) for f in files]
    for n, _ in new:
        if len(n) >= 12:
            print(f'skipping {n}: names must be shorter than 12 characters')
    new = [(n, d) for n, d in new if n not in SKIP and len(n) < 12]
    tones = dict(read_pack(a.base)) if a.base else {}
    replaced = [n for n, _ in new if n in tones]
    tones.update(new)
    tones = sorted(tones.items())
    if a.base:
        print(f'from {a.base}: replaced {", ".join(replaced) or "none"}; '
              f'added {", ".join(n for n, _ in new if n not in replaced) or "none"}')
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
