"""Aggregate an rv32sim PC dump per source line or per (inlined) function.

usage: python lineprof.py prog.elf pcdump.txt [addr2line] [top-N] [line|func]

Bench only, never part of the firmware.
"""
import subprocess
import sys
from collections import defaultdict

elf, dump = sys.argv[1], sys.argv[2]
a2l = sys.argv[3] if len(sys.argv) > 3 else "riscv32-esp-elf-addr2line"
top = int(sys.argv[4]) if len(sys.argv) > 4 else 40
mode = sys.argv[5] if len(sys.argv) > 5 else "line"

rows = [l.split() for l in open(dump)]
addrs = chr(10).join("0x" + r[0] for r in rows)
out = subprocess.run([a2l, "-f", "-e", elf], input=addrs,
                     capture_output=True, text=True).stdout.splitlines()
funcs, lines = out[0::2], out[1::2]

agg = defaultdict(lambda: [0, 0])
tot_c = 0
for r, fn, loc in zip(rows, funcs, lines):
    loc = loc.split(" (")[0].replace("\\", "/").split("/")[-1]
    key = fn if mode == "func" else loc
    agg[key][0] += int(r[1])
    agg[key][1] += int(r[2])
    tot_c += int(r[2])

for key, (i, c) in sorted(agg.items(), key=lambda kv: -kv[1][1])[:top]:
    print(f"{i:12d} {c:12d} {100.0 * c / tot_c:5.1f}%  {key}")
