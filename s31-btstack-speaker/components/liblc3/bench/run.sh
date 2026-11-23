#!/bin/sh
# liblc3 benchmark / correctness runner (bench only, never in firmware).
#
# Builds, into bench/out/:
#   - host_orig / host_opt : host harness against pristine (bench/orig) and
#                            optimized (../src) liblc3, clang -O2 -ffast-math
#   - rv32sim              : RV32IMFC instruction-set simulator / profiler
#   - rv_orig.elf / rv_opt.elf : the same encoder loop compiled by the
#                            ESP-IDF RISC-V gcc with the firmware's flags
# Then generates the test signal, encodes it with all four, and compares
# the bitstreams (decoded with the pristine decoder).
#
# Usage: sh run.sh [rv-seconds]     (default: 1 s through the simulator)
#
# Env overrides: CC (host clang/gcc), RVGCC (riscv32-esp-elf-gcc)

set -e
cd "$(dirname "$0")"
B=$(pwd)
O=$B/out
mkdir -p "$O"

CC=${CC:-clang}
RVGCC=${RVGCC:-riscv32-esp-elf-gcc}
EXE=
case "$(uname -s)" in MINGW*|MSYS*|CYGWIN*) EXE=.exe ;; esac

# Strict IEEE on the host: checks that the changes are algorithmically exact
# (with -ffast-math the host compiler reassociates the reference reductions).
HOSTFLAGS="-O2 -fno-fast-math -ffp-contract=off -std=gnu11 -w -D_USE_MATH_DEFINES"
RVFLAGS="-mabi=ilp32f -march=rv32imafc_zicsr_zifencei_zaamo_zalrsc_xesploop_xespv \
 -mtune=esp-base -ffunction-sections -fdata-sections -std=gnu23 -O2 \
 -fstrict-volatile-bitfields -fno-jump-tables -fno-tree-switch-conversion \
 -ffast-math -O3 -funroll-loops -w -g"

echo "== building host harness"
$CC $HOSTFLAGS -I"$B/orig/include" -I"$B/orig/src" "$B/bench.c" "$B"/orig/src/*.c -o "$O/host_orig$EXE"
$CC $HOSTFLAGS -I"$B/../include" -I"$B/../src" "$B/bench.c" "$B"/../src/*.c -o "$O/host_opt$EXE"
$CC -O2 -mfma -w "$B/rv32sim.c" -o "$O/rv32sim$EXE" 2>/dev/null || \
  $CC -O2 -w "$B/rv32sim.c" -o "$O/rv32sim$EXE"

echo "== building RV32 targets"
for v in orig opt; do
  if [ $v = orig ]; then L=$B/orig; else L=$B/..; fi
  $RVGCC $RVFLAGS -I"$L/include" -I"$L/src" -nostartfiles -T "$B/rv/link.ld" \
    "$B/rv/crt0.S" "$B/rv/rv_main.c" "$L"/src/*.c -o "$O/rv_$v.elf" -lc -lgcc
done

echo "== test signal"
"$O/host_orig$EXE" gen "$O/test.pcm" 6
RVSEC=${1:-1}
# first N seconds for the simulator (480 stereo s16 samples per frame)
head -c $((RVSEC * 48000 * 4)) "$O/test.pcm" > "$O/test_rv.pcm"

echo "== host encode (orig / opt)"
"$O/host_orig$EXE" enc "$O/test.pcm" "$O/host_orig.lc3" 10
"$O/host_opt$EXE"  enc "$O/test.pcm" "$O/host_opt.lc3" 10

echo "== host compare"
"$O/host_orig$EXE" cmp "$O/test.pcm" "$O/host_orig.lc3" "$O/host_opt.lc3"

echo "== RV32 simulation (${RVSEC}s): orig"
"$O/rv32sim$EXE" "$O/rv_orig.elf" "$O/test_rv.pcm" "$O/rv_orig.lc3" 2> "$O/prof_orig.txt"
head -25 "$O/prof_orig.txt"
echo "== RV32 simulation (${RVSEC}s): opt"
"$O/rv32sim$EXE" "$O/rv_opt.elf" "$O/test_rv.pcm" "$O/rv_opt.lc3" 2> "$O/prof_opt.txt"
head -25 "$O/prof_opt.txt"

echo "== RV32 compare (orig vs opt, firmware compiler)"
"$O/host_orig$EXE" cmp "$O/test_rv.pcm" "$O/rv_orig.lc3" "$O/rv_opt.lc3"
