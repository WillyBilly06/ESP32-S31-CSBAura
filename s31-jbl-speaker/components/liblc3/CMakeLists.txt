# Vendored Google liblc3 (Apache-2.0) built as a local component so the app can
# decode LC3 in the host. Header lc3.h is exposed publicly; the src/*.h internals
# stay private.
#
# Only src/*.c is compiled: bench/ (host benchmark, RV32 simulator and the
# pristine reference copy bench/orig/) is never part of the firmware.
file(GLOB LC3_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c")

# Memory placement of the 48 kHz / 10 ms encoder hot path (see the .lf files):
#   LC3_ENCODER_IRAM         hot encoder functions in IRAM    (~22.6 KB, ON)
#   LC3_ENCODER_TABLES_DRAM  per-frame encoder tables in DRAM (~23 KB,  ON)
# e.g. `idf.py -DLC3_ENCODER_TABLES_DRAM=ON build`
option(LC3_ENCODER_IRAM "liblc3: encoder hot functions in IRAM" ON)
option(LC3_ENCODER_TABLES_DRAM "liblc3: 48k/10ms encoder tables in DRAM" ON)

set(LC3_LDFRAGMENTS "")
if(LC3_ENCODER_IRAM)
    list(APPEND LC3_LDFRAGMENTS "linker_iram.lf")
endif()
if(LC3_ENCODER_TABLES_DRAM)
    list(APPEND LC3_LDFRAGMENTS "linker_dram.lf")
endif()

idf_component_register(SRCS ${LC3_SRCS}
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "src"
                       LDFRAGMENTS ${LC3_LDFRAGMENTS})

# liblc3 uses some aggressive float math; silence non-fatal warnings.
# -O3 + unrolling: the encoder's MDCT/LTPF/SNS loops are the hot path on the S31's FPU.
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error -ffast-math -O3 -funroll-loops)

# Never signal LTPF from the encoder (see src/lc3.c): removes the decoder-side
# pitch comb filter that garbles tones at <= 109 B/frame, keeping 100 B frames.
target_compile_definitions(${COMPONENT_LIB} PRIVATE LC3_ENCODER_NO_LTPF=1 LC3_ENCODER_FULLBAND=1 LC3_ENCODER_JBL_COMPAT=1 LC3_ENCODER_JBL_NF=1 LC3_ENCODER_JBL_PITCH=1 LC3_ENCODER_STATS=1)
