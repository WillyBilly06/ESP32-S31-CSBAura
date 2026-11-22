# Vendored Google liblc3 (Apache-2.0) built as a local component so the app can
# decode LC3 in the host. Header lc3.h is exposed publicly; the src/*.h internals
# stay private.
file(GLOB LC3_SRCS "src/*.c")

idf_component_register(SRCS ${LC3_SRCS}
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "src")

# liblc3 uses some aggressive float math; silence non-fatal warnings.
# -O3 + unrolling: the encoder's MDCT/LTPF/SNS loops are the hot path on the S31's FPU.
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error -ffast-math -O3 -funroll-loops)
