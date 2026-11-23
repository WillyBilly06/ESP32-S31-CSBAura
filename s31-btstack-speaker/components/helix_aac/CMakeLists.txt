# AAC-LC decoder for the A2DP sink: RealNetworks Helix fixed-point AAC decoder
# (RPSL, see the license block in each libhelix_aac source), LC core only - no
# SBR / HE-AAC - plus a small LATM parser (A2DP AAC is MPEG-4 LATM in RTP).
# Taken from the ESP32-S31-A2DP-Codecs Bluedroid fork.
idf_component_register(
    SRCS
        "aac_decoder_helix.c"
        "latm_parser.c"
        "libhelix_aac/aacdec.c"
        "libhelix_aac/aactabs.c"
        "libhelix_aac/bitstream.c"
        "libhelix_aac/buffers.c"
        "libhelix_aac/dct4.c"
        "libhelix_aac/decelmnt.c"
        "libhelix_aac/dequant.c"
        "libhelix_aac/fft.c"
        "libhelix_aac/filefmt.c"
        "libhelix_aac/huffman.c"
        "libhelix_aac/hufftabs.c"
        "libhelix_aac/imdct.c"
        "libhelix_aac/noiseless.c"
        "libhelix_aac/pns.c"
        "libhelix_aac/stproc.c"
        "libhelix_aac/tns.c"
        "libhelix_aac/trigtabs.c"
    INCLUDE_DIRS "."
    PRIV_INCLUDE_DIRS "libhelix_aac"
    PRIV_REQUIRES esp_common log heap)

# assembly.h picks its portable-C path with ESP32 defined (a platform selector,
# valid for the S31's RISC-V core too)
target_compile_definitions(${COMPONENT_LIB} PRIVATE "ESP32")
target_compile_options(${COMPONENT_LIB} PRIVATE -O2 -Wno-unused-variable -Wno-unused-but-set-variable
                       -Wno-unused-function -Wno-maybe-uninitialized)
