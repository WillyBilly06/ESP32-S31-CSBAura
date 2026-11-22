# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.22)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# "Trim" the build. Include the minimal set of components, main, and anything it depends on.
idf_build_set_property(MINIMAL_BUILD ON)
# JBL One talks to a real Xtreme 5 over GATT on Classic Bluetooth (BR/EDR), so
# compile Bluedroid's GATT-over-BR/EDR (ATT on L2CAP + SDP records) in.
idf_build_set_property(COMPILE_DEFINITIONS "CLASSIC_BT_GATT_INCLUDED=1" APPEND)
# NOTE: BTC_AV_SINK_PREFER_MASTER (idf-patches/0002) is deliberately NOT set:
# phones refuse the role switch, the LMP requests collide, and Bluedroid then
# drops the link to retry as master. As slave every Auracast SDU still goes out.
# Log the SBC bitpool the phone really sends (idf btc_a2dp_sink.c tap).
idf_build_set_property(COMPILE_DEFINITIONS "BTC_A2DP_SINK_BITPOOL_STATS=1" APPEND)
# Pin the phone's SBC bitpool at 53 (idf-patches/0005; this IDF fork ignores
# esp_a2d_sink_register_stream_endpoint, so the stack's own sink caps are used).
idf_build_set_property(COMPILE_DEFINITIONS "A2D_SBC_SINK_MIN_BITPOOL=53" APPEND)
# Decode all SBC frames of a media packet, then call the A2DP data callback
# once per packet instead of once per SBC frame (idf-patches/0006).
idf_build_set_property(COMPILE_DEFINITIONS "A2D_SBC_DECODE_BATCH=1" APPEND)
project(s31_jbl_speaker)
