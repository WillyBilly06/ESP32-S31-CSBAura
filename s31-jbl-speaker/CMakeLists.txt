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
# Every component compiled from source (app, Bluedroid, LE Audio host,
# FreeRTOS, IDF): float a/b -> a*(1/b) where it helps (rounding-only change;
# keeps IEEE NaN/Inf/ordering, unlike full -ffast-math). Integer division by
# constants is already strength-reduced to multiply/shift at -O2; prebuilt
# blobs (controller, PHY, coex, libc/libm) are not affected.
idf_build_set_property(COMPILE_OPTIONS "-freciprocal-math" APPEND)
# Stay BR/EDR master: refuse role switches (idf-patches/0007) so the S31's
# controller schedules the phone's A2DP traffic around the Auracast BIG.
idf_build_set_property(COMPILE_DEFINITIONS "BTM_NO_ROLE_SWITCH=1" APPEND)
# No sniff mode on BR/EDR (idf-patches/0007): sniff anchors phase-lock to the
# 20 ms BIG and get starved -> supervision timeouts ~8 s after connect.
idf_build_set_property(COMPILE_DEFINITIONS "BTM_NO_SNIFF_MODE=1" APPEND)
# A2DP media MTU 675 = 2-DH5 payload (679) - L2CAP header (4): at bitpool 53
# the phone packs 5 SBC frames (~608 B) per packet, so every media packet is
# ONE baseband exchange that fits between BIG events (1008 needed two).
idf_build_set_property(COMPILE_DEFINITIONS "BTA_AV_MAX_A2DP_MTU=675" APPEND)
project(s31_jbl_speaker)
