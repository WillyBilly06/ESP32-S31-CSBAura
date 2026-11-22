# ESP32-S31 JBL speaker on BTstack (Bluedroid replaced).
#   core 1: BT controller + BTstack run loop (HCI, L2CAP, A2DP/AVDTP, ATT, ISO)
#   core 0: audio DSP (SBC decode + resampler, LC3 encode, LC3 decode)
cmake_minimum_required(VERSION 3.22)
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../s31-jbl-speaker/components/liblc3")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
idf_build_set_property(MINIMAL_BUILD ON)
idf_build_set_property(COMPILE_OPTIONS "-freciprocal-math" APPEND)
project(s31_btstack_speaker)
