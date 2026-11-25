# ESP32-S31 JBL speaker on BTstack (Bluedroid replaced).
#   core 1: BT controller + BTstack run loop (HCI, L2CAP, A2DP/AVDTP, ATT, ISO)
#   core 0: audio DSP (SBC decode + resampler, LC3 encode, LC3 decode)
cmake_minimum_required(VERSION 3.22)
# LC3 (liblc3), AAC (helix_aac) and BTstack (btstack_s31) live in components/
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
idf_build_set_property(MINIMAL_BUILD ON)
idf_build_set_property(COMPILE_OPTIONS "-freciprocal-math" APPEND)
# BT controller LP timer on the main crystal (IDF fork btdm_lp.c): the default
# 136 kHz RC is not accurate enough for BIG/ISO timing.
idf_build_set_property(COMPILE_DEFINITIONS "BTDM_LPCLK_FORCE_MAIN_XTAL=1" APPEND)
# S31_DEV=1: development build (serial console commands, INFO logs, BTstack error log, 2 s
# statistics). Default 0 = the release build S31Flasher makes: errors only, no console.
# Pass it on every build (idf.py -DS31_DEV=1 build): a CMake cache variable sticks otherwise.
if(NOT DEFINED S31_DEV)
    set(S31_DEV 0)
endif()
idf_build_set_property(COMPILE_DEFINITIONS "S31_DEV=${S31_DEV}" APPEND)
project(s31_btstack_speaker)
