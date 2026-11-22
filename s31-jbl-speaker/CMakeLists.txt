# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.22)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# "Trim" the build. Include the minimal set of components, main, and anything it depends on.
idf_build_set_property(MINIMAL_BUILD ON)
# JBL One talks to a real Xtreme 5 over GATT on Classic Bluetooth (BR/EDR), so
# compile Bluedroid's GATT-over-BR/EDR (ATT on L2CAP + SDP records) in.
idf_build_set_property(COMPILE_DEFINITIONS "CLASSIC_BT_GATT_INCLUDED=1" APPEND)
# Ask for the BR/EDR master role on the A2DP link (BTA_AV_FEAT_MASTER) so our
# controller, not the phone, decides when Classic packets go - around the BIG.
idf_build_set_property(COMPILE_DEFINITIONS "BTC_AV_SINK_PREFER_MASTER=1" APPEND)
project(s31_jbl_speaker)
