cmake_minimum_required(VERSION 3.22)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# Trim the build to only what main needs — avoids pulling the Classic/A2DP
# sources that don't compile in a BLE-only config on this tree.
idf_build_set_property(MINIMAL_BUILD ON)
project(s31_sniffer)
