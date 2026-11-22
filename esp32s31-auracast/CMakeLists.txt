# Top-level project CMakeLists for the ESP32-S31 Auracast demo.
# Build:  idf.py set-target esp32s31  &&  idf.py build flash monitor
cmake_minimum_required(VERSION 3.16)

set(PROJECT_VER "0.1.0")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32s31_auracast)
