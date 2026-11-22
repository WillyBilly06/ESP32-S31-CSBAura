# ESP32-S31 CSB/CPB controller-capability probe.
#   idf.py set-target esp32s31 && idf.py build flash monitor
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(s31_csb_probe)
