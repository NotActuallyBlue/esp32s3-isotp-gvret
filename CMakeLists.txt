cmake_minimum_required(VERSION 3.16.0)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ISOTP-BLE)
idf_build_set_property(COMPILE_OPTIONS "-Wno-error" APPEND)