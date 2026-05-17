cmake_minimum_required(VERSION 3.16)

set(IDF_TARGET esp32c6 CACHE STRING "Target SoC")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(prototracer_remote)

spiffs_create_partition_image(storage data FLASH_IN_PROJECT)