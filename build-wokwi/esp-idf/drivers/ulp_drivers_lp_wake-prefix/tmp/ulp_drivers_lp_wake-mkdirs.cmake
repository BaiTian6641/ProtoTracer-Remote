# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file LICENSE.rst or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "C:/esp/v6.0/esp-idf/components/ulp/cmake")
  file(MAKE_DIRECTORY "C:/esp/v6.0/esp-idf/components/ulp/cmake")
endif()
file(MAKE_DIRECTORY
  "C:/Users/weyst/Documents/ProtoTracer-Remote/firmware/build-wokwi/esp-idf/drivers/ulp_drivers_lp_wake"
  "C:/Users/weyst/Documents/ProtoTracer-Remote/firmware/build-wokwi/esp-idf/drivers/ulp_drivers_lp_wake-prefix"
  "C:/Users/weyst/Documents/ProtoTracer-Remote/firmware/build-wokwi/esp-idf/drivers/ulp_drivers_lp_wake-prefix/tmp"
  "C:/Users/weyst/Documents/ProtoTracer-Remote/firmware/build-wokwi/esp-idf/drivers/ulp_drivers_lp_wake-prefix/src/ulp_drivers_lp_wake-stamp"
  "C:/Users/weyst/Documents/ProtoTracer-Remote/firmware/build-wokwi/esp-idf/drivers/ulp_drivers_lp_wake-prefix/src"
  "C:/Users/weyst/Documents/ProtoTracer-Remote/firmware/build-wokwi/esp-idf/drivers/ulp_drivers_lp_wake-prefix/src/ulp_drivers_lp_wake-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/weyst/Documents/ProtoTracer-Remote/firmware/build-wokwi/esp-idf/drivers/ulp_drivers_lp_wake-prefix/src/ulp_drivers_lp_wake-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Users/weyst/Documents/ProtoTracer-Remote/firmware/build-wokwi/esp-idf/drivers/ulp_drivers_lp_wake-prefix/src/ulp_drivers_lp_wake-stamp${cfgdir}") # cfgdir has leading slash
endif()
