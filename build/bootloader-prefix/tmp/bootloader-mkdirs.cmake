# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "F:/WorkSpace/Chip/Espressif/frameworks/esp-idf-v5.3.1/components/bootloader/subproject"
  "F:/WorkSpace/Codes/ESP32/enc28j60/build/bootloader"
  "F:/WorkSpace/Codes/ESP32/enc28j60/build/bootloader-prefix"
  "F:/WorkSpace/Codes/ESP32/enc28j60/build/bootloader-prefix/tmp"
  "F:/WorkSpace/Codes/ESP32/enc28j60/build/bootloader-prefix/src/bootloader-stamp"
  "F:/WorkSpace/Codes/ESP32/enc28j60/build/bootloader-prefix/src"
  "F:/WorkSpace/Codes/ESP32/enc28j60/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "F:/WorkSpace/Codes/ESP32/enc28j60/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "F:/WorkSpace/Codes/ESP32/enc28j60/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
