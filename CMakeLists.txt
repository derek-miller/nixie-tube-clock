cmake_minimum_required(VERSION 2.8.4)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/ArduinoToolchain.cmake)
set(PROJECT_NAME NixieClock)
project(${PROJECT_NAME})

set(${CMAKE_PROJECT_NAME}_BOARD mega)
set(${CMAKE_PROJECT_NAME}_PORT /dev/tty.usbmodem14201)
set(${CMAKE_PROJECT_NAME}_SERIAL screen /dev/tty.usbmodem14201)

set(${CMAKE_PROJECT_NAME}_ARDLIBS Timezone Time NTPClient)
set(${CMAKE_PROJECT_NAME}_SKETCH NixieClock.ino)

set(mega.build.mcu atmega2560)
set(mega.upload.protocol wiring)
set(mega.upload.speed 115200)

generate_arduino_firmware(${CMAKE_PROJECT_NAME})
