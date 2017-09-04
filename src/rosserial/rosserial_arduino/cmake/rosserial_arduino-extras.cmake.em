cmake_minimum_required(VERSION 2.8.3)

@[if DEVELSPACE]@
set(ROSSERIAL_ARDUINO_TOOLCHAIN "@(CMAKE_CURRENT_SOURCE_DIR)/arduino-cmake/cmake/ArduinoToolchain.cmake")
@[else]@
set(ROSSERIAL_ARDUINO_TOOLCHAIN "${rosserial_arduino_DIR}/../arduino-cmake/cmake/ArduinoToolchain.cmake")
@[end if]@

