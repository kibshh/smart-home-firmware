cmake_minimum_required(VERSION 3.16.0)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Add drivers and src directories as extra component directories
# Note: ESP-IDF doesn't recurse into subdirectories, so we add each driver directory individually
set(EXTRA_COMPONENT_DIRS 
    "${CMAKE_SOURCE_DIR}/drivers/gpio"
    "${CMAKE_SOURCE_DIR}/drivers/adc"
    "${CMAKE_SOURCE_DIR}/drivers/dac"
    "${CMAKE_SOURCE_DIR}/drivers/i2c"
    "${CMAKE_SOURCE_DIR}/drivers/spi_master"
    "${CMAKE_SOURCE_DIR}/drivers/spi_slave"
    "${CMAKE_SOURCE_DIR}/drivers/timer"
    "${CMAKE_SOURCE_DIR}/drivers/pwm"
    "${CMAKE_SOURCE_DIR}/drivers/uart"
    "${CMAKE_SOURCE_DIR}/drivers/rmt"
    "${CMAKE_SOURCE_DIR}/src"
)

project(smart_home_firmware)
