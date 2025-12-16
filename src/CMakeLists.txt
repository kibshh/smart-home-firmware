# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

FILE(GLOB_RECURSE app_sources 
    ${CMAKE_SOURCE_DIR}/src/*.c
    ${CMAKE_SOURCE_DIR}/drivers/**/*.c
)

idf_component_register(
    SRCS ${app_sources}
    INCLUDE_DIRS 
        ${CMAKE_SOURCE_DIR}/src
        ${CMAKE_SOURCE_DIR}/drivers/gpio
        ${CMAKE_SOURCE_DIR}/drivers/adc
        ${CMAKE_SOURCE_DIR}/drivers/dac
        ${CMAKE_SOURCE_DIR}/drivers/i2c
        ${CMAKE_SOURCE_DIR}/drivers/spi/master
        ${CMAKE_SOURCE_DIR}/drivers/spi/slave
)
