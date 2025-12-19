# Main application component
set(app_sources
    "main.c"
)

idf_component_register(
    SRCS ${app_sources}
    INCLUDE_DIRS "."
    REQUIRES 
        gpio
        adc
        dac
        i2c
        spi_master
        spi_slave
        timer
        uart
)
