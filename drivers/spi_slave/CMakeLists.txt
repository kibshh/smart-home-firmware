# Component name is set by directory name (spi_slave)

idf_component_register(
    SRCS "src/spi_slave_driver.c"
    INCLUDE_DIRS "inc"
    REQUIRES driver
)

