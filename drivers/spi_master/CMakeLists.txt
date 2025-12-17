# Component name is set by directory name (spi_master)

idf_component_register(
    SRCS "src/spi_master_driver.c"
    INCLUDE_DIRS "inc"
    REQUIRES driver
)

