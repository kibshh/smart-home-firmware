idf_component_register(
    SRCS "src/adc_driver.c"
    INCLUDE_DIRS "inc"
    REQUIRES esp_adc hal
)
