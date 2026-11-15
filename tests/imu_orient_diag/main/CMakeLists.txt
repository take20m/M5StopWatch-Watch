idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio esp_timer freertos log m5sw_board m5sw_power otool_bmi270 m5sw_test_support
)
