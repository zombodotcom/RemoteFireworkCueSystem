idf_component_register(
    SRCS "firing_box_main.cpp" "expander_driver.cpp"
    INCLUDE_DIRS "."
    REQUIRES fireworkcore esp_driver_i2c esp_timer log freertos
)
