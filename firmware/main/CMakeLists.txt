idf_component_register(
    SRCS "firing_box_main.cpp" "expander_driver.cpp" "arm_switch.cpp" "status_leds.cpp"
    INCLUDE_DIRS "."
    REQUIRES fireworkcore esp_driver_i2c esp_driver_gpio esp_timer log freertos led_strip
)
