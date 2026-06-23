#include "status_leds.h"

void StatusLeds::begin() {
    led_strip_config_t sc = {};
    sc.strip_gpio_num = gpio_;
    sc.max_leds = count_;
    led_strip_rmt_config_t rc = {};
    rc.resolution_hz = 10 * 1000 * 1000;
    led_strip_new_rmt_device(&sc, &rc, &strip_);
    led_strip_clear(strip_);
}

void StatusLeds::show(fw::BoxState state, bool estopped, bool linkAlive, uint32_t nowMs) {
    uint8_t r=0,g=0,b=0;
    if (estopped)                              { bool on = (nowMs / 150) % 2; r = on ? 60 : 0; }  // fast red blink
    else if (state == fw::BoxState::ARMED)     { g = 60; }                                        // green
    else if (!linkAlive)                       { r = 40; g = 20; }                                // amber = link lost
    else                                       { r = 60; }                                        // red = SAFE
    led_strip_set_pixel(strip_, 0, r, g, b);
    led_strip_refresh(strip_);
}
