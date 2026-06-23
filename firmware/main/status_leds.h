#pragma once
#include "led_strip.h"
#include "box_controller.h"

class StatusLeds {
public:
    StatusLeds(int gpio, int count) : gpio_(gpio), count_(count) {}
    void begin();
    void show(fw::BoxState state, bool estopped, bool linkAlive, uint32_t nowMs);
private:
    led_strip_handle_t strip_ = nullptr;
    int gpio_, count_;
};
