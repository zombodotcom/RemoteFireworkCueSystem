#pragma once
#include "driver/gpio.h"

class ArmSwitch {
public:
    explicit ArmSwitch(int gpio) : gpio_(gpio) {}
    void begin();
    bool isOn();          // true = armed position (switch closed to active level)
private:
    int gpio_;
};
