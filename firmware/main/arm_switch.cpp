#include "arm_switch.h"

void ArmSwitch::begin() {
    gpio_config_t io = {};
    io.pin_bit_mask = 1ULL << gpio_;
    io.mode = GPIO_MODE_INPUT;
    io.pull_up_en = GPIO_PULLUP_ENABLE;     // switch closes to GND -> LOW = armed
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io);
}

bool ArmSwitch::isOn() {
    // Closed (armed) pulls the line LOW against the internal pull-up.
    return gpio_get_level((gpio_num_t)gpio_) == 0;
}
