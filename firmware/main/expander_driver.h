#pragma once
#include "channel_driver.h"
#include "expander_codec.h"
#include "driver/i2c_master.h"
#include "esp_err.h"

class ExpanderChannelDriver : public fw::ChannelDriver {
public:
    ExpanderChannelDriver(i2c_master_bus_handle_t bus, uint8_t addr, fwbox::FireLevel lvl);
    esp_err_t begin();                 // add device, write all-OFF word
    void allOff() override;
    void setChannel(uint8_t channel, bool on) override;
private:
    esp_err_t writeWord(uint16_t word);
    i2c_master_dev_handle_t dev_ = nullptr;
    i2c_master_bus_handle_t bus_;
    uint8_t addr_;
    fwbox::FireLevel lvl_;
    uint16_t word_;
};
