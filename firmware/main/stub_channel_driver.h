#pragma once
#include "channel_driver.h"
// Build-only placeholder so the skeleton links. Plan 3 replaces this with the
// MCP23017/PCF8575 I²C driver. Intentionally does nothing.
class StubChannelDriver : public fw::ChannelDriver {
public:
    void allOff() override {}
    void setChannel(uint8_t /*channel*/, bool /*on*/) override {}
};
