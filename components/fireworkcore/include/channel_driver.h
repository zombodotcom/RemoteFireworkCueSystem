#pragma once
#include <cstdint>
namespace fw {
class ChannelDriver {
public:
    virtual ~ChannelDriver() {}
    virtual void allOff() = 0;                       // force every channel de-energized
    virtual void setChannel(uint8_t channel, bool on) = 0;
};
} // namespace fw
