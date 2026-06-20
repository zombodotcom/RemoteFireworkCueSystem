#pragma once
#include "channel_driver.h"
#include "protocol.h"

struct FakeChannelDriver : public fw::ChannelDriver {
    int allOffCount = 0;
    bool on[fw::MAX_CHANNELS] = {false};
    void allOff() override { allOffCount++; for (int i=0;i<fw::MAX_CHANNELS;i++) on[i]=false; }
    void setChannel(uint8_t ch, bool state) override { if (ch < fw::MAX_CHANNELS) on[ch]=state; }
    int countOn() const { int n=0; for (int i=0;i<fw::MAX_CHANNELS;i++) if(on[i]) n++; return n; }
};
