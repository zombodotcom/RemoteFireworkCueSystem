#pragma once
#include <cstdint>
#include "arming.h"
#include "recent_ids.h"
#include "protocol.h"
#include "channel_driver.h"

namespace fw {

struct BoxConfig {
    uint8_t  boxId = 0;
    uint32_t fireMs = 400;
    ArmingConfig arming = ArmingConfig();
};

class BoxController {
public:
    BoxController(ChannelDriver& driver, BoxConfig cfg = BoxConfig());
    void begin();                                   // boot: all-off, SAFE
    void setPhysicalSwitch(bool on, uint32_t nowMs);
    void onCommand(const CommandPacket& pkt, uint32_t nowMs);
    void tick(uint32_t nowMs);
    BoxState state() const { return arm_.state(); }
    bool canFire(uint32_t nowMs) const { return arm_.canFire(nowMs); }

private:
    void energize(uint8_t ch, uint32_t nowMs);
    void deenergizeAll();

    ChannelDriver& drv_;
    BoxConfig cfg_;
    ArmingStateMachine arm_;
    RecentIds<32> seen_;
    bool     firing_[MAX_CHANNELS];
    uint32_t offAtMs_[MAX_CHANNELS];
    BoxState prevState_;
};

} // namespace fw
