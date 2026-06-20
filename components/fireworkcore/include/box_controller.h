#pragma once
#include <cstdint>
#include "arming.h"
#include "recent_ids.h"
#include "protocol.h"
#include "channel_driver.h"

namespace fw {

enum class CommandResult : uint8_t {
    IGNORED,    // not addressed to us, bad CRC, control msg, or unknown type
    FIRED,      // FIRE accepted and channel energized
    DUPLICATE,  // FIRE for an id already fired (already handled; ACK so the controller stops retrying)
    REJECTED    // FIRE addressed to us but blocked by an interlock (not armed / out of range)
};

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
    CommandResult onCommand(const CommandPacket& pkt, uint32_t nowMs);
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
};

} // namespace fw
