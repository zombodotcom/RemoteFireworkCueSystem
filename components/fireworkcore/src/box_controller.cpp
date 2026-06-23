#include "box_controller.h"

namespace fw {

BoxController::BoxController(ChannelDriver& driver, BoxConfig cfg)
    : drv_(driver), cfg_(cfg), arm_(cfg.arming) {
    for (int i = 0; i < MAX_CHANNELS; i++) { firing_[i] = false; offAtMs_[i] = 0; }
}

void BoxController::deenergizeAll() {
    drv_.allOff();
    for (int i = 0; i < MAX_CHANNELS; i++) firing_[i] = false;
}

void BoxController::begin() {
    deenergizeAll();          // outputs off before anything else (boot-safe)
}

void BoxController::energize(uint8_t ch, uint32_t nowMs) {
    if (ch >= MAX_CHANNELS) return;          // defensive: never index out of range
    drv_.setChannel(ch, true);
    firing_[ch] = true;
    offAtMs_[ch] = nowMs + cfg_.fireMs;
}

void BoxController::setPhysicalSwitch(bool on, uint32_t nowMs) {
    BoxState oldState = arm_.state();
    arm_.setPhysicalSwitch(on, nowMs);
    BoxState newState = arm_.state();
    // If state changed from ARMED to something else, force all outputs off immediately.
    if (oldState == BoxState::ARMED && newState != BoxState::ARMED) {
        deenergizeAll();
    }
}

CommandResult BoxController::onCommand(const CommandPacket& pkt, uint32_t nowMs) {
    if (!crcValid(pkt)) return CommandResult::IGNORED;   // corrupt packet ignored
    switch ((MsgType)pkt.type) {
        case MsgType::ARM:       arm_.arm(pkt.nonce, nowMs); return CommandResult::IGNORED;
        case MsgType::DISARM:    arm_.disarm(nowMs); return CommandResult::IGNORED;
        case MsgType::HEARTBEAT: arm_.heartbeat(nowMs); return CommandResult::IGNORED;
        case MsgType::ESTOP:     arm_.estop(nowMs); deenergizeAll(); return CommandResult::IGNORED;
        case MsgType::FIRE:
            if (pkt.boxId != cfg_.boxId) return CommandResult::IGNORED;      // not ours
            if (!channelInRange(pkt.targetChannel)) return CommandResult::REJECTED;
            if (!arm_.canFire(nowMs)) return CommandResult::REJECTED;        // not armed etc: do NOT record id
            if (seen_.seenOrRecord(pkt.id)) return CommandResult::DUPLICATE; // already fired this id
            energize(pkt.targetChannel, nowMs);
            return CommandResult::FIRED;
        default: return CommandResult::IGNORED;    // ACK or unknown — ignore on the box
    }
}

void BoxController::tick(uint32_t nowMs) {
    arm_.update(nowMs);
    // Invariant: not ARMED => no channel energized. Only WRITE when something is
    // actually on (avoids hammering the I2C bus every idle tick, and avoids a
    // missing/flaky expander stalling the loop on repeated write timeouts).
    if (arm_.state() != BoxState::ARMED) {
        bool anyOn = false;
        for (int i = 0; i < MAX_CHANNELS; i++) { if (firing_[i]) { anyOn = true; break; } }
        if (anyOn) deenergizeAll();
    }
    // Expire bounded fire pulses (when still ARMED).
    for (int i = 0; i < MAX_CHANNELS; i++) {
        if (firing_[i] && nowMs >= offAtMs_[i]) { drv_.setChannel((uint8_t)i, false); firing_[i] = false; }
    }
}

} // namespace fw
