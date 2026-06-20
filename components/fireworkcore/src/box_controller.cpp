#include "box_controller.h"

namespace fw {

BoxController::BoxController(ChannelDriver& driver, BoxConfig cfg)
    : drv_(driver), cfg_(cfg), arm_(cfg.arming), prevState_(BoxState::SAFE) {
    for (int i = 0; i < MAX_CHANNELS; i++) { firing_[i] = false; offAtMs_[i] = 0; }
}

void BoxController::deenergizeAll() {
    drv_.allOff();
    for (int i = 0; i < MAX_CHANNELS; i++) firing_[i] = false;
}

void BoxController::begin() {
    deenergizeAll();          // outputs off before anything else (boot-safe)
    prevState_ = arm_.state(); // SAFE (never boot armed)
}

void BoxController::energize(uint8_t ch, uint32_t nowMs) {
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

void BoxController::onCommand(const CommandPacket& pkt, uint32_t nowMs) {
    if (!crcValid(pkt)) return;                     // corrupt packet ignored
    switch ((MsgType)pkt.type) {
        case MsgType::ARM:       arm_.arm(pkt.nonce, nowMs); break;
        case MsgType::DISARM:    arm_.disarm(nowMs); break;
        case MsgType::HEARTBEAT: arm_.heartbeat(nowMs); break;
        case MsgType::ESTOP:     arm_.estop(nowMs); deenergizeAll(); break;
        case MsgType::FIRE:
            if (pkt.boxId != cfg_.boxId) break;     // not addressed to this box
            if (!channelInRange(pkt.targetChannel)) break;
            if (seen_.seenOrRecord(pkt.id)) break;  // duplicate fire id
            if (!arm_.canFire(nowMs)) break;        // all interlocks
            energize(pkt.targetChannel, nowMs);
            break;
        default: break;                             // ACK or unknown — ignore on the box
    }
}

void BoxController::tick(uint32_t nowMs) {
    // Capture state before update to detect transitions caused by command/switch changes.
    BoxState stateBeforeUpdate = arm_.state();
    arm_.update(nowMs);
    // Any transition out of ARMED forces outputs off (firmware force-off contract).
    if (stateBeforeUpdate == BoxState::ARMED && arm_.state() != BoxState::ARMED) deenergizeAll();
    prevState_ = arm_.state();
    // Expire bounded fire pulses.
    for (int i = 0; i < MAX_CHANNELS; i++) {
        if (firing_[i] && nowMs >= offAtMs_[i]) { drv_.setChannel((uint8_t)i, false); firing_[i] = false; }
    }
}

} // namespace fw
