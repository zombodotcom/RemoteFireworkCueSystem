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

void BoxController::onCommand(const CommandPacket& pkt, uint32_t nowMs) {
    if (!crcValid(pkt)) return;                     // corrupt packet ignored
    switch ((MsgType)pkt.type) {
        case MsgType::ARM:       arm_.arm(pkt.nonce, nowMs); break;
        case MsgType::DISARM:    arm_.disarm(nowMs); break;
        case MsgType::HEARTBEAT: arm_.heartbeat(nowMs); break;
        case MsgType::ESTOP:     arm_.estop(nowMs); deenergizeAll(); break;
        case MsgType::FIRE:
            if (pkt.boxId != cfg_.boxId) break;       // not addressed to this box
            if (!channelInRange(pkt.targetChannel)) break;
            if (!arm_.canFire(nowMs)) break;          // rejected: do NOT record id (retry may fire later)
            if (seen_.seenOrRecord(pkt.id)) break;    // duplicate of an already-fired id -> no double-fire
            energize(pkt.targetChannel, nowMs);
            break;
        default: break;                             // ACK or unknown — ignore on the box
    }
}

void BoxController::tick(uint32_t nowMs) {
    arm_.update(nowMs);
    // Invariant: if not ARMED, no channel may be energized. Idempotent — covers
    // every path to SAFE (disarm command, heartbeat-loss, switch-off, E-STOP)
    // without relying on edge-detection timing (see arming.h FIRMWARE CONTRACT).
    if (arm_.state() != BoxState::ARMED) deenergizeAll();
    // Expire bounded fire pulses (when still ARMED).
    for (int i = 0; i < MAX_CHANNELS; i++) {
        if (firing_[i] && nowMs >= offAtMs_[i]) { drv_.setChannel((uint8_t)i, false); firing_[i] = false; }
    }
}

} // namespace fw
