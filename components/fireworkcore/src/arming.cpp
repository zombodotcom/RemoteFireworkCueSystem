#include "arming.h"

namespace fw {

ArmingStateMachine::ArmingStateMachine(ArmingConfig cfg)
    : cfg_(cfg), state_(BoxState::SAFE), switchOn_(false),
      estopped_(false), sequenceRunning_(false), haveNonce_(false),
      lastNonce_(0), lastHeartbeatMs_(0) {}

void ArmingStateMachine::goSafe() {
    state_ = BoxState::SAFE;
    haveNonce_ = false;        // each arm session is independent (reset-per-session)
    sequenceRunning_ = false;  // a disarm/estop/switch-off ends any running sequence
}

void ArmingStateMachine::setPhysicalSwitch(bool on, uint32_t /*nowMs*/) {
    switchOn_ = on;
    if (!on) goSafe();
}

bool ArmingStateMachine::arm(uint32_t nonce, uint32_t nowMs) {
    if (!switchOn_ || estopped_) return false;
    if (haveNonce_ && nonce == lastNonce_) return false; // replay
    lastNonce_ = nonce;
    haveNonce_ = true;
    lastHeartbeatMs_ = nowMs;
    state_ = BoxState::ARMED;
    return true;
}

void ArmingStateMachine::disarm(uint32_t /*nowMs*/) { goSafe(); }

void ArmingStateMachine::estop(uint32_t /*nowMs*/) { estopped_ = true; goSafe(); }

void ArmingStateMachine::clearEstop(uint32_t /*nowMs*/) { estopped_ = false; }

void ArmingStateMachine::heartbeat(uint32_t nowMs) { lastHeartbeatMs_ = nowMs; }

void ArmingStateMachine::setSequenceRunning(bool running) { sequenceRunning_ = running; }

bool ArmingStateMachine::heartbeatFresh(uint32_t nowMs) const {
    return (nowMs - lastHeartbeatMs_) <= cfg_.heartbeatTimeoutMs;
}

void ArmingStateMachine::update(uint32_t nowMs) {
    if (state_ != BoxState::ARMED) return;
    if (!switchOn_ || estopped_) { goSafe(); return; }
    if (!sequenceRunning_ && !heartbeatFresh(nowMs)) goSafe();
}

bool ArmingStateMachine::canFire(uint32_t nowMs) const {
    if (state_ != BoxState::ARMED) return false;
    if (!switchOn_ || estopped_) return false;
    if (sequenceRunning_) return true;
    return heartbeatFresh(nowMs);
}

} // namespace fw
