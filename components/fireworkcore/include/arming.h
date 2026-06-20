#pragma once
#include <cstdint>

namespace fw {

enum class BoxState : uint8_t { SAFE, ARMED };

struct ArmingConfig {
    uint32_t heartbeatTimeoutMs = 2000;
};

class ArmingStateMachine {
public:
    explicit ArmingStateMachine(ArmingConfig cfg = ArmingConfig());

    void setPhysicalSwitch(bool on, uint32_t nowMs);
    bool arm(uint32_t nonce, uint32_t nowMs);
    void disarm(uint32_t nowMs);
    void estop(uint32_t nowMs);
    void clearEstop(uint32_t nowMs);
    void heartbeat(uint32_t nowMs);
    void setSequenceRunning(bool running);
    void update(uint32_t nowMs);

    BoxState state() const { return state_; }
    bool canFire(uint32_t nowMs) const;

private:
    bool heartbeatFresh(uint32_t nowMs) const;
    void goSafe();

    ArmingConfig cfg_;
    BoxState state_;
    bool switchOn_;
    bool estopped_;
    bool sequenceRunning_;
    bool haveNonce_;
    uint32_t lastNonce_;
    uint32_t lastHeartbeatMs_;
};

} // namespace fw
