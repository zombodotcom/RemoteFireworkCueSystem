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
    // FIRMWARE CONTRACT: drive this every control loop from the controller's
    // sequence engine, e.g. setSequenceRunning(scheduler.running()). While true,
    // the heartbeat dead-man is intentionally suspended so a committed show is
    // not aborted by a brief link blip. Forgetting to set it false when the show
    // ends would leave the box ARMED with the dead-man disabled. Every SAFE
    // transition (disarm/estop/switch-off) clears it automatically.
    void setSequenceRunning(bool running);
    void update(uint32_t nowMs);

    BoxState state() const { return state_; }
    // FIRMWARE CONTRACT (force-off): physical outputs live outside this core.
    // The firmware MUST de-energize ALL channels on every loop whenever
    // canFire(now) is false (equivalently whenever state()==SAFE). Assert "off"
    // continuously and idempotently — do not rely on edge-detecting the SAFE
    // transition.
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
