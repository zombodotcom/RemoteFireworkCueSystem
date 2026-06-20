#pragma once
#include <cstdint>
#include "box_link.h"
#include "sequence.h"

namespace fw {

// ShowRunner: controller logic that runs timed sequences, emits continuous
// heartbeats while armed, and owns arm/disarm/E-STOP — all driving a BoxLink.
// All time is injected as nowMs; no clock reads.
class ShowRunner {
public:
    ShowRunner(BoxLink& link, uint32_t heartbeatPeriodMs = 500);

    // Arm: pick a fresh monotonically increasing nonce, call link.arm(nonce,now).
    void arm(uint32_t nowMs);

    // Disarm: call link.disarm(now), set armed_=false, stop sequence.
    void disarm(uint32_t nowMs);

    // E-STOP: call link.estop(now), set armed_=false, stop sequence.
    void estop(uint32_t nowMs);

    // Fire one channel manually — only when armed and link is not pending an ACK.
    void fireManual(uint8_t boxId, uint8_t channel, uint32_t nowMs);

    // Load a sequence (does not start it).
    void loadSequence(const SeqStep* steps, size_t count);

    // Start the loaded sequence — only if armed_.
    void startSequence(uint32_t nowMs);

    // Stop the sequence.
    void stopSequence(uint32_t nowMs);

    bool sequenceRunning() const;
    bool armed() const { return armed_; }

    // Drive the runner: emit heartbeats, fire due sequence steps, call link_.tick.
    void tick(uint32_t nowMs);

private:
    BoxLink&           link_;
    SequenceScheduler  sched_;
    uint32_t           hbPeriodMs_;
    uint32_t           lastHbMs_;
    uint32_t           nonceCtr_;
    bool               armed_;
    bool               hbStarted_;   // true once the first HB has been sent in this arm session
};

} // namespace fw
