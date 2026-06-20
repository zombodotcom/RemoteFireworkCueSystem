#pragma once
#include <cstdint>
#include "protocol.h"

namespace fw {

// Abstract transport — implemented by ESP-NOW glue on device, FakeTransport in tests.
class Transport {
public:
    virtual void send(uint8_t boxId, const CommandPacket& pkt) = 0;
    virtual ~Transport() {}
};

struct BoxLinkConfig {
    uint32_t ackTimeoutMs = 120;
    uint8_t  maxRetries   = 3;
};

// BoxLink: controller-side ESP-NOW TX with ACK tracking + retry.
// Single FIRE-in-flight design — controller fires cues sequentially.
// All time injected as nowMs; no clock reads.
class BoxLink {
public:
    explicit BoxLink(Transport& transport, BoxLinkConfig cfg = BoxLinkConfig());

    // Send a FIRE command to boxId/channel; returns the msg id used. Marks that id pending.
    uint32_t fire(uint8_t boxId, uint8_t channel, uint32_t nowMs);

    // Immediate send, no ACK tracking.
    void arm(uint32_t nonce, uint32_t nowMs);
    void disarm(uint32_t nowMs);
    void estop(uint32_t nowMs);
    void heartbeat(uint32_t nowMs);

    // Called when an ACK arrives (responseToId matches). Clears pending.
    void onAck(uint32_t responseToId, uint32_t nowMs);

    // Drive retries. Resends un-ACKed FIRE if ackTimeoutMs elapsed (same id for box dedup).
    // After maxRetries resends with no ACK, drops to lastFailedId_ and clears pending.
    void tick(uint32_t nowMs);

    bool     pendingAck()    const;
    uint32_t lastFailedId()  const;

private:
    Transport&    transport_;
    BoxLinkConfig cfg_;

    uint32_t nextId_      = 1;   // monotonically increasing, starts at 1
    uint32_t pendingId_   = 0;   // 0 = nothing pending
    uint8_t  pendingBox_  = 0;
    uint8_t  pendingCh_   = 0;
    uint32_t sentAtMs_    = 0;
    uint8_t  retries_     = 0;
    uint32_t lastFailedId_ = 0;

    void sendFire(uint8_t boxId, uint8_t channel, uint32_t id);
    CommandPacket buildPacket(MsgType type, uint32_t id, uint8_t boxId,
                              uint8_t channel, uint32_t nonce, const char* cmd);
};

} // namespace fw
