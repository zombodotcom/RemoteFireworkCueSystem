#include "box_link.h"
#include <cstring>

namespace fw {

BoxLink::BoxLink(Transport& transport, BoxLinkConfig cfg)
    : transport_(transport), cfg_(cfg) {}

// Build a CommandPacket, fill cmd string for log readability, compute CRC.
CommandPacket BoxLink::buildPacket(MsgType type, uint32_t id, uint8_t boxId,
                                   uint8_t channel, uint32_t nonce, const char* cmd) {
    CommandPacket pkt;
    pkt.type          = static_cast<uint8_t>(type);
    pkt.id            = id;
    pkt.boxId         = boxId;
    pkt.targetChannel = channel;
    pkt.nonce         = nonce;
    std::memset(pkt.cmd, 0, sizeof(pkt.cmd));
    if (cmd) {
        std::strncpy(pkt.cmd, cmd, sizeof(pkt.cmd) - 1);
    }
    pkt.crc = computeCrc(pkt);
    return pkt;
}

void BoxLink::sendFire(uint8_t boxId, uint8_t channel, uint32_t id) {
    CommandPacket pkt = buildPacket(MsgType::FIRE, id, boxId, channel, 0, "FIRE");
    transport_.send(boxId, pkt);
}

uint32_t BoxLink::fire(uint8_t boxId, uint8_t channel, uint32_t nowMs) {
    uint32_t id = nextId_++;
    pendingId_  = id;
    pendingBox_ = boxId;
    pendingCh_  = channel;
    sentAtMs_   = nowMs;
    retries_    = 0;
    sendFire(boxId, channel, id);
    return id;
}

void BoxLink::arm(uint32_t nonce, uint32_t nowMs) {
    (void)nowMs;
    CommandPacket pkt = buildPacket(MsgType::ARM, nextId_++, 0, 0, nonce, "ARM");
    transport_.send(0, pkt);
}

void BoxLink::disarm(uint32_t nowMs) {
    (void)nowMs;
    CommandPacket pkt = buildPacket(MsgType::DISARM, nextId_++, 0, 0, 0, "DISARM");
    transport_.send(0, pkt);
}

void BoxLink::estop(uint32_t nowMs) {
    (void)nowMs;
    CommandPacket pkt = buildPacket(MsgType::ESTOP, nextId_++, 0, 0, 0, "ESTOP");
    transport_.send(0, pkt);
}

void BoxLink::heartbeat(uint32_t nowMs) {
    (void)nowMs;
    CommandPacket pkt = buildPacket(MsgType::HEARTBEAT, nextId_++, 0, 0, 0, "HB");
    transport_.send(0, pkt);
}

void BoxLink::onAck(uint32_t responseToId, uint32_t nowMs) {
    (void)nowMs;
    if (pendingId_ != 0 && responseToId == pendingId_) {
        pendingId_ = 0;
    }
}

void BoxLink::tick(uint32_t nowMs) {
    if (pendingId_ == 0) return;
    if (nowMs - sentAtMs_ < cfg_.ackTimeoutMs) return;

    if (retries_ < cfg_.maxRetries) {
        // Resend the SAME id so the box's dedup rejects duplicates if it fires.
        sendFire(pendingBox_, pendingCh_, pendingId_);
        retries_++;
        sentAtMs_ = nowMs;
    } else {
        // Exhausted retries — give up.
        lastFailedId_ = pendingId_;
        pendingId_ = 0;
    }
}

bool BoxLink::pendingAck() const {
    return pendingId_ != 0;
}

uint32_t BoxLink::lastFailedId() const {
    return lastFailedId_;
}

} // namespace fw
