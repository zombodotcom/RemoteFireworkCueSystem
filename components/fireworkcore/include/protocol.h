#pragma once
#include <cstdint>
#include <cstddef>
#include "crc32.h"

namespace fw {

static const uint8_t MAX_CHANNELS = 16;

enum class MsgType : uint8_t { FIRE=1, ACK=2, ARM=3, DISARM=4, HEARTBEAT=5, ESTOP=6, STATUS=7, DISP_STATUS=8, DISP_EVENT=9 };

#pragma pack(push, 1)
struct CommandPacket {
    uint8_t  type;
    char     cmd[8];
    uint32_t id;
    uint8_t  boxId;
    uint8_t  targetChannel;
    uint32_t nonce;
    uint32_t crc;
};
struct AckPacket {
    uint8_t  type;
    uint32_t responseToId;
    char     note[16];
    uint8_t  deviceStatus;
    uint32_t timestamp;
    uint32_t crc;
};
struct StatusPacket {
    uint8_t  type;             // MsgType::STATUS
    uint8_t  boxId;
    uint8_t  state;            // 0 = SAFE, 1 = ARMED
    uint16_t firedBitmap;      // bit i set => channel i fired this arm session
    uint8_t  lastFiredChannel; // 0xFF = none
    uint32_t timestamp;        // box ms clock (diagnostic)
    uint32_t crc;
};
// Controller -> display (CYD) broadcast frames. No CRC field: ESP-NOW MAC-layer
// CRC validates the frame; the receiver checks type + exact length.
struct DisplayStatusPacket {
    uint8_t  type;          // MsgType::DISP_STATUS
    uint8_t  boxState;      // 0 = SAFE, 1 = ARMED
    uint8_t  boxLinkAlive;  // 0/1
    int8_t   rssi;
    uint16_t firedBitmap;
    int8_t   lastFired;     // -1 = none
    uint8_t  seqRunning;    // 0/1
    uint8_t  faultCode;     // 0 none, 1 estop, 2 link, 3 fire-failed
    uint32_t uptimeMs;
    uint32_t freeHeap;
    uint32_t apClients;
    uint32_t fired;
    uint32_t acked;
    uint32_t failed;
    uint32_t retries;
    uint32_t lastAckMs;
    uint32_t boxLastHeardMs;
};
struct DisplayEventPacket {
    uint8_t  type;          // MsgType::DISP_EVENT
    uint8_t  sev;           // 0 info / 1 warn / 2 err
    uint32_t seq;
    char     msg[48];
};
#pragma pack(pop)

inline uint32_t computeCrc(const CommandPacket& p) {
    return crc32(reinterpret_cast<const uint8_t*>(&p), sizeof(p) - sizeof(p.crc));
}
inline uint32_t computeCrc(const AckPacket& p) {
    return crc32(reinterpret_cast<const uint8_t*>(&p), sizeof(p) - sizeof(p.crc));
}
inline uint32_t computeCrc(const StatusPacket& p) {
    return crc32(reinterpret_cast<const uint8_t*>(&p), sizeof(p) - sizeof(p.crc));
}
inline bool crcValid(const CommandPacket& p) { return p.crc == computeCrc(p); }
inline bool crcValid(const AckPacket& p)     { return p.crc == computeCrc(p); }
inline bool crcValid(const StatusPacket& p)  { return p.crc == computeCrc(p); }

// Channel-range gate (spec §5 FIRE gate #5). Firmware composes this with CRC
// validity + dedup (RecentIds) before firing a channel.
inline bool channelInRange(uint8_t channel) { return channel < MAX_CHANNELS; }

} // namespace fw
