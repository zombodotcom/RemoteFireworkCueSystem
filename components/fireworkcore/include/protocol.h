#pragma once
#include <cstdint>
#include <cstddef>
#include "crc32.h"

namespace fw {

static const uint8_t MAX_CHANNELS = 16;

enum class MsgType : uint8_t { FIRE=1, ACK=2, ARM=3, DISARM=4, HEARTBEAT=5, ESTOP=6, STATUS=7 };

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
