#pragma once
#include <cstdint>
#include <cstddef>
#include "crc32.h"

namespace fw {

static const uint8_t MAX_CHANNELS = 16;

enum class MsgType : uint8_t { FIRE=1, ACK=2, ARM=3, DISARM=4, HEARTBEAT=5, ESTOP=6 };

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
#pragma pack(pop)

inline uint32_t computeCrc(const CommandPacket& p) {
    return crc32(reinterpret_cast<const uint8_t*>(&p), sizeof(p) - sizeof(p.crc));
}
inline uint32_t computeCrc(const AckPacket& p) {
    return crc32(reinterpret_cast<const uint8_t*>(&p), sizeof(p) - sizeof(p.crc));
}
inline bool crcValid(const CommandPacket& p) { return p.crc == computeCrc(p); }
inline bool crcValid(const AckPacket& p)     { return p.crc == computeCrc(p); }

} // namespace fw
