#include "crc32.h"
namespace fw {
uint32_t crc32(const uint8_t* data, size_t len) {
    uint32_t crc = 0xFFFFFFFFu;
    while (len--) {
        uint8_t b = *data++;
        for (int i = 0; i < 8; i++) {
            uint32_t mask = -(int32_t)((crc ^ b) & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
            b >>= 1;
        }
    }
    return ~crc;
}
} // namespace fw
