#pragma once
#include <cstdint>
namespace fwbox {

enum class FireLevel { ACTIVE_HIGH, ACTIVE_LOW };

inline uint16_t allOffWord(FireLevel lvl) {
    return lvl == FireLevel::ACTIVE_HIGH ? 0x0000u : 0xFFFFu;
}

// Set `channel`'s bit so it means `on` for the given polarity. Out-of-range: unchanged.
inline uint16_t applyChannel(uint16_t word, uint8_t channel, bool on, FireLevel lvl) {
    if (channel >= 16) return word;
    const uint16_t mask = (uint16_t)(1u << channel);
    bool bitHigh = (lvl == FireLevel::ACTIVE_HIGH) ? on : !on; // active-low: on => bit low
    if (bitHigh) word |= mask; else word &= (uint16_t)~mask;
    return word;
}

} // namespace fwbox
