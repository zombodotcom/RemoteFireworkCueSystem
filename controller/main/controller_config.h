#pragma once
#include <cstdint>

namespace ctrl {

constexpr char AP_SSID[]     = "FireControl";
constexpr char AP_PASS[]     = "pyro1234";   // WPA2 requires >= 8 chars
constexpr uint8_t WIFI_CHAN  = 1;

// Box peer MAC addresses — STA MACs printed on each box's label.
// Box 0: b0:cb:d8:8a:4d:1c (known); Box 1: placeholder, fill before pairing.
constexpr uint8_t BOX_MAC[2][6] = {
    { 0xb0, 0xcb, 0xd8, 0x8a, 0x4d, 0x1c },   // box 0 — fill from box label
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },   // box 1 — TODO: fill from box label
};

constexpr uint32_t HEARTBEAT_MS   = 500;
constexpr uint32_t ACK_TIMEOUT_MS = 120;
constexpr uint8_t  MAX_RETRIES    = 3;

} // namespace ctrl
