#pragma once
#include <cstdint>

// ---- Local secrets override ----------------------------------------------
// Real AP credentials live in `secrets.h` (git-ignored). Copy
// `secrets.example.h` -> `secrets.h` and set your own. If secrets.h is absent
// (e.g. a fresh clone), the safe placeholder defaults below are used so the
// project still builds. NEVER commit secrets.h — the repo is public.
#if defined(__has_include)
#  if __has_include("secrets.h")
#    include "secrets.h"
#  endif
#endif
#ifndef FW_AP_SSID
#  define FW_AP_SSID "FireControl"
#endif
#ifndef FW_AP_PASS
#  define FW_AP_PASS "changeme123"   // WPA2 requires >= 8 chars; override in secrets.h
#endif

namespace ctrl {

constexpr char AP_SSID[]     = FW_AP_SSID;
constexpr char AP_PASS[]     = FW_AP_PASS;
constexpr uint8_t WIFI_CHAN  = 1;

// Box peer MAC addresses — STA MACs printed on each box's label.
// Box 0: b0:cb:d8:8a:4d:1c (known); Box 1: placeholder, fill before pairing.
constexpr uint8_t BOX_MAC[2][6] = {
    { 0xb0, 0xcb, 0xd8, 0x8a, 0x4d, 0x1c },   // box 0
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },   // box 1 — TODO: fill from box label
};

constexpr uint32_t HEARTBEAT_MS   = 500;
constexpr uint32_t ACK_TIMEOUT_MS = 120;
constexpr uint8_t  MAX_RETRIES    = 3;

} // namespace ctrl
