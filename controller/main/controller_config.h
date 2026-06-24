#pragma once
#include <cstdint>

// ---- Local secrets / pairing overrides -----------------------------------
// Real AP credentials live in `secrets.h`; board-pairing MACs live in
// `pairing.h`. Both are git-ignored: copy the matching `*.example.h`, fill in,
// and the build picks them up. If absent (e.g. a fresh clone) the placeholder
// defaults below are used so the project still builds. NEVER commit either file.
#if defined(__has_include)
#  if __has_include("secrets.h")
#    include "secrets.h"
#  endif
#  if __has_include("pairing.h")
#    include "pairing.h"
#  endif
#endif
#ifndef FW_AP_SSID
#  define FW_AP_SSID "FireControl"
#endif
#ifndef FW_AP_PASS
#  define FW_AP_PASS "changeme123"   // WPA2 requires >= 8 chars; override in secrets.h
#endif
#ifndef PAIR_BOX0_MAC
#  define PAIR_BOX0_MAC { 0xb0, 0xcb, 0xd8, 0x8a, 0x4d, 0x1c }   // canonical box 0 STA
#endif
#ifndef PAIR_BOX1_MAC
#  define PAIR_BOX1_MAC { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }   // box 1 unconfigured
#endif

namespace ctrl {

constexpr char AP_SSID[]     = FW_AP_SSID;
constexpr char AP_PASS[]     = FW_AP_PASS;
constexpr uint8_t WIFI_CHAN  = 1;

// Box peer MAC addresses — STA MACs printed on each box's label.
// Defaults are the canonical boards; override per setup in pairing.h
// (PAIR_BOX0_MAC / PAIR_BOX1_MAC). See pairing.example.h.
constexpr uint8_t BOX_MAC[2][6] = {
    PAIR_BOX0_MAC,   // box 0
    PAIR_BOX1_MAC,   // box 1
};

constexpr uint32_t HEARTBEAT_MS   = 500;
constexpr uint32_t ACK_TIMEOUT_MS = 120;
constexpr uint8_t  MAX_RETRIES    = 3;

} // namespace ctrl
