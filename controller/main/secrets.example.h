#pragma once
// Controller AP credentials template.
//
//   cp secrets.example.h secrets.h   (secrets.h is git-ignored)
//
// Then edit secrets.h with your own SSID/password. The build uses secrets.h
// when present, otherwise the placeholder defaults in controller_config.h.
// Do NOT put real credentials here — this file is committed to the repo.
#define FW_AP_SSID "FireControl"
#define FW_AP_PASS "changeme123"   // WPA2 requires >= 8 characters
