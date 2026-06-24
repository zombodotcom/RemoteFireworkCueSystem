#pragma once
// Copy to pairing.h (git-ignored) to pair a specific controller + box set.
//
//   cp pairing.example.h pairing.h        (then edit)
//   # or auto-generate from plugged-in boards:
//   pwsh scripts/gen-pairing.ps1 -ControllerPort COMx -Box0Port COMy
//
// Each box's STA MAC comes from `esptool read-mac` on that board. The
// controller's own SoftAP MAC (needed by the box + CYD) is printed in its boot
// log: "controller booted: AP \"<ssid>\" up, MAC <aa:bb:...>".
#define PAIR_BOX0_MAC { 0xb0, 0xcb, 0xd8, 0x8a, 0x4d, 0x1c }   // box 0 STA MAC
// #define PAIR_BOX1_MAC { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }  // box 1 STA MAC (optional)
