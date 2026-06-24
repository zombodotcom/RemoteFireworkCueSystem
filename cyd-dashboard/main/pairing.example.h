#pragma once
// Copy to pairing.h (git-ignored). The CYD only accepts display frames from the
// controller's SoftAP MAC (anti-spoof). Must match the box's PAIR_CONTROLLER_MAC
// and the controller's boot-log MAC.
//
//   cp pairing.example.h pairing.h        (then edit)
//   # or: pwsh scripts/gen-pairing.ps1 -ControllerPort COMx -Box0Port COMy
#define PAIR_CONTROLLER_MAC { 0xb0, 0xcb, 0xd8, 0x89, 0x9e, 0x69 }   // controller SoftAP MAC
