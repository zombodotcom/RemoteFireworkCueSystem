#pragma once
// Copy to pairing.h (git-ignored). The box sends ACK/telemetry to the
// controller's SoftAP MAC, printed in the controller boot log
// ("controller booted: AP ... MAC ...").
//
//   cp pairing.example.h pairing.h        (then edit)
//   # or: pwsh scripts/gen-pairing.ps1 -ControllerPort COMx -Box0Port COMy
#define PAIR_CONTROLLER_MAC { 0xb0, 0xcb, 0xd8, 0x89, 0x9e, 0x69 }   // controller SoftAP MAC
