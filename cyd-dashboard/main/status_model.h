#pragma once
#include <cstdint>
struct StatusModel {
    bool     controllerReachable = false; // set by client on HTTP success
    bool     boxPresent          = false; // boxes[] had an entry
    bool     boxArmed            = false; // box state == 1
    bool     boxLinkAlive        = false; // boxes[0].linkAlive
    int      rssi                = 0;      // boxes[0].rssi (dBm)
    uint16_t firedBitmap         = 0;      // boxes[0].firedBitmap
    int      lastFired           = -1;     // boxes[0].lastFired (-1 = none)
    bool     seqRunning          = false;  // top-level seqRunning
    uint32_t lastUpdateMs        = 0;      // set by client
};
