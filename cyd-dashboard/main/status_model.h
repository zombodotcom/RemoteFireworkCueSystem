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

    // Diagnostics (from /api/status "diag")
    struct {
        uint32_t uptimeMs = 0, freeHeap = 0, apClients = 0;
        uint32_t fired = 0, acked = 0, failed = 0, retries = 0, lastAckMs = 0;
    } diag;
    bool     faultActive = false;     // /api/status fault.active
    char     faultMsg[32] = {0};      // /api/status fault.msg
    uint32_t boxLastHeardMs = 0;      // boxes[0].lastHeardMs
};

struct LogEv { int seq; int sev; char msg[48]; };
