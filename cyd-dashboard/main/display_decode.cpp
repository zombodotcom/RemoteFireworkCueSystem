#include "display_decode.h"
#include <cstring>

void applyDisplayStatus(const fw::DisplayStatusPacket& p, StatusModel& m) {
    m.boxPresent     = true;                 // the controller broadcasts box 0's data
    m.boxArmed       = (p.boxState == 1);
    m.boxLinkAlive   = (p.boxLinkAlive != 0);
    m.rssi           = p.rssi;
    m.firedBitmap    = p.firedBitmap;
    m.lastFired      = p.lastFired;
    m.seqRunning     = (p.seqRunning != 0);
    m.boxLastHeardMs = p.boxLastHeardMs;
    m.diag.uptimeMs  = p.uptimeMs;
    m.diag.freeHeap  = p.freeHeap;
    m.diag.apClients = p.apClients;
    m.diag.fired     = p.fired;
    m.diag.acked     = p.acked;
    m.diag.failed    = p.failed;
    m.diag.retries   = p.retries;
    m.diag.lastAckMs = p.lastAckMs;
    m.faultActive    = (p.faultCode != 0);
    const char* fm = (p.faultCode == 1) ? "ESTOP" :
                     (p.faultCode == 2) ? "box link lost" :
                     (p.faultCode == 3) ? "fire failed" : "";
    std::strncpy(m.faultMsg, fm, sizeof(m.faultMsg) - 1);
    m.faultMsg[sizeof(m.faultMsg) - 1] = '\0';
}

void toLogEv(const fw::DisplayEventPacket& p, LogEv& e) {
    e.seq = (int)p.seq;
    e.sev = (int)p.sev;
    std::strncpy(e.msg, p.msg, sizeof(e.msg) - 1);
    e.msg[sizeof(e.msg) - 1] = '\0';
}
