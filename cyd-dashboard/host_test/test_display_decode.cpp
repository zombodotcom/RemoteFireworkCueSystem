#include "check.h"
#include "display_decode.h"
#include <cstring>

void test_apply_display_status() {
    fw::DisplayStatusPacket p{};
    p.type = (uint8_t)fw::MsgType::DISP_STATUS;
    p.boxState = 1; p.boxLinkAlive = 1; p.rssi = -42;
    p.firedBitmap = 5; p.lastFired = 2; p.seqRunning = 1; p.faultCode = 2;
    p.uptimeMs = 61000; p.freeHeap = 142000; p.apClients = 1;
    p.fired = 3; p.acked = 3; p.failed = 1; p.retries = 2; p.lastAckMs = 8;
    p.boxLastHeardMs = 250;
    StatusModel m;
    applyDisplayStatus(p, m);
    CHECK(m.boxPresent);
    CHECK(m.boxArmed);
    CHECK(m.boxLinkAlive);
    CHECK_EQ(m.rssi, -42);
    CHECK_EQ((int)m.firedBitmap, 5);
    CHECK_EQ(m.lastFired, 2);
    CHECK(m.seqRunning);
    CHECK_EQ((int)m.diag.failed, 1);
    CHECK_EQ((int)m.diag.lastAckMs, 8);
    CHECK_EQ((int)m.boxLastHeardMs, 250);
    CHECK(m.faultActive);
    CHECK(std::strcmp(m.faultMsg, "box link lost") == 0);
}
void test_apply_display_status_safe_nofault() {
    fw::DisplayStatusPacket p{};
    p.boxState = 0; p.boxLinkAlive = 0; p.faultCode = 0; p.lastFired = -1;
    StatusModel m;
    applyDisplayStatus(p, m);
    CHECK(!m.boxArmed);
    CHECK(!m.boxLinkAlive);
    CHECK(!m.faultActive);
    CHECK_EQ((int)std::strlen(m.faultMsg), 0);
    CHECK_EQ(m.lastFired, -1);
}
void test_to_logev() {
    fw::DisplayEventPacket p{};
    p.seq = 7; p.sev = 2;
    std::strncpy(p.msg, "FIRE FAILED ch3", sizeof(p.msg) - 1);
    LogEv e;
    toLogEv(p, e);
    CHECK_EQ(e.seq, 7);
    CHECK_EQ(e.sev, 2);
    CHECK(std::strcmp(e.msg, "FIRE FAILED ch3") == 0);
}
int main() {
    RUN(test_apply_display_status);
    RUN(test_apply_display_status_safe_nofault);
    RUN(test_to_logev);
    return REPORT();
}
