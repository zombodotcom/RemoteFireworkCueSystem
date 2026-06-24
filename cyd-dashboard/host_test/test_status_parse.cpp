#include "check.h"
#include "status_parse.h"
#include <cstring>

static const char* JSON_ARMED =
  "{\"armed\":true,\"seqRunning\":false,\"lastFailedBox\":null,"
  "\"boxes\":[{\"id\":0,\"linkAlive\":true,\"rssi\":-52,\"state\":1,"
  "\"firedBitmap\":5,\"lastFired\":2}]}";

static const char* JSON_SAFE_NOLINK =
  "{\"armed\":false,\"seqRunning\":true,\"lastFailedBox\":null,"
  "\"boxes\":[{\"id\":0,\"linkAlive\":false,\"rssi\":0,\"state\":0,"
  "\"firedBitmap\":0,\"lastFired\":-1}]}";

static const char* JSON_EMPTY_BOXES =
  "{\"armed\":false,\"seqRunning\":false,\"lastFailedBox\":null,\"boxes\":[]}";

static const char* JSON_DIAG =
  "{\"armed\":false,\"seqRunning\":false,\"lastFailedBox\":null,"
  "\"boxes\":[{\"id\":0,\"linkAlive\":true,\"rssi\":-40,\"state\":0,"
  "\"firedBitmap\":0,\"lastFired\":-1,\"lastHeardMs\":250}],"
  "\"diag\":{\"uptimeMs\":61000,\"freeHeap\":142000,\"apClients\":1,"
  "\"fired\":3,\"acked\":3,\"failed\":1,\"retries\":2,\"lastAckMs\":8},"
  "\"fault\":{\"active\":true,\"msg\":\"box link lost\"}}";

static const char* JSON_EVENTS =
  "{\"lastSeq\":3,\"events\":["
  "{\"seq\":1,\"t\":100,\"sev\":0,\"msg\":\"controller up\"},"
  "{\"seq\":2,\"t\":200,\"sev\":1,\"msg\":\"ARM -> box0\"},"
  "{\"seq\":3,\"t\":300,\"sev\":2,\"msg\":\"FIRE FAILED ch3\"}]}";

void test_parse_armed_box() {
    StatusModel m;
    CHECK(parseStatus(JSON_ARMED, m));
    CHECK(m.boxPresent);
    CHECK(m.boxArmed);            // state == 1
    CHECK(m.boxLinkAlive);
    CHECK_EQ(m.rssi, -52);
    CHECK_EQ((int)m.firedBitmap, 5);
    CHECK_EQ(m.lastFired, 2);
    CHECK(!m.seqRunning);
}
void test_parse_safe_nolink_seq() {
    StatusModel m;
    CHECK(parseStatus(JSON_SAFE_NOLINK, m));
    CHECK(m.boxPresent);
    CHECK(!m.boxArmed);          // state == 0
    CHECK(!m.boxLinkAlive);
    CHECK_EQ(m.rssi, 0);
    CHECK_EQ(m.lastFired, -1);
    CHECK(m.seqRunning);
}
void test_parse_empty_boxes() {
    StatusModel m;
    CHECK(parseStatus(JSON_EMPTY_BOXES, m));  // valid, but no box
    CHECK(!m.boxPresent);
}
void test_parse_malformed_returns_false() {
    StatusModel m;
    CHECK(!parseStatus("{\"armed\":false}", m));  // no "boxes"
    CHECK(!parseStatus("", m));
    CHECK(!parseStatus(nullptr, m));
}
void test_parse_diag_fault_lastheard() {
    StatusModel m;
    CHECK(parseStatus(JSON_DIAG, m));
    CHECK_EQ((int)m.diag.uptimeMs, 61000);
    CHECK_EQ((int)m.diag.apClients, 1);
    CHECK_EQ((int)m.diag.failed, 1);
    CHECK_EQ((int)m.diag.lastAckMs, 8);
    CHECK_EQ((int)m.boxLastHeardMs, 250);
    CHECK(m.faultActive);
    CHECK(std::strcmp(m.faultMsg, "box link lost") == 0);
}
void test_parse_events() {
    LogEv evs[8];
    int n = parseEvents(JSON_EVENTS, evs, 8);
    CHECK_EQ(n, 3);
    CHECK_EQ(evs[0].seq, 1);
    CHECK_EQ(evs[0].sev, 0);
    CHECK(std::strcmp(evs[0].msg, "controller up") == 0);
    CHECK_EQ(evs[2].sev, 2);
    CHECK(std::strcmp(evs[2].msg, "FIRE FAILED ch3") == 0);
}
void test_parse_events_empty_and_cap() {
    LogEv evs[2];
    CHECK_EQ(parseEvents("{\"lastSeq\":0,\"events\":[]}", evs, 2), 0);
    CHECK_EQ(parseEvents(JSON_EVENTS, evs, 2), 2);   // capped at maxOut
    CHECK_EQ(parseEvents(nullptr, evs, 2), 0);
}
int main() {
    RUN(test_parse_armed_box);
    RUN(test_parse_safe_nolink_seq);
    RUN(test_parse_empty_boxes);
    RUN(test_parse_malformed_returns_false);
    RUN(test_parse_diag_fault_lastheard);
    RUN(test_parse_events);
    RUN(test_parse_events_empty_and_cap);
    return REPORT();
}
