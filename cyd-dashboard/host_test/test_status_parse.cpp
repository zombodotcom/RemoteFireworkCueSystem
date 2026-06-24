#include "check.h"
#include "status_parse.h"

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

void test_parse_armed_box() {
    StatusModel m;
    CHECK(parseStatus(JSON_ARMED, m));
    CHECK(m.boxPresent);
    CHECK(m.boxArmed);            // state == 1
    CHECK(m.boxLinkAlive);
    CHECK_EQ(m.rssi, -52);
    CHECK_EQ((int)m.firedBitmap, 5);
    CHECK(!m.seqRunning);
}
void test_parse_safe_nolink_seq() {
    StatusModel m;
    CHECK(parseStatus(JSON_SAFE_NOLINK, m));
    CHECK(m.boxPresent);
    CHECK(!m.boxArmed);          // state == 0
    CHECK(!m.boxLinkAlive);
    CHECK_EQ(m.rssi, 0);
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
int main() {
    RUN(test_parse_armed_box);
    RUN(test_parse_safe_nolink_seq);
    RUN(test_parse_empty_boxes);
    RUN(test_parse_malformed_returns_false);
    return REPORT();
}
