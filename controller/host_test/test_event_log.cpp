#include "check.h"
#include "event_log.h"
using namespace ctrl;

void test_push_seq_and_since() {
    EventLog log;
    log.push(SEV_INFO, "a", 10);
    log.push(SEV_WARN, "b", 20);
    log.push(SEV_ERR,  "c", 30);
    CHECK_EQ((int)log.lastSeq(), 3);
    Event out[8];
    size_t n = log.since(0, out, 8);
    CHECK_EQ((int)n, 3);
    CHECK_EQ((int)out[0].seq, 1);           // oldest first
    CHECK_EQ((int)out[2].seq, 3);
    CHECK_EQ((int)out[2].sev, (int)SEV_ERR);
    // incremental: only newer than seq 2
    n = log.since(2, out, 8);
    CHECK_EQ((int)n, 1);
    CHECK_EQ((int)out[0].seq, 3);
    // nothing newer than lastSeq
    CHECK_EQ((int)log.since(log.lastSeq(), out, 8), 0);
}
void test_overflow_drops_oldest() {
    EventLog log;
    for (int i = 0; i < (int)EventLog::CAP + 5; ++i) log.push(SEV_INFO, "x", (uint32_t)i);
    CHECK_EQ((int)log.lastSeq(), (int)EventLog::CAP + 5);   // 37
    Event out[64];
    size_t n = log.since(0, out, 64);
    CHECK_EQ((int)n, (int)EventLog::CAP);                   // capped at 32
    CHECK_EQ((int)out[0].seq, 6);                           // oldest kept = 37-32+1
    CHECK_EQ((int)out[n-1].seq, 37);                        // newest
}
void test_maxout_cap_returns_oldest_first() {
    EventLog log;
    for (int i = 1; i <= 5; ++i) log.push(SEV_INFO, "y", 0);
    Event out[2];
    size_t n = log.since(0, out, 2);
    CHECK_EQ((int)n, 2);
    CHECK_EQ((int)out[0].seq, 1);
    CHECK_EQ((int)out[1].seq, 2);
}
int main() {
    RUN(test_push_seq_and_since);
    RUN(test_overflow_drops_oldest);
    RUN(test_maxout_cap_returns_oldest_first);
    return REPORT();
}
