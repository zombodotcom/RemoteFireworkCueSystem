#include "check.h"
#include "recent_ids.h"

void test_new_id_is_not_duplicate() {
    fw::RecentIds<4> r;
    CHECK(!r.seenOrRecord(10));
}
void test_repeat_id_is_duplicate() {
    fw::RecentIds<4> r;
    r.seenOrRecord(10);
    CHECK(r.seenOrRecord(10));
}
void test_evicted_id_seen_again_as_new() {
    fw::RecentIds<2> r;            // capacity 2
    r.seenOrRecord(1);
    r.seenOrRecord(2);
    r.seenOrRecord(3);            // evicts 1
    CHECK(!r.seenOrRecord(1));    // 1 no longer remembered
}

int main() {
    RUN(test_new_id_is_not_duplicate);
    RUN(test_repeat_id_is_duplicate);
    RUN(test_evicted_id_seen_again_as_new);
    return REPORT();
}
