#include "check.h"
#include "sequence.h"
using namespace fw;

static SeqStep STEPS[] = {
    {0,    0, 1},
    {1000, 0, 2},
    {1500, 1, 5},
};

void test_not_running_before_start() {
    SequenceScheduler s;
    s.load(STEPS, 3);
    SeqStep out[8];
    CHECK(!s.running());
    CHECK_EQ(s.due(0, out, 8), 0);
}
void test_fires_due_steps_in_order() {
    SequenceScheduler s;
    s.load(STEPS, 3);
    s.start(10000);                  // start offset
    SeqStep out[8];
    CHECK_EQ(s.due(10000, out, 8), 1);   // elapsed 0 -> step @0
    CHECK_EQ(out[0].channel, 1);
    CHECK_EQ(s.due(10500, out, 8), 0);   // nothing new yet
    CHECK_EQ(s.due(11600, out, 8), 2);   // elapsed 1600 -> steps @1000 and @1500
    CHECK_EQ(out[0].channel, 2);
    CHECK_EQ(out[1].channel, 5);
}
void test_steps_fire_once() {
    SequenceScheduler s;
    s.load(STEPS, 3);
    s.start(0);
    SeqStep out[8];
    s.due(2000, out, 8);             // all three become due
    CHECK_EQ(s.due(3000, out, 8), 0);    // none repeat
}
void test_stop_halts() {
    SequenceScheduler s;
    s.load(STEPS, 3);
    s.start(0);
    s.stop();
    SeqStep out[8];
    CHECK_EQ(s.due(2000, out, 8), 0);
    CHECK(!s.running());
}
void test_running_false_after_all_fired() {
    SequenceScheduler s;
    s.load(STEPS, 3);
    s.start(0);
    SeqStep out[8];
    s.due(2000, out, 8);             // all become due and fire
    CHECK(!s.running());
}
void test_partial_delivery_when_outcap_small() {
    SeqStep three[] = { {0,0,1}, {0,0,2}, {0,0,3} };  // all due at elapsed 0
    SequenceScheduler s;
    s.load(three, 3);
    s.start(0);
    SeqStep out[1];
    CHECK_EQ(s.due(0, out, 1), 1);   // only room for one this call
    CHECK_EQ(out[0].channel, 1);
    CHECK(s.running());              // two steps still deferred
    CHECK_EQ(s.due(0, out, 1), 1);
    CHECK_EQ(out[0].channel, 2);
    CHECK(s.running());
    CHECK_EQ(s.due(0, out, 1), 1);
    CHECK_EQ(out[0].channel, 3);
    CHECK(!s.running());             // all delivered -> done
}

int main() {
    RUN(test_not_running_before_start);
    RUN(test_fires_due_steps_in_order);
    RUN(test_steps_fire_once);
    RUN(test_stop_halts);
    RUN(test_running_false_after_all_fired);
    RUN(test_partial_delivery_when_outcap_small);
    return REPORT();
}
