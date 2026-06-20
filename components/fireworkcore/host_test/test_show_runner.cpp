// test_show_runner.cpp — host test for ShowRunner
// TDD: this file is written BEFORE show_runner.h/cpp exist.
#include "check.h"
#include "show_runner.h"   // will not compile until show_runner.h is created (RED phase)
#include <vector>

using namespace fw;

// ---------------------------------------------------------------------------
// FakeTransport (copied from test_box_link.cpp pattern)
// ---------------------------------------------------------------------------
struct Sent { uint8_t box; CommandPacket pkt; };
struct FakeTransport : public Transport {
    std::vector<Sent> sent;
    void send(uint8_t boxId, const CommandPacket& p) override { sent.push_back({boxId, p}); }
};
static int countType(FakeTransport& t, MsgType ty) {
    int n = 0;
    for (auto& s : t.sent) if ((MsgType)s.pkt.type == ty) n++;
    return n;
}

// ---------------------------------------------------------------------------
// Test 1: tick emits heartbeat at configured cadence while armed
// arm(0) → tick(0) emits 1 HB; tick(100) emits none; tick(500) emits another.
// ---------------------------------------------------------------------------
void test_heartbeat_cadence_while_armed() {
    FakeTransport t;
    BoxLink link(t);
    ShowRunner runner(link, 500); // heartbeatPeriodMs = 500

    runner.arm(0);
    CHECK_EQ(countType(t, MsgType::HEARTBEAT), 0); // arm itself doesn't emit HB

    runner.tick(0);
    CHECK_EQ(countType(t, MsgType::HEARTBEAT), 1); // first HB on first tick

    runner.tick(100);
    CHECK_EQ(countType(t, MsgType::HEARTBEAT), 1); // not yet due (100 < 500)

    runner.tick(499);
    CHECK_EQ(countType(t, MsgType::HEARTBEAT), 1); // still not due

    runner.tick(500);
    CHECK_EQ(countType(t, MsgType::HEARTBEAT), 2); // second HB at t=500
}

// ---------------------------------------------------------------------------
// Test 2: no heartbeat emitted while disarmed
// ---------------------------------------------------------------------------
void test_no_heartbeat_while_disarmed() {
    FakeTransport t;
    BoxLink link(t);
    ShowRunner runner(link, 500);

    // Tick many times without arming — no heartbeats.
    runner.tick(0);
    runner.tick(500);
    runner.tick(1000);
    CHECK_EQ(countType(t, MsgType::HEARTBEAT), 0);

    // Arm, get some HBs.
    runner.arm(0);
    runner.tick(0);
    CHECK_EQ(countType(t, MsgType::HEARTBEAT), 1);

    // Disarm, then tick — no more HBs.
    runner.disarm(0);
    runner.tick(500);
    runner.tick(1000);
    CHECK_EQ(countType(t, MsgType::HEARTBEAT), 1); // no new ones
}

// ---------------------------------------------------------------------------
// Test 3: arm uses a fresh, monotonically increasing nonce each call
// arm → disarm → arm: second ARM packet has a greater nonce than the first.
// ---------------------------------------------------------------------------
void test_arm_uses_increasing_nonce() {
    FakeTransport t;
    BoxLink link(t);
    ShowRunner runner(link, 500);

    runner.arm(0);
    // First ARM packet
    int armCount = countType(t, MsgType::ARM);
    CHECK_EQ(armCount, 1);
    uint32_t nonce1 = 0;
    for (auto& s : t.sent) {
        if ((MsgType)s.pkt.type == MsgType::ARM) { nonce1 = s.pkt.nonce; break; }
    }

    runner.disarm(0);
    runner.arm(1);
    // Second ARM packet
    uint32_t nonce2 = 0;
    int found = 0;
    for (auto& s : t.sent) {
        if ((MsgType)s.pkt.type == MsgType::ARM) {
            if (++found == 2) { nonce2 = s.pkt.nonce; break; }
        }
    }
    CHECK_EQ(countType(t, MsgType::ARM), 2);
    CHECK(nonce2 > nonce1);
}

// ---------------------------------------------------------------------------
// Test 4: sequence fires cues in order, respecting 1-deep ACK constraint
// Load two steps both due at t=0. tick(0) fires the first (box0 ch1) and pends.
// tick(0) again leaves second pending since ACK not yet received.
// After acking the first fire, tick(1) sends the second (box1 ch5).
// ---------------------------------------------------------------------------
void test_sequence_fires_in_order_with_ack_serialization() {
    FakeTransport t;
    BoxLink link(t);
    ShowRunner runner(link, 500);

    SeqStep steps[] = {
        {0, 0, 1},  // timeMs=0, boxId=0, channel=1
        {0, 1, 5},  // timeMs=0, boxId=1, channel=5
    };
    runner.loadSequence(steps, 2);
    runner.arm(0);
    runner.startSequence(0);

    // tick(0): first step fires, second is held back because ACK pending
    runner.tick(0);
    CHECK_EQ(countType(t, MsgType::FIRE), 1);
    CHECK(link.pendingAck());

    // Verify first fire was box0, ch1
    uint32_t firstFireId = 0;
    for (auto& s : t.sent) {
        if ((MsgType)s.pkt.type == MsgType::FIRE) {
            CHECK_EQ(s.box, 0);
            CHECK_EQ(s.pkt.targetChannel, 1);
            firstFireId = s.pkt.id;
            break;
        }
    }
    CHECK(firstFireId != 0);

    // tick again while still pending — should NOT fire a second cue
    runner.tick(1);
    CHECK_EQ(countType(t, MsgType::FIRE), 1); // still only 1

    // ACK the first fire
    link.onAck(firstFireId, 2);
    CHECK(!link.pendingAck());

    // tick again — second step should now fire
    runner.tick(2);
    CHECK_EQ(countType(t, MsgType::FIRE), 2);

    // Verify second fire was box1, ch5
    int fireIdx = 0;
    for (auto& s : t.sent) {
        if ((MsgType)s.pkt.type == MsgType::FIRE) {
            if (++fireIdx == 2) {
                CHECK_EQ(s.box, 1);
                CHECK_EQ(s.pkt.targetChannel, 5);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Test 5: estop stops sequence, sends ESTOP, armed()==false, sequenceRunning()==false
// ---------------------------------------------------------------------------
void test_estop_stops_everything() {
    FakeTransport t;
    BoxLink link(t);
    ShowRunner runner(link, 500);

    SeqStep steps[] = { {1000, 0, 1} };
    runner.loadSequence(steps, 1);
    runner.arm(0);
    runner.startSequence(0);
    CHECK(runner.sequenceRunning());
    CHECK(runner.armed());

    runner.estop(0);

    CHECK_EQ(countType(t, MsgType::ESTOP), 1);
    CHECK(!runner.armed());
    CHECK(!runner.sequenceRunning());
}

// ---------------------------------------------------------------------------
// Test 6: disarm stops heartbeats and sequence but does NOT send ESTOP
// ---------------------------------------------------------------------------
void test_disarm_stops_sequence_no_estop() {
    FakeTransport t;
    BoxLink link(t);
    ShowRunner runner(link, 500);

    SeqStep steps[] = { {1000, 0, 1} };
    runner.loadSequence(steps, 1);
    runner.arm(0);
    runner.startSequence(0);
    CHECK(runner.sequenceRunning());

    runner.disarm(0);

    CHECK_EQ(countType(t, MsgType::DISARM), 1);
    CHECK_EQ(countType(t, MsgType::ESTOP),  0); // disarm != estop
    CHECK(!runner.armed());
    CHECK(!runner.sequenceRunning());
}

// ---------------------------------------------------------------------------
// Test 7: fireManual ignored when disarmed; fires when armed and link idle
// ---------------------------------------------------------------------------
void test_fire_manual_armed_only() {
    FakeTransport t;
    BoxLink link(t);
    ShowRunner runner(link, 500);

    // Attempt manual fire while disarmed — must be ignored
    runner.fireManual(0, 3, 0);
    CHECK_EQ(countType(t, MsgType::FIRE), 0);

    // Arm, then manual fire should work
    runner.arm(0);
    runner.fireManual(0, 3, 0);
    CHECK_EQ(countType(t, MsgType::FIRE), 1);
    CHECK(link.pendingAck());

    // Another manual fire while pending — must be ignored
    runner.fireManual(0, 4, 0);
    CHECK_EQ(countType(t, MsgType::FIRE), 1);
}

// ---------------------------------------------------------------------------
// Test 8: startSequence is a no-op when not armed
// ---------------------------------------------------------------------------
void test_start_sequence_requires_armed() {
    FakeTransport t;
    BoxLink link(t);
    ShowRunner runner(link, 500);

    SeqStep steps[] = { {0, 0, 1} };
    runner.loadSequence(steps, 1);
    runner.startSequence(0); // not armed yet
    CHECK(!runner.sequenceRunning());

    runner.arm(0);
    runner.startSequence(0);
    CHECK(runner.sequenceRunning());
}

// ---------------------------------------------------------------------------
// Test 9: tick always calls link_.tick (verifies retry traffic still flows)
// One indirect check: after a fire times out through runner.tick calls,
// link.lastFailedId() becomes nonzero (retry machinery ran).
// ---------------------------------------------------------------------------
void test_tick_drives_link_tick() {
    FakeTransport t;
    BoxLinkConfig cfg;
    cfg.ackTimeoutMs = 100;
    cfg.maxRetries   = 1;
    BoxLink link(t, cfg);
    ShowRunner runner(link, 500);

    runner.arm(0);
    runner.fireManual(0, 1, 0);
    CHECK(link.pendingAck());

    // Run runner.tick enough to exhaust retries (1 resend, then give up).
    runner.tick(110); // triggers first retry
    runner.tick(220); // triggers give-up
    CHECK(!link.pendingAck());
    CHECK(link.lastFailedId() != 0);
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main() {
    RUN(test_heartbeat_cadence_while_armed);
    RUN(test_no_heartbeat_while_disarmed);
    RUN(test_arm_uses_increasing_nonce);
    RUN(test_sequence_fires_in_order_with_ack_serialization);
    RUN(test_estop_stops_everything);
    RUN(test_disarm_stops_sequence_no_estop);
    RUN(test_fire_manual_armed_only);
    RUN(test_start_sequence_requires_armed);
    RUN(test_tick_drives_link_tick);
    return REPORT();
}
