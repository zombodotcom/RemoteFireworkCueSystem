#include "check.h"
#include "arming.h"
using namespace fw;

static ArmingStateMachine armed_at(uint32_t now) {
    ArmingStateMachine m;            // default 2000ms timeout
    m.setPhysicalSwitch(true, now);
    CHECK(m.arm(1, now));
    return m;
}

void test_constructs_safe() {
    ArmingStateMachine m;
    CHECK_EQ((int)m.state(), (int)BoxState::SAFE);
}
void test_cannot_arm_with_switch_off() {
    ArmingStateMachine m;
    m.setPhysicalSwitch(false, 0);
    CHECK(!m.arm(1, 0));
    CHECK_EQ((int)m.state(), (int)BoxState::SAFE);
}
void test_arms_with_switch_on() {
    ArmingStateMachine m;
    m.setPhysicalSwitch(true, 0);
    CHECK(m.arm(1, 0));
    CHECK_EQ((int)m.state(), (int)BoxState::ARMED);
}
void test_nonce_replay_rejected_within_session() {
    ArmingStateMachine m;
    m.setPhysicalSwitch(true, 0);
    CHECK(m.arm(7, 0));
    CHECK(!m.arm(7, 0));   // same nonce replayed while still armed -> rejected
}
void test_nonce_reusable_after_disarm() {
    ArmingStateMachine m;
    m.setPhysicalSwitch(true, 0);
    CHECK(m.arm(7, 0));
    m.disarm(0);
    CHECK(m.arm(7, 0));    // per-session reset: nonce 7 accepted again in a NEW session
}
void test_switch_off_disarms() {
    ArmingStateMachine m = armed_at(0);
    m.setPhysicalSwitch(false, 100);
    CHECK_EQ((int)m.state(), (int)BoxState::SAFE);
}
void test_estop_latches_until_cleared() {
    ArmingStateMachine m = armed_at(0);
    m.estop(50);
    CHECK_EQ((int)m.state(), (int)BoxState::SAFE);
    CHECK(!m.arm(2, 60));            // still latched
    m.clearEstop(70);
    CHECK(m.arm(3, 80));
}
void test_heartbeat_timeout_disarms_when_idle() {
    ArmingStateMachine m = armed_at(0);
    m.update(2001);                  // > 2000ms since last heartbeat
    CHECK_EQ((int)m.state(), (int)BoxState::SAFE);
}
void test_heartbeat_keeps_armed() {
    ArmingStateMachine m = armed_at(0);
    m.heartbeat(1500);
    m.update(2001);                  // only 501ms since heartbeat
    CHECK_EQ((int)m.state(), (int)BoxState::ARMED);
}
void test_sequence_running_ignores_heartbeat_timeout() {
    ArmingStateMachine m = armed_at(0);
    m.setSequenceRunning(true);
    m.update(99999);
    CHECK_EQ((int)m.state(), (int)BoxState::ARMED);
}
void test_canfire_requires_all_gates() {
    ArmingStateMachine m = armed_at(0);
    CHECK(m.canFire(100));
    m.estop(120);
    CHECK(!m.canFire(130));
}
void test_canfire_false_on_stale_heartbeat_without_update() {
    ArmingStateMachine m = armed_at(0);   // lastHeartbeat = 0, timeout 2000ms
    CHECK(m.canFire(1999));               // still fresh
    CHECK(!m.canFire(2001));              // stale heartbeat rejected even though update() was not called
}
void test_sequence_flag_cleared_on_disarm() {
    ArmingStateMachine m = armed_at(0);
    m.setSequenceRunning(true);
    m.disarm(0);                          // goSafe() must clear sequenceRunning_
    m.setPhysicalSwitch(true, 0);
    CHECK(m.arm(9, 0));                   // new session
    m.update(99999);                      // heartbeat long stale; if seq flag had persisted, machine would stay ARMED
    CHECK_EQ((int)m.state(), (int)BoxState::SAFE);  // proves the flag was cleared
}

int main() {
    RUN(test_constructs_safe);
    RUN(test_cannot_arm_with_switch_off);
    RUN(test_arms_with_switch_on);
    RUN(test_nonce_replay_rejected_within_session);
    RUN(test_nonce_reusable_after_disarm);
    RUN(test_switch_off_disarms);
    RUN(test_estop_latches_until_cleared);
    RUN(test_heartbeat_timeout_disarms_when_idle);
    RUN(test_heartbeat_keeps_armed);
    RUN(test_sequence_running_ignores_heartbeat_timeout);
    RUN(test_canfire_requires_all_gates);
    RUN(test_canfire_false_on_stale_heartbeat_without_update);
    RUN(test_sequence_flag_cleared_on_disarm);
    return REPORT();
}
