#include "check.h"
#include "sim_bindings.h"

void test_boots_safe() {
    rig_reset();
    CHECK_EQ(rig_box_state(0), 0);   // SAFE
    CHECK_EQ(rig_box_state(1), 0);
    CHECK_EQ(rig_box_switch(0), 0);
    CHECK_EQ(rig_channel_firing(0, 3), 0);
}
void test_arm_requires_switch() {
    rig_reset();
    CHECK_EQ(rig_arm(1, 0), 0);       // no switches on -> none arm
    rig_set_switch(0, 1, 0);
    rig_heartbeat(0);
    CHECK_EQ(rig_arm(2, 0), 1);       // only box 0 armed
    CHECK_EQ(rig_box_state(0), 1);
    CHECK_EQ(rig_box_state(1), 0);
}
void test_fire_when_armed_then_pulse_expires() {
    rig_reset();
    rig_set_switch(0, 1, 0);
    rig_heartbeat(0);
    rig_arm(1, 0);
    CHECK_EQ(rig_fire(0, 3, 0), 1);
    CHECK_EQ(rig_channel_firing(0, 3), 1);
    rig_tick(401);                    // > FIRE_MS(400), heartbeat still fresh (<2000)
    CHECK_EQ(rig_channel_firing(0, 3), 0);
    CHECK_EQ(rig_box_state(0), 1);    // still armed
}
void test_fire_rejected_when_estopped() {
    rig_reset();
    rig_set_switch(0, 1, 0);
    rig_heartbeat(0);
    rig_arm(1, 0);
    rig_estop(0);
    CHECK_EQ(rig_box_can_fire(0, 0), 0);
    CHECK_EQ(rig_fire(0, 5, 0), 0);
    CHECK_EQ(rig_channel_firing(0, 5), 0);
}
void test_fire_rejected_out_of_range() {
    rig_reset();
    rig_set_switch(0, 1, 0);
    rig_heartbeat(0);
    rig_arm(1, 0);
    CHECK_EQ(rig_fire(0, 16, 0), 0);  // MAX_CHANNELS == 16, so 16 is out of range
}
void test_idle_heartbeat_loss_disarms() {
    rig_reset();
    rig_set_switch(0, 1, 0);
    rig_heartbeat(0);
    rig_arm(1, 0);
    rig_tick(2001);                   // idle, no heartbeat since 0 -> disarm
    CHECK_EQ(rig_box_state(0), 0);
}
void test_sequence_fires_cues_in_order() {
    rig_reset();
    rig_set_switch(0, 1, 0);
    rig_set_switch(1, 1, 0);
    rig_heartbeat(0);
    rig_arm(1, 0);
    uint32_t steps[] = { 0,0,1,  100,1,5 };   // t0: box0 ch1 ; t100: box1 ch5
    rig_load_sequence(steps, 2);
    rig_start_sequence(0);
    rig_heartbeat(0); rig_tick(0);
    CHECK_EQ(rig_channel_firing(0, 1), 1);
    CHECK_EQ(rig_channel_firing(1, 5), 0);
    rig_heartbeat(100); rig_tick(100);
    CHECK_EQ(rig_channel_firing(1, 5), 1);
}
void test_sequence_running_keeps_armed_without_heartbeat() {
    rig_reset();
    rig_set_switch(0, 1, 0);
    rig_heartbeat(0);
    rig_arm(1, 0);
    uint32_t steps[] = { 5000,0,2 };          // far-future step keeps scheduler running
    rig_load_sequence(steps, 1);
    rig_start_sequence(0);
    rig_tick(3000);                            // no heartbeat since 0, but seq running -> stays armed
    CHECK_EQ(rig_box_state(0), 1);
    CHECK_EQ(rig_seq_running(), 1);
}
void test_estop_mid_show_skips_remaining_cues() {
    rig_reset();
    rig_set_switch(0, 1, 0);
    rig_heartbeat(0);
    rig_arm(1, 0);
    uint32_t steps[] = { 0,0,1,  200,0,2 };
    rig_load_sequence(steps, 2);
    rig_start_sequence(0);
    rig_heartbeat(0); rig_tick(0);
    CHECK_EQ(rig_channel_firing(0, 1), 1);
    rig_estop(50);                             // kill mid-show
    rig_tick(200);                             // second cue must NOT fire
    CHECK_EQ(rig_channel_firing(0, 2), 0);
    CHECK_EQ(rig_seq_running(), 0);
}

void test_manual_fire_blocked_after_sequence_done_with_stale_heartbeat() {
    rig_reset();
    rig_set_switch(0, 1, 0);
    rig_heartbeat(0);
    rig_arm(1, 0);
    uint32_t steps[] = { 0,0,1 };          // single cue at t=0
    rig_load_sequence(steps, 1);
    rig_start_sequence(0);
    rig_tick(0);                            // dispatches the only cue; scheduler now done
    // No tick since; heartbeat last at 0. At t=5000 the heartbeat is stale (>2000ms).
    // Before the fix, sequenceRunning_ was still true -> canFire true -> fire allowed (BUG).
    CHECK_EQ(rig_box_can_fire(0, 5000), 0);
    CHECK_EQ(rig_fire(0, 2, 5000), 0);      // must be rejected
    CHECK_EQ(rig_channel_firing(0, 2), 0);
}

int main() {
    RUN(test_boots_safe);
    RUN(test_arm_requires_switch);
    RUN(test_fire_when_armed_then_pulse_expires);
    RUN(test_fire_rejected_when_estopped);
    RUN(test_fire_rejected_out_of_range);
    RUN(test_idle_heartbeat_loss_disarms);
    RUN(test_sequence_fires_cues_in_order);
    RUN(test_sequence_running_keeps_armed_without_heartbeat);
    RUN(test_estop_mid_show_skips_remaining_cues);
    RUN(test_manual_fire_blocked_after_sequence_done_with_stale_heartbeat);
    return REPORT();
}
