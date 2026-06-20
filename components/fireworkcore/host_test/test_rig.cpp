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

int main() {
    RUN(test_boots_safe);
    RUN(test_arm_requires_switch);
    RUN(test_fire_when_armed_then_pulse_expires);
    RUN(test_fire_rejected_when_estopped);
    RUN(test_fire_rejected_out_of_range);
    RUN(test_idle_heartbeat_loss_disarms);
    return REPORT();
}
