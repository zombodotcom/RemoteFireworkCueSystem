#include "check.h"
#include "box_controller.h"
#include "fake_channel_driver.h"
using namespace fw;

static CommandPacket cmd(MsgType t, uint32_t id, uint8_t boxId, uint8_t ch, uint32_t nonce) {
    CommandPacket p{};
    p.type = (uint8_t)t; p.id = id; p.boxId = boxId; p.targetChannel = ch; p.nonce = nonce;
    p.crc = computeCrc(p);
    return p;
}
static BoxController armedBox(FakeChannelDriver& drv, uint32_t now) {
    BoxController b(drv, BoxConfig{});
    b.begin();
    b.setPhysicalSwitch(true, now);
    b.onCommand(cmd(MsgType::ARM, 1, 0, 0, 100), now);   // arm with nonce 100
    return b;
}

void test_begin_is_safe_and_alloff() {
    FakeChannelDriver drv;
    BoxController box(drv, BoxConfig{});
    box.begin();
    CHECK_EQ((int)box.state(), (int)BoxState::SAFE);   // never boot armed
    CHECK(drv.allOffCount >= 1);                        // outputs forced off at boot
    CHECK_EQ(drv.countOn(), 0);
}

void test_fire_when_armed_energizes_then_expires() {
    FakeChannelDriver drv;
    BoxController b = armedBox(drv, 0);
    CHECK_EQ((int)b.state(), (int)BoxState::ARMED);
    b.onCommand(cmd(MsgType::FIRE, 10, 0, 3, 0), 0);
    CHECK(drv.on[3]);                       // channel 3 energized
    b.tick(401);                            // > FIRE_MS(400)
    CHECK(!drv.on[3]);                      // auto de-energized
    CHECK_EQ((int)b.state(), (int)BoxState::ARMED);  // still armed (heartbeat fresh at arm)
}

void test_fire_ignored_when_disarmed() {
    FakeChannelDriver drv;
    BoxController b(drv, BoxConfig{});
    b.begin();
    b.onCommand(cmd(MsgType::FIRE, 11, 0, 2, 0), 0);  // SAFE
    CHECK(!drv.on[2]);
}

void test_fire_wrong_box_ignored() {
    FakeChannelDriver drv;
    BoxController b = armedBox(drv, 0);     // this box is boxId 0
    b.onCommand(cmd(MsgType::FIRE, 12, 1, 4, 0), 0);  // addressed to box 1
    CHECK(!drv.on[4]);
}

void test_fire_out_of_range_ignored() {
    FakeChannelDriver drv;
    BoxController b = armedBox(drv, 0);
    b.onCommand(cmd(MsgType::FIRE, 13, 0, 16, 0), 0); // MAX_CHANNELS==16 -> 16 invalid
    CHECK_EQ(drv.countOn(), 0);
}

void test_duplicate_fire_id_ignored() {
    FakeChannelDriver drv;
    BoxController b = armedBox(drv, 0);
    b.onCommand(cmd(MsgType::FIRE, 20, 0, 5, 0), 0);
    b.tick(401); CHECK(!drv.on[5]);         // first fire expired
    b.onCommand(cmd(MsgType::FIRE, 20, 0, 5, 0), 500); // SAME id replayed
    CHECK(!drv.on[5]);                      // duplicate rejected, not re-fired
}

void test_corrupt_crc_ignored() {
    FakeChannelDriver drv;
    BoxController b = armedBox(drv, 0);
    CommandPacket p = cmd(MsgType::FIRE, 30, 0, 6, 0);
    p.targetChannel = 7;                    // tamper after CRC
    b.onCommand(p, 0);
    CHECK_EQ(drv.countOn(), 0);             // bad CRC -> ignored
}

void test_arm_requires_physical_switch() {
    FakeChannelDriver drv;
    BoxController b(drv, BoxConfig{});
    b.begin();
    b.onCommand(cmd(MsgType::ARM, 1, 0, 0, 100), 0);   // switch off
    CHECK_EQ((int)b.state(), (int)BoxState::SAFE);
    b.setPhysicalSwitch(true, 0);
    b.onCommand(cmd(MsgType::ARM, 2, 0, 0, 101), 0);
    CHECK_EQ((int)b.state(), (int)BoxState::ARMED);
}

void test_switch_off_disarms_and_kills_outputs() {
    FakeChannelDriver drv;
    BoxController b = armedBox(drv, 0);
    b.onCommand(cmd(MsgType::FIRE, 40, 0, 1, 0), 0);
    CHECK(drv.on[1]);
    b.setPhysicalSwitch(false, 10);    // physical kill
    b.tick(11);                        // tick sees ARMED->SAFE
    CHECK_EQ((int)b.state(), (int)BoxState::SAFE);
    CHECK_EQ(drv.countOn(), 0);        // all outputs forced off
}

void test_estop_kills_outputs_and_latches() {
    FakeChannelDriver drv;
    BoxController b = armedBox(drv, 0);
    b.onCommand(cmd(MsgType::FIRE, 41, 0, 2, 0), 0);
    CHECK(drv.on[2]);
    b.onCommand(cmd(MsgType::ESTOP, 42, 0, 0, 0), 5);
    CHECK_EQ(drv.countOn(), 0);                       // E-STOP forces all off
    CHECK_EQ((int)b.state(), (int)BoxState::SAFE);
    b.onCommand(cmd(MsgType::ARM, 43, 0, 0, 102), 6); // still latched
    CHECK_EQ((int)b.state(), (int)BoxState::SAFE);
}

void test_heartbeat_loss_disarms_when_idle() {
    FakeChannelDriver drv;
    BoxController b = armedBox(drv, 0);   // armed at t=0, heartbeat at 0
    b.tick(2001);                          // > 2000ms timeout, no heartbeat
    CHECK_EQ((int)b.state(), (int)BoxState::SAFE);
}

void test_heartbeat_keeps_armed() {
    FakeChannelDriver drv;
    BoxController b = armedBox(drv, 0);
    b.onCommand(cmd(MsgType::HEARTBEAT, 50, 0, 0, 0), 1500);
    b.tick(2001);                          // only 501ms since last heartbeat
    CHECK_EQ((int)b.state(), (int)BoxState::ARMED);
}

int main() {
    RUN(test_begin_is_safe_and_alloff);
    RUN(test_fire_when_armed_energizes_then_expires);
    RUN(test_fire_ignored_when_disarmed);
    RUN(test_fire_wrong_box_ignored);
    RUN(test_fire_out_of_range_ignored);
    RUN(test_duplicate_fire_id_ignored);
    RUN(test_corrupt_crc_ignored);
    RUN(test_arm_requires_physical_switch);
    RUN(test_switch_off_disarms_and_kills_outputs);
    RUN(test_estop_kills_outputs_and_latches);
    RUN(test_heartbeat_loss_disarms_when_idle);
    RUN(test_heartbeat_keeps_armed);
    return REPORT();
}
