#include "check.h"
#include <cstring>
#include "protocol.h"

void test_command_crc_roundtrip() {
    fw::CommandPacket p{};
    p.type = (uint8_t)fw::MsgType::FIRE;
    std::strncpy(p.cmd, "FIRE", sizeof(p.cmd));
    p.id = 42; p.boxId = 1; p.targetChannel = 7; p.nonce = 0;
    p.crc = fw::computeCrc(p);
    CHECK(fw::crcValid(p));
}
void test_command_crc_detects_tamper() {
    fw::CommandPacket p{};
    p.type = (uint8_t)fw::MsgType::FIRE;
    p.id = 42; p.boxId = 1; p.targetChannel = 7;
    p.crc = fw::computeCrc(p);
    p.targetChannel = 8;            // tamper after CRC
    CHECK(!fw::crcValid(p));
}

void test_channel_in_range() {
    CHECK(fw::channelInRange(0));
    CHECK(fw::channelInRange(15));
    CHECK(!fw::channelInRange(16));
    CHECK(!fw::channelInRange(200));
}
void test_ack_crc_roundtrip_and_tamper() {
    fw::AckPacket a{};
    a.type = (uint8_t)fw::MsgType::ACK;
    a.responseToId = 99; a.deviceStatus = 1; a.timestamp = 1234;
    std::strncpy(a.note, "IGNITED OK", sizeof(a.note));
    a.crc = fw::computeCrc(a);
    CHECK(fw::crcValid(a));
    a.deviceStatus = 2;            // tamper after CRC
    CHECK(!fw::crcValid(a));
}

void test_status_crc_roundtrip_and_tamper() {
    fw::StatusPacket s{};
    s.type = (uint8_t)fw::MsgType::STATUS;
    s.boxId = 0; s.state = 1; s.firedBitmap = 0b00000101; s.lastFiredChannel = 2; s.timestamp = 10;
    s.crc = fw::computeCrc(s);
    CHECK(fw::crcValid(s));
    s.firedBitmap = 0b00000111;     // tamper after CRC
    CHECK(!fw::crcValid(s));
}

void test_display_packet_sizes() {
    // Packed wire sizes must be stable and fit the 250-byte ESP-NOW limit.
    CHECK_EQ((int)sizeof(fw::DisplayStatusPacket), 45);
    CHECK_EQ((int)sizeof(fw::DisplayEventPacket), 54);
    CHECK((int)sizeof(fw::DisplayStatusPacket) <= 250);
    CHECK((int)sizeof(fw::DisplayEventPacket) <= 250);
    fw::DisplayStatusPacket s{};
    s.type = (uint8_t)fw::MsgType::DISP_STATUS;
    CHECK_EQ((int)s.type, 8);
    fw::DisplayEventPacket e{};
    e.type = (uint8_t)fw::MsgType::DISP_EVENT;
    CHECK_EQ((int)e.type, 9);
}

int main() {
    RUN(test_command_crc_roundtrip);
    RUN(test_command_crc_detects_tamper);
    RUN(test_channel_in_range);
    RUN(test_ack_crc_roundtrip_and_tamper);
    RUN(test_status_crc_roundtrip_and_tamper);
    RUN(test_display_packet_sizes);
    return REPORT();
}
