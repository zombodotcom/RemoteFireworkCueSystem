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

int main() {
    RUN(test_command_crc_roundtrip);
    RUN(test_command_crc_detects_tamper);
    return REPORT();
}
