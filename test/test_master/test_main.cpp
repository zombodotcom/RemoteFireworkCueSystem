#include <Arduino.h>
#include <unity.h>
#include "../../src/main.cpp"  // If needed to access CRC, etc.

void test_crc_generation(void) {
    CommandPacket pkt;
    strcpy(pkt.cmd, "FIRE");
    pkt.id = 123;
    pkt.crc = calculateCRC(pkt);
    TEST_ASSERT_EQUAL_UINT32(pkt.crc, calculateCRC(pkt));
}

void setup() {
    UNITY_BEGIN();
    RUN_TEST(test_crc_generation);
    UNITY_END();
}

void loop() {}
