#include <Arduino.h>
#include <unity.h>
#include "../../src/main.cpp"

void test_arm_state_save_load(void) {
    saveArmState(true);
    TEST_ASSERT_TRUE(loadArmState());

    saveArmState(false);
    TEST_ASSERT_FALSE(loadArmState());
}

void setup() {
    EEPROM.begin(4);
    UNITY_BEGIN();
    RUN_TEST(test_arm_state_save_load);
    UNITY_END();
}

void loop() {}
