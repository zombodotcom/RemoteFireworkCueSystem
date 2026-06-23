#pragma once
#include "expander_codec.h"
#include <cstdint>

// ---- Deferred polarity decision (finalize after the SSR polarity test, Task 6) ----
// Safe default until tested: active-low boards (all-OFF = all bits HIGH).
namespace board {
constexpr fwbox::FireLevel FIRE_LEVEL = fwbox::FireLevel::ACTIVE_LOW;
// Expander: PCF8575 (default; MCP23017 register sequence differs — see expander_driver.cpp)
#define EXPANDER_PCF8575 1   // set to 0 and EXPANDER_MCP23017 1 if standardizing on MCP23017
constexpr uint8_t EXPANDER_I2C_ADDR = 0x20;   // A0..A2 = GND
constexpr int I2C_SDA_GPIO = 21;
constexpr int I2C_SCL_GPIO = 22;
constexpr int I2C_FREQ_HZ  = 100000;          // 100kHz; long SSR ribbon -> keep modest
constexpr int ARM_SWITCH_GPIO = 4;            // input w/ pull; switch closes to the active level
constexpr int STATUS_LED_GPIO = 5;            // addressable LED data
constexpr int STATUS_LED_COUNT = 1;
constexpr uint8_t THIS_BOX_ID = 0;            // box 1 = 0, box 2 = 1 (flash per box)
// Controller MAC the box accepts traffic from (fill in from the controller's printed MAC).
constexpr uint8_t CONTROLLER_MAC[6] = {0xb0,0xcb,0xd8,0x89,0x9e,0x69}; // controller SoftAP MAC
}
