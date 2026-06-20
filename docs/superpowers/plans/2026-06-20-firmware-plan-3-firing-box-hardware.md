# Firmware Plan 3 — Firing-Box Hardware Bring-up

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Turn the build-only firing-box skeleton into a real ESP32 firing box: an I²C expander `ChannelDriver` (16 SSR outputs), a physical arm-switch GPIO, addressable-LED status, and ESP-NOW receive→`BoxController`→ACK — then flash it and verify every interlock on the bench with LEDs (no pyro).

**Architecture:** Keep the host-tested `BoxController` brain unchanged. Add ESP-IDF hardware adapters in `firmware/main/`: a pure, host-tested bit-packing helper (`expander_codec`) that maps channel+polarity → the expander's 16-bit port word; a real `ExpanderChannelDriver` implementing `fw::ChannelDriver` over the IDF `i2c_master` driver using that helper; GPIO arm-switch polling; `led_strip` status; and an `esp_now` receive path that feeds `CommandPacket`s into `box.onCommand` and sends `AckPacket`s back. Polarity/expander selection is two compile-time constants finalized after the SSR polarity test.

**Tech Stack:** ESP-IDF v6.0.1 (`idf.py`, **PowerShell only**), C++11, IDF components `driver` (i2c_master, gpio), `esp_now`, `esp_wifi`, `led_strip`. Host-testable codec uses CMake/Ninja/g++ + CTest (Git Bash).

## Global Constraints

- Do NOT change `components/fireworkcore/` safety logic. Hardware adapters live in `firmware/main/`. The host-testable `expander_codec` may live in `firmware/main/` with a host test, OR in `components/fireworkcore/` if it stays framework-free — prefer `firmware/main/` + a dedicated host test target to keep the core minimal.
- **Boot-safe hardware rule (spec §3):** every SSR input has an external pull resistor holding it in the OFF state, so a floating/booting expander cannot fire. The expander only ever drives a pin to the active level deliberately. `ExpanderChannelDriver` construction must write the all-OFF port word before anything else, and `BoxController::begin()` calls `allOff()` first.
- **Expander + polarity (deferred until SSR boards arrive ~2026-06-24):** two constants in one header, `EXPANDER_TYPE` (`MCP23017` | `PCF8575`) and `FIRE_LEVEL` (`HIGH` | `LOW`). Default to the safest assumption until the polarity test (Task 6): `FIRE_LEVEL = LOW` (active-low) with the all-OFF word = all-bits-HIGH. Both boxes use identical firmware + expander.
- `MAX_CHANNELS = 16`; channel→bit mapping is GPA0..7 then GPB0..7 (MCP23017) or P0..P7,P10..P17 (PCF8575) — the codec owns this mapping.
- ESP-NOW: the box peers with the controller MAC; on a valid `CommandPacket` (CRC checked inside `onCommand`) it calls `box.onCommand(pkt, nowMs)` and, for a FIRE it acted on, sends an `AckPacket`. Time is `esp_timer_get_time()/1000`.
- The control loop ticks `box.tick(nowMs)` at ~20ms and must never block longer than the heartbeat timeout; the task WDT stays enabled.
- **Toolchain:** `idf.py` runs in **PowerShell** after `& 'C:\esp\v6.0.1\esp-idf\export.ps1'`. Host codec tests run in Git Bash + ctest. Flashing: `idf.py -p <COMx> flash monitor` (the user runs this, or confirms the COM port).
- Firmware build artifacts (`firmware/build/`, `sdkconfig`, `managed_components/`) stay git-ignored.
- **Verification ceiling:** Tasks 1 is host-tested; Tasks 2–5 are build-only until an ESP32 is connected, then flash-and-observe; Task 6 needs the SSR boards. The plan marks each task's verification level explicitly.

---

## File Structure

```
firmware/main/expander_codec.h            NEW: pure channel/polarity -> 16-bit word helpers (host-testable)
firmware/main/board_config.h              NEW: EXPANDER_TYPE, FIRE_LEVEL, pin assignments, controller MAC
firmware/main/expander_driver.h/.cpp      NEW: ExpanderChannelDriver : fw::ChannelDriver over i2c_master
firmware/main/arm_switch.h/.cpp           NEW: debounced GPIO arm-switch read
firmware/main/status_leds.h/.cpp          NEW: led_strip status (SAFE/ARMED/E-STOP/link-lost)
firmware/main/espnow_link.h/.cpp          NEW: esp_now init, RX -> CommandPacket, ACK TX
firmware/main/firing_box_main.cpp         MODIFY: wire real driver + arm switch + LEDs + esp_now into BoxController
firmware/main/CMakeLists.txt              MODIFY: add new srcs + REQUIRES (driver esp_wifi esp_now led_strip nvs_flash)
firmware/test_codec/                      NEW: host CMake + CTest for expander_codec (Git Bash)
docs/POLARITY_TEST.md                     NEW: the safe LED polarity procedure (Task 6)
```

---

## Task 1: Expander codec (pure, host-tested)

**Files:**
- Create: `firmware/main/expander_codec.h`
- Create: `firmware/test_codec/CMakeLists.txt`
- Create: `firmware/test_codec/test_codec.cpp`
- Create: `firmware/test_codec/check.h` (copy of the host harness assert macros)

**Interfaces:**
- Produces (framework-free, in `namespace fwbox`):
  - `enum class FireLevel { ACTIVE_HIGH, ACTIVE_LOW };`
  - `uint16_t allOffWord(FireLevel lvl);` — the 16-bit port word with every channel OFF (`0x0000` for ACTIVE_HIGH, `0xFFFF` for ACTIVE_LOW).
  - `uint16_t applyChannel(uint16_t word, uint8_t channel, bool on, FireLevel lvl);` — returns `word` with `channel`'s bit set to the level meaning `on`/off. `channel>=16` returns `word` unchanged.

- [ ] **Step 1: Write the failing test**

`firmware/test_codec/check.h`: copy the contents of `components/fireworkcore/host_test/check.h` verbatim (same `CHECK`/`CHECK_EQ`/`RUN`/`REPORT` macros).

`firmware/test_codec/test_codec.cpp`:
```cpp
#include "check.h"
#include "expander_codec.h"
using namespace fwbox;

void test_alloff_word_per_polarity() {
    CHECK_EQ(allOffWord(FireLevel::ACTIVE_HIGH), 0x0000u);
    CHECK_EQ(allOffWord(FireLevel::ACTIVE_LOW),  0xFFFFu);
}
void test_apply_active_high() {
    uint16_t w = allOffWord(FireLevel::ACTIVE_HIGH); // 0x0000
    w = applyChannel(w, 0, true, FireLevel::ACTIVE_HIGH);
    CHECK_EQ(w, 0x0001u);                              // bit0 high = on
    w = applyChannel(w, 0, false, FireLevel::ACTIVE_HIGH);
    CHECK_EQ(w, 0x0000u);
    w = applyChannel(w, 15, true, FireLevel::ACTIVE_HIGH);
    CHECK_EQ(w, 0x8000u);
}
void test_apply_active_low() {
    uint16_t w = allOffWord(FireLevel::ACTIVE_LOW);  // 0xFFFF
    w = applyChannel(w, 0, true, FireLevel::ACTIVE_LOW);
    CHECK_EQ(w, 0xFFFEu);                              // bit0 low = on
    w = applyChannel(w, 0, false, FireLevel::ACTIVE_LOW);
    CHECK_EQ(w, 0xFFFFu);
}
void test_out_of_range_no_change() {
    uint16_t w = 0x1234u;
    CHECK_EQ(applyChannel(w, 16, true, FireLevel::ACTIVE_HIGH), 0x1234u);
}

int main() {
    RUN(test_alloff_word_per_polarity);
    RUN(test_apply_active_high);
    RUN(test_apply_active_low);
    RUN(test_out_of_range_no_change);
    return REPORT();
}
```

`firmware/test_codec/CMakeLists.txt`:
```cmake
cmake_minimum_required(VERSION 3.16)
project(expander_codec_test CXX)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
enable_testing()
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../main ${CMAKE_CURRENT_SOURCE_DIR})
add_executable(test_codec test_codec.cpp)
add_test(NAME test_codec COMMAND test_codec)
```

- [ ] **Step 2: Configure + build — verify FAIL (Git Bash)**

Run:
```
cmake -S firmware/test_codec -B build/test_codec -G Ninja -DCMAKE_CXX_COMPILER=g++
cmake --build build/test_codec
```
Expected: FAIL — `expander_codec.h` not found.

- [ ] **Step 3: Write the codec**

`firmware/main/expander_codec.h`:
```cpp
#pragma once
#include <cstdint>
namespace fwbox {

enum class FireLevel { ACTIVE_HIGH, ACTIVE_LOW };

inline uint16_t allOffWord(FireLevel lvl) {
    return lvl == FireLevel::ACTIVE_HIGH ? 0x0000u : 0xFFFFu;
}

// Set `channel`'s bit so it means `on` for the given polarity. Out-of-range: unchanged.
inline uint16_t applyChannel(uint16_t word, uint8_t channel, bool on, FireLevel lvl) {
    if (channel >= 16) return word;
    const uint16_t mask = (uint16_t)(1u << channel);
    bool bitHigh = (lvl == FireLevel::ACTIVE_HIGH) ? on : !on; // active-low: on => bit low
    if (bitHigh) word |= mask; else word &= (uint16_t)~mask;
    return word;
}

} // namespace fwbox
```

- [ ] **Step 4: Build + run — verify PASS**

Run:
```
cmake --build build/test_codec
ctest --test-dir build/test_codec --output-on-failure
```
Expected: `test_codec` PASSES (4 tests).

- [ ] **Step 5: Commit**

```bash
git add firmware/main/expander_codec.h firmware/test_codec/
git commit -m "feat(fw): host-tested expander bit-packing codec (channel+polarity -> port word)"
```

---

## Task 2: board_config.h + ExpanderChannelDriver (I²C) — build-only

**Files:**
- Create: `firmware/main/board_config.h`
- Create: `firmware/main/expander_driver.h`
- Create: `firmware/main/expander_driver.cpp`
- Modify: `firmware/main/CMakeLists.txt`

**Interfaces:**
- Produces: `class ExpanderChannelDriver : public fw::ChannelDriver` with `esp_err_t begin();`, `void allOff() override;`, `void setChannel(uint8_t,bool) override;`. Holds a shadow `uint16_t word_` updated via the codec and written to the expander over I²C on each change.

- [ ] **Step 1: board_config.h (pins, polarity, MAC — placeholders documented)**

`firmware/main/board_config.h`:
```cpp
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
constexpr uint8_t CONTROLLER_MAC[6] = {0x00,0x00,0x00,0x00,0x00,0x00};
}
```

- [ ] **Step 2: ExpanderChannelDriver (PCF8575 path; MCP23017 path documented)**

`firmware/main/expander_driver.h`:
```cpp
#pragma once
#include "channel_driver.h"
#include "expander_codec.h"
#include "driver/i2c_master.h"
#include "esp_err.h"

class ExpanderChannelDriver : public fw::ChannelDriver {
public:
    ExpanderChannelDriver(i2c_master_bus_handle_t bus, uint8_t addr, fwbox::FireLevel lvl);
    esp_err_t begin();                 // add device, write all-OFF word
    void allOff() override;
    void setChannel(uint8_t channel, bool on) override;
private:
    esp_err_t writeWord(uint16_t word);
    i2c_master_dev_handle_t dev_ = nullptr;
    i2c_master_bus_handle_t bus_;
    uint8_t addr_;
    fwbox::FireLevel lvl_;
    uint16_t word_;
};
```

`firmware/main/expander_driver.cpp`:
```cpp
#include "expander_driver.h"
#include "esp_log.h"
static const char* TAG = "expander";

ExpanderChannelDriver::ExpanderChannelDriver(i2c_master_bus_handle_t bus, uint8_t addr, fwbox::FireLevel lvl)
    : bus_(bus), addr_(addr), lvl_(lvl), word_(fwbox::allOffWord(lvl)) {}

esp_err_t ExpanderChannelDriver::begin() {
    i2c_device_config_t dc = {};
    dc.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dc.device_address = addr_;
    dc.scl_speed_hz = 100000;
    esp_err_t err = i2c_master_bus_add_device(bus_, &dc, &dev_);
    if (err != ESP_OK) { ESP_LOGE(TAG, "add_device failed: %d", err); return err; }
    word_ = fwbox::allOffWord(lvl_);
    return writeWord(word_);            // outputs OFF before anything else
}

// PCF8575: a plain 2-byte write sets P0..P7 (low byte) then P10..P17 (high byte).
esp_err_t ExpanderChannelDriver::writeWord(uint16_t word) {
    uint8_t buf[2] = { (uint8_t)(word & 0xFF), (uint8_t)(word >> 8) };
    return i2c_master_transmit(dev_, buf, sizeof(buf), 50 /*ms*/);
}

void ExpanderChannelDriver::allOff() {
    word_ = fwbox::allOffWord(lvl_);
    writeWord(word_);
}

void ExpanderChannelDriver::setChannel(uint8_t channel, bool on) {
    word_ = fwbox::applyChannel(word_, channel, on, lvl_);
    writeWord(word_);
}
```
> **MCP23017 variant (if standardizing on it):** in `begin()`, first configure both ports as outputs by writing `0x00` to IODIRA(0x00)/IODIRB(0x01), then drive outputs via OLATA(0x14)/OLATB(0x15) with a 3-byte `{reg, low, high}` transmit. The codec's port word maps GPA0..7 = bits 0..7, GPB0..7 = bits 8..15. Implement behind `#if EXPANDER_PCF8575` / `#else`.

- [ ] **Step 3: Update the app component requirements**

Edit `firmware/main/CMakeLists.txt` to add the new sources and IDF deps:
```cmake
idf_component_register(
    SRCS "firing_box_main.cpp" "expander_driver.cpp" "arm_switch.cpp" "status_leds.cpp" "espnow_link.cpp"
    INCLUDE_DIRS "."
    REQUIRES fireworkcore driver esp_wifi esp_now led_strip nvs_flash esp_timer log
)
```
> (`arm_switch.cpp`, `status_leds.cpp`, `espnow_link.cpp` are created in Tasks 3–4; add them here now so the manifest is complete, but if you build between tasks, temporarily list only the files that exist.)

- [ ] **Step 4: Build for esp32 — verify it COMPILES (PowerShell)**

> Build-only (no hardware). After Task 4 the full app links; to compile-check just this driver before the other files exist, temporarily reduce `SRCS` to existing files.
In **PowerShell**:
```powershell
& 'C:\esp\v6.0.1\esp-idf\export.ps1'
Set-Location firmware
idf.py build 2>&1 | Select-Object -Last 15
```
Expected: compiles (the `i2c_master`/`esp_err` headers resolve). Confirm against the IDF i2c_master example at `C:\esp\v6.0.1\esp-idf\examples\peripherals\i2c\i2c_tools` if the API signatures differ in v6.0.1.

- [ ] **Step 5: Commit**

```bash
git add firmware/main/board_config.h firmware/main/expander_driver.h firmware/main/expander_driver.cpp firmware/main/CMakeLists.txt
git commit -m "feat(fw): I2C expander ChannelDriver (PCF8575) + board config (build-only)"
```

---

## Task 3: Arm-switch GPIO + LED status — build-only

**Files:**
- Create: `firmware/main/arm_switch.h` / `arm_switch.cpp`
- Create: `firmware/main/status_leds.h` / `status_leds.cpp`

**Interfaces:**
- `class ArmSwitch { void begin(); bool isOn(); };` — `gpio_config` input with pull, returns debounced closed/open. "Closed" means the operator armed the box.
- `class StatusLeds { void begin(); void show(fw::BoxState state, bool estopped, bool linkAlive); };` — drives the addressable LED: red=SAFE, green=ARMED, fast-red-blink=E-STOP, amber=link lost.

- [ ] **Step 1: ArmSwitch**

`firmware/main/arm_switch.h`:
```cpp
#pragma once
#include "driver/gpio.h"
class ArmSwitch {
public:
    explicit ArmSwitch(int gpio) : gpio_(gpio) {}
    void begin();
    bool isOn();          // true = armed position (switch closed to active level)
private:
    int gpio_;
};
```
`firmware/main/arm_switch.cpp`:
```cpp
#include "arm_switch.h"
void ArmSwitch::begin() {
    gpio_config_t io = {};
    io.pin_bit_mask = 1ULL << gpio_;
    io.mode = GPIO_MODE_INPUT;
    io.pull_up_en = GPIO_PULLUP_ENABLE;     // switch closes to GND -> LOW = armed
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io);
}
bool ArmSwitch::isOn() {
    // Closed (armed) pulls the line LOW against the internal pull-up.
    return gpio_get_level((gpio_num_t)gpio_) == 0;
}
```
> Debounce is handled by the 20ms control-loop cadence + the state machine; for a slide/key switch this is sufficient. If a noisy switch is used, sample twice ~10ms apart in `isOn()`.

- [ ] **Step 2: StatusLeds (led_strip)**

`firmware/main/status_leds.h`:
```cpp
#pragma once
#include "led_strip.h"
#include "box_controller.h"
class StatusLeds {
public:
    StatusLeds(int gpio, int count) : gpio_(gpio), count_(count) {}
    void begin();
    void show(fw::BoxState state, bool estopped, bool linkAlive, uint32_t nowMs);
private:
    led_strip_handle_t strip_ = nullptr;
    int gpio_, count_;
};
```
`firmware/main/status_leds.cpp`:
```cpp
#include "status_leds.h"
void StatusLeds::begin() {
    led_strip_config_t sc = {};
    sc.strip_gpio_num = gpio_;
    sc.max_leds = count_;
    led_strip_rmt_config_t rc = {};
    rc.resolution_hz = 10 * 1000 * 1000;
    led_strip_new_rmt_device(&sc, &rc, &strip_);
    led_strip_clear(strip_);
}
void StatusLeds::show(fw::BoxState state, bool estopped, bool linkAlive, uint32_t nowMs) {
    uint8_t r=0,g=0,b=0;
    if (estopped)            { bool on = (nowMs / 150) % 2; r = on ? 60 : 0; }     // fast red blink
    else if (state == fw::BoxState::ARMED) { g = 60; }                            // green
    else if (!linkAlive)     { r = 40; g = 20; }                                  // amber = link lost
    else                     { r = 60; }                                          // red = SAFE
    led_strip_set_pixel(strip_, 0, r, g, b);
    led_strip_refresh(strip_);
}
```

- [ ] **Step 3: Build-only compile-check (PowerShell)** — same as Task 2 Step 4 once these files are in `SRCS`. Confirm `led_strip`/`gpio` headers resolve. Reference `C:\esp\v6.0.1\esp-idf\examples\peripherals\rmt\led_strip` for the v6 led_strip API.

- [ ] **Step 4: Commit**

```bash
git add firmware/main/arm_switch.h firmware/main/arm_switch.cpp firmware/main/status_leds.h firmware/main/status_leds.cpp
git commit -m "feat(fw): arm-switch GPIO + addressable LED status (build-only)"
```

---

## Task 4: ESP-NOW link (RX→onCommand, ACK TX) + full app wiring — build-only

**Files:**
- Create: `firmware/main/espnow_link.h` / `espnow_link.cpp`
- Modify: `firmware/main/firing_box_main.cpp`

**Interfaces:**
- `class EspNowLink { esp_err_t begin(const uint8_t controllerMac[6]); void onPacket(std::function<void(const fw::CommandPacket&)>); esp_err_t sendAck(const fw::AckPacket&); uint32_t lastRxMs(); };` — wraps `esp_now_init`, registers the controller as a peer, and routes received `CommandPacket`s (length-checked) to the callback. `lastRxMs` lets the app drive the link-lost LED.

- [ ] **Step 1: EspNowLink**

`firmware/main/espnow_link.h`:
```cpp
#pragma once
#include "protocol.h"
#include "esp_err.h"
#include <functional>
#include <cstdint>
class EspNowLink {
public:
    esp_err_t begin(const uint8_t controllerMac[6]);
    void setOnCommand(std::function<void(const fw::CommandPacket&)> cb) { cb_ = cb; }
    esp_err_t sendAck(const fw::AckPacket& ack);
private:
    static void rxTrampoline(const esp_now_recv_info_t* info, const uint8_t* data, int len);
    std::function<void(const fw::CommandPacket&)> cb_;
    uint8_t ctrlMac_[6];
};
```
`firmware/main/espnow_link.cpp`:
```cpp
#include "espnow_link.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include <cstring>
static const char* TAG = "espnow";
static EspNowLink* g_self = nullptr;

esp_err_t EspNowLink::begin(const uint8_t controllerMac[6]) {
    g_self = this;
    memcpy(ctrlMac_, controllerMac, 6);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    if (esp_now_init() != ESP_OK) return ESP_FAIL;
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, controllerMac, 6);
    peer.channel = 1; peer.encrypt = false;
    esp_now_add_peer(&peer);
    return esp_now_register_recv_cb(&EspNowLink::rxTrampoline);
}

void EspNowLink::rxTrampoline(const esp_now_recv_info_t* /*info*/, const uint8_t* data, int len) {
    if (!g_self || !g_self->cb_) return;
    if (len < (int)sizeof(fw::CommandPacket)) return;     // length guard; CRC checked in onCommand
    fw::CommandPacket pkt;
    memcpy(&pkt, data, sizeof(pkt));
    g_self->cb_(pkt);
}

esp_err_t EspNowLink::sendAck(const fw::AckPacket& ack) {
    return esp_now_send(ctrlMac_, (const uint8_t*)&ack, sizeof(ack));
}
```
> Note: the RX callback runs in the WiFi task. Keep `onCommand` non-blocking (it is — no I/O except the I²C write, which is fast). If I²C latency becomes a concern, post the packet to a queue and process it in the control-loop task. Start simple; revisit if the WDT trips.

- [ ] **Step 2: Wire the full app**

Rewrite `firmware/main/firing_box_main.cpp` to construct the I²C bus, the real driver, arm switch, LEDs, and esp_now, then run the control loop:
```cpp
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/i2c_master.h"
#include "box_controller.h"
#include "board_config.h"
#include "expander_driver.h"
#include "arm_switch.h"
#include "status_leds.h"
#include "espnow_link.h"

static const char* TAG = "firing_box";

extern "C" void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());

    // I2C bus
    i2c_master_bus_config_t bc = {};
    bc.i2c_port = -1;                    // auto
    bc.sda_io_num = (gpio_num_t)board::I2C_SDA_GPIO;
    bc.scl_io_num = (gpio_num_t)board::I2C_SCL_GPIO;
    bc.clk_source = I2C_CLK_SRC_DEFAULT;
    bc.glitch_ignore_cnt = 7;
    i2c_master_bus_handle_t bus = nullptr;
    ESP_ERROR_CHECK(i2c_master_new_master_bus(&bc, &bus));

    static ExpanderChannelDriver driver(bus, board::EXPANDER_I2C_ADDR, board::FIRE_LEVEL);
    ESP_ERROR_CHECK(driver.begin());     // outputs OFF first

    fw::BoxConfig cfg; cfg.boxId = board::THIS_BOX_ID;
    static fw::BoxController box(driver, cfg);
    box.begin();                         // boot-safe: SAFE, all off

    static ArmSwitch armSwitch(board::ARM_SWITCH_GPIO); armSwitch.begin();
    static StatusLeds leds(board::STATUS_LED_GPIO, board::STATUS_LED_COUNT); leds.begin();

    static EspNowLink link;
    static volatile uint32_t lastRxMs = 0;
    link.setOnCommand([](const fw::CommandPacket& pkt) {
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
        lastRxMs = now;
        fw::CommandResult r = box.onCommand(pkt, now);
        // ACK both FIRED and DUPLICATE so a lost ACK self-heals on the controller's
        // same-id retry; do NOT ACK a REJECTED cue (genuinely not fired).
        if (r == fw::CommandResult::FIRED || r == fw::CommandResult::DUPLICATE) {
            fw::AckPacket ack{}; ack.type = (uint8_t)fw::MsgType::ACK;
            ack.responseToId = pkt.id;
            ack.deviceStatus = (r == fw::CommandResult::FIRED) ? 1 : 2; // 1=IGNITED_OK, 2=ALREADY_FIRED
            ack.timestamp = now; ack.crc = fw::computeCrc(ack);
            link.sendAck(ack);
        }
    });
    ESP_ERROR_CHECK(link.begin(board::CONTROLLER_MAC));

    ESP_LOGI(TAG, "firing box %u booted: SAFE, outputs off", cfg.boxId);

    // SAFETY: the box never calls setSequenceRunning(true) — heartbeat dead-man always active.
    while (true) {
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
        box.setPhysicalSwitch(armSwitch.isOn(), now);   // hardware-authoritative
        box.tick(now);
        bool linkAlive = (now - lastRxMs) < 2000;
        leds.show(box.state(), false /*estopped surfaced via state*/, linkAlive, now);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
```
> `cfg` is captured by the lambda — make it `static` (shown) so the capture is valid for the program lifetime.

- [ ] **Step 3: Full esp32 build — verify it links (PowerShell)**

In **PowerShell**:
```powershell
& 'C:\esp\v6.0.1\esp-idf\export.ps1'
Set-Location firmware
idf.py build 2>&1 | Select-Object -Last 20
```
Expected: `Project build complete.` + `firmware/build/firing_box.bin`. This links the real driver + arm switch + LEDs + esp_now + BoxController. Resolve any IDF v6.0.1 API mismatches against the on-disk examples (`esp_now`, `i2c_master`, `led_strip`) or via context7 (`/espressif/esp-idf`).

- [ ] **Step 4: Commit**

```bash
git add firmware/main/espnow_link.h firmware/main/espnow_link.cpp firmware/main/firing_box_main.cpp
git commit -m "feat(fw): esp-now RX->onCommand + ACK, full firing-box app links for esp32"
```

---

## Task 5: Flash + bench bring-up with LEDs (NO pyro)

> **Hardware required:** one ESP32 + the MCP23017/PCF8575 expander + **LEDs (not the SSR board, not igniters)** wired to the expander outputs through current-limiting resistors, so each "fire" lights an LED. The user runs the flash/monitor; the agent interprets the serial log + asks the user what the LEDs do.

**Files:** none (verification task).

- [ ] **Step 1: Flash + monitor (user runs in PowerShell)**

Tell the user to connect the ESP32, find its COM port (Device Manager or `idf.py -p` list), then run in PowerShell:
```powershell
& 'C:\esp\v6.0.1\esp-idf\export.ps1'
Set-Location firmware
idf.py -p COM<N> flash monitor
```
Expected serial: `firing box 0 booted: SAFE, outputs off`. Status LED red (SAFE). No output LED lit.

- [ ] **Step 2: Never-boot-armed check**

With the arm switch in the ON position at power-up, reset the board. Confirm: LED still shows SAFE at boot (the box must require the switch to be toggled / a fresh ARM). Confirm no output LED flickers during boot (validates the external pull resistors + all-off-first).

- [ ] **Step 3: Interlock checks (drive commands from a second ESP32 or the controller once Plan 4 exists; until then, a tiny throwaway sender sketch)**

Without the controller yet, verify the *local* interlocks that don't need ESP-NOW:
- Arm switch OFF → status red (SAFE). Flip ON → still SAFE until an ARM command arrives (so it stays SAFE here — correct).
- Provide a temporary `sender` ESP-IDF app (or reuse Plan 4's controller) that sends ARM (with switch ON) → status green; then FIRE ch0 → output LED 0 pulses ~400ms then off, ACK logged; FIRE while switch OFF → ignored; E-STOP → status red-blink + any lit output dies; stop heartbeats while idle → within ~2s status goes amber/SAFE.
- Record observed behavior against expected for each.

- [ ] **Step 4: Document results**

Append a short "bench bring-up log" to the report (what each LED did per command). No commit unless a code fix was needed (then commit the fix and re-run).

---

## Task 6: SSR polarity test + finalize expander/polarity (when boards arrive ~2026-06-24)

**Files:**
- Create: `docs/POLARITY_TEST.md`
- Modify (only if the test dictates): `firmware/main/board_config.h`

- [ ] **Step 1: Write the safe polarity procedure**

`docs/POLARITY_TEST.md`:
```markdown
# SSR polarity test (no igniters connected)

Goal: determine whether a channel fires on a HIGH or LOW input, so FIRE_LEVEL is correct.

1. Power ONLY the logic side of one 16-ch SSR board (VCC+GND to 3.3V/5V). Leave the
   output/high-voltage side EMPTY — no igniters, no mains.
2. Most boards have a per-channel indicator LED. With a jumper on channel 1's IN pin:
   - IN -> GND (LOW): note if the channel's active LED lights.
   - IN -> VCC (HIGH): note if it lights.
3. Whichever level lights the "active" LED is the fire level:
   - lights on LOW  -> active-LOW  -> FIRE_LEVEL = ACTIVE_LOW  (all-OFF word = 0xFFFF)
   - lights on HIGH -> active-HIGH -> FIRE_LEVEL = ACTIVE_HIGH (all-OFF word = 0x0000;
     use MCP23017 + external pull-DOWN resistors so a floating boot = OFF)
4. If no per-channel LED: use a multimeter on IN vs the opto/SSR, or ask for the meter procedure.
```

- [ ] **Step 2: Set the constants to match**

Edit `firmware/main/board_config.h`: set `board::FIRE_LEVEL` to the tested value, and confirm the expander choice (PCF8575 fine for active-LOW; if active-HIGH, standardize on MCP23017 + buy a second one + add pull-downs). Both boxes flashed identically.

- [ ] **Step 3: Re-run Task 5 bench checks with the real SSR board** (still NO igniters — confirm the SSR's own channel LEDs follow fire commands). Then, and only then, is the box ready for a supervised live test per the pre-show checklist.

- [ ] **Step 4: Commit**

```bash
git add docs/POLARITY_TEST.md firmware/main/board_config.h
git commit -m "docs(fw): SSR polarity test procedure + finalized FIRE_LEVEL/expander"
```

---

## Done criteria

- `ctest --test-dir build/test_codec` passes (codec bit-math).
- `idf.py build` (esp32) links the full firing-box app with real driver + esp_now + LEDs.
- On the bench (LEDs, no pyro): boots SAFE/outputs-off, never-boot-armed holds, arm→fire→ACK works, every interlock (switch, E-STOP, heartbeat dead-man, dup-id, out-of-range) behaves, channel pulses are ~400ms.
- Polarity confirmed and `board_config.h` finalized; both boxes flash identical firmware.

## Notes
- The second box is the same firmware with `THIS_BOX_ID = 1` and its expander/MAC. Keep them identical otherwise.
- A live (pyro) test is out of scope for this plan — gated behind a written pre-show checklist (future).
