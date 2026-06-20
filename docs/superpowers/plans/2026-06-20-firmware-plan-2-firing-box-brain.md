# Firmware Plan 2 — Firing-Box Brain + Buildable ESP-IDF Skeleton

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the firing box's decision logic as a portable, host-tested `BoxController` (wrapping the tested `fireworkcore`) behind a `ChannelDriver` hardware interface, and stand up an ESP-IDF firing-box app that compiles for the ESP32 target — so all the safety logic is verified off-target now and only the hardware drivers + flashing remain for Plan 3.

**Architecture:** `BoxController` owns an `ArmingStateMachine` + `RecentIds` dedup + per-channel pulse timers, and drives an abstract `ChannelDriver` (energize/de-energize). `onCommand(CommandPacket, nowMs)` dispatches ARM/DISARM/FIRE/HEARTBEAT/ESTOP through the interlocks; `tick(nowMs)` advances arming and expires bounded fire pulses. All time is injected, so it is host-tested with a `FakeChannelDriver` in the existing CTest harness. A thin ESP-IDF app (`firmware/`) implements `ChannelDriver` against the hardware later; this plan ships a build-only skeleton with a stub driver that compiles for esp32.

**Tech Stack:** C++11 (portable `BoxController` in `components/fireworkcore/`), CMake/Ninja/g++ + CTest (host tests, Git Bash), ESP-IDF v6.0.1 + `idf.py` (firmware build, **PowerShell only**).

## Global Constraints

- `BoxController` and `ChannelDriver` are plain C++11 in `components/fireworkcore/` — NO Arduino/ESP-IDF/FreeRTOS headers; must compile with host g++.
- Reuse `fireworkcore` unchanged: `ArmingStateMachine`, `RecentIds`, `protocol.h` (`CommandPacket`, `MsgType`, `crcValid`, `channelInRange`, `MAX_CHANNELS`).
- All time is injected `uint32_t nowMs`; no clock reads in portable code.
- `FIRE_MS` default 400 (bounded pulse cap); heartbeat timeout default 2000ms (ArmingConfig).
- The box does NOT run sequences — the controller sends individual `FIRE` cues and continuous `HEARTBEAT`s. The box uses the heartbeat dead-man (no `setSequenceRunning` on the box).
- Safety invariants the tests must lock: never boot armed; physical switch off ⇒ SAFE + all channels off; E-STOP ⇒ all channels off + latched; FIRE honored only when `canFire` AND channel in range AND message id unseen; a fired channel auto-de-energizes after `FIRE_MS`; any ARMED→SAFE transition forces all channels off.
- **Toolchain (verified 2026-06-20):** ESP-IDF does NOT work in Git Bash on Windows. Firmware build runs in **PowerShell**: `& 'C:\esp\v6.0.1\esp-idf\export.ps1'` then `idf.py set-target esp32` / `idf.py build`. Host `fireworkcore` tests stay in Git Bash + ctest.
- Firmware verification in this plan is **build-only** (`idf.py build` succeeds for esp32). Flashing + on-hardware bring-up + real expander/esp-now drivers are Plan 3 (needs an ESP32 plugged in; SSR boards arrive ~2026-06-24).
- `firmware/build/`, `firmware/sdkconfig`, `firmware/managed_components/` are git-ignored.

---

## File Structure

```
components/fireworkcore/include/channel_driver.h   NEW: abstract ChannelDriver interface
components/fireworkcore/include/box_controller.h    NEW: BoxController
components/fireworkcore/src/box_controller.cpp       NEW: BoxController impl
components/fireworkcore/host_test/fake_channel_driver.h  NEW: test double
components/fireworkcore/host_test/test_box_controller.cpp NEW: CTest
components/fireworkcore/host_test/CMakeLists.txt     MODIFY: add box_controller.cpp + test_box_controller
firmware/CMakeLists.txt                              NEW: IDF project (top-level)
firmware/sdkconfig.defaults                          NEW
firmware/.gitignore                                  NEW
firmware/main/CMakeLists.txt                         NEW: app component (depends on fireworkcore)
firmware/main/firing_box_main.cpp                    NEW: thin app_main with a stub driver (build-only)
firmware/main/stub_channel_driver.h                  NEW: no-op ChannelDriver so the skeleton links
```

The ESP-IDF project references the existing `components/fireworkcore/` via `EXTRA_COMPONENT_DIRS`, so the firmware links the SAME tested core + BoxController.

---

## Task 1: ChannelDriver interface + BoxController skeleton (boot-safe)

**Files:**
- Create: `components/fireworkcore/include/channel_driver.h`
- Create: `components/fireworkcore/include/box_controller.h`
- Create: `components/fireworkcore/src/box_controller.cpp`
- Create: `components/fireworkcore/host_test/fake_channel_driver.h`
- Create: `components/fireworkcore/host_test/test_box_controller.cpp`
- Modify: `components/fireworkcore/host_test/CMakeLists.txt`

**Interfaces:**
- Produces:
  - `class fw::ChannelDriver { virtual void allOff()=0; virtual void setChannel(uint8_t ch, bool on)=0; virtual ~ChannelDriver(){} };`
  - `struct fw::BoxConfig { uint8_t boxId=0; uint32_t fireMs=400; ArmingConfig arming{}; };`
  - `class fw::BoxController` with: `BoxController(ChannelDriver& drv, BoxConfig cfg=BoxConfig());`, `void begin();`, `void setPhysicalSwitch(bool on, uint32_t nowMs);`, `void onCommand(const CommandPacket& pkt, uint32_t nowMs);`, `void tick(uint32_t nowMs);`, `BoxState state() const;`, `bool canFire(uint32_t nowMs) const;`
  - `class FakeChannelDriver : public fw::ChannelDriver` (test double) recording `allOffCount`, `on[MAX_CHANNELS]` bool state.

- [ ] **Step 1: Write the failing test (boot-safe behavior)**

`components/fireworkcore/host_test/fake_channel_driver.h`:

```cpp
#pragma once
#include "channel_driver.h"
#include "protocol.h"

struct FakeChannelDriver : public fw::ChannelDriver {
    int allOffCount = 0;
    bool on[fw::MAX_CHANNELS] = {false};
    void allOff() override { allOffCount++; for (int i=0;i<fw::MAX_CHANNELS;i++) on[i]=false; }
    void setChannel(uint8_t ch, bool state) override { if (ch < fw::MAX_CHANNELS) on[ch]=state; }
    int countOn() const { int n=0; for (int i=0;i<fw::MAX_CHANNELS;i++) if(on[i]) n++; return n; }
};
```

`components/fireworkcore/host_test/test_box_controller.cpp`:

```cpp
#include "check.h"
#include "box_controller.h"
#include "fake_channel_driver.h"
using namespace fw;

void test_begin_is_safe_and_alloff() {
    FakeChannelDriver drv;
    BoxController box(drv, BoxConfig{});
    box.begin();
    CHECK_EQ((int)box.state(), (int)BoxState::SAFE);   // never boot armed
    CHECK(drv.allOffCount >= 1);                        // outputs forced off at boot
    CHECK_EQ(drv.countOn(), 0);
}

int main() {
    RUN(test_begin_is_safe_and_alloff);
    return REPORT();
}
```

- [ ] **Step 2: Register the test + source**

Edit `components/fireworkcore/host_test/CMakeLists.txt`:
1. Add `box_controller.cpp` to `CORE_SRC`:
```cmake
set(CORE_SRC
    ${CORE}/src/crc32.cpp
    ${CORE}/src/arming.cpp
    ${CORE}/src/sequence.cpp
    ${CORE}/src/box_controller.cpp
)
```
2. Append (this target links only `CORE_SRC` — it does NOT need the rig's `sim_bindings.cpp`):
```cmake
add_executable(test_box_controller test_box_controller.cpp ${CORE_SRC})
add_test(NAME test_box_controller COMMAND test_box_controller)
```
> Note: adding `box_controller.cpp` to `CORE_SRC` means every other test executable (test_crc32, test_arming, …) now links it too. That is harmless (unused object code), and `test_rig` already links `CORE_SRC` + `sim_bindings.cpp` so it keeps working unchanged.

- [ ] **Step 3: Build — verify FAIL**

Run (Git Bash):
```
cmake -S components/fireworkcore/host_test -B build/host_test -G Ninja -DCMAKE_CXX_COMPILER=g++
cmake --build build/host_test
```
Expected: FAIL — `channel_driver.h` / `box_controller.h` not found.

- [ ] **Step 4: Write the headers**

`components/fireworkcore/include/channel_driver.h`:
```cpp
#pragma once
#include <cstdint>
namespace fw {
class ChannelDriver {
public:
    virtual ~ChannelDriver() {}
    virtual void allOff() = 0;                       // force every channel de-energized
    virtual void setChannel(uint8_t channel, bool on) = 0;
};
} // namespace fw
```

`components/fireworkcore/include/box_controller.h`:
```cpp
#pragma once
#include <cstdint>
#include "arming.h"
#include "recent_ids.h"
#include "protocol.h"
#include "channel_driver.h"

namespace fw {

struct BoxConfig {
    uint8_t  boxId = 0;
    uint32_t fireMs = 400;
    ArmingConfig arming = ArmingConfig();
};

class BoxController {
public:
    BoxController(ChannelDriver& driver, BoxConfig cfg = BoxConfig());
    void begin();                                   // boot: all-off, SAFE
    void setPhysicalSwitch(bool on, uint32_t nowMs);
    void onCommand(const CommandPacket& pkt, uint32_t nowMs);
    void tick(uint32_t nowMs);
    BoxState state() const { return arm_.state(); }
    bool canFire(uint32_t nowMs) const { return arm_.canFire(nowMs); }

private:
    void energize(uint8_t ch, uint32_t nowMs);
    void deenergizeAll();

    ChannelDriver& drv_;
    BoxConfig cfg_;
    ArmingStateMachine arm_;
    RecentIds<32> seen_;
    bool     firing_[MAX_CHANNELS];
    uint32_t offAtMs_[MAX_CHANNELS];
    BoxState prevState_;
};

} // namespace fw
```

- [ ] **Step 5: Write the implementation**

`components/fireworkcore/src/box_controller.cpp`:
```cpp
#include "box_controller.h"

namespace fw {

BoxController::BoxController(ChannelDriver& driver, BoxConfig cfg)
    : drv_(driver), cfg_(cfg), arm_(cfg.arming), prevState_(BoxState::SAFE) {
    for (int i = 0; i < MAX_CHANNELS; i++) { firing_[i] = false; offAtMs_[i] = 0; }
}

void BoxController::deenergizeAll() {
    drv_.allOff();
    for (int i = 0; i < MAX_CHANNELS; i++) firing_[i] = false;
}

void BoxController::begin() {
    deenergizeAll();          // outputs off before anything else (boot-safe)
    prevState_ = arm_.state(); // SAFE (never boot armed)
}

void BoxController::energize(uint8_t ch, uint32_t nowMs) {
    drv_.setChannel(ch, true);
    firing_[ch] = true;
    offAtMs_[ch] = nowMs + cfg_.fireMs;
}

void BoxController::setPhysicalSwitch(bool on, uint32_t nowMs) {
    arm_.setPhysicalSwitch(on, nowMs);
}

void BoxController::onCommand(const CommandPacket& pkt, uint32_t nowMs) {
    if (!crcValid(pkt)) return;                     // corrupt packet ignored
    switch ((MsgType)pkt.type) {
        case MsgType::ARM:       arm_.arm(pkt.nonce, nowMs); break;
        case MsgType::DISARM:    arm_.disarm(nowMs); break;
        case MsgType::HEARTBEAT: arm_.heartbeat(nowMs); break;
        case MsgType::ESTOP:     arm_.estop(nowMs); deenergizeAll(); break;
        case MsgType::FIRE:
            if (pkt.boxId != cfg_.boxId) break;     // not addressed to this box
            if (!channelInRange(pkt.targetChannel)) break;
            if (seen_.seenOrRecord(pkt.id)) break;  // duplicate fire id
            if (!arm_.canFire(nowMs)) break;        // all interlocks
            energize(pkt.targetChannel, nowMs);
            break;
        default: break;                             // ACK or unknown — ignore on the box
    }
}

void BoxController::tick(uint32_t nowMs) {
    arm_.update(nowMs);
    // Any transition out of ARMED forces outputs off (firmware force-off contract).
    if (prevState_ == BoxState::ARMED && arm_.state() != BoxState::ARMED) deenergizeAll();
    prevState_ = arm_.state();
    // Expire bounded fire pulses.
    for (int i = 0; i < MAX_CHANNELS; i++) {
        if (firing_[i] && nowMs >= offAtMs_[i]) { drv_.setChannel((uint8_t)i, false); firing_[i] = false; }
    }
}

} // namespace fw
```

- [ ] **Step 6: Build + run — verify PASS**

Run:
```
cmake --build build/host_test
ctest --test-dir build/host_test -R test_box_controller --output-on-failure
```
Expected: `test_box_controller` PASSES (1 test). Also `ctest --test-dir build/host_test --output-on-failure` — all suites still pass.

- [ ] **Step 7: Commit**

```bash
git add components/fireworkcore/include/channel_driver.h components/fireworkcore/include/box_controller.h components/fireworkcore/src/box_controller.cpp components/fireworkcore/host_test/fake_channel_driver.h components/fireworkcore/host_test/test_box_controller.cpp components/fireworkcore/host_test/CMakeLists.txt
git commit -m "feat(fw): ChannelDriver interface + boot-safe BoxController skeleton"
```

---

## Task 2: BoxController FIRE handling + bounded pulse

**Files:**
- Modify: `components/fireworkcore/host_test/test_box_controller.cpp`

**Interfaces:** Consumes Task 1's `BoxController`/`FakeChannelDriver`.

Helper to build a valid packet (add near the top of the test file, after the includes):
```cpp
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
```

- [ ] **Step 1: Add the failing FIRE tests**

Add these functions and their `RUN(...)` lines in `main()`:

```cpp
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
```

- [ ] **Step 2: Build + run — verify PASS**

Run:
```
cmake --build build/host_test
ctest --test-dir build/host_test -R test_box_controller --output-on-failure
```
Expected: `test_box_controller` PASSES (7 tests). (No implementation change needed — Task 1's `onCommand` already implements this; if any test fails, fix `box_controller.cpp` minimally until green.)

- [ ] **Step 3: Commit**

```bash
git add components/fireworkcore/host_test/test_box_controller.cpp
git commit -m "test(fw): BoxController FIRE interlocks (armed/disarmed/box/range/dedup/crc)"
```

---

## Task 3: BoxController ARM/DISARM/E-STOP/switch/heartbeat behavior

**Files:**
- Modify: `components/fireworkcore/host_test/test_box_controller.cpp`

**Interfaces:** Consumes Task 1/2 helpers.

- [ ] **Step 1: Add the failing tests**

Add these functions + `RUN(...)` lines:

```cpp
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
```

- [ ] **Step 2: Build + run the full suite — verify PASS**

Run:
```
cmake --build build/host_test
ctest --test-dir build/host_test --output-on-failure
```
Expected: `test_box_controller` PASSES (12 tests); all 7 host suites pass. (If a test fails, fix `box_controller.cpp` minimally — likely the ARMED→SAFE force-off in `tick` or E-STOP path.)

- [ ] **Step 3: Commit**

```bash
git add components/fireworkcore/host_test/test_box_controller.cpp
git commit -m "test(fw): BoxController arm/switch/estop/heartbeat interlocks"
```

---

## Task 4: ESP-IDF firing-box app skeleton (builds for esp32)

**Files:**
- Create: `firmware/CMakeLists.txt`
- Create: `firmware/sdkconfig.defaults`
- Create: `firmware/.gitignore`
- Create: `firmware/main/CMakeLists.txt`
- Create: `firmware/main/stub_channel_driver.h`
- Create: `firmware/main/firing_box_main.cpp`

**Interfaces:** Consumes `components/fireworkcore/` (`BoxController`, `ChannelDriver`). Produces a flashable-later esp32 app that compiles now.

> **Build environment:** all `idf.py` commands run in **PowerShell** after `& 'C:\esp\v6.0.1\esp-idf\export.ps1'`. They do NOT work in Git Bash.

- [ ] **Step 1: Top-level IDF project file**

`firmware/CMakeLists.txt`:
```cmake
cmake_minimum_required(VERSION 3.16)
# Reuse the host-tested safety core as an IDF component.
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../components")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(firing_box)
```

- [ ] **Step 2: Defaults + gitignore**

`firmware/sdkconfig.defaults`:
```
CONFIG_ESP_TASK_WDT_INIT=y
CONFIG_COMPILER_CXX_EXCEPTIONS=n
```

`firmware/.gitignore`:
```
build/
sdkconfig
sdkconfig.old
managed_components/
dependencies.lock
```

- [ ] **Step 3: App component manifest**

`firmware/main/CMakeLists.txt`:
```cmake
idf_component_register(
    SRCS "firing_box_main.cpp"
    INCLUDE_DIRS "."
    REQUIRES fireworkcore
)
```

- [ ] **Step 4: Stub ChannelDriver (build-only; real I²C driver is Plan 3)**

`firmware/main/stub_channel_driver.h`:
```cpp
#pragma once
#include "channel_driver.h"
// Build-only placeholder so the skeleton links. Plan 3 replaces this with the
// MCP23017/PCF8575 I²C driver. Intentionally does nothing.
class StubChannelDriver : public fw::ChannelDriver {
public:
    void allOff() override {}
    void setChannel(uint8_t /*channel*/, bool /*on*/) override {}
};
```

- [ ] **Step 5: Thin app_main wiring the brain (no real peripherals yet)**

`firmware/main/firing_box_main.cpp`:
```cpp
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "box_controller.h"
#include "stub_channel_driver.h"

static const char* TAG = "firing_box";

extern "C" void app_main(void) {
    static StubChannelDriver driver;
    fw::BoxConfig cfg;            // boxId 0, fireMs 400, default arming
    static fw::BoxController box(driver, cfg);
    box.begin();                  // boot-safe: outputs off, SAFE

    ESP_LOGI(TAG, "firing box booted: SAFE, outputs off");

    // Skeleton loop. Plan 3 adds: arm-switch GPIO -> setPhysicalSwitch,
    // esp_now RX -> box.onCommand, led_strip status, and the real ChannelDriver.
    while (true) {
        uint32_t nowMs = (uint32_t)(esp_timer_get_time() / 1000);
        box.tick(nowMs);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
```

- [ ] **Step 6: Build for esp32 — verify it compiles (PowerShell)**

In **PowerShell** (from repo root):
```powershell
& 'C:\esp\v6.0.1\esp-idf\export.ps1'
Set-Location firmware
idf.py set-target esp32
idf.py build
```
Expected: ends with `Project build complete.` and writes `firmware/build/firing_box.bin`. (No board needed — this is a compile/link check that the firmware links the `fireworkcore` component + `BoxController`.)

- [ ] **Step 7: Confirm no build artifacts staged, then commit source only**

Run (Git Bash is fine for git):
```
git status -s firmware/
```
Expected: only the source/config files appear (no `build/`, `sdkconfig`, `managed_components/`).
```bash
git add firmware/CMakeLists.txt firmware/sdkconfig.defaults firmware/.gitignore firmware/main/CMakeLists.txt firmware/main/stub_channel_driver.h firmware/main/firing_box_main.cpp
git commit -m "feat(fw): ESP-IDF firing-box skeleton building for esp32 (stub driver)"
```

---

## Done criteria

- `ctest --test-dir build/host_test --output-on-failure` passes all 7 suites (incl. `test_box_controller`, 12 tests).
- `BoxController` encodes every firing-box safety invariant from the spec, host-verified with a fake driver.
- `idf.py build` (esp32) completes for `firmware/`, linking the real `fireworkcore` component — proving the brain compiles on-target.
- No firmware build artifacts committed.

## Next plan (Plan 3 — needs hardware)

- Real `ChannelDriver` over MCP23017/PCF8575 I²C (finalize after the polarity test).
- Arm-switch GPIO (`gpio_config` + pull) → `setPhysicalSwitch`; `led_strip` status; `esp_now` RX → `box.onCommand` + ACK TX; task-WDT.
- Flash to an ESP32, bench-test with LEDs (no pyro): arm→fire→ACK, every interlock, never-boot-armed, E-STOP, heartbeat dead-man.
- **Reuse note:** the sim rig (`sim_bindings.cpp`) and the firmware could later share `BoxController` for one source of truth per box — a DRY refactor to consider once both exist.
