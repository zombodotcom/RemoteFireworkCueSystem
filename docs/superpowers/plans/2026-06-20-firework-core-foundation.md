# Firework Safety Core + Shared Protocol — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build and unit-test the hardware-independent safety core of the firework system — shared packet protocol, CRC32, arming state machine, duplicate-ID dedup, and sequence scheduler — entirely on the host, with zero microcontroller hardware.

**Architecture:** All logic lives in a single PlatformIO library `lib/fireworkcore/` written in plain, dependency-free C++ (no Arduino headers). A `native` PlatformIO environment compiles and runs Unity tests on the host. The ESP8266/ESP32 firmware in later plans links this same library, so the safety logic is identical and tested off-target. Time is injected (`nowMs` passed in) so the state machine is deterministic and testable without a clock.

**Tech Stack:** C++11, PlatformIO, Unity test framework (`pio test -e native`). No Arduino dependencies in this library.

## Global Constraints

- Library code in `lib/fireworkcore/` MUST NOT include `<Arduino.h>` or any board header — it must compile on the host `native` env.
- All time is passed in as `uint32_t nowMs` (milliseconds); no code in this library calls a clock directly.
- `MAX_CHANNELS` per box = 16. Two boxes = 32 channels system-wide (box id distinguishes them).
- Fire pulse cap `FIRE_MS` and heartbeat timeout are configuration values, not magic numbers; defaults: `FIRE_MS = 400`, `heartbeatTimeoutMs = 2000`.
- CRC32: polynomial `0xEDB88320`, init `0xFFFFFFFF`, final XOR `0xFFFFFFFF` (matches the proven baseline; known vector CRC32("123456789") == `0xCBF43926`).
- Every task ends green (`pio test -e native` passes) and is committed.
- Never boot armed: a freshly constructed `ArmingStateMachine` is always `SAFE`.

---

## File Structure

- `platformio.ini` — add `[env:native]` for host tests; keep existing board envs.
- `lib/fireworkcore/crc32.h` / `crc32.cpp` — CRC32 routine (pure).
- `lib/fireworkcore/protocol.h` — message types, `CommandPacket`, `AckPacket`, constants, helpers to (re)compute CRC over a packet.
- `lib/fireworkcore/recent_ids.h` — fixed-size ring buffer for duplicate message-ID rejection.
- `lib/fireworkcore/arming.h` / `arming.cpp` — the box arming/safety state machine (the safety-critical heart).
- `lib/fireworkcore/sequence.h` / `sequence.cpp` — timeline scheduler that yields due steps given elapsed time.
- `test/test_<unit>/` — one Unity test folder per unit (each holds a single `.cpp` with its own `main()`; PlatformIO builds one runner per folder, so they must not share a folder).

---

## Task 1: Native test environment + CRC32

**Files:**
- Modify: `platformio.ini`
- Create: `lib/fireworkcore/crc32.h`
- Create: `lib/fireworkcore/crc32.cpp`
- Test: `test/test_crc32/test_crc32.cpp`

**Interfaces:**
- Consumes: nothing.
- Produces: `uint32_t fw::crc32(const uint8_t* data, size_t len);`

- [ ] **Step 1: Add the native test env to `platformio.ini`**

Append (keep existing `[env]`, `[env:esp8266_master]`, `[env:esp8266_slave]`):

```ini
[env:native]
platform = native
test_framework = unity
build_flags = -std=c++11
```

- [ ] **Step 2: Write the failing test**

`test/test_crc32/test_crc32.cpp`:

```cpp
#include <unity.h>
#include <cstring>
#include "crc32.h"

void test_crc32_known_vector() {
    const char* s = "123456789";
    uint32_t c = fw::crc32(reinterpret_cast<const uint8_t*>(s), 9);
    TEST_ASSERT_EQUAL_HEX32(0xCBF43926u, c);
}

void test_crc32_empty_is_zero() {
    TEST_ASSERT_EQUAL_HEX32(0x00000000u, fw::crc32(nullptr, 0));
}

void setUp() {}
void tearDown() {}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_crc32_known_vector);
    RUN_TEST(test_crc32_empty_is_zero);
    return UNITY_END();
}
```

- [ ] **Step 3: Run the test to verify it fails**

Run: `pio test -e native`
Expected: FAIL — `crc32.h` not found / undefined reference.

- [ ] **Step 4: Write the implementation**

`lib/fireworkcore/crc32.h`:

```cpp
#pragma once
#include <cstdint>
#include <cstddef>
namespace fw { uint32_t crc32(const uint8_t* data, size_t len); }
```

`lib/fireworkcore/crc32.cpp`:

```cpp
#include "crc32.h"
namespace fw {
uint32_t crc32(const uint8_t* data, size_t len) {
    uint32_t crc = 0xFFFFFFFFu;
    while (len--) {
        uint8_t b = *data++;
        for (int i = 0; i < 8; i++) {
            uint32_t mask = -(int32_t)((crc ^ b) & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
            b >>= 1;
        }
    }
    return ~crc;
}
} // namespace fw
```

- [ ] **Step 5: Run the test to verify it passes**

Run: `pio test -e native`
Expected: PASS (2 tests).

- [ ] **Step 6: Commit**

```bash
git add platformio.ini lib/fireworkcore/crc32.h lib/fireworkcore/crc32.cpp test/test_crc32/test_crc32.cpp
git commit -m "feat(core): native test env + CRC32 with known-vector test"
```

---

## Task 2: Shared protocol header

**Files:**
- Create: `lib/fireworkcore/protocol.h`
- Test: `test/test_protocol/test_protocol.cpp`

**Interfaces:**
- Consumes: `fw::crc32`.
- Produces:
  - `enum class fw::MsgType : uint8_t { FIRE=1, ACK=2, ARM=3, DISARM=4, HEARTBEAT=5, ESTOP=6 };`
  - `struct fw::CommandPacket { uint8_t type; char cmd[8]; uint32_t id; uint8_t boxId; uint8_t targetChannel; uint32_t nonce; uint32_t crc; };`
  - `struct fw::AckPacket { uint8_t type; uint32_t responseToId; char note[16]; uint8_t deviceStatus; uint32_t timestamp; uint32_t crc; };`
  - `uint32_t fw::computeCrc(const CommandPacket&);`
  - `uint32_t fw::computeCrc(const AckPacket&);`
  - `bool fw::crcValid(const CommandPacket&);`
  - `bool fw::crcValid(const AckPacket&);`

- [ ] **Step 1: Write the failing test**

`test/test_protocol/test_protocol.cpp`:

```cpp
#include <unity.h>
#include <cstring>
#include "protocol.h"

void test_command_crc_roundtrip() {
    fw::CommandPacket p{};
    p.type = (uint8_t)fw::MsgType::FIRE;
    std::strncpy(p.cmd, "FIRE", sizeof(p.cmd));
    p.id = 42; p.boxId = 1; p.targetChannel = 7; p.nonce = 0;
    p.crc = fw::computeCrc(p);
    TEST_ASSERT_TRUE(fw::crcValid(p));
}

void test_command_crc_detects_tamper() {
    fw::CommandPacket p{};
    p.type = (uint8_t)fw::MsgType::FIRE;
    p.id = 42; p.boxId = 1; p.targetChannel = 7;
    p.crc = fw::computeCrc(p);
    p.targetChannel = 8;            // tamper after CRC
    TEST_ASSERT_FALSE(fw::crcValid(p));
}

void setUp() {}
void tearDown() {}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_command_crc_roundtrip);
    RUN_TEST(test_command_crc_detects_tamper);
    return UNITY_END();
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `pio test -e native -f test_protocol`
Expected: FAIL — `protocol.h` not found.

- [ ] **Step 3: Write the implementation**

`lib/fireworkcore/protocol.h`:

```cpp
#pragma once
#include <cstdint>
#include <cstddef>
#include "crc32.h"

namespace fw {

static const uint8_t MAX_CHANNELS = 16;

enum class MsgType : uint8_t { FIRE=1, ACK=2, ARM=3, DISARM=4, HEARTBEAT=5, ESTOP=6 };

#pragma pack(push, 1)
struct CommandPacket {
    uint8_t  type;
    char     cmd[8];
    uint32_t id;
    uint8_t  boxId;
    uint8_t  targetChannel;
    uint32_t nonce;
    uint32_t crc;
};
struct AckPacket {
    uint8_t  type;
    uint32_t responseToId;
    char     note[16];
    uint8_t  deviceStatus;
    uint32_t timestamp;
    uint32_t crc;
};
#pragma pack(pop)

inline uint32_t computeCrc(const CommandPacket& p) {
    return crc32(reinterpret_cast<const uint8_t*>(&p), sizeof(p) - sizeof(p.crc));
}
inline uint32_t computeCrc(const AckPacket& p) {
    return crc32(reinterpret_cast<const uint8_t*>(&p), sizeof(p) - sizeof(p.crc));
}
inline bool crcValid(const CommandPacket& p) { return p.crc == computeCrc(p); }
inline bool crcValid(const AckPacket& p)     { return p.crc == computeCrc(p); }

} // namespace fw
```

- [ ] **Step 4: Run to verify it passes**

Run: `pio test -e native -f test_protocol`
Expected: PASS (2 tests).

- [ ] **Step 5: Commit**

```bash
git add lib/fireworkcore/protocol.h test/test_protocol/test_protocol.cpp
git commit -m "feat(core): shared packet protocol with CRC helpers"
```

---

## Task 3: Duplicate-ID ring buffer

**Files:**
- Create: `lib/fireworkcore/recent_ids.h`
- Test: `test/test_recent_ids/test_recent_ids.cpp`

**Interfaces:**
- Consumes: nothing.
- Produces: `class fw::RecentIds<N>` with `bool seenOrRecord(uint32_t id)` — returns `true` if `id` was already recorded (duplicate), otherwise records it and returns `false`.

- [ ] **Step 1: Write the failing test**

`test/test_recent_ids/test_recent_ids.cpp`:

```cpp
#include <unity.h>
#include "recent_ids.h"

void test_new_id_is_not_duplicate() {
    fw::RecentIds<4> r;
    TEST_ASSERT_FALSE(r.seenOrRecord(10));
}

void test_repeat_id_is_duplicate() {
    fw::RecentIds<4> r;
    r.seenOrRecord(10);
    TEST_ASSERT_TRUE(r.seenOrRecord(10));
}

void test_evicted_id_seen_again_as_new() {
    fw::RecentIds<2> r;           // capacity 2
    r.seenOrRecord(1);
    r.seenOrRecord(2);
    r.seenOrRecord(3);            // evicts 1
    TEST_ASSERT_FALSE(r.seenOrRecord(1)); // 1 no longer remembered
}

void setUp() {}
void tearDown() {}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_new_id_is_not_duplicate);
    RUN_TEST(test_repeat_id_is_duplicate);
    RUN_TEST(test_evicted_id_seen_again_as_new);
    return UNITY_END();
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `pio test -e native -f test_recent_ids`
Expected: FAIL — `recent_ids.h` not found.

- [ ] **Step 3: Write the implementation**

`lib/fireworkcore/recent_ids.h`:

```cpp
#pragma once
#include <cstdint>
#include <cstddef>

namespace fw {

template <size_t N>
class RecentIds {
public:
    RecentIds() : head_(0), count_(0) {
        for (size_t i = 0; i < N; i++) buf_[i] = 0;
    }
    // true if id already present (duplicate); else record and return false.
    bool seenOrRecord(uint32_t id) {
        for (size_t i = 0; i < count_; i++) {
            if (buf_[i] == id) return true;
        }
        buf_[head_] = id;
        head_ = (head_ + 1) % N;
        if (count_ < N) count_++;
        return false;
    }
private:
    uint32_t buf_[N];
    size_t head_;
    size_t count_;
};

} // namespace fw
```

- [ ] **Step 4: Run to verify it passes**

Run: `pio test -e native -f test_recent_ids`
Expected: PASS (3 tests).

- [ ] **Step 5: Commit**

```bash
git add lib/fireworkcore/recent_ids.h test/test_recent_ids/test_recent_ids.cpp
git commit -m "feat(core): recent-id ring buffer for fire dedup"
```

---

## Task 4: Arming state machine (safety-critical core)

**Files:**
- Create: `lib/fireworkcore/arming.h`
- Create: `lib/fireworkcore/arming.cpp`
- Test: `test/test_arming/test_arming.cpp`

**Interfaces:**
- Consumes: nothing.
- Produces:
  - `enum class fw::BoxState : uint8_t { SAFE, ARMED };`
  - `struct fw::ArmingConfig { uint32_t heartbeatTimeoutMs = 2000; };`
  - `class fw::ArmingStateMachine` with:
    - `explicit ArmingStateMachine(ArmingConfig cfg = {});`
    - `void setPhysicalSwitch(bool on, uint32_t nowMs);`
    - `bool arm(uint32_t nonce, uint32_t nowMs);` — returns true iff it transitioned to ARMED.
    - `void disarm(uint32_t nowMs);`
    - `void estop(uint32_t nowMs);`
    - `void clearEstop(uint32_t nowMs);`
    - `void heartbeat(uint32_t nowMs);`
    - `void setSequenceRunning(bool running);`
    - `void update(uint32_t nowMs);`
    - `BoxState state() const;`
    - `bool canFire(uint32_t nowMs) const;`

**Behavior contract (these become the tests):**
1. Construction → `SAFE` (never boot armed).
2. `arm()` succeeds only when physical switch is ON and not E-STOPped; otherwise returns false and stays SAFE.
3. Re-using the exact previous accepted nonce is rejected (replay protection).
4. Physical switch OFF at any time → immediately SAFE.
5. `estop()` → SAFE and latches; `arm()` fails until `clearEstop()`.
6. While ARMED and **not** sequence-running: if `now - lastHeartbeat > timeout`, `update()` drops to SAFE.
7. While ARMED and sequence-running: heartbeat timeout does NOT disarm.
8. `canFire()` is true only when ARMED, switch ON, not E-STOPped, and (heartbeat fresh OR sequence-running).

- [ ] **Step 1: Write the failing tests**

`test/test_arming/test_arming.cpp`:

```cpp
#include <unity.h>
#include "arming.h"
using namespace fw;

static ArmingStateMachine armed_at(uint32_t now) {
    ArmingStateMachine m;            // default 2000ms timeout
    m.setPhysicalSwitch(true, now);
    bool ok = m.arm(1, now);
    TEST_ASSERT_TRUE(ok);
    return m;
}

void test_constructs_safe() {
    ArmingStateMachine m;
    TEST_ASSERT_EQUAL(int(BoxState::SAFE), int(m.state()));
}

void test_cannot_arm_with_switch_off() {
    ArmingStateMachine m;
    m.setPhysicalSwitch(false, 0);
    TEST_ASSERT_FALSE(m.arm(1, 0));
    TEST_ASSERT_EQUAL(int(BoxState::SAFE), int(m.state()));
}

void test_arms_with_switch_on() {
    ArmingStateMachine m;
    m.setPhysicalSwitch(true, 0);
    TEST_ASSERT_TRUE(m.arm(1, 0));
    TEST_ASSERT_EQUAL(int(BoxState::ARMED), int(m.state()));
}

void test_nonce_replay_rejected() {
    ArmingStateMachine m;
    m.setPhysicalSwitch(true, 0);
    TEST_ASSERT_TRUE(m.arm(7, 0));
    m.disarm(0);
    TEST_ASSERT_FALSE(m.arm(7, 0));    // same nonce reused
    TEST_ASSERT_TRUE(m.arm(8, 0));     // new nonce ok
}

void test_switch_off_disarms() {
    ArmingStateMachine m = armed_at(0);
    m.setPhysicalSwitch(false, 100);
    TEST_ASSERT_EQUAL(int(BoxState::SAFE), int(m.state()));
}

void test_estop_latches_until_cleared() {
    ArmingStateMachine m = armed_at(0);
    m.estop(50);
    TEST_ASSERT_EQUAL(int(BoxState::SAFE), int(m.state()));
    TEST_ASSERT_FALSE(m.arm(2, 60));   // still latched
    m.clearEstop(70);
    TEST_ASSERT_TRUE(m.arm(3, 80));
}

void test_heartbeat_timeout_disarms_when_idle() {
    ArmingStateMachine m = armed_at(0);
    m.update(2001);                    // > 2000ms since last heartbeat
    TEST_ASSERT_EQUAL(int(BoxState::SAFE), int(m.state()));
}

void test_heartbeat_keeps_armed() {
    ArmingStateMachine m = armed_at(0);
    m.heartbeat(1500);
    m.update(2001);                    // only 501ms since heartbeat
    TEST_ASSERT_EQUAL(int(BoxState::ARMED), int(m.state()));
}

void test_sequence_running_ignores_heartbeat_timeout() {
    ArmingStateMachine m = armed_at(0);
    m.setSequenceRunning(true);
    m.update(99999);
    TEST_ASSERT_EQUAL(int(BoxState::ARMED), int(m.state()));
}

void test_canfire_requires_all_gates() {
    ArmingStateMachine m = armed_at(0);
    TEST_ASSERT_TRUE(m.canFire(100));
    m.estop(120);
    TEST_ASSERT_FALSE(m.canFire(130));
}

void setUp() {}
void tearDown() {}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_constructs_safe);
    RUN_TEST(test_cannot_arm_with_switch_off);
    RUN_TEST(test_arms_with_switch_on);
    RUN_TEST(test_nonce_replay_rejected);
    RUN_TEST(test_switch_off_disarms);
    RUN_TEST(test_estop_latches_until_cleared);
    RUN_TEST(test_heartbeat_timeout_disarms_when_idle);
    RUN_TEST(test_heartbeat_keeps_armed);
    RUN_TEST(test_sequence_running_ignores_heartbeat_timeout);
    RUN_TEST(test_canfire_requires_all_gates);
    return UNITY_END();
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `pio test -e native -f test_arming`
Expected: FAIL — `arming.h` not found.

- [ ] **Step 3: Write the header**

`lib/fireworkcore/arming.h`:

```cpp
#pragma once
#include <cstdint>

namespace fw {

enum class BoxState : uint8_t { SAFE, ARMED };

struct ArmingConfig {
    uint32_t heartbeatTimeoutMs = 2000;
};

class ArmingStateMachine {
public:
    explicit ArmingStateMachine(ArmingConfig cfg = ArmingConfig());

    void setPhysicalSwitch(bool on, uint32_t nowMs);
    bool arm(uint32_t nonce, uint32_t nowMs);
    void disarm(uint32_t nowMs);
    void estop(uint32_t nowMs);
    void clearEstop(uint32_t nowMs);
    void heartbeat(uint32_t nowMs);
    void setSequenceRunning(bool running);
    void update(uint32_t nowMs);

    BoxState state() const { return state_; }
    bool canFire(uint32_t nowMs) const;

private:
    bool heartbeatFresh(uint32_t nowMs) const;
    void goSafe();

    ArmingConfig cfg_;
    BoxState state_;
    bool switchOn_;
    bool estopped_;
    bool sequenceRunning_;
    bool haveNonce_;
    uint32_t lastNonce_;
    uint32_t lastHeartbeatMs_;
};

} // namespace fw
```

- [ ] **Step 4: Write the implementation**

`lib/fireworkcore/arming.cpp`:

```cpp
#include "arming.h"

namespace fw {

ArmingStateMachine::ArmingStateMachine(ArmingConfig cfg)
    : cfg_(cfg), state_(BoxState::SAFE), switchOn_(false),
      estopped_(false), sequenceRunning_(false), haveNonce_(false),
      lastNonce_(0), lastHeartbeatMs_(0) {}

void ArmingStateMachine::goSafe() { state_ = BoxState::SAFE; }

void ArmingStateMachine::setPhysicalSwitch(bool on, uint32_t /*nowMs*/) {
    switchOn_ = on;
    if (!on) goSafe();
}

bool ArmingStateMachine::arm(uint32_t nonce, uint32_t nowMs) {
    if (!switchOn_ || estopped_) return false;
    if (haveNonce_ && nonce == lastNonce_) return false; // replay
    lastNonce_ = nonce;
    haveNonce_ = true;
    lastHeartbeatMs_ = nowMs;
    state_ = BoxState::ARMED;
    return true;
}

void ArmingStateMachine::disarm(uint32_t /*nowMs*/) { goSafe(); }

void ArmingStateMachine::estop(uint32_t /*nowMs*/) {
    estopped_ = true;
    goSafe();
}

void ArmingStateMachine::clearEstop(uint32_t /*nowMs*/) { estopped_ = false; }

void ArmingStateMachine::heartbeat(uint32_t nowMs) { lastHeartbeatMs_ = nowMs; }

void ArmingStateMachine::setSequenceRunning(bool running) { sequenceRunning_ = running; }

bool ArmingStateMachine::heartbeatFresh(uint32_t nowMs) const {
    return (nowMs - lastHeartbeatMs_) <= cfg_.heartbeatTimeoutMs;
}

void ArmingStateMachine::update(uint32_t nowMs) {
    if (state_ != BoxState::ARMED) return;
    if (!switchOn_ || estopped_) { goSafe(); return; }
    if (!sequenceRunning_ && !heartbeatFresh(nowMs)) goSafe();
}

bool ArmingStateMachine::canFire(uint32_t nowMs) const {
    if (state_ != BoxState::ARMED) return false;
    if (!switchOn_ || estopped_) return false;
    if (sequenceRunning_) return true;
    return heartbeatFresh(nowMs);
}

} // namespace fw
```

- [ ] **Step 5: Run to verify it passes**

Run: `pio test -e native -f test_arming`
Expected: PASS (10 tests).

- [ ] **Step 6: Commit**

```bash
git add lib/fireworkcore/arming.h lib/fireworkcore/arming.cpp test/test_arming/test_arming.cpp
git commit -m "feat(core): arming state machine with full interlock tests"
```

---

## Task 5: Sequence scheduler

**Files:**
- Create: `lib/fireworkcore/sequence.h`
- Create: `lib/fireworkcore/sequence.cpp`
- Test: `test/test_sequence/test_sequence.cpp`

**Interfaces:**
- Consumes: nothing.
- Produces:
  - `struct fw::SeqStep { uint32_t timeMs; uint8_t boxId; uint8_t channel; };`
  - `class fw::SequenceScheduler` with:
    - `void load(const SeqStep* steps, size_t count);`
    - `void start(uint32_t nowMs);`
    - `void stop();`
    - `bool running() const;`
    - `size_t due(uint32_t nowMs, SeqStep* out, size_t outCap);` — copies steps whose `timeMs <= elapsed` and not yet fired into `out` (up to `outCap`), marks them fired, returns how many. After the last step fires, `running()` becomes false on the next `due()` call.

**Behavior contract:**
1. Before `start()`, `due()` returns 0 and `running()` is false.
2. Steps fire once, in order, when elapsed (`nowMs - startMs`) reaches their `timeMs`.
3. A step is never returned twice.
4. `stop()` halts; subsequent `due()` returns 0.
5. After all steps have fired, `running()` returns false.

- [ ] **Step 1: Write the failing tests**

`test/test_sequence/test_sequence.cpp`:

```cpp
#include <unity.h>
#include "sequence.h"
using namespace fw;

static SeqStep STEPS[] = {
    {0,    0, 1},
    {1000, 0, 2},
    {1500, 1, 5},
};

void test_not_running_before_start() {
    SequenceScheduler s;
    s.load(STEPS, 3);
    SeqStep out[8];
    TEST_ASSERT_FALSE(s.running());
    TEST_ASSERT_EQUAL(0, s.due(0, out, 8));
}

void test_fires_due_steps_in_order() {
    SequenceScheduler s;
    s.load(STEPS, 3);
    s.start(10000);                 // start offset
    SeqStep out[8];
    size_t n = s.due(10000, out, 8); // elapsed 0 -> step @0
    TEST_ASSERT_EQUAL(1, n);
    TEST_ASSERT_EQUAL(1, out[0].channel);
    TEST_ASSERT_EQUAL(0, s.due(10500, out, 8)); // nothing new yet
    n = s.due(11600, out, 8);        // elapsed 1600 -> steps @1000 and @1500
    TEST_ASSERT_EQUAL(2, n);
    TEST_ASSERT_EQUAL(2, out[0].channel);
    TEST_ASSERT_EQUAL(5, out[1].channel);
}

void test_steps_fire_once() {
    SequenceScheduler s;
    s.load(STEPS, 3);
    s.start(0);
    SeqStep out[8];
    s.due(2000, out, 8);             // all three become due
    TEST_ASSERT_EQUAL(0, s.due(3000, out, 8)); // none repeat
}

void test_stop_halts() {
    SequenceScheduler s;
    s.load(STEPS, 3);
    s.start(0);
    s.stop();
    SeqStep out[8];
    TEST_ASSERT_EQUAL(0, s.due(2000, out, 8));
    TEST_ASSERT_FALSE(s.running());
}

void test_running_false_after_all_fired() {
    SequenceScheduler s;
    s.load(STEPS, 3);
    s.start(0);
    SeqStep out[8];
    s.due(2000, out, 8);
    s.due(2001, out, 8);             // triggers completion check
    TEST_ASSERT_FALSE(s.running());
}

void setUp() {}
void tearDown() {}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_not_running_before_start);
    RUN_TEST(test_fires_due_steps_in_order);
    RUN_TEST(test_steps_fire_once);
    RUN_TEST(test_stop_halts);
    RUN_TEST(test_running_false_after_all_fired);
    return UNITY_END();
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `pio test -e native -f test_sequence`
Expected: FAIL — `sequence.h` not found.

- [ ] **Step 3: Write the header**

`lib/fireworkcore/sequence.h`:

```cpp
#pragma once
#include <cstdint>
#include <cstddef>

namespace fw {

static const size_t MAX_SEQ_STEPS = 64;

struct SeqStep {
    uint32_t timeMs;
    uint8_t  boxId;
    uint8_t  channel;
};

class SequenceScheduler {
public:
    SequenceScheduler();
    void load(const SeqStep* steps, size_t count);
    void start(uint32_t nowMs);
    void stop();
    bool running() const { return running_; }
    size_t due(uint32_t nowMs, SeqStep* out, size_t outCap);

private:
    SeqStep steps_[MAX_SEQ_STEPS];
    bool    fired_[MAX_SEQ_STEPS];
    size_t  count_;
    uint32_t startMs_;
    bool     running_;
};

} // namespace fw
```

- [ ] **Step 4: Write the implementation**

`lib/fireworkcore/sequence.cpp`:

```cpp
#include "sequence.h"

namespace fw {

SequenceScheduler::SequenceScheduler() : count_(0), startMs_(0), running_(false) {}

void SequenceScheduler::load(const SeqStep* steps, size_t count) {
    if (count > MAX_SEQ_STEPS) count = MAX_SEQ_STEPS;
    count_ = count;
    for (size_t i = 0; i < count_; i++) { steps_[i] = steps[i]; fired_[i] = false; }
    running_ = false;
}

void SequenceScheduler::start(uint32_t nowMs) {
    startMs_ = nowMs;
    running_ = (count_ > 0);
    for (size_t i = 0; i < count_; i++) fired_[i] = false;
}

void SequenceScheduler::stop() { running_ = false; }

size_t SequenceScheduler::due(uint32_t nowMs, SeqStep* out, size_t outCap) {
    if (!running_) return 0;
    uint32_t elapsed = nowMs - startMs_;
    size_t n = 0;
    size_t remaining = 0;
    for (size_t i = 0; i < count_; i++) {
        if (fired_[i]) continue;
        if (steps_[i].timeMs <= elapsed) {
            if (n < outCap) { out[n++] = steps_[i]; fired_[i] = true; }
            else remaining++;          // due but no room this call
        } else {
            remaining++;
        }
    }
    if (remaining == 0) running_ = false;
    return n;
}

} // namespace fw
```

- [ ] **Step 5: Run to verify it passes**

Run: `pio test -e native -f test_sequence`
Expected: PASS (5 tests).

- [ ] **Step 6: Run the full suite**

Run: `pio test -e native`
Expected: PASS (all tests across crc32, protocol, recent_ids, arming, sequence).

- [ ] **Step 7: Commit**

```bash
git add lib/fireworkcore/sequence.h lib/fireworkcore/sequence.cpp test/test_sequence/test_sequence.cpp
git commit -m "feat(core): sequence scheduler with timeline tests"
```

---

## Done criteria

- `pio test -e native` passes all five suites.
- `lib/fireworkcore/` contains only host-compilable, dependency-free C++.
- Existing `esp8266_master` / `esp8266_slave` build envs are untouched and still present.
- The arming state machine encodes every safety gate from the spec §5, verified by tests.

## Next plans (not in this document)

- **Plan 2 — Firing-box firmware:** ESP8266 wraps `fireworkcore`, adds the expander channel-driver abstraction (`EXPANDER_TYPE`/`FIRE_LEVEL`), non-blocking fire pulse, NeoPixel status, ESP-NOW receive. Bench-tested with LEDs (no pyro).
- **Plan 3 — Controller firmware:** ESP32 SoftAP + web server + ESP-NOW + heartbeat + sequence engine driving both boxes.
- **Plan 4 — Web UI (PWA):** arm/disarm, E-STOP, manual cues, groups, sequence builder/runner.
