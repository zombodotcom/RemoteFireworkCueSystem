# Firework Safety Core + Shared Protocol — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build and unit-test the hardware-independent safety core of the firework system — shared packet protocol, CRC32, arming state machine, duplicate-ID dedup, and sequence scheduler — entirely on the host, with zero microcontroller hardware.

**Architecture:** The logic lives in an ESP-IDF **component** `components/fireworkcore/`, written in plain, dependency-free C++ (no IDF or Arduino headers). The same component is later linked by the ESP32 firmware (controller and box roles) in Plans 2–4, so the safety logic is identical and tested off-target. A separate, IDF-independent **host-test build** (`components/fireworkcore/host_test/`) compiles the pure sources with the host g++ and runs them under CTest. Time is injected (`nowMs` passed in) so the state machine is deterministic and testable without a clock.

**Tech Stack:** C++11, ESP-IDF v6.0 (`C:\esp\v6.0.1\esp-idf`) for the component registration; **host tests** use MinGW UCRT **g++ 15.2**, **CMake**, **Ninja**, **CTest** — no PlatformIO, no IDF invocation, no board.

## Global Constraints

- Source in `components/fireworkcore/` MUST NOT include `<Arduino.h>`, `<esp_*.h>`, `freertos/*`, or any board/framework header — it must compile on the host with plain g++.
- All time is passed in as `uint32_t nowMs` (milliseconds); no code in this component reads a clock.
- `MAX_CHANNELS` per box = 16. Two boxes = 32 channels system-wide (`boxId` distinguishes them).
- Fire pulse cap `FIRE_MS` and heartbeat timeout are configuration values, not magic numbers; defaults: `FIRE_MS = 400`, `heartbeatTimeoutMs = 2000`.
- CRC32: polynomial `0xEDB88320`, init `0xFFFFFFFF`, final XOR `0xFFFFFFFF` (matches the proven baseline; known vector CRC32("123456789") == `0xCBF43926`).
- Every task ends green (CTest passes) and is committed.
- Never boot armed: a freshly constructed `ArmingStateMachine` is always `SAFE`.
- Host build/run commands (run from repo root, forward slashes OK in Git Bash/PowerShell):
  - Configure: `cmake -S components/fireworkcore/host_test -B build/host_test -G Ninja -DCMAKE_CXX_COMPILER=g++`
  - Build: `cmake --build build/host_test`
  - Run all: `ctest --test-dir build/host_test --output-on-failure`
  - Run one: `ctest --test-dir build/host_test -R <name> --output-on-failure`

---

## File Structure

```
components/fireworkcore/
  include/        crc32.h  protocol.h  recent_ids.h  arming.h  sequence.h
  src/            crc32.cpp  arming.cpp  sequence.cpp
  CMakeLists.txt  (IDF component registration — used by firmware in later plans)
  host_test/
    check.h           tiny assert harness (no deps)
    CMakeLists.txt    host build; one test executable per unit, registered with CTest
    test_crc32.cpp  test_protocol.cpp  test_recent_ids.cpp  test_arming.cpp  test_sequence.cpp
build/              (git-ignored) host-test build output
```

Each unit is a separate test executable so failures are isolated. Header-only units (`protocol.h`, `recent_ids.h`) have no `.cpp`; the three sources (`crc32`, `arming`, `sequence`) are linked into every test executable, which is harmless.

---

## Task 1: Host-test harness + component skeleton + CRC32

**Files:**
- Create: `components/fireworkcore/include/crc32.h`
- Create: `components/fireworkcore/src/crc32.cpp`
- Create: `components/fireworkcore/CMakeLists.txt`
- Create: `components/fireworkcore/host_test/check.h`
- Create: `components/fireworkcore/host_test/CMakeLists.txt`
- Create: `components/fireworkcore/host_test/test_crc32.cpp`
- Modify: `.gitignore` (ignore `build/`)

**Interfaces:**
- Consumes: nothing.
- Produces: `uint32_t fw::crc32(const uint8_t* data, size_t len);`

- [ ] **Step 1: Ignore the build dir**

Append a line to `.gitignore`:

```
build/
```

- [ ] **Step 2: Create the assert harness**

`components/fireworkcore/host_test/check.h`:

```cpp
#pragma once
#include <cstdio>
#include <cstdint>

static int g_failures = 0;

#define CHECK(cond) \
    do { if (!(cond)) { std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); ++g_failures; } } while (0)

#define CHECK_EQ(a, b) \
    do { long long _a=(long long)(a), _b=(long long)(b); \
         if (_a != _b) { std::printf("FAIL %s:%d: %s(%lld) == %s(%lld)\n", __FILE__, __LINE__, #a,_a,#b,_b); ++g_failures; } } while (0)

#define RUN(test) do { std::printf("RUN  %s\n", #test); test(); } while (0)

#define REPORT() ( g_failures == 0 ? (std::printf("OK\n"), 0) : (std::printf("%d FAILURE(S)\n", g_failures), 1) )
```

- [ ] **Step 3: Write the failing test**

`components/fireworkcore/host_test/test_crc32.cpp`:

```cpp
#include "check.h"
#include "crc32.h"

void test_crc32_known_vector() {
    const char* s = "123456789";
    CHECK_EQ(fw::crc32(reinterpret_cast<const uint8_t*>(s), 9), 0xCBF43926u);
}
void test_crc32_empty_is_zero() {
    CHECK_EQ(fw::crc32(nullptr, 0), 0x00000000u);
}

int main() {
    RUN(test_crc32_known_vector);
    RUN(test_crc32_empty_is_zero);
    return REPORT();
}
```

- [ ] **Step 4: Create the host CMake (crc32 only for now)**

`components/fireworkcore/host_test/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.16)
project(fireworkcore_host_test CXX)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
enable_testing()

set(CORE ${CMAKE_CURRENT_SOURCE_DIR}/..)
include_directories(${CORE}/include ${CMAKE_CURRENT_SOURCE_DIR})

# Pure-core sources linked into every test (harmless if unused by a given test).
set(CORE_SRC
    ${CORE}/src/crc32.cpp
)

add_executable(test_crc32 test_crc32.cpp ${CORE_SRC})
add_test(NAME test_crc32 COMMAND test_crc32)
```

- [ ] **Step 5: Configure and build — verify it FAILS to build**

Run:
```
cmake -S components/fireworkcore/host_test -B build/host_test -G Ninja -DCMAKE_CXX_COMPILER=g++
cmake --build build/host_test
```
Expected: build FAILS — `crc32.h` not found / `fw::crc32` undefined.

- [ ] **Step 6: Write the implementation**

`components/fireworkcore/include/crc32.h`:

```cpp
#pragma once
#include <cstdint>
#include <cstddef>
namespace fw { uint32_t crc32(const uint8_t* data, size_t len); }
```

`components/fireworkcore/src/crc32.cpp`:

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

- [ ] **Step 7: Create the IDF component registration**

`components/fireworkcore/CMakeLists.txt` (used by firmware in later plans; harmless now):

```cmake
idf_component_register(
    SRCS "src/crc32.cpp" "src/arming.cpp" "src/sequence.cpp"
    INCLUDE_DIRS "include"
)
```

- [ ] **Step 8: Build and run — verify it PASSES**

Run:
```
cmake --build build/host_test
ctest --test-dir build/host_test --output-on-failure
```
Expected: `test_crc32` PASSES (prints `OK`).

> Note: `build/host_test` references `arming.cpp`/`sequence.cpp` only via the IDF `CMakeLists.txt`, not the host build yet, so the missing sources don't break the host build. The component `CMakeLists.txt` is not compiled until Plan 2 runs `idf.py`.

- [ ] **Step 9: Commit**

```bash
git add .gitignore components/fireworkcore
git commit -m "feat(core): host-test harness + IDF component skeleton + CRC32"
```

---

## Task 2: Shared protocol header

**Files:**
- Create: `components/fireworkcore/include/protocol.h`
- Create: `components/fireworkcore/host_test/test_protocol.cpp`
- Modify: `components/fireworkcore/host_test/CMakeLists.txt`

**Interfaces:**
- Consumes: `fw::crc32`.
- Produces:
  - `enum class fw::MsgType : uint8_t { FIRE=1, ACK=2, ARM=3, DISARM=4, HEARTBEAT=5, ESTOP=6 };`
  - `struct fw::CommandPacket { uint8_t type; char cmd[8]; uint32_t id; uint8_t boxId; uint8_t targetChannel; uint32_t nonce; uint32_t crc; };`
  - `struct fw::AckPacket { uint8_t type; uint32_t responseToId; char note[16]; uint8_t deviceStatus; uint32_t timestamp; uint32_t crc; };`
  - `uint32_t fw::computeCrc(const CommandPacket&);` / `uint32_t fw::computeCrc(const AckPacket&);`
  - `bool fw::crcValid(const CommandPacket&);` / `bool fw::crcValid(const AckPacket&);`

- [ ] **Step 1: Write the failing test**

`components/fireworkcore/host_test/test_protocol.cpp`:

```cpp
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
```

- [ ] **Step 2: Register the test in host CMake**

Append to `components/fireworkcore/host_test/CMakeLists.txt`:

```cmake
add_executable(test_protocol test_protocol.cpp ${CORE_SRC})
add_test(NAME test_protocol COMMAND test_protocol)
```

- [ ] **Step 3: Configure + build — verify FAIL**

Run:
```
cmake -S components/fireworkcore/host_test -B build/host_test -G Ninja -DCMAKE_CXX_COMPILER=g++
cmake --build build/host_test
```
Expected: build FAILS — `protocol.h` not found.

- [ ] **Step 4: Write the implementation**

`components/fireworkcore/include/protocol.h`:

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

- [ ] **Step 5: Build + run — verify PASS**

Run:
```
cmake --build build/host_test
ctest --test-dir build/host_test -R test_protocol --output-on-failure
```
Expected: `test_protocol` PASSES.

- [ ] **Step 6: Commit**

```bash
git add components/fireworkcore/include/protocol.h components/fireworkcore/host_test/test_protocol.cpp components/fireworkcore/host_test/CMakeLists.txt
git commit -m "feat(core): shared packet protocol with CRC helpers"
```

---

## Task 3: Duplicate-ID ring buffer

**Files:**
- Create: `components/fireworkcore/include/recent_ids.h`
- Create: `components/fireworkcore/host_test/test_recent_ids.cpp`
- Modify: `components/fireworkcore/host_test/CMakeLists.txt`

**Interfaces:**
- Consumes: nothing.
- Produces: `class fw::RecentIds<N>` with `bool seenOrRecord(uint32_t id)` — returns `true` if `id` was already recorded (duplicate), else records it and returns `false`.

- [ ] **Step 1: Write the failing test**

`components/fireworkcore/host_test/test_recent_ids.cpp`:

```cpp
#include "check.h"
#include "recent_ids.h"

void test_new_id_is_not_duplicate() {
    fw::RecentIds<4> r;
    CHECK(!r.seenOrRecord(10));
}
void test_repeat_id_is_duplicate() {
    fw::RecentIds<4> r;
    r.seenOrRecord(10);
    CHECK(r.seenOrRecord(10));
}
void test_evicted_id_seen_again_as_new() {
    fw::RecentIds<2> r;            // capacity 2
    r.seenOrRecord(1);
    r.seenOrRecord(2);
    r.seenOrRecord(3);            // evicts 1
    CHECK(!r.seenOrRecord(1));    // 1 no longer remembered
}

int main() {
    RUN(test_new_id_is_not_duplicate);
    RUN(test_repeat_id_is_duplicate);
    RUN(test_evicted_id_seen_again_as_new);
    return REPORT();
}
```

- [ ] **Step 2: Register the test**

Append to `components/fireworkcore/host_test/CMakeLists.txt`:

```cmake
add_executable(test_recent_ids test_recent_ids.cpp ${CORE_SRC})
add_test(NAME test_recent_ids COMMAND test_recent_ids)
```

- [ ] **Step 3: Configure + build — verify FAIL**

Run:
```
cmake -S components/fireworkcore/host_test -B build/host_test -G Ninja -DCMAKE_CXX_COMPILER=g++
cmake --build build/host_test
```
Expected: build FAILS — `recent_ids.h` not found.

- [ ] **Step 4: Write the implementation**

`components/fireworkcore/include/recent_ids.h`:

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

- [ ] **Step 5: Build + run — verify PASS**

Run:
```
cmake --build build/host_test
ctest --test-dir build/host_test -R test_recent_ids --output-on-failure
```
Expected: `test_recent_ids` PASSES (3 checks).

- [ ] **Step 6: Commit**

```bash
git add components/fireworkcore/include/recent_ids.h components/fireworkcore/host_test/test_recent_ids.cpp components/fireworkcore/host_test/CMakeLists.txt
git commit -m "feat(core): recent-id ring buffer for fire dedup"
```

---

## Task 4: Arming state machine (safety-critical core)

**Files:**
- Create: `components/fireworkcore/include/arming.h`
- Create: `components/fireworkcore/src/arming.cpp`
- Create: `components/fireworkcore/host_test/test_arming.cpp`
- Modify: `components/fireworkcore/host_test/CMakeLists.txt`

**Interfaces:**
- Consumes: nothing.
- Produces:
  - `enum class fw::BoxState : uint8_t { SAFE, ARMED };`
  - `struct fw::ArmingConfig { uint32_t heartbeatTimeoutMs = 2000; };`
  - `class fw::ArmingStateMachine` with:
    - `explicit ArmingStateMachine(ArmingConfig cfg = {});`
    - `void setPhysicalSwitch(bool on, uint32_t nowMs);`
    - `bool arm(uint32_t nonce, uint32_t nowMs);` — returns true iff it transitioned to ARMED.
    - `void disarm(uint32_t nowMs);` / `void estop(uint32_t nowMs);` / `void clearEstop(uint32_t nowMs);`
    - `void heartbeat(uint32_t nowMs);` / `void setSequenceRunning(bool running);` / `void update(uint32_t nowMs);`
    - `BoxState state() const;` / `bool canFire(uint32_t nowMs) const;`

**Behavior contract (becomes the tests):**
1. Construction → `SAFE` (never boot armed).
2. `arm()` succeeds only when physical switch ON and not E-STOPped; else returns false and stays SAFE.
3. Re-using the exact previous accepted nonce is rejected (replay protection).
4. Physical switch OFF at any time → immediately SAFE.
5. `estop()` → SAFE and latches; `arm()` fails until `clearEstop()`.
6. While ARMED and NOT sequence-running: `update()` drops to SAFE if `now - lastHeartbeat > timeout`.
7. While ARMED and sequence-running: heartbeat timeout does NOT disarm.
8. `canFire()` true only when ARMED, switch ON, not E-STOPped, and (heartbeat fresh OR sequence-running).

- [ ] **Step 1: Write the failing tests**

`components/fireworkcore/host_test/test_arming.cpp`:

```cpp
#include "check.h"
#include "arming.h"
using namespace fw;

static ArmingStateMachine armed_at(uint32_t now) {
    ArmingStateMachine m;            // default 2000ms timeout
    m.setPhysicalSwitch(true, now);
    CHECK(m.arm(1, now));
    return m;
}

void test_constructs_safe() {
    ArmingStateMachine m;
    CHECK_EQ((int)m.state(), (int)BoxState::SAFE);
}
void test_cannot_arm_with_switch_off() {
    ArmingStateMachine m;
    m.setPhysicalSwitch(false, 0);
    CHECK(!m.arm(1, 0));
    CHECK_EQ((int)m.state(), (int)BoxState::SAFE);
}
void test_arms_with_switch_on() {
    ArmingStateMachine m;
    m.setPhysicalSwitch(true, 0);
    CHECK(m.arm(1, 0));
    CHECK_EQ((int)m.state(), (int)BoxState::ARMED);
}
void test_nonce_replay_rejected() {
    ArmingStateMachine m;
    m.setPhysicalSwitch(true, 0);
    CHECK(m.arm(7, 0));
    m.disarm(0);
    CHECK(!m.arm(7, 0));             // same nonce reused
    CHECK(m.arm(8, 0));              // new nonce ok
}
void test_switch_off_disarms() {
    ArmingStateMachine m = armed_at(0);
    m.setPhysicalSwitch(false, 100);
    CHECK_EQ((int)m.state(), (int)BoxState::SAFE);
}
void test_estop_latches_until_cleared() {
    ArmingStateMachine m = armed_at(0);
    m.estop(50);
    CHECK_EQ((int)m.state(), (int)BoxState::SAFE);
    CHECK(!m.arm(2, 60));            // still latched
    m.clearEstop(70);
    CHECK(m.arm(3, 80));
}
void test_heartbeat_timeout_disarms_when_idle() {
    ArmingStateMachine m = armed_at(0);
    m.update(2001);                  // > 2000ms since last heartbeat
    CHECK_EQ((int)m.state(), (int)BoxState::SAFE);
}
void test_heartbeat_keeps_armed() {
    ArmingStateMachine m = armed_at(0);
    m.heartbeat(1500);
    m.update(2001);                  // only 501ms since heartbeat
    CHECK_EQ((int)m.state(), (int)BoxState::ARMED);
}
void test_sequence_running_ignores_heartbeat_timeout() {
    ArmingStateMachine m = armed_at(0);
    m.setSequenceRunning(true);
    m.update(99999);
    CHECK_EQ((int)m.state(), (int)BoxState::ARMED);
}
void test_canfire_requires_all_gates() {
    ArmingStateMachine m = armed_at(0);
    CHECK(m.canFire(100));
    m.estop(120);
    CHECK(!m.canFire(130));
}

int main() {
    RUN(test_constructs_safe);
    RUN(test_cannot_arm_with_switch_off);
    RUN(test_arms_with_switch_on);
    RUN(test_nonce_replay_rejected);
    RUN(test_switch_off_disarms);
    RUN(test_estop_latches_until_cleared);
    RUN(test_heartbeat_timeout_disarms_when_idle);
    RUN(test_heartbeat_keeps_armed);
    RUN(test_sequence_running_ignores_heartbeat_timeout);
    RUN(test_canfire_requires_all_gates);
    return REPORT();
}
```

- [ ] **Step 2: Register the test and add the source**

Edit `components/fireworkcore/host_test/CMakeLists.txt`:
1. Add `arming.cpp` to `CORE_SRC` so it reads:

```cmake
set(CORE_SRC
    ${CORE}/src/crc32.cpp
    ${CORE}/src/arming.cpp
)
```

2. Append:

```cmake
add_executable(test_arming test_arming.cpp ${CORE_SRC})
add_test(NAME test_arming COMMAND test_arming)
```

- [ ] **Step 3: Configure + build — verify FAIL**

Run:
```
cmake -S components/fireworkcore/host_test -B build/host_test -G Ninja -DCMAKE_CXX_COMPILER=g++
cmake --build build/host_test
```
Expected: build FAILS — `arming.h` not found.

- [ ] **Step 4: Write the header**

`components/fireworkcore/include/arming.h`:

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

- [ ] **Step 5: Write the implementation**

`components/fireworkcore/src/arming.cpp`:

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

void ArmingStateMachine::estop(uint32_t /*nowMs*/) { estopped_ = true; goSafe(); }

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

- [ ] **Step 6: Build + run — verify PASS**

Run:
```
cmake --build build/host_test
ctest --test-dir build/host_test -R test_arming --output-on-failure
```
Expected: `test_arming` PASSES (all checks, prints `OK`).

- [ ] **Step 7: Commit**

```bash
git add components/fireworkcore/include/arming.h components/fireworkcore/src/arming.cpp components/fireworkcore/host_test/test_arming.cpp components/fireworkcore/host_test/CMakeLists.txt
git commit -m "feat(core): arming state machine with full interlock tests"
```

---

## Task 5: Sequence scheduler

**Files:**
- Create: `components/fireworkcore/include/sequence.h`
- Create: `components/fireworkcore/src/sequence.cpp`
- Create: `components/fireworkcore/host_test/test_sequence.cpp`
- Modify: `components/fireworkcore/host_test/CMakeLists.txt`

**Interfaces:**
- Consumes: nothing.
- Produces:
  - `struct fw::SeqStep { uint32_t timeMs; uint8_t boxId; uint8_t channel; };`
  - `class fw::SequenceScheduler` with:
    - `void load(const SeqStep* steps, size_t count);`
    - `void start(uint32_t nowMs);` / `void stop();` / `bool running() const;`
    - `size_t due(uint32_t nowMs, SeqStep* out, size_t outCap);` — copies steps whose `timeMs <= elapsed` and not yet fired into `out` (up to `outCap`), marks them fired, returns how many. When no steps remain unfired, `running()` becomes false.

**Behavior contract:**
1. Before `start()`, `due()` returns 0 and `running()` is false.
2. Steps fire once, in order, when elapsed (`nowMs - startMs`) reaches their `timeMs`.
3. A step is never returned twice.
4. `stop()` halts; subsequent `due()` returns 0.
5. After all steps have fired, `running()` returns false.

- [ ] **Step 1: Write the failing tests**

`components/fireworkcore/host_test/test_sequence.cpp`:

```cpp
#include "check.h"
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
    CHECK(!s.running());
    CHECK_EQ(s.due(0, out, 8), 0);
}
void test_fires_due_steps_in_order() {
    SequenceScheduler s;
    s.load(STEPS, 3);
    s.start(10000);                  // start offset
    SeqStep out[8];
    CHECK_EQ(s.due(10000, out, 8), 1);   // elapsed 0 -> step @0
    CHECK_EQ(out[0].channel, 1);
    CHECK_EQ(s.due(10500, out, 8), 0);   // nothing new yet
    CHECK_EQ(s.due(11600, out, 8), 2);   // elapsed 1600 -> steps @1000 and @1500
    CHECK_EQ(out[0].channel, 2);
    CHECK_EQ(out[1].channel, 5);
}
void test_steps_fire_once() {
    SequenceScheduler s;
    s.load(STEPS, 3);
    s.start(0);
    SeqStep out[8];
    s.due(2000, out, 8);             // all three become due
    CHECK_EQ(s.due(3000, out, 8), 0);    // none repeat
}
void test_stop_halts() {
    SequenceScheduler s;
    s.load(STEPS, 3);
    s.start(0);
    s.stop();
    SeqStep out[8];
    CHECK_EQ(s.due(2000, out, 8), 0);
    CHECK(!s.running());
}
void test_running_false_after_all_fired() {
    SequenceScheduler s;
    s.load(STEPS, 3);
    s.start(0);
    SeqStep out[8];
    s.due(2000, out, 8);             // all become due and fire
    CHECK(!s.running());
}

int main() {
    RUN(test_not_running_before_start);
    RUN(test_fires_due_steps_in_order);
    RUN(test_steps_fire_once);
    RUN(test_stop_halts);
    RUN(test_running_false_after_all_fired);
    return REPORT();
}
```

- [ ] **Step 2: Register the test and add the source**

Edit `components/fireworkcore/host_test/CMakeLists.txt`:
1. Add `sequence.cpp` to `CORE_SRC`:

```cmake
set(CORE_SRC
    ${CORE}/src/crc32.cpp
    ${CORE}/src/arming.cpp
    ${CORE}/src/sequence.cpp
)
```

2. Append:

```cmake
add_executable(test_sequence test_sequence.cpp ${CORE_SRC})
add_test(NAME test_sequence COMMAND test_sequence)
```

- [ ] **Step 3: Configure + build — verify FAIL**

Run:
```
cmake -S components/fireworkcore/host_test -B build/host_test -G Ninja -DCMAKE_CXX_COMPILER=g++
cmake --build build/host_test
```
Expected: build FAILS — `sequence.h` not found.

- [ ] **Step 4: Write the header**

`components/fireworkcore/include/sequence.h`:

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

- [ ] **Step 5: Write the implementation**

`components/fireworkcore/src/sequence.cpp`:

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

- [ ] **Step 6: Build + run the FULL suite — verify PASS**

Run:
```
cmake --build build/host_test
ctest --test-dir build/host_test --output-on-failure
```
Expected: all 5 tests PASS (`test_crc32`, `test_protocol`, `test_recent_ids`, `test_arming`, `test_sequence`).

- [ ] **Step 7: Commit**

```bash
git add components/fireworkcore/include/sequence.h components/fireworkcore/src/sequence.cpp components/fireworkcore/host_test/test_sequence.cpp components/fireworkcore/host_test/CMakeLists.txt
git commit -m "feat(core): sequence scheduler with timeline tests"
```

---

## Done criteria

- `ctest --test-dir build/host_test --output-on-failure` passes all five test executables.
- `components/fireworkcore/` contains only host-compilable, framework-free C++ in `include/` + `src/`, plus an IDF `CMakeLists.txt` ready for firmware to consume.
- The arming state machine encodes every safety gate from spec §5, verified by tests.
- `build/` is git-ignored; no build artifacts committed.

## Next plans (not in this document)

- **Plan 2 — ESP-IDF project skeleton + firing-box firmware:** top-level IDF project (`CMakeLists.txt`, `main/`, `sdkconfig.defaults`, role flag), box role wraps `fireworkcore`, adds the expander channel-driver (`EXPANDER_TYPE`/`FIRE_LEVEL`), non-blocking fire pulse on a FreeRTOS task, `led_strip` status, `esp_now` receive. Bench-tested with an ESP32 + MCP23017 + LEDs (no pyro).
- **Plan 3 — Controller firmware:** ESP32 SoftAP + `esp_http_server`/WebSocket + `esp_now` + heartbeat + sequence engine driving both boxes; NVS config storage.
- **Plan 4 — Web UI (PWA):** arm/disarm, E-STOP, manual cues, groups, sequence builder/runner served from controller flash.
