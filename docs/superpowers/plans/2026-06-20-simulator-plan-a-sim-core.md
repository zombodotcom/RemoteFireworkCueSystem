# Simulator Plan A — Behavioral Sim Core Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Run the real `fireworkcore` safety logic in the browser as a virtual rig (1 controller + 2 firing boxes) compiled to WebAssembly, with a minimal Svelte UI to arm, fire manual cues, run a sequence, and watch the real interlocks — all with no ESP32.

**Architecture:** A thin `sim_bindings.cpp` composes the existing `fireworkcore` classes into a singleton "rig" exposed via a C ABI. It is host-tested in the existing CTest harness first, then compiled to a single self-contained ES module with Emscripten. A Vite + Svelte app loads that module, drives it from a simulated clock, and renders box/LED/channel state read through discrete getters.

**Tech Stack:** C++11, Emscripten (emsdk), CMake/Ninja/g++ (host tests), Node 22 + Vite + Svelte + TypeScript + Vitest.

## Global Constraints

- `components/fireworkcore/include/*` and `src/*` are REUSED UNCHANGED. Do not edit the core; all new C++ goes in `components/fireworkcore/wasm/`.
- The rig wraps `fw::ArmingStateMachine`, `fw::RecentIds`, `fw::SequenceScheduler`, and `protocol.h` (`channelInRange`, `MAX_CHANNELS`). No safety logic is reimplemented.
- `FIRE_MS = 400` (fire-pulse cap), heartbeat timeout = 2000ms (the `ArmingConfig` default).
- The rig honors the firmware contracts from the core: each `rig_tick` calls `setSequenceRunning(scheduler.running())` before `update()`, and outputs (channel `firing`) are gated by `canFire(nowMs)`.
- Two boxes, ids `0` and `1`; 16 channels each (`MAX_CHANNELS`).
- All timestamps are an injected `uint32_t nowMs` (the sim clock). No wall-clock reads in C++.
- Host rig tests run in the existing build: `cmake -S components/fireworkcore/host_test -B build/host_test -G Ninja -DCMAKE_CXX_COMPILER=g++` then `ctest --test-dir build/host_test -R test_rig --output-on-failure`.
- WASM output is a single self-contained ES module (`-sSINGLE_FILE=1`, `-sMODULARIZE=1`, `-sEXPORT_ES6=1`) at `webui/src/core/fireworkcore.js`.
- `webui/` build artifacts and `node_modules/` are git-ignored.

---

## File Structure

```
components/fireworkcore/wasm/
  sim_bindings.h     extern "C" rig API declarations
  sim_bindings.cpp   the rig (virtual controller + 2 boxes) over fireworkcore
components/fireworkcore/host_test/
  test_rig.cpp       CTest host tests for the rig (added to existing CMakeLists)
webui/
  package.json  vite.config.ts  tsconfig.json  .gitignore  index.html
  scripts/build-wasm.sh    emcc build -> src/core/fireworkcore.js
  src/core/fireworkcore.js (generated; git-ignored)
  src/core/wasm.ts         loads the module, typed wrappers for the C ABI
  src/core/connection.ts   SystemConnection interface + SimConnection (+ sim-clock loop)
  src/stores.ts            Svelte stores: snapshot, simRunning
  src/components/ControllerPanel.svelte  BoxPanel.svelte  ChannelGrid.svelte  TransportControls.svelte
  src/App.svelte  src/main.ts
  test/wasm.smoke.test.ts  Vitest: module loads, arm->fire reflects in getters
```

---

## Task 1: Install and pin Emscripten

**Files:** none in-repo except a short README note.
- Create: `webui/EMSDK.md`

**Interfaces:** Produces a working `emcc` on PATH (within an activated emsdk shell).

- [ ] **Step 1: Clone and install emsdk**

Run (PowerShell or Git Bash), from a directory OUTSIDE the repo (e.g. `C:\`):
```bash
git clone https://github.com/emscripten-core/emsdk.git /c/emsdk
cd /c/emsdk
./emsdk install 3.1.74
./emsdk activate 3.1.74
```
Expected: downloads the toolchain (~1 GB) and prints "set up the following tools... emcc".

- [ ] **Step 2: Verify emcc works (Git Bash PATH, not emsdk_env.sh)**

In Git Bash, sourcing `emsdk_env.sh` does NOT put `emcc` on PATH; add the emscripten dir directly:
```bash
export PATH="/c/emsdk/upstream/emscripten:$PATH"
emcc --version
```
Expected: prints `emcc (Emscripten ...) 6.0.0` (latest at install time).

- [ ] **Step 3: Compile a hello-world to confirm the pipeline**

```bash
export PATH="/c/emsdk/upstream/emscripten:$PATH"
printf '#include <cstdio>\nint main(){std::puts("ok");}' > /tmp/h.cpp
emcc /tmp/h.cpp -o ./_probe.mjs -sSINGLE_FILE=1 -sMODULARIZE=1 -sEXPORT_ES6=1
node --input-type=module -e "import M from './_probe.mjs'; await M();" && rm ./_probe.mjs
```
Expected: prints `ok`. (Note: Node ESM on Windows needs a relative or drive-letter `file:///C:/...` specifier — a bare `/tmp/...` path errors with `ERR_INVALID_FILE_URL_PATH`.)

- [ ] **Step 4: Record the version and activation steps**

Create `webui/EMSDK.md`:
```markdown
# Emscripten for the simulator

Installed version: **6.0.0** (latest at install), at `C:\emsdk`.

Activation in Git Bash (sourcing `emsdk_env.sh` does NOT work here):
    export PATH="/c/emsdk/upstream/emscripten:$PATH"

The build script `webui/scripts/build-wasm.sh` self-activates this on PATH, so
normally you just run: `bash webui/scripts/build-wasm.sh` (override `EMSDK_DIR`
if emsdk is elsewhere).
```

- [ ] **Step 5: Commit**

```bash
git add webui/EMSDK.md
git commit -m "chore(sim): install and pin Emscripten 3.1.74"
```

> If 3.1.74 is unavailable, use `./emsdk install latest && ./emsdk activate latest`, then record the actual version printed by `emcc --version` in `webui/EMSDK.md` and use it everywhere this plan says 3.1.74.

---

## Task 2: Rig core — arming, manual fire, pulse, getters (host-tested)

**Files:**
- Create: `components/fireworkcore/wasm/sim_bindings.h`
- Create: `components/fireworkcore/wasm/sim_bindings.cpp`
- Create: `components/fireworkcore/host_test/test_rig.cpp`
- Modify: `components/fireworkcore/host_test/CMakeLists.txt`

**Interfaces:**
- Consumes: `fw::ArmingStateMachine`, `fw::RecentIds`, `fw::channelInRange`, `fw::MAX_CHANNELS`.
- Produces (extern "C"): `rig_reset()`, `rig_set_switch(int box,int on,uint32_t now)`, `rig_heartbeat(uint32_t now)`, `int rig_arm(uint32_t nonce,uint32_t now)`, `rig_disarm(uint32_t now)`, `rig_estop(uint32_t now)`, `rig_clear_estop(uint32_t now)`, `int rig_fire(int box,int ch,uint32_t now)`, `rig_tick(uint32_t now)`, `int rig_box_state(int box)`, `int rig_box_switch(int box)`, `int rig_box_estopped(int box)`, `int rig_box_can_fire(int box,uint32_t now)`, `int rig_channel_firing(int box,int ch)`, `int rig_channel_msleft(int box,int ch,uint32_t now)`. (Sequence functions added in Task 3.)

- [ ] **Step 1: Write the failing tests**

`components/fireworkcore/host_test/test_rig.cpp`:

```cpp
#include "check.h"
#include "sim_bindings.h"

void test_boots_safe() {
    rig_reset();
    CHECK_EQ(rig_box_state(0), 0);   // SAFE
    CHECK_EQ(rig_box_state(1), 0);
    CHECK_EQ(rig_box_switch(0), 0);
    CHECK_EQ(rig_channel_firing(0, 3), 0);
}
void test_arm_requires_switch() {
    rig_reset();
    CHECK_EQ(rig_arm(1, 0), 0);       // no switches on -> none arm
    rig_set_switch(0, 1, 0);
    rig_heartbeat(0);
    CHECK_EQ(rig_arm(2, 0), 1);       // only box 0 armed
    CHECK_EQ(rig_box_state(0), 1);
    CHECK_EQ(rig_box_state(1), 0);
}
void test_fire_when_armed_then_pulse_expires() {
    rig_reset();
    rig_set_switch(0, 1, 0);
    rig_heartbeat(0);
    rig_arm(1, 0);
    CHECK_EQ(rig_fire(0, 3, 0), 1);
    CHECK_EQ(rig_channel_firing(0, 3), 1);
    rig_tick(401);                    // > FIRE_MS(400), heartbeat still fresh (<2000)
    CHECK_EQ(rig_channel_firing(0, 3), 0);
    CHECK_EQ(rig_box_state(0), 1);    // still armed
}
void test_fire_rejected_when_estopped() {
    rig_reset();
    rig_set_switch(0, 1, 0);
    rig_heartbeat(0);
    rig_arm(1, 0);
    rig_estop(0);
    CHECK_EQ(rig_box_can_fire(0, 0), 0);
    CHECK_EQ(rig_fire(0, 5, 0), 0);
    CHECK_EQ(rig_channel_firing(0, 5), 0);
}
void test_fire_rejected_out_of_range() {
    rig_reset();
    rig_set_switch(0, 1, 0);
    rig_heartbeat(0);
    rig_arm(1, 0);
    CHECK_EQ(rig_fire(0, 16, 0), 0);  // MAX_CHANNELS == 16, so 16 is out of range
}
void test_idle_heartbeat_loss_disarms() {
    rig_reset();
    rig_set_switch(0, 1, 0);
    rig_heartbeat(0);
    rig_arm(1, 0);
    rig_tick(2001);                   // idle, no heartbeat since 0 -> disarm
    CHECK_EQ(rig_box_state(0), 0);
}

int main() {
    RUN(test_boots_safe);
    RUN(test_arm_requires_switch);
    RUN(test_fire_when_armed_then_pulse_expires);
    RUN(test_fire_rejected_when_estopped);
    RUN(test_fire_rejected_out_of_range);
    RUN(test_idle_heartbeat_loss_disarms);
    return REPORT();
}
```

- [ ] **Step 2: Register the host test**

Append to `components/fireworkcore/host_test/CMakeLists.txt`:

```cmake
add_executable(test_rig test_rig.cpp ${CORE_SRC} ${CORE}/wasm/sim_bindings.cpp)
target_include_directories(test_rig PRIVATE ${CORE}/wasm)
add_test(NAME test_rig COMMAND test_rig)
```

- [ ] **Step 3: Configure + build — verify FAIL**

Run:
```
cmake -S components/fireworkcore/host_test -B build/host_test -G Ninja -DCMAKE_CXX_COMPILER=g++
cmake --build build/host_test
```
Expected: build FAILS — `sim_bindings.h` not found.

- [ ] **Step 4: Write the header**

`components/fireworkcore/wasm/sim_bindings.h`:

```cpp
#pragma once
#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

void rig_reset(void);
void rig_set_switch(int boxId, int on, uint32_t nowMs);
void rig_heartbeat(uint32_t nowMs);
int  rig_arm(uint32_t nonce, uint32_t nowMs);     // returns number of boxes that armed
void rig_disarm(uint32_t nowMs);
void rig_estop(uint32_t nowMs);
void rig_clear_estop(uint32_t nowMs);
int  rig_fire(int boxId, int channel, uint32_t nowMs);   // 1 if the cue energized a channel

// Sequence API (implemented in Task 3)
void rig_load_sequence(const uint32_t* triples, int count); // [timeMs,boxId,channel] * count
void rig_start_sequence(uint32_t nowMs);
void rig_stop_sequence(uint32_t nowMs);
int  rig_seq_running(void);

void rig_tick(uint32_t nowMs);

int  rig_box_state(int boxId);                 // 0 SAFE, 1 ARMED
int  rig_box_switch(int boxId);                // 0/1
int  rig_box_estopped(int boxId);              // 0/1
int  rig_box_can_fire(int boxId, uint32_t nowMs);
int  rig_channel_firing(int boxId, int channel);
int  rig_channel_msleft(int boxId, int channel, uint32_t nowMs);

#ifdef __cplusplus
}
#endif
```

- [ ] **Step 5: Write the implementation (rig core; sequence funcs are stubs filled in Task 3)**

`components/fireworkcore/wasm/sim_bindings.cpp`:

```cpp
#include "sim_bindings.h"
#include "arming.h"
#include "recent_ids.h"
#include "protocol.h"
#include "sequence.h"

using namespace fw;

namespace {

static const uint32_t FIRE_MS = 400;

struct Channel { bool firing = false; uint32_t offAtMs = 0; };

struct Box {
    ArmingStateMachine arm;
    RecentIds<32> seen;
    Channel ch[MAX_CHANNELS];
    bool switchOn = false;   // mirror (core exposes no getter)
    bool estopped = false;   // mirror
};

struct Rig {
    Box boxes[2];
    SequenceScheduler scheduler;
    bool seqRunning = false;
    uint32_t nextMsgId = 1;
};

Rig g;

bool validBox(int b) { return b == 0 || b == 1; }

void energize(Box& b, int channel, uint32_t now) {
    b.ch[channel].firing = true;
    b.ch[channel].offAtMs = now + FIRE_MS;
}

} // namespace

extern "C" {

void rig_reset(void) { g = Rig(); }

void rig_set_switch(int boxId, int on, uint32_t nowMs) {
    if (!validBox(boxId)) return;
    g.boxes[boxId].switchOn = (on != 0);
    g.boxes[boxId].arm.setPhysicalSwitch(on != 0, nowMs);
}

void rig_heartbeat(uint32_t nowMs) {
    for (int i = 0; i < 2; i++) g.boxes[i].arm.heartbeat(nowMs);
}

int rig_arm(uint32_t nonce, uint32_t nowMs) {
    int n = 0;
    for (int i = 0; i < 2; i++) if (g.boxes[i].arm.arm(nonce, nowMs)) n++;
    return n;
}

void rig_disarm(uint32_t nowMs) {
    for (int i = 0; i < 2; i++) g.boxes[i].arm.disarm(nowMs);
}

void rig_estop(uint32_t nowMs) {
    g.scheduler.stop();
    g.seqRunning = false;
    for (int i = 0; i < 2; i++) { g.boxes[i].estopped = true; g.boxes[i].arm.estop(nowMs); }
}

void rig_clear_estop(uint32_t nowMs) {
    for (int i = 0; i < 2; i++) { g.boxes[i].estopped = false; g.boxes[i].arm.clearEstop(nowMs); }
}

int rig_fire(int boxId, int channel, uint32_t nowMs) {
    if (!validBox(boxId)) return 0;
    if (!channelInRange((uint8_t)channel)) return 0;
    Box& b = g.boxes[boxId];
    if (!b.arm.canFire(nowMs)) return 0;
    uint32_t id = g.nextMsgId++;
    if (b.seen.seenOrRecord(id)) return 0;   // mirrors firmware dedup
    energize(b, channel, nowMs);
    return 1;
}

void rig_tick(uint32_t nowMs) {
    g.seqRunning = g.scheduler.running();
    for (int i = 0; i < 2; i++) {
        g.boxes[i].arm.setSequenceRunning(g.seqRunning);   // firmware contract
        g.boxes[i].arm.update(nowMs);
    }
    if (g.scheduler.running()) {
        SeqStep due[MAX_SEQ_STEPS];
        size_t n = g.scheduler.due(nowMs, due, MAX_SEQ_STEPS);
        for (size_t k = 0; k < n; k++) {
            int boxId = due[k].boxId;
            if (!validBox(boxId)) continue;
            Box& b = g.boxes[boxId];
            if (b.arm.canFire(nowMs) && channelInRange(due[k].channel))
                energize(b, due[k].channel, nowMs);
        }
        g.seqRunning = g.scheduler.running();
    }
    for (int i = 0; i < 2; i++)
        for (int c = 0; c < MAX_CHANNELS; c++)
            if (g.boxes[i].ch[c].firing && nowMs >= g.boxes[i].ch[c].offAtMs)
                g.boxes[i].ch[c].firing = false;
}

// Sequence API — bodies completed in Task 3 (declared here so the unit links).
void rig_load_sequence(const uint32_t* triples, int count) {
    SeqStep steps[MAX_SEQ_STEPS];
    if (count < 0) count = 0;
    if ((size_t)count > MAX_SEQ_STEPS) count = (int)MAX_SEQ_STEPS;
    for (int i = 0; i < count; i++) {
        steps[i].timeMs  = triples[i * 3 + 0];
        steps[i].boxId   = (uint8_t)triples[i * 3 + 1];
        steps[i].channel = (uint8_t)triples[i * 3 + 2];
    }
    g.scheduler.load(steps, (size_t)count);
}
void rig_start_sequence(uint32_t nowMs) { g.scheduler.start(nowMs); g.seqRunning = g.scheduler.running(); }
void rig_stop_sequence(uint32_t nowMs)  { (void)nowMs; g.scheduler.stop(); g.seqRunning = false; }
int  rig_seq_running(void) { return g.seqRunning ? 1 : 0; }

int rig_box_state(int boxId)    { return validBox(boxId) ? (int)g.boxes[boxId].arm.state() : 0; }
int rig_box_switch(int boxId)   { return validBox(boxId) && g.boxes[boxId].switchOn ? 1 : 0; }
int rig_box_estopped(int boxId) { return validBox(boxId) && g.boxes[boxId].estopped ? 1 : 0; }
int rig_box_can_fire(int boxId, uint32_t nowMs) { return validBox(boxId) && g.boxes[boxId].arm.canFire(nowMs) ? 1 : 0; }
int rig_channel_firing(int boxId, int channel) {
    if (!validBox(boxId) || !channelInRange((uint8_t)channel)) return 0;
    return g.boxes[boxId].ch[channel].firing ? 1 : 0;
}
int rig_channel_msleft(int boxId, int channel, uint32_t nowMs) {
    if (!validBox(boxId) || !channelInRange((uint8_t)channel)) return 0;
    Channel& c = g.boxes[boxId].ch[channel];
    if (!c.firing || nowMs >= c.offAtMs) return 0;
    return (int)(c.offAtMs - nowMs);
}

} // extern "C"
```

> Note: this file already contains the Task 3 sequence bodies (they are simple and live in the same unit). Task 3 only adds the sequence-specific *tests*; if you implement Task 2 from this listing, the sequence functions are present but unverified until Task 3.

- [ ] **Step 6: Build + run — verify PASS**

Run:
```
cmake -S components/fireworkcore/host_test -B build/host_test -G Ninja -DCMAKE_CXX_COMPILER=g++
cmake --build build/host_test
ctest --test-dir build/host_test -R test_rig --output-on-failure
```
Expected: `test_rig` PASSES (6 tests). Also confirm the full suite is still green: `ctest --test-dir build/host_test --output-on-failure` (6 suites).

- [ ] **Step 7: Commit**

```bash
git add components/fireworkcore/wasm/sim_bindings.h components/fireworkcore/wasm/sim_bindings.cpp components/fireworkcore/host_test/test_rig.cpp components/fireworkcore/host_test/CMakeLists.txt
git commit -m "feat(sim): rig core (arming, manual fire, pulse, getters) host-tested"
```

---

## Task 3: Rig sequence dispatch (host-tested)

**Files:**
- Modify: `components/fireworkcore/host_test/test_rig.cpp`

**Interfaces:**
- Consumes the `rig_load_sequence`/`rig_start_sequence`/`rig_stop_sequence`/`rig_seq_running` functions already present in `sim_bindings.cpp` from Task 2.
- Produces: verified sequence-dispatch behavior.

- [ ] **Step 1: Add the failing sequence tests**

In `components/fireworkcore/host_test/test_rig.cpp`, add these functions before `main()`:

```cpp
void test_sequence_fires_cues_in_order() {
    rig_reset();
    rig_set_switch(0, 1, 0);
    rig_set_switch(1, 1, 0);
    rig_heartbeat(0);
    rig_arm(1, 0);
    uint32_t steps[] = { 0,0,1,  100,1,5 };   // t0: box0 ch1 ; t100: box1 ch5
    rig_load_sequence(steps, 2);
    rig_start_sequence(0);
    rig_heartbeat(0); rig_tick(0);
    CHECK_EQ(rig_channel_firing(0, 1), 1);
    CHECK_EQ(rig_channel_firing(1, 5), 0);
    rig_heartbeat(100); rig_tick(100);
    CHECK_EQ(rig_channel_firing(1, 5), 1);
}
void test_sequence_running_keeps_armed_without_heartbeat() {
    rig_reset();
    rig_set_switch(0, 1, 0);
    rig_heartbeat(0);
    rig_arm(1, 0);
    uint32_t steps[] = { 5000,0,2 };          // far-future step keeps scheduler running
    rig_load_sequence(steps, 1);
    rig_start_sequence(0);
    rig_tick(3000);                            // no heartbeat since 0, but seq running -> stays armed
    CHECK_EQ(rig_box_state(0), 1);
    CHECK_EQ(rig_seq_running(), 1);
}
void test_estop_mid_show_skips_remaining_cues() {
    rig_reset();
    rig_set_switch(0, 1, 0);
    rig_heartbeat(0);
    rig_arm(1, 0);
    uint32_t steps[] = { 0,0,1,  200,0,2 };
    rig_load_sequence(steps, 2);
    rig_start_sequence(0);
    rig_heartbeat(0); rig_tick(0);
    CHECK_EQ(rig_channel_firing(0, 1), 1);
    rig_estop(50);                             // kill mid-show
    rig_tick(200);                             // second cue must NOT fire
    CHECK_EQ(rig_channel_firing(0, 2), 0);
    CHECK_EQ(rig_seq_running(), 0);
}
```

And add to `main()` (before `return REPORT();`):

```cpp
    RUN(test_sequence_fires_cues_in_order);
    RUN(test_sequence_running_keeps_armed_without_heartbeat);
    RUN(test_estop_mid_show_skips_remaining_cues);
```

- [ ] **Step 2: Build — verify the new tests pass (sequence bodies already exist from Task 2)**

Run:
```
cmake --build build/host_test
ctest --test-dir build/host_test -R test_rig --output-on-failure
```
Expected: `test_rig` PASSES (now 9 tests). If `test_sequence_*` FAIL, fix the sequence bodies in `sim_bindings.cpp` (they were written in Task 2) until green — this is the TDD check for the dispatch logic.

- [ ] **Step 3: Full suite green**

Run: `ctest --test-dir build/host_test --output-on-failure`
Expected: all 6 suites pass.

- [ ] **Step 4: Commit**

```bash
git add components/fireworkcore/host_test/test_rig.cpp
git commit -m "test(sim): cover rig sequence dispatch, seq-armed override, estop skip"
```

---

## Task 4: Emscripten build script → `fireworkcore.js`

**Files:**
- Create: `webui/scripts/build-wasm.sh`
- Create: `webui/.gitignore`

**Interfaces:**
- Produces `webui/src/core/fireworkcore.js` — a single-file ES module whose default export is an async factory `() => Promise<Module>`; `Module.ccall(name, returnType, argTypes, args)` invokes the rig C ABI; `Module._malloc`/`Module._free`/`Module.setValue` are available for array marshalling.

- [ ] **Step 1: Write the build script**

`webui/scripts/build-wasm.sh`:

```bash
#!/usr/bin/env bash
# Build the fireworkcore rig to a single-file ES module.
# Self-activates emsdk on PATH (Git Bash: sourcing emsdk_env.sh does NOT work;
# emcc.exe lives in upstream/emscripten). Override EMSDK_DIR if installed elsewhere.
set -euo pipefail

EMSDK_DIR="${EMSDK_DIR:-/c/emsdk}"
export PATH="$EMSDK_DIR/upstream/emscripten:$PATH"
command -v emcc >/dev/null || { echo "emcc not found; check EMSDK_DIR ($EMSDK_DIR)"; exit 1; }

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
CORE="$ROOT/components/fireworkcore"
OUT="$ROOT/webui/src/core/fireworkcore.js"
mkdir -p "$(dirname "$OUT")"

EXPORTS='["_rig_reset","_rig_set_switch","_rig_heartbeat","_rig_arm","_rig_disarm","_rig_estop","_rig_clear_estop","_rig_fire","_rig_load_sequence","_rig_start_sequence","_rig_stop_sequence","_rig_seq_running","_rig_tick","_rig_box_state","_rig_box_switch","_rig_box_estopped","_rig_box_can_fire","_rig_channel_firing","_rig_channel_msleft","_malloc","_free"]'

emcc \
  "$CORE/src/crc32.cpp" "$CORE/src/arming.cpp" "$CORE/src/sequence.cpp" \
  "$CORE/wasm/sim_bindings.cpp" \
  -I "$CORE/include" -I "$CORE/wasm" \
  -O2 -std=c++11 \
  -sMODULARIZE=1 -sEXPORT_ES6=1 -sSINGLE_FILE=1 -sALLOW_MEMORY_GROWTH=1 \
  -sEXPORTED_FUNCTIONS="$EXPORTS" \
  -sEXPORTED_RUNTIME_METHODS='["ccall","cwrap","setValue","getValue"]' \
  -o "$OUT"

echo "built $OUT"
```

- [ ] **Step 2: Ignore generated + node artifacts**

`webui/.gitignore`:
```
node_modules/
dist/
src/core/fireworkcore.js
```

- [ ] **Step 3: Build the module**

Run (the script self-activates emsdk on PATH):
```bash
bash webui/scripts/build-wasm.sh
```
Expected: prints `built .../webui/src/core/fireworkcore.js` and the file exists.

- [ ] **Step 4: Smoke-test the module in Node**

Run:
```bash
node --input-type=module -e "
import Module from './webui/src/core/fireworkcore.js';
const m = await Module();
m.ccall('rig_reset', null, [], []);
m.ccall('rig_set_switch', null, ['number','number','number'], [0,1,0]);
m.ccall('rig_heartbeat', null, ['number'], [0]);
const armed = m.ccall('rig_arm', 'number', ['number','number'], [1,0]);
const fired = m.ccall('rig_fire', 'number', ['number','number','number'], [0,3,0]);
const firing = m.ccall('rig_channel_firing', 'number', ['number','number'], [0,3]);
console.log('armed=' + armed + ' fired=' + fired + ' firing=' + firing);
if (armed!==1 || fired!==1 || firing!==1) process.exit(1);
console.log('WASM SMOKE OK');
"
```
Expected: prints `armed=1 fired=1 firing=1` then `WASM SMOKE OK` (exit 0).

- [ ] **Step 5: Commit (the generated .js is git-ignored — only the script + gitignore are committed)**

```bash
git add webui/scripts/build-wasm.sh webui/.gitignore
git commit -m "build(sim): emcc single-file ES module build for the rig"
```

---

## Task 5: Vite + Svelte scaffold, typed WASM wrapper, SimConnection, Vitest smoke

**Files:**
- Create: `webui/package.json`, `webui/vite.config.ts`, `webui/tsconfig.json`, `webui/index.html`, `webui/src/main.ts`, `webui/src/App.svelte` (placeholder)
- Create: `webui/src/core/wasm.ts`
- Create: `webui/src/core/connection.ts`
- Create: `webui/src/stores.ts`
- Create: `webui/test/wasm.smoke.test.ts`

**Interfaces:**
- Consumes: `webui/src/core/fireworkcore.js` (from Task 4).
- Produces:
  - `wasm.ts`: `loadRig(): Promise<Rig>` where `Rig` has typed methods `reset()`, `setSwitch(box,on,now)`, `heartbeat(now)`, `arm(nonce,now):number`, `disarm(now)`, `estop(now)`, `clearEstop(now)`, `fire(box,ch,now):number`, `loadSequence(triples:number[],now?)`, `startSequence(now)`, `stopSequence(now)`, `seqRunning():boolean`, `tick(now)`, and getters `boxState(box)`, `boxSwitch(box)`, `boxEstopped(box)`, `boxCanFire(box,now)`, `channelFiring(box,ch)`, `channelMsLeft(box,ch,now)`.
  - `connection.ts`: `interface SystemConnection { … }` and `class SimConnection implements SystemConnection`.
  - `stores.ts`: `snapshot` (writable store of `Snapshot`), `Snapshot` type.

- [ ] **Step 1: Scaffold package files**

`webui/package.json`:
```json
{
  "name": "firework-sim",
  "private": true,
  "type": "module",
  "scripts": {
    "dev": "vite",
    "build": "vite build",
    "test": "vitest run"
  },
  "devDependencies": {
    "@sveltejs/vite-plugin-svelte": "^3.1.2",
    "svelte": "^4.2.19",
    "typescript": "^5.5.4",
    "vite": "^5.4.0",
    "vitest": "^2.0.5"
  }
}
```

`webui/vite.config.ts`:
```ts
import { defineConfig } from "vite";
import { svelte } from "@sveltejs/vite-plugin-svelte";

export default defineConfig({
  plugins: [svelte()],
  test: { environment: "node" },
});
```

`webui/tsconfig.json`:
```json
{
  "compilerOptions": {
    "target": "ES2020",
    "module": "ESNext",
    "moduleResolution": "bundler",
    "strict": true,
    "allowJs": true,
    "noEmit": true,
    "skipLibCheck": true,
    "types": ["svelte", "vitest/globals"]
  },
  "include": ["src", "test"]
}
```

`webui/index.html`:
```html
<!doctype html>
<html lang="en">
  <head><meta charset="utf-8" /><title>Firework Simulator</title></head>
  <body><div id="app"></div><script type="module" src="/src/main.ts"></script></body>
</html>
```

`webui/src/App.svelte` (placeholder, replaced in Task 6):
```svelte
<script lang="ts">
</script>
<h1>Firework Simulator</h1>
```

`webui/src/main.ts`:
```ts
import App from "./App.svelte";
const app = new App({ target: document.getElementById("app")! });
export default app;
```

- [ ] **Step 2: Install deps**

Run: `cd webui && npm install`
Expected: installs without errors; `node_modules/` created.

- [ ] **Step 3: Write the typed WASM wrapper**

`webui/src/core/wasm.ts`:
```ts
// @ts-ignore - generated single-file ES module, no types
import Module from "./fireworkcore.js";

export interface Rig {
  reset(): void;
  setSwitch(box: number, on: boolean, now: number): void;
  heartbeat(now: number): void;
  arm(nonce: number, now: number): number;
  disarm(now: number): void;
  estop(now: number): void;
  clearEstop(now: number): void;
  fire(box: number, ch: number, now: number): number;
  loadSequence(triples: number[]): void;
  startSequence(now: number): void;
  stopSequence(now: number): void;
  seqRunning(): boolean;
  tick(now: number): void;
  boxState(box: number): number;     // 0 SAFE, 1 ARMED
  boxSwitch(box: number): boolean;
  boxEstopped(box: number): boolean;
  boxCanFire(box: number, now: number): boolean;
  channelFiring(box: number, ch: number): boolean;
  channelMsLeft(box: number, ch: number, now: number): number;
}

export async function loadRig(): Promise<Rig> {
  const m = await Module();
  const N = "number";
  const call = (fn: string, ret: string | null, types: string[], args: any[]) =>
    m.ccall(fn, ret as any, types as any, args);

  return {
    reset: () => call("rig_reset", null, [], []),
    setSwitch: (box, on, now) => call("rig_set_switch", null, [N, N, N], [box, on ? 1 : 0, now]),
    heartbeat: (now) => call("rig_heartbeat", null, [N], [now]),
    arm: (nonce, now) => call("rig_arm", N, [N, N], [nonce, now]),
    disarm: (now) => call("rig_disarm", null, [N], [now]),
    estop: (now) => call("rig_estop", null, [N], [now]),
    clearEstop: (now) => call("rig_clear_estop", null, [N], [now]),
    fire: (box, ch, now) => call("rig_fire", N, [N, N, N], [box, ch, now]),
    loadSequence: (triples) => {
      const count = Math.floor(triples.length / 3);
      const ptr = m._malloc(triples.length * 4);
      for (let i = 0; i < triples.length; i++) m.setValue(ptr + i * 4, triples[i], "i32");
      call("rig_load_sequence", null, [N, N], [ptr, count]);
      m._free(ptr);
    },
    startSequence: (now) => call("rig_start_sequence", null, [N], [now]),
    stopSequence: (now) => call("rig_stop_sequence", null, [N], [now]),
    seqRunning: () => call("rig_seq_running", N, [], []) === 1,
    tick: (now) => call("rig_tick", null, [N], [now]),
    boxState: (box) => call("rig_box_state", N, [N], [box]),
    boxSwitch: (box) => call("rig_box_switch", N, [N], [box]) === 1,
    boxEstopped: (box) => call("rig_box_estopped", N, [N], [box]) === 1,
    boxCanFire: (box, now) => call("rig_box_can_fire", N, [N, N], [box, now]) === 1,
    channelFiring: (box, ch) => call("rig_channel_firing", N, [N, N], [box, ch]) === 1,
    channelMsLeft: (box, ch, now) => call("rig_channel_msleft", N, [N, N, N], [box, ch, now]),
  };
}
```

- [ ] **Step 4: Write stores + SimConnection**

`webui/src/stores.ts`:
```ts
import { writable } from "svelte/store";

export interface ChannelView { firing: boolean; msLeft: number; }
export interface BoxView { id: number; switchOn: boolean; armed: boolean; estopped: boolean; canFire: boolean; channels: ChannelView[]; }
export interface Snapshot { now: number; seqRunning: boolean; boxes: BoxView[]; }

export const snapshot = writable<Snapshot>({ now: 0, seqRunning: false, boxes: [] });
```

`webui/src/core/connection.ts`:
```ts
import { loadRig, type Rig } from "./wasm";
import { snapshot, type Snapshot, type BoxView } from "../stores";

export interface SystemConnection {
  setSwitch(box: number, on: boolean): void;
  heartbeat(): void;
  arm(): void;
  disarm(): void;
  estop(): void;
  clearEstop(): void;
  fire(box: number, ch: number): void;
  loadSequence(triples: number[]): void;
  startSequence(): void;
  stopSequence(): void;
  start(): void;        // begin the sim-clock loop
  stop(): void;         // pause the loop
  setConnected(on: boolean): void; // when false, stop sending heartbeats (simulate phone drop)
}

const CHANNELS = 16;

export class SimConnection implements SystemConnection {
  private rig!: Rig;
  private now = 0;
  private last = 0;
  private nonce = 1;
  private timer: ReturnType<typeof setInterval> | null = null;
  private connected = true;

  static async create(): Promise<SimConnection> {
    const c = new SimConnection();
    c.rig = await loadRig();
    c.rig.reset();
    c.publish();
    return c;
  }

  setSwitch(box: number, on: boolean) { this.rig.setSwitch(box, on, this.now); this.publish(); }
  heartbeat() { this.rig.heartbeat(this.now); }
  arm() { this.rig.arm(this.nonce++, this.now); this.publish(); }
  disarm() { this.rig.disarm(this.now); this.publish(); }
  estop() { this.rig.estop(this.now); this.publish(); }
  clearEstop() { this.rig.clearEstop(this.now); this.publish(); }
  fire(box: number, ch: number) { this.rig.fire(box, ch, this.now); this.publish(); }
  loadSequence(triples: number[]) { this.rig.loadSequence(triples); }
  startSequence() { this.rig.startSequence(this.now); this.publish(); }
  stopSequence() { this.rig.stopSequence(this.now); this.publish(); }
  setConnected(on: boolean) { this.connected = on; }

  start() {
    if (this.timer) return;
    this.last = Date.now();
    this.timer = setInterval(() => {
      const t = Date.now();
      this.now += t - this.last;
      this.last = t;
      if (this.connected) this.rig.heartbeat(this.now);
      this.rig.tick(this.now);
      this.publish();
    }, 50);
  }
  stop() { if (this.timer) { clearInterval(this.timer); this.timer = null; } }

  private publish() {
    const boxes: BoxView[] = [];
    for (let b = 0; b < 2; b++) {
      const channels = [];
      for (let c = 0; c < CHANNELS; c++)
        channels.push({ firing: this.rig.channelFiring(b, c), msLeft: this.rig.channelMsLeft(b, c, this.now) });
      boxes.push({
        id: b, switchOn: this.rig.boxSwitch(b), armed: this.rig.boxState(b) === 1,
        estopped: this.rig.boxEstopped(b), canFire: this.rig.boxCanFire(b, this.now), channels,
      });
    }
    const snap: Snapshot = { now: this.now, seqRunning: this.rig.seqRunning(), boxes };
    snapshot.set(snap);
  }
}
```

- [ ] **Step 5: Write the Vitest WASM smoke test**

`webui/test/wasm.smoke.test.ts`:
```ts
import { describe, it, expect } from "vitest";
import { loadRig } from "../src/core/wasm";

describe("wasm rig", () => {
  it("arms and fires a channel", async () => {
    const rig = await loadRig();
    rig.reset();
    expect(rig.boxState(0)).toBe(0);          // SAFE
    rig.setSwitch(0, true, 0);
    rig.heartbeat(0);
    expect(rig.arm(1, 0)).toBe(1);            // only box 0 armed
    expect(rig.boxState(0)).toBe(1);          // ARMED
    expect(rig.fire(0, 3, 0)).toBe(1);
    expect(rig.channelFiring(0, 3)).toBe(true);
    rig.tick(401);                            // pulse expires after FIRE_MS
    expect(rig.channelFiring(0, 3)).toBe(false);
  });

  it("rejects fire when disarmed", async () => {
    const rig = await loadRig();
    rig.reset();
    expect(rig.fire(0, 0, 0)).toBe(0);
    expect(rig.channelFiring(0, 0)).toBe(false);
  });
});
```

- [ ] **Step 6: Build the WASM, then run Vitest**

Run (build-wasm.sh self-activates emsdk on PATH):
```bash
bash webui/scripts/build-wasm.sh
cd webui && npm run test
```
Expected: both tests pass (`2 passed`).

- [ ] **Step 7: Commit**

```bash
git add webui/package.json webui/vite.config.ts webui/tsconfig.json webui/index.html webui/src/main.ts webui/src/App.svelte webui/src/core/wasm.ts webui/src/core/connection.ts webui/src/stores.ts webui/test/wasm.smoke.test.ts
git commit -m "feat(sim): Vite+Svelte scaffold, typed WASM wrapper, SimConnection, smoke test"
```

---

## Task 6: Minimal Svelte harness UI

**Files:**
- Create: `webui/src/components/ControllerPanel.svelte`, `webui/src/components/BoxPanel.svelte`, `webui/src/components/ChannelGrid.svelte`, `webui/src/components/TransportControls.svelte`
- Modify: `webui/src/App.svelte`

**Interfaces:**
- Consumes: `SimConnection` (from Task 5), `snapshot` store, `BoxView`.

- [ ] **Step 1: ChannelGrid — 16 LED cells for one box**

`webui/src/components/ChannelGrid.svelte`:
```svelte
<script lang="ts">
  import type { BoxView } from "../stores";
  export let box: BoxView;
  export let onFire: (ch: number) => void;
</script>

<div class="grid">
  {#each box.channels as ch, i}
    <button class="cell" class:firing={ch.firing} on:click={() => onFire(i)} title={`ch ${i}`}>
      {i}
    </button>
  {/each}
</div>

<style>
  .grid { display: grid; grid-template-columns: repeat(8, 1fr); gap: 4px; }
  .cell { aspect-ratio: 1; border: 1px solid #444; background: #222; color: #888; cursor: pointer; }
  .cell.firing { background: gold; color: #000; box-shadow: 0 0 10px gold; }
</style>
```

- [ ] **Step 2: BoxPanel — one firing box**

`webui/src/components/BoxPanel.svelte`:
```svelte
<script lang="ts">
  import type { BoxView } from "../stores";
  import ChannelGrid from "./ChannelGrid.svelte";
  export let box: BoxView;
  export let onSwitch: (on: boolean) => void;
  export let onFire: (ch: number) => void;
</script>

<div class="box">
  <header>
    <strong>Box {box.id}</strong>
    <span class="lamp" class:armed={box.armed} class:estop={box.estopped}>
      {box.estopped ? "E-STOP" : box.armed ? "ARMED" : "SAFE"}
    </span>
    <label><input type="checkbox" checked={box.switchOn} on:change={(e) => onSwitch((e.target as HTMLInputElement).checked)} /> arm switch</label>
  </header>
  <ChannelGrid {box} {onFire} />
</div>

<style>
  .box { border: 1px solid #555; padding: 10px; border-radius: 6px; background: #1a1a1a; }
  header { display: flex; gap: 10px; align-items: center; margin-bottom: 8px; }
  .lamp { padding: 2px 8px; border-radius: 4px; background: #722; color: #fff; }
  .lamp.armed { background: #161; }
  .lamp.estop { background: #b00; }
</style>
```

- [ ] **Step 3: TransportControls — arm/estop/run + simulate disconnect**

`webui/src/components/TransportControls.svelte`:
```svelte
<script lang="ts">
  export let onArm: () => void;
  export let onDisarm: () => void;
  export let onEstop: () => void;
  export let onClearEstop: () => void;
  export let onRunDemo: () => void;
  export let onToggleConnected: (on: boolean) => void;
  let connected = true;
</script>

<div class="bar">
  <button on:click={onArm}>ARM</button>
  <button on:click={onDisarm}>Disarm</button>
  <button class="estop" on:click={onEstop}>E-STOP</button>
  <button on:click={onClearEstop}>Clear E-STOP</button>
  <button on:click={onRunDemo}>Run demo sequence</button>
  <label><input type="checkbox" bind:checked={connected} on:change={() => onToggleConnected(connected)} /> phone connected</label>
</div>

<style>
  .bar { display: flex; gap: 8px; align-items: center; margin: 12px 0; flex-wrap: wrap; }
  .estop { background: #b00; color: #fff; font-weight: bold; }
</style>
```

- [ ] **Step 4: ControllerPanel — wires everything to a SimConnection**

`webui/src/components/ControllerPanel.svelte`:
```svelte
<script lang="ts">
  import { onMount, onDestroy } from "svelte";
  import { snapshot } from "../stores";
  import { SimConnection } from "../core/connection";
  import BoxPanel from "./BoxPanel.svelte";
  import TransportControls from "./TransportControls.svelte";

  let conn: SimConnection | null = null;

  onMount(async () => {
    conn = await SimConnection.create();
    conn.start();
  });
  onDestroy(() => conn?.stop());

  // A small demo sequence: box0 ch0 @0, box0 ch1 @600, box1 ch5 @1200 (flat [timeMs,box,ch] triples)
  const demo = [0, 0, 0,  600, 0, 1,  1200, 1, 5];
  function runDemo() { conn?.loadSequence(demo); conn?.startSequence(); }
</script>

{#if conn}
  <TransportControls
    onArm={() => conn!.arm()}
    onDisarm={() => conn!.disarm()}
    onEstop={() => conn!.estop()}
    onClearEstop={() => conn!.clearEstop()}
    onRunDemo={runDemo}
    onToggleConnected={(on) => conn!.setConnected(on)}
  />
  <div class="boxes">
    {#each $snapshot.boxes as box (box.id)}
      <BoxPanel
        {box}
        onSwitch={(on) => conn!.setSwitch(box.id, on)}
        onFire={(ch) => conn!.fire(box.id, ch)}
      />
    {/each}
  </div>
  <p class="status">t={$snapshot.now}ms · sequence {$snapshot.seqRunning ? "running" : "idle"}</p>
{:else}
  <p>Loading WASM core…</p>
{/if}

<style>
  .boxes { display: grid; grid-template-columns: 1fr 1fr; gap: 16px; }
  .status { color: #888; font-family: monospace; }
</style>
```

- [ ] **Step 5: App shell**

Replace `webui/src/App.svelte`:
```svelte
<script lang="ts">
  import ControllerPanel from "./components/ControllerPanel.svelte";
</script>

<main>
  <h1>🎆 Firework Simulator</h1>
  <p class="sub">Virtual rig running the real <code>fireworkcore</code> via WASM — no hardware.</p>
  <ControllerPanel />
</main>

<style>
  :global(body) { background: #111; color: #ddd; font-family: system-ui, sans-serif; margin: 0; }
  main { max-width: 900px; margin: 0 auto; padding: 24px; }
  .sub { color: #999; }
</style>
```

- [ ] **Step 6: Type-check and build**

Run:
```bash
cd webui && npx svelte-check --tsconfig ./tsconfig.json || true
npm run build
```
Expected: `npm run build` completes and writes `webui/dist/`. (`svelte-check` may warn; the build must succeed.)

- [ ] **Step 7: Manual verification**

Run (emsdk-built `fireworkcore.js` must exist from Task 4/5):
```bash
cd webui && npm run dev
```
Open the printed localhost URL and confirm:
1. Two boxes render, both showing **SAFE**.
2. Toggle a box's **arm switch** on, click **ARM** → that box shows **ARMED** (the other stays SAFE until its switch is on).
3. Click a channel cell on an armed box → it flashes **gold** (~400ms) then clears.
4. Click a channel on a SAFE box → nothing fires.
5. Arm both, click **Run demo sequence** → cues fire in order across both boxes.
6. While armed and idle, uncheck **phone connected** → within ~2s the boxes drop to **SAFE** (heartbeat dead-man).
7. Click **E-STOP** → boxes latch SAFE and channels stop; **Clear E-STOP** then re-ARM restores.

- [ ] **Step 8: Commit**

```bash
git add webui/src/components webui/src/App.svelte
git commit -m "feat(sim): minimal Svelte harness — boxes, LEDs, arm/fire/estop, demo sequence"
```

---

## Done criteria

- `ctest --test-dir build/host_test --output-on-failure` passes all 6 suites (rig included).
- `cd webui && npm run test` passes the WASM smoke tests.
- `npm run dev` shows the virtual rig; all 7 manual checks in Task 6 Step 7 behave correctly.
- No edits were made to `components/fireworkcore/include/*` or `src/*`.
- Generated `fireworkcore.js` and `node_modules/` are git-ignored.

## Next plan (not in this document)

- **Plan B — Authoring + show building:** `config.ts` schema (channels/groups/sequences) + validation + save/load/export; `ChannelEditor`/`GroupEditor`/`SequenceEditor` components; group→step expansion feeding `SimConnection.loadSequence`; layout/polish. Needs no Emscripten beyond what Plan A installed.
