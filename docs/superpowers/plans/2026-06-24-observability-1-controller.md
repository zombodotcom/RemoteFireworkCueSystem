# Observability Plan 1 — Controller Infra + API

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give the controller an event log + diagnostics counters + a fault summary, exposed via `GET /api/events` (incremental) and an extended `/api/status`, so the web UI and CYD (later plans) can show what the controller is doing.

**Architecture:** A passive observer on the host-tested `BoxLink` surfaces fire/ACK/retry/fail events (with ACK latency); the control loop also logs commands, sequence start/done, and box link up/down into a pure ring-buffer `EventLog` (mutex-guarded), while maintaining diag counters and a derived fault code in the status snapshot. Two HTTP handlers serve it.

**Tech Stack:** ESP-IDF v6.0.1 (C++), portable `fireworkcore` core with host CTest, ESP-IDF `esp_http_server`.

## Global Constraints

- **ESP-IDF builds run only in PowerShell**, after `& 'C:\esp\v6.0.1\esp-idf\export.ps1'`, then `idf.py build` in `controller`.
- **Host core tests run in Git Bash** with **g++ forced**: `cmake -B build -G Ninja -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++` (CMake otherwise auto-picks a stray MSVC). Then `cmake --build build && ctest --test-dir build`.
- **Observability is strictly READ-ONLY:** it must NOT change firing, arming, retry, or dead-man behavior. The `BoxLink` observer hooks are passive (default no-op) and must not alter any control flow or timing in `BoxLink`.
- **Single-threaded discipline:** events/counters are written only from the control-loop task. The event ring is shared with the httpd task and MUST be guarded by a FreeRTOS mutex; hold the mutex only for the copy/push, never across I/O.
- `/api/status` stays **backward compatible** — existing keys unchanged; `diag`, `fault`, and per-box `lastHeardMs` are additive.
- `EventLog` and the JSON parser pieces are **pure C++** (no IDF headers) so they host-test.
- Event severities: `SEV_INFO=0, SEV_WARN=1, SEV_ERR=2`. Ring capacity `EVENT_CAP/CAP = 32`, `msg` ≤ 47 chars.

---

## File Structure

- `components/fireworkcore/include/box_link.h` / `src/box_link.cpp` — add passive `BoxLinkObserver` + hooks (Task 1).
- `components/fireworkcore/host_test/test_box_link.cpp` — observer tests (Task 1).
- `controller/main/event_log.h` / `event_log.cpp` — pure ring buffer (Task 2).
- `controller/host_test/{CMakeLists.txt,check.h,test_event_log.cpp}` — new host test (Task 2).
- `controller/main/web_server.h` — `Diag`, fault fields, `lastHeardMs`, extern `g_events`/`g_eventMtx` (Task 3).
- `controller/main/controller_main.cpp` — observer impl, event logging, counters, fault, diag (Task 3).
- `controller/main/CMakeLists.txt` — add `event_log.cpp` (Task 3).
- `controller/main/web_server.cpp` — `/api/events` handler + route, extended `/api/status` (Task 4).

---

### Task 1: BoxLink passive observer (fireworkcore, TDD)

**Files:**
- Modify: `components/fireworkcore/include/box_link.h`, `components/fireworkcore/src/box_link.cpp`
- Test: `components/fireworkcore/host_test/test_box_link.cpp`

**Interfaces:**
- Produces: `struct fw::BoxLinkObserver` with virtual `onFireSent(uint8_t box,uint8_t ch,uint32_t id)`, `onFireAck(uint8_t box,uint8_t ch,uint32_t id,uint32_t latencyMs)`, `onFireRetry(uint8_t box,uint8_t ch,uint32_t id,uint8_t attempt,uint8_t maxRetries)`, `onFireFailed(uint8_t box,uint8_t ch,uint32_t id)` (all default no-op); `void BoxLink::setObserver(BoxLinkObserver*)`.

- [ ] **Step 1: Write the failing tests**

Append to `components/fireworkcore/host_test/test_box_link.cpp` (after the `FakeTransport`/`countType` helpers) a recording observer and tests, and register them in `main`:
```cpp
struct RecObs : public BoxLinkObserver {
    int sent=0, ack=0, retry=0, failed=0;
    uint32_t lastLatency=0, lastAttempt=0, lastMax=0;
    void onFireSent(uint8_t, uint8_t, uint32_t) override { sent++; }
    void onFireAck(uint8_t, uint8_t, uint32_t, uint32_t lat) override { ack++; lastLatency=lat; }
    void onFireRetry(uint8_t, uint8_t, uint32_t, uint8_t a, uint8_t m) override { retry++; lastAttempt=a; lastMax=m; }
    void onFireFailed(uint8_t, uint8_t, uint32_t) override { failed++; }
};
void test_observer_fire_and_ack_latency() {
    FakeTransport t; BoxLink link(t); RecObs o; link.setObserver(&o);
    uint32_t id = link.fire(0, 3, 100);
    CHECK_EQ(o.sent, 1);
    link.onAck(id, 112);                 // 12 ms after send
    CHECK_EQ(o.ack, 1);
    CHECK_EQ((int)o.lastLatency, 12);
}
void test_observer_retry_then_fail() {
    FakeTransport t; BoxLink link(t); RecObs o; link.setObserver(&o);  // ackTimeout=120, maxRetries=3
    link.fire(0, 3, 0);
    uint32_t now = 0;
    for (int i=0;i<5;i++){ now += 130; link.tick(now); }
    CHECK_EQ(o.retry, 3);                // 3 retries
    CHECK_EQ((int)o.lastMax, 3);
    CHECK_EQ((int)o.lastAttempt, 3);
    CHECK_EQ(o.failed, 1);               // then gave up once
}
```
Add to `main()`:
```cpp
RUN(test_observer_fire_and_ack_latency);
RUN(test_observer_retry_then_fail);
```

- [ ] **Step 2: Run to verify it fails**

Run (Git Bash):
```bash
cd components/fireworkcore/host_test && cmake -B build -G Ninja -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++ && cmake --build build
```
Expected: compile error — `BoxLinkObserver` / `setObserver` not declared.

- [ ] **Step 3: Add the observer to `box_link.h`**

Before `class BoxLink`:
```cpp
// Passive observer for controller-side instrumentation (logging/telemetry).
// Hooks are best-effort and MUST NOT affect link behavior. Default no-ops.
struct BoxLinkObserver {
    virtual void onFireSent(uint8_t box, uint8_t ch, uint32_t id) {}
    virtual void onFireAck(uint8_t box, uint8_t ch, uint32_t id, uint32_t latencyMs) {}
    virtual void onFireRetry(uint8_t box, uint8_t ch, uint32_t id, uint8_t attempt, uint8_t maxRetries) {}
    virtual void onFireFailed(uint8_t box, uint8_t ch, uint32_t id) {}
    virtual ~BoxLinkObserver() {}
};
```
In `class BoxLink` public section:
```cpp
    void setObserver(BoxLinkObserver* obs) { observer_ = obs; }
```
In the private members:
```cpp
    BoxLinkObserver* observer_ = nullptr;
```

- [ ] **Step 4: Call the hooks in `box_link.cpp`**

In `fire()`, after `sendFire(boxId, channel, id);` and before `return id;`:
```cpp
    if (observer_) observer_->onFireSent(boxId, channel, id);
```
Replace `onAck()` with (uses `nowMs` now):
```cpp
void BoxLink::onAck(uint32_t responseToId, uint32_t nowMs) {
    if (pendingId_ != 0 && responseToId == pendingId_) {
        if (observer_) observer_->onFireAck(pendingBox_, pendingCh_, pendingId_, nowMs - sentAtMs_);
        pendingId_ = 0;
    }
}
```
In `tick()`, replace the retry/else block:
```cpp
    if (retries_ < cfg_.maxRetries) {
        sendFire(pendingBox_, pendingCh_, pendingId_);
        retries_++;
        sentAtMs_ = nowMs;
        if (observer_) observer_->onFireRetry(pendingBox_, pendingCh_, pendingId_, retries_, cfg_.maxRetries);
    } else {
        lastFailedId_ = pendingId_;
        if (observer_) observer_->onFireFailed(pendingBox_, pendingCh_, pendingId_);
        pendingId_ = 0;
    }
```

- [ ] **Step 5: Run tests to verify pass**

Run (Git Bash):
```bash
cd components/fireworkcore/host_test && cmake --build build && ctest --test-dir build -R test_box_link --output-on-failure
```
Expected: `test_box_link` PASS. Then full suite: `ctest --test-dir build` — all green (existing BoxLink behavior unchanged; observer is additive).

- [ ] **Step 6: Commit**

```bash
git add components/fireworkcore/include/box_link.h components/fireworkcore/src/box_link.cpp components/fireworkcore/host_test/test_box_link.cpp
git commit -m "feat(core): passive BoxLink observer (fire/ack-latency/retry/fail hooks)"
```

---

### Task 2: `EventLog` ring buffer (controller, pure, TDD)

**Files:**
- Create: `controller/main/event_log.h`, `controller/main/event_log.cpp`, `controller/host_test/CMakeLists.txt`, `controller/host_test/check.h`, `controller/host_test/test_event_log.cpp`

**Interfaces:**
- Produces: `namespace ctrl { enum Severity{SEV_INFO=0,SEV_WARN=1,SEV_ERR=2}; struct Event{uint32_t seq; uint32_t tMs; uint8_t sev; char msg[48];}; class EventLog { static const size_t CAP=32; void push(uint8_t sev,const char* msg,uint32_t tMs); size_t since(uint32_t afterSeq, Event* out, size_t maxOut) const; uint32_t lastSeq() const; }; }`. `push` assigns a monotonic seq (starting at 1), overwrites oldest when full; `since` copies events with `seq>afterSeq` oldest→newest up to `maxOut`.

- [ ] **Step 1: Header + test harness**

`controller/main/event_log.h`:
```cpp
#pragma once
#include <cstdint>
#include <cstddef>
namespace ctrl {
enum Severity : uint8_t { SEV_INFO = 0, SEV_WARN = 1, SEV_ERR = 2 };
struct Event {
    uint32_t seq;
    uint32_t tMs;
    uint8_t  sev;
    char     msg[48];
};
class EventLog {
public:
    static const size_t CAP = 32;
    void     push(uint8_t sev, const char* msg, uint32_t tMs);
    size_t   since(uint32_t afterSeq, Event* out, size_t maxOut) const;
    uint32_t lastSeq() const { return lastSeq_; }
private:
    Event    buf_[CAP];
    size_t   count_   = 0;   // valid entries (<= CAP)
    size_t   head_    = 0;   // next write index
    uint32_t lastSeq_ = 0;   // last assigned seq (0 = none)
};
} // namespace ctrl
```

`controller/host_test/check.h` (copy of the project's host-test macros):
```cpp
#pragma once
#include <cstdio>
#include <cstdint>
static int g_failures = 0;
#define CHECK(cond) do { if (!(cond)) { std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); ++g_failures; } } while (0)
#define CHECK_EQ(a, b) do { long long _a=(long long)(a), _b=(long long)(b); if (_a != _b) { std::printf("FAIL %s:%d: %s(%lld) == %s(%lld)\n", __FILE__, __LINE__, #a,_a,#b,_b); ++g_failures; } } while (0)
#define RUN(test) do { std::printf("RUN  %s\n", #test); test(); } while (0)
#define REPORT() ( g_failures == 0 ? (std::printf("OK\n"), 0) : (std::printf("%d FAILURE(S)\n", g_failures), 1) )
```

`controller/host_test/CMakeLists.txt`:
```cmake
cmake_minimum_required(VERSION 3.16)
project(controller_host_test CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
enable_testing()
include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../main)
add_executable(test_event_log test_event_log.cpp ../main/event_log.cpp)
add_test(NAME test_event_log COMMAND test_event_log)
```

- [ ] **Step 2: Write the failing tests**

`controller/host_test/test_event_log.cpp`:
```cpp
#include "check.h"
#include "event_log.h"
using namespace ctrl;

void test_push_seq_and_since() {
    EventLog log;
    log.push(SEV_INFO, "a", 10);
    log.push(SEV_WARN, "b", 20);
    log.push(SEV_ERR,  "c", 30);
    CHECK_EQ((int)log.lastSeq(), 3);
    Event out[8];
    size_t n = log.since(0, out, 8);
    CHECK_EQ((int)n, 3);
    CHECK_EQ((int)out[0].seq, 1);           // oldest first
    CHECK_EQ((int)out[2].seq, 3);
    CHECK_EQ((int)out[2].sev, (int)SEV_ERR);
    // incremental: only newer than seq 2
    n = log.since(2, out, 8);
    CHECK_EQ((int)n, 1);
    CHECK_EQ((int)out[0].seq, 3);
    // nothing newer than lastSeq
    CHECK_EQ((int)log.since(log.lastSeq(), out, 8), 0);
}
void test_overflow_drops_oldest() {
    EventLog log;
    for (int i = 0; i < (int)EventLog::CAP + 5; ++i) log.push(SEV_INFO, "x", (uint32_t)i);
    CHECK_EQ((int)log.lastSeq(), (int)EventLog::CAP + 5);   // 37
    Event out[64];
    size_t n = log.since(0, out, 64);
    CHECK_EQ((int)n, (int)EventLog::CAP);                   // capped at 32
    CHECK_EQ((int)out[0].seq, 6);                           // oldest kept = 37-32+1
    CHECK_EQ((int)out[n-1].seq, 37);                        // newest
}
void test_maxout_cap_returns_oldest_first() {
    EventLog log;
    for (int i = 1; i <= 5; ++i) log.push(SEV_INFO, "y", 0);
    Event out[2];
    size_t n = log.since(0, out, 2);
    CHECK_EQ((int)n, 2);
    CHECK_EQ((int)out[0].seq, 1);
    CHECK_EQ((int)out[1].seq, 2);
}
int main() {
    RUN(test_push_seq_and_since);
    RUN(test_overflow_drops_oldest);
    RUN(test_maxout_cap_returns_oldest_first);
    return REPORT();
}
```

Run (Git Bash):
```bash
cd controller/host_test && cmake -B build -G Ninja -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++ && cmake --build build
```
Expected: link error — `EventLog::push`/`since` undefined.

- [ ] **Step 3: Implement `event_log.cpp`**

`controller/main/event_log.cpp`:
```cpp
#include "event_log.h"
#include <cstring>
namespace ctrl {

void EventLog::push(uint8_t sev, const char* msg, uint32_t tMs) {
    Event& e = buf_[head_];
    e.seq = ++lastSeq_;
    e.tMs = tMs;
    e.sev = sev;
    std::strncpy(e.msg, msg ? msg : "", sizeof(e.msg) - 1);
    e.msg[sizeof(e.msg) - 1] = '\0';
    head_ = (head_ + 1) % CAP;
    if (count_ < CAP) ++count_;
}

size_t EventLog::since(uint32_t afterSeq, Event* out, size_t maxOut) const {
    size_t written = 0;
    // Walk valid entries oldest -> newest.
    for (size_t i = 0; i < count_ && written < maxOut; ++i) {
        size_t idx = (head_ + CAP - count_ + i) % CAP;
        if (buf_[idx].seq > afterSeq) out[written++] = buf_[idx];
    }
    return written;
}

} // namespace ctrl
```

- [ ] **Step 4: Run tests to verify pass**

Run (Git Bash):
```bash
cd controller/host_test && cmake --build build && ctest --test-dir build --output-on-failure
```
Expected: `test_event_log` PASS (3/3).

- [ ] **Step 5: Commit**

```bash
git add controller/main/event_log.h controller/main/event_log.cpp controller/host_test/
git commit -m "feat(ctrl): host-tested EventLog ring buffer (seq, since, overflow)"
```

---

### Task 3: Controller wiring — events, counters, fault (controller, build)

**Files:**
- Modify: `controller/main/web_server.h`, `controller/main/controller_main.cpp`, `controller/main/CMakeLists.txt`

**Interfaces:**
- Consumes: `fw::BoxLinkObserver`/`setObserver` (Task 1); `ctrl::EventLog`/`Event`/severities (Task 2).
- Produces: `struct Diag {...}` and `StatusSnapshot.diag`, `StatusSnapshot.faultCode` (0=none,1=estop,2=link,3=fire), per-box `lastHeardMs`; globals `extern ctrl::EventLog g_events; extern SemaphoreHandle_t g_eventMtx;`.

- [ ] **Step 1: Extend `web_server.h`**

Add near the top includes:
```cpp
#include "freertos/semphr.h"
#include "event_log.h"
```
Add a `Diag` struct and extend `BoxTelemetry` + `StatusSnapshot`:
```cpp
struct Diag {
    volatile uint32_t uptimeMs  = 0;
    volatile uint32_t freeHeap  = 0;
    volatile uint32_t apClients = 0;
    volatile uint32_t fired     = 0;
    volatile uint32_t acked     = 0;
    volatile uint32_t failed    = 0;
    volatile uint32_t retries   = 0;
    volatile uint32_t lastAckMs = 0;
};
```
In `BoxTelemetry` add: `volatile uint32_t lastHeardMs = 0;`
In `StatusSnapshot` add after `lastFailedBox`:
```cpp
    Diag             diag;
    volatile uint8_t faultCode;   // 0=none, 1=estop, 2=link, 3=fire-failed
```
At the bottom with the other externs:
```cpp
extern ctrl::EventLog     g_events;
extern SemaphoreHandle_t  g_eventMtx;
```

- [ ] **Step 2: Add `event_log.cpp` to the build**

In `controller/main/CMakeLists.txt`, add `"event_log.cpp"` to the `SRCS` list (keep all existing entries).

- [ ] **Step 3: Define globals + log helper + observer in `controller_main.cpp`**

Add includes near the top:
```cpp
#include <cstdarg>
#include "esp_heap_caps.h"
```
After the existing global definitions (`g_status` etc.):
```cpp
ctrl::EventLog    g_events;
SemaphoreHandle_t g_eventMtx = nullptr;

// Best-effort: format + push under the event mutex. Called from control loop only.
static void log_evt(uint8_t sev, const char* fmt, ...) {
    char m[48];
    va_list ap; va_start(ap, fmt); vsnprintf(m, sizeof(m), fmt, ap); va_end(ap);
    uint32_t t = (uint32_t)(esp_timer_get_time() / 1000);
    if (g_eventMtx && xSemaphoreTake(g_eventMtx, pdMS_TO_TICKS(5)) == pdTRUE) {
        g_events.push(sev, m, t);
        xSemaphoreGive(g_eventMtx);
    }
}

// Sticky fault flags maintained by the loop/observer.
static bool g_fireFailed = false;
static bool g_estop      = false;

// Passive BoxLink observer: logs fire lifecycle + bumps diag counters.
struct CtrlObserver : public fw::BoxLinkObserver {
    void onFireSent(uint8_t b, uint8_t c, uint32_t id) override {
        g_status.diag.fired++;
        log_evt(ctrl::SEV_INFO, "FIRE ch%u -> box%u (id%lu)", c, b, (unsigned long)id);
    }
    void onFireAck(uint8_t /*b*/, uint8_t /*c*/, uint32_t id, uint32_t lat) override {
        g_status.diag.acked++; g_status.diag.lastAckMs = lat; g_fireFailed = false;
        log_evt(ctrl::SEV_INFO, "ACK id%lu (%lums)", (unsigned long)id, (unsigned long)lat);
    }
    void onFireRetry(uint8_t /*b*/, uint8_t c, uint32_t /*id*/, uint8_t a, uint8_t m) override {
        g_status.diag.retries++;
        log_evt(ctrl::SEV_WARN, "RETRY ch%u %u/%u", c, a, m);
    }
    void onFireFailed(uint8_t /*b*/, uint8_t c, uint32_t /*id*/) override {
        g_status.diag.failed++; g_fireFailed = true;
        log_evt(ctrl::SEV_ERR, "FIRE FAILED ch%u (no ACK)", c);
    }
};
```

- [ ] **Step 4: Wire it into `app_main`**

After `g_cmdQueue = xQueueCreate(...)` / before `web.start`:
```cpp
    g_eventMtx = xSemaphoreCreateMutex();
    configASSERT(g_eventMtx);
    static CtrlObserver obs;
    link.setObserver(&obs);
    log_evt(ctrl::SEV_INFO, "controller up, AP %s", ctrl::AP_SSID);
```
In the command-drain `switch`, add logging (the FIRE case is already covered by `onFireSent`, so do NOT log FIRE here):
```cpp
                case CmdType::ARM:
                    runner.arm(now); g_estop = false; log_evt(ctrl::SEV_WARN, "ARM -> box0"); break;
                case CmdType::DISARM:
                    runner.disarm(now); log_evt(ctrl::SEV_INFO, "DISARM"); break;
                case CmdType::ESTOP:
                    runner.estop(now); g_estop = true; log_evt(ctrl::SEV_ERR, "ESTOP"); break;
                case CmdType::STOP:
                    runner.stopSequence(now); log_evt(ctrl::SEV_INFO, "SEQ stop"); break;
                case CmdType::RUN:
                    runner.loadSequence(g_runSteps, g_runCount);
                    runner.startSequence(now);
                    log_evt(ctrl::SEV_INFO, "SEQ start (%u cues)", (unsigned)g_runCount);
                    break;
```
(Keep `HEARTBEAT` and `FIRE` cases as they are — FIRE still calls `runner.fireManual(cmd.boxId, cmd.channel, now);`.)

Add sequence-done + link up/down detection. Before the `while(true)` loop add:
```cpp
    static bool prevSeq = false;
    static bool prevLink[2] = {false, false};
```
Replace the per-box `linkAlive` loop with one that logs transitions + sets `lastHeardMs`:
```cpp
        for (int b = 0; b < 2; b++) {
            bool alive = (lastStatusMs[b] != 0) && ((now - lastStatusMs[b]) < 1500);
            if (alive != prevLink[b]) {
                log_evt(alive ? ctrl::SEV_INFO : ctrl::SEV_ERR,
                        "box%d link %s", b, alive ? "up" : "lost");
                prevLink[b] = alive;
            }
            g_status.boxes[b].linkAlive  = alive;
            g_status.boxes[b].lastHeardMs = (lastStatusMs[b] != 0) ? (now - lastStatusMs[b]) : 0xFFFFFFFF;
        }
```
After `runner.tick(now);` extend the snapshot update:
```cpp
        bool seqNow = runner.sequenceRunning();
        if (prevSeq && !seqNow) log_evt(ctrl::SEV_INFO, "SEQ done");
        prevSeq = seqNow;

        g_status.armed      = runner.armed();
        g_status.seqRunning = seqNow;
        g_status.lastFailedBox = 0xFF;

        // Diagnostics.
        g_status.diag.uptimeMs = now;
        g_status.diag.freeHeap = (uint32_t)esp_get_free_heap_size();
        wifi_sta_list_t stalist;
        g_status.diag.apClients = (esp_wifi_ap_get_sta_list(&stalist) == ESP_OK) ? (uint32_t)stalist.num : 0;

        // Fault code: estop > fire-failed > any configured box link down.
        bool anyLinkDown = false;
        for (int b = 0; b < 2; b++) {
            bool configured = false;
            for (int i = 0; i < 6; i++) if (ctrl::BOX_MAC[b][i]) { configured = true; break; }
            if (configured && !g_status.boxes[b].linkAlive) anyLinkDown = true;
        }
        g_status.faultCode = g_estop ? 1 : (g_fireFailed ? 3 : (anyLinkDown ? 2 : 0));
```

- [ ] **Step 5: Build**

Run (PowerShell):
```powershell
& 'C:\esp\v6.0.1\esp-idf\export.ps1'; Set-Location 'C:\Users\zombo\Desktop\Programming\RemoteFireworkCueSystem\RemoteFireworkCueSystem\controller'; idf.py build
```
Expected: `Project build complete.`

- [ ] **Step 6: Commit**

```bash
git add controller/main/web_server.h controller/main/controller_main.cpp controller/main/CMakeLists.txt
git commit -m "feat(ctrl): log events + diag counters + fault code in the control loop"
```

---

### Task 4: API — `/api/events` + extended `/api/status` (controller, build)

**Files:**
- Modify: `controller/main/web_server.cpp`

**Interfaces:**
- Consumes: `g_events`/`g_eventMtx` (Task 3), `g_status.diag`/`faultCode`/`boxes[].lastHeardMs` (Task 3).
- Produces: `GET /api/events?since=<seq>` → `{"lastSeq":int,"events":[{"seq","t","sev","msg"},...]}`; `/api/status` with `diag`, `fault`, per-box `lastHeardMs`.

- [ ] **Step 1: Add the `/api/events` handler**

In `controller/main/web_server.cpp`, near `handle_status`, add (the file already includes the handler patterns; add `#include "event_log.h"` and `#include <cstdlib>` at the top if not present):
```cpp
static esp_err_t handle_events(httpd_req_t* req) {
    // Parse ?since=<seq> (default 0 = whole buffer).
    uint32_t since = 0;
    char q[40];
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK) {
        char v[16];
        if (httpd_query_key_value(q, "since", v, sizeof(v)) == ESP_OK) since = (uint32_t)strtoul(v, nullptr, 10);
    }

    static ctrl::Event evs[ctrl::EventLog::CAP];   // static: handler task, serialized by httpd
    size_t n = 0;
    uint32_t lastSeq = 0;
    if (xSemaphoreTake(g_eventMtx, pdMS_TO_TICKS(20)) == pdTRUE) {
        n = g_events.since(since, evs, ctrl::EventLog::CAP);
        lastSeq = g_events.lastSeq();
        xSemaphoreGive(g_eventMtx);
    }

    char buf[2600];
    int p = snprintf(buf, sizeof(buf), "{\"lastSeq\":%lu,\"events\":[", (unsigned long)lastSeq);
    for (size_t i = 0; i < n; ++i) {
        size_t rem = (p < (int)sizeof(buf)) ? sizeof(buf) - (size_t)p : 0;
        // msg is controller-authored (no unescaped quotes/backslashes), safe to inline.
        p += snprintf(buf + p, rem, "%s{\"seq\":%lu,\"t\":%lu,\"sev\":%u,\"msg\":\"%s\"}",
                      i ? "," : "", (unsigned long)evs[i].seq, (unsigned long)evs[i].tMs,
                      (unsigned)evs[i].sev, evs[i].msg);
    }
    size_t rem2 = (p < (int)sizeof(buf)) ? sizeof(buf) - (size_t)p : 0;
    p += snprintf(buf + p, rem2, "]}");

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
}
```

- [ ] **Step 2: Register the route**

In `WebServer::start`, add to the `routes[]` array (after `/api/status`):
```cpp
        { "/api/events",    HTTP_GET,  handle_events    },
```
(`max_uri_handlers` is already 16 — room for the 10th route.)

- [ ] **Step 3: Extend `/api/status` JSON**

In `handle_status`, enlarge `char buf[420];` to `char buf[640];`. Snapshot the diag/fault before building:
```cpp
    Diag    d        = g_status.diag;
    uint8_t fault    = g_status.faultCode;
    const char* fmsg = (fault==1)?"ESTOP":(fault==2)?"box link lost":(fault==3)?"fire failed":"";
```
Add `lastHeardMs` to each per-box object — change the per-box `snprintf` format string and args to include it:
```cpp
        n += snprintf(buf + n, rem,
            "%s{\"id\":%d,\"linkAlive\":%s,\"rssi\":%d,\"state\":%u,"
            "\"firedBitmap\":%u,\"lastFired\":%d,\"lastHeardMs\":%lu}",
            firstBox ? "" : ",", b,
            box.linkAlive ? "true" : "false",
            (int)box.rssi,
            (unsigned)box.state,
            (unsigned)box.firedBitmap,
            (box.lastFiredChannel == 0xFF) ? -1 : (int)box.lastFiredChannel,
            (unsigned long)box.lastHeardMs);
```
Replace the closing `"]}"` snprintf with one that appends `diag` + `fault`:
```cpp
    size_t rem2 = (n < (int)sizeof(buf)) ? sizeof(buf) - (size_t)n : 0;
    n += snprintf(buf + n, rem2,
        "],\"diag\":{\"uptimeMs\":%lu,\"freeHeap\":%lu,\"apClients\":%lu,"
        "\"fired\":%lu,\"acked\":%lu,\"failed\":%lu,\"retries\":%lu,\"lastAckMs\":%lu},"
        "\"fault\":{\"active\":%s,\"msg\":\"%s\"}}",
        (unsigned long)d.uptimeMs, (unsigned long)d.freeHeap, (unsigned long)d.apClients,
        (unsigned long)d.fired, (unsigned long)d.acked, (unsigned long)d.failed,
        (unsigned long)d.retries, (unsigned long)d.lastAckMs,
        fault ? "true" : "false", fmsg);
```

- [ ] **Step 4: Build, flash, sanity-check**

Run (PowerShell): build as in Task 3, then flash the controller (COM11; CP210x → custom reset env + BOOT-hold if `0x13`):
```powershell
$env:ESPTOOL_CUSTOM_RESET_SEQUENCE="D0|R1|W0.6|D1|R0|W0.4|D0"
Set-Location build; python -m esptool --chip esp32 -p COM11 -b 460800 --before default-reset --after hard-reset write-flash "@flash_args"
```
Expected: `Project build complete.` + `Hash of data verified.` Optional live check: from a device on the `FireControl` AP, open `http://192.168.4.1/api/events` — expect `{"lastSeq":N,"events":[{...,"msg":"controller up, AP FireControl"}, ...]}`; and `http://192.168.4.1/api/status` now includes `diag` + `fault` + per-box `lastHeardMs`.

- [ ] **Step 5: Commit**

```bash
git add controller/main/web_server.cpp
git commit -m "feat(ctrl): GET /api/events + diag/fault/lastHeardMs on /api/status"
```

---

## Self-Review Notes

- **Spec coverage:** event log (Tasks 2,3,4), severities (Task 2), fire/ack/retry/fail + latency via observer (Task 1,3), command/seq/link events (Task 3), diag counters incl. uptime/heap/clients/packet counts/latency (Task 3,4), per-box lastHeard (Task 3,4), fault summary (Task 3,4), `/api/events` incremental by `since` (Task 4), backward-compatible `/api/status` (Task 4), read-only/passive (Task 1 hooks are no-op-default + don't alter flow; Task 3 only reads runner state), mutex-guarded ring (Tasks 3,4). Out-of-scope items (persistence, streaming, touch control) correctly absent.
- **Type consistency:** `BoxLinkObserver` hook signatures identical in Task 1 def and Task 3 impl; `EventLog`/`Event`/`since`/`lastSeq`/`CAP`/severities identical across Tasks 2/3/4; `Diag` fields and `faultCode` encoding identical in Task 3 (def) and Task 4 (JSON).
- **Concurrency:** event ring guarded by `g_eventMtx` on both writer (`log_evt`, control loop) and reader (`handle_events`, httpd task); diag fields are volatile scalars snapshotted in the handler. No core firing/timing logic changed.
