# CYD ESP-NOW Display Transport Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Switch the CYD dashboard's data source from Wi-Fi-associate + HTTP polling to ESP-NOW broadcast, eliminating the PMF/SA-Query disconnect loop, with no UI change.

**Architecture:** The controller broadcasts a `DisplayStatusPacket` (~1 Hz) + `DisplayEventPacket` (per new event) over ESP-NOW. The CYD runs Wi-Fi STA without ever associating (channel 1, esp_now only), decodes the frames into the existing `StatusModel`/`LogEv[]`, and renders the existing 3-page dashboard. The controller keeps its AP + HTTP for the phone/website.

**Tech Stack:** ESP-IDF v6.0.1, ESP-NOW, portable `fireworkcore` protocol header (host-tested), LVGL (unchanged).

## Global Constraints

- **ESP-IDF builds run only in PowerShell** (`& 'C:\esp\v6.0.1\esp-idf\export.ps1'` then `idf.py build`). Host tests in **Git Bash with g++ forced** (`cmake -B build -G Ninja -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++`).
- **Strictly additive on the controller:** do NOT change the firing path, box ESP-NOW unicast, retry, or the dead-man. The display broadcast is extra output only.
- **CYD is read-only:** ESP-NOW is inbound display data; no control is ever sent.
- **No application CRC on display packets** — ESP-NOW MAC-layer CRC already validates frames; the receiver validates by **`type` byte + exact `len`**. Packed structs (`#pragma pack(1)`).
- Display packet structs live once in `components/fireworkcore/include/protocol.h`; the CYD project adds that dir to its include path (it does NOT call the CRC helpers).
- Channel is **1** (the controller AP's channel); the CYD pins it with `esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE)`.
- `controllerReachable` on the CYD = "a `DisplayStatusPacket` arrived in the last 3000 ms."

---

## File Structure
- `components/fireworkcore/include/protocol.h` — add `DISP_STATUS`/`DISP_EVENT` + the two packet structs (Task 1).
- `components/fireworkcore/host_test/test_protocol.cpp` — packing/size test (Task 1).
- `controller/main/espnow_tx.{h,cpp}` — broadcast peer + `sendDisplayStatus`/`sendDisplayEvent` (Task 2).
- `controller/main/controller_main.cpp` — broadcast status ~1 Hz + new events (Task 2).
- `cyd-dashboard/main/display_decode.{h,cpp}` — pure decode helpers (Task 3).
- `cyd-dashboard/host_test/{CMakeLists.txt,test_display_decode.cpp}` — host test (Task 3).
- `cyd-dashboard/main/espnow_rx.{h,cpp}` — STA-no-connect + esp_now recv + shared model/event ring (Task 4).
- `cyd-dashboard/main/main.cpp`, `cyd-dashboard/main/CMakeLists.txt` — wire-up; drop wifi/http sources (Task 4).

---

### Task 1: Display packet types (protocol, host TDD)

**Files:**
- Modify: `components/fireworkcore/include/protocol.h`, `components/fireworkcore/host_test/test_protocol.cpp`

**Interfaces:**
- Produces: `fw::MsgType::DISP_STATUS = 8`, `fw::MsgType::DISP_EVENT = 9`; `struct fw::DisplayStatusPacket { uint8_t type, boxState, boxLinkAlive; int8_t rssi; uint16_t firedBitmap; int8_t lastFired; uint8_t seqRunning, faultCode; uint32_t uptimeMs, freeHeap, apClients, fired, acked, failed, retries, lastAckMs, boxLastHeardMs; };` (packed, 45 bytes); `struct fw::DisplayEventPacket { uint8_t type, sev; uint32_t seq; char msg[48]; };` (packed, 54 bytes).

- [ ] **Step 1: Write the failing test**

Append to `components/fireworkcore/host_test/test_protocol.cpp` and register in `main`:
```cpp
void test_display_packet_sizes() {
    // Packed wire sizes must be stable and fit the 250-byte ESP-NOW limit.
    CHECK_EQ((int)sizeof(fw::DisplayStatusPacket), 45);
    CHECK_EQ((int)sizeof(fw::DisplayEventPacket), 54);
    CHECK((int)sizeof(fw::DisplayStatusPacket) <= 250);
    CHECK((int)sizeof(fw::DisplayEventPacket) <= 250);
    fw::DisplayStatusPacket s{};
    s.type = (uint8_t)fw::MsgType::DISP_STATUS;
    CHECK_EQ((int)s.type, 8);
    fw::DisplayEventPacket e{};
    e.type = (uint8_t)fw::MsgType::DISP_EVENT;
    CHECK_EQ((int)e.type, 9);
}
```
Add `RUN(test_display_packet_sizes);` to `main()`.

- [ ] **Step 2: Run to verify it fails**

Run (Git Bash):
```bash
cd components/fireworkcore/host_test && cmake -B build -G Ninja -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++ && cmake --build build
```
Expected: compile error — `DisplayStatusPacket`/`DISP_STATUS` undeclared.

- [ ] **Step 3: Add the types in `protocol.h`**

Extend the enum:
```cpp
enum class MsgType : uint8_t { FIRE=1, ACK=2, ARM=3, DISARM=4, HEARTBEAT=5, ESTOP=6, STATUS=7, DISP_STATUS=8, DISP_EVENT=9 };
```
Inside the `#pragma pack(push, 1)` … `#pragma pack(pop)` region, after `StatusPacket`:
```cpp
// Controller -> display (CYD) broadcast frames. No CRC field: ESP-NOW MAC-layer
// CRC validates the frame; the receiver checks type + exact length.
struct DisplayStatusPacket {
    uint8_t  type;          // MsgType::DISP_STATUS
    uint8_t  boxState;      // 0 = SAFE, 1 = ARMED
    uint8_t  boxLinkAlive;  // 0/1
    int8_t   rssi;
    uint16_t firedBitmap;
    int8_t   lastFired;     // -1 = none
    uint8_t  seqRunning;    // 0/1
    uint8_t  faultCode;     // 0 none, 1 estop, 2 link, 3 fire-failed
    uint32_t uptimeMs;
    uint32_t freeHeap;
    uint32_t apClients;
    uint32_t fired;
    uint32_t acked;
    uint32_t failed;
    uint32_t retries;
    uint32_t lastAckMs;
    uint32_t boxLastHeardMs;
};
struct DisplayEventPacket {
    uint8_t  type;          // MsgType::DISP_EVENT
    uint8_t  sev;           // 0 info / 1 warn / 2 err
    uint32_t seq;
    char     msg[48];
};
```

- [ ] **Step 4: Run tests to verify pass**

Run (Git Bash):
```bash
cd components/fireworkcore/host_test && cmake --build build && ctest --test-dir build -R test_protocol --output-on-failure
```
Expected: `test_protocol` PASS. Then `ctest --test-dir build` — all green.

- [ ] **Step 5: Commit**

```bash
git add components/fireworkcore/include/protocol.h components/fireworkcore/host_test/test_protocol.cpp
git commit -m "feat(core): DisplayStatusPacket/DisplayEventPacket ESP-NOW frames"
```

---

### Task 2: Controller broadcasts display frames (build)

**Files:**
- Modify: `controller/main/espnow_tx.h`, `controller/main/espnow_tx.cpp`, `controller/main/controller_main.cpp`

**Interfaces:**
- Consumes: `fw::DisplayStatusPacket`/`DisplayEventPacket` (Task 1); `g_status`/`g_status.diag`/`g_status.faultCode` and `g_events`/`g_eventMtx` (existing).
- Produces: `void EspNowTransport::sendDisplayStatus(const fw::DisplayStatusPacket&)`, `void EspNowTransport::sendDisplayEvent(const fw::DisplayEventPacket&)` (broadcast to FF:FF:FF:FF:FF:FF).

- [ ] **Step 1: Declare the senders + broadcast peer**

In `controller/main/espnow_tx.h`, in the public section after `receiveStatus`:
```cpp
    void sendDisplayStatus(const fw::DisplayStatusPacket& p);
    void sendDisplayEvent(const fw::DisplayEventPacket& e);
```

- [ ] **Step 2: Add the broadcast peer + senders in `espnow_tx.cpp`**

In `begin()`, after the box-peer loop and before `esp_now_register_recv_cb`:
```cpp
    // Broadcast peer for display frames (CYD listens; no association needed).
    esp_now_peer_info_t bpeer = {};
    memset(bpeer.peer_addr, 0xFF, 6);
    bpeer.channel = ctrl::WIFI_CHAN;
    bpeer.ifidx   = WIFI_IF_AP;
    bpeer.encrypt = false;
    err = esp_now_add_peer(&bpeer);
    if (err != ESP_OK) { ESP_LOGE(TAG, "add broadcast peer: %s", esp_err_to_name(err)); return err; }
```
At the end of the file (before the closing of the TU), add:
```cpp
static const uint8_t kBroadcast[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
void EspNowTransport::sendDisplayStatus(const fw::DisplayStatusPacket& p) {
    esp_now_send(kBroadcast, reinterpret_cast<const uint8_t*>(&p), sizeof(p));
}
void EspNowTransport::sendDisplayEvent(const fw::DisplayEventPacket& e) {
    esp_now_send(kBroadcast, reinterpret_cast<const uint8_t*>(&e), sizeof(e));
}
```

- [ ] **Step 3: Broadcast status + events from the control loop**

In `controller/main/controller_main.cpp`, add before the `while (true)` loop:
```cpp
    static uint32_t lastDispMs = 0;
    static uint32_t lastDispSeq = 0;
```
Inside the loop, after the existing diag/fault computation (after `g_status.faultCode = ...`) and before `vTaskDelay`:
```cpp
        // --- Broadcast display status (~1 Hz) for ESP-NOW display panels (CYD) ---
        if (now - lastDispMs >= 1000) {
            lastDispMs = now;
            fw::DisplayStatusPacket dp{};
            dp.type           = (uint8_t)fw::MsgType::DISP_STATUS;
            dp.boxState       = g_status.boxes[0].state;
            dp.boxLinkAlive   = g_status.boxes[0].linkAlive ? 1 : 0;
            dp.rssi           = g_status.boxes[0].rssi;
            dp.firedBitmap    = g_status.boxes[0].firedBitmap;
            dp.lastFired      = (g_status.boxes[0].lastFiredChannel == 0xFF) ? -1 : (int8_t)g_status.boxes[0].lastFiredChannel;
            dp.seqRunning     = g_status.seqRunning ? 1 : 0;
            dp.faultCode      = g_status.faultCode;
            dp.uptimeMs       = g_status.diag.uptimeMs;
            dp.freeHeap       = g_status.diag.freeHeap;
            dp.apClients      = g_status.diag.apClients;
            dp.fired          = g_status.diag.fired;
            dp.acked          = g_status.diag.acked;
            dp.failed         = g_status.diag.failed;
            dp.retries        = g_status.diag.retries;
            dp.lastAckMs      = g_status.diag.lastAckMs;
            dp.boxLastHeardMs = g_status.boxes[0].lastHeardMs;
            tx.sendDisplayStatus(dp);
        }
        // --- Broadcast any new events (bounded per tick) ---
        if (g_eventMtx && xSemaphoreTake(g_eventMtx, pdMS_TO_TICKS(5)) == pdTRUE) {
            ctrl::Event evs[4];
            size_t en = g_events.since(lastDispSeq, evs, 4);
            xSemaphoreGive(g_eventMtx);
            for (size_t i = 0; i < en; ++i) {
                fw::DisplayEventPacket ep{};
                ep.type = (uint8_t)fw::MsgType::DISP_EVENT;
                ep.sev  = evs[i].sev;
                ep.seq  = evs[i].seq;
                std::strncpy(ep.msg, evs[i].msg, sizeof(ep.msg) - 1);
                tx.sendDisplayEvent(ep);
                lastDispSeq = evs[i].seq;
            }
        }
```
Ensure `#include <cstring>` is present near the top of `controller_main.cpp` (add if missing).

- [ ] **Step 4: Build**

Run (PowerShell):
```powershell
& 'C:\esp\v6.0.1\esp-idf\export.ps1'; Set-Location 'C:\Users\zombo\Desktop\Programming\RemoteFireworkCueSystem\RemoteFireworkCueSystem\controller'; idf.py build
```
Expected: `Project build complete.` (Do NOT flash — Task 4 flashes both boards.)

- [ ] **Step 5: Commit**

```bash
git add controller/main/espnow_tx.h controller/main/espnow_tx.cpp controller/main/controller_main.cpp
git commit -m "feat(ctrl): broadcast DisplayStatus/Event over ESP-NOW for display panels"
```

---

### Task 3: CYD pure decode helpers (host TDD)

**Files:**
- Create: `cyd-dashboard/main/display_decode.h`, `cyd-dashboard/main/display_decode.cpp`, `cyd-dashboard/host_test/test_display_decode.cpp`
- Modify: `cyd-dashboard/host_test/CMakeLists.txt`

**Interfaces:**
- Consumes: `fw::DisplayStatusPacket`/`DisplayEventPacket` (Task 1, via protocol.h); `StatusModel`/`LogEv` (status_model.h).
- Produces: `void applyDisplayStatus(const fw::DisplayStatusPacket& p, StatusModel& m)` (maps fields; sets `boxPresent=true`, `faultActive`/`faultMsg` from `faultCode`; does NOT touch `controllerReachable`/`lastUpdateMs`); `void toLogEv(const fw::DisplayEventPacket& p, LogEv& e)`.

- [ ] **Step 1: Header + decode impl**

`cyd-dashboard/main/display_decode.h`:
```cpp
#pragma once
#include "status_model.h"
#include "protocol.h"
void applyDisplayStatus(const fw::DisplayStatusPacket& p, StatusModel& m);
void toLogEv(const fw::DisplayEventPacket& p, LogEv& e);
```

`cyd-dashboard/main/display_decode.cpp`:
```cpp
#include "display_decode.h"
#include <cstring>

void applyDisplayStatus(const fw::DisplayStatusPacket& p, StatusModel& m) {
    m.boxPresent     = true;                 // the controller broadcasts box 0's data
    m.boxArmed       = (p.boxState == 1);
    m.boxLinkAlive   = (p.boxLinkAlive != 0);
    m.rssi           = p.rssi;
    m.firedBitmap    = p.firedBitmap;
    m.lastFired      = p.lastFired;
    m.seqRunning     = (p.seqRunning != 0);
    m.boxLastHeardMs = p.boxLastHeardMs;
    m.diag.uptimeMs  = p.uptimeMs;
    m.diag.freeHeap  = p.freeHeap;
    m.diag.apClients = p.apClients;
    m.diag.fired     = p.fired;
    m.diag.acked     = p.acked;
    m.diag.failed    = p.failed;
    m.diag.retries   = p.retries;
    m.diag.lastAckMs = p.lastAckMs;
    m.faultActive    = (p.faultCode != 0);
    const char* fm = (p.faultCode == 1) ? "ESTOP" :
                     (p.faultCode == 2) ? "box link lost" :
                     (p.faultCode == 3) ? "fire failed" : "";
    std::strncpy(m.faultMsg, fm, sizeof(m.faultMsg) - 1);
    m.faultMsg[sizeof(m.faultMsg) - 1] = '\0';
}

void toLogEv(const fw::DisplayEventPacket& p, LogEv& e) {
    e.seq = (int)p.seq;
    e.sev = (int)p.sev;
    std::strncpy(e.msg, p.msg, sizeof(e.msg) - 1);
    e.msg[sizeof(e.msg) - 1] = '\0';
}
```

- [ ] **Step 2: Write the failing test + CMake**

`cyd-dashboard/host_test/test_display_decode.cpp`:
```cpp
#include "check.h"
#include "display_decode.h"
#include <cstring>

void test_apply_display_status() {
    fw::DisplayStatusPacket p{};
    p.type = (uint8_t)fw::MsgType::DISP_STATUS;
    p.boxState = 1; p.boxLinkAlive = 1; p.rssi = -42;
    p.firedBitmap = 5; p.lastFired = 2; p.seqRunning = 1; p.faultCode = 2;
    p.uptimeMs = 61000; p.freeHeap = 142000; p.apClients = 1;
    p.fired = 3; p.acked = 3; p.failed = 1; p.retries = 2; p.lastAckMs = 8;
    p.boxLastHeardMs = 250;
    StatusModel m;
    applyDisplayStatus(p, m);
    CHECK(m.boxPresent);
    CHECK(m.boxArmed);
    CHECK(m.boxLinkAlive);
    CHECK_EQ(m.rssi, -42);
    CHECK_EQ((int)m.firedBitmap, 5);
    CHECK_EQ(m.lastFired, 2);
    CHECK(m.seqRunning);
    CHECK_EQ((int)m.diag.failed, 1);
    CHECK_EQ((int)m.diag.lastAckMs, 8);
    CHECK_EQ((int)m.boxLastHeardMs, 250);
    CHECK(m.faultActive);
    CHECK(std::strcmp(m.faultMsg, "box link lost") == 0);
}
void test_apply_display_status_safe_nofault() {
    fw::DisplayStatusPacket p{};
    p.boxState = 0; p.boxLinkAlive = 0; p.faultCode = 0; p.lastFired = -1;
    StatusModel m;
    applyDisplayStatus(p, m);
    CHECK(!m.boxArmed);
    CHECK(!m.boxLinkAlive);
    CHECK(!m.faultActive);
    CHECK_EQ((int)std::strlen(m.faultMsg), 0);
    CHECK_EQ(m.lastFired, -1);
}
void test_to_logev() {
    fw::DisplayEventPacket p{};
    p.seq = 7; p.sev = 2;
    std::strncpy(p.msg, "FIRE FAILED ch3", sizeof(p.msg) - 1);
    LogEv e;
    toLogEv(p, e);
    CHECK_EQ(e.seq, 7);
    CHECK_EQ(e.sev, 2);
    CHECK(std::strcmp(e.msg, "FIRE FAILED ch3") == 0);
}
int main() {
    RUN(test_apply_display_status);
    RUN(test_apply_display_status_safe_nofault);
    RUN(test_to_logev);
    return REPORT();
}
```

Replace `cyd-dashboard/host_test/CMakeLists.txt` with (drops the removed status_parse test, adds the decode test + the fireworkcore include path):
```cmake
cmake_minimum_required(VERSION 3.16)
project(cyd_host_test CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
enable_testing()
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../main
    ${CMAKE_CURRENT_SOURCE_DIR}/../../components/fireworkcore/include)
add_executable(test_display_decode test_display_decode.cpp ../main/display_decode.cpp)
add_test(NAME test_display_decode COMMAND test_display_decode)
```
Delete the old `cyd-dashboard/host_test/test_status_parse.cpp`.

- [ ] **Step 3: Run to verify (RED then GREEN)**

Run (Git Bash):
```bash
cd cyd-dashboard/host_test && rm -rf build && cmake -B build -G Ninja -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++ && cmake --build build && ctest --test-dir build --output-on-failure
```
Expected: `test_display_decode` PASS (3/3). (If `display_decode.cpp` weren't written yet it would fail to link — write it per Step 1 first, so this is GREEN.)

- [ ] **Step 4: Commit**

```bash
git add cyd-dashboard/main/display_decode.h cyd-dashboard/main/display_decode.cpp cyd-dashboard/host_test/CMakeLists.txt cyd-dashboard/host_test/test_display_decode.cpp
git rm cyd-dashboard/host_test/test_status_parse.cpp
git commit -m "feat(cyd): host-tested DisplayStatus/Event decode helpers"
```

---

### Task 4: CYD ESP-NOW receiver + wire-up + hardware verify

**Files:**
- Create: `cyd-dashboard/main/espnow_rx.h`, `cyd-dashboard/main/espnow_rx.cpp`
- Modify: `cyd-dashboard/main/main.cpp`, `cyd-dashboard/main/CMakeLists.txt`
- Delete: `cyd-dashboard/main/wifi_sta.{h,cpp}`, `cyd-dashboard/main/status_client.{h,cpp}`, `cyd-dashboard/main/status_parse.{h,cpp}`

**Interfaces:**
- Consumes: `applyDisplayStatus`/`toLogEv` (Task 3); `display_init`/`touch_init`/`dashboard_*` (existing).
- Produces: `void espnow_rx_start(void)`; `void espnow_rx_snapshot(StatusModel& out, uint32_t nowMs)` (copies the latest status + sets `out.controllerReachable = (nowMs - lastStatusMs) < 3000`); `int espnow_rx_events(LogEv* out, int maxOut)` (copies the accumulated event ring, oldest→newest).

- [ ] **Step 1: Implement `espnow_rx`**

`cyd-dashboard/main/espnow_rx.h`:
```c
#pragma once
#include "status_model.h"
#ifdef __cplusplus
extern "C" {
#endif
void espnow_rx_start(void);
void espnow_rx_snapshot(StatusModel* out, unsigned long nowMs);
int  espnow_rx_events(LogEv* out, int maxOut);
#ifdef __cplusplus
}
#endif
```

`cyd-dashboard/main/espnow_rx.cpp`:
```cpp
#include "espnow_rx.h"
#include "display_decode.h"
#include "protocol.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <cstring>

static const char* TAG = "espnow_rx";
static SemaphoreHandle_t s_mtx;
static StatusModel       s_model;        // latest decoded status
static uint32_t          s_lastStatusMs; // ms of last DISP_STATUS
static LogEv             s_ring[16];
static int               s_count;
static int               s_maxSeq;

static void recv_cb(const esp_now_recv_info_t* /*info*/, const uint8_t* data, int len) {
    if (len < 1) return;
    uint8_t type = data[0];
    if (type == (uint8_t)fw::MsgType::DISP_STATUS && len == (int)sizeof(fw::DisplayStatusPacket)) {
        fw::DisplayStatusPacket p; std::memcpy(&p, data, sizeof(p));
        if (xSemaphoreTake(s_mtx, pdMS_TO_TICKS(5)) == pdTRUE) {
            applyDisplayStatus(p, s_model);
            s_lastStatusMs = (uint32_t)(esp_timer_get_time() / 1000);
            xSemaphoreGive(s_mtx);
        }
    } else if (type == (uint8_t)fw::MsgType::DISP_EVENT && len == (int)sizeof(fw::DisplayEventPacket)) {
        fw::DisplayEventPacket p; std::memcpy(&p, data, sizeof(p));
        LogEv e; toLogEv(p, e);
        if (xSemaphoreTake(s_mtx, pdMS_TO_TICKS(5)) == pdTRUE) {
            if (e.seq > s_maxSeq) {                  // new event -> append (drop oldest if full)
                if (s_count == 16) { std::memmove(&s_ring[0], &s_ring[1], 15 * sizeof(LogEv)); s_count = 15; }
                s_ring[s_count++] = e;
                s_maxSeq = e.seq;
            }
            xSemaphoreGive(s_mtx);
        }
    }
}

void espnow_rx_start(void) {
    s_mtx = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    // Do NOT associate. Pin the controller's channel and listen via ESP-NOW.
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(&recv_cb));
    ESP_LOGI(TAG, "ESP-NOW display RX up on channel 1 (no AP association)");
}

void espnow_rx_snapshot(StatusModel* out, unsigned long nowMs) {
    if (xSemaphoreTake(s_mtx, portMAX_DELAY) == pdTRUE) {
        *out = s_model;
        out->controllerReachable = (s_lastStatusMs != 0) && ((uint32_t)nowMs - s_lastStatusMs < 3000);
        xSemaphoreGive(s_mtx);
    }
}

int espnow_rx_events(LogEv* out, int maxOut) {
    int n = 0;
    if (xSemaphoreTake(s_mtx, portMAX_DELAY) == pdTRUE) {
        for (int i = 0; i < s_count && n < maxOut; ++i) out[n++] = s_ring[i];
        xSemaphoreGive(s_mtx);
    }
    return n;
}
```
(`esp_timer.h` is pulled transitively; add `#include "esp_timer.h"` if the build complains.)

- [ ] **Step 2: Rewrite `main.cpp`**

Replace `cyd-dashboard/main/main.cpp` with:
```cpp
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "display.h"
#include "touch.h"
#include "dashboard.h"
#include "espnow_rx.h"
#include "status_model.h"

extern "C" void app_main(void) {
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    lv_display_t* disp = display_init();
    touch_init(disp);
    dashboard_create();
    espnow_rx_start();

    static StatusModel model;
    static LogEv evs[16];
    while (true) {
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
        espnow_rx_snapshot(&model, now);
        int n = espnow_rx_events(evs, 16);
        if (n > 0) dashboard_set_events(evs, n);
        dashboard_update(model);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
```

- [ ] **Step 3: Update CMake (drop wifi/http, add espnow_rx/decode + fireworkcore include)**

Replace `cyd-dashboard/main/CMakeLists.txt` with:
```cmake
idf_component_register(
    SRCS "main.cpp" "display_decode.cpp" "espnow_rx.cpp" "display.cpp" "dashboard.cpp" "touch.cpp"
    INCLUDE_DIRS "." "../../components/fireworkcore/include")
```
Delete the now-unused sources:
```bash
git rm cyd-dashboard/main/wifi_sta.h cyd-dashboard/main/wifi_sta.cpp cyd-dashboard/main/status_client.h cyd-dashboard/main/status_client.cpp cyd-dashboard/main/status_parse.h cyd-dashboard/main/status_parse.cpp
```

- [ ] **Step 4: Build + flash the CYD**

Run (PowerShell):
```powershell
& 'C:\esp\v6.0.1\esp-idf\export.ps1'; Set-Location 'C:\Users\zombo\Desktop\Programming\RemoteFireworkCueSystem\RemoteFireworkCueSystem\cyd-dashboard'; idf.py build
```
Expected: `Project build complete.` Flash COM12 (auto-reset):
```powershell
Set-Location build; python -m esptool --chip esp32 -p COM12 -b 460800 --before default-reset --after hard-reset write-flash "@flash_args"
```
Capture COM12 serial (timed PowerShell SerialPort read ~6 s, no idf.py monitor): expect `ESP-NOW display RX up on channel 1` and **no** `wifi: ... disconnected` / `SA query` lines.

- [ ] **Step 5: Flash the controller (Task 2 build) + end-to-end hardware verify**

The controller build is from Task 2. Flash COM11 (CP210x → custom reset env + BOOT-hold if `0x13`):
```powershell
$env:ESPTOOL_CUSTOM_RESET_SEQUENCE="D0|R1|W0.6|D1|R0|W0.4|D0"
Set-Location 'C:\Users\zombo\Desktop\Programming\RemoteFireworkCueSystem\RemoteFireworkCueSystem\controller\build'; python -m esptool --chip esp32 -p COM11 -b 460800 --before default-reset --after hard-reset write-flash "@flash_args"
```
With box + controller running, verify on the CYD: live SAFE dashboard with link/RSSI, **steady** (no SAFE↔NO-CTRL flapping over a minute — the core goal). Arm/fire from the phone → events appear on the Log page. Pull controller power → CYD shows NO CTRL within ~3 s, recovers when powered back.

- [ ] **Step 6: Commit**

```bash
git add cyd-dashboard/main/espnow_rx.h cyd-dashboard/main/espnow_rx.cpp cyd-dashboard/main/main.cpp cyd-dashboard/main/CMakeLists.txt
git commit -m "feat(cyd): receive display data over ESP-NOW (no AP association)"
```
(The `git rm` of wifi/http sources from Step 3 is included in this commit.)

---

## Self-Review Notes
- **Spec coverage:** new frames (Task 1), controller broadcast incl. events-by-seq (Task 2), CYD STA-no-connect + esp_now recv + 3 s reachability (Task 4), pure host-tested decode (Task 3), UI unchanged (dashboard.* untouched), additive on controller (Task 2 only adds sends), read-only (CYD never transmits). Phone/web HTTP path untouched. Box safely ignores broadcasts (type+len mismatch in its receive path).
- **Type consistency:** `DisplayStatusPacket`/`DisplayEventPacket` fields identical across Tasks 1/2/3/4; `applyDisplayStatus`/`toLogEv`/`espnow_rx_*` signatures match between producer and consumer tasks; `StatusModel`/`LogEv` reused unchanged.
- **Concurrency:** `espnow_rx` recv callback (Wi-Fi task) and the render loop both guard `s_model`/`s_ring` with `s_mtx`; the callback never calls LVGL.
- **Channel:** CYD pins channel 1 to match the controller AP; broadcast uses `ifidx = WIFI_IF_AP` on the controller.
