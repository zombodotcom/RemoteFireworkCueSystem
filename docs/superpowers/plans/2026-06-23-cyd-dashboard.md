# CYD Status Dashboard Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A read-only ESP-IDF firmware for the CYD (ESP32-2432S028R, ST7789) that joins the controller's Wi-Fi AP, polls `GET /api/status`, and shows a status-hero dashboard (ARMED/SAFE, box link+RSSI, fired grid, sequence) on the TFT.

**Architecture:** `WiFi STA → status_client (HTTP poll ~1 Hz) → parseStatus(json) → StatusModel → dashboard (LVGL render)`. No box/controller changes — third consumer of the existing telemetry. The JSON parser is a pure, dependency-free, host-tested function.

**Tech Stack:** ESP-IDF v6.0.1 (C/C++), `esp_lcd` built-in ST7789 driver, `espressif/esp_lvgl_port` + `lvgl/lvgl` v9 for UI, `esp_http_client`, hand-rolled string-scan JSON extractor.

## Global Constraints

- **ESP-IDF builds run only in PowerShell**, after `& 'C:\esp\v6.0.1\esp-idf\export.ps1'`, then `idf.py build` in `cyd-dashboard`. Target is **esp32** (`idf.py set-target esp32` once).
- **Host parser test runs in Git Bash** (g++ + CMake + Ninja + CTest), mirroring `components/fireworkcore/host_test`.
- **Read-only:** the CYD has NO control authority. It only GETs `/api/status`. It never POSTs, arms, or fires.
- **CYD on COM12** (CH340, auto-reset works — no BOOT-hold). MAC `a4:f0:0f:5f:4f:00`.
- **Confirmed panel config (do not re-derive — proven on bench):** ST7789 via `esp_lcd_new_panel_st7789`; SPI2_HOST; SCLK 14, MOSI 13, MISO 12, CS 15, DC 2, RST −1; backlight GPIO 21 active-HIGH; 240×320 native; pclk 20 MHz; **invert OFF**; RGB565 needs a **byte-swap** (under LVGL: `lvgl_port_display_cfg_t.flags.swap_bytes = true`).
- **AP creds live only in git-ignored `main/secrets.h`** (`FW_AP_SSID`, `FW_AP_PASS`), with a committed `secrets.example.h`. SSID `FireControl`. The controller AP is WPA2-PSK, **PMF required** — IDF STA is PMF-capable by default, so a plain STA connect works.
- Controller telemetry URL: `http://192.168.4.1/api/status`. Its shape (consumed here):
  `{"armed":bool,"seqRunning":bool,"lastFailedBox":null|int,"boxes":[{"id":int,"linkAlive":bool,"rssi":int,"state":0|1,"firedBitmap":int,"lastFired":int}]}` (`boxes` may be empty).

---

## File Structure (new top-level project `cyd-dashboard/`)

- `CMakeLists.txt` — IDF project file.
- `sdkconfig.defaults` — enable LVGL fonts + color depth.
- `main/CMakeLists.txt`, `main/idf_component.yml` — sources + managed deps.
- `main/secrets.example.h` (committed) + `main/secrets.h` (git-ignored).
- `main/status_model.h` — `StatusModel` struct.
- `main/status_parse.{h,cpp}` — pure `parseStatus()` (host-testable, no IDF deps).
- `main/wifi_sta.{h,cpp}` — Wi-Fi STA connect + connected flag.
- `main/status_client.{h,cpp}` — HTTP poll + parse → shared model.
- `main/display.{h,cpp}` — esp_lcd ST7789 + esp_lvgl_port init.
- `main/dashboard.{h,cpp}` — LVGL status-hero widgets + `update(model)`.
- `main/main.cpp` — wire-up.
- `host_test/CMakeLists.txt`, `host_test/check.h`, `host_test/test_status_parse.cpp`.
- `.gitignore` — `build/`, `sdkconfig`, `main/secrets.h`, `managed_components/`, `dependencies.lock`.

---

### Task 1: Project scaffold + Wi-Fi STA

**Files:**
- Create: `cyd-dashboard/CMakeLists.txt`, `cyd-dashboard/main/CMakeLists.txt`, `cyd-dashboard/main/idf_component.yml`, `cyd-dashboard/.gitignore`, `cyd-dashboard/main/secrets.example.h`, `cyd-dashboard/main/secrets.h`, `cyd-dashboard/main/wifi_sta.h`, `cyd-dashboard/main/wifi_sta.cpp`, `cyd-dashboard/main/main.cpp`

**Interfaces:**
- Produces: `void wifi_sta_start(void);` (begins connect, auto-reconnects); `bool wifi_sta_connected(void);` (true once GOT_IP).

- [ ] **Step 1: Project + component files**

`cyd-dashboard/CMakeLists.txt`:
```cmake
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(cyd_dashboard)
```

`cyd-dashboard/main/CMakeLists.txt`:
```cmake
idf_component_register(
    SRCS "main.cpp" "wifi_sta.cpp" "status_parse.cpp" "status_client.cpp" "display.cpp" "dashboard.cpp"
    INCLUDE_DIRS ".")
```

`cyd-dashboard/main/idf_component.yml`:
```yaml
dependencies:
  idf: ">=5.0"
  lvgl/lvgl: "~9.2.0"
  espressif/esp_lvgl_port: "^2.4.0"
```

`cyd-dashboard/.gitignore`:
```
build/
sdkconfig
sdkconfig.old
managed_components/
dependencies.lock
main/secrets.h
```

`cyd-dashboard/main/secrets.example.h`:
```c
#pragma once
// Copy to secrets.h (git-ignored) and fill in the controller AP credentials.
#define FW_AP_SSID "FireControl"
#define FW_AP_PASS "changeme"
```

`cyd-dashboard/main/secrets.h` (git-ignored — create locally with the real password):
```c
#pragma once
#define FW_AP_SSID "FireControl"
#define FW_AP_PASS "fireworks2026"
```

- [ ] **Step 2: Wi-Fi STA module**

`cyd-dashboard/main/wifi_sta.h`:
```c
#pragma once
#ifdef __cplusplus
extern "C" {
#endif
void wifi_sta_start(void);
bool wifi_sta_connected(void);
#ifdef __cplusplus
}
#endif
```

`cyd-dashboard/main/wifi_sta.cpp`:
```cpp
#include "wifi_sta.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include <cstring>
#if __has_include("secrets.h")
#include "secrets.h"
#else
#include "secrets.example.h"
#endif

static const char* TAG = "wifi_sta";
static volatile bool s_connected = false;

static void on_wifi(void*, esp_event_base_t base, int32_t id, void*) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        s_connected = false;
        ESP_LOGW(TAG, "disconnected — retrying");
        esp_wifi_connect();
    }
}
static void on_ip(void*, esp_event_base_t, int32_t, void* data) {
    s_connected = true;
    ESP_LOGI(TAG, "got IP — connected to %s", FW_AP_SSID);
}

void wifi_sta_start(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &on_wifi, nullptr, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &on_ip, nullptr, nullptr));

    wifi_config_t wc = {};
    std::strncpy((char*)wc.sta.ssid, FW_AP_SSID, sizeof(wc.sta.ssid) - 1);
    std::strncpy((char*)wc.sta.password, FW_AP_PASS, sizeof(wc.sta.password) - 1);
    wc.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wc.sta.pmf_cfg.capable = true;     // controller AP requires PMF
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());
}

bool wifi_sta_connected(void) { return s_connected; }
```

- [ ] **Step 3: Minimal main that brings up Wi-Fi**

`cyd-dashboard/main/main.cpp`:
```cpp
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "wifi_sta.h"

static const char* TAG = "cyd";

extern "C" void app_main(void) {
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    wifi_sta_start();
    while (true) {
        ESP_LOGI(TAG, "wifi connected: %s", wifi_sta_connected() ? "yes" : "no");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```

(Create empty stubs so the project links: `status_parse.cpp`, `status_client.cpp`, `display.cpp`, `dashboard.cpp` each containing just `// implemented in a later task`. They are added to sources here but filled later. To keep this task's build self-contained, instead register ONLY the files that exist now — set `main/CMakeLists.txt` SRCS to `"main.cpp" "wifi_sta.cpp"` for this task, and expand the SRCS list in the task that adds each file.)

- [ ] **Step 4: set-target, build, flash, verify Wi-Fi join**

Run (PowerShell):
```powershell
& 'C:\esp\v6.0.1\esp-idf\export.ps1'; Set-Location 'C:\Users\zombo\Desktop\Programming\RemoteFireworkCueSystem\RemoteFireworkCueSystem\cyd-dashboard'; idf.py set-target esp32; idf.py build
```
Expected: `Project build complete.` Then flash (CYD auto-resets):
```powershell
Set-Location build; python -m esptool --chip esp32 -p COM12 -b 460800 --before default-reset --after hard-reset write-flash "@flash_args"
```
With the controller powered (AP up), the serial monitor (`idf.py -p COM12 monitor`) should log `got IP — connected to FireControl` and then `wifi connected: yes`. Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add cyd-dashboard/CMakeLists.txt cyd-dashboard/main/CMakeLists.txt cyd-dashboard/main/idf_component.yml cyd-dashboard/.gitignore cyd-dashboard/main/secrets.example.h cyd-dashboard/main/wifi_sta.h cyd-dashboard/main/wifi_sta.cpp cyd-dashboard/main/main.cpp
git commit -m "feat(cyd): project scaffold + WiFi STA join to controller AP"
```
(Do NOT add `main/secrets.h` — it is git-ignored.)

---

### Task 2: `parseStatus()` + host test (TDD)

**Files:**
- Create: `cyd-dashboard/main/status_model.h`, `cyd-dashboard/main/status_parse.h`, `cyd-dashboard/main/status_parse.cpp`, `cyd-dashboard/host_test/CMakeLists.txt`, `cyd-dashboard/host_test/check.h`, `cyd-dashboard/host_test/test_status_parse.cpp`

**Interfaces:**
- Produces: `struct StatusModel { bool controllerReachable; bool boxPresent; bool boxArmed; bool boxLinkAlive; int rssi; uint16_t firedBitmap; bool seqRunning; uint32_t lastUpdateMs; };` and `bool parseStatus(const char* json, StatusModel& out);` — populates box/seq fields; returns false on null/empty/malformed (no `"boxes"`). Does NOT set `controllerReachable`/`lastUpdateMs`.

- [ ] **Step 1: Model + test harness header**

`cyd-dashboard/main/status_model.h`:
```cpp
#pragma once
#include <cstdint>
struct StatusModel {
    bool     controllerReachable = false; // set by client on HTTP success
    bool     boxPresent          = false; // boxes[] had an entry
    bool     boxArmed            = false; // box state == 1
    bool     boxLinkAlive        = false; // boxes[0].linkAlive
    int      rssi                = 0;      // boxes[0].rssi (dBm)
    uint16_t firedBitmap         = 0;      // boxes[0].firedBitmap
    bool     seqRunning          = false;  // top-level seqRunning
    uint32_t lastUpdateMs        = 0;      // set by client
};
```

`cyd-dashboard/host_test/check.h` (copy of the project's host-test macros):
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

- [ ] **Step 2: Write the failing tests**

`cyd-dashboard/host_test/test_status_parse.cpp`:
```cpp
#include "check.h"
#include "status_parse.h"

static const char* JSON_ARMED =
  "{\"armed\":true,\"seqRunning\":false,\"lastFailedBox\":null,"
  "\"boxes\":[{\"id\":0,\"linkAlive\":true,\"rssi\":-52,\"state\":1,"
  "\"firedBitmap\":5,\"lastFired\":2}]}";

static const char* JSON_SAFE_NOLINK =
  "{\"armed\":false,\"seqRunning\":true,\"lastFailedBox\":null,"
  "\"boxes\":[{\"id\":0,\"linkAlive\":false,\"rssi\":0,\"state\":0,"
  "\"firedBitmap\":0,\"lastFired\":-1}]}";

static const char* JSON_EMPTY_BOXES =
  "{\"armed\":false,\"seqRunning\":false,\"lastFailedBox\":null,\"boxes\":[]}";

void test_parse_armed_box() {
    StatusModel m;
    CHECK(parseStatus(JSON_ARMED, m));
    CHECK(m.boxPresent);
    CHECK(m.boxArmed);            // state == 1
    CHECK(m.boxLinkAlive);
    CHECK_EQ(m.rssi, -52);
    CHECK_EQ((int)m.firedBitmap, 5);
    CHECK(!m.seqRunning);
}
void test_parse_safe_nolink_seq() {
    StatusModel m;
    CHECK(parseStatus(JSON_SAFE_NOLINK, m));
    CHECK(m.boxPresent);
    CHECK(!m.boxArmed);          // state == 0
    CHECK(!m.boxLinkAlive);
    CHECK_EQ(m.rssi, 0);
    CHECK(m.seqRunning);
}
void test_parse_empty_boxes() {
    StatusModel m;
    CHECK(parseStatus(JSON_EMPTY_BOXES, m));  // valid, but no box
    CHECK(!m.boxPresent);
}
void test_parse_malformed_returns_false() {
    StatusModel m;
    CHECK(!parseStatus("{\"armed\":false}", m));  // no "boxes"
    CHECK(!parseStatus("", m));
    CHECK(!parseStatus(nullptr, m));
}
int main() {
    RUN(test_parse_armed_box);
    RUN(test_parse_safe_nolink_seq);
    RUN(test_parse_empty_boxes);
    RUN(test_parse_malformed_returns_false);
    return REPORT();
}
```

`cyd-dashboard/host_test/CMakeLists.txt`:
```cmake
cmake_minimum_required(VERSION 3.16)
project(cyd_host_test CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
enable_testing()
include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../main)
add_executable(test_status_parse test_status_parse.cpp ../main/status_parse.cpp)
add_test(NAME test_status_parse COMMAND test_status_parse)
```

`cyd-dashboard/main/status_parse.h`:
```cpp
#pragma once
#include "status_model.h"
// Parse the controller's /api/status JSON into the box/seq fields of `out`.
// Returns false if json is null/empty or has no "boxes" array.
bool parseStatus(const char* json, StatusModel& out);
```

- [ ] **Step 3: Run test to verify it fails**

Run (Git Bash):
```bash
cd cyd-dashboard/host_test && cmake -B build -G Ninja && cmake --build build
```
Expected: link error — `parseStatus` undefined (status_parse.cpp not yet implemented).

- [ ] **Step 4: Implement the parser**

`cyd-dashboard/main/status_parse.cpp`:
```cpp
#include "status_parse.h"
#include <cstring>
#include <cstdlib>

// Return a pointer to the first value char after `"key":`, searching from `from`.
static const char* valueAfter(const char* from, const char* key) {
    if (!from) return nullptr;
    char pat[40];
    int n = 0;
    pat[n++] = '"';
    for (const char* k = key; *k && n < 37; ++k) pat[n++] = *k;
    pat[n++] = '"';
    pat[n] = 0;
    const char* p = std::strstr(from, pat);
    if (!p) return nullptr;
    p = std::strchr(p + n, ':');
    if (!p) return nullptr;
    ++p;
    while (*p == ' ' || *p == '\t') ++p;
    return p;
}
static bool boolAt(const char* p, bool& out) {
    if (!p) return false;
    if (std::strncmp(p, "true", 4) == 0)  { out = true;  return true; }
    if (std::strncmp(p, "false", 5) == 0) { out = false; return true; }
    return false;
}
static bool intAt(const char* p, int& out) {
    if (!p) return false;
    char* end = nullptr;
    long v = std::strtol(p, &end, 10);
    if (end == p) return false;
    out = (int)v;
    return true;
}

bool parseStatus(const char* json, StatusModel& out) {
    if (!json || !*json) return false;

    bool b;
    if (boolAt(valueAfter(json, "seqRunning"), b)) out.seqRunning = b;

    const char* boxes = std::strstr(json, "\"boxes\"");
    if (!boxes) return false;                 // malformed for our purposes
    const char* lb = std::strchr(boxes, '[');
    if (!lb) return false;
    const char* q = lb + 1;
    while (*q == ' ' || *q == '\t' || *q == '\n' || *q == '\r') ++q;
    if (*q == ']') { out.boxPresent = false; return true; }   // empty array

    const char* obj = std::strchr(lb, '{');
    if (!obj) { out.boxPresent = false; return true; }
    out.boxPresent = true;

    int iv; bool bv;
    if (boolAt(valueAfter(obj, "linkAlive"), bv))  out.boxLinkAlive = bv;
    if (intAt(valueAfter(obj, "rssi"), iv))        out.rssi = iv;
    if (intAt(valueAfter(obj, "state"), iv))       out.boxArmed = (iv == 1);
    if (intAt(valueAfter(obj, "firedBitmap"), iv)) out.firedBitmap = (uint16_t)iv;
    return true;
}
```

- [ ] **Step 5: Run tests to verify they pass**

Run (Git Bash):
```bash
cd cyd-dashboard/host_test && cmake --build build && ctest --test-dir build --output-on-failure
```
Expected: `test_status_parse` PASS (4/4).

- [ ] **Step 6: Commit**

```bash
git add cyd-dashboard/main/status_model.h cyd-dashboard/main/status_parse.h cyd-dashboard/main/status_parse.cpp cyd-dashboard/host_test/
git commit -m "feat(cyd): host-tested parseStatus() for /api/status JSON"
```

---

### Task 3: `status_client` — poll + parse into a shared model

**Files:**
- Create: `cyd-dashboard/main/status_client.h`, `cyd-dashboard/main/status_client.cpp`
- Modify: `cyd-dashboard/main/CMakeLists.txt` (add `status_parse.cpp status_client.cpp` to SRCS), `cyd-dashboard/main/main.cpp` (call the poll + log the model)

**Interfaces:**
- Consumes: `parseStatus()` (Task 2); `wifi_sta_connected()` (Task 1).
- Produces: `void status_client_poll_once(StatusModel& model);` — does one HTTP GET of `/api/status`; on success sets `model.controllerReachable=true`, runs `parseStatus`, stamps `lastUpdateMs`; on any failure sets `model.controllerReachable=false` and leaves the previous box fields intact (stale).

- [ ] **Step 1: Implement the client**

`cyd-dashboard/main/status_client.h`:
```cpp
#pragma once
#include "status_model.h"
void status_client_poll_once(StatusModel& model);
```

`cyd-dashboard/main/status_client.cpp`:
```cpp
#include "status_client.h"
#include "status_parse.h"
#include "esp_http_client.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <cstring>

static const char* TAG = "status_client";
static const char* URL = "http://192.168.4.1/api/status";

// Accumulate the response body into a fixed buffer.
struct Accum { char buf[512]; int len; };

static esp_err_t on_evt(esp_http_client_event_t* e) {
    if (e->event_id == HTTP_EVENT_ON_DATA) {
        Accum* a = (Accum*)e->user_data;
        int n = e->data_len;
        if (a->len + n > (int)sizeof(a->buf) - 1) n = sizeof(a->buf) - 1 - a->len;
        if (n > 0) { std::memcpy(a->buf + a->len, e->data, n); a->len += n; }
    }
    return ESP_OK;
}

void status_client_poll_once(StatusModel& model) {
    Accum acc{}; acc.len = 0;
    esp_http_client_config_t cfg = {};
    cfg.url = URL;
    cfg.timeout_ms = 800;
    cfg.event_handler = on_evt;
    cfg.user_data = &acc;
    esp_http_client_handle_t c = esp_http_client_init(&cfg);
    esp_err_t err = esp_http_client_perform(c);
    int status = esp_http_client_get_status_code(c);
    esp_http_client_cleanup(c);

    if (err != ESP_OK || status != 200) {
        model.controllerReachable = false;
        ESP_LOGW(TAG, "poll failed (err=%s status=%d)", esp_err_to_name(err), status);
        return;
    }
    acc.buf[acc.len] = 0;
    model.controllerReachable = true;
    parseStatus(acc.buf, model);
    model.lastUpdateMs = (uint32_t)(esp_timer_get_time() / 1000);
    ESP_LOGI(TAG, "armed=%d link=%d rssi=%d fired=0x%04x seq=%d present=%d",
             model.boxArmed, model.boxLinkAlive, model.rssi,
             (unsigned)model.firedBitmap, model.seqRunning, model.boxPresent);
}
```

- [ ] **Step 2: Wire the poll into `main.cpp`**

Replace the `while(true)` loop body in `main.cpp` with:
```cpp
    static StatusModel model;
    while (true) {
        if (wifi_sta_connected()) status_client_poll_once(model);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
```
Add `#include "status_client.h"` and `#include "status_model.h"` at the top of `main.cpp`. Update `main/CMakeLists.txt` SRCS to: `"main.cpp" "wifi_sta.cpp" "status_parse.cpp" "status_client.cpp"`.

- [ ] **Step 3: Build, flash, verify against the live controller**

Run (PowerShell): build + flash COM12 as in Task 1 Step 4. With the controller AND box powered, `idf.py -p COM12 monitor` should log a line like `armed=0 link=1 rssi=-55 fired=0x0000 seq=0 present=1` — confirming the full Wi-Fi→HTTP→parse path against real telemetry. Expected: PASS.

- [ ] **Step 4: Commit**

```bash
git add cyd-dashboard/main/status_client.h cyd-dashboard/main/status_client.cpp cyd-dashboard/main/main.cpp cyd-dashboard/main/CMakeLists.txt
git commit -m "feat(cyd): poll /api/status over HTTP and parse into StatusModel"
```

---

### Task 4: Display + LVGL bring-up (static label proof)

**Files:**
- Create: `cyd-dashboard/main/display.h`, `cyd-dashboard/main/display.cpp`, `cyd-dashboard/sdkconfig.defaults`
- Modify: `cyd-dashboard/main/idf_component.yml` (add LVGL deps), `cyd-dashboard/main/CMakeLists.txt` (add `display.cpp`), `cyd-dashboard/main/main.cpp` (call `display_init` + show a label)

**Interfaces:**
- Produces: `lv_display_t* display_init(void);` — brings up SPI + ST7789 + esp_lvgl_port and returns the LVGL display. After it returns, LVGL calls must be wrapped in `lvgl_port_lock(0)` / `lvgl_port_unlock()`.

- [ ] **Step 1: Add LVGL dependencies (version-pinned) + font config**

Task 1 omitted LVGL from `idf_component.yml` (unused then) and flagged that `esp_lvgl_port 2.8` (resolved from `^2.4.0`) needs **LVGL ≥ 9.3** for `LV_COLOR_FORMAT_RGB565_SWAPPED` (which is what `flags.swap_bytes` maps to). Set `cyd-dashboard/main/idf_component.yml` to:
```yaml
dependencies:
  idf: ">=5.0"
  lvgl/lvgl: "~9.3.0"
  espressif/esp_lvgl_port: "^2.4.0"
```
If the build still errors on a missing `swap_bytes` field or an `RGB565_SWAPPED` symbol, that is the known byte-order knob: confirm the resolved `esp_lvgl_port` version's `lvgl_port_display_cfg_t` field name (`flags.swap_bytes` in 2.x) and keep the swap enabled by whatever field that version exposes — the panel REQUIRES the RGB565 byte-swap (proven on the bench). Report the exact versions resolved.

`cyd-dashboard/sdkconfig.defaults`:
```
CONFIG_LV_COLOR_DEPTH_16=y
CONFIG_LV_FONT_MONTSERRAT_14=y
CONFIG_LV_FONT_MONTSERRAT_28=y
CONFIG_LV_FONT_MONTSERRAT_48=y
CONFIG_FREERTOS_HZ=1000
```
(Delete any existing `cyd-dashboard/sdkconfig` so the defaults take effect, then rebuild.)

- [ ] **Step 2: Implement display_init**

`cyd-dashboard/main/display.h`:
```c
#pragma once
#include "lvgl.h"
#ifdef __cplusplus
extern "C" {
#endif
lv_display_t* display_init(void);
#ifdef __cplusplus
}
#endif
```

`cyd-dashboard/main/display.cpp`:
```cpp
#include "display.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_st7789.h"
#include "esp_lvgl_port.h"

#define PIN_SCLK 14
#define PIN_MOSI 13
#define PIN_MISO 12
#define PIN_CS   15
#define PIN_DC    2
#define PIN_RST  -1
#define PIN_BL   21
#define LCD_H 240   // native portrait width
#define LCD_V 320   // native portrait height

lv_display_t* display_init(void) {
    gpio_config_t bk = {};
    bk.mode = GPIO_MODE_OUTPUT;
    bk.pin_bit_mask = 1ULL << PIN_BL;
    gpio_config(&bk);
    gpio_set_level((gpio_num_t)PIN_BL, 1);   // backlight on

    spi_bus_config_t bus = {};
    bus.sclk_io_num = PIN_SCLK;
    bus.mosi_io_num = PIN_MOSI;
    bus.miso_io_num = PIN_MISO;
    bus.quadwp_io_num = -1;
    bus.quadhd_io_num = -1;
    bus.max_transfer_sz = LCD_H * 80 * sizeof(uint16_t);
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_handle_t io = NULL;
    esp_lcd_panel_io_spi_config_t iocfg = {};
    iocfg.dc_gpio_num = PIN_DC;
    iocfg.cs_gpio_num = PIN_CS;
    iocfg.pclk_hz = 20 * 1000 * 1000;
    iocfg.lcd_cmd_bits = 8;
    iocfg.lcd_param_bits = 8;
    iocfg.spi_mode = 0;
    iocfg.trans_queue_depth = 10;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &iocfg, &io));

    esp_lcd_panel_handle_t panel = NULL;
    esp_lcd_panel_dev_config_t devcfg = {};
    devcfg.reset_gpio_num = PIN_RST;
    devcfg.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
    devcfg.bits_per_pixel = 16;
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io, &devcfg, &panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel, false));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));

    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    ESP_ERROR_CHECK(lvgl_port_init(&lvgl_cfg));

    lvgl_port_display_cfg_t disp_cfg = {};
    disp_cfg.io_handle = io;
    disp_cfg.panel_handle = panel;
    disp_cfg.buffer_size = LCD_H * 40;
    disp_cfg.double_buffer = true;
    disp_cfg.hres = LCD_H;
    disp_cfg.vres = LCD_V;
    disp_cfg.rotation.swap_xy = true;    // landscape 320x240
    disp_cfg.rotation.mirror_x = true;
    disp_cfg.rotation.mirror_y = false;
    disp_cfg.flags.buff_dma = true;
    disp_cfg.flags.swap_bytes = true;    // RGB565 byte order for this panel
    return lvgl_port_add_disp(&disp_cfg);
}
```

- [ ] **Step 3: Show a centered label from main**

In `main.cpp`, after `wifi_sta_start();` add:
```cpp
    display_init();
    if (lvgl_port_lock(0)) {
        lv_obj_t* lbl = lv_label_create(lv_screen_active());
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_28, 0);
        lv_label_set_text(lbl, "CYD OK");
        lv_obj_center(lbl);
        lvgl_port_unlock();
    }
```
Add `#include "display.h"` and `#include "esp_lvgl_port.h"`. Add `display.cpp` to `main/CMakeLists.txt` SRCS.

- [ ] **Step 4: Build, flash, verify on the panel**

Delete `cyd-dashboard/sdkconfig` (so defaults apply), then build + flash COM12 (PowerShell, as Task 1). Expected on the CYD: a black screen with **"CYD OK"** centered in white, landscape orientation. If text is upside-down or mirrored, flip `mirror_x`/`mirror_y` in `display.cpp` and reflash until upright. Expected: PASS (legible "CYD OK").

- [ ] **Step 5: Commit**

```bash
git add cyd-dashboard/main/display.h cyd-dashboard/main/display.cpp cyd-dashboard/main/main.cpp cyd-dashboard/main/CMakeLists.txt cyd-dashboard/sdkconfig.defaults
git commit -m "feat(cyd): ST7789 + LVGL bring-up (esp_lvgl_port), static label proof"
```

---

### Task 5: Dashboard widgets + `update(model)`

**Files:**
- Create: `cyd-dashboard/main/dashboard.h`, `cyd-dashboard/main/dashboard.cpp`
- Modify: `cyd-dashboard/main/CMakeLists.txt` (add `dashboard.cpp`)

**Interfaces:**
- Consumes: `display_init()` (Task 4); `StatusModel` (Task 2); `lvgl_port_lock/unlock`.
- Produces: `void dashboard_create(void);` (builds the static widget tree on the active screen) and `void dashboard_update(const StatusModel& m);` (refreshes widget text/colors; takes the LVGL lock internally).

- [ ] **Step 1: Implement the dashboard**

`cyd-dashboard/main/dashboard.h`:
```cpp
#pragma once
#include "status_model.h"
void dashboard_create(void);
void dashboard_update(const StatusModel& m);
```

`cyd-dashboard/main/dashboard.cpp`:
```cpp
#include "dashboard.h"
#include "esp_lvgl_port.h"
#include "lvgl.h"
#include <cstdio>

static lv_obj_t* s_banner = nullptr;   // ARMED/SAFE/NO CONTROLLER/NO BOX LINK
static lv_obj_t* s_boxline = nullptr;  // link + rssi
static lv_obj_t* s_seqline = nullptr;  // sequence
static lv_obj_t* s_cells[16] = {nullptr};

static const lv_color_t C_GREEN = LV_COLOR_MAKE(0x14, 0x80, 0x2a);
static const lv_color_t C_RED   = LV_COLOR_MAKE(0xc0, 0x10, 0x10);
static const lv_color_t C_GREY  = LV_COLOR_MAKE(0x55, 0x55, 0x55);
static const lv_color_t C_AMBER = LV_COLOR_MAKE(0xc8, 0x80, 0x00);
static const lv_color_t C_CELL_OFF = LV_COLOR_MAKE(0x22, 0x22, 0x22);
static const lv_color_t C_CELL_ON  = LV_COLOR_MAKE(0xe0, 0xa0, 0x20);

void dashboard_create(void) {
    if (!lvgl_port_lock(0)) return;
    lv_obj_t* scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);

    // Hero banner (top, full width)
    s_banner = lv_label_create(scr);
    lv_obj_set_style_text_font(s_banner, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(s_banner, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(s_banner, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(s_banner, 8, 0);
    lv_obj_set_width(s_banner, 320);
    lv_obj_set_style_text_align(s_banner, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(s_banner, LV_ALIGN_TOP_MID, 0, 0);
    lv_label_set_text(s_banner, "...");

    // Box line
    s_boxline = lv_label_create(scr);
    lv_obj_set_style_text_font(s_boxline, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_boxline, lv_color_white(), 0);
    lv_obj_align(s_boxline, LV_ALIGN_TOP_LEFT, 6, 80);
    lv_label_set_text(s_boxline, "");

    // Sequence line
    s_seqline = lv_label_create(scr);
    lv_obj_set_style_text_font(s_seqline, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_seqline, lv_color_white(), 0);
    lv_obj_align(s_seqline, LV_ALIGN_TOP_LEFT, 6, 104);
    lv_label_set_text(s_seqline, "");

    // Fired grid: 2 rows x 8 cells along the bottom
    for (int i = 0; i < 16; ++i) {
        lv_obj_t* c = lv_obj_create(scr);
        lv_obj_set_size(c, 34, 34);
        lv_obj_set_style_radius(c, 4, 0);
        lv_obj_set_style_border_width(c, 1, 0);
        lv_obj_set_style_bg_color(c, C_CELL_OFF, 0);
        int col = i % 8, row = i / 8;
        lv_obj_align(c, LV_ALIGN_TOP_LEFT, 6 + col * 38, 140 + row * 40);
        s_cells[i] = c;
    }
    lvgl_port_unlock();
}

void dashboard_update(const StatusModel& m) {
    if (!lvgl_port_lock(0)) return;

    if (!m.controllerReachable) {
        lv_label_set_text(s_banner, "NO CONTROLLER");
        lv_obj_set_style_bg_color(s_banner, C_GREY, 0);
    } else if (!m.boxPresent || !m.boxLinkAlive) {
        lv_label_set_text(s_banner, "NO BOX LINK");
        lv_obj_set_style_bg_color(s_banner, C_AMBER, 0);
    } else if (m.boxArmed) {
        lv_label_set_text(s_banner, "ARMED");
        lv_obj_set_style_bg_color(s_banner, C_RED, 0);
    } else {
        lv_label_set_text(s_banner, "SAFE");
        lv_obj_set_style_bg_color(s_banner, C_GREEN, 0);
    }

    char line[48];
    if (m.controllerReachable && m.boxLinkAlive)
        std::snprintf(line, sizeof(line), "Box0  LINK OK  %d dBm", m.rssi);
    else
        std::snprintf(line, sizeof(line), "Box0  LINK DOWN");
    lv_label_set_text(s_boxline, line);

    lv_label_set_text(s_seqline, m.seqRunning ? "Sequence: running" : "Sequence: idle");

    for (int i = 0; i < 16; ++i) {
        bool on = (m.firedBitmap >> i) & 0x1;
        lv_obj_set_style_bg_color(s_cells[i], on ? C_CELL_ON : C_CELL_OFF, 0);
    }
    lvgl_port_unlock();
}
```

Add `dashboard.cpp` to `main/CMakeLists.txt` SRCS.

- [ ] **Step 2: Build to verify it compiles**

Build (PowerShell) as in Task 1. Expected: `Project build complete.` (No on-hardware check yet — wired up in Task 6.)

- [ ] **Step 3: Commit**

```bash
git add cyd-dashboard/main/dashboard.h cyd-dashboard/main/dashboard.cpp cyd-dashboard/main/CMakeLists.txt
git commit -m "feat(cyd): status-hero dashboard widgets + update(model)"
```

---

### Task 6: Integrate + end-to-end hardware verification

**Files:**
- Modify: `cyd-dashboard/main/main.cpp`

**Interfaces:**
- Consumes: `display_init`, `dashboard_create`, `dashboard_update`, `status_client_poll_once`, `wifi_sta_start`, `wifi_sta_connected`.

- [ ] **Step 1: Final main wiring**

Replace `cyd-dashboard/main/main.cpp` entirely with:
```cpp
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "wifi_sta.h"
#include "display.h"
#include "dashboard.h"
#include "status_client.h"
#include "status_model.h"

extern "C" void app_main(void) {
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    display_init();
    dashboard_create();
    wifi_sta_start();

    static StatusModel model;       // controllerReachable defaults false
    dashboard_update(model);        // shows "NO CONTROLLER" until first good poll

    while (true) {
        if (wifi_sta_connected()) status_client_poll_once(model);
        else                      model.controllerReachable = false;
        dashboard_update(model);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```
(Remove the temporary label/poll code from Tasks 3–4; this is the final loop.)

- [ ] **Step 2: Build + flash**

Build + flash COM12 (PowerShell, as Task 1). Expected: `Project build complete.` and `Hash of data verified.`

- [ ] **Step 3: End-to-end hardware verification**

With box (COM9) and controller (COM11) powered:
1. CYD boots → shows **NO CONTROLLER** (grey) until it joins the AP, then within ~1–2 s flips to **SAFE** (green) with `Box0  LINK OK  -NN dBm` and `Sequence: idle`.
2. Arm the box (GPIO4→GND jumper + ARM from the phone UI) → banner flips to **ARMED** (red).
3. Fire a channel → its cell lights amber.
4. Power off the box → within ~2 s banner → **NO BOX LINK** (amber).
5. Power off the controller → banner → **NO CONTROLLER** (grey).
Expected: each transition appears on the CYD within ~1–2 s.

- [ ] **Step 4: Commit**

```bash
git add cyd-dashboard/main/main.cpp
git commit -m "feat(cyd): wire display + poll loop; end-to-end status dashboard"
```

---

## Self-Review Notes

- **Spec coverage:** read-only (no POST anywhere) ✓ (Tasks 3,6); ESP-IDF + esp_lcd ST7789 + LVGL ✓ (Tasks 1,4,5); WiFi-STA poll `/api/status` ✓ (Tasks 1,3); confirmed panel config incl. byte-swap (`flags.swap_bytes`) ✓ (Task 4); status-hero layout (banner/box line/seq/fired grid) ✓ (Task 5); honest states NO CONTROLLER vs NO BOX LINK ✓ (Task 5,6); `parseStatus` host-tested ✓ (Task 2); secrets git-ignored ✓ (Task 1); own `cyd-dashboard/` project ✓. Out-of-scope items (touch, multi-box, battery) correctly absent.
- **Type consistency:** `StatusModel` fields are identical across Tasks 2/3/5/6; `parseStatus`, `status_client_poll_once`, `display_init`, `dashboard_create`, `dashboard_update`, `wifi_sta_start`, `wifi_sta_connected` signatures match between producer and consumer tasks.
- **Integration risk:** the only unproven stack is LVGL/esp_lvgl_port; Task 4 isolates it as a static-label proof on the real panel before any dashboard widgets, and the panel SPI config itself is already bench-proven. Mirror flags (`mirror_x`/`mirror_y`) are tuned by eye in Task 4 Step 4.
