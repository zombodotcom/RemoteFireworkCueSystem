# Observability Plan 3 — CYD Touch-Paging (Dashboard / Log / Diag)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add the event log + diagnostics to the CYD as tap-paged screens — tap anywhere cycles Dashboard → Event Log → Diagnostics → back — with the ARMED/SAFE state always visible and the Dashboard enriched with latest-event + key stat chips. Fed by Plan 1's `/api/events` + extended `/api/status`.

**Architecture:** The CYD parser gains `diag`/`fault`/`lastHeard` fields + a pure `parseEvents()` (host-tested). `status_client` parses those and polls `/api/events`. The XPT2046 touchscreen is enabled (navigation only — never fires). The dashboard becomes three LVGL page-containers shown one at a time; a screen tap advances the page. State stays visible (big banner on Dashboard; slim color bar on Log/Diag).

**Tech Stack:** ESP-IDF v6.0.1, `esp_lcd` ST7789 + `esp_lcd_touch_xpt2046` + `esp_lvgl_port`/LVGL 9.3.

## Global Constraints

- **READ-ONLY firing:** touch is for NAVIGATION ONLY. It must never arm/fire/POST. The CYD has no control authority.
- ESP-IDF build in **PowerShell** (`export.ps1` + `idf.py build`, cyd-dashboard dir); flash **COM12** (auto-reset works). Host parser tests in **Git Bash**, g++ forced (`-DCMAKE_CXX_COMPILER=g++`).
- Parser pieces are **pure C++** (no IDF) so they host-test. `parseEvents` and the extended `parseStatus` live in `status_parse.{h,cpp}`.
- Confirmed panel config (display.cpp, bench-proven): ST7789, SPI2_HOST, SCLK14/MOSI13/MISO12/CS15/DC2/RST-1/BL21-high, invert OFF, `flags.swap_bytes`, landscape hres=320/vres=240. **Touch is a SEPARATE SPI bus** (SPI3_HOST): T_CLK 25, T_MOSI 32, T_MISO 39, T_CS 33, T_IRQ 36 (XPT2046).
- All LVGL widget calls wrapped in `lvgl_port_lock(0)`/`unlock` (LVGL runs in its own task); touch/page-tap callbacks run in the LVGL task (no lock needed inside them).
- `/api/events` shape: `{lastSeq, events:[{seq,t,sev,msg}]}`; `/api/status` adds `diag` + `fault` + per-box `lastHeardMs`.

---

## File Structure
- `cyd-dashboard/main/status_model.h` — add `Diag`/fault/lastHeard fields; `LogEv` type (Task 1).
- `cyd-dashboard/main/status_parse.{h,cpp}` — parse diag/fault/lastHeard; `parseEvents()` (Task 1).
- `cyd-dashboard/host_test/test_status_parse.cpp` — extend (Task 1).
- `cyd-dashboard/main/status_client.{h,cpp}` — diag/fault into model + `status_client_poll_events()` (Task 2).
- `cyd-dashboard/main/idf_component.yml` — add `esp_lcd_touch_xpt2046` (Task 3).
- `cyd-dashboard/main/touch.{h,cpp}` — XPT2046 init + `lvgl_port_add_touch` (Task 3).
- `cyd-dashboard/main/dashboard.{h,cpp}` — 3 page-containers + tap-cycle + enriched dashboard + log/diag render (Task 4).
- `cyd-dashboard/main/main.cpp` — poll events, init touch, feed events to dashboard (Task 5).

---

### Task 1: Parser + model extensions (host TDD)

**Files:**
- Modify: `cyd-dashboard/main/status_model.h`, `cyd-dashboard/main/status_parse.h`, `cyd-dashboard/main/status_parse.cpp`, `cyd-dashboard/host_test/test_status_parse.cpp`, `cyd-dashboard/host_test/CMakeLists.txt`

**Interfaces:**
- Produces: `StatusModel` gains `struct {uint32_t uptimeMs,freeHeap,apClients,fired,acked,failed,retries,lastAckMs;} diag;`, `bool faultActive; char faultMsg[32]; uint32_t boxLastHeardMs;`. `struct LogEv { int seq; int sev; char msg[48]; };` and `int parseEvents(const char* json, LogEv* out, int maxOut)` (parses `/api/events` array oldest→newest, returns count, ≤maxOut). `parseStatus` also fills the new diag/fault/lastHeard fields.

- [ ] **Step 1: Extend the model + header**

In `cyd-dashboard/main/status_model.h`, add inside `struct StatusModel` (after `lastUpdateMs`):
```cpp
    // Diagnostics (from /api/status "diag")
    struct {
        uint32_t uptimeMs = 0, freeHeap = 0, apClients = 0;
        uint32_t fired = 0, acked = 0, failed = 0, retries = 0, lastAckMs = 0;
    } diag;
    bool     faultActive = false;     // /api/status fault.active
    char     faultMsg[32] = {0};      // /api/status fault.msg
    uint32_t boxLastHeardMs = 0;      // boxes[0].lastHeardMs
```
At the bottom of `status_model.h` (outside the struct):
```cpp
struct LogEv { int seq; int sev; char msg[48]; };
```
In `cyd-dashboard/main/status_parse.h`, add:
```cpp
// Parse /api/events JSON ({"lastSeq":..,"events":[{seq,t,sev,msg},..]}) into out[],
// oldest->newest, up to maxOut. Returns the number written (0 if none/malformed).
int parseEvents(const char* json, LogEv* out, int maxOut);
```

- [ ] **Step 2: Write the failing tests**

Append to `cyd-dashboard/host_test/test_status_parse.cpp` (and register in `main`):
```cpp
static const char* JSON_DIAG =
  "{\"armed\":false,\"seqRunning\":false,\"lastFailedBox\":null,"
  "\"boxes\":[{\"id\":0,\"linkAlive\":true,\"rssi\":-40,\"state\":0,"
  "\"firedBitmap\":0,\"lastFired\":-1,\"lastHeardMs\":250}],"
  "\"diag\":{\"uptimeMs\":61000,\"freeHeap\":142000,\"apClients\":1,"
  "\"fired\":3,\"acked\":3,\"failed\":1,\"retries\":2,\"lastAckMs\":8},"
  "\"fault\":{\"active\":true,\"msg\":\"box link lost\"}}";

static const char* JSON_EVENTS =
  "{\"lastSeq\":3,\"events\":["
  "{\"seq\":1,\"t\":100,\"sev\":0,\"msg\":\"controller up\"},"
  "{\"seq\":2,\"t\":200,\"sev\":1,\"msg\":\"ARM -> box0\"},"
  "{\"seq\":3,\"t\":300,\"sev\":2,\"msg\":\"FIRE FAILED ch3\"}]}";

void test_parse_diag_fault_lastheard() {
    StatusModel m;
    CHECK(parseStatus(JSON_DIAG, m));
    CHECK_EQ((int)m.diag.uptimeMs, 61000);
    CHECK_EQ((int)m.diag.apClients, 1);
    CHECK_EQ((int)m.diag.failed, 1);
    CHECK_EQ((int)m.diag.lastAckMs, 8);
    CHECK_EQ((int)m.boxLastHeardMs, 250);
    CHECK(m.faultActive);
    CHECK(std::strcmp(m.faultMsg, "box link lost") == 0);
}
void test_parse_events() {
    LogEv evs[8];
    int n = parseEvents(JSON_EVENTS, evs, 8);
    CHECK_EQ(n, 3);
    CHECK_EQ(evs[0].seq, 1);
    CHECK_EQ(evs[0].sev, 0);
    CHECK(std::strcmp(evs[0].msg, "controller up") == 0);
    CHECK_EQ(evs[2].sev, 2);
    CHECK(std::strcmp(evs[2].msg, "FIRE FAILED ch3") == 0);
}
void test_parse_events_empty_and_cap() {
    LogEv evs[2];
    CHECK_EQ(parseEvents("{\"lastSeq\":0,\"events\":[]}", evs, 2), 0);
    CHECK_EQ(parseEvents(JSON_EVENTS, evs, 2), 2);   // capped at maxOut
    CHECK_EQ(parseEvents(nullptr, evs, 2), 0);
}
```
Add `#include <cstring>` at the top of the test if not present, and register:
```cpp
RUN(test_parse_diag_fault_lastheard);
RUN(test_parse_events);
RUN(test_parse_events_empty_and_cap);
```

Run (Git Bash):
```bash
cd cyd-dashboard/host_test && cmake -B build -G Ninja -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++ && cmake --build build
```
Expected: compile/link error — `parseEvents` / new fields undefined.

- [ ] **Step 3: Implement the parser extensions**

In `cyd-dashboard/main/status_parse.cpp`, add `#include <cstdio>` at top. In `parseStatus`, before `return true;`, add diag/fault/lastHeard parsing:
```cpp
    // lastHeardMs is inside the first box object (obj scope)
    if (intAt(valueAfter(obj, "lastHeardMs"), iv)) out.boxLastHeardMs = (uint32_t)iv;

    // diag block (top-level)
    const char* d = std::strstr(json, "\"diag\"");
    if (d) {
        if (intAt(valueAfter(d, "uptimeMs"), iv))  out.diag.uptimeMs  = (uint32_t)iv;
        if (intAt(valueAfter(d, "freeHeap"), iv))  out.diag.freeHeap  = (uint32_t)iv;
        if (intAt(valueAfter(d, "apClients"), iv)) out.diag.apClients = (uint32_t)iv;
        if (intAt(valueAfter(d, "fired"), iv))     out.diag.fired     = (uint32_t)iv;
        if (intAt(valueAfter(d, "acked"), iv))     out.diag.acked     = (uint32_t)iv;
        if (intAt(valueAfter(d, "failed"), iv))    out.diag.failed    = (uint32_t)iv;
        if (intAt(valueAfter(d, "retries"), iv))   out.diag.retries   = (uint32_t)iv;
        if (intAt(valueAfter(d, "lastAckMs"), iv)) out.diag.lastAckMs = (uint32_t)iv;
    }
    // fault block (top-level)
    const char* f = std::strstr(json, "\"fault\"");
    if (f) {
        if (boolAt(valueAfter(f, "active"), bv)) out.faultActive = bv;
        const char* mp = valueAfter(f, "msg");   // points at the opening quote
        if (mp && *mp == '"') {
            ++mp; int k = 0;
            while (*mp && *mp != '"' && k < (int)sizeof(out.faultMsg) - 1) out.faultMsg[k++] = *mp++;
            out.faultMsg[k] = '\0';
        }
    }
```
Add `parseEvents` at the end of the file:
```cpp
int parseEvents(const char* json, LogEv* out, int maxOut) {
    if (!json || maxOut <= 0) return 0;
    const char* p = std::strstr(json, "\"events\"");
    if (!p) return 0;
    p = std::strchr(p, '[');
    if (!p) return 0;
    int n = 0;
    while (n < maxOut) {
        const char* obj = std::strchr(p, '{');
        if (!obj) break;
        // stop if the next '{' is past the array close ']'
        const char* close = std::strchr(p, ']');
        if (close && close < obj) break;
        int iv;
        out[n].seq = intAt(valueAfter(obj, "seq"), iv) ? iv : 0;
        out[n].sev = intAt(valueAfter(obj, "sev"), iv) ? iv : 0;
        out[n].msg[0] = '\0';
        const char* mp = valueAfter(obj, "msg");
        if (mp && *mp == '"') {
            ++mp; int k = 0;
            while (*mp && *mp != '"' && k < (int)sizeof(out[n].msg) - 1) out[n].msg[k++] = *mp++;
            out[n].msg[k] = '\0';
        }
        ++n;
        p = std::strchr(obj, '}');     // advance past this object
        if (!p) break;
        ++p;
    }
    return n;
}
```
(`valueAfter`/`intAt`/`boolAt` are the existing file-static helpers — `parseEvents` is in the same translation unit so it can call them.)

- [ ] **Step 4: Run tests to verify pass**

Run (Git Bash):
```bash
cd cyd-dashboard/host_test && cmake --build build && ctest --test-dir build --output-on-failure
```
Expected: `test_status_parse` PASS (existing 4 + 3 new cases).

- [ ] **Step 5: Commit**

```bash
git add cyd-dashboard/main/status_model.h cyd-dashboard/main/status_parse.h cyd-dashboard/main/status_parse.cpp cyd-dashboard/host_test/test_status_parse.cpp
git commit -m "feat(cyd): parse diag/fault/lastHeard + parseEvents (host-tested)"
```

---

### Task 2: status_client — diag/fault + events poll

**Files:**
- Modify: `cyd-dashboard/main/status_client.h`, `cyd-dashboard/main/status_client.cpp`

**Interfaces:**
- Consumes: extended `parseStatus`, `parseEvents`, `LogEv` (Task 1).
- Produces: `int status_client_poll_events(uint32_t& sinceSeq, LogEv* out, int maxOut)` — GETs `/api/events?since=<sinceSeq>`, parses into `out[]`, advances `sinceSeq` to the max seq seen, returns count. `status_client_poll_once` already fills diag/fault via the extended `parseStatus` (no client change needed for that beyond Task 1).

- [ ] **Step 1: Declare the events poll**

In `cyd-dashboard/main/status_client.h`:
```cpp
#include "status_model.h"   // (already) — LogEv lives here
void status_client_poll_once(StatusModel& model);
int  status_client_poll_events(uint32_t& sinceSeq, LogEv* out, int maxOut);
```

- [ ] **Step 2: Implement the events poll**

In `cyd-dashboard/main/status_client.cpp`, add a URL + the function (reuse the `Accum`/`on_evt` pattern; bump the accumulate buffer for events by using a local larger Accum):
```cpp
static const char* EVENTS_URL = "http://192.168.4.1/api/events";

struct AccumE { char buf[2700]; int len; };
static esp_err_t on_evt_e(esp_http_client_event_t* e) {
    if (e->event_id == HTTP_EVENT_ON_DATA) {
        AccumE* a = (AccumE*)e->user_data;
        int n = e->data_len;
        if (a->len + n > (int)sizeof(a->buf) - 1) n = sizeof(a->buf) - 1 - a->len;
        if (n > 0) { std::memcpy(a->buf + a->len, e->data, n); a->len += n; }
    }
    return ESP_OK;
}

int status_client_poll_events(uint32_t& sinceSeq, LogEv* out, int maxOut) {
    static AccumE acc;
    acc.len = 0;
    char url[80];
    std::snprintf(url, sizeof(url), "%s?since=%lu", EVENTS_URL, (unsigned long)sinceSeq);
    esp_http_client_config_t cfg = {};
    cfg.url = url; cfg.timeout_ms = 800; cfg.event_handler = on_evt_e; cfg.user_data = &acc;
    esp_http_client_handle_t c = esp_http_client_init(&cfg);
    if (!c) return 0;
    esp_err_t err = esp_http_client_perform(c);
    int status = esp_http_client_get_status_code(c);
    esp_http_client_cleanup(c);
    if (err != ESP_OK || status != 200) return 0;
    acc.buf[acc.len] = 0;
    int n = parseEvents(acc.buf, out, maxOut);
    for (int i = 0; i < n; ++i) if ((uint32_t)out[i].seq > sinceSeq) sinceSeq = (uint32_t)out[i].seq;
    return n;
}
```
Add `#include <cstdio>` at the top of `status_client.cpp` if not present.

- [ ] **Step 3: Build**

Run (PowerShell): `& 'C:\esp\v6.0.1\esp-idf\export.ps1'; Set-Location 'C:\Users\zombo\Desktop\Programming\RemoteFireworkCueSystem\RemoteFireworkCueSystem\cyd-dashboard'; idf.py build`
Expected: `Project build complete.`

- [ ] **Step 4: Commit**

```bash
git add cyd-dashboard/main/status_client.h cyd-dashboard/main/status_client.cpp
git commit -m "feat(cyd): poll /api/events incrementally (parse into LogEv[])"
```

---

### Task 3: Enable the XPT2046 touchscreen (navigation input)

**Files:**
- Modify: `cyd-dashboard/main/idf_component.yml`
- Create: `cyd-dashboard/main/touch.h`, `cyd-dashboard/main/touch.cpp`
- Modify: `cyd-dashboard/main/CMakeLists.txt` (add `touch.cpp`)

**Interfaces:**
- Consumes: the LVGL display from `display_init()`.
- Produces: `void touch_init(lv_display_t* disp);` — brings up the XPT2046 on SPI3 and registers it as an LVGL touch input via `lvgl_port_add_touch`.

- [ ] **Step 1: Add the touch component**

In `cyd-dashboard/main/idf_component.yml`, add under `dependencies`:
```yaml
  espressif/esp_lcd_touch_xpt2046: "^1.0.0"
```

- [ ] **Step 2: Implement `touch.cpp`**

`cyd-dashboard/main/touch.h`:
```c
#pragma once
#include "lvgl.h"
#ifdef __cplusplus
extern "C" {
#endif
void touch_init(lv_display_t* disp);
#ifdef __cplusplus
}
#endif
```

`cyd-dashboard/main/touch.cpp`:
```cpp
#include "touch.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_touch_xpt2046.h"
#include "esp_lvgl_port.h"

#define T_SCLK 25
#define T_MOSI 32
#define T_MISO 39
#define T_CS   33
#define T_IRQ  36
#define LCD_H_LANDSCAPE 320
#define LCD_V_LANDSCAPE 240

void touch_init(lv_display_t* disp) {
    spi_bus_config_t bus = {};
    bus.sclk_io_num = T_SCLK;
    bus.mosi_io_num = T_MOSI;
    bus.miso_io_num = T_MISO;
    bus.quadwp_io_num = -1;
    bus.quadhd_io_num = -1;
    bus.max_transfer_sz = 0;
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &bus, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_handle_t tio = NULL;
    esp_lcd_panel_io_spi_config_t iocfg = ESP_LCD_TOUCH_IO_SPI_XPT2046_CONFIG(T_CS);
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI3_HOST, &iocfg, &tio));

    esp_lcd_touch_config_t tcfg = {};
    tcfg.x_max = LCD_H_LANDSCAPE;
    tcfg.y_max = LCD_V_LANDSCAPE;
    tcfg.rst_gpio_num = GPIO_NUM_NC;
    tcfg.int_gpio_num = (gpio_num_t)T_IRQ;
    tcfg.flags.swap_xy = 1;     // match the landscape display rotation
    tcfg.flags.mirror_x = 1;
    tcfg.flags.mirror_y = 0;
    esp_lcd_touch_handle_t tp = NULL;
    ESP_ERROR_CHECK(esp_lcd_touch_new_spi_xpt2046(tio, &tcfg, &tp));

    lvgl_port_touch_cfg_t pt = {};
    pt.disp = disp;
    pt.handle = tp;
    lvgl_port_add_touch(&pt);
}
```
Add `"touch.cpp"` to `cyd-dashboard/main/CMakeLists.txt` SRCS.

- [ ] **Step 3: Build**

Run (PowerShell): build as in Task 2 (pulls `esp_lcd_touch_xpt2046`).
Expected: `Project build complete.` (If `display_init()` doesn't yet return the `lv_display_t*` to a caller, that wiring is Task 5; this task only needs `touch.cpp` to compile — it's not called yet.)

- [ ] **Step 4: Commit**

```bash
git add cyd-dashboard/main/idf_component.yml cyd-dashboard/main/touch.h cyd-dashboard/main/touch.cpp cyd-dashboard/main/CMakeLists.txt
git commit -m "feat(cyd): XPT2046 touch input via esp_lcd_touch + lvgl_port_add_touch"
```

---

### Task 4: Dashboard → 3 tap-paged screens

**Files:**
- Modify: `cyd-dashboard/main/dashboard.h`, `cyd-dashboard/main/dashboard.cpp`

**Interfaces:**
- Consumes: `StatusModel` (with diag/fault), `LogEv` (Task 1).
- Produces: `void dashboard_create(void)` (builds 3 page-containers + tap handler); `void dashboard_update(const StatusModel& m)`; `void dashboard_set_events(const LogEv* evs, int n)` (latest events for the log page + the dashboard's latest-event line).

- [ ] **Step 1: Header**

Replace `cyd-dashboard/main/dashboard.h` with:
```cpp
#pragma once
#include "status_model.h"
void dashboard_create(void);
void dashboard_update(const StatusModel& m);
void dashboard_set_events(const LogEv* evs, int n);
```

- [ ] **Step 2: Pages + tap-cycle in dashboard.cpp**

In `cyd-dashboard/main/dashboard.cpp`, keep the existing palette/helpers/pulse timer and the existing Dashboard widgets, but wrap them in a paging structure. Add at file scope:
```cpp
static lv_obj_t* s_pageDash = nullptr;   // container for the existing dashboard widgets
static lv_obj_t* s_pageLog  = nullptr;
static lv_obj_t* s_pageDiag = nullptr;
static lv_obj_t* s_dots[3]  = {nullptr};
static int s_page = 0;                    // 0=dash,1=log,2=diag
static lv_obj_t* s_logRows[8] = {nullptr};
static lv_obj_t* s_diagText = nullptr;
static lv_obj_t* s_stateBarLog = nullptr; // slim state bar atop log/diag
static lv_obj_t* s_stateBarDiag = nullptr;
static lv_obj_t* s_lastEvtLine = nullptr; // latest-event line on the dashboard
static bool s_faultUnseen = false;

static void show_page(int p) {
    s_page = (p % 3 + 3) % 3;
    lv_obj_add_flag(s_pageDash, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_pageLog,  LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_pageDiag, LV_OBJ_FLAG_HIDDEN);
    lv_obj_t* cur = s_page == 0 ? s_pageDash : s_page == 1 ? s_pageLog : s_pageDiag;
    lv_obj_remove_flag(cur, LV_OBJ_FLAG_HIDDEN);
    for (int i = 0; i < 3; ++i)
        lv_obj_set_style_bg_color(s_dots[i], i == s_page ? lv_color_white() : C_CELLOFF_BD, 0);
    if (s_page == 1) s_faultUnseen = false;   // viewing the log clears the unseen-fault marker
}
static void screen_tap_cb(lv_event_t* /*e*/) { show_page(s_page + 1); }
```

Build the structure in `dashboard_create()` (the existing widget code moves under `s_pageDash`):
- Create the three full-screen containers as children of `lv_screen_active()`, each 320×240, no border/scroll, bg `C_BG`. Parent the existing header/card/stats/bar/grid widgets to `s_pageDash` instead of `scr`.
- **Log page (`s_pageLog`):** a slim state bar `s_stateBarLog` (full width, 22px, top) + 8 `s_logRows[i]` labels (montserrat_14, monospace-ish) stacked below.
- **Diag page (`s_pageDiag`):** a slim state bar `s_stateBarDiag` + one multi-line `s_diagText` label (montserrat_14).
- **Page dots:** three 8px dots bottom-center (`s_dots[0..3]`).
- **Latest-event line `s_lastEvtLine`:** a montserrat_14 label at the very bottom of `s_pageDash`.
- Register the tap handler once: `lv_obj_add_event_cb(lv_screen_active(), screen_tap_cb, LV_EVENT_CLICKED, nullptr);`
- End with `show_page(0);` then the existing `lv_timer_create(...)` and `lvgl_port_unlock()`.

The full `dashboard_create` body (replacing the current one — preserves all existing dashboard widgets, now parented to `s_pageDash`):
```cpp
void dashboard_create(void) {
    if (!lvgl_port_lock(0)) return;
    lv_obj_t* scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, C_BG, 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    auto mkPage = [&](void) {
        lv_obj_t* pg = lv_obj_create(scr);
        lv_obj_set_size(pg, 320, 240);
        lv_obj_set_pos(pg, 0, 0);
        lv_obj_remove_flag(pg, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_bg_color(pg, C_BG, 0);
        lv_obj_set_style_bg_opa(pg, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(pg, 0, 0);
        lv_obj_set_style_pad_all(pg, 0, 0);
        return pg;
    };
    s_pageDash = mkPage();
    s_pageLog  = mkPage();
    s_pageDiag = mkPage();

    // ----- Dashboard page (existing widgets, parented to s_pageDash) -----
    lv_obj_t* parent = s_pageDash;   // (the existing create code below uses `parent` not `scr`)
    // ... [existing header dot/title/link, hero card+state, stats row, bar, grid] ...
    // Replace every `lv_*_create(scr)` in the existing dashboard build with `lv_*_create(parent)`
    // and every `lv_obj_align(x, ALIGN, ...)` stays the same (aligns within the page).
    // Then add the latest-event line:
    s_lastEvtLine = lv_label_create(parent);
    lv_obj_set_style_text_font(s_lastEvtLine, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_lastEvtLine, C_MUTED, 0);
    lv_label_set_long_mode(s_lastEvtLine, LV_LABEL_LONG_DOT);
    lv_obj_set_width(s_lastEvtLine, 300);
    lv_label_set_text(s_lastEvtLine, "");
    lv_obj_align(s_lastEvtLine, LV_ALIGN_BOTTOM_LEFT, 10, -6);

    // ----- Log page -----
    s_stateBarLog = lv_label_create(s_pageLog);
    lv_obj_set_style_text_font(s_stateBarLog, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_stateBarLog, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(s_stateBarLog, LV_OPA_COVER, 0);
    lv_obj_set_width(s_stateBarLog, 320);
    lv_obj_set_style_pad_all(s_stateBarLog, 4, 0);
    lv_obj_align(s_stateBarLog, LV_ALIGN_TOP_MID, 0, 0);
    lv_label_set_text(s_stateBarLog, "EVENTS");
    for (int i = 0; i < 8; ++i) {
        s_logRows[i] = lv_label_create(s_pageLog);
        lv_obj_set_style_text_font(s_logRows[i], &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(s_logRows[i], C_MUTED, 0);
        lv_label_set_long_mode(s_logRows[i], LV_LABEL_LONG_DOT);
        lv_obj_set_width(s_logRows[i], 308);
        lv_label_set_text(s_logRows[i], "");
        lv_obj_align(s_logRows[i], LV_ALIGN_TOP_LEFT, 6, 30 + i * 25);
    }

    // ----- Diag page -----
    s_stateBarDiag = lv_label_create(s_pageDiag);
    lv_obj_set_style_text_font(s_stateBarDiag, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_stateBarDiag, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(s_stateBarDiag, LV_OPA_COVER, 0);
    lv_obj_set_width(s_stateBarDiag, 320);
    lv_obj_set_style_pad_all(s_stateBarDiag, 4, 0);
    lv_obj_align(s_stateBarDiag, LV_ALIGN_TOP_MID, 0, 0);
    lv_label_set_text(s_stateBarDiag, "DIAGNOSTICS");
    s_diagText = lv_label_create(s_pageDiag);
    lv_obj_set_style_text_font(s_diagText, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_diagText, lv_color_white(), 0);
    lv_obj_align(s_diagText, LV_ALIGN_TOP_LEFT, 10, 34);
    lv_label_set_text(s_diagText, "");

    // ----- Page dots (on each page, bottom-center) -----
    for (int i = 0; i < 3; ++i) {
        s_dots[i] = lv_obj_create(scr);
        lv_obj_set_size(s_dots[i], 8, 8);
        lv_obj_remove_flag(s_dots[i], LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_radius(s_dots[i], 4, 0);
        lv_obj_set_style_border_width(s_dots[i], 0, 0);
        lv_obj_set_style_bg_color(s_dots[i], C_CELLOFF_BD, 0);
        lv_obj_align(s_dots[i], LV_ALIGN_BOTTOM_MID, (i - 1) * 14, -2);
    }

    lv_obj_add_event_cb(scr, screen_tap_cb, LV_EVENT_CLICKED, nullptr);
    show_page(0);
    lv_timer_create(pulse_timer_cb, 120, nullptr);
    lvgl_port_unlock();
}
```
(Note: keep all existing widget statics — `s_card`, `s_state`, `s_link`, `s_fired`, `s_last`, `s_bar`, `s_seq`, `s_cellBox[]`, `s_cellNum[]`, `s_dot` — just parent them to `parent` (= `s_pageDash`).)

- [ ] **Step 3: Extend `dashboard_update` + add `dashboard_set_events`**

In `dashboard_update(const StatusModel& m)`, after the existing banner/link/fired/seq/grid updates (still under the one `lvgl_port_lock`), add:
- Update the slim state bars to mirror the banner word + color:
```cpp
    // mirror state onto the Log/Diag slim bars
    lv_label_set_text(s_stateBarLog, word);
    lv_label_set_text(s_stateBarDiag, word);
    lv_obj_set_style_bg_color(s_stateBarLog, top, 0);
    lv_obj_set_style_bg_color(s_stateBarDiag, top, 0);
```
(`word`/`top` are the locals already computed for the banner — keep them in scope.)
- Build the diag text:
```cpp
    char dt[256];
    std::snprintf(dt, sizeof(dt),
        "uptime  %lus\nheap    %luk\nclients %lu\nack     %lums\n"
        "fired   %lu\nacked   %lu\nfailed  %lu\nretries %lu\nbox heard %lums ago",
        (unsigned long)(m.diag.uptimeMs/1000), (unsigned long)(m.diag.freeHeap/1024),
        (unsigned long)m.diag.apClients, (unsigned long)m.diag.lastAckMs,
        (unsigned long)m.diag.fired, (unsigned long)m.diag.acked,
        (unsigned long)m.diag.failed, (unsigned long)m.diag.retries,
        (unsigned long)m.boxLastHeardMs);
    lv_label_set_text(s_diagText, dt);
```
- Fault badge on the Log dot:
```cpp
    if (m.faultActive) s_faultUnseen = (s_page != 1);
    lv_obj_set_style_bg_color(s_dots[1],
        s_faultUnseen ? C_ARMED : (s_page == 1 ? lv_color_white() : C_CELLOFF_BD), 0);
```

Add `dashboard_set_events`:
```cpp
void dashboard_set_events(const LogEv* evs, int n) {
    if (!lvgl_port_lock(0)) return;
    // newest 8, newest at the bottom row
    int show = n < 8 ? n : 8;
    int start = n - show;
    for (int i = 0; i < 8; ++i) {
        if (i < show) {
            const LogEv& e = evs[start + i];
            lv_color_t c = e.sev == 2 ? C_ARMED : e.sev == 1 ? C_AMBER : C_MUTED;
            lv_obj_set_style_text_color(s_logRows[i], c, 0);
            lv_label_set_text(s_logRows[i], e.msg);
        } else {
            lv_label_set_text(s_logRows[i], "");
        }
    }
    if (n > 0) lv_label_set_text(s_lastEvtLine, evs[n - 1].msg);  // latest-event line on the dashboard
    lvgl_port_unlock();
}
```

- [ ] **Step 4: Build**

Run (PowerShell): build as in Task 2.
Expected: `Project build complete.`

- [ ] **Step 5: Commit**

```bash
git add cyd-dashboard/main/dashboard.h cyd-dashboard/main/dashboard.cpp
git commit -m "feat(cyd): 3 tap-paged screens (Dashboard/Log/Diag) + enriched dashboard"
```

---

### Task 5: Wire-up + hardware verification

**Files:**
- Modify: `cyd-dashboard/main/main.cpp`

**Interfaces:**
- Consumes: `display_init` (returns `lv_display_t*`), `touch_init`, `dashboard_*`, `status_client_poll_events`.

- [ ] **Step 1: Final main**

Replace `cyd-dashboard/main/main.cpp` with:
```cpp
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "wifi_sta.h"
#include "display.h"
#include "touch.h"
#include "dashboard.h"
#include "status_client.h"
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
    wifi_sta_start();

    static StatusModel model;
    static LogEv evs[16];
    static uint32_t sinceSeq = 0;
    dashboard_update(model);

    while (true) {
        if (wifi_sta_connected()) {
            status_client_poll_once(model);
            int n = status_client_poll_events(sinceSeq, evs, 16);
            if (n > 0) dashboard_set_events(evs, n);
        } else {
            model.controllerReachable = false;
        }
        dashboard_update(model);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```
(If `display_init()` currently returns `void`, change its signature to return the `lv_display_t*` it already creates — it calls `lvgl_port_add_disp` which returns the display; return that.)

- [ ] **Step 2: Build + flash**

Build (PowerShell) as in Task 2, then flash COM12 (auto-reset):
```powershell
Set-Location build; python -m esptool --chip esp32 -p COM12 -b 460800 --before default-reset --after hard-reset write-flash "@flash_args"
```
Expected: `Project build complete.` + `Hash of data verified.`

- [ ] **Step 3: Hardware verification (user-in-loop)**

With controller (COM11) + box powered:
1. CYD shows the **Dashboard** (state + link + grid + a latest-event line at the bottom).
2. **Tap the screen** → **Event Log** page (slim state bar on top + recent messages, colored). Tap → **Diagnostics** (uptime/heap/clients/ack/counts). Tap → back to Dashboard.
3. Arm/fire from the phone → new messages appear on the Log page; the **dashboard latest-event line** updates.
4. Cause a fault (power off box) → the **Log page-dot turns red**; tapping to the Log page clears the red marker.
Expected: paging is tap-driven only (no auto-change); state visible on every page.

- [ ] **Step 4: Commit**

```bash
git add cyd-dashboard/main/main.cpp
git commit -m "feat(cyd): wire touch + events poll + tap-paged dashboard end-to-end"
```

---

## Self-Review Notes
- **Spec coverage:** event log page (Tasks 1,2,4), diagnostics page (Tasks 1,4), tap-to-page nav touch-only (Tasks 3,4), state always visible (slim bars Task 4), enriched dashboard latest-event + chips (Task 4), fault badge on page dot (Task 4), parser host-tested (Task 1). Read-only firing preserved (touch only switches pages).
- **Type consistency:** `LogEv` identical across model (Task 1), client (Task 2), dashboard (Task 4); `parseEvents`/`status_client_poll_events` signatures match; `dashboard_set_events` matches caller (Task 5).
- **Risk:** Task 4 is a substantial LVGL refactor (parenting existing widgets to a page container) + Task 3 touch bring-up — both verified on hardware in Task 5 (orientation/`swap_xy` touch flags may need a tweak by eye, like the display bring-up did). `display_init` return-type change is noted in Tasks 3/5.
