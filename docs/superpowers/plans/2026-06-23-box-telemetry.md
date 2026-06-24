# Box Telemetry → Web UI (Tier B) + Link-Loss Verification (Tier A) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Show each firing box's real arming state, link/RSSI, and per-channel fired map on the controller's web UI, then prove the link-loss dead-man on hardware.

**Architecture:** The box autonomously pushes a CRC-protected `StatusPacket` to the controller every ~750 ms (independent of arm state, so telemetry is visible while SAFE). The controller's ESP-NOW RX callback captures the packet plus its RSSI, queues it, and the control loop folds it into the `g_status` snapshot (computing link-alive from freshness). `GET /api/status` gains a `boxes[]` array; the Svelte web app renders a per-box card. Firing and the safety interlock paths are not touched — telemetry is read-only.

**Tech Stack:** ESP-IDF v6.0.1 (C++), portable `fireworkcore` C++ core with host CTest, Vite + Svelte + TypeScript web UI embedded as a gzipped single file.

## Global Constraints

- **ESP-IDF builds run only in PowerShell**, after `& 'C:\esp\v6.0.1\esp-idf\export.ps1'`, then `idf.py build` in the project dir (`firmware` or `controller`).
- **Host core tests run in Git Bash** (MinGW UCRT g++ + CMake + Ninja + CTest).
- **Single-threaded discipline:** ESP-NOW RX callbacks run in the WiFi task — they may ONLY queue data, never touch `BoxController`/`BoxLink`/`ShowRunner`. All consumption happens in the control loop.
- **Never widen the firing or interlock paths.** This feature adds a read-only reporting path only. Do not modify `arming.*`, the FIRE gate, or the heartbeat dead-man.
- **All ESP-NOW packets are CRC-protected** with the existing `crc32` helpers; validate CRC before trusting any received packet.
- `MAX_CHANNELS == 16`. RSSI is a signed dBm value (store `int8_t`).
- **Flashing (this machine):** custom reset env `$env:ESPTOOL_CUSTOM_RESET_SEQUENCE="D0|R1|W0.6|D1|R0|W0.4|D0"` + `python -m esptool ... write-flash "@flash_args"` from the `build` dir; if auto-reset fails (`0x13`), hold BOOT and use `--before no-reset`. Box = COM9, controller = COM11.

---

## File Structure

- `components/fireworkcore/include/protocol.h` — add `MsgType::STATUS` + `StatusPacket` + CRC overloads (Task 1).
- `components/fireworkcore/host_test/test_protocol.cpp` — StatusPacket CRC test (Task 1).
- `components/fireworkcore/include/box_controller.h` / `src/box_controller.cpp` — fired-channel tracking (Task 2).
- `components/fireworkcore/host_test/test_box_controller.cpp` — fired-map tests (Task 2).
- `firmware/main/espnow_link.h` / `espnow_link.cpp` — `sendStatus()` (Task 3).
- `firmware/main/firing_box_main.cpp` — 750 ms status push (Task 3).
- `controller/main/espnow_tx.h` / `espnow_tx.cpp` — accept STATUS, capture RSSI, status queue (Task 4).
- `controller/main/web_server.h` — `BoxTelemetry` + `StatusSnapshot.boxes[]` (Task 5).
- `controller/main/controller_main.cpp` — drain status queue, compute link-alive (Task 5).
- `controller/main/web_server.cpp` — `boxes[]` in `/api/status` JSON (Task 5).
- `webui/src/stores.ts`, `webui/src/core/live_connection.ts`, `webui/src/core/connection.ts` — telemetry data shape + mapping (Task 6).
- `webui/src/components/BoxPanel.svelte`, `ChannelGrid.svelte` — link/RSSI badge + fired cells (Task 6).
- `controller/main/www/index.html.gz` — rebuilt embedded UI (Task 6, git-ignored artifact).

---

### Task 1: Protocol — `StatusPacket` + `MsgType::STATUS`

**Files:**
- Modify: `components/fireworkcore/include/protocol.h`
- Test: `components/fireworkcore/host_test/test_protocol.cpp`

**Interfaces:**
- Produces: `fw::MsgType::STATUS` (value 7); `struct fw::StatusPacket { uint8_t type, boxId, state; uint16_t firedBitmap; uint8_t lastFiredChannel; uint32_t timestamp, crc; }`; `uint32_t fw::computeCrc(const StatusPacket&)`; `bool fw::crcValid(const StatusPacket&)`.

- [ ] **Step 1: Write the failing test**

Add to `components/fireworkcore/host_test/test_protocol.cpp` (new function + register in `main`):

```cpp
void test_status_crc_roundtrip_and_tamper() {
    fw::StatusPacket s{};
    s.type = (uint8_t)fw::MsgType::STATUS;
    s.boxId = 0; s.state = 1; s.firedBitmap = 0b00000101; s.lastFiredChannel = 2; s.timestamp = 10;
    s.crc = fw::computeCrc(s);
    CHECK(fw::crcValid(s));
    s.firedBitmap = 0b00000111;     // tamper after CRC
    CHECK(!fw::crcValid(s));
}
```

In `main()` add: `RUN(test_status_crc_roundtrip_and_tamper);`

- [ ] **Step 2: Run test to verify it fails (won't compile yet)**

Run (Git Bash):
```bash
cd components/fireworkcore/host_test && cmake -B build -G Ninja && cmake --build build
```
Expected: compile error — `StatusPacket` / `MsgType::STATUS` not declared.

- [ ] **Step 3: Implement in `protocol.h`**

Extend the enum:
```cpp
enum class MsgType : uint8_t { FIRE=1, ACK=2, ARM=3, DISARM=4, HEARTBEAT=5, ESTOP=6, STATUS=7 };
```

Inside `#pragma pack(push, 1)` … `#pragma pack(pop)`, after `AckPacket`:
```cpp
struct StatusPacket {
    uint8_t  type;             // MsgType::STATUS
    uint8_t  boxId;
    uint8_t  state;            // 0 = SAFE, 1 = ARMED
    uint16_t firedBitmap;      // bit i set => channel i fired this arm session
    uint8_t  lastFiredChannel; // 0xFF = none
    uint32_t timestamp;        // box ms clock (diagnostic)
    uint32_t crc;
};
```

After the existing CRC helpers:
```cpp
inline uint32_t computeCrc(const StatusPacket& p) {
    return crc32(reinterpret_cast<const uint8_t*>(&p), sizeof(p) - sizeof(p.crc));
}
inline bool crcValid(const StatusPacket& p) { return p.crc == computeCrc(p); }
```

- [ ] **Step 4: Run test to verify it passes**

Run (Git Bash):
```bash
cd components/fireworkcore/host_test && cmake --build build && ctest --test-dir build -R test_protocol --output-on-failure
```
Expected: `test_protocol` PASS.

- [ ] **Step 5: Commit**

```bash
git add components/fireworkcore/include/protocol.h components/fireworkcore/host_test/test_protocol.cpp
git commit -m "feat(core): add StatusPacket + MsgType::STATUS telemetry message"
```

---

### Task 2: BoxController — per-channel fired tracking

**Files:**
- Modify: `components/fireworkcore/include/box_controller.h`, `components/fireworkcore/src/box_controller.cpp`
- Test: `components/fireworkcore/host_test/test_box_controller.cpp`

**Interfaces:**
- Consumes: `fw::StatusPacket` shape (Task 1) — informs what to expose.
- Produces: `uint16_t BoxController::firedChannelMask() const`; `uint8_t BoxController::lastFiredChannel() const`. Mask bit `i` set once channel `i` energizes; both reset on each fresh SAFE→ARMED transition and at `begin()`.

- [ ] **Step 1: Write the failing tests**

Add to `components/fireworkcore/host_test/test_box_controller.cpp` (the file already defines `armedBox()` and `cmd()` helpers):

```cpp
void test_fired_mask_sets_on_fire() {
    FakeChannelDriver drv;
    BoxController b = armedBox(drv, 0);
    CHECK_EQ((int)b.firedChannelMask(), 0);          // nothing fired yet
    b.onCommand(cmd(MsgType::FIRE, 20, 0, 3, 0), 0);
    CHECK_EQ((int)b.firedChannelMask(), (1 << 3));    // bit 3 set
    CHECK_EQ((int)b.lastFiredChannel(), 3);
    b.onCommand(cmd(MsgType::FIRE, 21, 0, 5, 0), 0);
    CHECK_EQ((int)b.firedChannelMask(), (1 << 3) | (1 << 5));
    CHECK_EQ((int)b.lastFiredChannel(), 5);
}

void test_fired_mask_clears_on_new_arm_session() {
    FakeChannelDriver drv;
    BoxController b = armedBox(drv, 0);
    b.onCommand(cmd(MsgType::FIRE, 22, 0, 3, 0), 0);
    CHECK_EQ((int)b.firedChannelMask(), (1 << 3));
    b.setPhysicalSwitch(false, 1);                   // switch off -> SAFE
    b.setPhysicalSwitch(true, 2);                    // switch back on
    b.onCommand(cmd(MsgType::ARM, 99, 0, 0, 200), 2);// new arm session (new nonce)
    CHECK_EQ((int)b.firedChannelMask(), 0);          // map cleared for the new session
    CHECK_EQ((int)b.lastFiredChannel(), 0xFF);
}
```

In `main()` add:
```cpp
RUN(test_fired_mask_sets_on_fire);
RUN(test_fired_mask_clears_on_new_arm_session);
```

- [ ] **Step 2: Run test to verify it fails**

Run (Git Bash):
```bash
cd components/fireworkcore/host_test && cmake --build build
```
Expected: compile error — `firedChannelMask` / `lastFiredChannel` not members.

- [ ] **Step 3: Implement**

In `box_controller.h`, add public methods and private members:
```cpp
    uint16_t firedChannelMask() const { return firedEver_; }
    uint8_t  lastFiredChannel() const { return lastFired_; }
```
```cpp
    uint16_t firedEver_ = 0;
    uint8_t  lastFired_ = 0xFF;
```

In `box_controller.cpp`:

`begin()` — reset the map alongside the boot-safe all-off:
```cpp
void BoxController::begin() {
    firedEver_ = 0;
    lastFired_ = 0xFF;
    deenergizeAll();          // outputs off before anything else (boot-safe)
}
```

`energize()` — record the fired channel:
```cpp
void BoxController::energize(uint8_t ch, uint32_t nowMs) {
    if (ch >= MAX_CHANNELS) return;
    drv_.setChannel(ch, true);
    firing_[ch] = true;
    offAtMs_[ch] = nowMs + cfg_.fireMs;
    firedEver_ |= (uint16_t)(1u << ch);
    lastFired_ = ch;
}
```

`onCommand()` ARM case — clear the map on a fresh arm session only:
```cpp
        case MsgType::ARM: {
            bool wasArmed = arm_.state() == BoxState::ARMED;
            if (arm_.arm(pkt.nonce, nowMs) && !wasArmed) {
                firedEver_ = 0;
                lastFired_ = 0xFF;
            }
            return CommandResult::IGNORED;
        }
```

- [ ] **Step 4: Run tests to verify they pass**

Run (Git Bash):
```bash
cd components/fireworkcore/host_test && cmake --build build && ctest --test-dir build -R test_box_controller --output-on-failure
```
Expected: `test_box_controller` PASS. Also run the full suite to confirm no regressions: `ctest --test-dir build --output-on-failure`.

- [ ] **Step 5: Commit**

```bash
git add components/fireworkcore/include/box_controller.h components/fireworkcore/src/box_controller.cpp components/fireworkcore/host_test/test_box_controller.cpp
git commit -m "feat(core): track per-channel fired bitmap + last-fired channel"
```

---

### Task 3: Box firmware — push `StatusPacket` every ~750 ms

**Files:**
- Modify: `firmware/main/espnow_link.h`, `firmware/main/espnow_link.cpp`, `firmware/main/firing_box_main.cpp`

**Interfaces:**
- Consumes: `fw::StatusPacket` (Task 1); `BoxController::firedChannelMask()`, `lastFiredChannel()`, `state()` (Task 2).
- Produces: `esp_err_t EspNowLink::sendStatus(const fw::StatusPacket&)`; the box transmits a STATUS packet to `board::CONTROLLER_MAC` every ~750 ms.

- [ ] **Step 1: Add `sendStatus` to the link**

In `firmware/main/espnow_link.h`, after `sendAck`:
```cpp
    esp_err_t sendStatus(const fw::StatusPacket& status);
```

In `firmware/main/espnow_link.cpp`, after `sendAck`'s definition:
```cpp
esp_err_t EspNowLink::sendStatus(const fw::StatusPacket& status) {
    return esp_now_send(ctrlMac_, (const uint8_t*)&status, sizeof(status));
}
```

- [ ] **Step 2: Emit status from the control loop**

In `firmware/main/firing_box_main.cpp`, add a timer alongside `lastRxMs` (near line 51):
```cpp
    uint32_t lastStatusMs = 0;
```
Inside the `while (true)` loop, after `box.tick(now);` and before `vTaskDelay(...)`:
```cpp
        // Telemetry push (~750 ms). Read-only; never gates firing/interlock.
        if (now - lastStatusMs >= 750) {
            lastStatusMs = now;
            fw::StatusPacket st{};
            st.type             = (uint8_t)fw::MsgType::STATUS;
            st.boxId            = cfg.boxId;
            st.state            = (box.state() == fw::BoxState::ARMED) ? 1 : 0;
            st.firedBitmap      = box.firedChannelMask();
            st.lastFiredChannel = box.lastFiredChannel();
            st.timestamp        = now;
            st.crc              = fw::computeCrc(st);
            link.sendStatus(st);
        }
```

- [ ] **Step 3: Build the box firmware**

Run (PowerShell):
```powershell
& 'C:\esp\v6.0.1\esp-idf\export.ps1'; Set-Location firmware; idf.py build
```
Expected: `Project build complete.`

- [ ] **Step 4: Commit**

```bash
git add firmware/main/espnow_link.h firmware/main/espnow_link.cpp firmware/main/firing_box_main.cpp
git commit -m "feat(fw): box pushes StatusPacket telemetry every ~750ms"
```

---

### Task 4: Controller — receive STATUS, capture RSSI, queue it

**Files:**
- Modify: `controller/main/espnow_tx.h`, `controller/main/espnow_tx.cpp`

**Interfaces:**
- Consumes: `fw::StatusPacket` (Task 1).
- Produces: `struct EspNowTransport::StatusReport { uint8_t boxId, state, lastFiredChannel; uint16_t firedBitmap; int8_t rssi; }`; `bool EspNowTransport::receiveStatus(StatusReport& out)` (non-blocking, control-loop only).

- [ ] **Step 1: Declare the report struct + queue + accessor**

In `controller/main/espnow_tx.h`, inside `class EspNowTransport`, public section after `receiveAck`:
```cpp
    struct StatusReport {
        uint8_t  boxId;
        uint8_t  state;
        uint8_t  lastFiredChannel;
        uint16_t firedBitmap;
        int8_t   rssi;
    };
    // Non-blocking dequeue of one box status report. Control loop only.
    bool receiveStatus(StatusReport& out);
```
Private section, after `ackQueue_`:
```cpp
    QueueHandle_t statusQueue_ = nullptr;
```

- [ ] **Step 2: Create the queue in `begin()`**

In `controller/main/espnow_tx.cpp`, in `begin()` right after the `ackQueue_` creation:
```cpp
    statusQueue_ = xQueueCreate(8, sizeof(StatusReport));
    if (!statusQueue_) { ESP_LOGE(TAG, "status xQueueCreate failed"); return ESP_ERR_NO_MEM; }
```

- [ ] **Step 3: Handle STATUS in the RX callback (capture RSSI)**

In `controller/main/espnow_tx.cpp`, replace the body of `rxCallback` so it branches on the message type. Note: the `info` param is now used for RSSI.

```cpp
void EspNowTransport::rxCallback(const esp_now_recv_info_t* info,
                                  const uint8_t* data, int len) {
    if (!g_self || len < 1) return;
    uint8_t type = data[0];

    if (type == static_cast<uint8_t>(fw::MsgType::ACK)) {
        if (!g_self->ackQueue_) return;
        if (len < static_cast<int>(sizeof(fw::AckPacket))) return;
        fw::AckPacket ack;
        memcpy(&ack, data, sizeof(ack));
        if (!fw::crcValid(ack)) { ESP_LOGW(TAG, "ACK CRC mismatch - dropped"); return; }
        xQueueSend(g_self->ackQueue_, &ack.responseToId, 0);
        return;
    }

    if (type == static_cast<uint8_t>(fw::MsgType::STATUS)) {
        if (!g_self->statusQueue_) return;
        if (len < static_cast<int>(sizeof(fw::StatusPacket))) return;
        fw::StatusPacket st;
        memcpy(&st, data, sizeof(st));
        if (!fw::crcValid(st)) { ESP_LOGW(TAG, "STATUS CRC mismatch - dropped"); return; }
        StatusReport r{};
        r.boxId            = st.boxId;
        r.state            = st.state;
        r.lastFiredChannel = st.lastFiredChannel;
        r.firedBitmap      = st.firedBitmap;
        r.rssi             = (info && info->rx_ctrl) ? (int8_t)info->rx_ctrl->rssi : 0;
        xQueueSend(g_self->statusQueue_, &r, 0);
        return;
    }
}
```

- [ ] **Step 4: Implement `receiveStatus`**

In `controller/main/espnow_tx.cpp`, after `receiveAck`:
```cpp
bool EspNowTransport::receiveStatus(StatusReport& out) {
    if (!statusQueue_) return false;
    return xQueueReceive(statusQueue_, &out, 0) == pdTRUE;
}
```

- [ ] **Step 5: Build the controller firmware**

Run (PowerShell):
```powershell
& 'C:\esp\v6.0.1\esp-idf\export.ps1'; Set-Location controller; idf.py build
```
Expected: `Project build complete.`

- [ ] **Step 6: Commit**

```bash
git add controller/main/espnow_tx.h controller/main/espnow_tx.cpp
git commit -m "feat(ctrl): receive box STATUS packets + capture RSSI into a queue"
```

---

### Task 5: Controller — fold telemetry into snapshot + `/api/status` JSON

**Files:**
- Modify: `controller/main/web_server.h`, `controller/main/controller_main.cpp`, `controller/main/web_server.cpp`

**Interfaces:**
- Consumes: `EspNowTransport::StatusReport`, `receiveStatus()` (Task 4).
- Produces: `struct BoxTelemetry { volatile bool linkAlive; volatile int8_t rssi; volatile uint8_t state; volatile uint16_t firedBitmap; volatile uint8_t lastFiredChannel; }`; `StatusSnapshot.boxes[2]`; `/api/status` JSON with a `boxes[]` array.

- [ ] **Step 1: Extend the snapshot struct**

In `controller/main/web_server.h`, before `struct StatusSnapshot`:
```cpp
struct BoxTelemetry {
    volatile bool     linkAlive        = false;
    volatile int8_t   rssi             = 0;
    volatile uint8_t  state            = 0;      // 0 = SAFE, 1 = ARMED
    volatile uint16_t firedBitmap      = 0;
    volatile uint8_t  lastFiredChannel = 0xFF;
};
```
Add to `struct StatusSnapshot`, after `lastFailedBox`:
```cpp
    BoxTelemetry boxes[2];
```

- [ ] **Step 2: Drain the status queue + compute link-alive in the control loop**

In `controller/main/controller_main.cpp`, in `app_main()` before the `while (true)` loop:
```cpp
    static uint32_t lastStatusMs[2] = {0, 0};
```
Inside the loop, right after the existing ACK drain (`while (tx.receiveAck(ack_id)) { ... }`):
```cpp
        // Drain box telemetry (WiFi task queued it; consume in control loop only).
        EspNowTransport::StatusReport sr;
        while (tx.receiveStatus(sr)) {
            if (sr.boxId < 2) {
                g_status.boxes[sr.boxId].state            = sr.state;
                g_status.boxes[sr.boxId].firedBitmap      = sr.firedBitmap;
                g_status.boxes[sr.boxId].lastFiredChannel = sr.lastFiredChannel;
                g_status.boxes[sr.boxId].rssi             = sr.rssi;
                lastStatusMs[sr.boxId] = now;
            }
        }
        for (int b = 0; b < 2; b++) {
            g_status.boxes[b].linkAlive =
                (lastStatusMs[b] != 0) && ((now - lastStatusMs[b]) < 1500);
        }
```

- [ ] **Step 3: Emit `boxes[]` in `/api/status`**

In `controller/main/web_server.cpp`, in `handle_status`, enlarge `buf` (the existing local) to `char buf[420];` and replace the single `snprintf` block that builds the JSON with the builder below, which keeps the existing top-level fields exactly and appends the `boxes` array. Build the `lastFailedBox` token first (a `%s` field can't carry both a number and `null`):
```cpp
    char lastBoxTok[8];
    if (lastBox == 0xFF) snprintf(lastBoxTok, sizeof(lastBoxTok), "null");
    else                 snprintf(lastBoxTok, sizeof(lastBoxTok), "%u", lastBox);

    int n = snprintf(buf, sizeof(buf),
        "{\"armed\":%s,\"seqRunning\":%s,\"lastFailedBox\":%s,\"boxes\":[",
        armed ? "true" : "false", seq ? "true" : "false", lastBoxTok);

    for (int b = 0; b < 2; b++) {
        n += snprintf(buf + n, sizeof(buf) - n,
            "%s{\"id\":%d,\"linkAlive\":%s,\"rssi\":%d,\"state\":%u,"
            "\"firedBitmap\":%u,\"lastFired\":%d}",
            b ? "," : "", b,
            g_status.boxes[b].linkAlive ? "true" : "false",
            (int)g_status.boxes[b].rssi,
            (unsigned)g_status.boxes[b].state,
            (unsigned)g_status.boxes[b].firedBitmap,
            (g_status.boxes[b].lastFiredChannel == 0xFF)
                ? -1 : (int)g_status.boxes[b].lastFiredChannel);
    }
    n += snprintf(buf + n, sizeof(buf) - n, "]}");
```

Remove the old `if (lastBox == 0xFF) { snprintf(...) } else { snprintf(...) }` block that previously produced the body, so only this new builder runs. Leave the `httpd_resp_set_type` + `httpd_resp_send(req, buf, ...)` tail unchanged.

- [ ] **Step 4: Build the controller firmware**

Run (PowerShell):
```powershell
& 'C:\esp\v6.0.1\esp-idf\export.ps1'; Set-Location controller; idf.py build
```
Expected: `Project build complete.`

- [ ] **Step 5: Commit**

```bash
git add controller/main/web_server.h controller/main/controller_main.cpp controller/main/web_server.cpp
git commit -m "feat(ctrl): surface box telemetry (state/link/rssi/fired) on /api/status"
```

---

### Task 6: Web UI — per-box telemetry card

**Files:**
- Modify: `webui/src/stores.ts`, `webui/src/core/live_connection.ts`, `webui/src/core/connection.ts`, `webui/src/components/BoxPanel.svelte`, `webui/src/components/ChannelGrid.svelte`
- Rebuild artifact: `controller/main/www/index.html.gz`

**Interfaces:**
- Consumes: `/api/status` `boxes[]` shape (Task 5): `{id, linkAlive, rssi, state, firedBitmap, lastFired}`.
- Produces: `ChannelView.fired: boolean`; `BoxView.linkAlive: boolean`, `BoxView.rssi: number | null`; rendered link/RSSI badge + fired cells.

- [ ] **Step 1: Extend the view types**

In `webui/src/stores.ts`:
```ts
export interface ChannelView { firing: boolean; msLeft: number; fired: boolean; }
export interface BoxView { id: number; switchOn: boolean; armed: boolean; estopped: boolean; canFire: boolean; linkAlive: boolean; rssi: number | null; channels: ChannelView[]; }
```

- [ ] **Step 2: Map real telemetry in `live_connection.ts`**

Replace the `StatusResponse` interface (around line 22) with:
```ts
interface BoxStatus {
  id: number;
  linkAlive: boolean;
  rssi: number;
  state: number;        // 0 = SAFE, 1 = ARMED
  firedBitmap: number;
  lastFired: number;    // -1 = none
}
interface StatusResponse {
  armed: boolean;
  seqRunning: boolean;
  lastFailedBox: number | null;
  boxes?: BoxStatus[];
}
```

Replace `applyStatus` with a version that uses `boxes[]` when present:
```ts
  private applyStatus(s: StatusResponse): void {
    const CHANNELS = 16;
    const src = s.boxes && s.boxes.length
      ? s.boxes
      : [{ id: 0, linkAlive: false, rssi: 0, state: s.armed ? 1 : 0, firedBitmap: 0, lastFired: -1 }];

    const boxes = src.map((b) => {
      const channels = Array.from({ length: CHANNELS }, (_unused, c) => ({
        firing: false,                                   // hardware fires physically; no live "on" feedback
        msLeft: 0,
        fired: (b.firedBitmap & (1 << c)) !== 0,
      }));
      const armed = b.state === 1;
      return {
        id: b.id,
        switchOn: armed,            // proxy: armed implies the physical switch is on
        armed,
        estopped: false,            // no API field; estop only observable on hardware
        canFire: armed && !s.seqRunning,
        linkAlive: b.linkAlive,
        rssi: b.linkAlive ? b.rssi : null,
        channels,
      };
    });

    snapshot.set({ now: Date.now(), seqRunning: s.seqRunning, boxes });
  }
```

- [ ] **Step 3: Keep the Sim producing the new fields**

In `webui/src/core/connection.ts`, the `SimConnection.publish()` builds `BoxView`s. Update the channel push and the box object so the types match (sim has no link/fired telemetry, so use neutral values):

Change the channel push line to:
```ts
        channels.push({ firing: this.rig.channelFiring(b, c), msLeft: this.rig.channelMsLeft(b, c, this.now), fired: this.rig.channelFiring(b, c) });
```
Change the `boxes.push({...})` object to include the new fields:
```ts
      boxes.push({
        id: b, switchOn: this.rig.boxSwitch(b), armed: this.rig.boxState(b) === 1,
        estopped: this.rig.boxEstopped(b), canFire: this.rig.boxCanFire(b, this.now),
        linkAlive: this.connected, rssi: null, channels,
      });
```

- [ ] **Step 4: Render link/RSSI in `BoxPanel.svelte`**

In `webui/src/components/BoxPanel.svelte`, add a link badge to the `<header>` after the `.lamp` span:
```svelte
    <span class="link" class:dead={!box.linkAlive}>
      {box.linkAlive ? (box.rssi !== null ? `${box.rssi} dBm` : "link") : "no link"}
    </span>
```
Add to the `<style>` block:
```css
  .link { padding: 2px 8px; border-radius: 4px; background: #161; color: #fff; font-size: 0.85em; }
  .link.dead { background: #555; color: #bbb; }
```

- [ ] **Step 5: Render fired cells in `ChannelGrid.svelte`**

In `webui/src/components/ChannelGrid.svelte`, add `class:fired={ch.fired}` to the cell button:
```svelte
    <button class="cell" class:firing={ch.firing} class:fired={ch.fired} on:click={() => onFire(i)} title={labels[i] ?? `ch ${i}`}>
```
Add to the `<style>` block (after `.cell.firing`):
```css
  .cell.fired { background: #432; color: #fb8; border-color: #a63; }
```

- [ ] **Step 6: Build the UI and re-embed it**

Run (Git Bash):
```bash
cd webui && npm install && npm run build && gzip -9 -c dist/index.html > ../controller/main/www/index.html.gz
```
Expected: `dist/index.html` produced; `controller/main/www/index.html.gz` updated. Then rebuild the controller so the new UI is embedded (PowerShell):
```powershell
& 'C:\esp\v6.0.1\esp-idf\export.ps1'; Set-Location controller; idf.py build
```
Expected: `Project build complete.`

- [ ] **Step 7: Commit**

```bash
git add webui/src/stores.ts webui/src/core/live_connection.ts webui/src/core/connection.ts webui/src/components/BoxPanel.svelte webui/src/components/ChannelGrid.svelte
git commit -m "feat(web): per-box telemetry card (state, link/RSSI, fired grid)"
```
(Note: `controller/main/www/index.html.gz` is git-ignored — it is a build artifact regenerated at flash time.)

---

### Task 7: Hardware verification (Tier A dead-man + Tier B telemetry)

**Files:** none — this is a live validation against the two ESP32 boards (box = COM9, controller = COM11).

**Interfaces:**
- Consumes: clean flashed firmware from Tasks 3–6.

- [ ] **Step 1: Flash both boards**

Build outputs already exist from Tasks 3–6. Flash each from its `build` dir (PowerShell). Box first (hold BOOT if `0x13`):
```powershell
$env:ESPTOOL_CUSTOM_RESET_SEQUENCE="D0|R1|W0.6|D1|R0|W0.4|D0"
Set-Location firmware/build;   python -m esptool --chip esp32 -p COM9  -b 460800 --before default-reset --after hard-reset write-flash "@flash_args"
Set-Location ../../controller/build; python -m esptool --chip esp32 -p COM11 -b 460800 --before default-reset --after hard-reset write-flash "@flash_args"
```
Expected: `Hash of data verified.` for both. (Box: if `0x13`, hold BOOT and re-run with `--before no-reset`.)

- [ ] **Step 2: Verify telemetry while SAFE**

Join the controller's `FireControl` AP, open `http://192.168.4.1`. Confirm Box 0's card shows **SAFE**, a live **link** badge with an RSSI value (e.g. `-50 dBm`), and an empty fired grid. This proves box-push telemetry works without arming.

- [ ] **Step 3: Verify ARMED echo + fired map**

With the box's GPIO4→GND arm jumper in place, ARM from the web UI. Confirm the card flips to **ARMED**. Fire a channel; confirm that channel's cell turns to the **fired** color and stays lit (per-channel fired map), and `lastFired` tracks it.

- [ ] **Step 4: Verify the link-loss dead-man (Tier A) on hardware**

With the box ARMED and reporting, **cut the controller's power** (unplug COM11). Within ~2 s confirm on the box serial (COM9) that it transitions to **SAFE** (the heartbeat dead-man auto-disarm). Re-power the controller; confirm the box card returns and shows the box (now SAFE, fired map cleared on the next arm). 

To watch the box serial during the cut (PowerShell, separate window):
```powershell
& 'C:\esp\v6.0.1\esp-idf\export.ps1'; Set-Location firmware; idf.py -p COM9 monitor
```
Expected: box logs the disarm / `SAFE` transition within `heartbeatTimeoutMs` (2000 ms) of the controller going dark.

- [ ] **Step 5: Done — no commit (validation only)**

Record the result. If any step fails, capture the serial line and treat it as a bug against the relevant task.

---

## Self-Review Notes

- **Spec coverage:** Armed/SAFE echo (Tasks 2,3,5,6), link-alive + RSSI (Tasks 4,5,6), per-channel fired map (Tasks 2,3,5,6), autonomous box-push transport (Tasks 1,3), StatusPacket/MsgType (Task 1), controller-side RSSI + freshness (Tasks 4,5), backward-compatible `/api/status` (Task 5), Tier A live proof (Task 7). Battery / 2nd box / OTA / shows explicitly out of scope per spec.
- **Type consistency:** `StatusPacket` fields match across protocol (Task 1), box push (Task 3), controller parse (Task 4); `StatusReport` (Task 4) → `BoxTelemetry` (Task 5) → JSON keys `{id,linkAlive,rssi,state,firedBitmap,lastFired}` → `BoxStatus` (Task 6) all align. `firedChannelMask()`/`lastFiredChannel()` named identically in Tasks 2 and 3.
- **Single-threaded discipline:** RX callback (Task 4) only queues; control loop (Task 5) consumes — no core objects touched from the WiFi task.
