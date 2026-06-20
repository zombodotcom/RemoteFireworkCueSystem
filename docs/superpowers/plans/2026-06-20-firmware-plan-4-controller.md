# Firmware Plan 4 — ESP32 Controller (SoftAP + Web UI + ESP-NOW + Sequences)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the ESP32 controller: it hosts a WiFi SoftAP, serves the existing `webui/` app, talks ESP-NOW to both firing boxes (ARM/DISARM/FIRE/HEARTBEAT/ESTOP with ACK + retry), runs saved timed sequences on-device, and persists the show config in NVS — turning the simulator's UI into the real remote.

**Architecture:** Reuse the host-tested `fireworkcore` (`SequenceScheduler`, `protocol.h`, CRC). Add controller-side portable logic — `BoxLink` (per-box ESP-NOW TX with ACK-tracking + retry) and `ShowRunner` (drives `SequenceScheduler` cues to `BoxLink` + emits continuous HEARTBEATs + owns ARM/DISARM/E-STOP) — both host-tested behind transport/clock interfaces. A thin ESP-IDF app wires those to real `esp_now`, an `esp_http_server` serving the embedded `webui/dist` + a small JSON/WebSocket API, and NVS config storage. The web app's existing `SystemConnection` interface gains a `LiveConnection` talking to this API (no UI redesign).

**Tech Stack:** ESP-IDF v6.0.1 (`idf.py`, **PowerShell only**), C++11, IDF `esp_wifi`/`esp_now`/`esp_http_server`/`nvs_flash`/`json` (cJSON). Portable controller logic host-tested with CMake/Ninja/g++ + CTest (Git Bash). Web: the existing Vite/Svelte `webui/` (Plans A/B), built to static files and embedded.

## Global Constraints

- Reuse `fireworkcore` unchanged (`SequenceScheduler`, `CommandPacket`/`AckPacket`/`MsgType`, `crcValid`/`computeCrc`, `channelInRange`). New portable controller code is framework-free C++11; new IDF code lives in `controller/`.
- The controller is the only device that runs sequences. It sends **continuous HEARTBEATs** (every ~500ms) to both boxes while connected/armed, individual **FIRE** cues on schedule, and **ARM/DISARM/ESTOP** broadcasts. It tracks ACKs and **retries** an un-ACKed FIRE up to N times with the SAME message id (so the box dedups duplicates).
- **E-STOP is authoritative and immediate:** sends ESTOP to both boxes and halts any running sequence.
- **Heartbeat/arm gating mirrors the spec:** the phone heartbeats the controller; if the phone drops while idle+armed, the controller disarms (stops heartbeating the boxes → box dead-man disarms). A *committed* sequence runs to completion on the controller regardless of the phone; the physical box switches remain the ultimate kill.
- Config is the SAME JSON schema as `webui/src/lib/config.ts` (spec §5); stored in NVS; `ShowRunner` consumes the expanded flat `[timeMs,boxId,channel]` triples (group expansion happens in the web app before upload, OR in a small on-device expander — this plan expands in the web app and uploads flat sequences, matching `expandSequence`).
- SoftAP: SSID/password from a config header; WPA2; offline (no STA/router). Decide a UI passcode gate (spec open item) — default: WPA2 password is the gate; no separate UI passcode in this plan.
- **Toolchain:** `idf.py` in **PowerShell** (`& 'C:\esp\v6.0.1\esp-idf\export.ps1'`); host tests in Git Bash + ctest; web build in Git Bash (`npm run build`). Flashing: user runs `idf.py -p COMx flash monitor`.
- Build artifacts (`controller/build`, `sdkconfig`, `managed_components`, `webui/dist`) git-ignored.
- **Verification ceiling:** Tasks 1–2 host-tested; Tasks 3–6 build-only until an ESP32 is connected, then flash-and-observe. Until a real firing box exists/flashed, the controller can be exercised against the **browser simulator acting as the box** (loopback) or a second ESP32 running Plan 3 firmware.

---

## File Structure

```
components/fireworkcore/include/box_link.h        NEW: Transport iface + BoxLink (TX/ACK/retry) — portable
components/fireworkcore/src/box_link.cpp           NEW
components/fireworkcore/include/show_runner.h      NEW: ShowRunner (sequence + heartbeat + arm/estop) — portable
components/fireworkcore/src/show_runner.cpp        NEW
components/fireworkcore/host_test/test_box_link.cpp     NEW
components/fireworkcore/host_test/test_show_runner.cpp  NEW
components/fireworkcore/host_test/CMakeLists.txt   MODIFY
controller/CMakeLists.txt                          NEW: IDF project
controller/sdkconfig.defaults                      NEW (AP + http server tuning)
controller/.gitignore                              NEW
controller/main/CMakeLists.txt                     NEW (REQUIRES fireworkcore esp_wifi esp_now esp_http_server nvs_flash json)
controller/main/controller_config.h                NEW: SSID/pw, box MACs, timings
controller/main/espnow_tx.h/.cpp                   NEW: real Transport impl over esp_now (+ ACK RX)
controller/main/config_store.h/.cpp                NEW: NVS load/save of the show JSON blob
controller/main/web_server.h/.cpp                  NEW: esp_http_server serving webui + JSON API
controller/main/controller_main.cpp                NEW: app_main wiring
controller/main/www/                                NEW: embedded built web assets (generated from webui/dist)
webui/src/core/live_connection.ts                  NEW: LiveConnection implements SystemConnection (HTTP/WS)
webui/scripts/embed-www.sh                          NEW: build webui + copy dist into controller/main/www
```

---

## Task 1: BoxLink — ESP-NOW TX with ACK tracking + retry (host-tested)

**Files:**
- Create: `components/fireworkcore/include/box_link.h`, `src/box_link.cpp`
- Create: `components/fireworkcore/host_test/test_box_link.cpp`
- Modify: `components/fireworkcore/host_test/CMakeLists.txt`

**Interfaces:**
- Produces:
  - `class fw::Transport { virtual void send(uint8_t boxId, const CommandPacket&)=0; virtual ~Transport(){} };`
  - `struct fw::BoxLinkConfig { uint32_t ackTimeoutMs=120; uint8_t maxRetries=3; };`
  - `class fw::BoxLink` with: `BoxLink(Transport&, BoxLinkConfig=...);` `uint32_t fire(uint8_t boxId, uint8_t channel, uint32_t nowMs);` (returns the msg id used) `void arm(uint32_t nonce,uint32_t nowMs);` `void disarm(uint32_t);` `void estop(uint32_t);` `void heartbeat(uint32_t);` `void onAck(uint32_t responseToId, uint32_t nowMs);` `void tick(uint32_t nowMs);` (resends un-ACKed FIREs whose ackTimeout elapsed, up to maxRetries) `bool pendingAck() const;` `uint32_t lastFailedId() const;` (an id that exhausted retries, 0 if none).

**Behavior contract (tests):** a `fire` sends once and marks the id pending; `onAck` clears it; if no ACK within `ackTimeoutMs`, `tick` resends the SAME id (so the box dedups); after `maxRetries` resends with no ACK, the id is dropped to `lastFailedId` and pending clears; ARM/DISARM/ESTOP/HEARTBEAT send immediately (no ACK tracking).

- [ ] **Step 1: Write failing tests** — `test_box_link.cpp` with a `FakeTransport` recording `(boxId, CommandPacket)` sends:
```cpp
#include "check.h"
#include "box_link.h"
#include <vector>
using namespace fw;
struct Sent { uint8_t box; CommandPacket pkt; };
struct FakeTransport : public Transport {
    std::vector<Sent> sent;
    void send(uint8_t boxId, const CommandPacket& p) override { sent.push_back({boxId, p}); }
};
static int countType(FakeTransport& t, MsgType ty){int n=0;for(auto&s:t.sent)if((MsgType)s.pkt.type==ty)n++;return n;}

void test_fire_sends_once_and_pends() {
    FakeTransport t; BoxLink link(t);
    uint32_t id = link.fire(0, 3, 0);
    CHECK_EQ(countType(t, MsgType::FIRE), 1);
    CHECK(link.pendingAck());
    CHECK(id != 0);
}
void test_ack_clears_pending() {
    FakeTransport t; BoxLink link(t);
    uint32_t id = link.fire(0, 3, 0);
    link.onAck(id, 10);
    CHECK(!link.pendingAck());
}
void test_resend_same_id_after_timeout() {
    FakeTransport t; BoxLink link(t);            // default ackTimeoutMs=120
    uint32_t id = link.fire(0, 3, 0);
    link.tick(50);  CHECK_EQ(countType(t, MsgType::FIRE), 1);   // not yet
    link.tick(130); CHECK_EQ(countType(t, MsgType::FIRE), 2);   // resent
    CHECK_EQ(t.sent.back().pkt.id, id);                          // SAME id (box dedups)
}
void test_gives_up_after_max_retries() {
    FakeTransport t; BoxLink link(t);            // maxRetries=3
    uint32_t id = link.fire(0, 3, 0);
    uint32_t now = 0;
    for (int i=0;i<5;i++){ now += 130; link.tick(now); }
    CHECK_EQ(countType(t, MsgType::FIRE), 4);    // 1 initial + 3 retries
    CHECK(!link.pendingAck());
    CHECK_EQ(link.lastFailedId(), id);
}
void test_control_msgs_send_immediately() {
    FakeTransport t; BoxLink link(t);
    link.arm(99, 0); link.disarm(0); link.estop(0); link.heartbeat(0);
    CHECK_EQ(countType(t, MsgType::ARM), 1);
    CHECK_EQ(countType(t, MsgType::DISARM), 1);
    CHECK_EQ(countType(t, MsgType::ESTOP), 1);
    CHECK_EQ(countType(t, MsgType::HEARTBEAT), 1);
}
int main(){RUN(test_fire_sends_once_and_pends);RUN(test_ack_clears_pending);RUN(test_resend_same_id_after_timeout);RUN(test_gives_up_after_max_retries);RUN(test_control_msgs_send_immediately);return REPORT();}
```
Register in `host_test/CMakeLists.txt`: add `box_link.cpp` (and later `show_runner.cpp`) to `CORE_SRC`; add `add_executable(test_box_link test_box_link.cpp ${CORE_SRC})` + `add_test`.

- [ ] **Step 2: Verify RED** (Git Bash): `cmake -S components/fireworkcore/host_test -B build/host_test -G Ninja -DCMAKE_CXX_COMPILER=g++ && cmake --build build/host_test` → fails (box_link.h missing).

- [ ] **Step 3: Implement `box_link.h`/`.cpp`** — a single-FIRE-in-flight design (one pending id; controller fires cues sequentially fast enough that 1-deep is fine; if a sequence needs overlap, the ShowRunner serializes through this). Build each `CommandPacket` with `computeCrc`, monotonically increasing `nextId_` (start 1), set `cmd`="FIRE"/"ARM"/etc. for log readability, fill `boxId`/`targetChannel`/`nonce`. Track `pendingId_`, `pendingBox_`, `pendingCh_`, `sentAtMs_`, `retries_`, `lastFailedId_`. `tick`: if pending and `now - sentAtMs_ >= ackTimeoutMs`: if `retries_ < maxRetries` resend same id, `retries_++`, `sentAtMs_=now`; else `lastFailedId_=pendingId_`, clear pending.

- [ ] **Step 4: Verify GREEN** (Git Bash): `cmake --build build/host_test && ctest --test-dir build/host_test -R test_box_link --output-on-failure` → 5 tests pass; full suite green.

- [ ] **Step 5: Commit** — `git commit -m "feat(ctrl): BoxLink esp-now TX with ACK tracking + retry (host-tested)"`.

---

## Task 2: ShowRunner — sequence + heartbeat + arm/E-STOP (host-tested)

**Files:**
- Create: `components/fireworkcore/include/show_runner.h`, `src/show_runner.cpp`
- Create: `components/fireworkcore/host_test/test_show_runner.cpp`
- Modify: `host_test/CMakeLists.txt`

**Interfaces:**
- `class fw::ShowRunner` wrapping a `SequenceScheduler` + a `BoxLink&`:
  - `ShowRunner(BoxLink&, uint32_t heartbeatPeriodMs=500);`
  - `void arm(uint32_t nowMs);` (generates a fresh nonce, calls link.arm) `void disarm(uint32_t);` `void estop(uint32_t);` (link.estop + stop sequence)
  - `void fireManual(uint8_t boxId, uint8_t channel, uint32_t nowMs);` (armed-only)
  - `void loadSequence(const SeqStep* steps, size_t count);` `void startSequence(uint32_t nowMs);` `void stopSequence(uint32_t nowMs);` `bool sequenceRunning() const;`
  - `void tick(uint32_t nowMs);` — emits HEARTBEAT every `heartbeatPeriodMs` while armed; pulls due sequence steps and `fire`s them via BoxLink; calls `link.tick`.
  - `bool armed() const;`

**Behavior contract (tests):** tick emits a heartbeat at the configured cadence while armed (not while disarmed); starting a loaded sequence fires its cues in order at their times via the link; E-STOP stops the sequence and sends ESTOP; disarm stops heartbeats. Use a `FakeTransport` (from Task 1's pattern, shared via a small header or duplicated) and inspect sends.

- [ ] **Step 1: Write failing tests** covering: heartbeat-cadence-while-armed; no-heartbeat-while-disarmed; sequence fires cues in order (assert FIRE sends with expected channels at expected ticks); estop sends ESTOP + sequenceRunning()==false; manual fire only when armed. (Mirror the rig/sequence tests' structure.)

- [ ] **Step 2: Verify RED**, **Step 3: Implement**, **Step 4: Verify GREEN** (full suite), **Step 5: Commit** — `feat(ctrl): ShowRunner sequence+heartbeat+arm/estop (host-tested)`.

> After Tasks 1–2, all controller decision logic is host-verified. Tasks 3–6 are IDF wiring (build-only, then flash).

---

## Task 3: IDF controller project + NVS config store + real esp_now Transport (build-only)

**Files:**
- Create: `controller/CMakeLists.txt`, `sdkconfig.defaults`, `.gitignore`, `main/CMakeLists.txt`, `main/controller_config.h`
- Create: `controller/main/config_store.h/.cpp` (NVS blob load/save of the show JSON)
- Create: `controller/main/espnow_tx.h/.cpp` (`class EspNowTransport : public fw::Transport` + ACK RX callback → `BoxLink::onAck`)
- Create: `controller/main/controller_main.cpp` (skeleton: init wifi AP, esp_now, construct BoxLink+ShowRunner, tick loop — no web server yet)

**Interfaces:** `EspNowTransport::send` looks up the box MAC by `boxId` (from `controller_config.h` MAC table) and `esp_now_send`s the packet; its RX callback parses `AckPacket` (length+CRC) and routes `responseToId` to `BoxLink::onAck`. `ConfigStore::load(std::string&)` / `save(const std::string&)` store the show JSON under an NVS key.

- [ ] **Step 1** `controller/CMakeLists.txt` (EXTRA_COMPONENT_DIRS=../components, project `controller`), `main/CMakeLists.txt` REQUIRES `fireworkcore esp_wifi esp_now esp_http_server nvs_flash json esp_timer log`, `sdkconfig.defaults` (`CONFIG_ESP_WIFI_SOFTAP_SUPPORT=y`, raise `CONFIG_HTTPD_MAX_REQ_HDR_LEN`, `CONFIG_ESP_TASK_WDT_INIT=y`), `.gitignore` (build/, sdkconfig, managed_components/, www/* generated).
- [ ] **Step 2** `controller_config.h`: AP SSID/password, WiFi channel (1), `BOX_MAC[2][6]` table, heartbeat/ack timings.
- [ ] **Step 3** `config_store.cpp`: `nvs_open`, `nvs_get_blob`/`set_blob` under key `show_cfg`; return a default (empty channels labels) if absent.
- [ ] **Step 4** `espnow_tx.cpp`: init peers for both box MACs; `send(boxId,pkt)` → `esp_now_send(BOX_MAC[boxId], …)`; RX cb validates `AckPacket` and calls a registered `onAck`.
- [ ] **Step 5** `controller_main.cpp` skeleton: `nvs_flash_init`; AP up; `EspNowTransport` + `BoxLink` + `ShowRunner`; a FreeRTOS task ticking `runner.tick(now)` at 20ms.
- [ ] **Step 6** Build for esp32 (PowerShell `export.ps1` + `idf.py build`) — verify it links. Commit `feat(ctrl): IDF controller skeleton — AP, esp_now transport, NVS config (build-only)`.

---

## Task 4: Web server serving the UI + JSON API (build-only)

**Files:**
- Create: `controller/main/web_server.h/.cpp`
- Create: `webui/scripts/embed-www.sh`
- Modify: `controller/main/controller_main.cpp` (start the server), `controller/main/CMakeLists.txt` (EMBED_FILES the www assets)

**Interfaces:** `WebServer::start(ShowRunner&, ConfigStore&)` registers `esp_http_server` handlers:
- `GET /` and static assets → serve the embedded `webui/dist` (gzip the bundle; set content-types).
- `GET /api/config` → current show JSON; `POST /api/config` → validate + `ConfigStore::save` + reload.
- `POST /api/arm` / `/api/disarm` / `/api/estop` / `/api/fire` (body `{box,channel}`) / `/api/run` (body `{steps:[[t,box,ch]…]}`) / `/api/stop` → drive `ShowRunner`.
- `POST /api/heartbeat` (phone keep-alive) → refresh the phone-alive timer; if it lapses while idle+armed, the controller disarms.
- `GET /api/status` (or a WebSocket `/ws`) → JSON snapshot (armed, per-box online/last-ACK, sequenceRunning) for the UI to poll/subscribe.

- [ ] **Step 1** `embed-www.sh`: `cd webui && npm run build` then copy `webui/dist/*` into `controller/main/www/` (flattened, gzipped). Run it (Git Bash). 
- [ ] **Step 2** `main/CMakeLists.txt`: `EMBED_FILES` (or `target_add_binary_data`) the www assets so they're linked into flash.
- [ ] **Step 3** Implement `web_server.cpp` handlers calling into `ShowRunner`/`ConfigStore`; parse JSON bodies with cJSON; the `/api/run` handler loads the flat triples into `ShowRunner` and starts it.
- [ ] **Step 4** Start the server in `controller_main.cpp` after AP is up.
- [ ] **Step 5** Build for esp32 (PowerShell) — verify it links with embedded assets. Commit `feat(ctrl): http server serving webui + JSON control API (build-only)`.

---

## Task 5: LiveConnection in the web app (host/build-testable)

**Files:**
- Create: `webui/src/core/live_connection.ts`
- Modify: `webui/src/core/connection.ts` (export a factory choosing Sim vs Live), maybe `App.svelte` (a transport toggle or auto-detect)

**Interfaces:** `class LiveConnection implements SystemConnection` — the same interface `SimConnection` implements, but each method `fetch`es the controller API (`/api/arm`, `/api/fire`, …), posts a periodic heartbeat, and polls `/api/status` (or opens `/ws`) to update the `snapshot` store. Because the UI already targets `SystemConnection`, no component changes are required beyond choosing which connection to instantiate (e.g. Sim when served by Vite dev, Live when served from the controller — detect by `location.origin` or a build flag).

- [ ] **Step 1** Write `live_connection.ts` mapping each `SystemConnection` method to an API call; a 1s heartbeat `setInterval`; a status poller updating `snapshot`.
- [ ] **Step 2** Add a factory `createConnection()` that returns `SimConnection` under Vite dev (`import.meta.env.DEV`) and `LiveConnection` otherwise; wire `App.svelte`/`ControllerPanel` to use it.
- [ ] **Step 3** Verify `npm run build` + `npm run test` still green (Vitest doesn't exercise Live; keep Sim tests). Optionally add a Vitest test for `live_connection` against a mocked `fetch`.
- [ ] **Step 4** Re-run `embed-www.sh` so the controller embeds the updated bundle. Commit `feat(web): LiveConnection transport + Sim/Live factory`.

---

## Task 6: Flash + end-to-end bring-up

> **Hardware:** the controller ESP32; plus either a Plan-3 firing box (LEDs, no pyro) OR run with no box to verify AP+UI+commands (ACKs will simply time out and surface as "box offline").

- [ ] **Step 1** User flashes (PowerShell): `idf.py -p COMx flash monitor`. Confirm serial shows AP up + the controller MAC (record it into the box's `board_config.h` if pairing a real box).
- [ ] **Step 2** Phone/laptop joins the `FireControl` SoftAP; browse to the controller IP (`192.168.4.1`). Confirm the web UI loads from flash and `/api/status` responds.
- [ ] **Step 3** With a Plan-3 box powered (LEDs): in the UI, set the box's arm switch on, ARM → box LED green; fire a channel → box output LED pulses + UI shows ACK; build+run a small sequence → cues fire in order; E-STOP → box red-blink + UI reflects; pull the phone off WiFi while idle+armed → controller stops heartbeating → box disarms within ~2s.
- [ ] **Step 4** Without a box: confirm commands are accepted, ACKs time out, and the UI shows the box offline (BoxLink `lastFailedId`/status). 
- [ ] **Step 5** Document the bring-up log; commit any fixes. (Live pyro remains gated behind a written pre-show checklist — out of scope.)

---

## Done criteria

- Host suite passes incl. `test_box_link` + `test_show_runner` (controller decision logic verified off-target).
- `idf.py build` (esp32) links the controller with embedded web UI.
- Flashed: SoftAP serves the UI; arm/fire/sequence/E-STOP drive a real box (LED bench) with ACK + retry; phone-drop disarms an idle armed system; sequences run autonomously.
- Config persists in NVS and round-trips through the UI; the show JSON schema matches `webui/src/lib/config.ts`.

## Notes / open items
- Confirm SoftAP SSID/password and whether to add a UI passcode beyond WPA2 (spec open item) — default here: WPA2 only.
- Sequence overlap: BoxLink is 1-deep on ACK tracking; if a show needs many near-simultaneous cross-box cues with strict ACK on each, consider per-box pending slots. For typical cue spacing this is unnecessary.
- DRY/fidelity: once Plan 3 + this exist, consider the sim rig reusing `BoxController`, and the controller's `ShowRunner` being exactly what the simulator's "controller" models — one source of truth end to end.
