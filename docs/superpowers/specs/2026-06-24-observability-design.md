# Controller Observability — Event Log + Diagnostics (web + CYD)

**Date:** 2026-06-24
**Branch:** `observability` (off `main`)
**Status:** Design — approved for planning

## Goal

Let the operator *see what the controller is doing* — a live event/message feed, health/diagnostics, link timing, and faults front-and-center — on both the web UI and the CYD, so the system inspires confidence instead of being a black box.

Four things the operator wanted, all surfaced:
1. **Live event log** — timestamped human-readable messages of real actions.
2. **Health / diagnostics** — uptime, free heap, Wi-Fi clients, packet counters.
3. **Timing & link quality** — ACK latency, per-box "last heard," RSSI.
4. **Faults front-and-center** — failed fires, link loss, ESTOP shown as alerts, not buried.

## Background / current state

The controller's single-threaded control loop (`controller/main/controller_main.cpp`) already observes every event: UI commands (ARM/DISARM/ESTOP/STOP/FIRE/RUN) drained from `g_cmdQueue`, ACKs drained from the transport, and box telemetry. `fw::BoxLink` tracks pending FIRE, retries (`maxRetries`, `ackTimeoutMs`), and `lastFailedId()`. `/api/status` today returns `{armed, seqRunning, lastFailedBox, boxes[]}` (per-box state/linkAlive/rssi/firedBitmap/lastFired). The web UI (`webui/`) and CYD (`cyd-dashboard/`) poll `/api/status` ~1 Hz. There is no event history and no controller-health surface yet.

## Architecture

Three layers, built as **three sequenced plans** (clients depend on the controller API landing first):

```
control loop  --log_event()-->  EventLog (ring, mutex)  --GET /api/events?since=N-->  web + CYD
control loop  --counters----->  g_diag / g_status.diag  --GET /api/status (extended)-->  web + CYD
```

### Layer 1 — Controller (foundation)

- **`EventLog`** (`controller/main/event_log.{h,cpp}`) — a **pure, host-testable** ring buffer of the last `EVENT_CAP = 32` events. Each event: `{ uint32_t seq; uint32_t tMs; uint8_t sev; char msg[48]; }`. API:
  - `void push(uint8_t sev, const char* msg, uint32_t tMs)` — assigns the next monotonic `seq`; overwrites oldest when full.
  - `size_t since(uint32_t afterSeq, Event* out, size_t maxOut) const` — copies events with `seq > afterSeq` (oldest→newest), returns count.
  - `uint32_t lastSeq() const`.
  - Severities: `SEV_INFO=0, SEV_WARN=1, SEV_ERR=2`.
  - Pure C++ (no IDF headers) so it host-tests; the device wraps each call in a FreeRTOS mutex (written from the control loop, read from the httpd task).
- **`log_event` helper** in the controller that formats + pushes (under the mutex). Emitted at:
  | When | sev | message |
  |------|-----|---------|
  | boot | info | `controller up, AP <ssid>` |
  | ARM | warn | `ARM -> box0` |
  | DISARM | info | `DISARM` |
  | ESTOP | err | `ESTOP` |
  | sequence start | info | `SEQ start (<n> cues)` |
  | sequence done/stop | info | `SEQ done` / `SEQ stop` |
  | FIRE sent | info | `FIRE ch<c> -> box<b> (id<n>)` |
  | ACK | info | `ACK id<n> (<ms>ms)` |
  | retry | warn | `RETRY ch<c> <r>/<max>` |
  | fire failed | err | `FIRE FAILED ch<c> (no ACK)` |
  | box link down | err | `box<b> link lost` |
  | box link up | info | `box<b> link up` |
- **Diagnostics counters** maintained in the control loop, held in a `Diag` struct (extends the status snapshot): `uptimeMs` (from `esp_timer`), `freeHeap` (`esp_get_free_heap_size`), `apClients` (`esp_wifi_ap_get_sta_list`), `fired`, `acked`, `failed`, `retries`, `lastAckMs` (latency: now − sentAt at ACK). Per-box `lastHeardMs` (age since last StatusPacket; already tracked via `lastStatusMs`).
- **Fault summary** — derived each loop: `fault.active = estopped || anyConfiguredBoxLinkDown || fireFailedSticky` (sticky flag set on a failed fire, cleared on the next successful ARM or FIRE-ACK). `fault.msg` = the reason.

### API additions

- **`GET /api/events?since=<seq>`** → `{ "lastSeq": <int>, "events": [ {"seq":int,"t":int,"sev":0|1|2,"msg":"..."}, ... ] }`. Returns events with `seq > since`, oldest→newest, capped at `EVENT_CAP`. `since` omitted/0 ⇒ the full buffer. Bounded response (~32 × ~70 chars ≈ 2.3 KB; size the buffer accordingly).
- **`GET /api/status`** gains (existing keys unchanged — backward compatible):
  ```json
  "diag": { "uptimeMs":int, "freeHeap":int, "apClients":int,
            "fired":int, "acked":int, "failed":int, "retries":int, "lastAckMs":int },
  "fault": { "active":bool, "msg":"..." }
  ```
  and each `boxes[]` entry gains `"lastHeardMs": int`.

### Layer 2 — Website (`webui/`, Svelte)

- **Log panel** — scrolling, newest-at-bottom, auto-scroll (with a "stick to bottom" that pauses if the user scrolls up), each line colored by severity (info=muted, warn=amber, err=red). Polls `/api/events?since=<maxSeqSeen>` ~1 Hz, appends, caps client-side history (e.g., 200 lines).
- **Diagnostics panel** — stat grid from `/api/status.diag` (+ per-box last-heard / RSSI).
- **Fault banner** — top-of-page red banner when `fault.active`, showing `fault.msg`.
- Connection model extends `live_connection.ts` (the `StatusResponse` interface + a new events fetch); the Sim path provides empty/synthetic events so types hold.

### Layer 3 — CYD (`cyd-dashboard/`, touch-paged)

- **Enable the XPT2046 touchscreen for navigation only** (`esp_lcd_touch_xpt2046` + `lvgl_port_add_touch`). Touch never fires/arms — it only switches pages (read-only-firing guarantee intact). Touch pins: T_CLK 25, T_CS 33, T_MOSI 32, T_MISO 39, T_IRQ 36.
- **Tap anywhere cycles pages:** Dashboard → Event Log → Diagnostics → Dashboard. No auto-rotation, no ticker, no forced flips — the view changes only on a tap. A page-indicator (three dots) shows the current page; the Log dot turns **red** when there is an unseen `err` event.
- **ARMED/SAFE always visible:** the big hero banner on the Dashboard page; a slim color-coded state bar pinned at the top of the Log and Diagnostics pages.
- **Dashboard (enriched):** the current screen **plus** a latest-event line and a few key stat chips (link latency `lastAckMs`, `failed` count, uptime) so the main page is informative on its own. Fired grid sized to fit.
- **Event Log page:** the last ~8 messages, colored by severity.
- **Diagnostics page:** uptime, free heap, Wi-Fi clients, fired/acked/failed/retries, last ACK latency, per-box last-heard + RSSI.
- Polls `/api/status` and `/api/events` (incremental) ~1 Hz; the CYD parser (`status_parse`) gains a small `parseEvents()` (pure, host-tested) and `diag`/`fault`/`lastHeard` fields on `StatusModel`.

## Data flow

```
control loop: on each action -> log_event(sev,msg) [mutex] ; bump counters ; recompute fault
httpd task:   GET /api/events?since=N -> EventLog.since() [mutex] -> JSON
              GET /api/status -> snapshot + diag + fault -> JSON
web + CYD:    poll both ~1Hz, track maxSeq for incremental events, render log/diag/fault
```

## Error handling

- Event ring overflow silently drops the oldest (by design); `seq` stays monotonic so clients detect gaps if they fall behind (they just resync to `lastSeq`).
- Mutex held only for the brief copy/push — never across I/O.
- `/api/events` response is size-bounded; if the formatted JSON would exceed the buffer it is truncated at a whole event with a correct `lastSeq`.
- Observability is **read-only** — it never affects firing, arming, or the dead-man. A logging failure must never block the control loop (best-effort push).
- CYD touch is navigation-only; a touch driver fault degrades to "no paging," not a crash.

## Testing

- **Host (CTest):**
  - `EventLog` (new `controller/host_test/`): seq monotonic; `since()` returns the correct subset; overflow drops oldest and keeps the newest `EVENT_CAP`; `since(lastSeq)` returns empty.
  - CYD `parseEvents()` + extended `parseStatus` (existing `cyd-dashboard/host_test/`): parse an events array and the new `diag`/`fault`/`lastHeard` fields; empty events; malformed.
- **Web:** vitest for the events mapping (append/incremental/dedup by seq) in `live_connection`.
- **On hardware:** drive real actions (arm, fire, pull box power) and confirm the messages, counters, latency, fault banner/badge appear on the web UI and on each CYD page; tap-paging works.

## Decomposition (three plans, in order)

1. **Controller observability infra + API** — `EventLog` (host-tested), `log_event` wiring, diag counters, fault summary, `/api/events`, extended `/api/status`. Build-verified + host tests. *Touches the controller firmware — review carefully; must not alter firing/interlock paths.*
2. **Website observability** — Log panel, Diagnostics panel, fault banner; vitest for events mapping.
3. **CYD observability** — touch enable, tap-to-page (Dashboard/Log/Diag), enriched Dashboard, parser extensions (host-tested).

## Out of scope (YAGNI)

- Persisting events/log to flash or SD (in-RAM ring only).
- Streaming/WebSocket push (polling is sufficient at 1 Hz).
- Touch *control* (arm/fire from the CYD) — navigation only; firing stays with the physical switch + controller.
- Per-channel continuity / battery (separate hardware-gated work).
