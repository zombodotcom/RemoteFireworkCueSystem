# CYD ESP-NOW Display Transport

**Date:** 2026-06-24
**Branch:** `cyd-espnow` (off `main`)
**Status:** Design — approved for planning

## Goal

Make the CYD dashboard receive its data over **ESP-NOW broadcast** instead of associating to the controller's Wi-Fi AP and polling HTTP. The CYD (an ESP32 STA) cannot stay associated to the ESP-IDF softAP — PMF/SA-Query kicks it every few seconds, flapping the dashboard SAFE↔NO CTRL. ESP-NOW needs no association, so the problem disappears. This is how the box already talks to the controller.

The dashboard **UI does not change** — only the data source.

## Background / current state

- Box → controller: `fw::StatusPacket` unicast over ESP-NOW every 750 ms (unchanged by this work).
- Controller (`controller/`): Wi-Fi softAP (channel 1, WPA2, PMF required) + HTTP server (`/api/status`, `/api/events`) + ESP-NOW peer for the box. The control loop aggregates box telemetry + diag counters + an event log + a fault code.
- CYD (`cyd-dashboard/`): **today** joins the AP as a STA and HTTP-polls `/api/status` + `/api/events`, filling `StatusModel` / `LogEv[]` which the 3-page LVGL dashboard renders. The association is the problem.
- Confirmed on hardware: with both boards at the original PMF config, the CYD STA still loops SA-Query → disassoc every ~5 s. ESP-IDF softAP does not reliably answer the STA's SA-Query (worsened by softAP + ESP-NOW + box-traffic coexistence). Phones handle PMF fine, so the AP/HTTP path stays for the phone + website.

## Architecture

Two hops, cleanly separated:

```
box  --(StatusPacket, ESP-NOW unicast, 750ms)-->  controller
controller  --(DisplayStatusPacket + DisplayEventPacket, ESP-NOW BROADCAST)-->  CYD display(s)
controller  --(HTTP /api/status, /api/events)-->  phone / website   (unchanged)
```

The controller keeps everything it does and **adds** an ESP-NOW broadcast of the display data. The CYD **stops associating** and becomes an ESP-NOW listener.

### Wire format (new, controller → display)

Added to `components/fireworkcore/include/protocol.h` (single source of truth; the CYD project adds that dir to its include path). New `MsgType`: `DISP_STATUS = 8`, `DISP_EVENT = 9`. No application-level CRC — ESP-NOW already CRC-checks every frame at the MAC layer; the receiver validates by **type byte + exact length**. `#pragma pack(1)`.

```c
struct DisplayStatusPacket {
    uint8_t  type;            // MsgType::DISP_STATUS
    uint8_t  boxState;        // 0 = SAFE, 1 = ARMED
    uint8_t  boxLinkAlive;    // 0/1 (controller<->box link)
    int8_t   rssi;            // box rssi (controller's view)
    uint16_t firedBitmap;
    int8_t   lastFired;       // -1 = none
    uint8_t  seqRunning;      // 0/1
    uint8_t  faultCode;       // 0 none, 1 estop, 2 link, 3 fire-failed
    uint32_t uptimeMs, freeHeap, apClients, fired, acked, failed, retries, lastAckMs;
    uint32_t boxLastHeardMs;
};  // ~45 bytes, well under the 250-byte ESP-NOW limit

struct DisplayEventPacket {
    uint8_t  type;            // MsgType::DISP_EVENT
    uint8_t  sev;             // 0 info / 1 warn / 2 err
    uint32_t seq;             // monotonic; CYD dedups/orders by this
    char     msg[48];
};  // ~54 bytes
```

### Controller changes (`controller/`)

- Add a **broadcast peer** (`FF:FF:FF:FF:FF:FF`, `ifidx = WIFI_IF_AP`, no encrypt) in `espnow_tx`.
- Add `EspNowTransport::sendDisplayStatus(const DisplayStatusPacket&)` and `sendDisplayEvent(const DisplayEventPacket&)` (broadcast).
- In the control loop: build + broadcast a `DisplayStatusPacket` every ~1 s from `g_status`/diag/fault; and broadcast a `DisplayEventPacket` for each event whose `seq` exceeds the last broadcast seq (drain `g_events` since `lastBroadcastSeq`, a few per tick max to bound airtime).
- **Strictly additive** — no change to the firing path, the box ESP-NOW unicast, retry, or the dead-man. The box safely ignores these broadcasts (its receive path validates CRC/type/length and drops them).

### CYD changes (`cyd-dashboard/`) — data source swap, UI unchanged

- **Remove** `wifi_sta.cpp` (associate) and `status_client.cpp` (HTTP) and the JSON `parseStatus`/`parseEvents`.
- **Add** `espnow_rx.{h,cpp}`: Wi-Fi in STA mode but **never `esp_wifi_connect()`**; pin channel 1 (`esp_wifi_set_channel`); `esp_now_init` + a receive callback. The callback validates type+length and decodes `DisplayStatusPacket` → the shared `StatusModel` and `DisplayEventPacket` → a small `LogEv` ring, under a mutex; it timestamps the last status receipt.
- Pure mapping helpers (host-testable): `applyDisplayStatus(const DisplayStatusPacket&, StatusModel&)` and `toLogEv(const DisplayEventPacket&, LogEv&)` — plain field copies.
- `main.cpp`: init display → touch → dashboard → `espnow_rx_start()`; loop renders ~1 s: snapshot the shared model + drain new events, set `model.controllerReachable = (now - lastStatusRecvMs) < 3000`, then `dashboard_update` + `dashboard_set_events`.
- `status_model.h` (`StatusModel`, `LogEv`) and `dashboard.{h,cpp}` are unchanged. Add the fireworkcore include dir to the CYD `idf_component.yml`/CMake so it can `#include "protocol.h"`.

## Data flow

```
controller loop: aggregate -> DisplayStatusPacket (1Hz broadcast); new events -> DisplayEventPacket (broadcast)
CYD esp_now recv cb (wifi task): validate type+len -> decode -> shared StatusModel + LogEv ring [mutex] + lastStatusRecvMs
CYD main loop (~1Hz): snapshot under mutex -> controllerReachable = fresh<3s -> dashboard_update + set_events
```

## Error handling

- CYD on channel 1 with no association; if the controller is off/out of range, no broadcasts arrive → `controllerReachable` goes false after 3 s → dashboard shows **NO CTRL** (clean, no flapping).
- Malformed/foreign ESP-NOW frames (e.g., the box's unicast also seen, or noise): dropped by the type+length check.
- ESP-NOW broadcast is best-effort (no ACK); a dropped status frame just means the next one (1 s later) updates the panel. Events use a monotonic `seq` so the CYD can dedup and detect gaps (it shows what it receives; a missed event is cosmetic).
- Receive callback runs in the Wi-Fi task — it only validates + memcpy + mutex-guarded store; never touches LVGL (the main loop renders).
- Read-only: the CYD has no control path; ESP-NOW in is display data only.

## Testing

- **Host (CTest):** `applyDisplayStatus` and `toLogEv` field-mapping (feed a packet, assert the model). Reuse the cyd-dashboard host-test harness.
- **On hardware:** with box + controller running, the CYD shows live data and stays **steady** (no SA-Query/disconnect loop — the core goal). Fire/arm from the phone → events appear on the CYD Log page. Pull controller power → CYD flips to NO CTRL within ~3 s, recovers when it returns.

## Out of scope

- The phone/website path (still AP + HTTP — unaffected).
- Touch-navigation polish (tap targets) — tracked separately; this spec is the transport only.
- Encryption of the display broadcast (read-only status; ESP-NOW MAC CRC + type/len validation is sufficient).
- Changing the box → controller telemetry (unchanged).
