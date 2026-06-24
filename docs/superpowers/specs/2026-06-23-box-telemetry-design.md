# Box Telemetry → Web UI (Tier B) + Link-Loss Verification (Tier A)

**Date:** 2026-06-23
**Branch:** `32ch-webui-show`
**Status:** Design — approved for planning

## Goal

Give the operator real-time visibility of each firing box from the phone (the
controller's web UI), and prove the already-implemented link-loss dead-man on
real hardware.

Three telemetry fields, all with **zero extra hardware**:

1. **Armed / SAFE state** — the box's *real* arming state, echoed back (not what
   the controller assumes it sent).
2. **Link alive + RSSI** — is the box reachable now, and how strong is the radio.
3. **Per-channel fired map** — which channels have fired this arm session.

Battery voltage is explicitly **out of scope** here (needs a voltage divider +
ADC; deferred to the hardware tier). A second box is out of scope (already
configurable via the null-peer-skip in `BOX_MAC`).

## Background / current state

- Boxes run ESP-IDF firmware in WiFi **STA** mode, pinned to channel 1, paired to
  the controller's SoftAP via ESP-NOW. The box already has the controller as an
  ESP-NOW peer (that is how FIRE ACKs return today).
- `ShowRunner::tick()` emits a HEARTBEAT every `HEARTBEAT_MS` (500 ms) **only
  while armed**. The box dead-man (`ArmingStateMachine`, host-tested) auto-disarms
  (`goSafe()`) when no heartbeat arrives within `heartbeatTimeoutMs` (2000 ms).
  Because the controller is silent while SAFE, the box is radio-silent when idle —
  so telemetry needs its own path to be visible during setup.
- The web UI is a single embedded `index.html` (gzip, served from flash). The
  controller exposes `GET /api/status` returning `{armed, seqRunning, lastFailedBox}`.
- Protocol (`components/fireworkcore/include/protocol.h`): `MsgType { FIRE=1,
  ACK=2, ARM=3, DISARM=4, HEARTBEAT=5, ESTOP=6 }`. `MAX_CHANNELS = 16`.

## Architecture

### Transport: autonomous box-push

The box sends a small CRC-protected `StatusPacket` to the controller every
**~750 ms**, independent of arm state. Chosen over controller-poll because:

- It works while **SAFE** (setup time) — the controller's heartbeat only flows
  while armed.
- It keeps telemetry **off** the safety-critical armed-heartbeat path, so the
  interlock code is untouched.
- The box already peers with the controller, so it is a minimal addition.

### New protocol message

Add to `MsgType`: `STATUS = 7`.

```c
#pragma pack(push, 1)
struct StatusPacket {
    uint8_t  type;             // MsgType::STATUS
    uint8_t  boxId;
    uint8_t  state;            // 0 = SAFE, 1 = ARMED
    uint16_t firedBitmap;      // bit i set => channel i fired this arm session
    uint8_t  lastFiredChannel; // 0xFF = none
    uint32_t timestamp;        // box ms clock (diagnostic only)
    uint32_t crc;
};
#pragma pack(pop)
```

CRC computed/validated with the existing `crc32` helpers, matching the
`CommandPacket` / `AckPacket` pattern (`computeCrc` / `crcValid` overloads).

### Field derivation

| Field | Source | Computed by |
|-------|--------|-------------|
| Armed/SAFE | `box.state()` | box |
| firedBitmap | new `BoxController::firedChannelMask()` | box |
| RSSI | `esp_now_recv_info_t::rx_ctrl->rssi` on each StatusPacket | controller |
| linkAlive | "StatusPacket received < ~1.5 s ago" | controller |

## Components & changes

### 1. `components/fireworkcore` (portable, host-tested)
- `protocol.h`: add `MsgType::STATUS`, `StatusPacket`, `computeCrc`/`crcValid`
  overloads.
- `BoxController`: add `uint16_t firedEver_` bitmask. Set bit in `energize()`.
  Clear in `begin()` and on each successful new arm session (so the map reflects
  the current session, not stale history). Expose `uint16_t firedChannelMask()
  const` and `uint8_t lastFiredChannel() const`.
- Unit tests (CTest): firedChannelMask reflects energized channels; clears on new
  arm session; STATUS packet CRC round-trips; STATUS is IGNORED if received by a
  box (defensive — boxes never consume STATUS).

### 2. Box firmware (`firmware/main`)
- In the control loop, every ~750 ms build and send a `StatusPacket` (state +
  firedChannelMask + lastFiredChannel) to `board::CONTROLLER_MAC` via the existing
  ESP-NOW link send path. No change to the firing or interlock paths.

### 3. Controller firmware (`controller/main`)
- `espnow_tx.cpp` RX callback: also accept `STATUS` packets (length + type +
  CRC checks). Read `info->rx_ctrl->rssi`. Post `{boxId, state, firedBitmap,
  lastFiredChannel, rssi}` onto a new status queue (separate from the ACK queue).
- `controller_main.cpp` control loop: drain the status queue into `g_status`;
  compute `linkAlive` per box as `(now - lastStatusMs) < 1500`.
- `web_server.h`: extend `StatusSnapshot` with a per-box array
  `BoxTelemetry boxes[2] { bool linkAlive; int8_t rssi; uint8_t state;
  uint16_t firedBitmap; uint8_t lastFiredChannel; }`.
- `web_server.cpp` `handle_status`: add a `boxes[]` array to the JSON. Existing
  top-level fields unchanged (backward compatible).

### 4. Web UI (`index.html`)
- Poll `GET /api/status` ~every 1 s.
- Render a per-box **status card**:
  - ARMED/SAFE badge from the box's echoed `state` (green SAFE / red ARMED),
    visually distinct from the controller's own arm control.
  - Link indicator: solid dot + `RSSI dBm` when `linkAlive`, greyed "no link"
    when stale.
  - 16-cell fired grid; cell lit when its bit is set in `firedBitmap`.

## Data flow

```
box: state + firedMask --(StatusPacket, ESP-NOW, ~750ms)--> controller RX cb
  RX cb: capture rssi, CRC-check --> status queue
  control loop: drain queue --> g_status.boxes[] (+ linkAlive from freshness)
  GET /api/status --> JSON boxes[]
  web UI poll (~1s) --> render card (state badge, link/RSSI, fired grid)
```

## Error handling

- Bad-CRC or short STATUS packets are dropped silently in the RX callback (same
  as ACK handling today).
- No StatusPacket for >1.5 s → `linkAlive=false` → UI shows "no link". The card
  retains the last-known state/fired map (greyed) rather than blanking.
- An unconfigured (null-MAC) box never reports; its card is hidden or shown
  "not configured".
- Telemetry loss never affects firing or the interlock — it is read-only.

## Testing

- **Host (CTest):** firedChannelMask behavior; STATUS CRC round-trip; box ignores
  STATUS.
- **On hardware (Tier A live proof):**
  1. Arm the box; confirm its card flips to **ARMED** with good RSSI and a live
     link dot.
  2. Fire a channel; confirm its cell lights in the fired grid and
     `lastFiredChannel` updates.
  3. **Cut the controller's power**; confirm the box card flips to **SAFE** +
     "no link" within ~2 s (dead-man auto-disarm on real radios).

## Out of scope (tracked elsewhere)

- **C — WiFi-pull OTA:** next sub-project, its own spec (repartition to two OTA
  slots, `ENTER_OTA` ESP-NOW command, box joins SoftAP + HTTP-pulls image,
  refuse-OTA-unless-SAFE).
- **D — save/load shows + 32-button grid:** deferred.
- **E — continuity/igniter sense + battery voltage:** hardware-gated (SSR boards),
  deferred.
