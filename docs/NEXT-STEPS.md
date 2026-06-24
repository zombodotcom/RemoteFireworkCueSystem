# Remote Firework Cue System — Next Steps

_Status as of 2026-06-24 (merge of `32ch-webui-show` → `main`)._

## Done so far

- **Host-tested safety core** (`components/fireworkcore/`): arming state machine, dedup, sequence runner, CRC protocol — 9 host CTest suites green, reused unchanged by firmware.
- **Firing-box firmware** (`firmware/`) and **ESP32 controller** (`controller/`): SoftAP + web UI + ESP-NOW + sequences. Never-boot-armed, physical arm switch authoritative, heartbeat dead-man.
- **OTA ESP-NOW link validated on real hardware** — ARM→FIRE→ACK proven with the box physically armed.
- **Box → controller telemetry**: box pushes a `StatusPacket` (~750 ms); controller surfaces per-box `state / linkAlive / rssi / firedBitmap / lastFired` on `GET /api/status` (backward compatible).
- **Web UI** (`webui/`, Svelte): show authoring + live per-box card (vitest 17/17).
- **CYD status dashboard** (`cyd-dashboard/`, ESP32-2432S028R, **ST7789**): read-only panel, joins the AP, polls `/api/status`, status-hero layout with pulsing ARMED/alarm glow, heartbeat dot, fired grid + just-fired flash, last-fired. Panel driver/config bench-proven (see `cyd-panel-config` memory).

## Next — by priority

### 1. Safety verification (do before any real show)
- [ ] **Tier A live dead-man proof**: arm the box, cut the controller's power, confirm the box auto-disarms to SAFE within ~2 s on serial. (Logic is host-tested + wired; the on-radio proof is still pending.)
- [ ] **Telemetry hardware E2E** (parked): on the phone, verify SAFE/ARMED echo, RSSI, fired grid, and the `NO BOX LINK` / `NO CONTROLLER` transitions.

### 2. Hardware-gated (SSR boards arriving)
- [ ] **SSR polarity test** (active-HIGH vs active-LOW) → choose expander (PCF8575 vs MCP23017). Gates everything downstream. See `hardware-status` memory.
- [ ] Wire the real **2×16 SSR boards**; keep both boxes identical (same expander, same firmware, same boot-safe behavior).
- [ ] **Continuity / igniter sense** (per-channel) — know a cue is live before firing. Needs ADC/sense wiring.
- [ ] **Battery voltage telemetry** — voltage divider → spare ADC; add to `StatusPacket` → `/api/status` → phone + CYD.

### 3. Features
- [ ] **Queued / sequence progress** (deferred CYD add-on): controller exposes `seqIndex` / `seqTotal` / `nextChannel` in `/api/status`; parser + CYD + phone show "CUE 3/12 · next ch N". _Touches the controller — run it through review._
- [ ] **Second box (box 1)**: set `BOX_MAC[1]` + multi-box display on phone/CYD (already configurable; UI hides unconfigured boxes today).
- [ ] **Better phone UI**: responsive/mobile redesign of the Svelte app (currently not responsive). Optional **Capacitor APK** for an installed icon — a true PWA is blocked by the HTTP-only AP (no HTTPS).
- [ ] **OTA updates**: WiFi-pull (box temporarily joins the AP, HTTP-pulls the image), refuse-OTA-unless-SAFE, two-OTA-slot partition table. Removes the BOOT-button flashing dance.
- [ ] **Save/load shows to NVS** + 32-button manual fire grid in the web UI.

### 4. CYD polish (minor)
- [ ] Cell numbering: currently **0–15** (matches firmware/phone); switch to 1–16 if preferred.
- [ ] Optional: graphical RSSI signal bars, night/brightness dimming.

## Known tech debt (logged in reviews, non-blocking)
- **CYD display auth hardening**: the ESP-NOW display broadcast is now source-MAC-filtered (rejects foreign/spoofed frames), but a determined attacker who spoofs the controller's MAC could still forge a false "SAFE" on the panel. For a real deployment, add a pre-shared **HMAC + monotonic counter** to `DisplayStatusPacket`/`DisplayEventPacket` (key provisioned out-of-band), or switch to unicast ESP-NOW with a per-peer LMK. Keep `seq` as `uint32_t` end-to-end if revisiting. Also: CYD event log can stall after a *controller* reboot (seq resets) until seq catches up — status unaffected.
- `wifi_sta_start()` (CYD) is cold-boot-only / not idempotent; `lastUpdateMs` u32 wraps at ~49 days (relative-freshness use only).
- **Flashing reliability**: the box/controller CP210x dev boards need the manual BOOT-hold to enter download mode; the CYD's CH340 auto-resets fine. Consider the **1 µF EN→GND cap mod** on the dev boards for hands-free flashing (or do OTA, above).
- Build host tests with **g++ forced** (`-DCMAKE_CXX_COMPILER=g++`) — CMake auto-picks a stray MSVC otherwise.

## Build/flash quick reference
- **IDF (firmware/controller/cyd-dashboard):** PowerShell → `& 'C:\esp\v6.0.1\esp-idf\export.ps1'; idf.py build`. Flash dev boards (CP210x) with the custom reset env + BOOT-hold; CYD on **COM12** flashes with plain `--before default-reset`.
- **Host core tests:** Git Bash → `cmake -B build -G Ninja -DCMAKE_CXX_COMPILER=g++ && cmake --build build && ctest --test-dir build`.
- **Web UI:** `cd webui && npm run build && gzip -9 -c dist/index.html > ../controller/main/www/index.html.gz` then rebuild the controller.
- **Secrets:** AP creds live only in git-ignored `*/main/secrets.h` (controller + cyd-dashboard).
