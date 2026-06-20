# Wireless Firework Show Controller — Design

- **Date:** 2026-06-20
- **Branch:** `32ch-webui-show` (off `4channel-ssr`)
- **Status:** Approved design, pre-implementation
- **Supersedes:** the 4-channel single-master/single-slave build (kept as the proven baseline)
- **Platform:** ESP32 for **both** controller and firing boxes, on **ESP-IDF v6.0.1** (installed at `C:\esp\v6.0.1\esp-idf`). One firmware codebase, one chip family, a role flag distinguishes controller vs box. Host unit tests build with MinGW UCRT g++ + CMake + Ninja.
- **Toolchain reality (verified 2026-06-20):** ESP-IDF on Windows does **NOT** support Git Bash/MSys (`export.sh` aborts with "MSys/Mingw is not supported"). `idf.py` must be driven from **PowerShell**: activate with `& 'C:\esp\v6.0.1\esp-idf\export.ps1'` then `idf.py set-target esp32` / `idf.py build`. The host-only `fireworkcore` tests continue to use Git Bash + CMake/Ninja/g++ (unaffected). So: firmware = PowerShell + idf.py; safety-core unit tests = Git Bash + ctest.

## 1. Goal

Rebuild the remote firework cue system to:

- Drive **32 channels** across **two 16-channel SSR boards** (one per firing box).
- Be controlled from a **phone via a web UI** the system hosts itself (no internet, no app store).
- Be **configurable**: channel labels, groups/macros, and saved timed sequences.
- Support three firing modes: **manual single cues**, **groups/macros**, and **timed sequences** (scripted show).
- Be **safe above all else** — multiple independent interlocks, fail-safe by default.

Keep ESP-NOW as the wireless link. Reuse the proven *algorithms* from last year's working build (CRC32, packet layout, ACK handshake, persisted arm state) — reimplemented as portable C++ under ESP-IDF rather than copied from the Arduino code.

Out of scope for this phase: music/timecode-synced shows (leave a hook for it later).

## 2. Topology

```
   Phone (browser/PWA)
        │  WiFi SoftAP + web UI + heartbeat
        ▼
   CONTROLLER  (ESP32)
   • hosts SoftAP + serves web app
   • holds canonical config (flash)
   • runs arming state + sequence engine
   • E-STOP authority, app-heartbeat watchdog
        │  ESP-NOW (per-box MAC, retried, ACKed)
        ├──────────────► Firing Box A (ESP32 + I²C expander) → 16 SSR channels
        └──────────────► Firing Box B (ESP32 + I²C expander) → 16 SSR channels
```

The phone **proposes**, the controller **decides**, the box **verifies**. Three independent checks before any channel fires.

## 3. Hardware

### Framework & MCU
- **ESP32 on ESP-IDF v6.0 for everything** — controller and both boxes. Single firmware codebase with a compile-time/NVS **role flag** (`CONTROLLER` | `BOX`). ESP-IDF gives FreeRTOS task isolation (a high-priority safety/ESP-NOW task the web server cannot starve), native `esp_http_server`/WebSocket, `esp_now`, NVS config storage, and watchdogs.

### Controller
- **ESP32.** Hosts SoftAP + web server + ESP-NOW concurrently; ample RAM/flash for UI + config.

### Firing boxes (×2, identical)
- **ESP32** — "dumb but safe" endpoint.
- **One I²C GPIO expander** per box converting 2 wires → 16 outputs, wired to the SSR board's 16 logic input pins. **Kept even though the ESP32 has enough GPIO**, because several ESP32 pins are strapping/boot-glitch pins that must never touch an igniter; the expander gives a deterministic all-off boot state and a single-write all-off for E-STOP.
- **Both boxes must be identical**: same expander type, same firmware, same boot-safe behavior. Two differently-behaving boxes = two things to validate and two ways to be surprised.

### Deferred hardware decision — expander + SSR polarity
SSR boards arrive ~2026-06-24; they use 16 individual logic input pins. Polarity (active-HIGH vs active-LOW) is unknown until tested and **gates the expander choice**:

- **Active-LOW** (fire on LOW): standardize on **PCF8575** (boots all-pins-HIGH = safe-off; 3 owned). MCP23017 kept as spare.
- **Active-HIGH** (fire on HIGH): standardize on **MCP23017** + pull-down resistors (buy one more so the pair matches). PCF8575 unsafe here because it boots all-HIGH.

Firmware abstracts this behind a **channel-driver interface** with two compile-time constants — `EXPANDER_TYPE` (`MCP23017` | `PCF8575`) and `FIRE_LEVEL` (`HIGH` | `LOW`) — so the polarity test only flips two values. Nothing else in the system depends on this.

**Boot-safe hardware rule:** every SSR input gets a **pull resistor holding it in the OFF state**, so a floating/booting/crashed expander physically cannot fire. The expander only ever drives a pin to the active level deliberately.

Owned today: ESP8266 D1 minis (now spares/bench-LED rigs), 1× MCP23017, 3× PCF8575. **To acquire:** 3× ESP32 dev boards (1 controller + 2 boxes; prices ≈ ESP8266). The host-testable safety core (Plan 1) needs no hardware at all; box bring-up later uses an ESP32 + the MCP23017 + LEDs (no SSR/pyro).

## 4. Components & responsibilities

### Controller (ESP32) — the only "smart" device
- Hosts SoftAP (e.g. SSID `FireControl`, WPA2) and serves the web app (PWA) from flash.
- Holds the **canonical config** in flash: channel labels, groups, saved sequences. Survives reboot and phone swaps.
- Runs the **arming logic**, **E-STOP authority**, and **sequence engine** (timeline execution).
- ESP-NOW to both boxes: addresses by MAC, retries un-ACKed cues, tracks box online/offline.
- Maintains the **app-heartbeat watchdog**.

### Firing Box A & B (ESP8266 + expander) — dumb but safe
- On boot: force expander outputs to SSR-off, state = SAFE (disarmed), regardless of prior state.
- Read the **physical arm switch** every loop — hardware-authoritative; off ⇒ instant SAFE.
- Accept only validated `FIRE` commands; pulse the SSR for a firmware-bounded duration; ACK.
- **Self-watchdog**: no valid controller traffic for the timeout while idle ⇒ force outputs off / SAFE.
- Enforce all interlocks **locally**, independent of the controller.

### Web app (PWA, runs in phone browser) — pure UI
- Arm/disarm, E-STOP, manual cue fire (with confirm), group buttons, sequence builder + runner.
- Sends commands + 1s heartbeat to the controller; renders live status pushed back.
- Never fires anything directly; reads truth from the controller.

## 5. Safety model

### Per-box state machine (runs independently on each box)
```
BOOT → force expander all-OFF → SAFE (never boots armed)
SAFE  ── phys switch ON + controller ARM (fresh nonce) ──► ARMED
ARMED ── ANY of: phys switch OFF / E-STOP / heartbeat lost (idle) / disarm cmd / watchdog ──► SAFE
ARMED ── valid FIRE ──► FIRING (one channel, bounded pulse) ──► ARMED
```

### Gates to enter/stay ARMED (all required)
1. **Physical arm switch ON** — read every loop, hardware-authoritative.
2. **Controller ARM command** — deliberate, carries a fresh **nonce**. Replay protection is **reset-per-session**: a nonce replayed while still armed is rejected, but nonce tracking clears on any return to SAFE (disarm/E-STOP/switch-off), so each arming session is independent and a controller reboot can never lock you out of arming. The physical switch is always the real gate.
3. **Heartbeat alive while idle** — valid controller heartbeat within timeout. During a committed sequence the box honors the controller's scheduled cues; a brief link blip does not drop it.
4. **Not E-STOPped** — E-STOP latches SAFE until deliberate re-arm.

### Gates to FIRE a channel (on top of ARMED)
5. Valid `FIRE` packet: correct **CRC**, in-range channel, **unseen message ID** (no double-fire on resend).
6. Pulse is **time-bounded in firmware** (`FIRE_MS`) — a channel physically cannot stay on longer than the cap, regardless of the command. Hard ceiling against stuck-on outputs.

### Link-loss policy (per user decision)
- **Armed but idle (manual):** heartbeat enforced — lose the phone for the timeout ⇒ box disarms. Wander-off protection.
- **Committed timed sequence:** once deliberately started, the **controller runs it to completion autonomously**; the phone is not required to keep it alive, and a brief WiFi blip never aborts it. The **physical arm switch on each box is the ultimate kill**, plus phone E-STOP whenever connected.
- WiFi flakiness can never *fire* anything and never *aborts a running show*.

### Tunable parameters (initial values)
- **Heartbeat timeout (idle auto-disarm):** ~2 s.
- **Max fire pulse `FIRE_MS`:** ~400 ms.

### Independence principle
Phone, controller, and box each say "no" independently — three locks, not one. A misbehaving controller or malformed packet still cannot make a box fire.

## 6. Protocol & data flow

### Phone ↔ Controller (over SoftAP)
HTTP + WebSocket API. Commands: `arm`, `disarm`, `estop`, `fire(ch)`, `fireGroup(id)`, `runSequence(id)`, `stop`, and a 1 s `heartbeat`. Controller pushes live status: armed state, per-box online/offline, last ACKs, sequence progress.

### Controller ↔ Boxes (ESP-NOW)
Extends the proven packet design:
- `CommandPacket { type, cmd, msgID, targetChannel, crc }` — reused.
- `AckPacket { type, responseToID, note, status, timestamp, crc }` — reused.
- New message types: `ARM`, `DISARM`, `HEARTBEAT`, `ESTOP`, carrying a **nonce** so arm/disarm can't be replayed.
- Controller addresses each box by MAC, **retries** un-ACKed cues, surfaces failures to the phone ("Box A didn't ACK channel 12").

## 6.5 LED / status indication

Both boxes and the controller carry status LEDs (NeoPixel/addressable). These are **safety feedback, not decoration** — at a glance you must know whether the system is hot.

- **Firing box:** an **arm indicator** (red = SAFE, blue/green = ARMED) and per-channel state (idle / firing flash). A distinct color/pattern for E-STOPped and for "lost controller heartbeat." Power-on shows SAFE.
- **Controller:** overall armed state, per-box online/offline, sequence-running indicator, and an E-STOP/fault pattern.
- LED state is **driven by the safety state machine**, so the lights can never show ARMED while the box is actually SAFE (single source of truth).
- LED rendering must be **non-blocking** (no `delay()` in the fire/idle path) so status updates never stall the arm-switch / E-STOP checks.

## 7. Configuration & sequences

Stored as JSON in controller flash, editable from the app, persistent across reboots:
- **Channels:** `{ id, label, box, pin }` — human label per physical channel.
- **Groups/macros:** named sets of channel ids.
- **Sequences:** ordered `{ time_ms, target (channel id or group id) }` steps; multiple shows can be saved. Controller executes the timeline and dispatches cues at the scheduled offsets.

## 8. Testing

- **Host-testable logic** (CRC, dup-ID rejection, state machine, sequence scheduler) as plain unit tests — no hardware.
- **Bench bring-up** with LEDs on the expander (no pyro): validate the full arm → fire → ACK path and every interlock (physical switch, heartbeat loss, E-STOP, never-boot-armed, watchdog).
- **Polarity test** when boards arrive, then real SSR boards verified with a multimeter / test lamp before any pyro.
- A written **pre-show checklist** committed to the repo.

## 9. Reuse from the existing build

The 4-channel single-master/single-slave Arduino firmware worked last year and is the reference. Carried forward as **designs, reimplemented in portable C++ under ESP-IDF**: the CRC32 routine, the `CommandPacket`/`AckPacket` layout, the ACK handshake, persisted arm state (Arduino `EEPROM` → ESP-IDF **NVS**), and the status-LED patterns (Arduino `Adafruit_NeoPixel` → ESP-IDF `led_strip` RMT driver). Same proven logic, new framework.

## 9.5 Improvements over the baseline code

The baseline worked but is barebones. Concrete improvements to bake in:

1. **Non-blocking fire pulse (safety fix).** The old slave fires with a blocking `delay(500)`, during which it cannot check the arm switch or E-STOP. Replace with a timer/tick-scheduled pulse (FreeRTOS task + `esp_timer`) so a hot channel can still be aborted/disarmed mid-pulse. Critical with 32 channels + sequences.
2. **Shared protocol/safety component.** `CommandPacket`/`AckPacket`/CRC and the arming logic are duplicated across `master.cpp`/`slave.cpp` today and can silently drift. Consolidate into one portable `fireworkcore` component shared by both roles and host-tested.
3. **Recent-ID ring buffer** for dedup instead of a single `lastProcessedFireID`, so fast back-to-back cues in a sequence are deduped robustly.
4. **ESP-NOW send-status callback + retry.** Controller confirms delivery and retries un-ACKed cues instead of fire-and-hope; failures surface to the phone.
5. **Authenticated arm (nonce)** so ARM/DISARM cannot be replayed.
6. **Consistent, deliberate GPIO config** (baseline used `INPUT` on master arm switch vs `INPUT_PULLUP` on slave). All safety inputs explicitly configured (`gpio_config` with defined pull) and consistent across roles.
7. **Non-blocking LED rendering** (see §6.5) — no `delay()` in the fire/idle path.

## 10. Open items (do not block design)

- Confirm SSR polarity → finalize expander (PCF8575 vs MCP23017) and `FIRE_LEVEL`. (~2026-06-24)
- Buy 3× ESP32 dev boards (controller + 2 boxes); confirm exact variant (e.g. ESP32-WROOM-32 devkit).
- Final SoftAP SSID/password and whether to add a UI passcode gate.
