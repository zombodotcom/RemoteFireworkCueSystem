# Remote Firework Cue System

A wireless, **safety-first** firework firing system: a phone/web UI authors and runs shows, an ESP32
**controller** relays commands over ESP-NOW to one or more ESP32 **firing boxes** (2×16 = 32 channels),
and a touchscreen **CYD dashboard** shows live status. Built for July 4th.

> **Reading this cold?** This file is the orientation doc. The crown jewel is the **Safety model**
> below — read it before touching any firing path. For the running task list see
> [`docs/NEXT-STEPS.md`](docs/NEXT-STEPS.md). Live, point-in-time state lives in the
> `~/.claude/.../memory/` notes (`MEMORY.md` index).

---

## Safety model (non-negotiable)

Three independent locks: **phone proposes, controller decides, box verifies.** Detailed rationale in
`docs/superpowers/specs/2026-06-20-wireless-firework-show-controller-design.md`.

- **Never boots armed.** Boot always forces expander outputs OFF and state = SAFE, regardless of prior state.
- **Physical arm switch is authoritative** — read every loop, hardware-gated. Software can never override it.
- **To be ARMED (all required):** physical switch ON + deliberate controller ARM command (with nonce) +
  heartbeat alive + not E-STOPped. Nonce replay protection resets per session (clears on every return to
  SAFE, so a controller reboot never locks out arming).
- **To FIRE (on top of ARMED):** valid CRC + in-range channel + unseen message ID (no double-fire) +
  firmware-bounded pulse `FIRE_MS` (~400 ms hard ceiling; a command cannot override it).
- **Force-off invariant:** the box de-energizes ALL channels every tick whenever not ARMED (idempotent —
  covers disarm / E-STOP / switch-off / heartbeat-loss with no missed-edge bug).
- **Link-loss policy:** idle+armed enforces a ~2 s heartbeat dead-man (lost phone → box disarms). A
  *committed* timed sequence runs to completion autonomously on the controller; a WiFi blip never fires
  anything and never aborts a running show. The physical switch is always the ultimate kill.
- **Boot-safe hardware:** every SSR input has a pull resistor holding it OFF, so a floating/booting/crashed
  expander physically cannot fire.
- **CYD is read-only** — it never fires or POSTs; it only displays. Its ESP-NOW input is filtered by the
  controller's source MAC (anti-spoof).

---

## Architecture

```
 phone / web UI ──HTTP──► CONTROLLER (ESP32 SoftAP "FireControl")
                              │  ├─ ESP-NOW unicast ──► FIRING BOX(es)  (StatusPacket back, ~750ms)
                              │  └─ ESP-NOW broadcast ─► CYD DASHBOARD   (DisplayStatus/Event frames, ~1Hz)
                              └─ serves the bundled web UI from flash
```

- **Phone/website** stays on WiFi+HTTP to the controller's SoftAP (channel 1, WPA2+PMF).
- **CYD does NOT associate** to the AP — an ESP32 STA can't stay associated to the IDF softAP (PMF/SA-Query
  kicks it every ~5 s). It listens passively over ESP-NOW on channel 1 instead. This is why the dashboard is
  stable now. (See the `cyd-panel-config` memory for the full story.)

### Repo layout (active = ESP-IDF v6.0.1)

| Path | What it is |
|------|-----------|
| `components/fireworkcore/` | **Shared, host-tested safety core** — arming state machine, dedup, sequence runner, CRC protocol, `protocol.h` wire format. Reused unchanged by all firmware. |
| `firmware/` | **Firing-box** firmware (ESP32). Arm switch, ESP-NOW link, expander driver, status LEDs, boot-safe behavior. |
| `controller/` | **Controller** firmware. SoftAP + web server, ESP-NOW (box unicast + display broadcast), event log / diag / fault code, serves the embedded web UI. |
| `cyd-dashboard/` | **CYD** (ESP32-2432S028R, **ST7789** panel) read-only LVGL dashboard over ESP-NOW. Tap-paged Dashboard / Log / Diag. |
| `webui/` | **Svelte** web UI (show authoring + live status). Built to a gzipped single file embedded in the controller. |
| `docs/` | Design specs, plans (`docs/superpowers/`), and `NEXT-STEPS.md`. |
| `src/`, `include/`, `lib/`, `platformio.ini` | **Legacy** ESP8266/PlatformIO prototype — superseded by the ESP-IDF projects above. Not the active system. |
| `oldfiles/` | Archived scratch. |

---

## Build & flash quick reference

> **ESP-IDF builds run ONLY in PowerShell.** Host unit tests run in Git Bash with g++ forced.

**ESP-IDF firmware** (`firmware/`, `controller/`, `cyd-dashboard/`):
```powershell
& 'C:\esp\v6.0.1\esp-idf\export.ps1'
idf.py build
idf.py -p COMxx flash monitor
```
- **Box / controller dev boards (CP210x):** flash **hands-free** by setting the custom reset env first:
  `$env:ESPTOOL_CUSTOM_RESET_SEQUENCE="D0|R1|W0.6|D1|R0|W0.4|D0"`, then `--before default-reset`. Only fall
  back to the BOOT-hold dance + `--before no-reset` if you hit a `0x13` boot-mode error. (esptool here is
  v5.3 — hyphen syntax.)
- **CYD (CH340) on COM12:** auto-resets, flash with plain `--before default-reset`.

**Host core tests** (Git Bash — CMake auto-picks a stray MSVC otherwise):
```bash
cmake -B build -G Ninja -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++
cmake --build build && ctest --test-dir build
```

**Web UI** (rebuild + embed into the controller):
```bash
cd webui && npm run build
gzip -9 -c dist/index.html > ../controller/main/www/index.html.gz
# then rebuild the controller
```
> `npm run build` needs the WASM sim built first (`bash webui/scripts/build-wasm.sh`, requires emsdk) —
> otherwise it fails on `fireworkcore.js`.

**Secrets:** AP credentials live ONLY in git-ignored `*/main/secrets.h` (controller + cyd-dashboard).
SSID `FireControl`. Never commit them.

**Pairing:** per-board ESP-NOW MACs live in git-ignored `*/main/pairing.h` (same pattern as secrets;
committed defaults fall back via `__has_include`). Regenerate for a new board set with
`pwsh scripts/gen-pairing.ps1 -ControllerPort COMx -Box0Port COMy`.

---

## Current state

- ✅ Host-tested safety core (CTest suites green), reused by firmware.
- ✅ Box + controller firmware: SoftAP, web UI, ESP-NOW, sequences. Never-boot-armed, switch-authoritative,
  heartbeat dead-man.
- ✅ OTA ESP-NOW link validated on real hardware (ARM→FIRE→ACK with the box physically armed).
- ✅ Box→controller telemetry surfaced on `/api/status`.
- ✅ Svelte web UI (show authoring + live box card).
- ✅ CYD dashboard over ESP-NOW — stable, no association flapping; source-MAC anti-spoof on the display channel.

**Next up (see [`docs/NEXT-STEPS.md`](docs/NEXT-STEPS.md) for the full list):** Tier-A live dead-man proof,
SSR hardware/polarity test (boards arriving ~2026-06-24), second box, OTA pull, responsive phone UI,
save/load shows, and the deferred display-channel HMAC hardening.

---

## Office work log

> _Append what you do at the office below — date it, note which board/COM port, and update
> `docs/NEXT-STEPS.md` if priorities shift. Keep this README's Safety model section accurate above all._

- _(nothing yet)_
