# Firework Browser Simulator + Show Authoring — Design

- **Date:** 2026-06-20
- **Branch:** `32ch-webui-show`
- **Status:** Approved design, pre-implementation
- **Related:** builds on the safety core from `2026-06-20-firework-core-foundation.md`; complements the system design `2026-06-20-wireless-firework-show-controller-design.md`

## 1. Goal

A **browser-based simulator** that needs **no ESP32s**, so the system can be designed, configured, and exercised on a laptop:

- **Author config & shows:** label the 32 channels, define groups/macros, build timed sequences; save/load/export the config as JSON (the same schema the controller will use).
- **Simulate real behavior:** a virtual controller + two virtual firing boxes with on-screen LEDs, arm switches, channels and an E-STOP. Arm, fire manual cues, run sequences, and watch the **real interlocks** play out (never-boot-armed, physical-switch authority, heartbeat dead-man, E-STOP latch, bounded fire pulse).

Fidelity comes from running the **actual safety core** (`fireworkcore`) compiled to **WebAssembly** — the simulator behaves exactly like the firmware because it *is* the firmware logic. No safety logic is reimplemented.

Out of scope here: the live WebSocket transport to a real controller (the interface seam is defined, but the implementation is deferred until controller firmware exists).

## 2. Locked decisions

- **Stack:** Vite + Svelte (single interactive control panel; tiny static bundle that later fits ESP32 flash).
- **Fidelity:** real `fireworkcore` C++ compiled to WASM via **Emscripten** — one source of truth, zero drift.
- **Reuse:** `fireworkcore` sources are used **unchanged**; one thin `sim_bindings.cpp` wraps them into a virtual rig.
- **Transport seam:** a `SystemConnection` interface; `SimConnection` (WASM rig) now, `LiveConnection` (WebSocket) later.
- **Config schema:** the authoring tool produces the **same JSON** the controller will consume.

## 3. Architecture

```
  ┌──────────────────────── Browser (Vite + Svelte) ────────────────────────┐
  │  Authoring UI         Virtual Rig UI          SystemConnection (iface)    │
  │  • channel labels     • controller panel       ├─ SimConnection  ◄ now    │
  │  • groups/macros      • 2 box panels           └─ LiveConnection ◄ later  │
  │  • sequence editor    • 16 LED cells / box        (WebSocket → ESP32)     │
  │  • save/load/export   • arm switches, E-STOP                              │
  │        │                      │                        │                 │
  │        └─ config JSON ────────┴────────────────────────┘                 │
  │                                │ calls                                    │
  │                    ┌───────────▼────────────┐                            │
  │                    │   fireworkcore.wasm    │  ← same tested C++ core     │
  │                    │  virtual controller +  │     + sim_bindings.cpp      │
  │                    │  2 virtual boxes       │     driven by tick(simClock)│
  │                    └────────────────────────┘                            │
  └──────────────────────────────────────────────────────────────────────────┘
```

**Time model (key):** because `fireworkcore` takes injected `nowMs`, the browser owns a **simulated clock**. An animation loop calls `rig_tick(nowMs)` each frame with the sim clock; the UI can run real-time, fast-forward, pause, or single-step. A "phone heartbeat" is simulated by the loop calling `rig_heartbeat(nowMs)` each tick — stopping it on demand demonstrates the idle dead-man auto-disarm.

## 4. The virtual rig (`sim_bindings.cpp`)

A singleton "rig" composed entirely from existing `fireworkcore` types: one controller view + two boxes, each box owning an `ArmingStateMachine`, a `RecentIds<>` dedup, and 16 channel pulse timers; the controller owns one `SequenceScheduler`. Exposed to JS as a small C ABI (Emscripten `cwrap`/`ccall`), with state returned as a JSON snapshot string.

**API (all timestamps are the sim clock `nowMs`):**
- `rig_reset()` — construct fresh: both boxes SAFE, switches off, outputs off (never-boot-armed).
- `rig_set_switch(boxId, on, nowMs)` — toggle a box's physical arm switch.
- `rig_heartbeat(nowMs)` — controller→boxes keep-alive (called each tick while "connected").
- `rig_arm(nonce, nowMs)` — controller broadcasts ARM to both boxes; each admits only if its switch is on and not E-STOPped.
- `rig_disarm(nowMs)` / `rig_estop(nowMs)` / `rig_clear_estop(nowMs)`.
- `rig_fire(boxId, channel, nowMs)` — manual cue; honored only if that box `canFire(nowMs)` and the channel is in range and the message id is unseen.
- `rig_load_sequence(jsonPtr)` — load a flat list of `SeqStep{timeMs, boxId, channel}` (the JS expands groups to channel steps before loading).
- `rig_start_sequence(nowMs)` / `rig_stop_sequence(nowMs)`.
- `rig_tick(nowMs)` — advance: (1) `update()` each box's arming; (2) if a sequence is running, pull `due(nowMs)` steps and attempt each fire; (3) expire channel pulses older than `FIRE_MS`. Mirrors the firmware loop, non-blocking.
- `rig_snapshot_json()` — returns:
  ```json
  {
    "now": 12345,
    "controller": { "armRequested": true, "sequenceRunning": false },
    "boxes": [
      { "id": 0, "switchOn": true, "state": "ARMED", "estopped": false, "canFire": true,
        "channels": [ { "firing": false, "msLeft": 0 }, ... 16 ] },
      { "id": 1, ... }
    ]
  }
  ```

The rig dispatches a sequence cue exactly as the firmware will: a due step calls the same `canFire` gate before energizing a channel pulse, so a disarmed/E-STOPped box visibly skips cues mid-show.

## 5. Config schema (shared with the controller)

```jsonc
{
  "version": 1,
  "channels": [ { "id": "c0", "label": "left rack 1", "boxId": 0, "channel": 0 }, ... up to 32 ],
  "groups":   [ { "id": "g_finale", "label": "Finale", "members": ["c8","c9", ...] } ],
  "sequences":[ { "id": "s1", "label": "Main show",
                  "steps": [ { "timeMs": 0, "targetType": "channel", "targetId": "c0" },
                             { "timeMs": 1500, "targetType": "group", "targetId": "g_finale" } ] } ]
}
```

Stored in `localStorage`, with file **export/import** (JSON download/upload). Group steps are expanded to per-channel `SeqStep`s in JS before `rig_load_sequence`.

## 6. Components & files

**Reused unchanged:** `components/fireworkcore/include/*`, `components/fireworkcore/src/*`.

**New:**
- `components/fireworkcore/wasm/sim_bindings.cpp` — the rig + C ABI (host-compilable too).
- `webui/` — Vite + Svelte app:
  - `src/core/wasm.ts` — loads the Emscripten module, wraps the C ABI as typed functions.
  - `src/core/connection.ts` — `SystemConnection` interface + `SimConnection` (wraps the rig + owns the sim-clock loop).
  - `src/lib/config.ts` — schema types, load/save/export/import, group→step expansion, validation.
  - `src/components/` — `ControllerPanel.svelte`, `BoxPanel.svelte`, `ChannelGrid.svelte`, `ChannelEditor.svelte`, `GroupEditor.svelte`, `SequenceEditor.svelte`, `TransportControls.svelte` (play/pause/step/speed + simulate-disconnect).
  - `src/stores/` — Svelte stores for snapshot + config.
  - `src/App.svelte`, `src/main.ts`.
- `webui/scripts/build-wasm.sh` — emcc build: `fireworkcore` srcs + `sim_bindings.cpp` → `webui/src/core/fireworkcore.{js,wasm}`.
- `webui/package.json`, `vite.config.ts`, `tsconfig.json`.

## 7. Data flow

UI event → `SimConnection.method()` → WASM C-ABI call (real `fireworkcore` logic) → the sim-clock loop calls `rig_tick(now)` then `rig_snapshot_json()` → parsed into the snapshot store → Svelte re-renders boxes/LEDs/channels. Authoring writes `config.ts` state → `localStorage`; "Run" expands the chosen sequence's groups to steps and calls `rig_load_sequence` + `rig_start_sequence`.

## 8. Testing

- **Rig host tests (CTest, no browser):** `sim_bindings.cpp` is pure C++, so add `test/test_rig` to the existing host build — arm→fire lights a channel; disarmed/E-STOPped fire is rejected; a sequence fires its cues in order and a mid-show E-STOP skips the rest; a pulse clears after `FIRE_MS`; idle heartbeat-loss disarms. This tests the rig logic before any WASM/Emscripten is involved.
- **Vitest (web):** config schema load/save/export/import + group expansion; a WASM smoke test (module loads, `rig_reset` then arm→fire reflects in the snapshot).
- **Manual:** `npm run dev`, drive the panels.

## 9. Toolchain

- **Present:** Node v22 + npm 10 (Vite); MinGW g++ + CMake + Ninja (rig host tests reuse the existing CTest harness).
- **To install:** **Emscripten (emsdk)** — not currently installed (~1 GB one-time). Pin a known-good version in `webui/scripts/build-wasm.sh` and document the `emsdk install/activate` steps. The **authoring UI and config tests need no Emscripten**; only the behavioral sim (WASM) does — so authoring work can proceed before/independently of the emsdk install.

## 10. Decomposition (two plans)

- **Plan A — Behavioral sim core (build first; highest uncertainty):** install/pin emsdk; write `sim_bindings.cpp` + its CTest host tests; the emcc build script; `src/core/wasm.ts` + `SimConnection`; a minimal Svelte harness rendering the 2 boxes (LEDs, arm switches, E-STOP, manual fire) driven by the sim clock. **Done = you can arm/fire/run a hard-coded sequence in the browser and watch real interlocks, no hardware.**
- **Plan B — Authoring + show building:** `config.ts` schema + validation + save/load/export; `ChannelEditor`/`GroupEditor`/`SequenceEditor`; group→step expansion; wire authored sequences into the rig; layout/polish.

## 11. Open items (non-blocking)

- Pin the exact emsdk version (latest stable at implementation time).
- Decide whether `webui/` build output later becomes the shipped device UI (the `SystemConnection` seam already allows it; not committed now).
- Visual styling/branding of the panels — deferred to Plan B polish.
