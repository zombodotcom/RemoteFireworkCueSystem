# CYD Status Dashboard

**Date:** 2026-06-23
**Branch:** `32ch-webui-show`
**Status:** Design — approved for planning

## Goal

Turn the owned **CYD (Cheap Yellow Display, ESP32-2432S028R, 2-USB variant)** into a standalone, read-only **status panel** for the firework system. It joins the controller's Wi-Fi AP and shows, at a glance from across the yard: the box's real ARMED/SAFE state, the box link + RSSI, which channels have fired, and whether a sequence is running.

**Read-only** — no touch controls. The physical arm switch + the controller remain the only firing authority. A touchscreen must never be able to arm or fire.

## Background / why this is cheap

The `32ch-webui-show` telemetry work already exposes everything this dashboard needs at `GET http://192.168.4.1/api/status`:

```json
{ "armed": false, "seqRunning": false, "lastFailedBox": null,
  "boxes": [ { "id": 0, "linkAlive": true, "rssi": -52, "state": 0,
              "firedBitmap": 0, "lastFired": -1 } ] }
```

So the CYD is just a **third consumer** of that JSON (like the phone), rendered on a TFT. **No box or controller firmware changes.**

Hardware confirmed on the bench (see memory `cyd-panel-config`): board on COM12, MAC `a4:f0:0f:5f:4f:00`, auto-reset works (no BOOT-hold). **Display is ST7789** (not ILI9341) — verified by an esp_lcd spike.

## Architecture

A single ESP-IDF app on the CYD, structured as four small units:

```
WiFi STA (join FireControl) ──> StatusClient (HTTP GET /api/status, ~1 Hz)
   ──> parseStatus(json) ──> StatusModel ──> Dashboard (LVGL render)
```

**Toolchain:** ESP-IDF v6.0.1 + `esp_lcd` (built-in ST7789 driver) + **LVGL** (`lvgl/lvgl` + `esp_lvgl_port` managed components) for fonts/labels/layout. Chosen to keep one toolchain across the whole system (user preference: IDF, not Arduino/TFT_eSPI).

### Confirmed panel bring-up (from the spike)
- SPI: `SPI2_HOST`, SCLK 14, MOSI 13, MISO 12, CS 15, DC 2, RST −1 (software reset), pclk 20 MHz, mode 0.
- Backlight: GPIO 21, active HIGH.
- Panel: `esp_lcd_new_panel_st7789`, 240×320 native, 16 bpp, `LCD_RGB_ELEMENT_ORDER_RGB`, **invert OFF**.
- Color byte order: the panel reads RGB565 big-endian. Under LVGL this is handled by `CONFIG_LV_COLOR_16_SWAP=y` (equivalent to the manual `__builtin_bswap16` proven in the spike). Landscape via LVGL rotation (320×240).

## Components

1. **`status_model.h`** — `StatusModel` struct: `bool controllerReachable; bool boxPresent; bool boxArmed; bool boxLinkAlive; int rssi; uint16_t firedBitmap; bool seqRunning; uint32_t lastUpdateMs;`
2. **`status_parse.{h,cpp}`** — pure `bool parseStatus(const char* json, StatusModel& out)`; uses a JSON parser (cJSON, bundled with IDF) to populate the model from box 0. Returns false on malformed input. **Host-unit-testable** — the only logic we can verify off-hardware.
3. **`status_client.{h,cpp}`** — Wi-Fi STA connect to `FireControl`, periodic `esp_http_client` GET of `/api/status`, hands the body to `parseStatus`. Owns reconnect/poll timing. Sets `controllerReachable=false` on any HTTP/Wi-Fi failure.
4. **`display.{h,cpp}`** — esp_lcd ST7789 + LVGL init using the confirmed config above.
5. **`dashboard.{h,cpp}`** — builds the LVGL screen (status-hero layout) and an `update(const StatusModel&)` that refreshes labels/colors/cells. No business logic — pure render.
6. **`main.cpp`** — wire-up: init display, start Wi-Fi/client, every ~1 s pull the latest model and call `dashboard.update`.
7. **`secrets.example.h`** + git-ignored **`secrets.h`** (`FW_AP_SSID`, `FW_AP_PASS`) + `.gitignore`. The AP password never enters the repo (same pattern as the controller).

Project lives in its own top-level dir **`cyd-dashboard/`**, isolated from the box/controller IDF projects.

## Layout (status-hero, 320×240 landscape)

- **Hero banner** (top, full width): big text **ARMED** (red bg) / **SAFE** (green bg), driven by `boxArmed` (the box's echoed `state`, the truthful "is it hot"). Degraded states replace the banner text: **NO CONTROLLER** (grey) when `!controllerReachable`; **NO BOX LINK** (amber) when reachable but `!boxLinkAlive`.
- **Box line:** `Box0  LINK OK  -52 dBm` / `LINK DOWN`, from `boxLinkAlive` + `rssi`.
- **Sequence line:** `Sequence: idle | running`, from `seqRunning`.
- **Fired strip:** 16 small cells (2 rows × 8), a cell lit when its bit is set in `firedBitmap`.

## Data flow & refresh

`main` loop ticks at ~1 s: `status_client` performs one GET, `parseStatus` updates the shared `StatusModel`, `dashboard.update` redraws. LVGL's own timer task handles flushing. Only changed widgets are updated (LVGL invalidates dirty regions) to avoid flicker.

## Error handling / states

| Condition | Detection | UI |
|-----------|-----------|----|
| Booting / joining AP | Wi-Fi not yet `GOT_IP` | "CONNECTING TO FireControl…" splash |
| AP joined, no data yet | no successful GET | "WAITING FOR CONTROLLER…" |
| Controller unreachable / GET fails / Wi-Fi drop | http error or disconnect | banner → **NO CONTROLLER**; last data greyed + "stale Ns"; auto-reconnect |
| Reachable but box silent (`boxes[]` empty or `linkAlive=false`) | parsed model | banner → **NO BOX LINK** |
| Malformed JSON | `parseStatus` false | treated as a failed poll (keep last good model) |

Telemetry loss on the CYD is purely cosmetic — it has no control authority.

## Testing

- **Host (CTest):** `parseStatus()` against captured `/api/status` strings — full model (armed, link, rssi, firedBitmap, seqRunning), the empty-`boxes[]` case, and malformed input → false. This reuses the project's existing host-test harness pattern.
- **On hardware:** display bring-up already proven (spike). Then against the live controller: SAFE card with RSSI; ARM (jumper) → banner red; fire a channel → cell lights; pull box power → **NO BOX LINK**; pull controller power → **NO CONTROLLER**.

## Out of scope (YAGNI)

- Touch controls / arming / firing from the screen (safety; explicitly deferred — may revisit as "E-STOP-only" later).
- Multi-box display beyond box 0 (only box 0 is configured; the model/layout can extend when box 1 exists).
- Battery/voltage, historical graphs, brightness control.
