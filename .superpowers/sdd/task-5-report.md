## Task 5 Report: Wire-up + hardware verification

### Status
DONE_WITH_CONCERNS

### What was implemented

Replaced `cyd-dashboard/main/main.cpp` with the verbatim brief code:
- Calls `display_init()` and captures the returned `lv_display_t*`
- Passes that pointer to `touch_init(disp)`
- Calls `dashboard_create()` and `wifi_sta_start()`
- Loop: polls status + events when WiFi connected; advances `sinceSeq` (by-ref); feeds non-empty event batches to `dashboard_set_events`; calls `dashboard_update(model)` every 1000ms

`display.h` and `display.cpp` were already correct — both already declared/implemented `display_init` returning `lv_display_t*` (prior task work). No changes needed there.

### Build result

```
Project build complete.
cyd_dashboard.bin binary size 0x13c9b0 bytes. Smallest app partition is 0x200000 bytes. 0xc3650 bytes (38%) free.
```

### Flash result

```
Hash of data verified.   (bootloader)
Hash of data verified.   (partition table)
Hash of data verified.   (app)
```

Flashed via: `python -m esptool --chip esp32 -p COM12 -b 460800 --before default_reset --after hard_reset write_flash --flash_mode dio --flash_freq 40m --flash_size 4MB ...`

Note: the `flash_args` file uses hyphen syntax (`--before default-reset`) but the installed esptool v4.9.0 requires underscores. Used explicit expanded command successfully.

### Serial tail (no panic confirmed)

```
I (845) LVGL: Starting LVGL task
I (1123) wifi:wifi driver task: 3ffdea94, prio:23, stack:6656, core=0
I (1215) phy_init: phy_version 4863,a3a4459,Oct 28 2025,14:30:06
I (1303) wifi:mode : sta (a4:f0:0f:5f:4f:00)
I (1332) wifi:state: init -> auth (0xb0)
I (1339) wifi:state: auth -> assoc (0x0)
I (3006) wifi:state: assoc -> run (0x10)
I (3045) wifi:connected with FireControl, aid = 1, channel 1, 40U, bssid = b0:cb:d8:89:9e:69
I (3056) wifi:security: WPA2-PSK-SHA256, phy: bgn, rssi: -40
I (4099) wifi_sta: got IP ??? connected to FireControl
I (4938) status_client: armed=0 link=1 rssi=-21 fired=0x0000 seq=0 present=1
I (6448) status_client: armed=0 link=1 rssi=-21 fired=0x0000 seq=0 present=1
I (7960) status_client: armed=0 link=1 rssi=-21 fired=0x0000 seq=0 present=1
```

No `Guru Meditation`, no abort, no reboot loop. Boot + LVGL start + WiFi join to FireControl AP + status_client poll loop all confirmed.

### Files changed

- `cyd-dashboard/main/main.cpp` — replaced with brief-specified wire-up (touch_init with disp, events poll, sinceSeq advance, dashboard_set_events)
- `display.h` and `display.cpp` — no changes needed (already correct from prior tasks)

### Self-review

- `display_init()` returns `lv_display_t*` — confirmed in display.h (line 6) and display.cpp (line 72, returns `lvgl_port_add_disp(&disp_cfg)`)
- `touch_init(disp)` called with the returned pointer — correct
- `status_client_poll_events(sinceSeq, evs, 16)` — `sinceSeq` is `uint32_t&` (by ref), advances automatically inside the function — correct
- `dashboard_set_events(evs, n)` only called when `n > 0` — correct
- `dashboard_update(model)` called on every iteration regardless of WiFi — correct
- `model.controllerReachable = false` set when WiFi not connected — correct
- Static locals (`model`, `evs`, `sinceSeq`) avoid stack pressure in `app_main` — correct

### Commit

`4b052d3` feat(cyd): wire touch + events poll + tap-paged dashboard end-to-end

## Final-review hardening

Three hardening-only changes applied (no behavior change at current data sizes):

1. **controller/main/web_server.cpp** — `handle_events`: grew `char buf[2600]` to `char buf[3200]`; added early-exit guard `if (p > (int)sizeof(buf) - 120) break;` as the first statement of the per-event loop, ensuring the trailing `]}` snprintf always has space and the JSON array is always well-formed.

2. **cyd-dashboard/main/status_client.cpp** — `struct AccumE`: grew `char buf[2700]` to `char buf[3300]` so the CYD receive buffer can accommodate the full 3200-byte controller response.

3. **cyd-dashboard/main/status_parse.cpp** — `parseEvents`: added comment `// NOTE: assumes event "msg" strings contain no ']' (controller-authored, fixed format).` at top of function body documenting the parser's assumption.

Build results (both `Project build complete.`):
- `controller`: `Project build complete.`
- `cyd-dashboard`: `Project build complete.`

### Concerns (pending user verification)

1. **Visual tap-through pending user**: The 3-page tap-paged dashboard (Dashboard / Event Log / Diagnostics), page dot fault badge, and slim state bars are all LVGL work from Tasks 3-4. This task only wires up `main.cpp`; visual correctness and touch orientation require physical device inspection by user.
2. **Touch orientation**: The XPT2046 `swap_xy` / mirror flags in `touch.cpp` may need adjustment by eye once the user taps through the pages. Swipe direction and hit areas may be inverted depending on panel orientation.
3. **`flash_args` hyphen/underscore mismatch**: The generated `flash_args` file uses `--before default-reset` (hyphen) but esptool v4.9.0 requires `--before default_reset` (underscore) and `write_flash` (underscore). The brief's flash command also uses hyphens. Future flash commands will need the underscore form.
