# Task 4 Report — ESP-NOW link + full app wiring

## Status: DONE

## Files Created
- `firmware/main/espnow_link.h` — EspNowLink class: `begin(controllerMac)`, `setOnCommand(cb)`, `sendAck(ack)`, `lastRxMs()`
- `firmware/main/espnow_link.cpp` — implementation with full WiFi stack init, esp_now init, peer add, static RX trampoline, lastRxMs_ tracking

## Files Modified
- `firmware/main/firing_box_main.cpp` — full app: NVS + I2C bus + ExpanderChannelDriver + BoxController + ArmSwitch + StatusLeds + EspNowLink; replaces StubChannelDriver skeleton
- `firmware/main/CMakeLists.txt` — added `espnow_link.cpp` to SRCS; added `esp_wifi nvs_flash esp_netif esp_event` to REQUIRES
- `firmware/main/status_leds.cpp` — safety hardening (see below)
- `firmware/main/expander_driver.cpp` — safety hardening (see below)

## Build Output (tail)
```
[5/9] Linking C static library esp-idf\main\libmain.a
[6/9] Generating esp-idf/esp_system/ld/sections.ld
[7/9] Linking CXX executable firing_box.elf
[8/9] Generating binary image from built executable
esptool v5.3.dev3
Successfully created ESP32 image.
Generated .../firmware/build/firing_box.bin
firing_box.bin binary size 0xbdff0 bytes. Smallest app partition is 0x100000 bytes. 0x42010 bytes (26%) free.
Project build complete.
```

## ESP-NOW / WiFi API Adjustments (IDF v6.0.1)

1. **`esp_now` is not a standalone component** — the task brief listed `esp_now` in CMakeLists REQUIRES, but IDF v6 folds `esp_now.h` into the `esp_wifi` component. Removed `esp_now` from REQUIRES; included via `esp_wifi`.

2. **Full WiFi stack init required before `esp_now_init()`** — the brief's `begin()` only called `esp_wifi_set_mode` + `esp_wifi_start`, but IDF v6 requires `esp_netif_init()` + `esp_event_loop_create_default()` + `esp_wifi_init(&cfg)` first (confirmed from espnow example at `C:\esp\v6.0.1\esp-idf\examples\wifi\espnow\main\espnow_example_main.c:45-59`). Added `esp_netif` and `esp_event` to REQUIRES.

3. **`i2c_new_master_bus` (not `i2c_master_new_master_bus`)** — the brief used `i2c_master_new_master_bus`, which does not exist in IDF v6.0.1. The correct function is `i2c_new_master_bus` (confirmed: `esp_driver_i2c/include/driver/i2c_master.h:122`). Fixed.

4. **RX callback signature** — `esp_now_recv_info_t*` is correct for IDF v6 (confirmed at example line 82). No change needed.

## ACK-Result Wiring
```
CommandResult::FIRED     → ACK with deviceStatus=1 (IGNITED_OK)
CommandResult::DUPLICATE → ACK with deviceStatus=2 (ALREADY_FIRED)  // self-heal lost ACK
CommandResult::REJECTED  → no ACK  // genuinely not fired; controller must know
CommandResult::IGNORED   → no ACK  // not addressed to this box or bad CRC
```
Per ACK-self-heal decision recorded in task-1-report (commit ff87246).

## Safety Hardening Applied

### status_leds.cpp
- Checked return value of `led_strip_new_rmt_device()` in `begin()`; on failure: log `ESP_LOGE`, set `strip_ = nullptr`, return early.
- Added `if (!strip_) return;` guard at top of `show()` — prevents a null-handle hard-fault if RMT init failed at runtime.

### expander_driver.cpp
- `allOff()` (E-STOP path): now captures return of `writeWord()` and logs `ESP_LOGE` on failure. A silent I2C failure during E-STOP means outputs may remain live — this makes the failure visible in the UART log.
- `setChannel()`: same treatment — `ESP_LOGE` on I2C write failure.
- `begin()`: replaced hardcoded `100000` Hz with `board::I2C_FREQ_HZ` (added `#include "board_config.h"`).

## Artifacts Not Staged
`firmware/build/`, `sdkconfig`, `managed_components/`, `dependencies.lock` are all gitignored. Only source files are staged.

## Concerns
One minor: `lastRxMs_` in `EspNowLink` is written from the WiFi task and read from the main control loop. On Xtensa, a 32-bit aligned read/write is atomic in practice, but marking it `volatile` would be cleaner for strict correctness. The brief uses `static volatile uint32_t lastRxMs` in `app_main`; the member-variable approach is equivalent for the 20 ms polling period. Low priority.
