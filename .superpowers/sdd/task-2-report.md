# Task 2: board_config.h + ExpanderChannelDriver (I2C) — COMPLETE

## Files Created

- `firmware/main/board_config.h` — pins, polarity (ACTIVE_LOW default), MAC placeholder, PCF8575 address
- `firmware/main/expander_driver.h` — `ExpanderChannelDriver : public fw::ChannelDriver` declaration
- `firmware/main/expander_driver.cpp` — PCF8575 I2C write via `i2c_master_transmit`

## Files Modified

- `firmware/main/CMakeLists.txt` — added `expander_driver.cpp` to SRCS; added `esp_driver_i2c` to REQUIRES (see API adjustment below)

## API Adjustment vs Brief

The brief says to add `driver` to CMakeLists REQUIRES. In IDF v6.0.1, the new i2c_master driver lives in the `esp_driver_i2c` component (not the legacy `driver` component). The header path `driver/i2c_master.h` is correct (it is in the `include/driver/` subdir of `esp_driver_i2c`), but the CMake component name must be `esp_driver_i2c`. This was verified by inspecting:
- `C:\esp\v6.0.1\esp-idf\components\esp_driver_i2c\CMakeLists.txt` (INCLUDE_DIRS = "include")
- `C:\esp\v6.0.1\esp-idf\examples\peripherals\i2c\i2c_tools\main\CMakeLists.txt` (uses PRIV_REQUIRES esp_driver_i2c)

All other code in `expander_driver.cpp` matches the brief verbatim:
- `i2c_device_config_t` with `dev_addr_length = I2C_ADDR_BIT_LEN_7`, `device_address`, `scl_speed_hz` confirmed correct by inspecting the IDF header and test examples
- `i2c_master_bus_add_device`, `i2c_master_transmit` both confirmed present in v6.0.1

## Build Command

```powershell
& 'C:\esp\v6.0.1\esp-idf\export.ps1' *> $null
Set-Location 'C:\Users\zombo\Desktop\Programming\RemoteFireworkCueSystem\RemoteFireworkCueSystem\firmware'
idf.py build 2>&1 | Select-Object -Last 30
```

## Build Output (tail)

```
Successfully created ESP32 image.

Generated .../firmware/build/firing_box.bin

firing_box.bin binary size 0x22180 bytes. Smallest app partition is 0x100000 bytes. 0xdde80 bytes (87%) free.

Project build complete. To flash, run:
 idf.py flash
```

## Artifacts Not Staged

`firmware/build/`, `sdkconfig`, and `managed_components/` are gitignored. Only source files are committed.

## CMakeLists Note for Tasks 3-4

The brief's final CMakeLists lists `arm_switch.cpp`, `status_leds.cpp`, `espnow_link.cpp` (Tasks 3-4). These are intentionally omitted from SRCS for this build-only task and will be added when those tasks create the files. The brief explicitly permits: "temporarily list only the files that exist."

## Concerns

None. Build links cleanly. The `driver` -> `esp_driver_i2c` CMake component name is the only delta from the brief; the `#include "driver/i2c_master.h"` header path is unchanged and correct.
