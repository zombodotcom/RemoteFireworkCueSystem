# P3 Task 4 — ESP-NOW RX Race Fix Report

## Problem
The esp_now RX callback ran in the WiFi task and directly called `box.onCommand()` (mutating BoxController state + I²C writes) and `link.sendAck()`. The main task called `box.tick()` / `box.setPhysicalSwitch()` concurrently with no lock — a data race on `firing_[]`/`arm_` state AND concurrent I²C transmits on the same device handle (e.g. an E-STOP `allOff()` colliding with a `setChannel()`).

## Fix: queue the RX packet, process single-threaded in app_main

### `espnow_link.h`
- Removed `std::function` callback model (`setOnCommand`, `cb_`, `lastRxMs_`).
- Added `QueueHandle_t rxQueue_` and `bool receive(fw::CommandPacket& out)` (non-blocking dequeue).
- Kept `esp_now.h` include for `esp_now_recv_info_t` in the trampoline signature.

### `espnow_link.cpp`
- `begin()` creates the queue first (`xQueueCreate(16, sizeof(fw::CommandPacket))`) before registering the recv callback.
- `rxTrampoline` memcopies the raw packet and calls `xQueueSend(g_self->rxQueue_, &pkt, 0)` (non-blocking, WiFi task context — not ISR). Drops silently if queue is full; controller retry logic recovers.
- `receive()`: `xQueueReceive(rxQueue_, &out, 0) == pdTRUE`.
- `sendAck()`: unchanged.

### `firing_box_main.cpp`
- Removed `link.setOnCommand(...)` lambda.
- Removed `link.lastRxMs()` call (field deleted from EspNowLink).
- `lastRxMs` is now a plain local `uint32_t` in app_main — no cross-task writes.
- Main loop drains `link.receive()` before `box.tick()`: all BoxController mutations and I²C writes happen exclusively in the app_main task.
- Removed unused `<functional>` include.

## Invariant achieved
`box.onCommand()`, `box.setPhysicalSwitch()`, and `box.tick()` all run in the same task. No mutex needed; BoxController stays framework-free / host-testable.

## Build result
```
Project build complete.
firing_box.bin binary size 0xbde30 bytes. Smallest app partition is 0x100000 bytes. 0x421d0 bytes (26%) free.
```

## Artifacts staged
None — build/ directory is gitignored; only source files committed.
