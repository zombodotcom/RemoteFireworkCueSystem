# Task 4a Report — Controller HTTP API

## Files Created / Modified

| File | Change |
|---|---|
| `controller/main/web_server.h` | Created — `CmdType` enum, `UiCommand` struct, `StatusSnapshot` struct, `WebServer` class, `constexpr MAX_RUN_STEPS = 64`, `extern` declarations for shared globals |
| `controller/main/web_server.cpp` | Created — all 9 route handlers + `WebServer::start()`, hand-rolled JSON parser |
| `controller/main/controller_main.cpp` | Modified — creates queue + snapshot, starts `WebServer`, drains cmd queue in loop before `runner.tick()`, writes snapshot after tick |
| `controller/main/CMakeLists.txt` | Modified — added `web_server.cpp` to SRCS; added `esp_http_server` to REQUIRES |

## Build Tail

```
controller.bin binary size 0xc8270 bytes. Smallest app partition is 0x100000 bytes.
0x37d90 bytes (22%) free.

Project build complete.
```

ESP-IDF v6.0.1, target ESP32, branch `32ch-webui-show`, commit `6b97334`.

## Command-Queue + Snapshot Concurrency Design

### Command queue (FreeRTOS `QueueHandle_t g_cmdQueue`)

- Depth 8, element type `UiCommand` (3 bytes: `CmdType` + `boxId` + `channel`).
- **Producers:** HTTP handler functions in the httpd task — call `xQueueSend(&cmd, 0)` (non-blocking). If the queue is full they respond HTTP 503.
- **Consumer:** The control-loop task, which calls `xQueueReceive(&cmd, 0)` in a `while` loop at the top of each 20 ms tick, before `runner.tick()`.
- **Invariant:** `runner.*` and `link.*` are only ever called from the control-loop task. HTTP handlers never touch them directly.

### RUN step buffer (SPSC shared memory)

- `fw::SeqStep g_runSteps[64]` + `size_t g_runCount` — plain globals defined in `controller_main.cpp`, declared `extern` in `web_server.h`.
- **Protocol:** `/api/run` handler writes `g_runSteps[]` and `g_runCount` **before** posting `RUN` to the queue. The control-loop reads them **only** when it dequeues a `RUN` command.
- Single-producer / single-consumer pattern — no mutex needed. A second `/api/run` request races only with itself (same httpd task is single-threaded per connection by default).

### Status snapshot (`StatusSnapshot g_status`)

- Three `volatile` fields: `armed` (bool), `seqRunning` (bool), `lastFailedBox` (uint8_t, 0xFF = none).
- **Writer:** Control loop, after `runner.tick()` each 20 ms tick.
- **Reader:** `GET /api/status` handler.
- `volatile` is sufficient on Xtensa: boolean and byte reads/writes are single instructions; no torn reads possible. No mutex or `std::atomic` needed for this handful of status flags.

## Endpoints

| Method | Path | Body | Response |
|---|---|---|---|
| GET | `/` | — | `text/html` placeholder page |
| POST | `/api/arm` | — | `{"ok":true}` |
| POST | `/api/disarm` | — | `{"ok":true}` |
| POST | `/api/estop` | — | `{"ok":true}` |
| POST | `/api/stop` | — | `{"ok":true}` |
| POST | `/api/heartbeat` | — | `{"ok":true}` (heartbeat is implicit in `runner.tick()`) |
| POST | `/api/fire` | `{"box":N,"channel":M}` | `{"ok":true}` |
| POST | `/api/run` | `{"steps":[[timeMs,box,ch],...]}` | `{"ok":true}` |
| GET | `/api/status` | — | `{"armed":bool,"seqRunning":bool,"lastFailedBox":N\|null}` |

Error responses: HTTP 400 `{"ok":false,"error":"..."}` for bad input; HTTP 503 `{"ok":false,"error":"queue full"}` if the command queue is saturated.

## esp_http_server API Notes (v6.0.1)

- `httpd_start(&server, &config)` — config via `HTTPD_DEFAULT_CONFIG()` macro; `config.max_uri_handlers` raised to 16 (default 8 is not enough for 9 routes).
- `httpd_register_uri_handler(server, &uri)` — `httpd_uri_t` has `.uri`, `.method`, `.handler`, `.user_ctx`. Designated initializers in a `static const` array cause C++ pedantic warnings with some compiler flags; avoided by using a local `Route` struct + field-by-field `httpd_uri_t` init in a loop.
- `httpd_req_recv(req, buf, len)` — returns bytes read, 0 on close, negative on error. Loop until `content_len` satisfied.
- `httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN)` — sends response and closes connection.
- `httpd_resp_set_status(req, "503 Service Unavailable")` — must be called before `httpd_resp_send`.

## cJSON Situation

`cJSON` is **not** a built-in IDF component in v6.0.1 — it is a managed component (`espressif/cjson`, requires `idf.py add-dependency` + internet). The plan assumed `json` was a built-in alias; it is not. Replaced with a hand-rolled parser (`strstr` + `sscanf`/`strtol`) that handles exactly the two input shapes (`fire` and `run`). This keeps the build dependency-free and avoids component manager invocation.

## Concerns

1. **Hand-rolled JSON parser is not general-purpose.** It handles only the two fixed schemas. Any future endpoint with a different shape needs its own parser snippet. Adding `espressif/cjson` via the component manager (requires one-time internet access) would be cleaner for Task 4b+.
2. **`lastFailedBox` is always 0xFF.** `ShowRunner` does not currently expose per-box failure info. The field is wired up and ready; the runner API needs extending to populate it.
3. **HEARTBEAT command is a no-op in the loop.** The runner sends heartbeats automatically in `tick()`. The endpoint exists for keep-alive signaling from the UI if needed, but the loop currently discards it. This is intentional and correct for now.
4. **No authentication.** The SoftAP is WPA2 (`pyro1234`), which provides network-layer access control, but the HTTP API has no token/session auth. Acceptable for a closed local network; worth noting before any internet-exposed deployment.
5. **Single httpd worker thread.** `HTTPD_DEFAULT_CONFIG()` uses one worker. `/api/run` with a large step array (4 KB body) could block the httpd task for a few milliseconds. Not an issue at 64-step cap but worth revisiting if the cap grows.
