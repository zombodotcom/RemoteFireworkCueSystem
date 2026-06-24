# Task 5 Report — Controller: fold telemetry into snapshot + `/api/status`

## Status: DONE

This is a build-only firmware task (no TDD).

---

## Implemented

### Step 1 — `controller/main/web_server.h`
Added `BoxTelemetry` struct with volatile fields (`linkAlive`, `rssi`, `state`, `firedBitmap`, `lastFiredChannel`) immediately before `StatusSnapshot`. Added `BoxTelemetry boxes[2];` as the last field of `StatusSnapshot`.

### Step 2 — `controller/main/controller_main.cpp`
- Added `static uint32_t lastStatusMs[2] = {0, 0};` before the `while(true)` loop.
- After the existing ACK drain block, added: drain of `tx.receiveStatus(sr)` writing fields into `g_status.boxes[sr.boxId]` and stamping `lastStatusMs[sr.boxId] = now`.
- Followed immediately by a `for (int b = 0; b < 2; b++)` loop computing `linkAlive = (lastStatusMs[b] != 0) && ((now - lastStatusMs[b]) < 1500)`.
- The drain runs in the control loop task only (same task as the ACK drain), not in any callback.

### Step 3 — `controller/main/web_server.cpp`
- Enlarged `buf` from `char buf[128]` to `char buf[420]`.
- Replaced the old `if (lastBox == 0xFF) { snprintf(...) } else { snprintf(...) }` block entirely.
- New builder: pre-formats `lastBoxTok` as `"null"` or the decimal number; then builds the JSON header with `snprintf`; then loops over both boxes appending the `boxes[]` array entries; then appends `"]}"`  — all using the incremental `n += snprintf(buf + n, ...)` pattern.
- The existing `httpd_resp_set_type` + `httpd_resp_send` tail is unchanged.
- Existing top-level keys (`armed`, `seqRunning`, `lastFailedBox`) are preserved exactly.

---

## Build Evidence

```
Project build complete. To flash, run: idf.py flash
controller.bin binary size 0xcba00 bytes. Smallest app partition is 0x100000 bytes. 0x34600 bytes (20%) free.
```

No warnings or errors. Binary increased within flash budget.

---

## Example JSON Output (hand-traced, idle state)

```json
{"armed":false,"seqRunning":false,"lastFailedBox":null,"boxes":[{"id":0,"linkAlive":false,"rssi":0,"state":0,"firedBitmap":0,"lastFired":-1},{"id":1,"linkAlive":false,"rssi":0,"state":0,"firedBitmap":0,"lastFired":-1}]}
```

When box 0 is live (ARMED, RSSI -62, channel 3 last fired, firedBitmap 0x0009):
```json
{"armed":true,"seqRunning":false,"lastFailedBox":null,"boxes":[{"id":0,"linkAlive":true,"rssi":-62,"state":1,"firedBitmap":9,"lastFired":3},{"id":1,"linkAlive":false,"rssi":0,"state":0,"firedBitmap":0,"lastFired":-1}]}
```

---

## Buffer Size Verification

Worst-case single box entry (all fields at max):
- `{"id":1,"linkAlive":true,"rssi":-128,"state":1,"firedBitmap":65535,"lastFired":15}` = ~82 chars
- Two boxes + separator comma = ~165 chars
- Header `{"armed":false,"seqRunning":false,"lastFailedBox":255,"boxes":[` = ~63 chars
- Closing `]}` = 2 chars
- Total worst case ~230 chars — well under `buf[420]`.

---

## Files Changed

- `controller/main/web_server.h` — added `BoxTelemetry` struct; added `boxes[2]` to `StatusSnapshot`
- `controller/main/controller_main.cpp` — added `lastStatusMs[2]`; added status drain + linkAlive computation after ACK drain
- `controller/main/web_server.cpp` — enlarged buf to 420; replaced old if/else snprintf with new builder

---

## Self-Review Checklist

- [x] JSON validity: commas between box objects (ternary `b ? "," : ""`); closing `]}` appended; `lastFailedBox` is `null` or a bare number (no quotes around number); `lastFired` is `-1` when `lastFiredChannel == 0xFF`.
- [x] `buf[420]` is large enough — worst case ~230 chars.
- [x] OLD if/else snprintf block fully removed — no dead code, no duplicate snprintf calls, no leftover variables (`buf[128]` gone).
- [x] Existing top-level keys (`armed`, `seqRunning`, `lastFailedBox`) are unchanged and in the same order.
- [x] Status drain is in the control loop only, immediately after the ACK drain — not in a callback.
- [x] `linkAlive` freshness: 1500 ms window, guarded by `lastStatusMs[b] != 0` so it stays false until the first status is received.
- [x] `receiveStatus()` is a Task 4 interface — consumed here but not modified.
- [x] Firing/arming logic untouched.

---

## Commit

`e3102f9` feat(ctrl): surface box telemetry (state/link/rssi/fired) on /api/status

## Concerns

None. The implementation matches the brief verbatim, the build is clean, and the buffer math has comfortable headroom.

---

## Fix wave 1

**Commit:** `fix(ctrl): snapshot volatile box telemetry + guard snprintf size in /api/status`

### What changed (`controller/main/web_server.cpp`, `handle_status` only)

**Fix 1 — volatile snapshot consistency:**
Added `BoxTelemetry box = g_status.boxes[b];` at the top of the `for` loop body. Copying the struct to a non-volatile local performs one read per volatile member (five total per iteration), eliminating the risk of the compiler re-reading any field between argument evaluations in the `snprintf` call. All six format arguments now reference `box.*` instead of `g_status.boxes[b].*`. This is consistent with the existing discipline of snapshotting `armed`/`seq`/`lastBox` into locals before formatting.

**Fix 2 — truncation-guard cast:**
The closing `"]}"`  snprintf (previously `snprintf(buf + n, sizeof(buf) - n, "]}")`) now computes `size_t rem2 = (n < (int)sizeof(buf)) ? sizeof(buf) - (size_t)n : 0` and passes `rem2`. The per-box snprintf inside the loop also uses the same `rem` guard pattern. The initial header snprintf still uses `sizeof(buf)` directly (n=0 at that point, so `sizeof(buf) - 0` cannot wrap). This eliminates the `size_t`/`int` mixed-sign subtraction that would wrap to a huge value if `n` ever exceeded the buffer.

**JSON output:** byte-identical to before for the normal (non-overflow) case. The `rem` guards only affect behavior when `n >= sizeof(buf)`, which cannot happen given the ~230-byte worst-case content in a 420-byte buffer.

### Build result

```
Project build complete. To flash, run: idf.py flash
controller.bin binary size 0xcba40 bytes. Smallest app partition is 0x100000 bytes. 0x345c0 bytes (20%) free.
```

Zero warnings, zero errors. Binary size increased by 64 bytes vs Task 5 baseline (guard logic + snapshot copy).

### Verification

ESP-IDF v6.0.1 build in PowerShell: clean compile, `Project build complete.`
