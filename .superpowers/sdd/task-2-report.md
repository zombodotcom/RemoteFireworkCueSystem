# Task 2: parseStatus() + host test (TDD) — Report

## Status
**DONE**

## Commit
- SHA: `4e90e2e`
- Message: `feat(cyd): host-tested parseStatus() for /api/status JSON`

## Implementation Summary

Implemented Task 2 using strict TDD: RED → GREEN → COMMIT cycle. Created a pure C++ parser (`parseStatus()`) that extracts fields from the controller's `/api/status` JSON response. No IDF headers, no dependencies — builds cleanly with standard C++ on any platform.

### Files Created
1. **`cyd-dashboard/main/status_model.h`** — Struct with 8 fields (controllerReachable, boxPresent, boxArmed, boxLinkAlive, rssi, firedBitmap, seqRunning, lastUpdateMs)
2. **`cyd-dashboard/main/status_parse.h`** — Pure function declaration: `bool parseStatus(const char* json, StatusModel& out);`
3. **`cyd-dashboard/main/status_parse.cpp`** — Full implementation using string scanning (NO cJSON, NO IDF headers; pure C++11 stdlib)
4. **`cyd-dashboard/host_test/check.h`** — Project's host-test assertion macros (CHECK, CHECK_EQ, RUN, REPORT)
5. **`cyd-dashboard/host_test/test_status_parse.cpp`** — 4 test functions with 10 assertions
6. **`cyd-dashboard/host_test/CMakeLists.txt`** — Cross-platform build config (includes ../main/status_parse.cpp)

## TDD Evidence

### RED Phase (Test Failure)
Initial stub implementation returned `false` always. Tests compiled and ran, showing 10 failures:
```
RUN  test_parse_armed_box
FAIL ... parseStatus(JSON_ARMED, m)        [returned false, wanted true]
FAIL ... m.boxPresent                      [false, wanted true]
FAIL ... m.boxArmed
FAIL ... m.boxLinkAlive
FAIL ... m.rssi(0) == -52(-52)             [0 vs -52]
FAIL ... (int)m.firedBitmap(0) == 5(5)    [0 vs 5]
RUN  test_parse_safe_nolink_seq
FAIL ... parseStatus(JSON_SAFE_NOLINK, m)
FAIL ... m.boxPresent
FAIL ... m.seqRunning
RUN  test_parse_empty_boxes
FAIL ... parseStatus(JSON_EMPTY_BOXES, m)
RUN  test_parse_malformed_returns_false
10 FAILURE(S)
```

### GREEN Phase (Implementation & Tests Pass)
Full implementation in `status_parse.cpp`:
- `valueAfter(from, key)` — locates `"key":` patterns and returns first value char (prevents "firedBitmap" matching "lastFired")
- `boolAt(p, out)` — parses "true"/"false" JSON literals
- `intAt(p, out)` — parses signed integers via strtol
- `parseStatus(json, out)` main logic:
  - Null/empty JSON → return false
  - No "boxes" field → return false  
  - Empty boxes array → return true, boxPresent=false
  - Populated box object → extract linkAlive, rssi, state (→ boxArmed if ==1), firedBitmap

All 4 tests now PASS:
```
RUN  test_parse_armed_box
RUN  test_parse_safe_nolink_seq
RUN  test_parse_empty_boxes
RUN  test_parse_malformed_returns_false
OK
```

## Files Changed

| File | Status |
|------|--------|
| `cyd-dashboard/main/status_model.h` | Created |
| `cyd-dashboard/main/status_parse.h` | Created |
| `cyd-dashboard/main/status_parse.cpp` | Created |
| `cyd-dashboard/host_test/check.h` | Created |
| `cyd-dashboard/host_test/test_status_parse.cpp` | Created |
| `cyd-dashboard/host_test/CMakeLists.txt` | Created |

## Self-Review Checklist

- [x] **Parser is pure C++** — No ESP-IDF, Arduino, or platform-specific headers; uses only `<cstring>` and `<cstdlib>`
- [x] **Empty boxes returns true + boxPresent=false** — Correctly skips box extraction and returns success
- [x] **Malformed/null/empty returns false** — Explicit null/empty checks; missing "boxes" → false
- [x] **state==1 → boxArmed** — Correctly maps: `out.boxArmed = (iv == 1)`
- [x] **Key collision prevention** — `valueAfter()` searches for exact `"key":` patterns; "firedBitmap" won't match "lastFired"
  - Tested with JSON having both `"firedBitmap":5` and `"lastFired":2` — correctly extracts 5
- [x] **All 4 tests pass** — test_parse_armed_box, test_parse_safe_nolink_seq, test_parse_empty_boxes, test_parse_malformed_returns_false
- [x] **No IDF/Arduino includes** — Parser compiles cleanly as pure C++ on Windows with standard headers

## Test Coverage

| Test | Purpose | Result |
|------|---------|--------|
| `test_parse_armed_box` | Armed state, link alive, RSSI -52, bitmap 5 | ✓ PASS |
| `test_parse_safe_nolink_seq` | Disarmed, link dead, RSSI 0, seqRunning true | ✓ PASS |
| `test_parse_empty_boxes` | Valid JSON but no box entries | ✓ PASS |
| `test_parse_malformed_returns_false` | Null, empty, missing "boxes" field | ✓ PASS |

## Concerns

None. Implementation is complete, tested, and follows the brief exactly.

---

**Completed:** 2026-06-23
