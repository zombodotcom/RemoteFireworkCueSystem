# Task 1: Expander Codec (Pure, Host-Tested) — COMPLETE

## TDD: RED Phase

**Command:**
```bash
cmake -S firmware/test_codec -B build/test_codec -G Ninja -DCMAKE_CXX_COMPILER=g++
cmake --build build/test_codec
```

**Result: FAIL (expected)**
```
C:/Users/zombo/Desktop/Programming/RemoteFireworkCueSystem/RemoteFireworkCueSystem/firmware/test_codec/test_codec.cpp:2:10: fatal error: expander_codec.h: No such file or directory
    2 | #include "expander_codec.h"
      |          ^~~~~~~~~~~~~~~~~~
compilation terminated.
```

## TDD: GREEN Phase

**Files Created:**

1. **`firmware/main/expander_codec.h`** (19 lines)
   - `enum class FireLevel { ACTIVE_HIGH, ACTIVE_LOW };`
   - `uint16_t allOffWord(FireLevel lvl);` — returns 0x0000 (ACTIVE_HIGH) or 0xFFFF (ACTIVE_LOW)
   - `uint16_t applyChannel(uint16_t word, uint8_t channel, bool on, FireLevel lvl);` — bit-packing logic

2. **`firmware/test_codec/check.h`** (21 lines)
   - Copied verbatim from `components/fireworkcore/host_test/check.h`
   - Macros: `CHECK`, `CHECK_EQ`, `RUN`, `REPORT`

3. **`firmware/test_codec/test_codec.cpp`** (30 lines)
   - 4 tests: `test_alloff_word_per_polarity`, `test_apply_active_high`, `test_apply_active_low`, `test_out_of_range_no_change`

4. **`firmware/test_codec/CMakeLists.txt`** (8 lines)
   - Standard C++11 project, includes `firmware/main` for header

**Build Command:**
```bash
cmake --build build/test_codec
```

**Result: SUCCESS**
```
[1/2] Building CXX object CMakeFiles/test_codec.dir/test_codec.cpp.obj
[2/2] Linking CXX executable test_codec.exe
```

**Test Execution:**
```bash
ctest --test-dir build/test_codec --output-on-failure
```

**Result: ALL PASS**
```
Test project C:/Users/zombo/Desktop/Programming/RemoteFireworkCueSystem/RemoteFireworkCueSystem/build/test_codec
    Start 1: test_codec
1/1 Test #1: test_codec .......................   Passed    0.04 sec

100% tests passed, 0 tests failed out of 1

Total Test time (real) =   0.05 sec
```

**Direct Test Output:**
```
RUN  test_alloff_word_per_polarity
RUN  test_apply_active_high
RUN  test_apply_active_low
RUN  test_out_of_range_no_change
OK
```

## Implementation Details

**Codec Logic:**
- **ACTIVE_HIGH:** On state = bit HIGH (1), Off state = bit LOW (0)
  - `allOffWord()` → 0x0000
  - `applyChannel(word, ch, true, ACTIVE_HIGH)` → sets bit `ch`
  - `applyChannel(word, ch, false, ACTIVE_HIGH)` → clears bit `ch`

- **ACTIVE_LOW:** On state = bit LOW (0), Off state = bit HIGH (1)
  - `allOffWord()` → 0xFFFF
  - `applyChannel(word, ch, true, ACTIVE_LOW)` → clears bit `ch` (inverted logic)
  - `applyChannel(word, ch, false, ACTIVE_LOW)` → sets bit `ch` (inverted logic)

- **Out-of-range:** Channels >= 16 return word unchanged (no-op safety)

**C++ Details:**
- Plain C++11, header-only (inline functions)
- Only dependency: `<cstdint>`
- No framework, no ESP-IDF, no hardware — pure logic

## Test Coverage

| Test | Purpose | Coverage |
|------|---------|----------|
| `test_alloff_word_per_polarity` | Verify initial states | ACTIVE_HIGH/LOW polarity paths |
| `test_apply_active_high` | Active-high bit logic | Set bit 0, clear bit 0, set bit 15 |
| `test_apply_active_low` | Active-low inverted logic | Clear bit 0 (on), set bit 0 (off) |
| `test_out_of_range_no_change` | Boundary guard | Channel 16 (out-of-range) |

All 4 tests **PASS**.

## Notes

- ✅ Codec is pure, testable, framework-free
- ✅ Ready for I²C driver integration (next task)
- ✅ Polarity logic is correct for both ACTIVE_HIGH and ACTIVE_LOW (flexible for SSR board decisions ~2026-06-24)
- ✅ Separate build directory (`build/test_codec`) keeps independent from main host suite
- ✅ No concerns — implementation matches spec exactly
