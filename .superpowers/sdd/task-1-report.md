## Task 1 Report: Host-test harness + IDF component skeleton + CRC32

### What Was Implemented

Per the brief, verbatim:

1. **`.gitignore`** — appended `build/` line.
2. **`components/fireworkcore/host_test/check.h`** — lightweight assert harness with `CHECK`, `CHECK_EQ`, `RUN`, `REPORT` macros. No external test framework dependency.
3. **`components/fireworkcore/host_test/test_crc32.cpp`** — two tests: known vector `CRC32("123456789") == 0xCBF43926`, and empty/null input == `0x00000000`.
4. **`components/fireworkcore/host_test/CMakeLists.txt`** — CMake project for host build using Ninja + g++, C++11, links `crc32.cpp` into `test_crc32` executable, registers with CTest.
5. **`components/fireworkcore/include/crc32.h`** — declaration of `fw::crc32(const uint8_t*, size_t)`.
6. **`components/fireworkcore/src/crc32.cpp`** — implementation: poly `0xEDB88320`, init `0xFFFFFFFF`, final XOR (`~crc`). No Arduino/IDF/FreeRTOS headers.
7. **`components/fireworkcore/CMakeLists.txt`** — IDF component registration stub listing `crc32.cpp`, `arming.cpp`, `sequence.cpp` (latter two do not exist yet; only consumed by `idf.py` in a later plan).

### TDD Evidence

**RED — Step 5 (before implementation files existed):**
```
cmake -S components/fireworkcore/host_test -B build/host_test -G Ninja -DCMAKE_CXX_COMPILER=g++
```
Output (exit code 1):
```
CMake Error at CMakeLists.txt:15 (add_executable):
  Cannot find source file:
    .../components/fireworkcore/src/crc32.cpp
  Tried extensions .c .C .c++ .cc .cpp ...
CMake Error at CMakeLists.txt:15 (add_executable):
  No SOURCES given to target: test_crc32
CMake Generate step failed.  Build files cannot be regenerated correctly.
```
Configure failed because `crc32.cpp` did not exist. Test could not compile — confirmed RED.

**GREEN — Step 8 (after implementation):**
```
cmake -S components/fireworkcore/host_test -B build/host_test -G Ninja -DCMAKE_CXX_COMPILER=g++
# -- Configuring done (0.2s)
# -- Generating done (0.1s)

cmake --build build/host_test
# [1/3] Building CXX object ... crc32.cpp.obj
# [2/3] Building CXX object ... test_crc32.cpp.obj
# [3/3] Linking CXX executable test_crc32.exe

ctest --test-dir build/host_test --output-on-failure
# Test project .../build/host_test
#     Start 1: test_crc32
# 1/1 Test #1: test_crc32 .......................   Passed    0.04 sec
# 100% tests passed, 0 tests failed out of 1
# Total Test time (real) =   0.05 sec
```

### Files Changed

| File | Action |
|------|--------|
| `.gitignore` | Modified — appended `build/` |
| `components/fireworkcore/CMakeLists.txt` | Created |
| `components/fireworkcore/include/crc32.h` | Created |
| `components/fireworkcore/src/crc32.cpp` | Created |
| `components/fireworkcore/host_test/check.h` | Created |
| `components/fireworkcore/host_test/CMakeLists.txt` | Created |
| `components/fireworkcore/host_test/test_crc32.cpp` | Created |

### Self-Review Findings

- All source files use only standard C++11 headers (`<cstdint>`, `<cstddef>`, `<cstdio>`). No Arduino/IDF/FreeRTOS includes present.
- CRC32 implementation matches the standard CRC-32/ISO-HDLC (aka "plain CRC32") algorithm: poly `0xEDB88320` (reflected), init `0xFFFFFFFF`, RefIn=true, RefOut=true, XorOut `0xFFFFFFFF`. Known-vector test confirms correctness.
- `test_crc32_empty_is_zero`: when `len == 0`, the while loop never executes, `crc` stays `0xFFFFFFFF`, `~crc == 0x00000000`. Correct.
- The IDF `CMakeLists.txt` lists `arming.cpp` and `sequence.cpp` — these do not exist and are not referenced by the host build, only by `idf.py` in a later plan. This is intentional per the brief.
- `g_failures` is a `static int` in `check.h`. If multiple TUs included `check.h`, each would get its own copy (ODR-safe for static, but counter would not accumulate across TUs). In this single-TU test binary that is fine. Later tasks that split tests across multiple `.cpp` files should move `g_failures` to a `.cpp` — noted as a concern for future tasks.
- Compiler: MinGW UCRT g++ 15.2.0 — confirmed by CMake detection output.

### Concerns

- **`g_failures` static in header**: safe for single-TU tests now, but will silently under-count failures if a later task links multiple test `.cpp` files into one executable. Will need to be addressed when that pattern appears.
- **No concerns blocking this task.** All deliverables are complete and verified.
