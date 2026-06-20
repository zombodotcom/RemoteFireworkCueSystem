#pragma once
#include <cstdio>
#include <cstdint>

// NOTE: file-static counter. Each test executable links exactly one TU that
// includes this header (the test's .cpp), so the single counter is correct.
// If a test binary ever links two TUs that both include check.h, move this to
// a check.cpp (or an inline accessor) so failures are not under-counted.
static int g_failures = 0;

#define CHECK(cond) \
    do { if (!(cond)) { std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); ++g_failures; } } while (0)

#define CHECK_EQ(a, b) \
    do { long long _a=(long long)(a), _b=(long long)(b); \
         if (_a != _b) { std::printf("FAIL %s:%d: %s(%lld) == %s(%lld)\n", __FILE__, __LINE__, #a,_a,#b,_b); ++g_failures; } } while (0)

#define RUN(test) do { std::printf("RUN  %s\n", #test); test(); } while (0)

#define REPORT() ( g_failures == 0 ? (std::printf("OK\n"), 0) : (std::printf("%d FAILURE(S)\n", g_failures), 1) )
