#include "status_parse.h"
#include <cstring>
#include <cstdlib>

// Return a pointer to the first value char after `"key":`, searching from `from`.
static const char* valueAfter(const char* from, const char* key) {
    if (!from) return nullptr;
    char pat[40];
    int n = 0;
    pat[n++] = '"';
    for (const char* k = key; *k && n < 37; ++k) pat[n++] = *k;
    pat[n++] = '"';
    pat[n] = 0;
    const char* p = std::strstr(from, pat);
    if (!p) return nullptr;
    p = std::strchr(p + n, ':');
    if (!p) return nullptr;
    ++p;
    while (*p == ' ' || *p == '\t') ++p;
    return p;
}
static bool boolAt(const char* p, bool& out) {
    if (!p) return false;
    if (std::strncmp(p, "true", 4) == 0)  { out = true;  return true; }
    if (std::strncmp(p, "false", 5) == 0) { out = false; return true; }
    return false;
}
static bool intAt(const char* p, int& out) {
    if (!p) return false;
    char* end = nullptr;
    long v = std::strtol(p, &end, 10);
    if (end == p) return false;
    out = (int)v;
    return true;
}

bool parseStatus(const char* json, StatusModel& out) {
    if (!json || !*json) return false;

    bool b;
    if (boolAt(valueAfter(json, "seqRunning"), b)) out.seqRunning = b;

    const char* boxes = std::strstr(json, "\"boxes\"");
    if (!boxes) return false;                 // malformed for our purposes
    const char* lb = std::strchr(boxes, '[');
    if (!lb) return false;
    const char* q = lb + 1;
    while (*q == ' ' || *q == '\t' || *q == '\n' || *q == '\r') ++q;
    if (*q == ']') { out.boxPresent = false; return true; }   // empty array

    const char* obj = std::strchr(lb, '{');
    if (!obj) { out.boxPresent = false; return true; }
    out.boxPresent = true;

    int iv; bool bv;
    if (boolAt(valueAfter(obj, "linkAlive"), bv))  out.boxLinkAlive = bv;
    if (intAt(valueAfter(obj, "rssi"), iv))        out.rssi = iv;
    if (intAt(valueAfter(obj, "state"), iv))       out.boxArmed = (iv == 1);
    if (intAt(valueAfter(obj, "firedBitmap"), iv)) out.firedBitmap = (uint16_t)iv;
    if (intAt(valueAfter(obj, "lastFired"), iv))   out.lastFired = iv;
    return true;
}
