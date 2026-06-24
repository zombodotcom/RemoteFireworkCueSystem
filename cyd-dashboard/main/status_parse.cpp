#include "status_parse.h"
#include <cstring>
#include <cstdlib>
#include <cstdio>

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

    // lastHeardMs is inside the first box object (obj scope)
    if (intAt(valueAfter(obj, "lastHeardMs"), iv)) out.boxLastHeardMs = (uint32_t)iv;

    // diag block (top-level)
    const char* d = std::strstr(json, "\"diag\"");
    if (d) {
        if (intAt(valueAfter(d, "uptimeMs"), iv))  out.diag.uptimeMs  = (uint32_t)iv;
        if (intAt(valueAfter(d, "freeHeap"), iv))  out.diag.freeHeap  = (uint32_t)iv;
        if (intAt(valueAfter(d, "apClients"), iv)) out.diag.apClients = (uint32_t)iv;
        if (intAt(valueAfter(d, "fired"), iv))     out.diag.fired     = (uint32_t)iv;
        if (intAt(valueAfter(d, "acked"), iv))     out.diag.acked     = (uint32_t)iv;
        if (intAt(valueAfter(d, "failed"), iv))    out.diag.failed    = (uint32_t)iv;
        if (intAt(valueAfter(d, "retries"), iv))   out.diag.retries   = (uint32_t)iv;
        if (intAt(valueAfter(d, "lastAckMs"), iv)) out.diag.lastAckMs = (uint32_t)iv;
    }
    // fault block (top-level)
    const char* f = std::strstr(json, "\"fault\"");
    if (f) {
        if (boolAt(valueAfter(f, "active"), bv)) out.faultActive = bv;
        const char* mp = valueAfter(f, "msg");   // points at the opening quote
        if (mp && *mp == '"') {
            ++mp; int k = 0;
            while (*mp && *mp != '"' && k < (int)sizeof(out.faultMsg) - 1) out.faultMsg[k++] = *mp++;
            out.faultMsg[k] = '\0';
        }
    }
    return true;
}

int parseEvents(const char* json, LogEv* out, int maxOut) {
    if (!json || maxOut <= 0) return 0;
    const char* p = std::strstr(json, "\"events\"");
    if (!p) return 0;
    p = std::strchr(p, '[');
    if (!p) return 0;
    int n = 0;
    while (n < maxOut) {
        const char* obj = std::strchr(p, '{');
        if (!obj) break;
        // stop if the next '{' is past the array close ']'
        const char* close = std::strchr(p, ']');
        if (close && close < obj) break;
        int iv;
        out[n].seq = intAt(valueAfter(obj, "seq"), iv) ? iv : 0;
        out[n].sev = intAt(valueAfter(obj, "sev"), iv) ? iv : 0;
        out[n].msg[0] = '\0';
        const char* mp = valueAfter(obj, "msg");
        if (mp && *mp == '"') {
            ++mp; int k = 0;
            while (*mp && *mp != '"' && k < (int)sizeof(out[n].msg) - 1) out[n].msg[k++] = *mp++;
            out[n].msg[k] = '\0';
        }
        ++n;
        p = std::strchr(obj, '}');     // advance past this object
        if (!p) break;
        ++p;
    }
    return n;
}
