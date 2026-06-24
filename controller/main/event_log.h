#pragma once
#include <cstdint>
#include <cstddef>
namespace ctrl {
enum Severity : uint8_t { SEV_INFO = 0, SEV_WARN = 1, SEV_ERR = 2 };
struct Event {
    uint32_t seq;
    uint32_t tMs;
    uint8_t  sev;
    char     msg[48];
};
class EventLog {
public:
    static const size_t CAP = 32;
    void     push(uint8_t sev, const char* msg, uint32_t tMs);
    size_t   since(uint32_t afterSeq, Event* out, size_t maxOut) const;
    uint32_t lastSeq() const { return lastSeq_; }
private:
    Event    buf_[CAP];
    size_t   count_   = 0;   // valid entries (<= CAP)
    size_t   head_    = 0;   // next write index
    uint32_t lastSeq_ = 0;   // last assigned seq (0 = none)
};
} // namespace ctrl
