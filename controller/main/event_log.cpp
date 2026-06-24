#include "event_log.h"
#include <cstring>
namespace ctrl {

void EventLog::push(uint8_t sev, const char* msg, uint32_t tMs) {
    Event& e = buf_[head_];
    e.seq = ++lastSeq_;
    e.tMs = tMs;
    e.sev = sev;
    std::strncpy(e.msg, msg ? msg : "", sizeof(e.msg) - 1);
    e.msg[sizeof(e.msg) - 1] = '\0';
    head_ = (head_ + 1) % CAP;
    if (count_ < CAP) ++count_;
}

size_t EventLog::since(uint32_t afterSeq, Event* out, size_t maxOut) const {
    size_t written = 0;
    // Walk valid entries oldest -> newest.
    for (size_t i = 0; i < count_ && written < maxOut; ++i) {
        size_t idx = (head_ + CAP - count_ + i) % CAP;
        if (buf_[idx].seq > afterSeq) out[written++] = buf_[idx];
    }
    return written;
}

} // namespace ctrl
