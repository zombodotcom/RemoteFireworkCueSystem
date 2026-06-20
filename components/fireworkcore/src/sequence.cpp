#include "sequence.h"

namespace fw {

SequenceScheduler::SequenceScheduler() : count_(0), startMs_(0), running_(false) {}

void SequenceScheduler::load(const SeqStep* steps, size_t count) {
    if (count > MAX_SEQ_STEPS) count = MAX_SEQ_STEPS;
    count_ = count;
    for (size_t i = 0; i < count_; i++) { steps_[i] = steps[i]; fired_[i] = false; }
    running_ = false;
}

void SequenceScheduler::start(uint32_t nowMs) {
    startMs_ = nowMs;
    running_ = (count_ > 0);
    for (size_t i = 0; i < count_; i++) fired_[i] = false;
}

void SequenceScheduler::stop() { running_ = false; }

size_t SequenceScheduler::due(uint32_t nowMs, SeqStep* out, size_t outCap) {
    if (!running_) return 0;
    uint32_t elapsed = nowMs - startMs_;
    size_t n = 0;
    size_t remaining = 0;
    for (size_t i = 0; i < count_; i++) {
        if (fired_[i]) continue;
        if (steps_[i].timeMs <= elapsed) {
            if (n < outCap) { out[n++] = steps_[i]; fired_[i] = true; }
            else remaining++;          // due but no room this call
        } else {
            remaining++;
        }
    }
    if (remaining == 0) running_ = false;
    return n;
}

} // namespace fw
