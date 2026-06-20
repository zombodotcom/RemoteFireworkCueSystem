#pragma once
#include <cstdint>
#include <cstddef>

namespace fw {

static const size_t MAX_SEQ_STEPS = 256;

struct SeqStep {
    uint32_t timeMs;
    uint8_t  boxId;
    uint8_t  channel;
};

class SequenceScheduler {
public:
    SequenceScheduler();
    void load(const SeqStep* steps, size_t count);
    void start(uint32_t nowMs);
    void stop();
    bool running() const { return running_; }
    size_t due(uint32_t nowMs, SeqStep* out, size_t outCap);

private:
    SeqStep steps_[MAX_SEQ_STEPS];
    bool    fired_[MAX_SEQ_STEPS];
    size_t  count_;
    uint32_t startMs_;
    bool     running_;
};

} // namespace fw
