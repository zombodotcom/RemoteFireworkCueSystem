#pragma once
#include <cstdint>
#include <cstddef>

namespace fw {

template <size_t N>
class RecentIds {
public:
    RecentIds() : head_(0), count_(0) {
        for (size_t i = 0; i < N; i++) buf_[i] = 0;
    }
    // true if id already present (duplicate); else record and return false.
    bool seenOrRecord(uint32_t id) {
        for (size_t i = 0; i < count_; i++) {
            if (buf_[i] == id) return true;
        }
        buf_[head_] = id;
        head_ = (head_ + 1) % N;
        if (count_ < N) count_++;
        return false;
    }
private:
    uint32_t buf_[N];
    size_t head_;
    size_t count_;
};

} // namespace fw
