#include "show_runner.h"

namespace fw {

// Small buffer for due steps pulled per tick.
// BoxLink is 1-deep, so we pull up to 16 due steps and fire them one at a time
// as the link frees up — but in practice only one will be fired per tick call
// because after the first fire link.pendingAck() becomes true.
static const size_t DUE_BUF = 16;

ShowRunner::ShowRunner(BoxLink& link, uint32_t heartbeatPeriodMs)
    : link_(link)
    , hbPeriodMs_(heartbeatPeriodMs)
    , lastHbMs_(0)
    , nonceCtr_(0)
    , armed_(false)
    , hbStarted_(false)
{}

void ShowRunner::arm(uint32_t nowMs) {
    armed_     = true;
    hbStarted_ = false;          // reset so tick() emits HB immediately
    uint32_t nonce = ++nonceCtr_; // fresh, always > previous
    link_.arm(nonce, nowMs);
}

void ShowRunner::disarm(uint32_t nowMs) {
    link_.disarm(nowMs);
    armed_ = false;
    sched_.stop();
    hbStarted_ = false;
}

void ShowRunner::estop(uint32_t nowMs) {
    link_.estop(nowMs);
    armed_ = false;
    sched_.stop();
    hbStarted_ = false;
}

void ShowRunner::fireManual(uint8_t boxId, uint8_t channel, uint32_t nowMs) {
    if (!armed_ || link_.pendingAck()) return;
    link_.fire(boxId, channel, nowMs);
}

void ShowRunner::loadSequence(const SeqStep* steps, size_t count) {
    sched_.load(steps, count);
}

void ShowRunner::startSequence(uint32_t nowMs) {
    if (!armed_) return;
    sched_.start(nowMs);
}

void ShowRunner::stopSequence(uint32_t /*nowMs*/) {
    sched_.stop();
}

bool ShowRunner::sequenceRunning() const {
    return sched_.running();
}

void ShowRunner::tick(uint32_t nowMs) {
    // 1. Emit heartbeat if armed (first tick after arm, then every hbPeriodMs_).
    if (armed_) {
        if (!hbStarted_) {
            link_.heartbeat(nowMs);
            lastHbMs_  = nowMs;
            hbStarted_ = true;
        } else if (nowMs - lastHbMs_ >= hbPeriodMs_) {
            link_.heartbeat(nowMs);
            lastHbMs_ = nowMs;
        }
    }

    // 2. Fire due sequence steps — one at a time respecting 1-deep ACK.
    if (sched_.running()) {
        SeqStep buf[DUE_BUF];
        // Pull all steps that are due right now.
        // SequenceScheduler marks pulled steps as fired, so we won't re-pull them.
        // We fire only the first one that fits when the link is free;
        // the rest are already marked fired by the scheduler. To preserve them we
        // need to be careful: actually the scheduler marks them fired on pull —
        // so we must only pull when we can actually fire.
        //
        // Strategy: pull with capacity 1 when the link is free, so we fire exactly
        // one step per tick when possible.  The scheduler will return the next due
        // step on the following tick once the link clears.
        if (!link_.pendingAck()) {
            size_t n = sched_.due(nowMs, buf, 1);
            if (n > 0) {
                link_.fire(buf[0].boxId, buf[0].channel, nowMs);
            }
        }
    }

    // 3. Drive BoxLink retries.
    link_.tick(nowMs);
}

} // namespace fw
