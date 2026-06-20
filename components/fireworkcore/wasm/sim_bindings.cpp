#include "sim_bindings.h"
#include "arming.h"
#include "recent_ids.h"
#include "protocol.h"
#include "sequence.h"

using namespace fw;

namespace {

static const uint32_t FIRE_MS = 400;

struct Channel { bool firing = false; uint32_t offAtMs = 0; };

struct Box {
    ArmingStateMachine arm;
    RecentIds<32> seen;
    Channel ch[MAX_CHANNELS];
    bool switchOn = false;   // mirror (core exposes no getter)
    bool estopped = false;   // mirror
};

struct Rig {
    Box boxes[2];
    SequenceScheduler scheduler;
    bool seqRunning = false;
    uint32_t nextMsgId = 1;
};

Rig g;

bool validBox(int b) { return b == 0 || b == 1; }

void syncSequenceFlag() {
    bool running = g.scheduler.running();
    g.seqRunning = running;
    for (int i = 0; i < 2; i++) g.boxes[i].arm.setSequenceRunning(running);
}

void energize(Box& b, int channel, uint32_t now) {
    b.ch[channel].firing = true;
    b.ch[channel].offAtMs = now + FIRE_MS;
}

} // namespace

extern "C" {

void rig_reset(void) { g = Rig(); }

void rig_set_switch(int boxId, int on, uint32_t nowMs) {
    if (!validBox(boxId)) return;
    g.boxes[boxId].switchOn = (on != 0);
    g.boxes[boxId].arm.setPhysicalSwitch(on != 0, nowMs);
}

void rig_heartbeat(uint32_t nowMs) {
    for (int i = 0; i < 2; i++) g.boxes[i].arm.heartbeat(nowMs);
}

int rig_arm(uint32_t nonce, uint32_t nowMs) {
    int n = 0;
    for (int i = 0; i < 2; i++) if (g.boxes[i].arm.arm(nonce, nowMs)) n++;
    return n;
}

void rig_disarm(uint32_t nowMs) {
    for (int i = 0; i < 2; i++) g.boxes[i].arm.disarm(nowMs);
}

void rig_estop(uint32_t nowMs) {
    g.scheduler.stop();
    g.seqRunning = false;
    for (int i = 0; i < 2; i++) { g.boxes[i].estopped = true; g.boxes[i].arm.estop(nowMs); }
}

void rig_clear_estop(uint32_t nowMs) {
    for (int i = 0; i < 2; i++) { g.boxes[i].estopped = false; g.boxes[i].arm.clearEstop(nowMs); }
}

int rig_fire(int boxId, int channel, uint32_t nowMs) {
    syncSequenceFlag();
    if (!validBox(boxId)) return 0;
    if (!channelInRange((uint8_t)channel)) return 0;
    Box& b = g.boxes[boxId];
    if (!b.arm.canFire(nowMs)) return 0;
    uint32_t id = g.nextMsgId++;
    // Structural mirror of the firmware's retransmit dedup. The sim issues a unique
    // id per fire, so this never rejects here; it exists so the rig exercises the
    // same RecentIds API the firmware uses.
    if (b.seen.seenOrRecord(id)) return 0;
    energize(b, channel, nowMs);
    return 1;
}

void rig_tick(uint32_t nowMs) {
    g.seqRunning = g.scheduler.running();
    for (int i = 0; i < 2; i++) {
        g.boxes[i].arm.setSequenceRunning(g.seqRunning);   // firmware contract
        g.boxes[i].arm.update(nowMs);
    }
    if (g.scheduler.running()) {
        SeqStep due[MAX_SEQ_STEPS];
        size_t n = g.scheduler.due(nowMs, due, MAX_SEQ_STEPS);
        for (size_t k = 0; k < n; k++) {
            int boxId = due[k].boxId;
            if (!validBox(boxId)) continue;
            Box& b = g.boxes[boxId];
            if (b.arm.canFire(nowMs) && channelInRange(due[k].channel))
                energize(b, due[k].channel, nowMs);
        }
        g.seqRunning = g.scheduler.running();
    }
    for (int i = 0; i < 2; i++)
        for (int c = 0; c < MAX_CHANNELS; c++)
            if (g.boxes[i].ch[c].firing && nowMs >= g.boxes[i].ch[c].offAtMs)
                g.boxes[i].ch[c].firing = false;
}

// Sequence API — bodies completed in Task 3 (declared here so the unit links).
int rig_load_sequence(const uint32_t* triples, int count) {
    SeqStep steps[MAX_SEQ_STEPS];
    if (count < 0) count = 0;
    if ((size_t)count > MAX_SEQ_STEPS) count = (int)MAX_SEQ_STEPS;
    for (int i = 0; i < count; i++) {
        steps[i].timeMs  = triples[i * 3 + 0];
        steps[i].boxId   = (uint8_t)triples[i * 3 + 1];
        steps[i].channel = (uint8_t)triples[i * 3 + 2];
    }
    g.scheduler.load(steps, (size_t)count);
    return count;
}
void rig_start_sequence(uint32_t nowMs) { g.scheduler.start(nowMs); g.seqRunning = g.scheduler.running(); }
void rig_stop_sequence(uint32_t nowMs)  { (void)nowMs; g.scheduler.stop(); g.seqRunning = false; }
int  rig_seq_running(void) { return g.seqRunning ? 1 : 0; }

int rig_box_state(int boxId)    { return validBox(boxId) ? (int)g.boxes[boxId].arm.state() : 0; }
int rig_box_switch(int boxId)   { return validBox(boxId) && g.boxes[boxId].switchOn ? 1 : 0; }
int rig_box_estopped(int boxId) { return validBox(boxId) && g.boxes[boxId].estopped ? 1 : 0; }
int rig_box_can_fire(int boxId, uint32_t nowMs) { syncSequenceFlag(); return validBox(boxId) && g.boxes[boxId].arm.canFire(nowMs) ? 1 : 0; }
int rig_channel_firing(int boxId, int channel) {
    if (!validBox(boxId) || !channelInRange((uint8_t)channel)) return 0;
    return g.boxes[boxId].ch[channel].firing ? 1 : 0;
}
int rig_channel_msleft(int boxId, int channel, uint32_t nowMs) {
    if (!validBox(boxId) || !channelInRange((uint8_t)channel)) return 0;
    Channel& c = g.boxes[boxId].ch[channel];
    if (!c.firing || nowMs >= c.offAtMs) return 0;
    return (int)(c.offAtMs - nowMs);
}

} // extern "C"
