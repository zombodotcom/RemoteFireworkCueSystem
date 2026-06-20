#pragma once
#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

void rig_reset(void);
void rig_set_switch(int boxId, int on, uint32_t nowMs);
void rig_heartbeat(uint32_t nowMs);
int  rig_arm(uint32_t nonce, uint32_t nowMs);     // returns number of boxes that armed
void rig_disarm(uint32_t nowMs);
void rig_estop(uint32_t nowMs);
void rig_clear_estop(uint32_t nowMs);
int  rig_fire(int boxId, int channel, uint32_t nowMs);   // 1 if the cue energized a channel

// Sequence API (implemented in Task 3)
void rig_load_sequence(const uint32_t* triples, int count); // [timeMs,boxId,channel] * count
void rig_start_sequence(uint32_t nowMs);
void rig_stop_sequence(uint32_t nowMs);
int  rig_seq_running(void);

void rig_tick(uint32_t nowMs);

int  rig_box_state(int boxId);                 // 0 SAFE, 1 ARMED
int  rig_box_switch(int boxId);                // 0/1
int  rig_box_estopped(int boxId);              // 0/1
int  rig_box_can_fire(int boxId, uint32_t nowMs);
int  rig_channel_firing(int boxId, int channel);
int  rig_channel_msleft(int boxId, int channel, uint32_t nowMs);

#ifdef __cplusplus
}
#endif
