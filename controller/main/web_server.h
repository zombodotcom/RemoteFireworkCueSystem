#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include "sequence.h"  // fw::SeqStep
#include <cstdint>
#include <cstddef>

enum class CmdType : uint8_t {
    ARM, DISARM, ESTOP, FIRE, RUN, STOP, HEARTBEAT
};

struct UiCommand {
    CmdType type;
    uint8_t boxId;
    uint8_t channel;
};

// Written by control loop each tick; read by status handler.
// volatile on individual fields — Xtensa single-instruction word reads are atomic.
struct StatusSnapshot {
    volatile bool    armed;
    volatile bool    seqRunning;
    volatile uint8_t lastFailedBox;   // 0xFF = none
};

class WebServer {
public:
    // Start the HTTP server. Both pointers must remain valid for the lifetime of the server.
    esp_err_t start(QueueHandle_t cmdQueue, StatusSnapshot* snap);
};

// Use constexpr to avoid ODR issue from static const in header.
constexpr size_t MAX_RUN_STEPS = 64;

// Shared state — written by web handlers (g_runSteps/g_runCount before RUN enqueue),
// read by control loop task only on RUN command.
extern fw::SeqStep    g_runSteps[MAX_RUN_STEPS];
extern size_t         g_runCount;
extern QueueHandle_t  g_cmdQueue;
extern StatusSnapshot g_status;
