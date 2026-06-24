#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "sequence.h"  // fw::SeqStep
#include "event_log.h"
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

struct Diag {
    volatile uint32_t uptimeMs  = 0;
    volatile uint32_t freeHeap  = 0;
    volatile uint32_t apClients = 0;
    volatile uint32_t fired     = 0;
    volatile uint32_t acked     = 0;
    volatile uint32_t failed    = 0;
    volatile uint32_t retries   = 0;
    volatile uint32_t lastAckMs = 0;
};

struct BoxTelemetry {
    volatile bool     linkAlive        = false;
    volatile int8_t   rssi             = 0;
    volatile uint8_t  state            = 0;      // 0 = SAFE, 1 = ARMED
    volatile uint16_t firedBitmap      = 0;
    volatile uint8_t  lastFiredChannel = 0xFF;
    volatile uint32_t lastHeardMs      = 0;
};

// Written by control loop each tick; read by status handler.
// volatile on individual fields — Xtensa single-instruction word reads are atomic.
struct StatusSnapshot {
    volatile bool    armed;
    volatile bool    seqRunning;
    volatile uint8_t lastFailedBox;   // 0xFF = none
    BoxTelemetry boxes[2];
    Diag             diag;
    volatile uint8_t faultCode;   // 0=none, 1=estop, 2=link, 3=fire-failed
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
extern ctrl::EventLog     g_events;
extern SemaphoreHandle_t  g_eventMtx;
