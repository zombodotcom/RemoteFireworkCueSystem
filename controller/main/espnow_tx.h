#pragma once
#include "box_link.h"
#include "protocol.h"
#include "esp_err.h"
#include "esp_now.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <cstdint>

// EspNowTransport: implements fw::Transport over esp_now.
// Brings up the WiFi SoftAP then inits esp_now on top of it.
// ACK RX callback queues responseToId; caller drains with receiveAck().
class EspNowTransport : public fw::Transport {
public:
    esp_err_t begin();

    // fw::Transport interface — called from control loop task only.
    void send(uint8_t boxId, const fw::CommandPacket& pkt) override;

    // Non-blocking dequeue of one ACK id. Returns true if an id was returned.
    // Call from control loop only (same task as send/BoxLink).
    bool receiveAck(uint32_t& responseToId);

    struct StatusReport {
        uint8_t  boxId;
        uint8_t  state;
        uint8_t  lastFiredChannel;
        uint16_t firedBitmap;
        int8_t   rssi;
    };
    // Non-blocking dequeue of one box status report. Control loop only.
    bool receiveStatus(StatusReport& out);

private:
    // Static trampoline: C callback can't capture `this`.
    static EspNowTransport* g_self;
    static void rxCallback(const esp_now_recv_info_t* info, const uint8_t* data, int len);

    QueueHandle_t ackQueue_ = nullptr;
    QueueHandle_t statusQueue_ = nullptr;
};
