#pragma once
#include "protocol.h"
#include "esp_err.h"
#include "esp_now.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <cstdint>

class EspNowLink {
public:
    esp_err_t begin(const uint8_t controllerMac[6]);   // creates the RX queue, inits wifi+esp_now
    bool receive(fw::CommandPacket& out);               // non-blocking dequeue; true if a packet was returned
    esp_err_t sendAck(const fw::AckPacket& ack);
private:
    static void rxTrampoline(const esp_now_recv_info_t* info, const uint8_t* data, int len);
    QueueHandle_t rxQueue_ = nullptr;
    uint8_t ctrlMac_[6];
};
