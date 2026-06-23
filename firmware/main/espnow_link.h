#pragma once
#include "protocol.h"
#include "esp_err.h"
#include "esp_now.h"
#include <functional>
#include <cstdint>

class EspNowLink {
public:
    esp_err_t begin(const uint8_t controllerMac[6]);
    void setOnCommand(std::function<void(const fw::CommandPacket&)> cb) { cb_ = cb; }
    esp_err_t sendAck(const fw::AckPacket& ack);
    uint32_t lastRxMs() const { return lastRxMs_; }
private:
    static void rxTrampoline(const esp_now_recv_info_t* info, const uint8_t* data, int len);
    std::function<void(const fw::CommandPacket&)> cb_;
    uint8_t ctrlMac_[6];
    uint32_t lastRxMs_ = 0;
};
