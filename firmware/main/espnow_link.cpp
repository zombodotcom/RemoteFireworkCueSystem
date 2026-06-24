#include "espnow_link.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_log.h"
#include <cstring>

static const char* TAG = "espnow";
static EspNowLink* g_self = nullptr;

esp_err_t EspNowLink::begin(const uint8_t controllerMac[6]) {
    g_self = this;
    memcpy(ctrlMac_, controllerMac, 6);

    // Create the RX queue before registering the recv callback.
    rxQueue_ = xQueueCreate(16, sizeof(fw::CommandPacket));
    if (!rxQueue_) { ESP_LOGE(TAG, "xQueueCreate failed"); return ESP_ERR_NO_MEM; }

    // WiFi stack must be initialised before esp_now_init().
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t wic = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wic));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    // Pin the ESP-NOW channel to match the controller's SoftAP channel. A STA
    // that never associates otherwise sits on the driver default; if box and
    // controller diverge, ESP-NOW silently delivers nothing. Must equal the
    // controller's WIFI_CHAN (controller_config.h, channel 1).
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));

    esp_err_t err = esp_now_init();
    if (err != ESP_OK) { ESP_LOGE(TAG, "esp_now_init failed: %d", err); return err; }

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, controllerMac, 6);
    peer.channel = 0;       // 0 = follow current WiFi channel
    peer.encrypt = false;
    err = esp_now_add_peer(&peer);
    if (err != ESP_OK) { ESP_LOGE(TAG, "add_peer failed: %d", err); return err; }

    err = esp_now_register_recv_cb(&EspNowLink::rxTrampoline);
    if (err != ESP_OK) { ESP_LOGE(TAG, "register_recv_cb failed: %d", err); }
    return err;
}

void EspNowLink::rxTrampoline(const esp_now_recv_info_t* /*info*/, const uint8_t* data, int len) {
    if (!g_self || !g_self->rxQueue_) return;
    if (len < (int)sizeof(fw::CommandPacket)) return;  // length guard; CRC checked in onCommand
    fw::CommandPacket pkt;
    memcpy(&pkt, data, sizeof(pkt));
    // esp_now recv cb runs in WiFi task context (not ISR) — use non-blocking xQueueSend.
    // Drop silently if queue is full; the controller's retry logic will recover.
    xQueueSend(g_self->rxQueue_, &pkt, 0);
}

bool EspNowLink::receive(fw::CommandPacket& out) {
    return rxQueue_ && xQueueReceive(rxQueue_, &out, 0) == pdTRUE;
}

esp_err_t EspNowLink::sendAck(const fw::AckPacket& ack) {
    return esp_now_send(ctrlMac_, (const uint8_t*)&ack, sizeof(ack));
}

esp_err_t EspNowLink::sendStatus(const fw::StatusPacket& status) {
    return esp_now_send(ctrlMac_, (const uint8_t*)&status, sizeof(status));
}
