#include "espnow_link.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <cstring>

static const char* TAG = "espnow";
static EspNowLink* g_self = nullptr;

esp_err_t EspNowLink::begin(const uint8_t controllerMac[6]) {
    g_self = this;
    memcpy(ctrlMac_, controllerMac, 6);

    // WiFi stack must be initialised before esp_now_init().
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t wic = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wic));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

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
    if (!g_self || !g_self->cb_) return;
    if (len < (int)sizeof(fw::CommandPacket)) return;  // length guard; CRC checked in onCommand
    g_self->lastRxMs_ = (uint32_t)(esp_timer_get_time() / 1000);
    fw::CommandPacket pkt;
    memcpy(&pkt, data, sizeof(pkt));
    g_self->cb_(pkt);
}

esp_err_t EspNowLink::sendAck(const fw::AckPacket& ack) {
    return esp_now_send(ctrlMac_, (const uint8_t*)&ack, sizeof(ack));
}
