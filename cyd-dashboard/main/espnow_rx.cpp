#include "espnow_rx.h"
#include "display_decode.h"
#include "protocol.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <cstring>

static const char* TAG = "espnow_rx";

// Only accept display frames from the controller's SoftAP MAC. This is a SAFETY
// display (ARMED/SAFE) — without a source check, any ESP-NOW device on channel 1
// could broadcast a forged "SAFE" frame and make the panel lie about a hot box.
// MAC-filtering stops interference + casual spoofing; a determined attacker who
// spoofs this MAC is out of scope here (would need a pre-shared HMAC/LMK — see
// NEXT-STEPS). Must match controller SoftAP MAC (firmware board_config CONTROLLER_MAC).
static const uint8_t kControllerMac[6] = {0xb0,0xcb,0xd8,0x89,0x9e,0x69};

static SemaphoreHandle_t s_mtx;
static StatusModel       s_model;        // latest decoded status
static uint32_t          s_lastStatusMs; // ms of last DISP_STATUS
static LogEv             s_ring[16];
static int               s_count;
static int               s_maxSeq;

static void recv_cb(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
    if (len < 1) return;
    // Drop any frame not from the controller (anti-spoof for the safety display).
    if (!info || std::memcmp(info->src_addr, kControllerMac, 6) != 0) return;
    uint8_t type = data[0];
    if (type == (uint8_t)fw::MsgType::DISP_STATUS && len == (int)sizeof(fw::DisplayStatusPacket)) {
        fw::DisplayStatusPacket p; std::memcpy(&p, data, sizeof(p));
        if (xSemaphoreTake(s_mtx, pdMS_TO_TICKS(5)) == pdTRUE) {
            applyDisplayStatus(p, s_model);
            s_lastStatusMs = (uint32_t)(esp_timer_get_time() / 1000);
            xSemaphoreGive(s_mtx);
        }
    } else if (type == (uint8_t)fw::MsgType::DISP_EVENT && len == (int)sizeof(fw::DisplayEventPacket)) {
        fw::DisplayEventPacket p; std::memcpy(&p, data, sizeof(p));
        LogEv e; toLogEv(p, e);
        if (xSemaphoreTake(s_mtx, pdMS_TO_TICKS(5)) == pdTRUE) {
            if (e.seq > s_maxSeq) {                  // new event -> append (drop oldest if full)
                if (s_count == 16) { std::memmove(&s_ring[0], &s_ring[1], 15 * sizeof(LogEv)); s_count = 15; }
                s_ring[s_count++] = e;
                s_maxSeq = e.seq;
            }
            xSemaphoreGive(s_mtx);
        }
    }
}

void espnow_rx_start(void) {
    s_mtx = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    // Do NOT associate. Pin the controller's channel and listen via ESP-NOW.
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(&recv_cb));
    ESP_LOGI(TAG, "ESP-NOW display RX up on channel 1 (no AP association)");
}

void espnow_rx_snapshot(StatusModel* out, unsigned long nowMs) {
    if (xSemaphoreTake(s_mtx, portMAX_DELAY) == pdTRUE) {
        *out = s_model;
        out->controllerReachable = (s_lastStatusMs != 0) && ((uint32_t)nowMs - s_lastStatusMs < 3000);
        xSemaphoreGive(s_mtx);
    }
}

int espnow_rx_events(LogEv* out, int maxOut) {
    int n = 0;
    if (xSemaphoreTake(s_mtx, portMAX_DELAY) == pdTRUE) {
        for (int i = 0; i < s_count && n < maxOut; ++i) out[n++] = s_ring[i];
        xSemaphoreGive(s_mtx);
    }
    return n;
}
