#include "espnow_tx.h"
#include "controller_config.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include <cstring>

static const char* TAG = "espnow_tx";

EspNowTransport* EspNowTransport::g_self = nullptr;

esp_err_t EspNowTransport::begin() {
    g_self = this;

    // Create the ACK queue before registering the RX callback.
    ackQueue_ = xQueueCreate(16, sizeof(uint32_t));
    if (!ackQueue_) { ESP_LOGE(TAG, "xQueueCreate failed"); return ESP_ERR_NO_MEM; }
    statusQueue_ = xQueueCreate(8, sizeof(StatusReport));
    if (!statusQueue_) { ESP_LOGE(TAG, "status xQueueCreate failed"); return ESP_ERR_NO_MEM; }

    // --- WiFi SoftAP init (must precede esp_now_init) ---
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t wic = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wic));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    wifi_config_t ap_cfg = {};
    // ssid/password are char arrays inside a union — assign field by field.
    strncpy(reinterpret_cast<char*>(ap_cfg.ap.ssid),
            ctrl::AP_SSID, sizeof(ap_cfg.ap.ssid) - 1);
    ap_cfg.ap.ssid_len = static_cast<uint8_t>(strlen(ctrl::AP_SSID));
    strncpy(reinterpret_cast<char*>(ap_cfg.ap.password),
            ctrl::AP_PASS, sizeof(ap_cfg.ap.password) - 1);
    ap_cfg.ap.channel        = ctrl::WIFI_CHAN;
    ap_cfg.ap.authmode       = WIFI_AUTH_WPA2_PSK;
    ap_cfg.ap.max_connection = 4;
    ap_cfg.ap.pmf_cfg.required = true;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    // --- ESP-NOW init ---
    esp_err_t err = esp_now_init();
    if (err != ESP_OK) { ESP_LOGE(TAG, "esp_now_init: %s", esp_err_to_name(err)); return err; }

    // Add both box MACs as peers (channel 0 = follow current WiFi channel).
    for (int i = 0; i < 2; i++) {
        // Skip an unconfigured (all-zero) box MAC so a single-box deployment
        // still boots; esp_now_send to it later fails gracefully (ESP_ERR_ESPNOW_NOT_FOUND).
        bool isNull = true;
        for (int b = 0; b < 6; b++) if (ctrl::BOX_MAC[i][b]) { isNull = false; break; }
        if (isNull) { ESP_LOGW(TAG, "BOX_MAC[%d] is null — skipping peer (not configured)", i); continue; }
        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, ctrl::BOX_MAC[i], 6);
        peer.channel = ctrl::WIFI_CHAN;   // match the SoftAP channel
        peer.ifidx   = WIFI_IF_AP;        // controller is AP-only — send via the AP interface, not STA (which is down)
        peer.encrypt = false;
        err = esp_now_add_peer(&peer);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "add_peer %d: %s", i, esp_err_to_name(err));
            return err;
        }
    }

    // Broadcast peer for display frames (CYD listens; no association needed).
    esp_now_peer_info_t bpeer = {};
    memset(bpeer.peer_addr, 0xFF, 6);
    bpeer.channel = ctrl::WIFI_CHAN;
    bpeer.ifidx   = WIFI_IF_AP;
    bpeer.encrypt = false;
    err = esp_now_add_peer(&bpeer);
    if (err != ESP_OK) { ESP_LOGE(TAG, "add broadcast peer: %s", esp_err_to_name(err)); return err; }

    err = esp_now_register_recv_cb(&EspNowTransport::rxCallback);
    if (err != ESP_OK) ESP_LOGE(TAG, "register_recv_cb: %s", esp_err_to_name(err));
    return err;
}

// fw::Transport::send — called from control loop task only.
void EspNowTransport::send(uint8_t boxId, const fw::CommandPacket& pkt) {
    if (boxId >= 2) { ESP_LOGE(TAG, "send: invalid boxId %u", boxId); return; }
    esp_err_t err = esp_now_send(ctrl::BOX_MAC[boxId],
                                 reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
    if (err != ESP_OK) ESP_LOGE(TAG, "esp_now_send: %s", esp_err_to_name(err));
}

bool EspNowTransport::receiveAck(uint32_t& responseToId) {
    if (!ackQueue_) return false;
    return xQueueReceive(ackQueue_, &responseToId, 0) == pdTRUE;
}

bool EspNowTransport::receiveStatus(StatusReport& out) {
    if (!statusQueue_) return false;
    return xQueueReceive(statusQueue_, &out, 0) == pdTRUE;
}

static const uint8_t kBroadcast[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
void EspNowTransport::sendDisplayStatus(const fw::DisplayStatusPacket& p) {
    esp_now_send(kBroadcast, reinterpret_cast<const uint8_t*>(&p), sizeof(p));
}
void EspNowTransport::sendDisplayEvent(const fw::DisplayEventPacket& e) {
    esp_now_send(kBroadcast, reinterpret_cast<const uint8_t*>(&e), sizeof(e));
}

// Static RX callback — runs in WiFi task, not the control-loop task.
// Must NOT touch BoxLink. Validate + memcpy + queue only.
void EspNowTransport::rxCallback(const esp_now_recv_info_t* info,
                                  const uint8_t* data, int len) {
    if (!g_self || len < 1) return;
    uint8_t type = data[0];

    if (type == static_cast<uint8_t>(fw::MsgType::ACK)) {
        if (!g_self->ackQueue_) return;
        if (len < static_cast<int>(sizeof(fw::AckPacket))) return;
        fw::AckPacket ack;
        memcpy(&ack, data, sizeof(ack));
        if (!fw::crcValid(ack)) { ESP_LOGW(TAG, "ACK CRC mismatch - dropped"); return; }
        xQueueSend(g_self->ackQueue_, &ack.responseToId, 0);
        return;
    }

    if (type == static_cast<uint8_t>(fw::MsgType::STATUS)) {
        if (!g_self->statusQueue_) return;
        if (len < static_cast<int>(sizeof(fw::StatusPacket))) return;
        fw::StatusPacket st;
        memcpy(&st, data, sizeof(st));
        if (!fw::crcValid(st)) { ESP_LOGW(TAG, "STATUS CRC mismatch - dropped"); return; }
        StatusReport r{};
        r.boxId            = st.boxId;
        r.state            = st.state;
        r.lastFiredChannel = st.lastFiredChannel;
        r.firedBitmap      = st.firedBitmap;
        r.rssi             = (info && info->rx_ctrl) ? (int8_t)info->rx_ctrl->rssi : 0;
        xQueueSend(g_self->statusQueue_, &r, 0);
        return;
    }
}
