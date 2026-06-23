#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "controller_config.h"
#include "config_store.h"
#include "espnow_tx.h"
#include "box_link.h"
#include "show_runner.h"

static const char* TAG = "controller";

extern "C" void app_main(void) {
    // NVS init (required by WiFi and ConfigStore).
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_err);

    static ConfigStore cfg;
    cfg.begin();

    // Bring up SoftAP + ESP-NOW transport.
    static EspNowTransport tx;
    ESP_ERROR_CHECK(tx.begin());

    // Log the AP MAC so boxes can be configured to pair.
    uint8_t ap_mac[6] = {};
    esp_wifi_get_mac(WIFI_IF_AP, ap_mac);
    ESP_LOGI(TAG, "controller booted: AP \"%s\" up, MAC " MACSTR,
             ctrl::AP_SSID, MAC2STR(ap_mac));

    // Construct BoxLink + ShowRunner using the host-tested portable logic.
    fw::BoxLinkConfig lk_cfg;
    lk_cfg.ackTimeoutMs = ctrl::ACK_TIMEOUT_MS;
    lk_cfg.maxRetries   = ctrl::MAX_RETRIES;
    static fw::BoxLink   link(tx, lk_cfg);
    static fw::ShowRunner runner(link, ctrl::HEARTBEAT_MS);

    // Control loop: drain ACKs, then tick ShowRunner at 20 ms.
    // No arming here — Task 4 adds the web API that drives arm/disarm/fire.
    // The runner stays disarmed; ShowRunner.tick() is a no-op when disarmed.
    while (true) {
        uint32_t now = static_cast<uint32_t>(esp_timer_get_time() / 1000);

        // Drain ACK queue (WiFi task posts here; we consume from control loop only).
        uint32_t ack_id = 0;
        while (tx.receiveAck(ack_id)) {
            link.onAck(ack_id, now);
        }

        runner.tick(now);

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
