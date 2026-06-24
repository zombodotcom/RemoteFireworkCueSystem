#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
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
#include "web_server.h"

static const char* TAG = "controller";

// Definitions of the shared globals declared (extern) in web_server.h.
fw::SeqStep    g_runSteps[MAX_RUN_STEPS];
size_t         g_runCount  = 0;
QueueHandle_t  g_cmdQueue  = nullptr;
StatusSnapshot g_status    = {};

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
    static fw::BoxLink    link(tx, lk_cfg);
    static fw::ShowRunner runner(link, ctrl::HEARTBEAT_MS);

    // Create the command queue (depth 8, element = UiCommand).
    g_cmdQueue = xQueueCreate(8, sizeof(UiCommand));
    configASSERT(g_cmdQueue);

    // Start the web server — handlers reach runner via g_cmdQueue / g_status.
    static WebServer web;
    ESP_ERROR_CHECK(web.start(g_cmdQueue, &g_status));

    // Control loop: drain cmd queue → drive runner, drain ACKs, tick runner at ~20 ms.
    static uint32_t lastStatusMs[2] = {0, 0};
    while (true) {
        uint32_t now = static_cast<uint32_t>(esp_timer_get_time() / 1000);

        // --- Drain the command queue (web handlers post here; only this task pops) ---
        UiCommand cmd;
        while (xQueueReceive(g_cmdQueue, &cmd, 0) == pdTRUE) {
            switch (cmd.type) {
                case CmdType::ARM:
                    runner.arm(now);
                    break;
                case CmdType::DISARM:
                    runner.disarm(now);
                    break;
                case CmdType::ESTOP:
                    runner.estop(now);
                    break;
                case CmdType::STOP:
                    runner.stopSequence(now);
                    break;
                case CmdType::HEARTBEAT:
                    // Heartbeat is implicit in runner.tick(); no explicit call needed.
                    break;
                case CmdType::FIRE:
                    runner.fireManual(cmd.boxId, cmd.channel, now);
                    break;
                case CmdType::RUN:
                    // g_runSteps/g_runCount were written by the handler before enqueue.
                    runner.loadSequence(g_runSteps, g_runCount);
                    runner.startSequence(now);
                    break;
            }
        }

        // Drain ACK queue (WiFi task posts here; we consume from control loop only).
        uint32_t ack_id = 0;
        while (tx.receiveAck(ack_id)) {
            link.onAck(ack_id, now);
        }

        // Drain box telemetry (WiFi task queued it; consume in control loop only).
        EspNowTransport::StatusReport sr;
        while (tx.receiveStatus(sr)) {
            if (sr.boxId < 2) {
                g_status.boxes[sr.boxId].state            = sr.state;
                g_status.boxes[sr.boxId].firedBitmap      = sr.firedBitmap;
                g_status.boxes[sr.boxId].lastFiredChannel = sr.lastFiredChannel;
                g_status.boxes[sr.boxId].rssi             = sr.rssi;
                lastStatusMs[sr.boxId] = now;
            }
        }
        for (int b = 0; b < 2; b++) {
            g_status.boxes[b].linkAlive =
                (lastStatusMs[b] != 0) && ((now - lastStatusMs[b]) < 1500);
        }

        runner.tick(now);

        // Update status snapshot — written here (control loop), read by status handler.
        g_status.armed         = runner.armed();
        g_status.seqRunning    = runner.sequenceRunning();
        g_status.lastFailedBox = 0xFF;  // Not yet exposed by ShowRunner API; 0xFF = none.

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
