#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"
#include "controller_config.h"
#include "config_store.h"
#include "espnow_tx.h"
#include "box_link.h"
#include "show_runner.h"
#include "web_server.h"
#include <cstdarg>
#include <cstring>

static const char* TAG = "controller";

// Definitions of the shared globals declared (extern) in web_server.h.
fw::SeqStep    g_runSteps[MAX_RUN_STEPS];
size_t         g_runCount  = 0;
QueueHandle_t  g_cmdQueue  = nullptr;
StatusSnapshot g_status    = {};
ctrl::EventLog    g_events;
SemaphoreHandle_t g_eventMtx = nullptr;

// Best-effort: format + push under the event mutex. Called from control loop only.
static void log_evt(uint8_t sev, const char* fmt, ...) {
    char m[48];
    va_list ap; va_start(ap, fmt); vsnprintf(m, sizeof(m), fmt, ap); va_end(ap);
    uint32_t t = (uint32_t)(esp_timer_get_time() / 1000);
    if (g_eventMtx && xSemaphoreTake(g_eventMtx, pdMS_TO_TICKS(5)) == pdTRUE) {
        g_events.push(sev, m, t);
        xSemaphoreGive(g_eventMtx);
    }
}

// Sticky fault flags maintained by the loop/observer.
static bool g_fireFailed = false;
static bool g_estop      = false;

// Passive BoxLink observer: logs fire lifecycle + bumps diag counters.
struct CtrlObserver : public fw::BoxLinkObserver {
    void onFireSent(uint8_t b, uint8_t c, uint32_t id) override {
        g_status.diag.fired = g_status.diag.fired + 1;
        log_evt(ctrl::SEV_INFO, "FIRE ch%u -> box%u (id%lu)", c, b, (unsigned long)id);
    }
    void onFireAck(uint8_t /*b*/, uint8_t /*c*/, uint32_t id, uint32_t lat) override {
        g_status.diag.acked = g_status.diag.acked + 1;
        g_status.diag.lastAckMs = lat; g_fireFailed = false;
        log_evt(ctrl::SEV_INFO, "ACK id%lu (%lums)", (unsigned long)id, (unsigned long)lat);
    }
    void onFireRetry(uint8_t /*b*/, uint8_t c, uint32_t /*id*/, uint8_t a, uint8_t m) override {
        g_status.diag.retries = g_status.diag.retries + 1;
        log_evt(ctrl::SEV_WARN, "RETRY ch%u %u/%u", c, a, m);
    }
    void onFireFailed(uint8_t /*b*/, uint8_t c, uint32_t /*id*/) override {
        g_status.diag.failed = g_status.diag.failed + 1; g_fireFailed = true;
        log_evt(ctrl::SEV_ERR, "FIRE FAILED ch%u (no ACK)", c);
    }
};

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

    g_eventMtx = xSemaphoreCreateMutex();
    configASSERT(g_eventMtx);
    static CtrlObserver obs;
    link.setObserver(&obs);
    log_evt(ctrl::SEV_INFO, "controller up, AP %s", ctrl::AP_SSID);

    // Start the web server — handlers reach runner via g_cmdQueue / g_status.
    static WebServer web;
    ESP_ERROR_CHECK(web.start(g_cmdQueue, &g_status));

    // Control loop: drain cmd queue → drive runner, drain ACKs, tick runner at ~20 ms.
    static uint32_t lastStatusMs[2] = {0, 0};
    static bool prevSeq = false;
    static bool prevLink[2] = {false, false};
    static uint32_t lastDispMs = 0;
    static uint32_t lastDispSeq = 0;
    while (true) {
        uint32_t now = static_cast<uint32_t>(esp_timer_get_time() / 1000);

        // --- Drain the command queue (web handlers post here; only this task pops) ---
        UiCommand cmd;
        while (xQueueReceive(g_cmdQueue, &cmd, 0) == pdTRUE) {
            switch (cmd.type) {
                case CmdType::ARM:
                    runner.arm(now); g_estop = false; log_evt(ctrl::SEV_WARN, "ARM -> box0"); break;
                case CmdType::DISARM:
                    runner.disarm(now); log_evt(ctrl::SEV_INFO, "DISARM"); break;
                case CmdType::ESTOP:
                    runner.estop(now); g_estop = true; log_evt(ctrl::SEV_ERR, "ESTOP"); break;
                case CmdType::STOP:
                    runner.stopSequence(now); log_evt(ctrl::SEV_INFO, "SEQ stop"); break;
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
                    log_evt(ctrl::SEV_INFO, "SEQ start (%u cues)", (unsigned)g_runCount);
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
            bool alive = (lastStatusMs[b] != 0) && ((now - lastStatusMs[b]) < 1500);
            if (alive != prevLink[b]) {
                log_evt(alive ? ctrl::SEV_INFO : ctrl::SEV_ERR,
                        "box%d link %s", b, alive ? "up" : "lost");
                prevLink[b] = alive;
            }
            g_status.boxes[b].linkAlive  = alive;
            g_status.boxes[b].lastHeardMs = (lastStatusMs[b] != 0) ? (now - lastStatusMs[b]) : 0xFFFFFFFF;
        }

        runner.tick(now);

        bool seqNow = runner.sequenceRunning();
        if (prevSeq && !seqNow) log_evt(ctrl::SEV_INFO, "SEQ done");
        prevSeq = seqNow;

        // Update status snapshot — written here (control loop), read by status handler.
        g_status.armed         = runner.armed();
        g_status.seqRunning    = seqNow;
        g_status.lastFailedBox = 0xFF;  // Not yet exposed by ShowRunner API; 0xFF = none.

        // Diagnostics.
        g_status.diag.uptimeMs = now;
        g_status.diag.freeHeap = (uint32_t)esp_get_free_heap_size();
        wifi_sta_list_t stalist;
        g_status.diag.apClients = (esp_wifi_ap_get_sta_list(&stalist) == ESP_OK) ? (uint32_t)stalist.num : 0;

        // Fault code: estop > fire-failed > any configured box link down.
        bool anyLinkDown = false;
        for (int b = 0; b < 2; b++) {
            bool configured = false;
            for (int i = 0; i < 6; i++) if (ctrl::BOX_MAC[b][i]) { configured = true; break; }
            if (configured && !g_status.boxes[b].linkAlive) anyLinkDown = true;
        }
        g_status.faultCode = g_estop ? 1 : (g_fireFailed ? 3 : (anyLinkDown ? 2 : 0));

        // --- Broadcast display status (~1 Hz) for ESP-NOW display panels (CYD) ---
        if (now - lastDispMs >= 1000) {
            lastDispMs = now;
            fw::DisplayStatusPacket dp{};
            dp.type           = (uint8_t)fw::MsgType::DISP_STATUS;
            dp.boxState       = g_status.boxes[0].state;
            dp.boxLinkAlive   = g_status.boxes[0].linkAlive ? 1 : 0;
            dp.rssi           = g_status.boxes[0].rssi;
            dp.firedBitmap    = g_status.boxes[0].firedBitmap;
            dp.lastFired      = (g_status.boxes[0].lastFiredChannel == 0xFF) ? -1 : (int8_t)g_status.boxes[0].lastFiredChannel;
            dp.seqRunning     = g_status.seqRunning ? 1 : 0;
            dp.faultCode      = g_status.faultCode;
            dp.uptimeMs       = g_status.diag.uptimeMs;
            dp.freeHeap       = g_status.diag.freeHeap;
            dp.apClients      = g_status.diag.apClients;
            dp.fired          = g_status.diag.fired;
            dp.acked          = g_status.diag.acked;
            dp.failed         = g_status.diag.failed;
            dp.retries        = g_status.diag.retries;
            dp.lastAckMs      = g_status.diag.lastAckMs;
            dp.boxLastHeardMs = g_status.boxes[0].lastHeardMs;
            tx.sendDisplayStatus(dp);
        }
        // --- Broadcast any new events (bounded per tick) ---
        if (g_eventMtx && xSemaphoreTake(g_eventMtx, pdMS_TO_TICKS(5)) == pdTRUE) {
            ctrl::Event evs[4];
            size_t en = g_events.since(lastDispSeq, evs, 4);
            xSemaphoreGive(g_eventMtx);
            for (size_t i = 0; i < en; ++i) {
                fw::DisplayEventPacket ep{};
                ep.type = (uint8_t)fw::MsgType::DISP_EVENT;
                ep.sev  = evs[i].sev;
                ep.seq  = evs[i].seq;
                std::strncpy(ep.msg, evs[i].msg, sizeof(ep.msg) - 1);
                tx.sendDisplayEvent(ep);
                lastDispSeq = evs[i].seq;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
