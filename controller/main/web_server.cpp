#include "web_server.h"
#include "esp_http_server.h"
#include "esp_log.h"

static const char* TAG = "web_server";

esp_err_t WebServer::start(QueueHandle_t /*cmdQueue*/, StatusSnapshot* /*snap*/) {
    ESP_LOGI(TAG, "web_server stub — not yet wired");
    return ESP_OK;
}
