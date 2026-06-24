#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "wifi_sta.h"

static const char* TAG = "cyd";

extern "C" void app_main(void) {
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    wifi_sta_start();
    while (true) {
        ESP_LOGI(TAG, "wifi connected: %s", wifi_sta_connected() ? "yes" : "no");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
