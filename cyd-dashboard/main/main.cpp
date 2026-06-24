#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "display.h"
#include "touch.h"
#include "dashboard.h"
#include "espnow_rx.h"
#include "status_model.h"

extern "C" void app_main(void) {
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    lv_display_t* disp = display_init();
    touch_init(disp);
    dashboard_create();
    espnow_rx_start();

    static StatusModel model;
    static LogEv evs[16];
    while (true) {
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
        espnow_rx_snapshot(&model, now);
        int n = espnow_rx_events(evs, 16);
        if (n > 0) dashboard_set_events(evs, n);
        dashboard_update(model);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
