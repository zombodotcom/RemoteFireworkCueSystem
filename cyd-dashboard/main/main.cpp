#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "wifi_sta.h"
#include "display.h"
#include "touch.h"
#include "dashboard.h"
#include "status_client.h"
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
    wifi_sta_start();

    static StatusModel model;
    static LogEv evs[16];
    static uint32_t sinceSeq = 0;
    dashboard_update(model);

    while (true) {
        if (wifi_sta_connected()) {
            status_client_poll_once(model);
            int n = status_client_poll_events(sinceSeq, evs, 16);
            if (n > 0) dashboard_set_events(evs, n);
        } else {
            model.controllerReachable = false;
        }
        dashboard_update(model);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
