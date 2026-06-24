#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "wifi_sta.h"
#include "status_model.h"
#include "status_client.h"
#include "display.h"
#include "esp_lvgl_port.h"

extern "C" void app_main(void) {
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    wifi_sta_start();
    display_init();
    if (lvgl_port_lock(0)) {
        lv_obj_t* lbl = lv_label_create(lv_screen_active());
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_28, 0);
        lv_label_set_text(lbl, "CYD OK");
        lv_obj_center(lbl);
        lvgl_port_unlock();
    }
    static StatusModel model;
    while (true) {
        if (wifi_sta_connected()) status_client_poll_once(model);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
