#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "box_controller.h"
#include "stub_channel_driver.h"

static const char* TAG = "firing_box";

extern "C" void app_main(void) {
    static StubChannelDriver driver;
    fw::BoxConfig cfg;            // boxId 0, fireMs 400, default arming
    static fw::BoxController box(driver, cfg);
    box.begin();                  // boot-safe: outputs off, SAFE

    ESP_LOGI(TAG, "firing box booted: SAFE, outputs off");

    // Skeleton loop. Plan 3 adds: arm-switch GPIO -> setPhysicalSwitch,
    // esp_now RX -> box.onCommand, led_strip status, and the real ChannelDriver.
    while (true) {
        uint32_t nowMs = (uint32_t)(esp_timer_get_time() / 1000);
        box.tick(nowMs);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
