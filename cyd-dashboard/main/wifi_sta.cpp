#include "wifi_sta.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include <cstring>
#if __has_include("secrets.h")
#include "secrets.h"
#else
#include "secrets.example.h"
#endif

static const char* TAG = "wifi_sta";
static volatile bool s_connected = false;

static void on_wifi(void*, esp_event_base_t base, int32_t id, void*) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        s_connected = false;
        ESP_LOGW(TAG, "disconnected — retrying");
        esp_wifi_connect();
    }
}
static void on_ip(void*, esp_event_base_t, int32_t, void* data) {
    s_connected = true;
    ESP_LOGI(TAG, "got IP — connected to %s", FW_AP_SSID);
}

void wifi_sta_start(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &on_wifi, nullptr, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &on_ip, nullptr, nullptr));

    wifi_config_t wc = {};
    std::strncpy((char*)wc.sta.ssid, FW_AP_SSID, sizeof(wc.sta.ssid) - 1);
    std::strncpy((char*)wc.sta.password, FW_AP_PASS, sizeof(wc.sta.password) - 1);
    wc.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wc.sta.pmf_cfg.capable = true;     // controller AP requires PMF
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());
}

bool wifi_sta_connected(void) { return s_connected; }
