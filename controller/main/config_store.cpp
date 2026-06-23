#include "config_store.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include <cstring>

static const char* TAG = "config_store";

esp_err_t ConfigStore::begin() {
    // nvs_flash_init() must be called in app_main before begin().
    return ESP_OK;
}

bool ConfigStore::load(std::string& out) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READONLY, &h);
    if (err == ESP_ERR_NVS_NOT_FOUND) return false;
    if (err != ESP_OK) { ESP_LOGE(TAG, "nvs_open: %s", esp_err_to_name(err)); return false; }

    size_t needed = 0;
    err = nvs_get_blob(h, NVS_KEY, nullptr, &needed);
    if (err == ESP_ERR_NVS_NOT_FOUND) { nvs_close(h); return false; }
    if (err != ESP_OK) { nvs_close(h); return false; }

    out.resize(needed);
    err = nvs_get_blob(h, NVS_KEY, &out[0], &needed);
    nvs_close(h);
    if (err != ESP_OK) { ESP_LOGE(TAG, "nvs_get_blob: %s", esp_err_to_name(err)); return false; }
    return true;
}

esp_err_t ConfigStore::save(const std::string& json) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) { ESP_LOGE(TAG, "nvs_open rw: %s", esp_err_to_name(err)); return err; }

    err = nvs_set_blob(h, NVS_KEY, json.c_str(), json.size());
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    if (err != ESP_OK) ESP_LOGE(TAG, "nvs_set_blob/commit: %s", esp_err_to_name(err));
    return err;
}
