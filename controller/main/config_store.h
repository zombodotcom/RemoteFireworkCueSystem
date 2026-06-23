#pragma once
#include "esp_err.h"
#include <string>

class ConfigStore {
public:
    esp_err_t begin();
    // Loads the stored JSON blob into `out`. Returns false if no config saved yet.
    bool load(std::string& out);
    esp_err_t save(const std::string& json);
private:
    static constexpr const char* NVS_NS  = "show";
    static constexpr const char* NVS_KEY = "cfg";
};
