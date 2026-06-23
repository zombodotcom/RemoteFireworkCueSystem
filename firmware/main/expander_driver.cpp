#include "expander_driver.h"
#include "board_config.h"
#include "esp_log.h"
static const char* TAG = "expander";

ExpanderChannelDriver::ExpanderChannelDriver(i2c_master_bus_handle_t bus, uint8_t addr, fwbox::FireLevel lvl)
    : bus_(bus), addr_(addr), lvl_(lvl), word_(fwbox::allOffWord(lvl)) {}

esp_err_t ExpanderChannelDriver::begin() {
    i2c_device_config_t dc = {};
    dc.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dc.device_address = addr_;
    dc.scl_speed_hz = board::I2C_FREQ_HZ;  // from board_config instead of hardcoded 100000
    esp_err_t err = i2c_master_bus_add_device(bus_, &dc, &dev_);
    if (err != ESP_OK) { ESP_LOGE(TAG, "add_device failed: %d", err); return err; }
    word_ = fwbox::allOffWord(lvl_);
    return writeWord(word_);            // outputs OFF before anything else
}

// PCF8575: a plain 2-byte write sets P0..P7 (low byte) then P10..P17 (high byte).
esp_err_t ExpanderChannelDriver::writeWord(uint16_t word) {
    uint8_t buf[2] = { (uint8_t)(word & 0xFF), (uint8_t)(word >> 8) };
    return i2c_master_transmit(dev_, buf, sizeof(buf), 50 /*ms*/);
}

void ExpanderChannelDriver::allOff() {
    word_ = fwbox::allOffWord(lvl_);
    esp_err_t err = writeWord(word_);
    if (err != ESP_OK) {
        // E-STOP path: log loudly — a silent I2C failure here means outputs may stay live
        ESP_LOGE(TAG, "allOff writeWord FAILED (I2C err %d) — outputs may not be safe!", err);
    }
}

void ExpanderChannelDriver::setChannel(uint8_t channel, bool on) {
    word_ = fwbox::applyChannel(word_, channel, on, lvl_);
    esp_err_t err = writeWord(word_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "setChannel(%u,%d) writeWord FAILED (I2C err %d)", channel, (int)on, err);
    }
}
