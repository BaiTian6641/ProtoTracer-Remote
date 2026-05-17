#pragma once

#include <cstdint>

#include "esp_err.h"

namespace prototracer
{
class I2cBus;

class Vcnl4035
{
public:
    struct Config
    {
        uint8_t address = 0x60;
        uint16_t proximity_low_threshold = 150;
        uint16_t proximity_high_threshold = 1200;
        bool enable_ambient = true;
        bool enable_proximity = true;
        bool enable_proximity_interrupts = true;
        bool enable_white_channel = true;
    };

    struct Sample
    {
        uint16_t proximity_1 = 0;
        uint16_t proximity_2 = 0;
        uint16_t proximity_3 = 0;
        uint16_t ambient = 0;
        uint16_t white = 0;
        uint8_t interrupt_flags = 0;
        bool gesture_ready = false;
        bool proximity_slot_1 = false;
        bool proximity_slot_2 = false;
        bool proximity_slot_3 = false;
        bool proximity_close = false;
        bool proximity_away = false;
        bool ambient_high = false;
        bool ambient_low = false;
    };

    esp_err_t init(I2cBus *bus, const Config &config);
    esp_err_t read_sample(Sample *out) const;
    esp_err_t set_channels_enabled(bool enable_proximity, bool enable_white_channel);
    bool ready() const;

private:
    esp_err_t apply_config_(const Config &config) const;
    esp_err_t write_word_(uint8_t reg, uint16_t value) const;
    esp_err_t read_word_(uint8_t reg, uint16_t *value) const;

    I2cBus *bus_ = nullptr;
    uint8_t address_ = 0;
    Config config_{};
    bool ready_ = false;
};
} // namespace prototracer