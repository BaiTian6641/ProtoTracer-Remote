#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"

namespace prototracer
{
class I2cBus;

class Lsm6dso
{
public:
    struct Config
    {
        uint8_t address = 0x6B;
        uint8_t wake_threshold = 0x03;
        uint8_t wake_duration = 0x00;
        bool route_sleep_change_to_int2 = true;
        bool enable_gyro = true;
    };

    struct Sample
    {
        int16_t x_raw = 0;
        int16_t y_raw = 0;
        int16_t z_raw = 0;
        float x_mg = 0.0f;
        float y_mg = 0.0f;
        float z_mg = 0.0f;
        bool gyro_valid = false;
        int16_t gx_raw = 0;
        int16_t gy_raw = 0;
        int16_t gz_raw = 0;
        float gx_dps = 0.0f;
        float gy_dps = 0.0f;
        float gz_dps = 0.0f;
        uint8_t wake_source = 0;
        bool wake_event = false;
        bool sleep_change = false;
        bool x_wake = false;
        bool y_wake = false;
        bool z_wake = false;
    };

    esp_err_t init(I2cBus *bus, const Config &config);
    esp_err_t read_who_am_i(uint8_t *out) const;
    esp_err_t read_sample(Sample *out) const;
    bool ready() const;

private:
    esp_err_t write_register_(uint8_t reg, uint8_t value) const;
    esp_err_t read_register_(uint8_t reg, uint8_t *value) const;
    esp_err_t read_registers_(uint8_t reg, uint8_t *data, size_t length) const;

    I2cBus *bus_ = nullptr;
    uint8_t address_ = 0;
    bool ready_ = false;
    bool gyro_enabled_ = false;
};
} // namespace prototracer