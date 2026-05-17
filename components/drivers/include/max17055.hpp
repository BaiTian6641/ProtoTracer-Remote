#pragma once

#include <cstdint>

#include "esp_err.h"

namespace prototracer
{
class I2cBus;

class Max17055
{
public:
    struct Config
    {
        uint8_t address = 0x36;
        float sense_resistor_ohms = 0.01f;
        bool enable_alert_pin = true;
        bool enable_soc_change_alert = true;
    };

    struct Sample
    {
        float voltage_v = 0.0f;
        float current_ma = 0.0f;
        float average_current_ma = 0.0f;
        float state_of_charge_percent = 0.0f;
        float reported_capacity_mah = 0.0f;
        float temperature_c = 0.0f;
        uint16_t status = 0;
        bool power_on_reset = false;
        bool soc_change_alert = false;
    };

    esp_err_t init(I2cBus *bus, const Config &config);
    esp_err_t read_sample(Sample *out) const;
    esp_err_t clear_status_alerts() const;
    bool ready() const;

private:
    esp_err_t read_register_(uint8_t reg, uint16_t *value) const;
    esp_err_t write_register_(uint8_t reg, uint16_t value) const;
    esp_err_t update_register_(uint8_t reg, uint16_t clear_mask, uint16_t set_mask) const;
    static int16_t twos_complement_(uint16_t value);

    I2cBus *bus_ = nullptr;
    uint8_t address_ = 0;
    float capacity_lsb_mah_ = 0.0f;
    float current_lsb_ma_ = 0.0f;
    bool ready_ = false;
};
} // namespace prototracer