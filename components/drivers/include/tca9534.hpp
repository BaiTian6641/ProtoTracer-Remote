#pragma once

#include <cstdint>

#include "esp_err.h"

namespace prototracer
{
class I2cBus;

class Tca9534
{
public:
    esp_err_t init(I2cBus *bus, uint8_t address);
    esp_err_t set_configuration(uint8_t mask) const;
    esp_err_t set_polarity(uint8_t mask) const;
    esp_err_t read_inputs(uint8_t *value) const;

private:
    esp_err_t write_register_(uint8_t reg, uint8_t value) const;
    esp_err_t read_register_(uint8_t reg, uint8_t *value) const;

    I2cBus *bus_ = nullptr;
    uint8_t address_ = 0;
};
} // namespace prototracer