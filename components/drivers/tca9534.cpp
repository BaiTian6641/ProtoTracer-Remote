#include "tca9534.hpp"

#include "i2c_bus.hpp"

#include "esp_log.h"

namespace
{
constexpr const char *TAG = "tca9534";

constexpr uint8_t kRegInputPort = 0x00;
constexpr uint8_t kRegPolarityInversion = 0x02;
constexpr uint8_t kRegConfiguration = 0x03;
} // namespace

namespace prototracer
{
esp_err_t Tca9534::init(I2cBus *bus, const uint8_t address)
{
    if (bus == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }

    bus_ = bus;
    address_ = address;
    ESP_LOGI(TAG, "Initialized TCA9534 at 0x%02x", address_);
    return ESP_OK;
}

esp_err_t Tca9534::set_configuration(const uint8_t mask) const
{
    return write_register_(kRegConfiguration, mask);
}

esp_err_t Tca9534::set_polarity(const uint8_t mask) const
{
    return write_register_(kRegPolarityInversion, mask);
}

esp_err_t Tca9534::read_inputs(uint8_t *value) const
{
    return read_register_(kRegInputPort, value);
}

esp_err_t Tca9534::write_register_(const uint8_t reg, const uint8_t value) const
{
    if (bus_ == nullptr)
    {
        return ESP_ERR_INVALID_STATE;
    }

    const uint8_t payload[2] = {reg, value};
    return bus_->write(address_, payload, sizeof(payload));
}

esp_err_t Tca9534::read_register_(const uint8_t reg, uint8_t *value) const
{
    if (bus_ == nullptr)
    {
        return ESP_ERR_INVALID_STATE;
    }
    if (value == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }

    return bus_->write_read(address_, &reg, 1, value, 1);
}
} // namespace prototracer