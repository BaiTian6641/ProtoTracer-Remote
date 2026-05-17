#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "driver/i2c_master.h"
#include "esp_err.h"

namespace prototracer
{
class I2cBus
{
public:
    struct Config
    {
        i2c_port_num_t port = I2C_NUM_0;
        gpio_num_t sda = GPIO_NUM_NC;
        gpio_num_t scl = GPIO_NUM_NC;
        uint32_t frequency_hz = 400000;
        bool enable_internal_pullups = true;
    };

    esp_err_t init(const Config &config);
    bool initialized() const;
    esp_err_t write(uint8_t address, const uint8_t *data, size_t length, uint32_t timeout_ms = 50) const;
    esp_err_t write_read(
        uint8_t address,
        const uint8_t *write_data,
        size_t write_length,
        uint8_t *read_data,
        size_t read_length,
        uint32_t timeout_ms = 50) const;

private:
    static constexpr size_t kMaxDevices = 8;

    esp_err_t ensure_device_(uint8_t address, i2c_master_dev_handle_t *handle) const;

    Config config_{};
    i2c_master_bus_handle_t bus_ = nullptr;
    mutable std::array<uint16_t, kMaxDevices> cached_addresses_{};
    mutable std::array<i2c_master_dev_handle_t, kMaxDevices> cached_devices_{};
    bool initialized_ = false;
};
} // namespace prototracer