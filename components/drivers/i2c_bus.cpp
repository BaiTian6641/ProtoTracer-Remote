#include "i2c_bus.hpp"

#include <algorithm>

#include "esp_check.h"
#include "esp_log.h"

namespace
{
constexpr const char *TAG = "i2c_bus";
} // namespace

namespace prototracer
{
esp_err_t I2cBus::init(const Config &config)
{
    if (initialized_)
    {
        return ESP_OK;
    }

    i2c_master_bus_config_t bus_config = {};
    bus_config.i2c_port = config.port;
    bus_config.sda_io_num = config.sda;
    bus_config.scl_io_num = config.scl;
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.glitch_ignore_cnt = 7;
    bus_config.intr_priority = 0;
    bus_config.trans_queue_depth = 0;
    bus_config.flags.enable_internal_pullup = config.enable_internal_pullups ? 1U : 0U;
    bus_config.flags.allow_pd = 0;

    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_config, &bus_), TAG, "I2C master bus creation failed");

    config_ = config;
    cached_addresses_.fill(I2C_DEVICE_ADDRESS_NOT_USED);
    cached_devices_.fill(nullptr);
    initialized_ = true;
    ESP_LOGI(TAG, "Initialized I2C master on port %d with SDA=%d SCL=%d @ %lu Hz", config.port, config.sda, config.scl, static_cast<unsigned long>(config.frequency_hz));
    return ESP_OK;
}

bool I2cBus::initialized() const
{
    return initialized_;
}

esp_err_t I2cBus::write(const uint8_t address, const uint8_t *data, const size_t length, const uint32_t timeout_ms) const
{
    if (!initialized_)
    {
        return ESP_ERR_INVALID_STATE;
    }
    if (data == nullptr || length == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_master_dev_handle_t device = nullptr;
    ESP_RETURN_ON_ERROR(ensure_device_(address, &device), TAG, "I2C device lookup failed");
    return i2c_master_transmit(device, data, length, static_cast<int>(timeout_ms));
}

esp_err_t I2cBus::write_read(
    const uint8_t address,
    const uint8_t *write_data,
    const size_t write_length,
    uint8_t *read_data,
    const size_t read_length,
    const uint32_t timeout_ms) const
{
    if (!initialized_)
    {
        return ESP_ERR_INVALID_STATE;
    }
    if (write_data == nullptr || write_length == 0 || read_data == nullptr || read_length == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_master_dev_handle_t device = nullptr;
    ESP_RETURN_ON_ERROR(ensure_device_(address, &device), TAG, "I2C device lookup failed");
    return i2c_master_transmit_receive(device, write_data, write_length, read_data, read_length, static_cast<int>(timeout_ms));
}

esp_err_t I2cBus::ensure_device_(const uint8_t address, i2c_master_dev_handle_t *handle) const
{
    if (!initialized_ || bus_ == nullptr)
    {
        return ESP_ERR_INVALID_STATE;
    }
    if (handle == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }

    for (size_t index = 0; index < cached_devices_.size(); ++index)
    {
        if (cached_devices_[index] != nullptr && cached_addresses_[index] == address)
        {
            *handle = cached_devices_[index];
            return ESP_OK;
        }
    }

    for (size_t index = 0; index < cached_devices_.size(); ++index)
    {
        if (cached_devices_[index] != nullptr)
        {
            continue;
        }

        i2c_device_config_t device_config = {};
        device_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        device_config.device_address = address;
        device_config.scl_speed_hz = config_.frequency_hz;
        device_config.scl_wait_us = 0;
        device_config.flags.disable_ack_check = 0;

        ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus_, &device_config, &cached_devices_[index]), TAG, "I2C device registration failed");
        cached_addresses_[index] = address;
        *handle = cached_devices_[index];
        return ESP_OK;
    }

    return ESP_ERR_NO_MEM;
}
} // namespace prototracer