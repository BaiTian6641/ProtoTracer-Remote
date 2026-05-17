#include "lsm6dso.hpp"

#include "i2c_bus.hpp"

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace
{
constexpr const char *TAG = "lsm6dso";

constexpr uint8_t kRegInt1Ctrl = 0x0D;
constexpr uint8_t kRegInt2Ctrl = 0x0E;
constexpr uint8_t kRegWhoAmI = 0x0F;
constexpr uint8_t kRegCtrl1Xl = 0x10;
constexpr uint8_t kRegCtrl2G = 0x11;
constexpr uint8_t kRegCtrl3C = 0x12;
constexpr uint8_t kRegCtrl6C = 0x15;
constexpr uint8_t kRegCtrl9Xl = 0x18;
constexpr uint8_t kRegWakeUpSrc = 0x1B;
constexpr uint8_t kRegOutxLG = 0x22;
constexpr uint8_t kRegOutxLA = 0x28;
constexpr uint8_t kRegTapCfg0 = 0x56;
constexpr uint8_t kRegTapCfg2 = 0x58;
constexpr uint8_t kRegWakeUpThs = 0x5B;
constexpr uint8_t kRegWakeUpDur = 0x5C;
constexpr uint8_t kRegMd1Cfg = 0x5E;
constexpr uint8_t kRegMd2Cfg = 0x5F;

constexpr uint8_t kWhoAmIValue = 0x6C;
constexpr float kAccelScale2gMgPerLsb = 0.061f;
constexpr float kGyroScale250DpsMdpsPerLsb = 8.75f;
constexpr uint8_t kCtrl2GValue104Hz250Dps = 0x40;

int16_t decode_axis(const uint8_t low, const uint8_t high)
{
    return static_cast<int16_t>(static_cast<uint16_t>(low) | (static_cast<uint16_t>(high) << 8));
}
} // namespace

namespace prototracer
{
esp_err_t Lsm6dso::init(I2cBus *bus, const Config &config)
{
    if (bus == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }

    bus_ = bus;
    address_ = config.address;

    uint8_t who_am_i = 0;
    ESP_RETURN_ON_ERROR(read_who_am_i(&who_am_i), TAG, "Failed to read WHO_AM_I");
    if (who_am_i != kWhoAmIValue)
    {
        return ESP_ERR_INVALID_RESPONSE;
    }

    ESP_RETURN_ON_ERROR(write_register_(kRegCtrl3C, 0x44), TAG, "CTRL3_C configuration failed");
    ESP_RETURN_ON_ERROR(write_register_(kRegCtrl9Xl, 0x02), TAG, "CTRL9_XL configuration failed");
    const uint8_t ctrl2g = config.enable_gyro ? kCtrl2GValue104Hz250Dps : 0x00;
    ESP_RETURN_ON_ERROR(write_register_(kRegCtrl2G, ctrl2g), TAG, "CTRL2_G configuration failed");
    gyro_enabled_ = config.enable_gyro;
    ESP_RETURN_ON_ERROR(write_register_(kRegCtrl1Xl, 0x10), TAG, "CTRL1_XL configuration failed");
    ESP_RETURN_ON_ERROR(write_register_(kRegCtrl6C, 0x10), TAG, "CTRL6_C low-power configuration failed");
    ESP_RETURN_ON_ERROR(write_register_(kRegInt1Ctrl, 0x00), TAG, "INT1 DRDY routing clear failed");
    ESP_RETURN_ON_ERROR(write_register_(kRegInt2Ctrl, 0x00), TAG, "INT2 DRDY routing clear failed");
    ESP_RETURN_ON_ERROR(write_register_(kRegTapCfg0, 0x10), TAG, "TAP_CFG0 configuration failed");
    ESP_RETURN_ON_ERROR(write_register_(kRegTapCfg2, 0x80), TAG, "TAP_CFG2 configuration failed");
    ESP_RETURN_ON_ERROR(write_register_(kRegWakeUpThs, static_cast<uint8_t>(config.wake_threshold & 0x3F)), TAG, "WAKE_UP_THS configuration failed");
    ESP_RETURN_ON_ERROR(write_register_(kRegWakeUpDur, static_cast<uint8_t>(config.wake_duration & 0x7F)), TAG, "WAKE_UP_DUR configuration failed");
    ESP_RETURN_ON_ERROR(write_register_(kRegMd1Cfg, 0x20), TAG, "MD1_CFG configuration failed");
    ESP_RETURN_ON_ERROR(write_register_(kRegMd2Cfg, config.route_sleep_change_to_int2 ? 0x80 : 0x00), TAG, "MD2_CFG configuration failed");

    vTaskDelay(pdMS_TO_TICKS(5));
    ready_ = true;
    ESP_LOGI(TAG, "LSM6DSO ready on 0x%02x with low-power wake detection enabled", address_);
    return ESP_OK;
}

esp_err_t Lsm6dso::read_who_am_i(uint8_t *out) const
{
    return read_register_(kRegWhoAmI, out);
}

esp_err_t Lsm6dso::read_sample(Sample *out) const
{
    if (out == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (!ready_)
    {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t wake_source = 0;
    uint8_t accel_raw[6] = {};
    ESP_RETURN_ON_ERROR(read_register_(kRegWakeUpSrc, &wake_source), TAG, "Failed to read WAKE_UP_SRC");
    ESP_RETURN_ON_ERROR(read_registers_(kRegOutxLA, accel_raw, sizeof(accel_raw)), TAG, "Failed to read accelerometer sample");

    out->x_raw = decode_axis(accel_raw[0], accel_raw[1]);
    out->y_raw = decode_axis(accel_raw[2], accel_raw[3]);
    out->z_raw = decode_axis(accel_raw[4], accel_raw[5]);
    out->x_mg = static_cast<float>(out->x_raw) * kAccelScale2gMgPerLsb;
    out->y_mg = static_cast<float>(out->y_raw) * kAccelScale2gMgPerLsb;
    out->z_mg = static_cast<float>(out->z_raw) * kAccelScale2gMgPerLsb;
    out->wake_source = wake_source;
    out->z_wake = (wake_source & (1U << 0)) != 0;
    out->y_wake = (wake_source & (1U << 1)) != 0;
    out->x_wake = (wake_source & (1U << 2)) != 0;
    out->wake_event = (wake_source & (1U << 3)) != 0;
    out->sleep_change = (wake_source & (1U << 6)) != 0;

    if (gyro_enabled_)
    {
        uint8_t gyro_raw[6] = {};
        const esp_err_t gyro_err = read_registers_(kRegOutxLG, gyro_raw, sizeof(gyro_raw));
        if (gyro_err == ESP_OK)
        {
            out->gx_raw = decode_axis(gyro_raw[0], gyro_raw[1]);
            out->gy_raw = decode_axis(gyro_raw[2], gyro_raw[3]);
            out->gz_raw = decode_axis(gyro_raw[4], gyro_raw[5]);
            out->gx_dps = static_cast<float>(out->gx_raw) * kGyroScale250DpsMdpsPerLsb / 1000.0f;
            out->gy_dps = static_cast<float>(out->gy_raw) * kGyroScale250DpsMdpsPerLsb / 1000.0f;
            out->gz_dps = static_cast<float>(out->gz_raw) * kGyroScale250DpsMdpsPerLsb / 1000.0f;
            out->gyro_valid = true;
        }
    }
    return ESP_OK;
}

bool Lsm6dso::ready() const
{
    return ready_;
}

esp_err_t Lsm6dso::write_register_(const uint8_t reg, const uint8_t value) const
{
    if (bus_ == nullptr)
    {
        return ESP_ERR_INVALID_STATE;
    }

    const uint8_t payload[2] = {reg, value};
    return bus_->write(address_, payload, sizeof(payload));
}

esp_err_t Lsm6dso::read_register_(const uint8_t reg, uint8_t *value) const
{
    return read_registers_(reg, value, 1);
}

esp_err_t Lsm6dso::read_registers_(const uint8_t reg, uint8_t *data, const size_t length) const
{
    if (bus_ == nullptr)
    {
        return ESP_ERR_INVALID_STATE;
    }
    if (data == nullptr || length == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    return bus_->write_read(address_, &reg, 1, data, length);
}
} // namespace prototracer