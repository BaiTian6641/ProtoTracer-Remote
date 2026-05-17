#include "max17055.hpp"

#include "i2c_bus.hpp"

#include <cmath>

#include "esp_check.h"
#include "esp_log.h"

namespace
{
constexpr const char *TAG = "max17055";

constexpr uint8_t kRegStatus = 0x00;
constexpr uint8_t kRegRepCap = 0x05;
constexpr uint8_t kRegRepSoc = 0x06;
constexpr uint8_t kRegTemp = 0x08;
constexpr uint8_t kRegVCell = 0x09;
constexpr uint8_t kRegCurrent = 0x0A;
constexpr uint8_t kRegAvgCurrent = 0x0B;
constexpr uint8_t kRegConfig = 0x1D;
constexpr uint8_t kRegConfig2 = 0xBB;
constexpr uint8_t kRegDevName = 0x21;

constexpr uint16_t kConfigAlertPinBit = 1U << 2;
constexpr uint16_t kConfig2SocChangeBit = 1U << 7;
constexpr float kVoltageScaleMv = 0.078125f;
} // namespace

namespace prototracer
{
esp_err_t Max17055::init(I2cBus *bus, const Config &config)
{
    if (bus == nullptr || config.sense_resistor_ohms <= 0.0f)
    {
        return ESP_ERR_INVALID_ARG;
    }

    bus_ = bus;
    address_ = config.address;

    const float sense_milliohms = config.sense_resistor_ohms * 1000.0f;
    capacity_lsb_mah_ = 5.0f / sense_milliohms;
    current_lsb_ma_ = 1.5625f / sense_milliohms;

    uint16_t device_rev = 0;
    ESP_RETURN_ON_ERROR(read_register_(kRegDevName, &device_rev), TAG, "Failed to read device revision");

    const uint16_t config_set_bits = config.enable_alert_pin ? kConfigAlertPinBit : 0;
    const uint16_t config2_set_bits = config.enable_soc_change_alert ? kConfig2SocChangeBit : 0;
    ESP_RETURN_ON_ERROR(update_register_(kRegConfig, kConfigAlertPinBit, config_set_bits), TAG, "Failed to configure alert pin");
    ESP_RETURN_ON_ERROR(update_register_(kRegConfig2, kConfig2SocChangeBit, config2_set_bits), TAG, "Failed to configure SOC alert");

    ready_ = true;
    ESP_LOGI(TAG, "MAX17055 ready on 0x%02x (device=0x%04x, Rsense=%.3f ohm)", address_, device_rev, static_cast<double>(config.sense_resistor_ohms));
    return ESP_OK;
}

esp_err_t Max17055::read_sample(Sample *out) const
{
    if (out == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (!ready_)
    {
        return ESP_ERR_INVALID_STATE;
    }

    uint16_t status = 0;
    uint16_t rep_cap = 0;
    uint16_t rep_soc = 0;
    uint16_t temp = 0;
    uint16_t voltage = 0;
    uint16_t current = 0;
    uint16_t average_current = 0;

    ESP_RETURN_ON_ERROR(read_register_(kRegStatus, &status), TAG, "Failed to read STATUS");
    ESP_RETURN_ON_ERROR(read_register_(kRegRepCap, &rep_cap), TAG, "Failed to read REP_CAP");
    ESP_RETURN_ON_ERROR(read_register_(kRegRepSoc, &rep_soc), TAG, "Failed to read REP_SOC");
    ESP_RETURN_ON_ERROR(read_register_(kRegTemp, &temp), TAG, "Failed to read TEMP");
    ESP_RETURN_ON_ERROR(read_register_(kRegVCell, &voltage), TAG, "Failed to read VCELL");
    ESP_RETURN_ON_ERROR(read_register_(kRegCurrent, &current), TAG, "Failed to read CURRENT");
    ESP_RETURN_ON_ERROR(read_register_(kRegAvgCurrent, &average_current), TAG, "Failed to read AVG_CURRENT");

    out->status = status;
    out->power_on_reset = (status & 0x0002U) != 0;
    out->soc_change_alert = (status & 0x0080U) != 0;
    out->reported_capacity_mah = static_cast<float>(twos_complement_(rep_cap)) * capacity_lsb_mah_;
    out->state_of_charge_percent = static_cast<float>(rep_soc) / 256.0f;
    out->temperature_c = static_cast<float>(twos_complement_(temp)) / 256.0f;
    out->voltage_v = (static_cast<float>(voltage) * kVoltageScaleMv) / 1000.0f;
    out->current_ma = static_cast<float>(twos_complement_(current)) * current_lsb_ma_;
    out->average_current_ma = static_cast<float>(twos_complement_(average_current)) * current_lsb_ma_;
    return ESP_OK;
}

esp_err_t Max17055::clear_status_alerts() const
{
    if (!ready_)
    {
        return ESP_ERR_INVALID_STATE;
    }

    return write_register_(kRegStatus, 0x0000);
}

bool Max17055::ready() const
{
    return ready_;
}

esp_err_t Max17055::read_register_(const uint8_t reg, uint16_t *value) const
{
    if (bus_ == nullptr)
    {
        return ESP_ERR_INVALID_STATE;
    }
    if (value == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t payload[2] = {};
    ESP_RETURN_ON_ERROR(bus_->write_read(address_, &reg, 1, payload, sizeof(payload)), TAG, "MAX17055 read failed");
    *value = static_cast<uint16_t>(payload[0]) | (static_cast<uint16_t>(payload[1]) << 8);
    return ESP_OK;
}

esp_err_t Max17055::write_register_(const uint8_t reg, const uint16_t value) const
{
    if (bus_ == nullptr)
    {
        return ESP_ERR_INVALID_STATE;
    }

    const uint8_t payload[3] = {
        reg,
        static_cast<uint8_t>(value & 0x00FFU),
        static_cast<uint8_t>((value >> 8) & 0x00FFU),
    };
    return bus_->write(address_, payload, sizeof(payload));
}

esp_err_t Max17055::update_register_(const uint8_t reg, const uint16_t clear_mask, const uint16_t set_mask) const
{
    uint16_t value = 0;
    ESP_RETURN_ON_ERROR(read_register_(reg, &value), TAG, "Register read before update failed");
    value = static_cast<uint16_t>((value & static_cast<uint16_t>(~clear_mask)) | set_mask);
    return write_register_(reg, value);
}

int16_t Max17055::twos_complement_(const uint16_t value)
{
    return static_cast<int16_t>(value);
}
} // namespace prototracer