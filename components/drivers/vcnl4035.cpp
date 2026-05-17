#include "vcnl4035.hpp"

#include "i2c_bus.hpp"

#include "esp_check.h"
#include "esp_log.h"

namespace
{
constexpr const char *TAG = "vcnl4035";

constexpr uint8_t kRegAlsConf = 0x00;
constexpr uint8_t kRegAlsHighThreshold = 0x01;
constexpr uint8_t kRegAlsLowThreshold = 0x02;
constexpr uint8_t kRegPsConf1And2 = 0x03;
constexpr uint8_t kRegPsConf3AndCurrent = 0x04;
constexpr uint8_t kRegPsCancellation = 0x05;
constexpr uint8_t kRegPsHighThreshold = 0x06;
constexpr uint8_t kRegPsLowThreshold = 0x07;
constexpr uint8_t kRegPsData1 = 0x08;
constexpr uint8_t kRegPsData2 = 0x09;
constexpr uint8_t kRegPsData3 = 0x0A;
constexpr uint8_t kRegAlsData = 0x0B;
constexpr uint8_t kRegWhiteData = 0x0C;
constexpr uint8_t kRegIntFlag = 0x0D;
constexpr uint8_t kRegId = 0x0E;

constexpr uint16_t kExpectedIdLowByte = 0x0080;

constexpr uint8_t kAlsIntEnable = 1U << 1;
constexpr uint8_t kAlsShutdown = 1U << 0;

constexpr uint8_t kPsDuty80 = 1U << 6;
constexpr uint8_t kPsIt8T = (1U << 3) | (1U << 2) | (1U << 1);
constexpr uint8_t kPsShutdown = 1U << 0;

constexpr uint8_t kPsIntBoth = (1U << 1) | (1U << 0);
constexpr uint8_t kPs16Bit = 1U << 3;

constexpr uint8_t kPsSmartPersistence = 1U << 4;
constexpr uint8_t kLed120mA = (1U << 1) | (1U << 0);

constexpr uint8_t kFlagGestureReady = 1U << 7;
constexpr uint8_t kFlagProximity3 = 1U << 6;
constexpr uint8_t kFlagAlsLow = 1U << 5;
constexpr uint8_t kFlagAlsHigh = 1U << 4;
constexpr uint8_t kFlagProximity2 = 1U << 3;
constexpr uint8_t kFlagProximity1 = 1U << 2;
constexpr uint8_t kFlagProximityClose = 1U << 1;
constexpr uint8_t kFlagProximityAway = 1U << 0;
} // namespace

namespace prototracer
{
esp_err_t Vcnl4035::init(I2cBus *bus, const Config &config)
{
    if (bus == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }

    bus_ = bus;
    address_ = config.address;

    uint16_t id = 0;
    ESP_RETURN_ON_ERROR(read_word_(kRegId, &id), TAG, "Failed to read ID register");
    if ((id & 0x00FFU) != kExpectedIdLowByte)
    {
        return ESP_ERR_INVALID_RESPONSE;
    }

    ESP_RETURN_ON_ERROR(apply_config_(config), TAG, "VCNL4035 configuration failed");
    config_ = config;
    ready_ = true;
    ESP_LOGI(TAG, "VCNL4035 ready on 0x%02x with threshold-based interrupt routing enabled", address_);
    return ESP_OK;
}

esp_err_t Vcnl4035::set_channels_enabled(const bool enable_proximity, const bool enable_white_channel)
{
    if (!ready_ || bus_ == nullptr)
    {
        return ESP_ERR_INVALID_STATE;
    }

    Config next = config_;
    next.enable_proximity = enable_proximity;
    next.enable_white_channel = enable_white_channel;
    ESP_RETURN_ON_ERROR(apply_config_(next), TAG, "VCNL4035 channel update failed");
    config_ = next;
    return ESP_OK;
}

esp_err_t Vcnl4035::apply_config_(const Config &config) const
{

    uint8_t als_low = 0;
    if (!config.enable_ambient)
    {
        als_low |= kAlsShutdown;
    }
    if (config.enable_ambient && config.enable_proximity_interrupts)
    {
        als_low |= kAlsIntEnable;
    }
    ESP_RETURN_ON_ERROR(write_word_(kRegAlsConf, als_low), TAG, "ALS configuration failed");

    uint8_t ps_conf1 = static_cast<uint8_t>(kPsDuty80 | kPsIt8T);
    if (!config.enable_proximity)
    {
        ps_conf1 |= kPsShutdown;
    }
    uint8_t ps_conf2 = kPs16Bit;
    if (config.enable_proximity_interrupts)
    {
        ps_conf2 |= kPsIntBoth;
    }
    ESP_RETURN_ON_ERROR(write_word_(kRegPsConf1And2, static_cast<uint16_t>(ps_conf1) | (static_cast<uint16_t>(ps_conf2) << 8)), TAG, "PS configuration failed");

    uint8_t ps_conf3 = kPsSmartPersistence;
    uint8_t ps_current = kLed120mA;
    if (!config.enable_white_channel)
    {
        ps_current |= 1U << 7;
    }
    ESP_RETURN_ON_ERROR(write_word_(kRegPsConf3AndCurrent, static_cast<uint16_t>(ps_conf3) | (static_cast<uint16_t>(ps_current) << 8)), TAG, "PS current configuration failed");

    ESP_RETURN_ON_ERROR(write_word_(kRegPsCancellation, 0x0000), TAG, "PS cancellation reset failed");
    ESP_RETURN_ON_ERROR(write_word_(kRegPsHighThreshold, config.proximity_high_threshold), TAG, "PS high threshold configuration failed");
    ESP_RETURN_ON_ERROR(write_word_(kRegPsLowThreshold, config.proximity_low_threshold), TAG, "PS low threshold configuration failed");
    ESP_RETURN_ON_ERROR(write_word_(kRegAlsHighThreshold, 0xFFFF), TAG, "ALS high threshold configuration failed");
    ESP_RETURN_ON_ERROR(write_word_(kRegAlsLowThreshold, 0x0000), TAG, "ALS low threshold configuration failed");
    return ESP_OK;
}

esp_err_t Vcnl4035::read_sample(Sample *out) const
{
    if (out == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (!ready_)
    {
        return ESP_ERR_INVALID_STATE;
    }

    uint16_t word = 0;
    ESP_RETURN_ON_ERROR(read_word_(kRegPsData1, &out->proximity_1), TAG, "Failed to read PS1");
    ESP_RETURN_ON_ERROR(read_word_(kRegPsData2, &out->proximity_2), TAG, "Failed to read PS2");
    ESP_RETURN_ON_ERROR(read_word_(kRegPsData3, &out->proximity_3), TAG, "Failed to read PS3");
    ESP_RETURN_ON_ERROR(read_word_(kRegAlsData, &out->ambient), TAG, "Failed to read ALS");
    ESP_RETURN_ON_ERROR(read_word_(kRegWhiteData, &out->white), TAG, "Failed to read white channel");
    ESP_RETURN_ON_ERROR(read_word_(kRegIntFlag, &word), TAG, "Failed to read INT flags");

    out->interrupt_flags = static_cast<uint8_t>(word >> 8);
    out->gesture_ready = (out->interrupt_flags & kFlagGestureReady) != 0;
    out->proximity_slot_3 = (out->interrupt_flags & kFlagProximity3) != 0;
    out->ambient_low = (out->interrupt_flags & kFlagAlsLow) != 0;
    out->ambient_high = (out->interrupt_flags & kFlagAlsHigh) != 0;
    out->proximity_slot_2 = (out->interrupt_flags & kFlagProximity2) != 0;
    out->proximity_slot_1 = (out->interrupt_flags & kFlagProximity1) != 0;
    out->proximity_close = (out->interrupt_flags & kFlagProximityClose) != 0;
    out->proximity_away = (out->interrupt_flags & kFlagProximityAway) != 0;
    return ESP_OK;
}

bool Vcnl4035::ready() const
{
    return ready_;
}

esp_err_t Vcnl4035::write_word_(const uint8_t reg, const uint16_t value) const
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

esp_err_t Vcnl4035::read_word_(const uint8_t reg, uint16_t *value) const
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
    ESP_RETURN_ON_ERROR(bus_->write_read(address_, &reg, 1, payload, sizeof(payload)), TAG, "VCNL4035 read failed");
    *value = static_cast<uint16_t>(payload[0]) | (static_cast<uint16_t>(payload[1]) << 8);
    return ESP_OK;
}
} // namespace prototracer