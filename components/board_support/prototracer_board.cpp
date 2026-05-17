#include "prototracer_board.hpp"

#include "esp_log.h"

namespace prototracer::board
{
namespace
{
constexpr const char *TAG = "board";
constexpr BoardCapabilities kCapabilities{};
} // namespace

const BoardCapabilities &capabilities()
{
    return kCapabilities;
}

const char *expander_input_name(const ExpanderInputBit bit)
{
    switch (bit)
    {
    case ExpanderInputBit::JoystickButton:
        return "joystick_button";
    case ExpanderInputBit::Button0:
        return "button_0";
    case ExpanderInputBit::Button1:
        return "button_1";
    case ExpanderInputBit::Button2:
        return "button_2";
    case ExpanderInputBit::GestureInterrupt:
        return "gesture_interrupt";
    case ExpanderInputBit::ImuInterrupt1:
        return "imu_interrupt_1";
    case ExpanderInputBit::ImuInterrupt2:
        return "imu_interrupt_2";
    case ExpanderInputBit::FuelGaugeAlarm:
        return "fuel_gauge_alarm";
    default:
        return "unknown";
    }
}

void log_summary()
{
    ESP_LOGI(TAG, "Board: %s", kBoardName);
    ESP_LOGI(TAG, "MCU: %s", kMcuName);
    ESP_LOGI(TAG, "I2C bus: SDA=%d SCL=%d", kPinI2cSda, kPinI2cScl);
    if (kPinExpanderInt == GPIO_NUM_NC)
    {
        ESP_LOGI(TAG, "Expander IRQ=disabled (polling only), IMU CS=%d, battery monitor=%d", kPinImuCs, kPinBatteryMonitor);
    }
    else
    {
        ESP_LOGI(TAG, "Expander IRQ=%d, IMU CS=%d, battery monitor=%d", kPinExpanderInt, kPinImuCs, kPinBatteryMonitor);
    }
    ESP_LOGI(TAG, "Display: EN=%d RES=%d DC=%d CS=%d SCLK=%d MOSI=%d", kPinDisplayEnable, kPinDisplayReset, kPinDisplayDc, kPinDisplayCs, kPinDisplaySclk, kPinDisplayMosi);
    ESP_LOGI(TAG, "Rear emitters: EN=%d, WS2812 data=%d", kPinRearEmitterEnable, kPinAddressableLedData);
    ESP_LOGI(TAG, "Joystick axes: X=%d Y=%d", kPinJoystickX, kPinJoystickY);
}
} // namespace prototracer::board