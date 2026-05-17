#pragma once

#include <cstdint>

#include "driver/gpio.h"

namespace prototracer::board
{
constexpr char kBoardName[] = "ProtoTracer Remote";
constexpr char kMcuName[] = "ESP32-C6-MINI-1-N4";

constexpr gpio_num_t kPinExpanderInt = GPIO_NUM_2;
constexpr gpio_num_t kPinImuCs = GPIO_NUM_3;
constexpr gpio_num_t kPinChargerEnable = GPIO_NUM_4;
constexpr gpio_num_t kPinSensorRailEnable = GPIO_NUM_5;
constexpr gpio_num_t kPinRearEmitterEnable = GPIO_NUM_0;
constexpr gpio_num_t kPinAddressableLedData = GPIO_NUM_1;
constexpr gpio_num_t kPinJoystickY = GPIO_NUM_6;
constexpr gpio_num_t kPinJoystickX = GPIO_NUM_7;
constexpr gpio_num_t kPinBatteryMonitor = GPIO_NUM_12;
constexpr gpio_num_t kPinDisplayEnable = GPIO_NUM_13;
constexpr gpio_num_t kPinDisplayReset = GPIO_NUM_14;
constexpr gpio_num_t kPinPowerStatus = GPIO_NUM_15;
constexpr gpio_num_t kPinBoot = GPIO_NUM_9;
constexpr gpio_num_t kPinDisplayDc = GPIO_NUM_18;
constexpr gpio_num_t kPinDisplayCs = GPIO_NUM_19;
constexpr gpio_num_t kPinDisplaySclk = GPIO_NUM_20;
constexpr gpio_num_t kPinDisplayMosi = GPIO_NUM_21;
constexpr gpio_num_t kPinI2cSda = GPIO_NUM_22;
constexpr gpio_num_t kPinI2cScl = GPIO_NUM_23;
constexpr gpio_num_t kPinUart0Rx = GPIO_NUM_17;
constexpr gpio_num_t kPinUart0Tx = GPIO_NUM_16;

constexpr uint8_t kTca9534Address = 0x20;
constexpr uint8_t kLsm6dsoAddress = 0x6B;
constexpr uint8_t kVcnl4035Address = 0x60;
constexpr uint8_t kMax17055Address = 0x36;

enum class ExpanderInputBit : uint8_t
{
    JoystickButton = 0,
    Button0 = 1,
    Button1 = 2,
    Button2 = 3,
    GestureInterrupt = 4,
    ImuInterrupt1 = 5,
    ImuInterrupt2 = 6,
    FuelGaugeAlarm = 7,
};

struct BoardCapabilities
{
    bool has_gesture_sensor = true;
    bool has_imu = true;
    bool has_joystick = true;
    bool has_addressable_leds = true;
    bool has_low_power_core = true;
    bool has_display_power_gating = true;
};

const BoardCapabilities &capabilities();
const char *expander_input_name(ExpanderInputBit bit);
void log_summary();
} // namespace prototracer::board