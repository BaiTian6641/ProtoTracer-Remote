#pragma once

#include <cstdint>

#include "max17055.hpp"
#include "lsm6dso.hpp"
#include "prototracer_board.hpp"
#include "vcnl4035.hpp"
#include "esp_err.h"
#include "prototracer_types.hpp"
#include "freertos/FreeRTOS.h"

namespace prototracer
{
class InputRouter;

struct InputSnapshot
{
    uint8_t raw_bits = 0xFF;

    bool is_asserted(board::ExpanderInputBit bit) const;
};

struct InputEvent
{
    enum class Type : uint8_t
    {
        ButtonPressed = 0,
        ButtonReleased = 1,
        SensorInterruptAsserted = 2,
        SensorInterruptReleased = 3,
    };

    Type type = Type::ButtonPressed;
    board::ExpanderInputBit source = board::ExpanderInputBit::JoystickButton;
    uint8_t raw_bits = 0xFF;
    uint32_t tick_ms = 0;

    bool asserted() const;
};

using InputEventCallback = void (*)(const InputEvent &event, void *context);

struct JoystickSample
{
    bool valid = false;
    bool x_available = false;
    bool x_digital = false;
    bool y_available = false;
    int raw_x = 0;
    int raw_y = 0;
    int normalized_x = 0;
    int normalized_y = 0;
    bool pressed = false;
    bool left = false;
    bool right = false;
    bool up = false;
    bool down = false;
};

struct MotionSample
{
    bool valid = false;
    int16_t x_raw = 0;
    int16_t y_raw = 0;
    int16_t z_raw = 0;
    float x_mg = 0.0f;
    float y_mg = 0.0f;
    float z_mg = 0.0f;
    bool gyro_valid = false;
    int16_t gx_raw = 0;
    int16_t gy_raw = 0;
    int16_t gz_raw = 0;
    float gx_dps = 0.0f;
    float gy_dps = 0.0f;
    float gz_dps = 0.0f;
    uint8_t wake_source = 0;
    bool wake_event = false;
    bool sleep_change = false;
    bool x_wake = false;
    bool y_wake = false;
    bool z_wake = false;
};

struct GestureSample
{
    bool valid = false;
    uint16_t proximity_1 = 0;
    uint16_t proximity_2 = 0;
    uint16_t proximity_3 = 0;
    uint16_t ambient = 0;
    uint16_t white = 0;
    uint8_t interrupt_flags = 0;
    bool gesture_ready = false;
    bool proximity_close = false;
    bool proximity_away = false;
    bool ambient_high = false;
    bool ambient_low = false;
};

struct FuelGaugeSample
{
    bool valid = false;
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

struct SensorEvent
{
    enum class Type : uint8_t
    {
        Motion = 0,
        Gesture = 1,
        FuelGauge = 2,
        Fault = 3,
    };

    Type type = Type::Fault;
    uint32_t tick_ms = 0;
    esp_err_t status = ESP_OK;
    board::ExpanderInputBit source = board::ExpanderInputBit::GestureInterrupt;
    MotionSample motion = {};
    GestureSample gesture = {};
    FuelGaugeSample battery = {};
};

class SensorHub
{
public:
    esp_err_t init();
    esp_err_t attach_input_router(InputRouter *router);
    esp_err_t wait_for_event(SensorEvent *out, TickType_t timeout) const;
    esp_err_t get_latest_motion(MotionSample *out) const;
    esp_err_t get_latest_gesture(GestureSample *out) const;
    esp_err_t get_latest_battery(FuelGaugeSample *out) const;
    esp_err_t refresh_battery(FuelGaugeSample *out) const;
    esp_err_t sample_motion_now(MotionSample *out) const;
    esp_err_t sample_gesture_now(GestureSample *out) const;
    esp_err_t configure_gesture_channels(bool enable_proximity, bool enable_white_channel) const;
};

class InputRouter
{
public:
    esp_err_t init();
    esp_err_t get_snapshot(InputSnapshot *out) const;
    esp_err_t read_joystick(JoystickSample *out) const;
    esp_err_t wait_for_event(InputEvent *out, TickType_t timeout) const;
    esp_err_t register_listener(InputEventCallback callback, void *context);
};

class PowerManager
{
public:
    esp_err_t init();
    BatteryChemistry classify_battery_chemistry() const;
    esp_err_t update_charger_control(BatteryChemistry chemistry);
    bool external_power_present() const;
    bool charger_enabled() const;
    void log_hardware_limits() const;
};

class StatusLed
{
public:
    esp_err_t init();
    esp_err_t set_mode(StatusLedMode mode);
    esp_err_t set_pixel(uint8_t index, uint8_t red, uint8_t green, uint8_t blue);
    esp_err_t refresh();
    esp_err_t clear();
    static constexpr uint8_t pixel_count = 4;
};

class DisplayService
{
public:
    static constexpr uint8_t kTestMenuMaxLines = 6;
    static constexpr uint8_t kTestMenuLineLength = 22;
    static constexpr uint8_t kTestMenuNoCursor = 0xFF;

    esp_err_t init();
    void set_language(UiLanguage language);
    esp_err_t set_brightness(uint8_t brightness);
    esp_err_t set_sleeping(bool sleeping);
    void set_signal_strength(uint8_t percent, bool visible);
    esp_err_t show_status(const char *title, const char *detail, StatusLedMode mode);
    esp_err_t show_provisioning(const char *portal_name, const char *hint, uint8_t percent);
    esp_err_t show_bind_progress(const char *phase, uint8_t percent);
    esp_err_t show_error(const char *title, const char *detail, const char *hint);
    esp_err_t show_update_progress(const char *phase, uint8_t percent);
    esp_err_t show_test_menu(const char *title,
                             const char *aux,
                             const char (*lines)[kTestMenuLineLength],
                             uint8_t line_count,
                             uint8_t selected_index);
};

class LowPowerController
{
public:
    esp_err_t init(bool enabled);
};
} // namespace prototracer