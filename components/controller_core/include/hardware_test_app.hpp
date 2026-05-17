#pragma once

#include "driver_services.hpp"
#include "esp_err.h"
#include "prototracer_types.hpp"

#include <cstdint>

namespace prototracer
{
class HardwareTestApp
{
public:
    HardwareTestApp(DisplayService &display,
                    StatusLed &status_led,
                    InputRouter &input_router,
                    SensorHub &sensor_hub,
                    PowerManager &power_manager,
                    UiLanguage language);

    // Runs the interactive hardware-test menu until the user picks "Resume".
    // Safe to skip by hardware fault paths; returns ESP_OK once the user exits.
    esp_err_t run();

    enum class MenuItem : uint8_t
    {
        Imu = 0,
        JoystickButtons = 1,
        Gesture = 2,
        Battery = 3,
        SideLeds = 4,
        Resume = 5,
        Count = 6,
    };

private:
    enum class ButtonEvent : uint8_t
    {
        None = 0,
        Up = 1,
        Down = 2,
        Left = 3,
        Right = 4,
        Select = 5,
        Back = 6,
        Action = 7,
    };

    struct InputState
    {
        bool joystick_button = false;
        bool button0 = false;
        bool button1 = false;
        bool button2 = false;
        bool joy_left = false;
        bool joy_right = false;
        bool joy_up = false;
        bool joy_down = false;
    };

    esp_err_t run_main_menu_();
    esp_err_t run_imu_test_();
    esp_err_t run_input_test_();
    esp_err_t run_gesture_test_();
    esp_err_t run_battery_test_();
    esp_err_t run_side_led_test_();

    void sample_inputs_(InputState *out);
    ButtonEvent poll_event_(InputState *current);
    void wait_release_();

    static const char *localized_(UiLanguage language, const char *english, const char *chinese);

    DisplayService &display_;
    StatusLed &status_led_;
    InputRouter &input_router_;
    SensorHub &sensor_hub_;
    PowerManager &power_manager_;
    UiLanguage language_;
    InputState last_state_{};
    bool initial_state_captured_ = false;
};
} // namespace prototracer
