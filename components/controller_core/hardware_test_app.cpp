#include "hardware_test_app.hpp"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "prototracer_board.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>

namespace
{
constexpr const char *TAG = "hw_test";
constexpr TickType_t kPollIntervalTicks = pdMS_TO_TICKS(40);
constexpr uint32_t kSampleIntervalMs = 120;

const char *menu_label(prototracer::HardwareTestApp::MenuItem item, prototracer::UiLanguage language);
} // namespace

namespace prototracer
{
HardwareTestApp::HardwareTestApp(DisplayService &display,
                                 StatusLed &status_led,
                                 InputRouter &input_router,
                                 SensorHub &sensor_hub,
                                 PowerManager &power_manager,
                                 const UiLanguage language)
    : display_(display),
      status_led_(status_led),
      input_router_(input_router),
      sensor_hub_(sensor_hub),
      power_manager_(power_manager),
      language_(language)
{
}

const char *HardwareTestApp::localized_(const UiLanguage language, const char *english, const char *chinese)
{
    return language == UiLanguage::Chinese ? chinese : english;
}

void HardwareTestApp::sample_inputs_(InputState *out)
{
    if (out == nullptr)
    {
        return;
    }
    InputSnapshot snapshot = {};
    if (input_router_.get_snapshot(&snapshot) != ESP_OK)
    {
        snapshot.raw_bits = 0xFF;
    }
    out->joystick_button = snapshot.is_asserted(board::ExpanderInputBit::JoystickButton);
    out->button0 = snapshot.is_asserted(board::ExpanderInputBit::Button0);
    out->button1 = snapshot.is_asserted(board::ExpanderInputBit::Button1);
    out->button2 = snapshot.is_asserted(board::ExpanderInputBit::Button2);

    JoystickSample joystick = {};
    if (input_router_.read_joystick(&joystick) == ESP_OK)
    {
        out->joy_left = joystick.left;
        out->joy_right = joystick.right;
        out->joy_up = joystick.up;
        out->joy_down = joystick.down;
    }
    else
    {
        out->joy_left = false;
        out->joy_right = false;
        out->joy_up = false;
        out->joy_down = false;
    }
}

HardwareTestApp::ButtonEvent HardwareTestApp::poll_event_(InputState *current)
{
    InputState state = {};
    sample_inputs_(&state);
    if (current != nullptr)
    {
        *current = state;
    }

    ButtonEvent event = ButtonEvent::None;
    if (!initial_state_captured_)
    {
        last_state_ = state;
        initial_state_captured_ = true;
        return ButtonEvent::None;
    }

    const auto rising = [](bool now, bool prev) { return now && !prev; };

    if (rising(state.joy_up, last_state_.joy_up))
    {
        event = ButtonEvent::Up;
    }
    else if (rising(state.joy_down, last_state_.joy_down))
    {
        event = ButtonEvent::Down;
    }
    else if (rising(state.joy_left, last_state_.joy_left))
    {
        event = ButtonEvent::Left;
    }
    else if (rising(state.joy_right, last_state_.joy_right))
    {
        event = ButtonEvent::Right;
    }
    else if (rising(state.joystick_button, last_state_.joystick_button) || rising(state.button0, last_state_.button0))
    {
        event = ButtonEvent::Select;
    }
    else if (rising(state.button2, last_state_.button2))
    {
        event = ButtonEvent::Back;
    }
    else if (rising(state.button1, last_state_.button1))
    {
        event = ButtonEvent::Action;
    }

    last_state_ = state;
    return event;
}

void HardwareTestApp::wait_release_()
{
    for (int i = 0; i < 25; ++i)
    {
        InputState state = {};
        sample_inputs_(&state);
        if (!state.joystick_button && !state.button0 && !state.button1 && !state.button2
            && !state.joy_up && !state.joy_down && !state.joy_left && !state.joy_right)
        {
            last_state_ = state;
            return;
        }
        vTaskDelay(kPollIntervalTicks);
    }
}

esp_err_t HardwareTestApp::run()
{
    ESP_LOGI(TAG, "Hardware test UI entered");
    initial_state_captured_ = false;
    wait_release_();
    return run_main_menu_();
}

esp_err_t HardwareTestApp::run_main_menu_()
{
    uint8_t cursor = 0;
    constexpr uint8_t item_count = static_cast<uint8_t>(MenuItem::Count);
    char lines[DisplayService::kTestMenuMaxLines][DisplayService::kTestMenuLineLength] = {};
    bool need_redraw = true;

    while (true)
    {
        if (need_redraw)
        {
            for (uint8_t i = 0; i < item_count && i < DisplayService::kTestMenuMaxLines; ++i)
            {
                std::snprintf(lines[i], sizeof(lines[i]), "%s", menu_label(static_cast<MenuItem>(i), language_));
            }
            display_.show_test_menu(localized_(language_, "HW Test", "硬件测试"),
                                    nullptr,
                                    lines,
                                    item_count,
                                    cursor);
            need_redraw = false;
        }

        InputState state = {};
        const ButtonEvent event = poll_event_(&state);
        switch (event)
        {
        case ButtonEvent::Up:
        case ButtonEvent::Left:
            cursor = (cursor == 0) ? (item_count - 1) : (cursor - 1);
            need_redraw = true;
            break;
        case ButtonEvent::Down:
        case ButtonEvent::Right:
            cursor = (cursor + 1) % item_count;
            need_redraw = true;
            break;
        case ButtonEvent::Select:
        {
            const MenuItem chosen = static_cast<MenuItem>(cursor);
            if (chosen == MenuItem::Resume)
            {
                ESP_LOGI(TAG, "Hardware test UI resumed normal boot");
                return ESP_OK;
            }
            wait_release_();
            switch (chosen)
            {
            case MenuItem::Imu:
                run_imu_test_();
                break;
            case MenuItem::JoystickButtons:
                run_input_test_();
                break;
            case MenuItem::Gesture:
                run_gesture_test_();
                break;
            case MenuItem::Battery:
                run_battery_test_();
                break;
            case MenuItem::SideLeds:
                run_side_led_test_();
                break;
            default:
                break;
            }
            wait_release_();
            need_redraw = true;
            break;
        }
        default:
            break;
        }

        vTaskDelay(kPollIntervalTicks);
    }
}

esp_err_t HardwareTestApp::run_imu_test_()
{
    ESP_LOGI(TAG, "IMU test entered");
    char lines[DisplayService::kTestMenuMaxLines][DisplayService::kTestMenuLineLength] = {};
    uint32_t last_sample_ms = 0;
    MotionSample sample = {};
    while (true)
    {
        const uint32_t now = static_cast<uint32_t>(esp_log_timestamp());
        if (now - last_sample_ms >= kSampleIntervalMs)
        {
            last_sample_ms = now;
            const esp_err_t err = sensor_hub_.sample_motion_now(&sample);
            if (err != ESP_OK)
            {
                std::snprintf(lines[0], sizeof(lines[0]), "%s", localized_(language_, "IMU offline", "IMU 离线"));
                std::snprintf(lines[1], sizeof(lines[1]), "%s", esp_err_to_name(err));
                std::snprintf(lines[2], sizeof(lines[2]), "%s", "");
                std::snprintf(lines[3], sizeof(lines[3]), "%s", "");
                std::snprintf(lines[4], sizeof(lines[4]), "%s", "");
                std::snprintf(lines[5], sizeof(lines[5]), "%s", localized_(language_, "B2 back", "B2 返回"));
            }
            else
            {
                std::snprintf(lines[0], sizeof(lines[0]), "AX%+4d AY%+4d", static_cast<int>(sample.x_mg), static_cast<int>(sample.y_mg));
                std::snprintf(lines[1], sizeof(lines[1]), "AZ%+4d W%u", static_cast<int>(sample.z_mg), sample.wake_event ? 1U : 0U);
                if (sample.gyro_valid)
                {
                    std::snprintf(lines[2], sizeof(lines[2]), "GX%+3d GY%+3d",
                                  static_cast<int>(sample.gx_dps),
                                  static_cast<int>(sample.gy_dps));
                    std::snprintf(lines[3], sizeof(lines[3]), "GZ%+3d", static_cast<int>(sample.gz_dps));
                }
                else
                {
                    std::snprintf(lines[2], sizeof(lines[2]), "%s", localized_(language_, "Gyro n/a", "陀螺无效"));
                    std::snprintf(lines[3], sizeof(lines[3]), "%s", "");
                }
                std::snprintf(lines[4], sizeof(lines[4]), "%s", localized_(language_, "Move to test", "移动测试"));
                std::snprintf(lines[5], sizeof(lines[5]), "%s", localized_(language_, "B2 back", "B2 返回"));
            }
            display_.show_test_menu(localized_(language_, "IMU", "惯性"),
                                    nullptr,
                                    lines,
                                    DisplayService::kTestMenuMaxLines,
                                    DisplayService::kTestMenuNoCursor);
        }

        const ButtonEvent event = poll_event_(nullptr);
        if (event == ButtonEvent::Back)
        {
            return ESP_OK;
        }
        vTaskDelay(kPollIntervalTicks);
    }
}

esp_err_t HardwareTestApp::run_input_test_()
{
    ESP_LOGI(TAG, "Input test entered");
    char lines[DisplayService::kTestMenuMaxLines][DisplayService::kTestMenuLineLength] = {};
    uint32_t last_sample_ms = 0;
    while (true)
    {
        const uint32_t now = static_cast<uint32_t>(esp_log_timestamp());
        if (now - last_sample_ms >= 80)
        {
            last_sample_ms = now;
            JoystickSample joystick = {};
            (void)input_router_.read_joystick(&joystick);
            InputSnapshot snapshot = {};
            (void)input_router_.get_snapshot(&snapshot);

            if (joystick.x_available && joystick.x_digital)
            {
                std::snprintf(lines[0], sizeof(lines[0]), "X D%u N%+3d", static_cast<unsigned>(joystick.raw_x != 0 ? 1 : 0), joystick.normalized_x);
            }
            else if (joystick.x_available)
            {
                std::snprintf(lines[0], sizeof(lines[0]), "X %4d N%+3d", joystick.raw_x, joystick.normalized_x);
            }
            else
            {
                std::snprintf(lines[0], sizeof(lines[0]), "%s", localized_(language_, "X n/a", "X 无效"));
            }

            if (joystick.y_available)
            {
                std::snprintf(lines[1], sizeof(lines[1]), "Y %4d N%+3d", joystick.raw_y, joystick.normalized_y);
            }
            else
            {
                std::snprintf(lines[1], sizeof(lines[1]), "%s", localized_(language_, "Y n/a", "Y 无效"));
            }

            std::snprintf(lines[2], sizeof(lines[2]), "L%u R%u U%u D%u",
                          joystick.left ? 1 : 0,
                          joystick.right ? 1 : 0,
                          joystick.up ? 1 : 0,
                          joystick.down ? 1 : 0);
            std::snprintf(lines[3], sizeof(lines[3]), "J%u 0:%u 1:%u 2:%u",
                          snapshot.is_asserted(board::ExpanderInputBit::JoystickButton) ? 1U : 0U,
                          snapshot.is_asserted(board::ExpanderInputBit::Button0) ? 1U : 0U,
                          snapshot.is_asserted(board::ExpanderInputBit::Button1) ? 1U : 0U,
                          snapshot.is_asserted(board::ExpanderInputBit::Button2) ? 1U : 0U);
            std::snprintf(lines[4], sizeof(lines[4]), "%s", joystick.x_digital ? localized_(language_, "X is 1-bit", "X 为 1 位") : localized_(language_, "Axes analog", "轴为模拟量"));
            std::snprintf(lines[5], sizeof(lines[5]), "%s", localized_(language_, "Hold B2 back", "长按 B2 返回"));

            display_.show_test_menu(localized_(language_, "Inputs", "输入测试"),
                                    nullptr,
                                    lines,
                                    DisplayService::kTestMenuMaxLines,
                                    DisplayService::kTestMenuNoCursor);
        }

        // For the input test we exit on a long-press of Button2 to avoid stealing
        // every B2 tap that the user may want to test. Use a 600 ms hold.
        InputState state = {};
        sample_inputs_(&state);
        if (state.button2)
        {
            int held_ms = 0;
            while (held_ms < 600)
            {
                vTaskDelay(pdMS_TO_TICKS(40));
                held_ms += 40;
                sample_inputs_(&state);
                if (!state.button2)
                {
                    break;
                }
            }
            if (state.button2)
            {
                wait_release_();
                return ESP_OK;
            }
        }

        last_state_ = state;
        vTaskDelay(kPollIntervalTicks);
    }
}

esp_err_t HardwareTestApp::run_gesture_test_()
{
    ESP_LOGI(TAG, "Gesture test entered");
    char lines[DisplayService::kTestMenuMaxLines][DisplayService::kTestMenuLineLength] = {};
    uint32_t last_sample_ms = 0;
    GestureSample sample = {};
    bool rear_emitters_enabled = false;
    uint8_t rear_brightness = 128;
    bool gesture_ir_enabled = true;
    bool gesture_white_enabled = true;
    esp_err_t config_err = sensor_hub_.configure_gesture_channels(gesture_ir_enabled, gesture_white_enabled);

    constexpr ledc_mode_t kRearPwmMode = LEDC_LOW_SPEED_MODE;
    constexpr ledc_timer_t kRearPwmTimer = LEDC_TIMER_0;
    constexpr ledc_channel_t kRearPwmChannel = LEDC_CHANNEL_0;
    constexpr uint32_t kRearPwmFrequencyHz = 2000;
    constexpr uint32_t kRearPwmMaxDuty = 255;

    ledc_timer_config_t timer_config = {};
    timer_config.speed_mode = kRearPwmMode;
    timer_config.timer_num = kRearPwmTimer;
    timer_config.duty_resolution = LEDC_TIMER_8_BIT;
    timer_config.freq_hz = kRearPwmFrequencyHz;
    timer_config.clk_cfg = LEDC_AUTO_CLK;

    ledc_channel_config_t channel_config = {};
    channel_config.gpio_num = board::kPinRearEmitterEnable;
    channel_config.speed_mode = kRearPwmMode;
    channel_config.channel = kRearPwmChannel;
    channel_config.timer_sel = kRearPwmTimer;
    channel_config.duty = 0;
    channel_config.hpoint = 0;

    const bool pwm_ready = ledc_timer_config(&timer_config) == ESP_OK && ledc_channel_config(&channel_config) == ESP_OK;

    const auto apply_rear_emitters = [&]() {
        if (pwm_ready)
        {
            const uint32_t duty = rear_emitters_enabled ? std::min<uint32_t>(rear_brightness, kRearPwmMaxDuty) : 0;
            ledc_set_duty(kRearPwmMode, kRearPwmChannel, duty);
            ledc_update_duty(kRearPwmMode, kRearPwmChannel);
        }
        else
        {
            gpio_set_level(board::kPinRearEmitterEnable, rear_emitters_enabled ? 1 : 0);
        }
    };
    apply_rear_emitters();

    while (true)
    {
        const uint32_t now = static_cast<uint32_t>(esp_log_timestamp());
        if (now - last_sample_ms >= kSampleIntervalMs)
        {
            last_sample_ms = now;
            const esp_err_t err = sensor_hub_.sample_gesture_now(&sample);
            std::snprintf(lines[0], sizeof(lines[0]), "Rear %3u%% IR%u W%u",
                          rear_emitters_enabled ? static_cast<unsigned>((static_cast<uint32_t>(rear_brightness) * 100U) / 255U) : 0U,
                          gesture_ir_enabled ? 1U : 0U,
                          gesture_white_enabled ? 1U : 0U);
            if (err != ESP_OK)
            {
                std::snprintf(lines[1], sizeof(lines[1]), "%s", localized_(language_, "Gesture off", "手势离线"));
                std::snprintf(lines[2], sizeof(lines[2]), "%s", config_err == ESP_OK ? esp_err_to_name(err) : esp_err_to_name(config_err));
                std::snprintf(lines[3], sizeof(lines[3]), "%s", localized_(language_, "Rear rail only", "仅后部发光"));
                std::snprintf(lines[4], sizeof(lines[4]), "%s", localized_(language_, "OK rear L/R dim", "确认后灯 左右调光"));
                std::snprintf(lines[5], sizeof(lines[5]), "%s", localized_(language_, "Up white B2 back", "上白通道 B2返回"));
            }
            else
            {
                std::snprintf(lines[1], sizeof(lines[1]), "P1 %4u P2 %4u", sample.proximity_1, sample.proximity_2);
                std::snprintf(lines[2], sizeof(lines[2]), "P3 %4u", sample.proximity_3);
                std::snprintf(lines[3], sizeof(lines[3]), "ALS %4u W %4u", sample.ambient, sample.white);
                std::snprintf(lines[4], sizeof(lines[4]), "G%u C%u A%u",
                              sample.gesture_ready ? 1U : 0U,
                              sample.proximity_close ? 1U : 0U,
                              sample.proximity_away ? 1U : 0U);
                std::snprintf(lines[5], sizeof(lines[5]), "%s", localized_(language_, "L/R dim B1 IR U W", "左右调光 B1红外 上白"));
            }
            display_.show_test_menu(localized_(language_, "Gesture", "手势"),
                                    nullptr,
                                    lines,
                                    DisplayService::kTestMenuMaxLines,
                                    DisplayService::kTestMenuNoCursor);
        }

        const ButtonEvent event = poll_event_(nullptr);
        if (event == ButtonEvent::Select)
        {
            rear_emitters_enabled = !rear_emitters_enabled;
            apply_rear_emitters();
            last_sample_ms = 0;
        }
        else if (event == ButtonEvent::Action)
        {
            gesture_ir_enabled = !gesture_ir_enabled;
            config_err = sensor_hub_.configure_gesture_channels(gesture_ir_enabled, gesture_white_enabled);
            last_sample_ms = 0;
        }
        else if (event == ButtonEvent::Left || event == ButtonEvent::Right)
        {
            const int delta = event == ButtonEvent::Right ? 32 : -32;
            rear_brightness = static_cast<uint8_t>(std::clamp<int>(static_cast<int>(rear_brightness) + delta, 16, 255));
            if (!rear_emitters_enabled)
            {
                rear_emitters_enabled = true;
            }
            apply_rear_emitters();
            last_sample_ms = 0;
        }
        else if (event == ButtonEvent::Up)
        {
            gesture_white_enabled = !gesture_white_enabled;
            config_err = sensor_hub_.configure_gesture_channels(gesture_ir_enabled, gesture_white_enabled);
            last_sample_ms = 0;
        }
        else if (event == ButtonEvent::Back)
        {
            rear_emitters_enabled = false;
            apply_rear_emitters();
            if (pwm_ready)
            {
                ledc_stop(kRearPwmMode, kRearPwmChannel, 0);
            }
            gpio_set_level(board::kPinRearEmitterEnable, 0);
            (void)sensor_hub_.configure_gesture_channels(true, true);
            return ESP_OK;
        }
        vTaskDelay(kPollIntervalTicks);
    }
}

esp_err_t HardwareTestApp::run_battery_test_()
{
    ESP_LOGI(TAG, "Battery test entered");
    char lines[DisplayService::kTestMenuMaxLines][DisplayService::kTestMenuLineLength] = {};
    uint32_t last_sample_ms = 0;
    FuelGaugeSample sample = {};
    while (true)
    {
        const uint32_t now = static_cast<uint32_t>(esp_log_timestamp());
        if (now - last_sample_ms >= 500)
        {
            last_sample_ms = now;
            const esp_err_t err = sensor_hub_.refresh_battery(&sample);
            const BatteryChemistry chem = (err == ESP_OK && sample.valid) ? power_manager_.classify_battery_chemistry() : BatteryChemistry::Unknown;
            (void)power_manager_.update_charger_control(chem);
            const char *chem_name = "Unk";
            switch (chem)
            {
            case BatteryChemistry::NiMH:
                chem_name = "NiMH";
                break;
            case BatteryChemistry::Alkaline:
                chem_name = "Alk";
                break;
            default:
                chem_name = "Unk";
                break;
            }
            const bool vbat_high = gpio_get_level(board::kPinBatteryMonitor) != 0;
            const bool pwr_stat = gpio_get_level(board::kPinPowerStatus) != 0;
            if (err != ESP_OK || !sample.valid)
            {
                std::snprintf(lines[0], sizeof(lines[0]), "%s", localized_(language_, "Gauge offline", "电量计离线"));
                std::snprintf(lines[1], sizeof(lines[1]), "%s", esp_err_to_name(err));
                std::snprintf(lines[2], sizeof(lines[2]), "PWR%u VB%u", pwr_stat ? 1U : 0U, vbat_high ? 1U : 0U);
                std::snprintf(lines[3], sizeof(lines[3]), "Chem %s", chem_name);
                std::snprintf(lines[4], sizeof(lines[4]), "Gate %s", power_manager_.charger_enabled() ? "ON" : "OFF");
                std::snprintf(lines[5], sizeof(lines[5]), "%s", localized_(language_, "B2 back", "B2 返回"));
            }
            else
            {
                std::snprintf(lines[0], sizeof(lines[0]), "V%.2f %2.0f%%", sample.voltage_v, sample.state_of_charge_percent);
                std::snprintf(lines[1], sizeof(lines[1]), "I%+4.0f A%+4.0f", sample.current_ma, sample.average_current_ma);
                std::snprintf(lines[2], sizeof(lines[2]), "T%.1f Q%.0f", sample.temperature_c, sample.reported_capacity_mah);
                std::snprintf(lines[3], sizeof(lines[3]), "PWR%u VB%u St%04X", pwr_stat ? 1U : 0U, vbat_high ? 1U : 0U, sample.status);
                std::snprintf(lines[4], sizeof(lines[4]), "Chem %s Gate %s", chem_name, power_manager_.charger_enabled() ? "ON" : "OFF");
                std::snprintf(lines[5], sizeof(lines[5]), "%s", localized_(language_, "B2 back", "B2 返回"));
            }
            display_.show_test_menu(localized_(language_, "Battery", "电池"),
                                    nullptr,
                                    lines,
                                    DisplayService::kTestMenuMaxLines,
                                    DisplayService::kTestMenuNoCursor);
        }

        const ButtonEvent event = poll_event_(nullptr);
        if (event == ButtonEvent::Back)
        {
            return ESP_OK;
        }
        vTaskDelay(kPollIntervalTicks);
    }
}

esp_err_t HardwareTestApp::run_side_led_test_()
{
    ESP_LOGI(TAG, "Side LED test entered");
    // The strip has 4 LEDs total (index 0..3). LED 0 is the front status LED;
    // LEDs 1..3 are the side emitters per hardware mapping.
    constexpr uint8_t kSideStart = 1;
    constexpr uint8_t kSideEnd = 3;
    constexpr uint8_t kColorCount = 6;
    const uint8_t colors[kColorCount][3] = {
        {64, 0, 0},   // red
        {0, 64, 0},   // green
        {0, 0, 64},   // blue
        {48, 48, 0},  // yellow
        {32, 0, 48},  // magenta
        {0, 48, 48},  // cyan
    };
    uint8_t selected_led = kSideStart; // 1..3, plus 0xFE = "all"
    uint8_t color_index = 0;
    bool need_redraw = true;

    auto apply_strip = [&]() {
        for (uint8_t i = 0; i < StatusLed::pixel_count; ++i)
        {
            uint8_t r = 0;
            uint8_t g = 0;
            uint8_t b = 0;
            if (i >= kSideStart && i <= kSideEnd)
            {
                const bool target = (selected_led == 0xFE) || (i == selected_led);
                if (target)
                {
                    r = colors[color_index][0];
                    g = colors[color_index][1];
                    b = colors[color_index][2];
                }
            }
            status_led_.set_pixel(i, r, g, b);
        }
        status_led_.refresh();
    };

    apply_strip();

    char lines[DisplayService::kTestMenuMaxLines][DisplayService::kTestMenuLineLength] = {};
    while (true)
    {
        if (need_redraw)
        {
            std::snprintf(lines[0], sizeof(lines[0]), "%s", localized_(language_, "Side LEDs", "侧灯测试"));
            if (selected_led == 0xFE)
            {
                std::snprintf(lines[1], sizeof(lines[1]), "%s", localized_(language_, "LED: ALL", "灯: 全部"));
            }
            else
            {
                std::snprintf(lines[1], sizeof(lines[1]), "LED: %u", static_cast<unsigned>(selected_led));
            }
            const char *cn = "?";
            switch (color_index)
            {
            case 0: cn = "RED"; break;
            case 1: cn = "GREEN"; break;
            case 2: cn = "BLUE"; break;
            case 3: cn = "YELLOW"; break;
            case 4: cn = "MAGENTA"; break;
            case 5: cn = "CYAN"; break;
            default: cn = "?"; break;
            }
            std::snprintf(lines[2], sizeof(lines[2]), "Color: %s", cn);
            std::snprintf(lines[3], sizeof(lines[3]), "%s", localized_(language_, "X/OK next LED", "X或确认切灯"));
            std::snprintf(lines[4], sizeof(lines[4]), "%s", localized_(language_, "U/D color", "上下切色"));
            std::snprintf(lines[5], sizeof(lines[5]), "%s", localized_(language_, "B1 ALL B2 back", "B1 全部 B2 返回"));
            display_.show_test_menu(localized_(language_, "Side LEDs", "侧灯测试"),
                                    nullptr,
                                    lines,
                                    DisplayService::kTestMenuMaxLines,
                                    DisplayService::kTestMenuNoCursor);
            need_redraw = false;
        }

        const ButtonEvent event = poll_event_(nullptr);
        switch (event)
        {
        case ButtonEvent::Right:
        case ButtonEvent::Left:
        case ButtonEvent::Select:
            if (selected_led == 0xFE || selected_led >= kSideEnd)
            {
                selected_led = kSideStart;
            }
            else
            {
                selected_led = static_cast<uint8_t>(selected_led + 1);
            }
            apply_strip();
            need_redraw = true;
            break;
        case ButtonEvent::Up:
            color_index = (color_index == 0) ? (kColorCount - 1) : (color_index - 1);
            apply_strip();
            need_redraw = true;
            break;
        case ButtonEvent::Down:
            color_index = (color_index + 1) % kColorCount;
            apply_strip();
            need_redraw = true;
            break;
        case ButtonEvent::Action:
            selected_led = 0xFE; // ALL
            apply_strip();
            need_redraw = true;
            break;
        case ButtonEvent::Back:
            // Restore strip to a quiet state before leaving.
            for (uint8_t i = 0; i < StatusLed::pixel_count; ++i)
            {
                status_led_.set_pixel(i, 0, 0, 0);
            }
            status_led_.refresh();
            status_led_.set_mode(StatusLedMode::Linked);
            return ESP_OK;
        default:
            break;
        }

        vTaskDelay(kPollIntervalTicks);
    }
}
} // namespace prototracer

namespace
{
const char *menu_label(const prototracer::HardwareTestApp::MenuItem item, const prototracer::UiLanguage language)
{
    using prototracer::HardwareTestApp;
    using prototracer::UiLanguage;
    const bool zh = language == UiLanguage::Chinese;
    switch (item)
    {
    case HardwareTestApp::MenuItem::Imu:
        return zh ? "1 IMU" : "1 IMU";
    case HardwareTestApp::MenuItem::JoystickButtons:
        return zh ? "2 输入" : "2 Inputs";
    case HardwareTestApp::MenuItem::Gesture:
        return zh ? "3 手势" : "3 Gesture";
    case HardwareTestApp::MenuItem::Battery:
        return zh ? "4 电池" : "4 Battery";
    case HardwareTestApp::MenuItem::SideLeds:
        return zh ? "5 WS2812" : "5 WS2812";
    case HardwareTestApp::MenuItem::Resume:
        return zh ? "6 继续" : "6 Resume";
    default:
        return "?";
    }
}
} // namespace
