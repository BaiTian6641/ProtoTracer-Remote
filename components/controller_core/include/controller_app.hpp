#pragma once

#include "config_storage.hpp"
#include "connectivity_services.hpp"
#include "driver_services.hpp"
#include "esp_err.h"
#include "prototracer_types.hpp"

#include <cstddef>
#include <string>

namespace prototracer
{
class ControllerApp
{
public:
    esp_err_t start();
    const ResolvedConfig &active_config() const;

private:
    esp_err_t initialize_system_();
    esp_err_t initialize_services_();
    esp_err_t resolve_config_();
    esp_err_t run_runtime_loop_();
    esp_err_t refresh_active_config_();
    esp_err_t select_initial_main_board_();
    esp_err_t select_ble_candidate_(const BlePeerCandidate *candidates, size_t count, std::string *out_peer_id);
    esp_err_t discover_main_board_();
    esp_err_t bind_last_seen_main_board_();
    esp_err_t update_runtime_display_(bool force);
    esp_err_t send_control_payload_(const char *payload);
    esp_err_t send_current_control_();
    esp_err_t send_expression_control_();
    bool handle_input_event_(const InputEvent &event);
    bool handle_input_snapshot_(const InputSnapshot &snapshot);
    bool handle_sensor_event_(const SensorEvent &event);
    bool handle_motion_sample_(const MotionSample &sample);
    bool handle_gesture_sample_(const GestureSample &sample);
    bool detect_shake_pair_(const MotionSample &sample, uint32_t now);
    bool handle_joystick_sample_(const JoystickSample &sample);
    bool activate_current_selection_();
    bool handle_back_action_();
    bool navigate_current_view_(int delta);
    bool adjust_current_control_(int delta);
    bool perform_current_page_action_();
    bool refresh_battery_and_charger_(const char *ok_status, const char *error_status);
    bool apply_expression_shortcut_(uint8_t expression_index, const char *status_text);
    bool randomize_expression_();
    bool toggle_shake_random_();
    bool factory_reset_();
    bool cycle_oled_brightness_();
    bool cycle_oled_timeout_();
    bool wake_display_(uint32_t now);
    bool sleep_display_if_idle_(uint32_t now);
    void apply_display_settings_();
    void update_signal_strength_();
    void select_relative_page_(int delta);
    void sync_main_board_state_();

    ConfigStorage config_storage_;
    NetworkManager network_manager_;
    PairingService pairing_service_;
    RepoClient repo_client_;
    OtaService ota_service_;
    SensorHub sensor_hub_;
    InputRouter input_router_;
    PowerManager power_manager_;
    StatusLed status_led_;
    DisplayService display_service_;
    LowPowerController low_power_controller_;
    ResolvedConfig seed_config_{};
    ResolvedConfig active_config_{};
    InputSnapshot last_input_snapshot_{};
    MotionSample last_motion_{};
    GestureSample last_gesture_{};
    FuelGaugeSample last_battery_{};
    JoystickSample last_joystick_{};
    BatteryChemistry last_battery_chemistry_ = BatteryChemistry::Unknown;
    std::string motion_summary_;
    std::string gesture_summary_;
    std::string control_status_;
    uint8_t current_page_index_ = 0;
    uint8_t expression_index_ = 0;
    uint8_t expression_count_ = 17;
    uint8_t brightness_level_ = 105;
    uint8_t oled_brightness_ = 192;
    uint8_t settings_cursor_ = 0;
    bool voice_enabled_ = true;
    bool shake_random_enabled_ = true;
    bool detail_view_active_ = false;
    bool last_power_status_level_ = false;
    bool display_sleeping_ = false;
    uint16_t hue_shift_degrees_ = 0;
    uint16_t oled_timeout_seconds_ = 30;
    int8_t last_joystick_direction_ = 0;
    uint32_t last_ui_refresh_ms_ = 0;
    uint32_t last_activity_ms_ = 0;
    uint32_t last_battery_poll_ms_ = 0;
    uint32_t last_power_status_poll_ms_ = 0;
    uint32_t last_joystick_poll_ms_ = 0;
    uint32_t last_motion_poll_ms_ = 0;
    uint32_t last_motion_log_ms_ = 0;
    uint32_t last_gesture_poll_ms_ = 0;
    uint32_t last_gesture_log_ms_ = 0;
    uint32_t last_motion_action_ms_ = 0;
    uint32_t last_gesture_action_ms_ = 0;
    uint32_t last_shake_peak_ms_ = 0;
    uint32_t last_state_sync_ms_ = 0;
    int8_t last_shake_direction_ = 0;
    bool last_gesture_proximity_close_ = false;
};
} // namespace prototracer