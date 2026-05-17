#include "controller_app.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include "config_manifest.hpp"
#include "hardware_test_app.hpp"
#include "prototracer_board.hpp"

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_system.h"
#include "nvs_flash.h"

namespace
{
constexpr const char *TAG = "controller_app";
constexpr uint32_t kUiRefreshIntervalMs = 200;
constexpr uint32_t kBatteryPollIntervalMs = 5000;
constexpr uint32_t kPowerStatusPollIntervalMs = 250;
constexpr uint32_t kJoystickPollIntervalMs = 60;
constexpr uint32_t kMotionPollIntervalMs = 120;
constexpr uint32_t kMotionLogIntervalMs = 2000;
constexpr uint32_t kGesturePollIntervalMs = 160;
constexpr uint32_t kGestureLogIntervalMs = 2000;
constexpr uint32_t kGestureActionCooldownMs = 700;
constexpr uint32_t kMotionActionCooldownMs = 1500;
constexpr uint32_t kShakePairWindowMs = 550;
constexpr float kShakeDeltaThresholdMg = 560.0f;
constexpr float kShakeVerticalDominance = 1.35f;
constexpr uint16_t kProximitySurpriseThreshold = 1200;
constexpr uint16_t kProximitySurpriseMargin = 120;
constexpr char kColorMarker = '\x1E';
constexpr char kSwitchMarker = '\x1F';
constexpr uint8_t kRemoteBrightnessStep = 16;
constexpr uint16_t kRemoteHueStepDegrees = 12;
constexpr uint8_t kSliderSegments = 10;
constexpr uint8_t kSettingsMenuItems = 6;
constexpr uint8_t kFactoryResetSettingsIndex = 4;
constexpr uint8_t kOledBrightnessStep = 32;
constexpr uint8_t kOledBrightnessMinimum = 32;
constexpr uint8_t kSurpriseExpressionIndex = 9;

enum class RuntimePage : uint8_t
{
    Expression = 0,
    Brightness = 1,
    Hue = 2,
    Settings = 3,
    Battery = 4,
    Link = 5,
    Count = 6,
};

enum class MotionUiAction : uint8_t
{
    None = 0,
    Home = 1,
    Battery = 2,
    RandomExpression = 3,
};

enum class GestureUiAction : uint8_t
{
    None = 0,
    Previous = 1,
    Next = 2,
    Surprise = 3,
};

const char *localized(const prototracer::UiLanguage language, const char *english, const char *chinese)
{
    return language == prototracer::UiLanguage::Chinese ? chinese : english;
}

const char *default_expression_name(const uint8_t index)
{
    static constexpr const char *kNames[] = {
        "Default",
        "Angry",
        "Doubt",
        "Frown",
        "Heart",
        "Sad",
        "Surprise",
        "Happy",
        "OwO",
        "Surprise",
        "Sleepy",
        "Curious",
        "Excited",
        "Wink",
        "Shy",
        "Focus",
        "Custom",
    };

    if (index < (sizeof(kNames) / sizeof(kNames[0])))
    {
        return kNames[index];
    }
    return nullptr;
}

const char *expression_name_for(const prototracer::VisualConfig &visual, const uint8_t index)
{
    if (index < visual.expression_names.size() && !visual.expression_names[index].empty())
    {
        return visual.expression_names[index].c_str();
    }

    const char *fallback = default_expression_name(index);
    return fallback != nullptr ? fallback : "Expression";
}

std::string shorten_text(const std::string &value, const size_t limit)
{
    if (value.size() <= limit)
    {
        return value;
    }

    if (limit <= 3)
    {
        return value.substr(0, limit);
    }

    return value.substr(0, limit - 3) + "...";
}

const char *animation_name_for(const prototracer::VisualConfig &visual)
{
    if (!visual.animation_name.empty())
    {
        return visual.animation_name.c_str();
    }

    if (!visual.animation_asset.empty())
    {
        return visual.animation_asset.c_str();
    }

    return "Animation";
}

uint32_t now_ms()
{
    return static_cast<uint32_t>(esp_log_timestamp());
}

RuntimePage runtime_page_from_index(const uint8_t index)
{
    switch (index)
    {
    case 1:
        return RuntimePage::Brightness;
    case 2:
        return RuntimePage::Hue;
    case 3:
        return RuntimePage::Settings;
    case 4:
        return RuntimePage::Battery;
    case 5:
        return RuntimePage::Link;
    case 0:
    default:
        return RuntimePage::Expression;
    }
}

const char *runtime_page_title(const RuntimePage page, const prototracer::UiLanguage language)
{
    switch (page)
    {
    case RuntimePage::Brightness:
        return localized(language, "Bright", "亮度");
    case RuntimePage::Hue:
        return localized(language, "Hue", "色相");
    case RuntimePage::Settings:
        return localized(language, "Settings", "设置");
    case RuntimePage::Battery:
        return localized(language, "Battery", "电池");
    case RuntimePage::Link:
        return localized(language, "Link", "连接");
    case RuntimePage::Expression:
    default:
        return localized(language, "Expr", "表情");
    }
}

void build_slider_bar(char *buffer, const size_t buffer_size, const int value, const int minimum, const int maximum)
{
    if (buffer == nullptr || buffer_size == 0)
    {
        return;
    }

    if (buffer_size < (kSliderSegments + 3))
    {
        buffer[0] = '\0';
        return;
    }

    const int clamped = std::clamp(value, minimum, maximum);
    const int range = std::max(1, maximum - minimum);
    const int filled = ((clamped - minimum) * kSliderSegments + (range / 2)) / range;

    buffer[0] = '[';
    for (uint8_t index = 0; index < kSliderSegments; ++index)
    {
        buffer[index + 1] = index < filled ? '#' : '-';
    }
    buffer[kSliderSegments + 1] = ']';
    buffer[kSliderSegments + 2] = '\0';
}

std::string shorten_peer_id(const std::string &peer_id)
{
    if (peer_id.empty())
    {
        return "Unbound";
    }

    constexpr size_t kMaxPeerChars = 12;
    return peer_id.size() <= kMaxPeerChars ? peer_id : peer_id.substr(0, kMaxPeerChars);
}

void format_ble_candidate_label(const prototracer::BlePeerCandidate &candidate, char *buffer, const size_t buffer_size)
{
    if (buffer == nullptr || buffer_size == 0)
    {
        return;
    }

    std::string name = candidate.display_name.empty() ? candidate.peer_id : candidate.display_name;
    if (name.size() > 14)
    {
        name = name.substr(0, 14);
    }
    std::snprintf(buffer, buffer_size, "%s %u%%", name.c_str(), static_cast<unsigned>(candidate.signal_percent));
}

uint32_t rgb888(const uint8_t red, const uint8_t green, const uint8_t blue)
{
    return (static_cast<uint32_t>(red) << 16) | (static_cast<uint32_t>(green) << 8) | blue;
}

uint32_t hue_shift_rgb888(const prototracer::VisualConfig &visual, const uint16_t hue_shift_degrees)
{
    const float red = static_cast<float>(visual.red) / 255.0f;
    const float green = static_cast<float>(visual.green) / 255.0f;
    const float blue = static_cast<float>(visual.blue) / 255.0f;
    const float maximum = std::max(red, std::max(green, blue));
    const float minimum = std::min(red, std::min(green, blue));
    const float delta = maximum - minimum;
    float hue = 0.0f;
    if (delta > 0.0001f)
    {
        if (maximum == red)
        {
            hue = 60.0f * std::fmod(((green - blue) / delta), 6.0f);
        }
        else if (maximum == green)
        {
            hue = 60.0f * (((blue - red) / delta) + 2.0f);
        }
        else
        {
            hue = 60.0f * (((red - green) / delta) + 4.0f);
        }
    }
    if (hue < 0.0f)
    {
        hue += 360.0f;
    }
    hue = std::fmod(hue + static_cast<float>(hue_shift_degrees), 360.0f);

    const float chroma = delta;
    const float x = chroma * (1.0f - std::fabs(std::fmod(hue / 60.0f, 2.0f) - 1.0f));
    const float match = maximum - chroma;
    float shifted_red = 0.0f;
    float shifted_green = 0.0f;
    float shifted_blue = 0.0f;
    if (hue < 60.0f)
    {
        shifted_red = chroma;
        shifted_green = x;
    }
    else if (hue < 120.0f)
    {
        shifted_red = x;
        shifted_green = chroma;
    }
    else if (hue < 180.0f)
    {
        shifted_green = chroma;
        shifted_blue = x;
    }
    else if (hue < 240.0f)
    {
        shifted_green = x;
        shifted_blue = chroma;
    }
    else if (hue < 300.0f)
    {
        shifted_red = x;
        shifted_blue = chroma;
    }
    else
    {
        shifted_red = chroma;
        shifted_blue = x;
    }

    const uint8_t out_red = static_cast<uint8_t>(std::clamp<int>(static_cast<int>((shifted_red + match) * 255.0f + 0.5f), 0, 255));
    const uint8_t out_green = static_cast<uint8_t>(std::clamp<int>(static_cast<int>((shifted_green + match) * 255.0f + 0.5f), 0, 255));
    const uint8_t out_blue = static_cast<uint8_t>(std::clamp<int>(static_cast<int>((shifted_blue + match) * 255.0f + 0.5f), 0, 255));
    return rgb888(out_red, out_green, out_blue);
}

void format_color_swatch_line(char *buffer, const size_t buffer_size, const char *label, const uint32_t rgb)
{
    if (buffer == nullptr || buffer_size == 0)
    {
        return;
    }
    std::snprintf(buffer, buffer_size, "%s%c%06X", label, kColorMarker, static_cast<unsigned>(rgb & 0xFFFFFFU));
}

const char *battery_label(const prototracer::BatteryChemistry chemistry, const prototracer::UiLanguage language)
{
    switch (chemistry)
    {
    case prototracer::BatteryChemistry::NiMH:
        return localized(language, "NiMH", "镍氢");
    case prototracer::BatteryChemistry::Alkaline:
        return localized(language, "Alkaline", "碱性");
    case prototracer::BatteryChemistry::Unknown:
    default:
        return localized(language, "Unknown", "未知");
    }
}

void format_settings_item_label(const uint8_t index,
                                const prototracer::UiLanguage language,
                                const bool voice_enabled,
                                const bool shake_random_enabled,
                                const uint8_t oled_brightness,
                                const uint16_t oled_timeout_seconds,
                                char *buffer,
                                const size_t buffer_size)
{
    if (buffer == nullptr || buffer_size == 0)
    {
        return;
    }

    switch (index)
    {
    case 0:
        std::snprintf(buffer,
                      buffer_size,
                      "%s%c%c",
                      localized(language, "Voice", "语音"),
                      kSwitchMarker,
                      voice_enabled ? '1' : '0');
        break;
    case 1:
        std::snprintf(buffer,
                      buffer_size,
                      "%s%c%c",
                      localized(language, "Shake", "摇动"),
                      kSwitchMarker,
                      shake_random_enabled ? '1' : '0');
        break;
    case 2:
        std::snprintf(buffer, buffer_size, "OLED %u%%", static_cast<unsigned>((static_cast<unsigned>(oled_brightness) * 100U) / 255U));
        break;
    case 3:
        if (oled_timeout_seconds == 0)
        {
            std::snprintf(buffer, buffer_size, "%s", localized(language, "Timeout off", "息屏 关"));
        }
        else
        {
            std::snprintf(buffer, buffer_size, "Timeout %us", static_cast<unsigned>(oled_timeout_seconds));
        }
        break;
    case 4:
        std::snprintf(buffer, buffer_size, "%s", localized(language, "Factory reset", "恢复出厂"));
        break;
    case 5:
    default:
        std::snprintf(buffer, buffer_size, "%s", localized(language, "< Back", "< 返回"));
        break;
    }
}

void preserve_dynamic_seed_values(prototracer::ResolvedConfig *candidate, const prototracer::ResolvedConfig &current)
{
    if (candidate == nullptr)
    {
        return;
    }

    if (candidate->controller.network.station_ssid.empty())
    {
        candidate->controller.network.station_ssid = current.controller.network.station_ssid;
    }
    if (candidate->controller.network.station_password.empty())
    {
        candidate->controller.network.station_password = current.controller.network.station_password;
    }
    if (candidate->controller.pairing.bound_peer_id.empty())
    {
        candidate->controller.pairing.bound_peer_id = current.controller.pairing.bound_peer_id;
    }
    if (candidate->controller.visual.expression_names.empty())
    {
        candidate->controller.visual.expression_names = current.controller.visual.expression_names;
    }
        if (candidate->controller.visual.animation_name.empty())
        {
            candidate->controller.visual.animation_name = current.controller.visual.animation_name;
        }
    candidate->controller.display = current.controller.display;
    candidate->controller.features.enable_shake_random = current.controller.features.enable_shake_random;
}

bool gesture_surprise_proximity(const prototracer::GestureSample &sample)
{
    const bool center_flag = sample.proximity_close &&
                             sample.proximity_2 >= sample.proximity_1 &&
                             sample.proximity_2 >= sample.proximity_3;
    const uint32_t center = sample.proximity_2;
    const bool center_raw = center >= kProximitySurpriseThreshold &&
                            center >= static_cast<uint32_t>(sample.proximity_1) + kProximitySurpriseMargin &&
                            center >= static_cast<uint32_t>(sample.proximity_3) + kProximitySurpriseMargin;
    return center_flag || center_raw;
}

MotionUiAction summarize_motion(const prototracer::MotionSample &sample,
                                const prototracer::UiLanguage language,
                                char *buffer,
                                const size_t buffer_size)
{
    if (buffer == nullptr || buffer_size == 0)
    {
        return MotionUiAction::None;
    }

    if (!sample.valid)
    {
        std::snprintf(buffer, buffer_size, "%s", localized(language, "Awaiting IMU", "等待 IMU"));
        return MotionUiAction::None;
    }

    const float abs_x = std::fabs(sample.x_mg);
    const float abs_y = std::fabs(sample.y_mg);
    const float abs_z = std::fabs(sample.z_mg);

    if (sample.wake_event && abs_y > 360.0f && abs_y >= (abs_x * kShakeVerticalDominance) && abs_y >= abs_z)
    {
        std::snprintf(buffer, buffer_size, "%s", localized(language, "Shake random", "摇动随机"));
        return MotionUiAction::RandomExpression;
    }
    if (sample.wake_event && abs_x >= abs_y && abs_x >= abs_z && abs_x > 360.0f)
    {
        std::snprintf(buffer, buffer_size, "%s", localized(language, "Horizontal shake", "水平摇动"));
        return MotionUiAction::None;
    }
    if (sample.wake_event && abs_z > 280.0f)
    {
        std::snprintf(buffer, buffer_size, "%s", localized(language, "Lift or tap", "抬起或轻敲"));
        return MotionUiAction::None;
    }

    std::snprintf(buffer, buffer_size, "X %.0f Y %.0f Z %.0f", sample.x_mg, sample.y_mg, sample.z_mg);
    return MotionUiAction::None;
}

GestureUiAction summarize_gesture(const prototracer::GestureSample &sample,
                                  const prototracer::UiLanguage language,
                                  char *buffer,
                                  const size_t buffer_size)
{
    if (buffer == nullptr || buffer_size == 0)
    {
        return GestureUiAction::None;
    }

    if (!sample.valid)
    {
        std::snprintf(buffer, buffer_size, "%s", localized(language, "Awaiting sensor", "等待传感器"));
        return GestureUiAction::None;
    }

    const uint16_t strongest = std::max(sample.proximity_1, std::max(sample.proximity_2, sample.proximity_3));
    if ((sample.gesture_ready || sample.proximity_close) && sample.proximity_1 == strongest && sample.proximity_1 > sample.proximity_3 + 180)
    {
        std::snprintf(buffer, buffer_size, "%s", localized(language, "Swipe left", "左滑"));
        return GestureUiAction::Previous;
    }
    if ((sample.gesture_ready || sample.proximity_close) && sample.proximity_3 == strongest && sample.proximity_3 > sample.proximity_1 + 180)
    {
        std::snprintf(buffer, buffer_size, "%s", localized(language, "Swipe right", "右滑"));
        return GestureUiAction::Next;
    }
    if (gesture_surprise_proximity(sample))
    {
        std::snprintf(buffer, buffer_size, "%s", localized(language, "Surprise", "惊喜"));
        return GestureUiAction::Surprise;
    }
    if (sample.proximity_away)
    {
        std::snprintf(buffer, buffer_size, "%s", localized(language, "Hand away", "手离开"));
        return GestureUiAction::None;
    }
    if (sample.ambient_low)
    {
        std::snprintf(buffer, buffer_size, "%s", localized(language, "Ambient low", "环境偏暗"));
        return GestureUiAction::None;
    }
    if (sample.ambient_high)
    {
        std::snprintf(buffer, buffer_size, "%s", localized(language, "Ambient high", "环境偏亮"));
        return GestureUiAction::None;
    }

    std::snprintf(buffer, buffer_size, "P1 %u P2 %u P3 %u", sample.proximity_1, sample.proximity_2, sample.proximity_3);
    return GestureUiAction::None;
}

const char *ready_detail_for_source(const prototracer::ConfigSourceKind source, const prototracer::UiLanguage language)
{
    switch (source)
    {
    case prototracer::ConfigSourceKind::MainBoard:
        return localized(language, "Main board", "主板");
    case prototracer::ConfigSourceKind::RemoteRepo:
        return localized(language, "Remote repo", "远程仓库");
    case prototracer::ConfigSourceKind::FilesystemImage:
        return localized(language, "Local image", "本地镜像");
    case prototracer::ConfigSourceKind::Failsafe:
    default:
        return localized(language, "Failsafe", "安全回退");
    }
}
} // namespace

namespace prototracer
{
const ResolvedConfig &ControllerApp::active_config() const
{
    return active_config_;
}

esp_err_t ControllerApp::start()
{
    ESP_RETURN_ON_ERROR(initialize_system_(), TAG, "System initialization failed");
    ESP_RETURN_ON_ERROR(initialize_services_(), TAG, "Service initialization failed");

    InputSnapshot startup_snapshot = {};
    const bool run_hardware_test = input_router_.get_snapshot(&startup_snapshot) == ESP_OK
                                   && startup_snapshot.is_asserted(board::ExpanderInputBit::Button2);
    ESP_LOGI(TAG, "Runtime stage: hardware test UI %s", run_hardware_test ? "requested by B2" : "skipped");
    if (run_hardware_test)
    {
        HardwareTestApp test_app(display_service_,
                                 status_led_,
                                 input_router_,
                                 sensor_hub_,
                                 power_manager_,
                                 seed_config_.controller.ui_language);
        const esp_err_t test_err = test_app.run();
        if (test_err != ESP_OK)
        {
            ESP_LOGW(TAG, "Hardware test UI exited with %s", esp_err_to_name(test_err));
        }
    }

    ESP_LOGI(TAG, "Runtime stage: resolve config");
    ESP_RETURN_ON_ERROR(resolve_config_(), TAG, "Configuration resolution failed");

    ESP_LOGI(TAG, "Runtime stage: OTA service init");
    ESP_RETURN_ON_ERROR(ota_service_.init(active_config_.controller), TAG, "OTA init failed");
    expression_count_ = std::max<uint8_t>(1, active_config_.controller.visual.expression_count);
    if (expression_index_ >= expression_count_)
    {
        expression_index_ = static_cast<uint8_t>(expression_count_ - 1);
    }
    apply_display_settings_();
    shake_random_enabled_ = active_config_.controller.features.enable_shake_random;

    if (!network_manager_.station_connected())
    {
        ESP_LOGI(TAG, "Runtime stage: connect saved Wi-Fi for relay update");
        (void)display_service_.show_update_progress(localized(active_config_.controller.ui_language, "Wi-Fi link", "连接 Wi-Fi"), 20);
        const esp_err_t station_err = network_manager_.connect_saved_station(active_config_.controller.network, 10000);
        if (station_err != ESP_OK)
        {
            ESP_LOGW(TAG, "Saved station connection for relay update is unavailable: %s", esp_err_to_name(station_err));
        }
    }

    if (network_manager_.station_connected())
    {
        ESP_LOGI(TAG, "Runtime stage: relay OTA check");
        (void)status_led_.set_mode(StatusLedMode::Updating);
        (void)display_service_.show_update_progress(localized(active_config_.controller.ui_language, "Relay update", "中继更新"), 55);
        const esp_err_t update_err = ota_service_.check_for_relay_update();
        if (update_err != ESP_OK && update_err != ESP_ERR_NOT_FOUND)
        {
            ESP_LOGW(TAG, "Relay OTA check did not complete: %s", esp_err_to_name(update_err));
        }
        (void)status_led_.set_mode(StatusLedMode::Linked);
    }

    ESP_LOGI(TAG, "Runtime stage: local update server");
    ESP_RETURN_ON_ERROR(ota_service_.start_local_update_server(active_config_.controller), TAG, "Local update server failed");
    ESP_LOGI(TAG, "Runtime stage: low power init");
    ESP_RETURN_ON_ERROR(low_power_controller_.init(active_config_.controller.features.enable_low_power_core), TAG, "LP core init failed");
    ESP_LOGI(TAG, "Runtime stage: persist active config");
    ESP_RETURN_ON_ERROR(config_storage_.persist_active_config(active_config_), TAG, "Failed to persist active config");

    (void)sensor_hub_.get_latest_motion(&last_motion_);
    (void)sensor_hub_.get_latest_gesture(&last_gesture_);
    (void)sensor_hub_.get_latest_battery(&last_battery_);
    last_battery_chemistry_ = power_manager_.classify_battery_chemistry();

    char summary[80] = {};
    summarize_motion(last_motion_, active_config_.controller.ui_language, summary, sizeof(summary));
    motion_summary_ = summary;
    summarize_gesture(last_gesture_, active_config_.controller.ui_language, summary, sizeof(summary));
    gesture_summary_ = summary;

    ESP_LOGI(TAG, "Controller app ready using config source: %s", config_source_name(active_config_.source));
    return run_runtime_loop_();
}

esp_err_t ControllerApp::initialize_system_()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(err, TAG, "NVS init failed");

    board::log_summary();
    return ESP_OK;
}

esp_err_t ControllerApp::initialize_services_()
{
    ESP_LOGI(TAG, "Boot stage: status LED");
    ESP_RETURN_ON_ERROR(status_led_.init(), TAG, "LED init failed");
    ESP_RETURN_ON_ERROR(status_led_.set_mode(StatusLedMode::Booting), TAG, "LED boot state failed");

    ESP_LOGI(TAG, "Boot stage: config storage");
    ESP_RETURN_ON_ERROR(config_storage_.init(), TAG, "Storage init failed");

    ESP_LOGI(TAG, "Boot stage: filesystem image config");
    ESP_RETURN_ON_ERROR(config_storage_.load_filesystem_image_config(&seed_config_), TAG, "Filesystem image config load failed");
    display_service_.set_language(seed_config_.controller.ui_language);

    ESP_LOGI(TAG, "Boot stage: display init");
    ESP_RETURN_ON_ERROR(display_service_.init(), TAG, "Display init failed");

    const auto show_boot_stage = [this](const char *english, const char *chinese) -> esp_err_t {
        const UiLanguage language = seed_config_.controller.ui_language;
        ESP_LOGI(TAG, "Boot stage: %s", english);
        return display_service_.show_status(
            localized(language, "Boot", "启动"),
            localized(language, english, chinese),
            StatusLedMode::Booting);
    };

    ESP_RETURN_ON_ERROR(
        display_service_.show_status(
            localized(seed_config_.controller.ui_language, "Boot", "启动"),
            localized(seed_config_.controller.ui_language, "Initialize", "初始化"),
            StatusLedMode::Booting),
        TAG,
        "Display boot state failed");
    ESP_RETURN_ON_ERROR(show_boot_stage("Network", "网络"), TAG, "Display network boot stage failed");
    ESP_RETURN_ON_ERROR(network_manager_.init(), TAG, "Network init failed");
    ESP_RETURN_ON_ERROR(show_boot_stage("Pairing", "配对"), TAG, "Display pairing boot stage failed");
    ESP_RETURN_ON_ERROR(pairing_service_.init(), TAG, "Pairing init failed");
    ESP_RETURN_ON_ERROR(show_boot_stage("Repo", "仓库"), TAG, "Display repo boot stage failed");
    ESP_RETURN_ON_ERROR(repo_client_.init(), TAG, "Repo client init failed");
    ESP_RETURN_ON_ERROR(show_boot_stage("Sensors", "传感器"), TAG, "Display sensor boot stage failed");
    ESP_RETURN_ON_ERROR(sensor_hub_.init(), TAG, "Sensor init failed");
    ESP_RETURN_ON_ERROR(show_boot_stage("Inputs", "输入"), TAG, "Display input boot stage failed");
    ESP_RETURN_ON_ERROR(input_router_.init(), TAG, "Input init failed");
    ESP_RETURN_ON_ERROR(sensor_hub_.attach_input_router(&input_router_), TAG, "Sensor event attach failed");
    ESP_RETURN_ON_ERROR(show_boot_stage("Power", "电源"), TAG, "Display power boot stage failed");
    ESP_RETURN_ON_ERROR(power_manager_.init(), TAG, "Power init failed");

    const bool sensor_rail_enabled = gpio_get_level(board::kPinSensorRailEnable) != 0;
    ESP_LOGI(TAG, "Boot services ready: SENSOR_EN=%d", sensor_rail_enabled ? 1 : 0);
    power_manager_.log_hardware_limits();
    return ESP_OK;
}

esp_err_t ControllerApp::resolve_config_()
{
    ResolvedConfig refreshed_seed = seed_config_;
    if (config_storage_.load_filesystem_image_config(&refreshed_seed) == ESP_OK)
    {
        preserve_dynamic_seed_values(&refreshed_seed, seed_config_);
        seed_config_ = refreshed_seed;
    }

    active_config_ = seed_config_;
    const UiLanguage seed_language = seed_config_.controller.ui_language;
    ESP_LOGI(TAG, "Resolve config: seed source=%s", config_source_name(active_config_.source));

    ResolvedConfig candidate = {};
    const bool first_ble_pairing = seed_config_.controller.pairing.bound_peer_id.empty();
    if (first_ble_pairing)
    {
        const esp_err_t select_err = select_initial_main_board_();
        if (select_err != ESP_OK)
        {
            ESP_LOGW(TAG, "First-boot main-board selection unavailable: %s", esp_err_to_name(select_err));
        }
    }

    if (!seed_config_.controller.pairing.bound_peer_id.empty())
    {
        ESP_LOGI(TAG, "Resolve config: connect selected main board");
        ESP_RETURN_ON_ERROR(display_service_.show_bind_progress(localized(seed_language, "Connect board", "连接主板"), 35), TAG, "Display pairing state failed");
        if (pairing_service_.pull_from_main_board(seed_config_.controller, &candidate) == ESP_OK)
        {
            preserve_dynamic_seed_values(&candidate, seed_config_);
            active_config_ = candidate;
            seed_config_ = active_config_;
            display_service_.set_language(active_config_.controller.ui_language);
            ESP_RETURN_ON_ERROR(status_led_.set_mode(StatusLedMode::Linked), TAG, "LED linked state failed");
            ESP_RETURN_ON_ERROR(
                display_service_.show_status(
                    localized(active_config_.controller.ui_language, "Ready", "就绪"),
                    ready_detail_for_source(active_config_.source, active_config_.controller.ui_language),
                    StatusLedMode::Linked),
                TAG,
                "Display ready state failed");
            return ESP_OK;
        }

        if (!first_ble_pairing)
        {
            (void)bind_last_seen_main_board_();
        }
    }

    ESP_LOGI(TAG, "Resolve config: connect saved Wi-Fi");
    ESP_RETURN_ON_ERROR(display_service_.show_bind_progress(localized(seed_language, "Wi-Fi link", "连接 Wi-Fi"), 45), TAG, "Display Wi-Fi state failed");
    const esp_err_t station_err = network_manager_.connect_saved_station(seed_config_.controller.network, 10000);
    if (station_err == ESP_OK)
    {
        ESP_LOGI(TAG, "Resolve config: fetch repo manifest");
        if (repo_client_.pull_from_repo(seed_config_.controller, &candidate) == ESP_OK)
        {
            preserve_dynamic_seed_values(&candidate, seed_config_);
            active_config_ = candidate;
            seed_config_ = active_config_;
            display_service_.set_language(active_config_.controller.ui_language);
            ESP_RETURN_ON_ERROR(status_led_.set_mode(StatusLedMode::Linked), TAG, "LED linked state failed");
            ESP_RETURN_ON_ERROR(
                display_service_.show_status(
                    localized(active_config_.controller.ui_language, "Ready", "就绪"),
                    ready_detail_for_source(active_config_.source, active_config_.controller.ui_language),
                    StatusLedMode::Linked),
                TAG,
                "Display repo-ready state failed");
            return ESP_OK;
        }
    }

    ESP_LOGW(TAG, "Falling back to filesystem image configuration");
    ESP_RETURN_ON_ERROR(
        display_service_.show_error(
            localized(seed_language, "Config", "配置"),
            localized(seed_language, "Remote source missing", "远程配置缺失"),
            localized(seed_language, "Filesystem fallback", "回退到文件系统配置")),
        TAG,
        "Display fallback state failed");
    ESP_RETURN_ON_ERROR(network_manager_.start_user_provisioning_portal(seed_config_.controller), TAG, "Provisioning portal start failed");
    ESP_RETURN_ON_ERROR(status_led_.set_mode(StatusLedMode::Provisioning), TAG, "LED provisioning state failed");
    ESP_RETURN_ON_ERROR(
        display_service_.show_provisioning(
            seed_config_.controller.network.provisioning_ap_prefix.empty() ? "PROTO-REMOTE" : seed_config_.controller.network.provisioning_ap_prefix.c_str(),
            localized(seed_language, "Join AP and load portal", "连接热点并打开门户"),
            100),
        TAG,
        "Display provisioning state failed");
    return ESP_OK;
}

esp_err_t ControllerApp::refresh_active_config_()
{
    const esp_err_t err = resolve_config_();
    if (err != ESP_OK)
    {
        const UiLanguage language = active_config_.controller.ui_language;
        status_led_.set_mode(StatusLedMode::Error);
        display_service_.show_error(
            localized(language, "Refresh", "刷新"),
            esp_err_to_name(err),
            localized(language, "Check source availability", "检查数据源是否可用"));
        return err;
    }

    apply_display_settings_();
    shake_random_enabled_ = active_config_.controller.features.enable_shake_random;
    return config_storage_.persist_active_config(active_config_);
}

esp_err_t ControllerApp::select_initial_main_board_()
{
    if (!seed_config_.controller.pairing.bound_peer_id.empty())
    {
        return ESP_OK;
    }

    const UiLanguage language = seed_config_.controller.ui_language;
    ESP_RETURN_ON_ERROR(display_service_.show_bind_progress(localized(language, "BLE scan", "蓝牙扫描"), 25), TAG, "Display BLE scan state failed");

    BlePeerCandidate candidates[DisplayService::kTestMenuMaxLines] = {};
    size_t candidate_count = 0;
    const esp_err_t scan_err = pairing_service_.scan_main_boards(seed_config_.controller, candidates, DisplayService::kTestMenuMaxLines, &candidate_count);
    if (scan_err != ESP_OK || candidate_count == 0)
    {
        ESP_LOGW(TAG, "No selectable ProtoTracer BLE main boards found: %s", esp_err_to_name(scan_err));
        (void)display_service_.show_error(
            localized(language, "Pairing", "配对"),
            localized(language, "No board found", "未找到主板"),
            localized(language, "Check main board power", "检查主板电源"));
        vTaskDelay(pdMS_TO_TICKS(1200));
        return scan_err == ESP_OK ? ESP_ERR_NOT_FOUND : scan_err;
    }

    std::string peer_id;
    const esp_err_t select_err = select_ble_candidate_(candidates, candidate_count, &peer_id);
    if (select_err != ESP_OK)
    {
        ESP_LOGW(TAG, "Main-board BLE selection cancelled: %s", esp_err_to_name(select_err));
        return select_err;
    }

    seed_config_.controller.pairing.bound_peer_id = peer_id;
    active_config_.controller.pairing.bound_peer_id = peer_id;
    ESP_LOGI(TAG, "User selected ProtoTracer BLE main board: %s", peer_id.c_str());
    return config_storage_.persist_active_config(active_config_);
}

esp_err_t ControllerApp::select_ble_candidate_(const BlePeerCandidate *candidates, const size_t count, std::string *out_peer_id)
{
    if (candidates == nullptr || count == 0 || out_peer_id == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }

    const UiLanguage language = seed_config_.controller.ui_language;
    uint8_t selected_index = 0;
    int8_t last_direction = 0;
    bool redraw = true;
    uint8_t previous_bits = 0xFF;
    InputSnapshot snapshot = {};
    if (input_router_.get_snapshot(&snapshot) == ESP_OK)
    {
        previous_bits = snapshot.raw_bits;
    }

    while (true)
    {
        if (redraw)
        {
            char lines[DisplayService::kTestMenuMaxLines][DisplayService::kTestMenuLineLength] = {};
            const uint8_t line_count = static_cast<uint8_t>(std::min<size_t>(count, DisplayService::kTestMenuMaxLines));
            for (uint8_t index = 0; index < line_count; ++index)
            {
                format_ble_candidate_label(candidates[index], lines[index], sizeof(lines[index]));
            }
            ESP_RETURN_ON_ERROR(
                display_service_.show_test_menu(
                    localized(language, "Select BLE", "选择蓝牙"),
                    localized(language, "Stick move B1 choose", "摇杆移动 B1选择"),
                    lines,
                    line_count,
                    selected_index),
                TAG,
                "Display BLE candidate list failed");
            redraw = false;
        }

        InputEvent event = {};
        while (input_router_.wait_for_event(&event, pdMS_TO_TICKS(40)) == ESP_OK)
        {
            if (event.type != InputEvent::Type::ButtonPressed)
            {
                continue;
            }
            if (event.source == board::ExpanderInputBit::Button2)
            {
                return ESP_ERR_INVALID_RESPONSE;
            }
            if (event.source == board::ExpanderInputBit::JoystickButton ||
                event.source == board::ExpanderInputBit::Button0 ||
                event.source == board::ExpanderInputBit::Button1)
            {
                *out_peer_id = candidates[selected_index].peer_id;
                return ESP_OK;
            }
        }

        if (input_router_.get_snapshot(&snapshot) == ESP_OK)
        {
            const auto is_pressed = [&snapshot](const board::ExpanderInputBit bit) {
                return snapshot.is_asserted(bit);
            };
            const auto was_pressed = [previous_bits](const board::ExpanderInputBit bit) {
                return (previous_bits & (1U << static_cast<uint8_t>(bit))) == 0;
            };

            const bool select_pressed = (is_pressed(board::ExpanderInputBit::JoystickButton) && !was_pressed(board::ExpanderInputBit::JoystickButton)) ||
                                        (is_pressed(board::ExpanderInputBit::Button0) && !was_pressed(board::ExpanderInputBit::Button0)) ||
                                        (is_pressed(board::ExpanderInputBit::Button1) && !was_pressed(board::ExpanderInputBit::Button1));
            if (select_pressed)
            {
                *out_peer_id = candidates[selected_index].peer_id;
                return ESP_OK;
            }
            if (is_pressed(board::ExpanderInputBit::Button2) && !was_pressed(board::ExpanderInputBit::Button2))
            {
                return ESP_ERR_INVALID_RESPONSE;
            }
            previous_bits = snapshot.raw_bits;
        }

        JoystickSample joystick = {};
        if (input_router_.read_joystick(&joystick) == ESP_OK && joystick.valid)
        {
            int8_t direction = 0;
            if (joystick.normalized_x >= 35)
            {
                direction = 1;
            }
            else if (joystick.normalized_x <= -35)
            {
                direction = -1;
            }

            if (direction != 0 && direction != last_direction)
            {
                const int next = (static_cast<int>(selected_index) + direction + static_cast<int>(count)) % static_cast<int>(count);
                selected_index = static_cast<uint8_t>(next);
                redraw = true;
            }
            last_direction = direction;
        }
    }
}

esp_err_t ControllerApp::bind_last_seen_main_board_()
{
    std::string bound_peer_id;
    if (!pairing_service_.get_last_main_board_binding(&bound_peer_id) || bound_peer_id.empty())
    {
        return ESP_ERR_NOT_FOUND;
    }

    active_config_.controller.pairing.bound_peer_id = bound_peer_id;
    seed_config_.controller.pairing.bound_peer_id = bound_peer_id;
    ESP_LOGI(TAG, "Bound main board peer for future direct connect: %s", bound_peer_id.c_str());
    return config_storage_.persist_active_config(active_config_);
}

esp_err_t ControllerApp::discover_main_board_()
{
    ResolvedConfig candidate = {};
    seed_config_.controller.pairing.bound_peer_id.clear();
    active_config_.controller.pairing.bound_peer_id.clear();

    ESP_RETURN_ON_ERROR(select_initial_main_board_(), TAG, "Main-board selection failed");
    ESP_RETURN_ON_ERROR(display_service_.show_bind_progress(localized(active_config_.controller.ui_language, "Connect board", "连接主板"), 55), TAG, "Display discover state failed");

    const esp_err_t err = pairing_service_.pull_from_main_board(seed_config_.controller, &candidate);
    if (err != ESP_OK)
    {
        return err;
    }

    preserve_dynamic_seed_values(&candidate, active_config_);
    active_config_ = candidate;
    seed_config_ = active_config_;
    expression_count_ = std::max<uint8_t>(1, active_config_.controller.visual.expression_count);
    if (expression_index_ >= expression_count_)
    {
        expression_index_ = static_cast<uint8_t>(expression_count_ - 1);
    }
    apply_display_settings_();
    display_service_.set_language(active_config_.controller.ui_language);
    ESP_RETURN_ON_ERROR(status_led_.set_mode(StatusLedMode::Linked), TAG, "LED linked state failed");
    return config_storage_.persist_active_config(active_config_);
}

void ControllerApp::select_relative_page_(const int delta)
{
    constexpr int kPageCount = static_cast<int>(RuntimePage::Count);
    int next = static_cast<int>(current_page_index_) + delta;
    next %= kPageCount;
    if (next < 0)
    {
        next += kPageCount;
    }
    current_page_index_ = static_cast<uint8_t>(next);
}

esp_err_t ControllerApp::send_control_payload_(const char *payload)
{
    if (payload == nullptr || payload[0] == '\0')
    {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Remote control payload: %s", payload);
    esp_err_t err = pairing_service_.send_control_command(payload);
    if (err == ESP_ERR_INVALID_STATE)
    {
        control_status_ = "Relink";
        const esp_err_t relink_err = discover_main_board_();
        if (relink_err == ESP_OK)
        {
            err = pairing_service_.send_control_command(payload);
        }
        else
        {
            err = relink_err;
        }
    }

    control_status_ = err == ESP_OK ? "Sent" : esp_err_to_name(err);
    ESP_LOGI(TAG, "Remote control send result: %s", control_status_.c_str());
    if (err == ESP_OK)
    {
        (void)status_led_.set_mode(StatusLedMode::Linked);
    }
    return err;
}

esp_err_t ControllerApp::send_expression_control_()
{
    char payload[96] = {};
    std::snprintf(payload, sizeof(payload), "{\"op\":\"control.set\",\"expression\":%u}", expression_index_);
    return send_control_payload_(payload);
}

esp_err_t ControllerApp::send_current_control_()
{
    char payload[96] = {};

    switch (runtime_page_from_index(current_page_index_))
    {
    case RuntimePage::Expression:
        std::snprintf(payload, sizeof(payload), "{\"op\":\"control.set\",\"expression\":%u}", expression_index_);
        break;
    case RuntimePage::Hue:
        std::snprintf(payload, sizeof(payload), "{\"op\":\"control.set\",\"hue_shift\":%u}", static_cast<unsigned>(hue_shift_degrees_));
        break;
    case RuntimePage::Settings:
        std::snprintf(payload, sizeof(payload), "{\"op\":\"control.set\",\"voice_enabled\":%s}", voice_enabled_ ? "true" : "false");
        break;
    case RuntimePage::Brightness:
        std::snprintf(payload, sizeof(payload), "{\"op\":\"control.set\",\"brightness\":%u}", brightness_level_);
        break;
    case RuntimePage::Battery:
    case RuntimePage::Link:
    default:
        return ESP_ERR_INVALID_ARG;
    }
    return send_control_payload_(payload);
}

bool ControllerApp::adjust_current_control_(const int delta)
{
    const RuntimePage page = runtime_page_from_index(current_page_index_);

    switch (page)
    {
    case RuntimePage::Expression:
    {
        const int count = std::max<int>(1, expression_count_);
        int next = static_cast<int>(expression_index_) + delta;
        next %= count;
        if (next < 0)
        {
            next += count;
        }
        if (next == expression_index_)
        {
            return false;
        }
        expression_index_ = static_cast<uint8_t>(next);
        return true;
    }
    case RuntimePage::Brightness:
    {
        const uint8_t next = static_cast<uint8_t>(std::clamp<int>(static_cast<int>(brightness_level_) + (delta * kRemoteBrightnessStep), 0, 255));
        if (next == brightness_level_)
        {
            return false;
        }

        brightness_level_ = next;
        return true;
    }
    case RuntimePage::Hue:
    {
        const uint16_t next = static_cast<uint16_t>(std::clamp<int>(static_cast<int>(hue_shift_degrees_) + (delta * kRemoteHueStepDegrees), 0, 360));
        if (next == hue_shift_degrees_)
        {
            return false;
        }

        hue_shift_degrees_ = next;
        return true;
    }
    case RuntimePage::Settings:
    case RuntimePage::Battery:
    case RuntimePage::Link:
    default:
        return false;
    }
}

bool ControllerApp::perform_current_page_action_()
{
    switch (runtime_page_from_index(current_page_index_))
    {
    case RuntimePage::Expression:
    case RuntimePage::Brightness:
    case RuntimePage::Hue:
        (void)send_current_control_();
        detail_view_active_ = false;
        settings_cursor_ = 0;
        last_joystick_direction_ = 0;
        return true;
    case RuntimePage::Settings:
        if (settings_cursor_ == 0)
        {
            voice_enabled_ = !voice_enabled_;
            (void)send_current_control_();
            detail_view_active_ = false;
            settings_cursor_ = 0;
            last_joystick_direction_ = 0;
            return true;
        }
        if (settings_cursor_ == 1)
        {
            return toggle_shake_random_();
        }
        if (settings_cursor_ == 2)
        {
            return cycle_oled_brightness_();
        }
        if (settings_cursor_ == 3)
        {
            return cycle_oled_timeout_();
        }
        if (settings_cursor_ == kFactoryResetSettingsIndex)
        {
            return factory_reset_();
        }
        return handle_back_action_();
    case RuntimePage::Battery:
        return refresh_battery_and_charger_("Gauge", "Gauge err");
    case RuntimePage::Link:
    {
        const esp_err_t err = discover_main_board_();
        control_status_ = err == ESP_OK ? "Link ready" : "Refresh err";
        if (err != ESP_OK)
        {
            (void)status_led_.set_mode(StatusLedMode::Error);
        }
        return true;
    }
    default:
        return false;
    }
}

bool ControllerApp::refresh_battery_and_charger_(const char *ok_status, const char *error_status)
{
    FuelGaugeSample battery = {};
    if (sensor_hub_.refresh_battery(&battery) == ESP_OK && battery.valid)
    {
        last_battery_ = battery;
        last_battery_chemistry_ = power_manager_.classify_battery_chemistry();
        (void)power_manager_.update_charger_control(last_battery_chemistry_);
        if (ok_status != nullptr)
        {
            control_status_ = ok_status;
        }
        return true;
    }

    last_battery_chemistry_ = BatteryChemistry::Unknown;
    (void)power_manager_.update_charger_control(last_battery_chemistry_);
    if (error_status != nullptr)
    {
        control_status_ = error_status;
    }
    return false;
}

bool ControllerApp::apply_expression_shortcut_(const uint8_t expression_index, const char *status_text)
{
    const uint8_t count = std::max<uint8_t>(1, expression_count_);
    expression_index_ = static_cast<uint8_t>(expression_index % count);
    const esp_err_t err = send_expression_control_();
    if (err == ESP_OK && status_text != nullptr)
    {
        control_status_ = status_text;
    }
    return true;
}

bool ControllerApp::randomize_expression_()
{
    const uint8_t count = std::max<uint8_t>(1, expression_count_);
    uint8_t next = static_cast<uint8_t>(esp_random() % count);
    if (count > 1 && next == expression_index_)
    {
        next = static_cast<uint8_t>((next + 1) % count);
    }
    return apply_expression_shortcut_(next, "Random");
}

bool ControllerApp::toggle_shake_random_()
{
    shake_random_enabled_ = !shake_random_enabled_;
    active_config_.controller.features.enable_shake_random = shake_random_enabled_;
    seed_config_.controller.features.enable_shake_random = shake_random_enabled_;
    last_shake_direction_ = 0;
    last_shake_peak_ms_ = 0;
    (void)config_storage_.persist_active_config(active_config_);
    control_status_ = shake_random_enabled_ ? "Shake on" : "Shake off";
    return true;
}

bool ControllerApp::factory_reset_()
{
    const UiLanguage language = active_config_.controller.ui_language;
    char lines[DisplayService::kTestMenuMaxLines][DisplayService::kTestMenuLineLength] = {};
    std::snprintf(lines[0], sizeof(lines[0]), "%s", localized(language, "Erase saved config", "清除已存配置"));
    std::snprintf(lines[1], sizeof(lines[1]), "%s", localized(language, "B1 confirm", "B1 确认"));
    std::snprintf(lines[2], sizeof(lines[2]), "%s", localized(language, "B2 cancel", "B2 取消"));
    if (display_service_.show_test_menu(localized(language, "Factory", "出厂"), nullptr, lines, 3, DisplayService::kTestMenuNoCursor) != ESP_OK)
    {
        return false;
    }

    uint8_t previous_bits = 0xFF;
    InputSnapshot snapshot = {};
    if (input_router_.get_snapshot(&snapshot) == ESP_OK)
    {
        previous_bits = snapshot.raw_bits;
    }

    while (true)
    {
        InputEvent event = {};
        while (input_router_.wait_for_event(&event, pdMS_TO_TICKS(50)) == ESP_OK)
        {
            if (event.type != InputEvent::Type::ButtonPressed)
            {
                continue;
            }
            if (event.source == board::ExpanderInputBit::Button2)
            {
                control_status_ = "Reset cancel";
                return true;
            }
            if (event.source == board::ExpanderInputBit::JoystickButton ||
                event.source == board::ExpanderInputBit::Button0 ||
                event.source == board::ExpanderInputBit::Button1)
            {
                const esp_err_t reset_err = config_storage_.factory_reset();
                if (reset_err != ESP_OK)
                {
                    control_status_ = esp_err_to_name(reset_err);
                    return true;
                }
                (void)display_service_.show_status(localized(language, "Reset", "重置"), localized(language, "Rebooting", "正在重启"), StatusLedMode::Booting);
                vTaskDelay(pdMS_TO_TICKS(500));
                esp_restart();
            }
        }

        if (input_router_.get_snapshot(&snapshot) == ESP_OK)
        {
            const auto is_pressed = [&snapshot](const board::ExpanderInputBit bit) {
                return snapshot.is_asserted(bit);
            };
            const auto was_pressed = [previous_bits](const board::ExpanderInputBit bit) {
                return (previous_bits & (1U << static_cast<uint8_t>(bit))) == 0;
            };

            const bool confirm_pressed = (is_pressed(board::ExpanderInputBit::JoystickButton) && !was_pressed(board::ExpanderInputBit::JoystickButton)) ||
                                         (is_pressed(board::ExpanderInputBit::Button0) && !was_pressed(board::ExpanderInputBit::Button0)) ||
                                         (is_pressed(board::ExpanderInputBit::Button1) && !was_pressed(board::ExpanderInputBit::Button1));
            if (confirm_pressed)
            {
                const esp_err_t reset_err = config_storage_.factory_reset();
                if (reset_err != ESP_OK)
                {
                    control_status_ = esp_err_to_name(reset_err);
                    return true;
                }
                (void)display_service_.show_status(localized(language, "Reset", "重置"), localized(language, "Rebooting", "正在重启"), StatusLedMode::Booting);
                vTaskDelay(pdMS_TO_TICKS(500));
                esp_restart();
            }
            if (is_pressed(board::ExpanderInputBit::Button2) && !was_pressed(board::ExpanderInputBit::Button2))
            {
                control_status_ = "Reset cancel";
                return true;
            }
            previous_bits = snapshot.raw_bits;
        }
    }
}

bool ControllerApp::cycle_oled_brightness_()
{
    uint16_t next = static_cast<uint16_t>(oled_brightness_) + kOledBrightnessStep;
    if (next > 255)
    {
        next = kOledBrightnessMinimum;
    }
    oled_brightness_ = static_cast<uint8_t>(next);
    active_config_.controller.display.oled_brightness = oled_brightness_;
    seed_config_.controller.display.oled_brightness = oled_brightness_;
    (void)display_service_.set_brightness(oled_brightness_);
    (void)config_storage_.persist_active_config(active_config_);
    control_status_ = "OLED";
    return true;
}

bool ControllerApp::cycle_oled_timeout_()
{
    static constexpr uint16_t kTimeoutOptions[] = {15, 30, 60, 120, 0};
    size_t index = 0;
    for (size_t candidate = 0; candidate < (sizeof(kTimeoutOptions) / sizeof(kTimeoutOptions[0])); ++candidate)
    {
        if (oled_timeout_seconds_ == kTimeoutOptions[candidate])
        {
            index = candidate;
            break;
        }
    }
    const uint16_t next = kTimeoutOptions[(index + 1) % (sizeof(kTimeoutOptions) / sizeof(kTimeoutOptions[0]))];
    oled_timeout_seconds_ = next;
    active_config_.controller.display.oled_timeout_seconds = oled_timeout_seconds_;
    seed_config_.controller.display.oled_timeout_seconds = oled_timeout_seconds_;
    (void)config_storage_.persist_active_config(active_config_);
    control_status_ = oled_timeout_seconds_ == 0 ? "No sleep" : "Timeout";
    return true;
}

bool ControllerApp::wake_display_(const uint32_t now)
{
    last_activity_ms_ = now;
    if (!display_sleeping_)
    {
        return false;
    }
    display_sleeping_ = false;
    (void)display_service_.set_sleeping(false);
    last_ui_refresh_ms_ = 0;
    return true;
}

bool ControllerApp::sleep_display_if_idle_(const uint32_t now)
{
    if (display_sleeping_ || oled_timeout_seconds_ == 0)
    {
        return false;
    }

    const uint32_t timeout_ms = static_cast<uint32_t>(oled_timeout_seconds_) * 1000U;
    if ((now - last_activity_ms_) < timeout_ms)
    {
        return false;
    }

    display_sleeping_ = true;
    (void)display_service_.set_sleeping(true);
    return true;
}

void ControllerApp::apply_display_settings_()
{
    oled_brightness_ = std::clamp<uint8_t>(active_config_.controller.display.oled_brightness, kOledBrightnessMinimum, 255);
    oled_timeout_seconds_ = active_config_.controller.display.oled_timeout_seconds;
    active_config_.controller.display.oled_brightness = oled_brightness_;
    seed_config_.controller.display = active_config_.controller.display;
    (void)display_service_.set_brightness(oled_brightness_);
}

void ControllerApp::update_signal_strength_()
{
    uint8_t signal_percent = 0;
    const bool visible = pairing_service_.get_signal_strength(&signal_percent);
    display_service_.set_signal_strength(signal_percent, visible);
}

bool ControllerApp::handle_input_event_(const InputEvent &event)
{
    if (event.asserted())
    {
        last_input_snapshot_.raw_bits = event.raw_bits;
        if (wake_display_(now_ms()))
        {
            return true;
        }
    }

    if (event.type != InputEvent::Type::ButtonPressed && event.type != InputEvent::Type::ButtonReleased)
    {
        return false;
    }

    last_input_snapshot_.raw_bits = event.raw_bits;
    if (event.type != InputEvent::Type::ButtonPressed)
    {
        return false;
    }

    ESP_LOGI(TAG, "Runtime button event: %s", board::expander_input_name(event.source));
    switch (event.source)
    {
    case board::ExpanderInputBit::JoystickButton:
    case board::ExpanderInputBit::Button0:
    case board::ExpanderInputBit::Button1:
        return activate_current_selection_();
    case board::ExpanderInputBit::Button2:
        return handle_back_action_();
    default:
        return false;
    }
}

bool ControllerApp::handle_input_snapshot_(const InputSnapshot &snapshot)
{
    const uint8_t previous_bits = last_input_snapshot_.raw_bits;
    last_input_snapshot_ = snapshot;

    constexpr uint8_t kButtonMask = (1U << static_cast<uint8_t>(board::ExpanderInputBit::JoystickButton))
                                  | (1U << static_cast<uint8_t>(board::ExpanderInputBit::Button0))
                                  | (1U << static_cast<uint8_t>(board::ExpanderInputBit::Button1))
                                  | (1U << static_cast<uint8_t>(board::ExpanderInputBit::Button2));
    const uint8_t previous_button_bits = previous_bits & kButtonMask;
    const uint8_t current_button_bits = snapshot.raw_bits & kButtonMask;

    if (current_button_bits == previous_button_bits)
    {
        return false;
    }

    const auto is_pressed = [&snapshot](const board::ExpanderInputBit bit) {
        return snapshot.is_asserted(bit);
    };
    const auto was_pressed = [previous_bits](const board::ExpanderInputBit bit) {
        return (previous_bits & (1U << static_cast<uint8_t>(bit))) == 0;
    };

    ESP_LOGI(
        TAG,
        "Buttons: raw=0x%02x E=%d J=%d B0=%d B1=%d B2=%d",
        snapshot.raw_bits,
        gpio_get_level(board::kPinSensorRailEnable) != 0 ? 1 : 0,
        is_pressed(board::ExpanderInputBit::JoystickButton) ? 1 : 0,
        is_pressed(board::ExpanderInputBit::Button0) ? 1 : 0,
        is_pressed(board::ExpanderInputBit::Button1) ? 1 : 0,
        is_pressed(board::ExpanderInputBit::Button2) ? 1 : 0);

    bool changed = true;
    const bool select_pressed = (is_pressed(board::ExpanderInputBit::JoystickButton) && !was_pressed(board::ExpanderInputBit::JoystickButton))
                             || (is_pressed(board::ExpanderInputBit::Button0) && !was_pressed(board::ExpanderInputBit::Button0))
                             || (is_pressed(board::ExpanderInputBit::Button1) && !was_pressed(board::ExpanderInputBit::Button1));
    if (select_pressed)
    {
        if (wake_display_(now_ms()))
        {
            return true;
        }
        changed = activate_current_selection_() || changed;
        last_activity_ms_ = now_ms();
    }
    else if (is_pressed(board::ExpanderInputBit::Button2) && !was_pressed(board::ExpanderInputBit::Button2))
    {
        if (wake_display_(now_ms()))
        {
            return true;
        }
        changed = handle_back_action_() || changed;
        last_activity_ms_ = now_ms();
    }

    return changed;
}

bool ControllerApp::handle_sensor_event_(const SensorEvent &event)
{
    const UiLanguage language = active_config_.controller.ui_language;

    switch (event.type)
    {
    case SensorEvent::Type::Motion:
        (void)language;
        return handle_motion_sample_(event.motion);
    case SensorEvent::Type::Gesture:
        (void)language;
        return handle_gesture_sample_(event.gesture);
    case SensorEvent::Type::FuelGauge:
        last_battery_ = event.battery;
        last_battery_chemistry_ = power_manager_.classify_battery_chemistry();
        (void)power_manager_.update_charger_control(last_battery_chemistry_);
        return true;
    case SensorEvent::Type::Fault:
        status_led_.set_mode(StatusLedMode::Error);
        display_service_.show_error(
            localized(language, "Sensor", "传感器"),
            esp_err_to_name(event.status),
            localized(language, "Check interrupt chain", "检查中断链路"));
        return true;
    default:
        return false;
    }

}

bool ControllerApp::handle_gesture_sample_(const GestureSample &sample)
{
    const UiLanguage language = active_config_.controller.ui_language;
    char summary[80] = {};
    const GestureUiAction action = summarize_gesture(sample, language, summary, sizeof(summary));
    const uint32_t now = now_ms();
    const bool surprise_proximity = sample.valid && gesture_surprise_proximity(sample);
    const bool surprise_started = surprise_proximity && !last_gesture_proximity_close_;

    last_gesture_ = sample;
    gesture_summary_ = summary;
    last_gesture_proximity_close_ = surprise_proximity;

    if (sample.valid && (now - last_gesture_log_ms_) >= kGestureLogIntervalMs)
    {
        ESP_LOGI(TAG,
                 "Gesture sample P1=%u P2=%u P3=%u flags=0x%02x",
                 static_cast<unsigned>(sample.proximity_1),
                 static_cast<unsigned>(sample.proximity_2),
                 static_cast<unsigned>(sample.proximity_3),
                 static_cast<unsigned>(sample.interrupt_flags));
        last_gesture_log_ms_ = now;
    }

    if (action != GestureUiAction::None)
    {
        (void)wake_display_(now);
    }

    if (action == GestureUiAction::Surprise && surprise_started && (now - last_gesture_action_ms_) >= kGestureActionCooldownMs)
    {
        last_gesture_action_ms_ = now;
        return apply_expression_shortcut_(kSurpriseExpressionIndex, "Surprise");
    }

    return action != GestureUiAction::None;
}

bool ControllerApp::detect_shake_pair_(const MotionSample &sample, const uint32_t now)
{
    int8_t direction = 0;
    if (last_motion_.valid)
    {
        const float delta_y = sample.y_mg - last_motion_.y_mg;
        const float abs_delta_x = std::fabs(sample.x_mg - last_motion_.x_mg);
        const float abs_delta_y = std::fabs(delta_y);
        const float abs_delta_z = std::fabs(sample.z_mg - last_motion_.z_mg);
        if (abs_delta_y >= kShakeDeltaThresholdMg &&
            abs_delta_y >= (abs_delta_x * kShakeVerticalDominance) &&
            abs_delta_y >= (abs_delta_z * kShakeVerticalDominance))
        {
            direction = delta_y > 0.0f ? 1 : -1;
        }
    }

    if (direction == 0 && sample.wake_event)
    {
        if (sample.y_wake)
        {
            direction = sample.y_mg >= 0.0f ? 1 : -1;
        }
    }

    if (direction == 0)
    {
        return false;
    }

    const bool paired = last_shake_direction_ != 0 &&
                        direction != last_shake_direction_ &&
                        (now - last_shake_peak_ms_) <= kShakePairWindowMs;
    last_shake_direction_ = direction;
    last_shake_peak_ms_ = now;
    return paired;
}

bool ControllerApp::handle_motion_sample_(const MotionSample &sample)
{
    const UiLanguage language = active_config_.controller.ui_language;
    char summary[80] = {};
    MotionUiAction action = summarize_motion(sample, language, summary, sizeof(summary));
    if (!shake_random_enabled_ && action == MotionUiAction::RandomExpression)
    {
        std::snprintf(summary, sizeof(summary), "%s", localized(language, "Shake off", "摇动关闭"));
        action = MotionUiAction::None;
    }
    const uint32_t now = now_ms();
    const bool shake_pair = shake_random_enabled_ && sample.valid && detect_shake_pair_(sample, now);
    const bool noteworthy = sample.wake_event || sample.sleep_change || shake_pair;

    last_motion_ = sample;
    motion_summary_ = summary;

    if (sample.valid && (now - last_motion_log_ms_) >= kMotionLogIntervalMs)
    {
        ESP_LOGI(TAG,
                 "IMU sample X=%d Y=%d Z=%d wake=0x%02x gyro=%s",
                 static_cast<int>(sample.x_mg),
                 static_cast<int>(sample.y_mg),
                 static_cast<int>(sample.z_mg),
                 sample.wake_source,
                 sample.gyro_valid ? "yes" : "no");
        last_motion_log_ms_ = now;
    }

    if (noteworthy)
    {
        control_status_ = shake_pair ? localized(language, "Shake random", "摇动随机") : motion_summary_;
        (void)wake_display_(now);
    }

    if (shake_random_enabled_ && (shake_pair || action == MotionUiAction::RandomExpression) && (now - last_motion_action_ms_) >= kMotionActionCooldownMs)
    {
        last_motion_action_ms_ = now;
        return randomize_expression_();
    }

    return noteworthy;
}

bool ControllerApp::handle_joystick_sample_(const JoystickSample &sample)
{
    if (!sample.valid)
    {
        return false;
    }

    const JoystickSample previous = last_joystick_;

    const bool diagnostics_changed = !last_joystick_.valid
                                     || sample.x_available != last_joystick_.x_available
                                     || sample.y_available != last_joystick_.y_available
                                     || sample.raw_x != last_joystick_.raw_x
                                     || sample.raw_y != last_joystick_.raw_y
                                     || sample.normalized_x != last_joystick_.normalized_x
                                     || sample.normalized_y != last_joystick_.normalized_y
                                     || sample.pressed != last_joystick_.pressed;
    last_joystick_ = sample;

    const int dx = sample.normalized_x - previous.normalized_x;
    const int dy = sample.normalized_y - previous.normalized_y;
    const bool loggable_change = !previous.valid
                                 || sample.x_available != previous.x_available
                                 || sample.y_available != previous.y_available
                                 || dx >= 3
                                 || dx <= -3
                                 || dy >= 3
                                 || dy <= -3
                                 || sample.pressed != previous.pressed;
    if (loggable_change)
    {
        ESP_LOGI(
            TAG,
            "Joystick: axes(X=%d,Y=%d) raw=(%d,%d) norm=(%d,%d) pressed=%d rail=%d",
            sample.x_available ? 1 : 0,
            sample.y_available ? 1 : 0,
            sample.raw_x,
            sample.raw_y,
            sample.normalized_x,
            sample.normalized_y,
            sample.pressed ? 1 : 0,
            gpio_get_level(board::kPinSensorRailEnable) != 0 ? 1 : 0);
    }

    int8_t direction = 0;
    if (sample.right)
    {
        direction = 1;
    }
    else if (sample.left)
    {
        direction = -1;
    }

    if (direction != 0 && wake_display_(now_ms()))
    {
        last_joystick_direction_ = direction;
        return true;
    }

    if (direction == last_joystick_direction_)
    {
        return diagnostics_changed;
    }

    last_joystick_direction_ = direction;
    if (direction == 0)
    {
        return diagnostics_changed;
    }

    const bool adjusted = navigate_current_view_(direction);
    last_activity_ms_ = now_ms();
    return diagnostics_changed || adjusted;
}

bool ControllerApp::activate_current_selection_()
{
    last_joystick_direction_ = 0;
    if (!detail_view_active_)
    {
        detail_view_active_ = true;
        if (runtime_page_from_index(current_page_index_) == RuntimePage::Settings)
        {
            settings_cursor_ = 0;
        }
        return true;
    }

    return perform_current_page_action_();
}

bool ControllerApp::handle_back_action_()
{
    if (!detail_view_active_)
    {
        return false;
    }

    detail_view_active_ = false;
    settings_cursor_ = 0;
    last_joystick_direction_ = 0;
    return true;
}

bool ControllerApp::navigate_current_view_(const int delta)
{
    if (delta == 0)
    {
        return false;
    }

    if (!detail_view_active_)
    {
        select_relative_page_(delta);
        return true;
    }

    switch (runtime_page_from_index(current_page_index_))
    {
    case RuntimePage::Expression:
    case RuntimePage::Brightness:
    case RuntimePage::Hue:
        return adjust_current_control_(delta);
    case RuntimePage::Settings:
    {
        int next = static_cast<int>(settings_cursor_) + delta;
        next %= kSettingsMenuItems;
        if (next < 0)
        {
            next += kSettingsMenuItems;
        }
        if (next == settings_cursor_)
        {
            return false;
        }
        settings_cursor_ = static_cast<uint8_t>(next);
        return true;
    }
    case RuntimePage::Battery:
    case RuntimePage::Link:
    default:
        return false;
    }
}

esp_err_t ControllerApp::update_runtime_display_(const bool force)
{
    if (display_sleeping_)
    {
        return ESP_OK;
    }

    const uint32_t now = now_ms();
    if (!force && (now - last_ui_refresh_ms_) < kUiRefreshIntervalMs)
    {
        return ESP_OK;
    }

    update_signal_strength_();

    const UiLanguage language = active_config_.controller.ui_language;
    const RuntimePage page = runtime_page_from_index(current_page_index_);
    char aux[72] = {};
    char lines[DisplayService::kTestMenuMaxLines][DisplayService::kTestMenuLineLength] = {};
    char slider[DisplayService::kTestMenuLineLength] = {};
    const std::string peer = shorten_peer_id(active_config_.controller.pairing.bound_peer_id);
    const char *status = control_status_.empty() ? "Ready" : control_status_.c_str();
    const bool usb_present = power_manager_.external_power_present();
    if (last_battery_.valid)
    {
        std::snprintf(aux,
                      sizeof(aux),
                      "%s %s Battery %.0f%%%s",
                      peer.c_str(),
                      status,
                      last_battery_.state_of_charge_percent,
                      usb_present ? " USB" : "");
    }
    else
    {
        std::snprintf(aux, sizeof(aux), "%s %s%s", peer.c_str(), status, usb_present ? " USB" : "");
    }

    uint8_t selected_index = 0;
    uint8_t line_count = 0;

    if (!detail_view_active_)
    {
        const std::string animation_title = shorten_text(animation_name_for(active_config_.controller.visual), 11);
        std::snprintf(lines[0], sizeof(lines[0]), "%s", animation_title.c_str());
        std::snprintf(lines[1], sizeof(lines[1]), "Bright %u", static_cast<unsigned>(brightness_level_));
        std::snprintf(lines[2], sizeof(lines[2]), "Hue %u deg", static_cast<unsigned>(hue_shift_degrees_));
        std::snprintf(lines[3], sizeof(lines[3]), "Settings V%s S%s", voice_enabled_ ? "1" : "0", shake_random_enabled_ ? "1" : "0");
        if (last_battery_.valid)
        {
            std::snprintf(lines[4], sizeof(lines[4]), "Battery %.0f%%", last_battery_.state_of_charge_percent);
        }
        else
        {
            std::snprintf(lines[4], sizeof(lines[4]), "%s", localized(language, "Battery --", "电池 --"));
        }
        std::snprintf(lines[5], sizeof(lines[5]), "Link %s", peer.c_str());
        selected_index = current_page_index_;
        line_count = 6;
        ESP_RETURN_ON_ERROR(display_service_.show_test_menu(localized(language, "Remote", "遥控"), aux, lines, line_count, selected_index), TAG, "Runtime display update failed");
        last_ui_refresh_ms_ = now;
        return ESP_OK;
    }

    switch (page)
    {
    case RuntimePage::Expression:
        std::snprintf(lines[0], sizeof(lines[0]), "%s", shorten_text(animation_name_for(active_config_.controller.visual), 16).c_str());
        build_slider_bar(slider, sizeof(slider), expression_index_, 0, std::max<int>(1, expression_count_) - 1);
        std::snprintf(lines[1], sizeof(lines[1]), "%s", slider);
        std::snprintf(lines[2], sizeof(lines[2]), "Face %u / %u", static_cast<unsigned>(expression_index_ + 1), static_cast<unsigned>(expression_count_));
        std::snprintf(lines[3], sizeof(lines[3]), "%s", expression_name_for(active_config_.controller.visual, expression_index_));
        std::snprintf(lines[4], sizeof(lines[4]), "%s", localized(language, "Stick adjust", "摇杆调整"));
        std::snprintf(lines[5], sizeof(lines[5]), "%s", localized(language, "B1/B0 apply B2 back", "B1/B0应用 B2返"));
        selected_index = DisplayService::kTestMenuNoCursor;
        line_count = 6;
        break;
    case RuntimePage::Brightness:
        std::snprintf(lines[0], sizeof(lines[0]), "Bright %u / 255", static_cast<unsigned>(brightness_level_));
        build_slider_bar(slider, sizeof(slider), brightness_level_, 0, 255);
        std::snprintf(lines[1], sizeof(lines[1]), "%s", slider);
        std::snprintf(lines[2], sizeof(lines[2]), "%s", localized(language, "Stick adjust", "摇杆调整"));
        std::snprintf(lines[3], sizeof(lines[3]), "%s", localized(language, "B1/B0 apply B2 back", "B1/B0应用 B2返"));
        selected_index = DisplayService::kTestMenuNoCursor;
        line_count = 4;
        break;
    case RuntimePage::Hue:
    {
        const uint32_t base_color = rgb888(active_config_.controller.visual.red, active_config_.controller.visual.green, active_config_.controller.visual.blue);
        const uint32_t shifted_color = hue_shift_rgb888(active_config_.controller.visual, hue_shift_degrees_);
        std::snprintf(lines[0], sizeof(lines[0]), "Hue %u deg", static_cast<unsigned>(hue_shift_degrees_));
        build_slider_bar(slider, sizeof(slider), hue_shift_degrees_, 0, 360);
        std::snprintf(lines[1], sizeof(lines[1]), "%s", slider);
        format_color_swatch_line(lines[2], sizeof(lines[2]), "Base", base_color);
        format_color_swatch_line(lines[3], sizeof(lines[3]), "Now", shifted_color);
        std::snprintf(lines[4], sizeof(lines[4]), "%s", localized(language, "Stick adjust", "摇杆调整"));
        std::snprintf(lines[5], sizeof(lines[5]), "%s", localized(language, "B1 apply B2", "B1应用 B2返"));
        selected_index = DisplayService::kTestMenuNoCursor;
        line_count = 6;
        break;
    }
    case RuntimePage::Settings:
        format_settings_item_label(0, language, voice_enabled_, shake_random_enabled_, oled_brightness_, oled_timeout_seconds_, lines[0], sizeof(lines[0]));
        format_settings_item_label(1, language, voice_enabled_, shake_random_enabled_, oled_brightness_, oled_timeout_seconds_, lines[1], sizeof(lines[1]));
        format_settings_item_label(2, language, voice_enabled_, shake_random_enabled_, oled_brightness_, oled_timeout_seconds_, lines[2], sizeof(lines[2]));
        format_settings_item_label(3, language, voice_enabled_, shake_random_enabled_, oled_brightness_, oled_timeout_seconds_, lines[3], sizeof(lines[3]));
        format_settings_item_label(4, language, voice_enabled_, shake_random_enabled_, oled_brightness_, oled_timeout_seconds_, lines[4], sizeof(lines[4]));
        format_settings_item_label(5, language, voice_enabled_, shake_random_enabled_, oled_brightness_, oled_timeout_seconds_, lines[5], sizeof(lines[5]));
        selected_index = settings_cursor_;
        line_count = 6;
        break;
    case RuntimePage::Battery:
    {
        const bool pwr_stat = gpio_get_level(board::kPinPowerStatus) != 0;
        const bool vbat_mon = gpio_get_level(board::kPinBatteryMonitor) != 0;
        if (last_battery_.valid)
        {
            std::snprintf(lines[0], sizeof(lines[0]), "%.2fV %.0f%%", last_battery_.voltage_v, last_battery_.state_of_charge_percent);
            std::snprintf(lines[1], sizeof(lines[1]), "%s Gate %s", battery_label(last_battery_chemistry_, language), power_manager_.charger_enabled() ? "ON" : "OFF");
        }
        else
        {
            std::snprintf(lines[0], sizeof(lines[0]), "%s", localized(language, "Gauge pending", "等待电量计"));
            std::snprintf(lines[1], sizeof(lines[1]), "%s", battery_label(last_battery_chemistry_, language));
        }
        std::snprintf(lines[2], sizeof(lines[2]), "PWR %u VB %u", pwr_stat ? 1U : 0U, vbat_mon ? 1U : 0U);
        std::snprintf(lines[3], sizeof(lines[3]), "%s", localized(language, "B1/B0 refresh B2", "B1/B0刷 B2返"));
        selected_index = DisplayService::kTestMenuNoCursor;
        line_count = 4;
        break;
    }
    case RuntimePage::Link:
    default:
        std::snprintf(lines[0], sizeof(lines[0]), "Peer %s", peer.c_str());
        std::snprintf(lines[1], sizeof(lines[1]), "Source %s", ready_detail_for_source(active_config_.source, language));
        std::snprintf(lines[2], sizeof(lines[2]), "%s", localized(language, "B1/B0 refresh link", "B1/B0刷新连接"));
        std::snprintf(lines[3], sizeof(lines[3]), "%s", status);
        std::snprintf(lines[4], sizeof(lines[4]), "%s", localized(language, "B2 back", "B2返回"));
        selected_index = DisplayService::kTestMenuNoCursor;
        line_count = 5;
        break;
    }

    ESP_RETURN_ON_ERROR(display_service_.show_test_menu(runtime_page_title(page, language), aux, lines, line_count, selected_index), TAG, "Runtime display update failed");
    last_ui_refresh_ms_ = now;
    return ESP_OK;
}

esp_err_t ControllerApp::run_runtime_loop_()
{
    current_page_index_ = 0;
    detail_view_active_ = false;
    settings_cursor_ = 0;
    control_status_ = active_config_.source == ConfigSourceKind::MainBoard ? "Linked" : "Ready";
    last_activity_ms_ = now_ms();
    last_ui_refresh_ms_ = 0;
    last_battery_poll_ms_ = 0;
    last_power_status_poll_ms_ = 0;
    last_joystick_poll_ms_ = 0;
    last_motion_poll_ms_ = 0;
    last_motion_log_ms_ = 0;
    last_gesture_poll_ms_ = 0;
    last_gesture_log_ms_ = 0;
    last_motion_action_ms_ = 0;
    last_gesture_action_ms_ = 0;
    last_shake_peak_ms_ = 0;
    last_shake_direction_ = 0;
    last_gesture_proximity_close_ = false;
    display_sleeping_ = false;
    (void)display_service_.set_sleeping(false);
    apply_display_settings_();
    last_power_status_level_ = power_manager_.external_power_present();
    (void)input_router_.get_snapshot(&last_input_snapshot_);
    (void)input_router_.read_joystick(&last_joystick_);
    (void)refresh_battery_and_charger_(nullptr, nullptr);

    ESP_RETURN_ON_ERROR(update_runtime_display_(true), TAG, "Initial runtime display update failed");

    while (true)
    {
        bool changed = false;

        InputEvent input_event = {};
        TickType_t input_timeout = pdMS_TO_TICKS(40);
        while (input_router_.wait_for_event(&input_event, input_timeout) == ESP_OK)
        {
            changed = handle_input_event_(input_event) || changed;
            input_timeout = 0;
        }

        InputSnapshot snapshot = last_input_snapshot_;
        if (input_router_.get_snapshot(&snapshot) == ESP_OK)
        {
            changed = handle_input_snapshot_(snapshot) || changed;
        }

        SensorEvent sensor_event = {};
        while (sensor_hub_.wait_for_event(&sensor_event, 0) == ESP_OK)
        {
            changed = handle_sensor_event_(sensor_event) || changed;
        }

        const uint32_t now = now_ms();
        if ((now - last_power_status_poll_ms_) >= kPowerStatusPollIntervalMs)
        {
            const bool power_status = power_manager_.external_power_present();
            if (power_status != last_power_status_level_)
            {
                last_power_status_level_ = power_status;
                changed = refresh_battery_and_charger_(power_status ? "USB" : "Power", "Gauge err") || changed;
            }
            last_power_status_poll_ms_ = now;
        }

        if ((now - last_joystick_poll_ms_) >= kJoystickPollIntervalMs)
        {
            JoystickSample joystick = {};
            if (input_router_.read_joystick(&joystick) == ESP_OK)
            {
                changed = handle_joystick_sample_(joystick) || changed;
            }
            last_joystick_poll_ms_ = now;
        }

        if ((now - last_motion_poll_ms_) >= kMotionPollIntervalMs)
        {
            MotionSample motion = {};
            const esp_err_t motion_err = active_config_.controller.features.enable_imu ? sensor_hub_.sample_motion_now(&motion) : ESP_ERR_INVALID_STATE;
            if (motion_err == ESP_OK)
            {
                changed = handle_motion_sample_(motion) || changed;
            }
            else if (active_config_.controller.features.enable_imu && (now - last_motion_log_ms_) >= kMotionLogIntervalMs)
            {
                ESP_LOGW(TAG, "IMU poll failed: %s", esp_err_to_name(motion_err));
                last_motion_log_ms_ = now;
            }
            last_motion_poll_ms_ = now;
        }

        if ((now - last_gesture_poll_ms_) >= kGesturePollIntervalMs)
        {
            GestureSample gesture = {};
            const esp_err_t gesture_err = active_config_.controller.features.enable_gesture ? sensor_hub_.sample_gesture_now(&gesture) : ESP_ERR_INVALID_STATE;
            if (gesture_err == ESP_OK)
            {
                changed = handle_gesture_sample_(gesture) || changed;
            }
            else if (active_config_.controller.features.enable_gesture && (now - last_gesture_log_ms_) >= kGestureLogIntervalMs)
            {
                ESP_LOGW(TAG, "Gesture poll failed: %s", esp_err_to_name(gesture_err));
                last_gesture_log_ms_ = now;
            }
            last_gesture_poll_ms_ = now;
        }

        if ((now - last_battery_poll_ms_) >= kBatteryPollIntervalMs)
        {
            changed = refresh_battery_and_charger_(nullptr, nullptr) || changed;
            last_battery_poll_ms_ = now;
        }

        if (sleep_display_if_idle_(now))
        {
            changed = false;
        }

        if (!display_sleeping_ && (changed || (now - last_ui_refresh_ms_) >= kUiRefreshIntervalMs))
        {
            ESP_RETURN_ON_ERROR(update_runtime_display_(changed), TAG, "Runtime display update failed");
        }
    }
}
} // namespace prototracer