#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace prototracer
{
enum class ConfigSourceKind : uint8_t
{
    MainBoard = 0,
    RemoteRepo = 1,
    FilesystemImage = 2,
    Failsafe = 3,
};

enum class BatteryChemistry : uint8_t
{
    Unknown = 0,
    NiMH = 1,
    Alkaline = 2,
};

enum class StatusLedMode : uint8_t
{
    Off = 0,
    Booting = 1,
    Provisioning = 2,
    Linked = 3,
    Updating = 4,
    Error = 5,
};

enum class UiLanguage : uint8_t
{
    English = 0,
    Chinese = 1,
};

struct RemoteRepoConfig
{
    std::string manifest_url;
    std::string asset_base_url;
    std::string auth_token_nvs_key;
    std::string auth_scheme;
    std::string accept_header;
};

struct PairingConfig
{
    std::string discovery_transport;
    std::string service_uuid;
    std::string rx_characteristic_uuid;
    std::string tx_characteristic_uuid;
    std::string config_endpoint;
    std::string bound_peer_id;
};

struct BlePeerCandidate
{
    std::string display_name;
    std::string peer_id;
    int8_t rssi = -100;
    uint8_t signal_percent = 0;
};

struct NetworkConfig
{
    std::string hostname_prefix;
    std::string provisioning_ap_prefix;
    std::string station_ssid;
    std::string station_password;
};

struct VisualConfig
{
    std::string animation_asset;
    std::string animation_name;
    uint8_t expression_count = 17;
    std::vector<std::string> expression_names;
    uint8_t red = 25;
    uint8_t green = 125;
    uint8_t blue = 235;
};

struct FeatureConfig
{
    bool enable_gesture = true;
    bool enable_imu = true;
    bool enable_shake_random = true;
    bool enable_ws2812 = true;
    bool enable_low_power_core = true;
    bool allow_manual_fs_upload = true;
};

struct DisplayConfig
{
    uint8_t oled_brightness = 192;
    uint16_t oled_timeout_seconds = 30;
};

struct ControllerConfig
{
    std::string device_id;
    std::string display_name;
    std::string hardware_revision;
    UiLanguage ui_language = UiLanguage::English;
    NetworkConfig network;
    PairingConfig pairing;
    RemoteRepoConfig repo;
    VisualConfig visual;
    FeatureConfig features;
    DisplayConfig display;
};

struct ResolvedConfig
{
    ControllerConfig controller;
    ConfigSourceKind source = ConfigSourceKind::Failsafe;
    std::string note;
};

inline const char *config_source_name(const ConfigSourceKind source)
{
    switch (source)
    {
    case ConfigSourceKind::MainBoard:
        return "main_board";
    case ConfigSourceKind::RemoteRepo:
        return "remote_repo";
    case ConfigSourceKind::FilesystemImage:
        return "filesystem_image";
    case ConfigSourceKind::Failsafe:
    default:
        return "failsafe";
    }
}

inline const char *battery_chemistry_name(const BatteryChemistry chemistry)
{
    switch (chemistry)
    {
    case BatteryChemistry::NiMH:
        return "nimh";
    case BatteryChemistry::Alkaline:
        return "alkaline";
    case BatteryChemistry::Unknown:
    default:
        return "unknown";
    }
}

inline const char *status_led_mode_name(const StatusLedMode mode)
{
    switch (mode)
    {
    case StatusLedMode::Booting:
        return "booting";
    case StatusLedMode::Provisioning:
        return "provisioning";
    case StatusLedMode::Linked:
        return "linked";
    case StatusLedMode::Updating:
        return "updating";
    case StatusLedMode::Error:
        return "error";
    case StatusLedMode::Off:
    default:
        return "off";
    }
}

inline UiLanguage ui_language_from_string(const std::string &value)
{
    if (value == "zh" || value == "zh-cn" || value == "zh_CN" || value == "cn" || value == "chinese")
    {
        return UiLanguage::Chinese;
    }

    return UiLanguage::English;
}

inline const char *ui_language_name(const UiLanguage language)
{
    switch (language)
    {
    case UiLanguage::Chinese:
        return "zh-CN";
    case UiLanguage::English:
    default:
        return "en";
    }
}
} // namespace prototracer