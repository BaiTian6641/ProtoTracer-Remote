#include "config_manifest.hpp"

#include <cstdio>

#include "cJSON.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "sdkconfig.h"

namespace
{
constexpr const char *TAG = "config_manifest";

std::string json_string(cJSON *object, const char *key, const char *fallback)
{
    cJSON *item = cJSON_GetObjectItemCaseSensitive(object, key);
    if (cJSON_IsString(item) && item->valuestring != nullptr)
    {
        return std::string(item->valuestring);
    }
    return std::string(fallback != nullptr ? fallback : "");
}

std::string json_string_first(cJSON *object, const char *key, const char *alternate_key, const char *fallback)
{
    const std::string value = json_string(object, key, nullptr);
    if (!value.empty())
    {
        return value;
    }

    if (alternate_key != nullptr)
    {
        return json_string(object, alternate_key, fallback);
    }

    return std::string(fallback != nullptr ? fallback : "");
}

bool json_bool(cJSON *object, const char *key, const bool fallback)
{
    cJSON *item = cJSON_GetObjectItemCaseSensitive(object, key);
    if (cJSON_IsBool(item))
    {
        return cJSON_IsTrue(item);
    }
    return fallback;
}

uint8_t json_color(cJSON *object, const char *key, const uint8_t fallback)
{
    cJSON *item = cJSON_GetObjectItemCaseSensitive(object, key);
    if (!cJSON_IsNumber(item))
    {
        return fallback;
    }

    int value = item->valueint;
    if (value < 0)
    {
        value = 0;
    }
    if (value > 255)
    {
        value = 255;
    }
    return static_cast<uint8_t>(value);
}

uint8_t json_uint8(cJSON *object, const char *key, const uint8_t fallback, const uint8_t minimum, const uint8_t maximum)
{
    cJSON *item = cJSON_GetObjectItemCaseSensitive(object, key);
    if (!cJSON_IsNumber(item))
    {
        return fallback;
    }

    int value = item->valueint;
    if (value < static_cast<int>(minimum))
    {
        value = minimum;
    }
    if (value > static_cast<int>(maximum))
    {
        value = maximum;
    }
    return static_cast<uint8_t>(value);
}

uint16_t json_uint16(cJSON *object, const char *key, const uint16_t fallback, const uint16_t minimum, const uint16_t maximum)
{
    cJSON *item = cJSON_GetObjectItemCaseSensitive(object, key);
    if (!cJSON_IsNumber(item))
    {
        return fallback;
    }

    int value = item->valueint;
    if (value < static_cast<int>(minimum))
    {
        value = minimum;
    }
    if (value > static_cast<int>(maximum))
    {
        value = maximum;
    }
    return static_cast<uint16_t>(value);
}

void parse_expression_names(cJSON *visual, prototracer::VisualConfig *out)
{
    if (visual == nullptr || out == nullptr)
    {
        return;
    }

    cJSON *names = cJSON_GetObjectItemCaseSensitive(visual, "expression_names");
    if (!cJSON_IsArray(names))
    {
        names = cJSON_GetObjectItemCaseSensitive(visual, "expressions");
    }
    if (!cJSON_IsArray(names))
    {
        return;
    }

    out->expression_names.clear();
    int index = 0;
    cJSON *item = nullptr;
    cJSON_ArrayForEach(item, names)
    {
        if (index >= 64)
        {
            break;
        }

        if (cJSON_IsString(item) && item->valuestring != nullptr)
        {
            out->expression_names.emplace_back(item->valuestring);
        }
        else if (cJSON_IsObject(item))
        {
            out->expression_names.emplace_back(json_string(item, "name", ""));
        }
        else
        {
            out->expression_names.emplace_back();
        }
        ++index;
    }
}
} // namespace

namespace prototracer
{
std::string derive_device_id()
{
    uint8_t mac[6] = {};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);

    char buffer[13] = {};
    std::snprintf(
        buffer,
        sizeof(buffer),
        "%02x%02x%02x%02x%02x%02x",
        mac[0],
        mac[1],
        mac[2],
        mac[3],
        mac[4],
        mac[5]);
    return std::string(buffer);
}

esp_err_t build_failsafe_config(ResolvedConfig *out)
{
    if (out == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }

    *out = {};
    out->controller.device_id = derive_device_id();
    out->controller.display_name = "ProtoTracer Remote";
    out->controller.hardware_revision = "proto-a0";
    out->controller.ui_language = UiLanguage::English;
    out->controller.network.hostname_prefix = "ptr-remote";
    out->controller.network.provisioning_ap_prefix = "ProtoTracer-Remote";
    out->controller.pairing.discovery_transport = "ble";
    out->controller.pairing.service_uuid = CONFIG_PROTOTRACER_MAINBOARD_SERVICE_UUID;
    out->controller.pairing.rx_characteristic_uuid = CONFIG_PROTOTRACER_MAINBOARD_RX_CHARACTERISTIC_UUID;
    out->controller.pairing.tx_characteristic_uuid = CONFIG_PROTOTRACER_MAINBOARD_TX_CHARACTERISTIC_UUID;
    out->controller.pairing.config_endpoint = "/api/remote/config";
    out->controller.repo.auth_token_nvs_key = "repo_access_token";
    out->controller.repo.auth_scheme = "bearer";
    out->controller.visual.animation_asset = "config/default_manifest.json";
    out->controller.visual.animation_name = "Protogen";
    out->controller.visual.expression_count = 17;
    out->controller.display.oled_brightness = 192;
    out->controller.display.oled_timeout_seconds = 30;
    out->source = ConfigSourceKind::Failsafe;
    out->note = "Compiled-in failsafe configuration";
    return ESP_OK;
}

esp_err_t parse_manifest_json(const char *json, const ConfigSourceKind source, ResolvedConfig *out)
{
    if (json == nullptr || out == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }

    const esp_err_t fallback_err = build_failsafe_config(out);
    if (fallback_err != ESP_OK)
    {
        return fallback_err;
    }

    cJSON *root = cJSON_Parse(json);
    if (root == nullptr)
    {
        ESP_LOGE(TAG, "Failed to parse manifest JSON");
        return ESP_ERR_INVALID_RESPONSE;
    }

    cJSON *device = cJSON_GetObjectItemCaseSensitive(root, "device");
    if (cJSON_IsObject(device))
    {
        out->controller.display_name = json_string(device, "display_name", out->controller.display_name.c_str());
        out->controller.hardware_revision = json_string(device, "hardware_revision", out->controller.hardware_revision.c_str());
        out->controller.ui_language = ui_language_from_string(json_string(device, "language", ui_language_name(out->controller.ui_language)));
    }

    cJSON *network = cJSON_GetObjectItemCaseSensitive(root, "network");
    if (cJSON_IsObject(network))
    {
        out->controller.network.hostname_prefix = json_string(network, "hostname_prefix", out->controller.network.hostname_prefix.c_str());
        out->controller.network.provisioning_ap_prefix = json_string(network, "provisioning_ap_prefix", out->controller.network.provisioning_ap_prefix.c_str());
        out->controller.network.station_ssid = json_string(network, "station_ssid", out->controller.network.station_ssid.c_str());
        out->controller.network.station_password = json_string(network, "station_password", out->controller.network.station_password.c_str());
    }

    cJSON *repo = cJSON_GetObjectItemCaseSensitive(root, "repo");
    if (cJSON_IsObject(repo))
    {
        out->controller.repo.manifest_url = json_string(repo, "manifest_url", out->controller.repo.manifest_url.c_str());
        out->controller.repo.asset_base_url = json_string(repo, "asset_base_url", out->controller.repo.asset_base_url.c_str());
        out->controller.repo.auth_token_nvs_key = json_string(repo, "auth_token_nvs_key", out->controller.repo.auth_token_nvs_key.c_str());
        out->controller.repo.auth_scheme = json_string(repo, "auth_scheme", out->controller.repo.auth_scheme.c_str());
        out->controller.repo.accept_header = json_string(repo, "accept_header", out->controller.repo.accept_header.c_str());
    }

    cJSON *pairing = cJSON_GetObjectItemCaseSensitive(root, "pairing");
    if (cJSON_IsObject(pairing))
    {
        out->controller.pairing.discovery_transport = json_string(pairing, "transport", out->controller.pairing.discovery_transport.c_str());
        out->controller.pairing.service_uuid = json_string(pairing, "service_uuid", out->controller.pairing.service_uuid.c_str());
        out->controller.pairing.rx_characteristic_uuid = json_string_first(pairing, "rx_uuid", "ble_rx_uuid", out->controller.pairing.rx_characteristic_uuid.c_str());
        out->controller.pairing.tx_characteristic_uuid = json_string_first(pairing, "tx_uuid", "ble_tx_uuid", out->controller.pairing.tx_characteristic_uuid.c_str());
        out->controller.pairing.config_endpoint = json_string(pairing, "config_endpoint", out->controller.pairing.config_endpoint.c_str());
        out->controller.pairing.bound_peer_id = json_string(pairing, "bound_peer_id", out->controller.pairing.bound_peer_id.c_str());
    }

    cJSON *visual = cJSON_GetObjectItemCaseSensitive(root, "visual");
    if (cJSON_IsObject(visual))
    {
        out->controller.visual.animation_asset = json_string(visual, "animation_asset", out->controller.visual.animation_asset.c_str());
        out->controller.visual.animation_name = json_string_first(visual, "animation_name", "user", out->controller.visual.animation_name.c_str());
        out->controller.visual.expression_count = json_uint8(visual, "expression_count", out->controller.visual.expression_count, 1, 64);
        parse_expression_names(visual, &out->controller.visual);
        out->controller.visual.red = json_color(visual, "red", out->controller.visual.red);
        out->controller.visual.green = json_color(visual, "green", out->controller.visual.green);
        out->controller.visual.blue = json_color(visual, "blue", out->controller.visual.blue);
    }

    cJSON *features = cJSON_GetObjectItemCaseSensitive(root, "features");
    if (cJSON_IsObject(features))
    {
        out->controller.features.enable_gesture = json_bool(features, "enable_gesture", out->controller.features.enable_gesture);
        out->controller.features.enable_imu = json_bool(features, "enable_imu", out->controller.features.enable_imu);
        out->controller.features.enable_shake_random = json_bool(features, "enable_shake_random", out->controller.features.enable_shake_random);
        out->controller.features.enable_ws2812 = json_bool(features, "enable_ws2812", out->controller.features.enable_ws2812);
        out->controller.features.enable_low_power_core = json_bool(features, "enable_low_power_core", out->controller.features.enable_low_power_core);
        out->controller.features.allow_manual_fs_upload = json_bool(features, "allow_manual_fs_upload", out->controller.features.allow_manual_fs_upload);
    }

    cJSON *display = cJSON_GetObjectItemCaseSensitive(root, "display");
    if (cJSON_IsObject(display))
    {
        out->controller.display.oled_brightness = json_uint8(display, "oled_brightness", out->controller.display.oled_brightness, 16, 255);
        out->controller.display.oled_timeout_seconds = json_uint16(display, "oled_timeout_seconds", out->controller.display.oled_timeout_seconds, 0, 600);
    }

    out->controller.device_id = derive_device_id();
    out->source = source;
    out->note = "Manifest parsed successfully";

    cJSON_Delete(root);
    return ESP_OK;
}
} // namespace prototracer