#include "config_storage.hpp"

#include <cstdio>
#include <string>

#include "config_manifest.hpp"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "nvs.h"

namespace
{
constexpr const char *TAG = "config_storage";

std::string load_nvs_string(nvs_handle_t handle, const char *key)
{
    size_t required = 0;
    if (nvs_get_str(handle, key, nullptr, &required) != ESP_OK || required == 0)
    {
        return {};
    }

    std::string value(required, '\0');
    if (nvs_get_str(handle, key, value.data(), &required) != ESP_OK)
    {
        return {};
    }

    if (!value.empty() && value.back() == '\0')
    {
        value.pop_back();
    }
    return value;
}

void overlay_nvs_string(nvs_handle_t handle, const char *key, std::string *target)
{
    if (target == nullptr)
    {
        return;
    }

    const std::string value = load_nvs_string(handle, key);
    if (!value.empty())
    {
        *target = value;
    }
}

void overlay_nvs_u8(nvs_handle_t handle, const char *key, uint8_t *target)
{
    if (target == nullptr)
    {
        return;
    }

    uint8_t value = 0;
    if (nvs_get_u8(handle, key, &value) == ESP_OK)
    {
        *target = value;
    }
}

void overlay_nvs_u16(nvs_handle_t handle, const char *key, uint16_t *target)
{
    if (target == nullptr)
    {
        return;
    }

    uint16_t value = 0;
    if (nvs_get_u16(handle, key, &value) == ESP_OK)
    {
        *target = value;
    }
}

void overlay_nvs_bool(nvs_handle_t handle, const char *key, bool *target)
{
    if (target == nullptr)
    {
        return;
    }

    uint8_t value = 0;
    if (nvs_get_u8(handle, key, &value) == ESP_OK)
    {
        *target = value != 0;
    }
}
} // namespace

namespace prototracer
{
esp_err_t ConfigStorage::init()
{
    if (mounted_)
    {
        return ESP_OK;
    }

    esp_vfs_spiffs_conf_t conf = {};
    conf.base_path = kMountPath;
    conf.partition_label = "storage";
    conf.max_files = 8;
    conf.format_if_mount_failed = true;

    const esp_err_t err = esp_vfs_spiffs_register(&conf);
    if (err != ESP_OK)
    {
        return err;
    }

    mounted_ = true;
    ESP_LOGI(TAG, "SPIFFS mounted at %s", kMountPath);
    return ESP_OK;
}

esp_err_t ConfigStorage::load_filesystem_image_config(ResolvedConfig *out) const
{
    if (out == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = ESP_OK;

    FILE *file = std::fopen(kManifestPath, "rb");
    if (file == nullptr)
    {
        ESP_LOGW(TAG, "Manifest %s not found, using failsafe defaults", kManifestPath);
        err = build_failsafe_config(out);
    }

    if (err == ESP_OK && file != nullptr)
    {
        std::fseek(file, 0, SEEK_END);
        const long size = std::ftell(file);
        std::rewind(file);
        if (size <= 0)
        {
            std::fclose(file);
            file = nullptr;
            err = build_failsafe_config(out);
        }
        else
        {
            std::string body(static_cast<size_t>(size), '\0');
            const size_t bytes_read = std::fread(body.data(), 1, body.size(), file);
            std::fclose(file);
            file = nullptr;
            if (bytes_read != body.size())
            {
                return ESP_ERR_INVALID_SIZE;
            }

            err = parse_manifest_json(body.c_str(), ConfigSourceKind::FilesystemImage, out);
        }
    }

    if (err != ESP_OK)
    {
        return err;
    }

    nvs_handle_t handle = 0;
    if (nvs_open(kControllerNamespace, NVS_READONLY, &handle) == ESP_OK)
    {
        overlay_nvs_string(handle, "device_id", &out->controller.device_id);
        overlay_nvs_string(handle, "display_name", &out->controller.display_name);
        overlay_nvs_string(handle, "bound_peer_id", &out->controller.pairing.bound_peer_id);
        overlay_nvs_u8(handle, "oled_bright", &out->controller.display.oled_brightness);
        overlay_nvs_u16(handle, "oled_timeout", &out->controller.display.oled_timeout_seconds);
        overlay_nvs_bool(handle, "shake_random", &out->controller.features.enable_shake_random);
        nvs_close(handle);
    }

    return ESP_OK;
}

esp_err_t ConfigStorage::persist_active_config(const ResolvedConfig &config) const
{
    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open(kControllerNamespace, NVS_READWRITE, &handle);
    if (err != ESP_OK)
    {
        return err;
    }

    err = nvs_set_str(handle, "device_id", config.controller.device_id.c_str());
    if (err == ESP_OK)
    {
        err = nvs_set_str(handle, "display_name", config.controller.display_name.c_str());
    }
    if (err == ESP_OK)
    {
        err = nvs_set_u8(handle, "cfg_source", static_cast<uint8_t>(config.source));
    }
    if (err == ESP_OK)
    {
        err = nvs_set_str(handle, "bound_peer_id", config.controller.pairing.bound_peer_id.c_str());
    }
    if (err == ESP_OK)
    {
        err = nvs_set_u8(handle, "oled_bright", config.controller.display.oled_brightness);
    }
    if (err == ESP_OK)
    {
        err = nvs_set_u16(handle, "oled_timeout", config.controller.display.oled_timeout_seconds);
    }
    if (err == ESP_OK)
    {
        err = nvs_set_u8(handle, "shake_random", config.controller.features.enable_shake_random ? 1U : 0U);
    }
    if (err == ESP_OK)
    {
        err = nvs_commit(handle);
    }

    nvs_close(handle);
    return err;
}

esp_err_t ConfigStorage::factory_reset() const
{
    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open(kControllerNamespace, NVS_READWRITE, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        return ESP_OK;
    }
    if (err != ESP_OK)
    {
        return err;
    }

    err = nvs_erase_all(handle);
    if (err == ESP_OK)
    {
        err = nvs_commit(handle);
    }

    nvs_close(handle);
    return err;
}
} // namespace prototracer