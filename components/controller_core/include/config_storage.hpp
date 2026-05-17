#pragma once

#include "esp_err.h"
#include "prototracer_types.hpp"

namespace prototracer
{
class ConfigStorage
{
public:
    static constexpr const char *kMountPath = "/storage";
    static constexpr const char *kManifestPath = "/storage/config/default_manifest.json";
    static constexpr const char *kControllerNamespace = "controller";

    esp_err_t init();
    esp_err_t load_filesystem_image_config(ResolvedConfig *out) const;
    esp_err_t persist_active_config(const ResolvedConfig &config) const;
    esp_err_t factory_reset() const;

private:
    bool mounted_ = false;
};
} // namespace prototracer