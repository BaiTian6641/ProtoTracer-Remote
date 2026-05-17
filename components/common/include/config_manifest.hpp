#pragma once

#include <string>

#include "esp_err.h"
#include "prototracer_types.hpp"

namespace prototracer
{
std::string derive_device_id();
esp_err_t build_failsafe_config(ResolvedConfig *out);
esp_err_t parse_manifest_json(const char *json, ConfigSourceKind source, ResolvedConfig *out);
} // namespace prototracer