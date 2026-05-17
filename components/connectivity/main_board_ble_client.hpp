#pragma once

#include <string>

#include "esp_err.h"
#include "prototracer_types.hpp"

namespace prototracer
{
esp_err_t init_main_board_ble_client();
esp_err_t pull_from_main_board_ble(const ControllerConfig &seed, ResolvedConfig *out);
esp_err_t send_main_board_ble_command(const char *payload);
bool get_last_main_board_ble_binding(std::string *out);
bool get_main_board_ble_signal_strength(uint8_t *out_percent);
} // namespace prototracer
