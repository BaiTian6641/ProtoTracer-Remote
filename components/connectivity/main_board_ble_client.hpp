#pragma once

#include <cstddef>
#include <string>

#include "esp_err.h"
#include "prototracer_types.hpp"

namespace prototracer
{

/// Callback invoked when the main board sends a JSON notification (e.g. control.state or pong).
/// The callback runs on the NimBLE host task — keep it short; queue work for the main loop if needed.
using MainBoardNotificationCallback = void (*)(const char *json, size_t length, void *user_data);

esp_err_t init_main_board_ble_client();
esp_err_t scan_main_board_ble_candidates(const ControllerConfig &seed, BlePeerCandidate *out_candidates, size_t max_candidates, size_t *out_count);
esp_err_t pull_from_main_board_ble(const ControllerConfig &seed, ResolvedConfig *out);
esp_err_t send_main_board_ble_command(const char *payload);
bool get_last_main_board_ble_binding(std::string *out);
bool get_main_board_ble_signal_strength(uint8_t *out_percent);

/// Register a callback to receive all JSON notifications from the main board's TX characteristic.
/// Pass nullptr to unregister. The callback fires for every complete JSON object received.
void set_main_board_ble_notification_callback(MainBoardNotificationCallback callback, void *user_data);

/// Retrieve the latest state snapshot synced from main-board control.state notifications.
/// Returns true if state has been received at least once since the last BLE connection.
bool get_main_board_ble_state(MainBoardState *out);

} // namespace prototracer
