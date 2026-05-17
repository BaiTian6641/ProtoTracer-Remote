#pragma once

#include <cstdint>
#include <string>

#include "esp_err.h"
#include "esp_http_server.h"
#include "prototracer_types.hpp"

namespace prototracer
{
class NetworkManager
{
public:
    esp_err_t init();
    esp_err_t connect_saved_station(const NetworkConfig &config, uint32_t timeout_ms);
    esp_err_t start_user_provisioning_portal(const ControllerConfig &config);
    bool station_connected() const;

private:
    bool initialized_ = false;
    bool wifi_started_ = false;
    bool station_connected_ = false;
};

class PairingService
{
public:
    esp_err_t init();
    esp_err_t pull_from_main_board(const ControllerConfig &seed, ResolvedConfig *out);
    esp_err_t send_control_command(const char *payload);
    bool get_last_main_board_binding(std::string *out) const;
    bool get_signal_strength(uint8_t *out_percent) const;

private:
    bool initialized_ = false;
};

class RepoClient
{
public:
    esp_err_t init();
    esp_err_t pull_from_repo(const ControllerConfig &seed, ResolvedConfig *out);

private:
    bool initialized_ = false;
};

class OtaService
{
public:
    esp_err_t init(const ControllerConfig &config);
    esp_err_t start_local_update_server(const ControllerConfig &config);
    esp_err_t check_for_relay_update(bool reboot_on_success = true, bool *updated = nullptr);

private:
    httpd_handle_t server_ = nullptr;
    ControllerConfig config_{};
};
} // namespace prototracer