#include "connectivity_services.hpp"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <string>
#include <sys/stat.h>

#include "cJSON.h"
#include "config_manifest.hpp"
#include "esp_app_desc.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "main_board_ble_client.hpp"
#include "nvs.h"
#include "sdkconfig.h"

namespace
{
constexpr const char *TAG = "connectivity";
constexpr const char *kStorageMountPath = "/storage";
constexpr const char *kFilesystemConfigDirectory = "/storage/config";
constexpr const char *kFilesystemManifestPath = "/storage/config/default_manifest.json";
constexpr size_t kMaxFilesystemUploadBytes = 24 * 1024;
constexpr const char *kRelayManifestPath = "manifest";
constexpr const char *kDefaultRelayFirmwareFilename = "remote-firmware.bin";
constexpr int kOtaHttpTimeoutMs = 15000;
constexpr size_t kOtaChunkBytes = 1024;
bool g_manual_fs_upload_enabled = false;

struct FirmwareManifest
{
    std::string version;
    std::string file;
    std::string md5;
};

esp_err_t send_json_response(httpd_req_t *request, const char *status, const char *body);

void fill_field(char *destination, const size_t destination_size, const std::string &value)
{
    if (destination_size == 0)
    {
        return;
    }

    std::memset(destination, 0, destination_size);
    std::strncpy(destination, value.c_str(), destination_size - 1);
}

std::string load_repo_token(const std::string &key)
{
    if (key.empty())
    {
        return {};
    }

    nvs_handle_t handle = 0;
    if (nvs_open(CONFIG_PROTOTRACER_REPO_NAMESPACE, NVS_READONLY, &handle) != ESP_OK)
    {
        return {};
    }

    size_t required = 0;
    if (nvs_get_str(handle, key.c_str(), nullptr, &required) != ESP_OK || required == 0)
    {
        nvs_close(handle);
        return {};
    }

    std::string token(required, '\0');
    if (nvs_get_str(handle, key.c_str(), token.data(), &required) != ESP_OK)
    {
        nvs_close(handle);
        return {};
    }

    nvs_close(handle);
    if (!token.empty() && token.back() == '\0')
    {
        token.pop_back();
    }
    return token;
}

std::string build_auth_header_value(const prototracer::RemoteRepoConfig &repo, const std::string &token)
{
    if (token.empty())
    {
        return {};
    }

    if (repo.auth_scheme == "token")
    {
        return std::string("token ") + token;
    }

    return std::string("Bearer ") + token;
}

std::string manifest_json_string(cJSON *object, const char *key, const char *fallback)
{
    cJSON *item = cJSON_GetObjectItemCaseSensitive(object, key);
    if (cJSON_IsString(item) && item->valuestring != nullptr)
    {
        return std::string(item->valuestring);
    }
    return std::string(fallback != nullptr ? fallback : "");
}

std::string manifest_json_string_first(cJSON *object, const char *key, const char *alternate_key, const char *fallback)
{
    const std::string value = manifest_json_string(object, key, nullptr);
    if (!value.empty())
    {
        return value;
    }

    if (alternate_key != nullptr)
    {
        return manifest_json_string(object, alternate_key, fallback);
    }

    return std::string(fallback != nullptr ? fallback : "");
}

std::string append_url_path(const std::string &base, const std::string &path)
{
    if (base.empty())
    {
        return path;
    }

    if (path.empty())
    {
        return base;
    }

    if (base.back() == '/')
    {
        return path.front() == '/' ? (base + path.substr(1)) : (base + path);
    }

    return path.front() == '/' ? (base + path) : (base + "/" + path);
}

bool is_http_url(const std::string &value)
{
    return value.rfind("http://", 0) == 0 || value.rfind("https://", 0) == 0;
}

std::string lowercase_ascii(const std::string &value)
{
    std::string lowered = value;
    for (char &ch : lowered)
    {
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    }
    return lowered;
}

int next_version_component(const std::string &version, size_t *index)
{
    if (index == nullptr)
    {
        return 0;
    }

    while (*index < version.size() && !std::isdigit(static_cast<unsigned char>(version[*index])))
    {
        ++(*index);
    }

    int value = 0;
    while (*index < version.size() && std::isdigit(static_cast<unsigned char>(version[*index])))
    {
        value = (value * 10) + (version[*index] - '0');
        ++(*index);
    }

    return value;
}

int compare_versions(const std::string &current_version, const std::string &remote_version)
{
    size_t current_index = 0;
    size_t remote_index = 0;

    while (current_index < current_version.size() || remote_index < remote_version.size())
    {
        const int current_part = next_version_component(current_version, &current_index);
        const int remote_part = next_version_component(remote_version, &remote_index);
        if (current_part != remote_part)
        {
            return current_part < remote_part ? -1 : 1;
        }
    }

    return 0;
}

std::string current_firmware_version()
{
    const esp_app_desc_t *description = esp_app_get_description();
    if (description == nullptr || description->version[0] == '\0')
    {
        return "0";
    }

    return std::string(description->version);
}

bool parse_firmware_manifest_json(const std::string &json, FirmwareManifest *manifest)
{
    if (manifest == nullptr)
    {
        return false;
    }

    cJSON *root = cJSON_Parse(json.c_str());
    if (root == nullptr)
    {
        return false;
    }

    manifest->version = manifest_json_string_first(root, "version", "fwv", "");
    manifest->file = manifest_json_string_first(root, "file", "filename", "");
    if (manifest->file.empty())
    {
        manifest->file = manifest_json_string_first(root, "bin", "url", kDefaultRelayFirmwareFilename);
    }
    manifest->md5 = lowercase_ascii(manifest_json_string_first(root, "md5", "file_md5", ""));

    cJSON_Delete(root);
    return !manifest->version.empty() && !manifest->file.empty();
}

esp_err_t fetch_http_text(const std::string &url, std::string *body)
{
    if (body == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }

    esp_http_client_config_t client_config = {};
    client_config.url = url.c_str();
    client_config.timeout_ms = kOtaHttpTimeoutMs;
    client_config.disable_auto_redirect = false;

    esp_http_client_handle_t client = esp_http_client_init(&client_config);
    if (client == nullptr)
    {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK)
    {
        esp_http_client_cleanup(client);
        return err;
    }

    esp_http_client_fetch_headers(client);
    const int status = esp_http_client_get_status_code(client);
    if (status != 200)
    {
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        ESP_LOGW(TAG, "HTTP GET failed with status %d for %s", status, url.c_str());
        return ESP_ERR_HTTP_FETCH_HEADER;
    }

    body->clear();
    char buffer[256] = {};
    int bytes_read = 0;
    while ((bytes_read = esp_http_client_read(client, buffer, sizeof(buffer))) > 0)
    {
        body->append(buffer, bytes_read);
    }

    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    return body->empty() ? ESP_ERR_INVALID_RESPONSE : ESP_OK;
}

esp_err_t stage_relay_firmware_update(const prototracer::ControllerConfig &config, const FirmwareManifest &manifest, const bool reboot_on_success)
{
    if (config.repo.asset_base_url.empty())
    {
        return ESP_ERR_NOT_FOUND;
    }

    const std::string firmware_url = is_http_url(manifest.file) ? manifest.file : append_url_path(config.repo.asset_base_url, manifest.file);
    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(nullptr);
    if (update_partition == nullptr)
    {
        ESP_LOGW(TAG, "No OTA update partition is available");
        return ESP_ERR_NOT_FOUND;
    }

    esp_http_client_config_t client_config = {};
    client_config.url = firmware_url.c_str();
    client_config.timeout_ms = kOtaHttpTimeoutMs;
    client_config.disable_auto_redirect = false;

    esp_http_client_handle_t client = esp_http_client_init(&client_config);
    if (client == nullptr)
    {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK)
    {
        esp_http_client_cleanup(client);
        return err;
    }

    const int image_size = esp_http_client_fetch_headers(client);
    const int status = esp_http_client_get_status_code(client);
    if (status != 200)
    {
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        ESP_LOGW(TAG, "Firmware relay returned HTTP %d for %s", status, firmware_url.c_str());
        return ESP_ERR_HTTP_FETCH_HEADER;
    }

    esp_ota_handle_t ota_handle = 0;
    err = esp_ota_begin(update_partition, image_size > 0 ? image_size : OTA_SIZE_UNKNOWN, &ota_handle);
    if (err != ESP_OK)
    {
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return err;
    }

    char buffer[kOtaChunkBytes] = {};
    int bytes_read = 0;
    size_t written_total = 0;
    int last_percent = -1;

    while ((bytes_read = esp_http_client_read(client, buffer, sizeof(buffer))) > 0)
    {
        err = esp_ota_write(ota_handle, buffer, bytes_read);
        if (err != ESP_OK)
        {
            esp_ota_abort(ota_handle);
            esp_http_client_close(client);
            esp_http_client_cleanup(client);
            return err;
        }

        written_total += static_cast<size_t>(bytes_read);

        if (image_size > 0)
        {
            const int percent = static_cast<int>((written_total * 100U) / static_cast<size_t>(image_size));
            if (percent / 10 != last_percent / 10)
            {
                last_percent = percent;
                ESP_LOGI(TAG, "Firmware OTA progress %d%%", percent);
            }
        }
    }

    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    if (bytes_read < 0)
    {
        esp_ota_abort(ota_handle);
        return ESP_FAIL;
    }

    err = esp_ota_end(ota_handle);
    if (err != ESP_OK)
    {
        return err;
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK)
    {
        return err;
    }

    ESP_LOGI(TAG,
             "Staged firmware update to version %s on partition '%s'",
             manifest.version.c_str(),
             update_partition->label);

    if (reboot_on_success)
    {
        vTaskDelay(pdMS_TO_TICKS(250));
        esp_restart();
    }

    return ESP_OK;
}

esp_err_t firmware_upload_handler(httpd_req_t *request)
{
    auto *service = static_cast<prototracer::OtaService *>(request->user_ctx);
    if (service == nullptr)
    {
        return send_json_response(request, "500 Internal Server Error", "{\"error\":\"OTA service unavailable\"}");
    }

    bool updated = false;
    const esp_err_t err = service->check_for_relay_update(false, &updated);
    if (err == ESP_ERR_NOT_FOUND)
    {
        return send_json_response(request, "404 Not Found", "{\"error\":\"No relay base URL configured for firmware update\"}");
    }
    if (err != ESP_OK)
    {
        return send_json_response(request, "502 Bad Gateway", "{\"error\":\"Relay firmware update failed\"}");
    }

    const esp_err_t response_err = send_json_response(
        request,
        "200 OK",
        updated ? "{\"status\":\"staged\",\"reboot_required\":true}" : "{\"status\":\"current\"}");
    if (updated)
    {
        vTaskDelay(pdMS_TO_TICKS(250));
        esp_restart();
    }
    return response_err;
}

esp_err_t root_get_handler(httpd_req_t *request)
{
    static constexpr const char *kBody = "ProtoTracer Remote local update service is running. Relay-based firmware OTA and optional filesystem manifest upload are available when configured.";
    httpd_resp_set_type(request, "text/plain");
    return httpd_resp_send(request, kBody, HTTPD_RESP_USE_STRLEN);
}

esp_err_t healthz_get_handler(httpd_req_t *request)
{
    httpd_resp_set_type(request, "application/json");
    return httpd_resp_sendstr(request, "{\"status\":\"ok\"}");
}

esp_err_t send_json_response(httpd_req_t *request, const char *status, const char *body)
{
    httpd_resp_set_status(request, status);
    httpd_resp_set_type(request, "application/json");
    return httpd_resp_sendstr(request, body);
}

esp_err_t read_request_body(httpd_req_t *request, std::string *body)
{
    if (request == nullptr || body == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (request->content_len <= 0)
    {
        return ESP_ERR_INVALID_SIZE;
    }
    if (request->content_len > static_cast<int>(kMaxFilesystemUploadBytes))
    {
        return ESP_ERR_NO_MEM;
    }

    body->assign(static_cast<size_t>(request->content_len), '\0');
    size_t offset = 0;
    while (offset < body->size())
    {
        const int received = httpd_req_recv(request, body->data() + offset, body->size() - offset);
        if (received == HTTPD_SOCK_ERR_TIMEOUT)
        {
            continue;
        }
        if (received <= 0)
        {
            return ESP_FAIL;
        }
        offset += static_cast<size_t>(received);
    }

    return ESP_OK;
}

esp_err_t ensure_directory_exists(const char *path)
{
    struct stat info = {};
    if (stat(path, &info) == 0)
    {
        return S_ISDIR(info.st_mode) ? ESP_OK : ESP_ERR_INVALID_STATE;
    }

    if (mkdir(path, 0775) == 0 || errno == EEXIST)
    {
        return ESP_OK;
    }

    return ESP_FAIL;
}

esp_err_t filesystem_upload_handler(httpd_req_t *request)
{
    if (!g_manual_fs_upload_enabled)
    {
        return send_json_response(request, "403 Forbidden", "{\"error\":\"Manual filesystem upload disabled\"}");
    }

    std::string body;
    const esp_err_t read_err = read_request_body(request, &body);
    if (read_err == ESP_ERR_INVALID_SIZE)
    {
        return send_json_response(request, "400 Bad Request", "{\"error\":\"Empty request body\"}");
    }
    if (read_err == ESP_ERR_NO_MEM)
    {
        return send_json_response(request, "413 Payload Too Large", "{\"error\":\"Manifest upload exceeds limit\"}");
    }
    if (read_err != ESP_OK)
    {
        return send_json_response(request, "400 Bad Request", "{\"error\":\"Failed to read request body\"}");
    }

    prototracer::ResolvedConfig parsed = {};
    if (prototracer::parse_manifest_json(body.c_str(), prototracer::ConfigSourceKind::FilesystemImage, &parsed) != ESP_OK)
    {
        return send_json_response(request, "400 Bad Request", "{\"error\":\"Invalid manifest JSON\"}");
    }

    if (ensure_directory_exists(kStorageMountPath) != ESP_OK || ensure_directory_exists(kFilesystemConfigDirectory) != ESP_OK)
    {
        return send_json_response(request, "500 Internal Server Error", "{\"error\":\"Failed to prepare filesystem path\"}");
    }

    FILE *file = std::fopen(kFilesystemManifestPath, "wb");
    if (file == nullptr)
    {
        return send_json_response(request, "500 Internal Server Error", "{\"error\":\"Failed to open manifest file\"}");
    }

    const size_t written = std::fwrite(body.data(), 1, body.size(), file);
    std::fclose(file);
    if (written != body.size())
    {
        return send_json_response(request, "500 Internal Server Error", "{\"error\":\"Failed to persist manifest\"}");
    }

    ESP_LOGI(TAG, "Stored manual filesystem manifest upload (%u bytes)", static_cast<unsigned>(body.size()));
    return send_json_response(request, "200 OK", "{\"status\":\"stored\",\"source\":\"filesystem\"}");
}

} // namespace

namespace prototracer
{
esp_err_t NetworkManager::init()
{
    if (initialized_)
    {
        return ESP_OK;
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t wifi_init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    initialized_ = true;
    ESP_LOGI(TAG, "Wi-Fi stack initialized");
    return ESP_OK;
}

esp_err_t NetworkManager::connect_saved_station(const NetworkConfig &config, const uint32_t timeout_ms)
{
    if (config.station_ssid.empty())
    {
        ESP_LOGW(TAG, "No saved station credentials available");
        return ESP_ERR_NOT_FOUND;
    }

    wifi_config_t wifi_config = {};
    fill_field(reinterpret_cast<char *>(wifi_config.sta.ssid), sizeof(wifi_config.sta.ssid), config.station_ssid);
    fill_field(reinterpret_cast<char *>(wifi_config.sta.password), sizeof(wifi_config.sta.password), config.station_password);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    if (!wifi_started_)
    {
        ESP_ERROR_CHECK(esp_wifi_start());
        wifi_started_ = true;
    }

    station_connected_ = false;
    ESP_ERROR_CHECK(esp_wifi_connect());

    const TickType_t delay_ticks = pdMS_TO_TICKS(250);
    const TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    TickType_t elapsed = 0;
    while (elapsed < timeout_ticks)
    {
        wifi_ap_record_t ap_info = {};
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK)
        {
            station_connected_ = true;
            ESP_LOGI(TAG, "Connected to station SSID '%s'", config.station_ssid.c_str());
            return ESP_OK;
        }

        vTaskDelay(delay_ticks);
        elapsed += delay_ticks;
    }

    ESP_LOGW(TAG, "Timed out connecting to station SSID '%s'", config.station_ssid.c_str());
    return ESP_ERR_TIMEOUT;
}

esp_err_t NetworkManager::start_user_provisioning_portal(const ControllerConfig &config)
{
    wifi_config_t ap_config = {};
    std::string ap_ssid = config.network.provisioning_ap_prefix.empty() ? "ProtoTracer-Remote" : config.network.provisioning_ap_prefix;
    const std::string suffix = config.device_id.size() >= 4 ? config.device_id.substr(config.device_id.size() - 4) : config.device_id;
    ap_ssid += "-" + suffix;

    fill_field(reinterpret_cast<char *>(ap_config.ap.ssid), sizeof(ap_config.ap.ssid), ap_ssid);
    fill_field(reinterpret_cast<char *>(ap_config.ap.password), sizeof(ap_config.ap.password), CONFIG_PROTOTRACER_PROVISIONING_AP_PASSWORD);
    ap_config.ap.ssid_len = ap_ssid.size();
    ap_config.ap.max_connection = 4;
    ap_config.ap.authmode = std::strlen(CONFIG_PROTOTRACER_PROVISIONING_AP_PASSWORD) == 0 ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    if (!wifi_started_)
    {
        ESP_ERROR_CHECK(esp_wifi_start());
        wifi_started_ = true;
    }

    ESP_LOGI(TAG, "Provisioning SoftAP started with SSID '%s'", ap_ssid.c_str());
    ESP_LOGI(TAG, "This is the NetWizard-style provisioning hook for a future captive portal implementation");
    return ESP_OK;
}

bool NetworkManager::station_connected() const
{
    return station_connected_;
}

esp_err_t PairingService::init()
{
    const esp_err_t err = init_main_board_ble_client();
    if (err == ESP_ERR_NOT_SUPPORTED)
    {
        initialized_ = false;
        ESP_LOGW(TAG, "Main-board BLE client is not available in this build; source priority will fall through to repo/filesystem");
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(err, TAG, "Main-board BLE client init failed");
    initialized_ = true;
    ESP_LOGI(TAG, "Pairing service initialized");
    return ESP_OK;
}

esp_err_t PairingService::pull_from_main_board(const ControllerConfig &seed, ResolvedConfig *out)
{
    if (out == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (!initialized_)
    {
        return ESP_ERR_NOT_SUPPORTED;
    }

    return pull_from_main_board_ble(seed, out);
}

esp_err_t PairingService::send_control_command(const char *payload)
{
    if (!initialized_)
    {
        return ESP_ERR_NOT_SUPPORTED;
    }

    return send_main_board_ble_command(payload);
}

bool PairingService::get_last_main_board_binding(std::string *out) const
{
    if (!initialized_ || out == nullptr)
    {
        return false;
    }

    return get_last_main_board_ble_binding(out);
}

bool PairingService::get_signal_strength(uint8_t *out_percent) const
{
    if (!initialized_ || out_percent == nullptr)
    {
        return false;
    }

    return get_main_board_ble_signal_strength(out_percent);
}

esp_err_t RepoClient::init()
{
    initialized_ = true;
    ESP_LOGI(TAG, "Repo client initialized");
    return ESP_OK;
}

esp_err_t RepoClient::pull_from_repo(const ControllerConfig &seed, ResolvedConfig *out)
{
    if (!initialized_ || out == nullptr)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (seed.repo.manifest_url.empty())
    {
        ESP_LOGI(TAG, "No repo manifest URL configured, skipping source priority #2");
        return ESP_ERR_NOT_FOUND;
    }

    esp_http_client_config_t client_config = {};
    client_config.url = seed.repo.manifest_url.c_str();
    client_config.timeout_ms = 8000;
    client_config.disable_auto_redirect = false;

    esp_http_client_handle_t client = esp_http_client_init(&client_config);
    if (client == nullptr)
    {
        return ESP_ERR_NO_MEM;
    }

    const std::string token = load_repo_token(seed.repo.auth_token_nvs_key);
    const std::string auth_value = build_auth_header_value(seed.repo, token);
    if (!auth_value.empty())
    {
        esp_http_client_set_header(client, "Authorization", auth_value.c_str());
    }
    if (!seed.repo.accept_header.empty())
    {
        esp_http_client_set_header(client, "Accept", seed.repo.accept_header.c_str());
    }

    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK)
    {
        esp_http_client_cleanup(client);
        return err;
    }

    esp_http_client_fetch_headers(client);
    const int status = esp_http_client_get_status_code(client);
    if (status != 200)
    {
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        ESP_LOGW(TAG, "Repo manifest request failed with HTTP status %d", status);
        return ESP_ERR_HTTP_FETCH_HEADER;
    }

    std::string body;
    char buffer[256] = {};
    int bytes_read = 0;
    while ((bytes_read = esp_http_client_read(client, buffer, sizeof(buffer))) > 0)
    {
        body.append(buffer, bytes_read);
    }

    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    if (body.empty())
    {
        return ESP_ERR_INVALID_RESPONSE;
    }

    err = parse_manifest_json(body.c_str(), ConfigSourceKind::RemoteRepo, out);
    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "Loaded configuration manifest from remote repo");
    }
    return err;
}

esp_err_t OtaService::init(const ControllerConfig &config)
{
    config_ = config;
    ESP_LOGI(TAG, "OTA service initialized for device '%s'", config.display_name.c_str());
    return ESP_OK;
}

esp_err_t OtaService::start_local_update_server(const ControllerConfig &config)
{
    config_ = config;
    if (server_ != nullptr)
    {
        return ESP_OK;
    }

    httpd_config_t http_config = HTTPD_DEFAULT_CONFIG();
    http_config.server_port = CONFIG_PROTOTRACER_HTTP_PORT;

    const esp_err_t err = httpd_start(&server_, &http_config);
    if (err != ESP_OK)
    {
        return err;
    }

    httpd_uri_t root_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_get_handler,
        .user_ctx = nullptr,
    };
    httpd_register_uri_handler(server_, &root_uri);

    httpd_uri_t health_uri = {
        .uri = "/healthz",
        .method = HTTP_GET,
        .handler = healthz_get_handler,
        .user_ctx = nullptr,
    };
    httpd_register_uri_handler(server_, &health_uri);

    httpd_uri_t firmware_uri = {
        .uri = "/api/update/firmware",
        .method = HTTP_POST,
        .handler = firmware_upload_handler,
        .user_ctx = this,
    };
    httpd_register_uri_handler(server_, &firmware_uri);

    g_manual_fs_upload_enabled = config.features.allow_manual_fs_upload;
    httpd_uri_t filesystem_uri = {
        .uri = "/api/update/filesystem",
        .method = HTTP_POST,
        .handler = filesystem_upload_handler,
        .user_ctx = this,
    };
    httpd_register_uri_handler(server_, &filesystem_uri);

    ESP_LOGI(TAG, "Local update server started on port %d for '%s'", CONFIG_PROTOTRACER_HTTP_PORT, config.display_name.c_str());
    ESP_LOGI(TAG, "This replaces the ElegantOTA-style portal with an IDF-native HTTP service with relay-based OTA support");
    ESP_LOGI(TAG, "Manual filesystem upload %s", g_manual_fs_upload_enabled ? "enabled" : "disabled");
    return ESP_OK;
}

esp_err_t OtaService::check_for_relay_update(const bool reboot_on_success, bool *updated)
{
    if (updated != nullptr)
    {
        *updated = false;
    }

    if (config_.repo.asset_base_url.empty())
    {
        ESP_LOGI(TAG, "No relay firmware base URL configured, skipping firmware update check");
        return ESP_ERR_NOT_FOUND;
    }

    const std::string manifest_url = append_url_path(config_.repo.asset_base_url, kRelayManifestPath);
    std::string manifest_json;
    ESP_RETURN_ON_ERROR(fetch_http_text(manifest_url, &manifest_json), TAG, "Failed to fetch relay firmware manifest");

    FirmwareManifest manifest = {};
    if (!parse_firmware_manifest_json(manifest_json, &manifest))
    {
        ESP_LOGW(TAG, "Relay firmware manifest could not be parsed");
        return ESP_ERR_INVALID_RESPONSE;
    }

    const std::string running_version = current_firmware_version();
    if (compare_versions(running_version, manifest.version) >= 0)
    {
        ESP_LOGI(TAG,
                 "Relay firmware is current (%s); manifest reports %s",
                 running_version.c_str(),
                 manifest.version.c_str());
        return ESP_OK;
    }

    ESP_LOGI(TAG,
             "Relay firmware update available: %s -> %s",
             running_version.c_str(),
             manifest.version.c_str());
    const esp_err_t err = stage_relay_firmware_update(config_, manifest, reboot_on_success);
    if (err == ESP_OK && updated != nullptr)
    {
        *updated = true;
    }
    return err;
}
} // namespace prototracer