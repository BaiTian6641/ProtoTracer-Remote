#include "main_board_ble_client.hpp"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <string>

#include "config_manifest.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "sdkconfig.h"

#if CONFIG_BT_NIMBLE_ENABLED
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_hs_mbuf.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "nimble/ble.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "store/config/ble_store_config.h"

extern "C" void ble_store_config_init(void);
#endif

namespace prototracer
{
namespace
{
constexpr const char *TAG = "main_board_ble";
constexpr EventBits_t kEventSynced = BIT0;
constexpr EventBits_t kEventReady = BIT1;
constexpr EventBits_t kEventFailed = BIT2;
constexpr EventBits_t kEventConfigPayload = BIT3;
constexpr EventBits_t kEventWriteComplete = BIT4;
constexpr EventBits_t kEventWriteFailed = BIT5;
constexpr TickType_t kSyncTimeout = pdMS_TO_TICKS(5000);
constexpr TickType_t kOperationTimeout = pdMS_TO_TICKS(12000);
constexpr TickType_t kConfigTimeout = pdMS_TO_TICKS(2500);
constexpr TickType_t kWriteChunkTimeout = pdMS_TO_TICKS(1500);
constexpr int kScanDurationMs = 8000;
constexpr int kConnectTimeoutMs = 10000;
constexpr size_t kMaxConfigPayloadBytes = 6144;
constexpr size_t kBleWriteChunkBytes = 20;

#if CONFIG_BT_NIMBLE_ENABLED
struct MainBoardBleClient
{
    EventGroupHandle_t events = nullptr;
    bool initialized = false;
    bool operation_active = false;
    bool connecting = false;
    bool ready = false;
    bool service_found = false;
    bool rx_found = false;
    bool tx_found = false;
    bool notifications_enabled = false;
    uint8_t own_addr_type = BLE_OWN_ADDR_PUBLIC;
    uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;
    uint16_t service_start_handle = 0;
    uint16_t service_end_handle = 0;
    uint16_t rx_value_handle = 0;
    uint16_t tx_value_handle = 0;
    uint16_t tx_cccd_handle = 0;
    uint8_t tx_properties = 0;
    ble_addr_t peer_addr = {};
    ble_uuid128_t service_uuid = {};
    bool peer_filter_enabled = false;
    uint8_t peer_filter[6] = {};
    bool peer_name_filter_enabled = false;
    std::string peer_name_filter;
    std::string peer_name;
    int8_t last_rssi = -100;
    bool last_rssi_valid = false;
    bool config_request_in_flight = false;
    std::string config_payload;
    std::string last_error;
    int write_status = 0;
};

MainBoardBleClient s_client;

int hex_value(const char c)
{
    if (c >= '0' && c <= '9')
    {
        return c - '0';
    }
    if (c >= 'a' && c <= 'f')
    {
        return 10 + (c - 'a');
    }
    if (c >= 'A' && c <= 'F')
    {
        return 10 + (c - 'A');
    }
    return -1;
}

bool parse_uuid128(const std::string &text, ble_uuid128_t *out)
{
    if (out == nullptr)
    {
        return false;
    }

    std::string hex;
    hex.reserve(32);
    for (const char ch : text)
    {
        if (std::isxdigit(static_cast<unsigned char>(ch)) != 0)
        {
            hex.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(ch))));
        }
    }

    if (hex.size() != 32)
    {
        return false;
    }

    ble_uuid128_t uuid = {};
    uuid.u.type = BLE_UUID_TYPE_128;
    for (size_t index = 0; index < 16; ++index)
    {
        const int upper = hex_value(hex[index * 2]);
        const int lower = hex_value(hex[(index * 2) + 1]);
        if (upper < 0 || lower < 0)
        {
            return false;
        }
        uuid.value[15 - index] = static_cast<uint8_t>((upper << 4) | lower);
    }

    *out = uuid;
    return true;
}

bool parse_peer_address(const std::string &text, uint8_t out[6])
{
    if (out == nullptr)
    {
        return false;
    }

    unsigned int bytes[6] = {};
    if (std::sscanf(text.c_str(), "%2x:%2x:%2x:%2x:%2x:%2x", &bytes[5], &bytes[4], &bytes[3], &bytes[2], &bytes[1], &bytes[0]) != 6)
    {
        return false;
    }

    for (size_t index = 0; index < 6; ++index)
    {
        out[index] = static_cast<uint8_t>(bytes[index]);
    }
    return true;
}

std::string peer_address_string(const ble_addr_t &address)
{
    char buffer[18] = {};
    std::snprintf(
        buffer,
        sizeof(buffer),
        "%02X:%02X:%02X:%02X:%02X:%02X",
        address.val[5],
        address.val[4],
        address.val[3],
        address.val[2],
        address.val[1],
        address.val[0]);
    return std::string(buffer);
}

bool has_peer_address(const ble_addr_t &address)
{
    for (const uint8_t value : address.val)
    {
        if (value != 0)
        {
            return true;
        }
    }
    return false;
}

uint8_t rssi_to_percent(const int8_t rssi)
{
    constexpr int kWeakRssi = -95;
    constexpr int kStrongRssi = -45;
    const int clamped = std::clamp<int>(rssi, kWeakRssi, kStrongRssi);
    return static_cast<uint8_t>(((clamped - kWeakRssi) * 100) / (kStrongRssi - kWeakRssi));
}

std::string current_bound_peer_id()
{
    if (!s_client.peer_name.empty())
    {
        return s_client.peer_name;
    }
    if (has_peer_address(s_client.peer_addr))
    {
        return peer_address_string(s_client.peer_addr);
    }
    return {};
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

std::string advertisement_name(const ble_hs_adv_fields &fields)
{
    if (fields.name == nullptr || fields.name_len == 0)
    {
        return {};
    }

    return std::string(reinterpret_cast<const char *>(fields.name), fields.name_len);
}

void set_failure(const char *message, const int rc = 0)
{
    if (message != nullptr)
    {
        s_client.last_error = rc == 0 ? std::string(message) : (std::string(message) + " rc=" + std::to_string(rc));
        if (rc == 0)
        {
            ESP_LOGW(TAG, "%s", message);
        }
        else
        {
            ESP_LOGW(TAG, "%s (rc=%d)", message, rc);
        }
    }
    xEventGroupSetBits(s_client.events, kEventFailed);
}

void clear_operation_state()
{
    s_client.operation_active = false;
    s_client.connecting = false;
    s_client.ready = false;
    s_client.service_found = false;
    s_client.rx_found = false;
    s_client.tx_found = false;
    s_client.notifications_enabled = false;
    s_client.service_start_handle = 0;
    s_client.service_end_handle = 0;
    s_client.rx_value_handle = 0;
    s_client.tx_value_handle = 0;
    s_client.tx_cccd_handle = 0;
    s_client.tx_properties = 0;
    s_client.peer_name.clear();
    s_client.config_request_in_flight = false;
    s_client.config_payload.clear();
    s_client.last_error.clear();
    xEventGroupClearBits(s_client.events, kEventReady | kEventFailed | kEventConfigPayload | kEventWriteComplete | kEventWriteFailed);
}

void reset_config_exchange()
{
    s_client.config_request_in_flight = false;
    s_client.config_payload.clear();
    xEventGroupClearBits(s_client.events, kEventConfigPayload | kEventFailed | kEventWriteComplete | kEventWriteFailed);
}

void disconnect_if_needed()
{
    if (s_client.conn_handle != BLE_HS_CONN_HANDLE_NONE)
    {
        ble_gap_terminate(s_client.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }
}

bool advertisement_matches(const ble_gap_disc_desc *disc, std::string *matched_name)
{
    if (disc == nullptr)
    {
        return false;
    }

    if (disc->event_type != BLE_HCI_ADV_RPT_EVTYPE_ADV_IND && disc->event_type != BLE_HCI_ADV_RPT_EVTYPE_DIR_IND)
    {
        return false;
    }

    struct ble_hs_adv_fields fields = {};
    if (ble_hs_adv_parse_fields(&fields, disc->data, disc->length_data) != 0)
    {
        return false;
    }

    const std::string advertised_name = advertisement_name(fields);
    if (matched_name != nullptr)
    {
        *matched_name = advertised_name;
    }

    bool service_match = false;

    for (uint8_t index = 0; index < fields.num_uuids128; ++index)
    {
        if (ble_uuid_cmp(&fields.uuids128[index].u, &s_client.service_uuid.u) == 0)
        {
            service_match = true;
            break;
        }
    }

    if (!service_match)
    {
        return false;
    }

    if (s_client.peer_filter_enabled)
    {
        return std::memcmp(disc->addr.val, s_client.peer_filter, sizeof(s_client.peer_filter)) == 0;
    }

    if (s_client.peer_name_filter_enabled)
    {
        return !advertised_name.empty() && lowercase_ascii(advertised_name) == lowercase_ascii(s_client.peer_name_filter);
    }

    return true;
}

std::string extract_json_payload(const std::string &source)
{
    const size_t start = source.find('{');
    if (start == std::string::npos)
    {
        return {};
    }

    bool in_string = false;
    bool escaped = false;
    int depth = 0;
    for (size_t index = start; index < source.size(); ++index)
    {
        const char current = source[index];
        if (escaped)
        {
            escaped = false;
            continue;
        }

        if (current == '\\')
        {
            escaped = in_string;
            continue;
        }

        if (current == '"')
        {
            in_string = !in_string;
            continue;
        }

        if (in_string)
        {
            continue;
        }

        if (current == '{')
        {
            ++depth;
        }
        else if (current == '}')
        {
            --depth;
            if (depth == 0)
            {
                return source.substr(start, (index - start) + 1);
            }
        }
    }

    return {};
}

void append_config_payload_chunk(const uint8_t *data, const size_t length)
{
    if (data == nullptr || length == 0)
    {
        return;
    }

    if ((s_client.config_payload.size() + length) > kMaxConfigPayloadBytes)
    {
        set_failure("Main-board config payload exceeded limit");
        return;
    }

    s_client.config_payload.append(reinterpret_cast<const char *>(data), length);
    const std::string json = extract_json_payload(s_client.config_payload);
    if (!json.empty())
    {
        s_client.config_payload = json;
        s_client.config_request_in_flight = false;
        xEventGroupSetBits(s_client.events, kEventConfigPayload);
    }
}

std::string build_config_request(const prototracer::ControllerConfig &seed)
{
    const std::string endpoint = seed.pairing.config_endpoint.empty() ? std::string("/api/remote/config") : seed.pairing.config_endpoint;
    return std::string("{\"op\":\"config.get\",\"endpoint\":\"") + endpoint + "\",\"device_id\":\"" + seed.device_id + "\"}\n";
}

int on_rx_chunk_written(uint16_t conn_handle, const struct ble_gatt_error *error, struct ble_gatt_attr *attr, void *arg)
{
    (void)conn_handle;
    (void)attr;
    (void)arg;

    if (error != nullptr && error->status == 0)
    {
        s_client.write_status = 0;
        xEventGroupSetBits(s_client.events, kEventWriteComplete);
        return 0;
    }

    s_client.write_status = error != nullptr ? error->status : -1;
    xEventGroupSetBits(s_client.events, kEventWriteFailed);
    return 0;
}

esp_err_t write_rx_payload_chunks(const char *label, const char *payload, const size_t payload_size)
{
    if (payload == nullptr || payload_size == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    for (size_t offset = 0; offset < payload_size; offset += kBleWriteChunkBytes)
    {
        const size_t chunk_length = std::min(kBleWriteChunkBytes, payload_size - offset);
        xEventGroupClearBits(s_client.events, kEventWriteComplete | kEventWriteFailed);
        s_client.write_status = 0;

        ESP_LOGI(
            TAG,
            "Main-board BLE %s chunk write: offset=%u bytes=%u/%u text='%.*s'",
            label != nullptr ? label : "payload",
            static_cast<unsigned>(offset),
            static_cast<unsigned>(chunk_length),
            static_cast<unsigned>(payload_size),
            static_cast<int>(chunk_length),
            payload + offset);

        const int rc = ble_gattc_write_flat(
            s_client.conn_handle,
            s_client.rx_value_handle,
            payload + offset,
            chunk_length,
            on_rx_chunk_written,
            nullptr);
        if (rc != 0)
        {
            ESP_LOGW(TAG, "Failed to write BLE %s chunk at offset %u (rc=%d)", label != nullptr ? label : "payload", static_cast<unsigned>(offset), rc);
            return ESP_FAIL;
        }

        const EventBits_t bits = xEventGroupWaitBits(
            s_client.events,
            kEventWriteComplete | kEventWriteFailed,
            pdTRUE,
            pdFALSE,
            kWriteChunkTimeout);

        if ((bits & kEventWriteComplete) == 0)
        {
            ESP_LOGW(
                TAG,
                "BLE %s chunk write did not complete at offset %u (bits=0x%lx status=%d)",
                label != nullptr ? label : "payload",
                static_cast<unsigned>(offset),
                static_cast<unsigned long>(bits),
                s_client.write_status);
            return (bits & kEventWriteFailed) != 0 ? ESP_FAIL : ESP_ERR_TIMEOUT;
        }

        vTaskDelay(pdMS_TO_TICKS(15));
    }

    return ESP_OK;
}

esp_err_t request_config_payload(const ControllerConfig &seed, ResolvedConfig *out)
{
    if (out == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_client.ready || s_client.conn_handle == BLE_HS_CONN_HANDLE_NONE || s_client.rx_value_handle == 0 || !s_client.notifications_enabled)
    {
        return ESP_ERR_INVALID_STATE;
    }

    reset_config_exchange();
    const std::string request = build_config_request(seed);
    s_client.config_request_in_flight = true;

    const esp_err_t write_err = write_rx_payload_chunks("config", request.data(), request.size());
    if (write_err != ESP_OK)
    {
        s_client.config_request_in_flight = false;
        set_failure("Failed to send main-board config request", write_err);
        return write_err;
    }
    ESP_LOGI(TAG, "Requested config payload from main board");

    const EventBits_t bits = xEventGroupWaitBits(
        s_client.events,
        kEventConfigPayload | kEventFailed,
        pdFALSE,
        pdFALSE,
        kConfigTimeout);

    if ((bits & kEventConfigPayload) == 0)
    {
        s_client.config_request_in_flight = false;
        return (bits & kEventFailed) != 0 ? ESP_FAIL : ESP_ERR_TIMEOUT;
    }

    const esp_err_t parse_err = parse_manifest_json(s_client.config_payload.c_str(), ConfigSourceKind::MainBoard, out);
    if (parse_err != ESP_OK)
    {
        set_failure("Main-board config payload was not valid manifest JSON", parse_err);
        return parse_err;
    }

    out->controller.pairing.bound_peer_id = current_bound_peer_id();
    out->note = "main-board manifest over BLE";
    return ESP_OK;
}

int on_subscribe_complete(uint16_t conn_handle, const struct ble_gatt_error *error, struct ble_gatt_attr *attr, void *arg)
{
    (void)conn_handle;
    (void)attr;
    (void)arg;

    if (error != nullptr && error->status == 0)
    {
        s_client.notifications_enabled = true;
        ESP_LOGI(TAG, "Subscribed to main-board TX notifications");
    }
    else
    {
        ESP_LOGW(TAG, "TX notification subscribe did not complete cleanly");
    }

    s_client.ready = true;
    xEventGroupSetBits(s_client.events, kEventReady);
    return 0;
}

void finalize_discovery()
{
    if ((s_client.tx_properties & BLE_GATT_CHR_PROP_NOTIFY) == 0)
    {
        s_client.ready = true;
        xEventGroupSetBits(s_client.events, kEventReady);
        return;
    }

    s_client.tx_cccd_handle = static_cast<uint16_t>(s_client.tx_value_handle + 1);
    const uint8_t notify_enable[2] = {0x01, 0x00};
    const int rc = ble_gattc_write_flat(
        s_client.conn_handle,
        s_client.tx_cccd_handle,
        notify_enable,
        sizeof(notify_enable),
        on_subscribe_complete,
        nullptr);
    if (rc != 0)
    {
        ESP_LOGW(TAG, "Failed to enable main-board TX notifications (rc=%d)", rc);
        s_client.ready = true;
        xEventGroupSetBits(s_client.events, kEventReady);
    }
}

int on_characteristic_discovery(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_chr *chr, void *arg)
{
    (void)arg;

    if (error == nullptr)
    {
        set_failure("Characteristic discovery returned null error context");
        return 0;
    }

    if (error->status == 0 && chr != nullptr)
    {
        const bool writable = (chr->properties & (BLE_GATT_CHR_PROP_WRITE | BLE_GATT_CHR_PROP_WRITE_NO_RSP)) != 0;
        const bool notifiable = (chr->properties & (BLE_GATT_CHR_PROP_NOTIFY | BLE_GATT_CHR_PROP_INDICATE)) != 0;

        if (!s_client.rx_found && writable)
        {
            s_client.rx_found = true;
            s_client.rx_value_handle = chr->val_handle;
        }

        if (!s_client.tx_found && notifiable)
        {
            s_client.tx_found = true;
            s_client.tx_value_handle = chr->val_handle;
            s_client.tx_properties = chr->properties;
        }
        return 0;
    }

    if (error->status == BLE_HS_EDONE)
    {
        if (!s_client.rx_found)
        {
            set_failure("Main-board writable RX characteristic not found");
            disconnect_if_needed();
            return 0;
        }

        if (!s_client.tx_found)
        {
            set_failure("Main-board notify TX characteristic not found");
            disconnect_if_needed();
            return 0;
        }

        ESP_LOGI(TAG,
             "Discovered main-board BLE service and UART characteristics rx_handle=%u tx_handle=%u tx_cccd=%u",
             static_cast<unsigned>(s_client.rx_value_handle),
             static_cast<unsigned>(s_client.tx_value_handle),
             static_cast<unsigned>(s_client.tx_cccd_handle));
        finalize_discovery();
        return 0;
    }

    set_failure("Characteristic discovery failed", error->status);
    disconnect_if_needed();
    return 0;
}

int on_service_discovery(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_svc *service, void *arg)
{
    (void)arg;

    if (error == nullptr)
    {
        set_failure("Service discovery returned null error context");
        return 0;
    }

    if (error->status == 0 && service != nullptr)
    {
        s_client.service_found = true;
        s_client.service_start_handle = service->start_handle;
        s_client.service_end_handle = service->end_handle;
        return 0;
    }

    if (error->status == BLE_HS_EDONE)
    {
        if (!s_client.service_found)
        {
            set_failure("Main-board BLE service not found");
            disconnect_if_needed();
            return 0;
        }

        const int rc = ble_gattc_disc_all_chrs(
            conn_handle,
            s_client.service_start_handle,
            s_client.service_end_handle,
            on_characteristic_discovery,
            nullptr);
        if (rc != 0)
        {
            set_failure("Failed to start characteristic discovery", rc);
            disconnect_if_needed();
        }
        return 0;
    }

    set_failure("Service discovery failed", error->status);
    disconnect_if_needed();
    return 0;
}

int gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;

    switch (event->type)
    {
    case BLE_GAP_EVENT_DISC:
        if (s_client.operation_active)
        {
            std::string advertised_name;
            if (!advertisement_matches(&event->disc, &advertised_name))
            {
                return 0;
            }

            const int cancel_rc = ble_gap_disc_cancel();
            if (cancel_rc != 0 && cancel_rc != BLE_HS_EALREADY)
            {
                set_failure("Failed to cancel scan before connect", cancel_rc);
                return 0;
            }

            const int connect_rc = ble_gap_connect(s_client.own_addr_type, &event->disc.addr, kConnectTimeoutMs, nullptr, gap_event, nullptr);
            if (connect_rc != 0)
            {
                set_failure("Failed to connect to main board", connect_rc);
                return 0;
            }

            s_client.connecting = true;
            s_client.peer_addr = event->disc.addr;
            s_client.peer_name = advertised_name;
            s_client.last_rssi = event->disc.rssi;
            s_client.last_rssi_valid = true;
            ESP_LOGI(TAG,
                     "Connecting to main board '%s' at %s RSSI=%d dBm",
                     s_client.peer_name.empty() ? "unknown" : s_client.peer_name.c_str(),
                     peer_address_string(s_client.peer_addr).c_str(),
                     static_cast<int>(s_client.last_rssi));
        }
        return 0;

    case BLE_GAP_EVENT_DISC_COMPLETE:
        if (s_client.operation_active && !s_client.connecting && s_client.conn_handle == BLE_HS_CONN_HANDLE_NONE)
        {
            set_failure("Main-board BLE scan completed without a match", event->disc_complete.reason);
        }
        return 0;

    case BLE_GAP_EVENT_CONNECT:
        s_client.connecting = false;
        if (event->connect.status != 0)
        {
            set_failure("Main-board BLE connection failed", event->connect.status);
            return 0;
        }

        s_client.conn_handle = event->connect.conn_handle;
        ESP_LOGI(TAG, "Connected to main board over BLE");
        {
            const int rc = ble_gattc_disc_svc_by_uuid(s_client.conn_handle, &s_client.service_uuid.u, on_service_discovery, nullptr);
            if (rc != 0)
            {
                set_failure("Failed to start service discovery", rc);
                disconnect_if_needed();
            }
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Main-board BLE link disconnected (reason=%d)", event->disconnect.reason);
        s_client.conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_client.connecting = false;
        s_client.ready = false;
        s_client.notifications_enabled = false;
        if (s_client.operation_active)
        {
            set_failure("Main-board BLE link disconnected during connect", event->disconnect.reason);
        }
        return 0;

    case BLE_GAP_EVENT_NOTIFY_RX:
        if (event->notify_rx.attr_handle == s_client.tx_value_handle && event->notify_rx.om != nullptr)
        {
            const uint16_t payload_len = OS_MBUF_PKTLEN(event->notify_rx.om);
            if (payload_len > 0)
            {
                std::string payload(payload_len, '\0');
                if (ble_hs_mbuf_to_flat(event->notify_rx.om, payload.data(), payload_len, nullptr) == 0)
                {
                    if (s_client.config_request_in_flight)
                    {
                        append_config_payload_chunk(reinterpret_cast<const uint8_t *>(payload.data()), payload.size());
                    }
                    ESP_LOGI(
                        TAG,
                        "Main-board notification received on handle %u (%s, %u bytes)",
                        static_cast<unsigned>(event->notify_rx.attr_handle),
                        event->notify_rx.indication ? "indication" : "notification",
                        static_cast<unsigned>(payload_len));
                    return 0;
                }
            }
        }

        ESP_LOGI(
            TAG,
            "Main-board notification received on handle %u (%s)",
            static_cast<unsigned>(event->notify_rx.attr_handle),
            event->notify_rx.indication ? "indication" : "notification");
        return 0;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "Main-board BLE MTU updated to %u", static_cast<unsigned>(event->mtu.value));
        return 0;

    default:
        return 0;
    }
}

void host_task(void *param)
{
    (void)param;
    ESP_LOGI(TAG, "NimBLE host task started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void on_reset(int reason)
{
    ESP_LOGW(TAG, "NimBLE host reset (reason=%d)", reason);
    xEventGroupClearBits(s_client.events, kEventSynced);
}

void on_sync()
{
    const int ensure_rc = ble_hs_util_ensure_addr(0);
    if (ensure_rc != 0)
    {
        set_failure("Failed to ensure BLE address", ensure_rc);
        return;
    }

    const int addr_rc = ble_hs_id_infer_auto(0, &s_client.own_addr_type);
    if (addr_rc != 0)
    {
        set_failure("Failed to infer BLE own address type", addr_rc);
        return;
    }

    xEventGroupSetBits(s_client.events, kEventSynced);
    ESP_LOGI(TAG, "NimBLE host synchronized");
}

esp_err_t start_scan()
{
    struct ble_gap_disc_params params = {};
    params.passive = 1;
    params.filter_duplicates = 1;
    params.itvl = 0;
    params.window = 0;
    params.filter_policy = 0;
    params.limited = 0;

    const int rc = ble_gap_disc(s_client.own_addr_type, kScanDurationMs, &params, gap_event, nullptr);
    if (rc != 0)
    {
        ESP_LOGW(TAG, "Failed to start BLE scan (rc=%d)", rc);
        return ESP_FAIL;
    }

    return ESP_OK;
}

#endif
} // namespace

esp_err_t init_main_board_ble_client()
{
#if !CONFIG_BT_NIMBLE_ENABLED
    ESP_LOGW(TAG, "NimBLE is not enabled in this build");
    return ESP_ERR_NOT_SUPPORTED;
#else
    if (s_client.initialized)
    {
        const EventBits_t bits = xEventGroupWaitBits(s_client.events, kEventSynced, pdFALSE, pdFALSE, 0);
        return (bits & kEventSynced) != 0 ? ESP_OK : ESP_ERR_INVALID_STATE;
    }

    s_client.events = xEventGroupCreate();
    if (s_client.events == nullptr)
    {
        return ESP_ERR_NO_MEM;
    }

    const int port_err = nimble_port_init();
    if (port_err != ESP_OK)
    {
        return ESP_FAIL;
    }

    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb = on_sync;
    ble_store_config_init();
    nimble_port_freertos_init(host_task);
    s_client.initialized = true;

    const EventBits_t bits = xEventGroupWaitBits(s_client.events, kEventSynced, pdFALSE, pdFALSE, kSyncTimeout);
    if ((bits & kEventSynced) == 0)
    {
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
#endif
}

esp_err_t pull_from_main_board_ble(const ControllerConfig &seed, ResolvedConfig *out)
{
#if !CONFIG_BT_NIMBLE_ENABLED
    (void)seed;
    (void)out;
    return ESP_ERR_NOT_SUPPORTED;
#else
    if (out == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_client.initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (!parse_uuid128(seed.pairing.service_uuid, &s_client.service_uuid))
    {
        ESP_LOGW(TAG, "Main-board BLE service UUID configuration is invalid");
        return ESP_ERR_INVALID_ARG;
    }

    s_client.peer_filter_enabled = parse_peer_address(seed.pairing.bound_peer_id, s_client.peer_filter);
    s_client.peer_name_filter_enabled = !s_client.peer_filter_enabled && !seed.pairing.bound_peer_id.empty();
    s_client.peer_name_filter = s_client.peer_name_filter_enabled ? seed.pairing.bound_peer_id : std::string();

    if (s_client.ready && s_client.conn_handle != BLE_HS_CONN_HANDLE_NONE)
    {
        const esp_err_t request_err = request_config_payload(seed, out);
        if (request_err == ESP_OK)
        {
            return ESP_OK;
        }

        ESP_LOGW(TAG, "Connected main board did not provide config payload: %s", esp_err_to_name(request_err));
        return request_err;
    }

    clear_operation_state();
    s_client.operation_active = true;

    if (start_scan() != ESP_OK)
    {
        s_client.operation_active = false;
        return ESP_FAIL;
    }

    const EventBits_t bits = xEventGroupWaitBits(s_client.events, kEventReady | kEventFailed, pdFALSE, pdFALSE, kOperationTimeout);
    s_client.operation_active = false;

    if ((bits & kEventReady) != 0 && s_client.conn_handle != BLE_HS_CONN_HANDLE_NONE)
    {
        const esp_err_t request_err = request_config_payload(seed, out);
        if (request_err == ESP_OK)
        {
            return ESP_OK;
        }

        ESP_LOGW(TAG, "Main-board BLE link ready but config fetch failed: %s", esp_err_to_name(request_err));
        return request_err;
    }

    ble_gap_disc_cancel();
    disconnect_if_needed();

    if (!s_client.last_error.empty())
    {
        ESP_LOGW(TAG, "Main-board BLE pull failed: %s", s_client.last_error.c_str());
    }
    return (bits & kEventFailed) != 0 ? ESP_ERR_NOT_FOUND : ESP_ERR_TIMEOUT;
#endif
}

esp_err_t send_main_board_ble_command(const char *payload)
{
#if !CONFIG_BT_NIMBLE_ENABLED
    (void)payload;
    return ESP_ERR_NOT_SUPPORTED;
#else
    if (payload == nullptr || payload[0] == '\0')
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_client.ready || s_client.conn_handle == BLE_HS_CONN_HANDLE_NONE || s_client.rx_value_handle == 0)
    {
        ESP_LOGW(TAG,
                 "Main-board BLE command not sent: ready=%d conn=%u rx_handle=%u payload=%s",
                 s_client.ready ? 1 : 0,
                 static_cast<unsigned>(s_client.conn_handle),
                 static_cast<unsigned>(s_client.rx_value_handle),
                 payload);
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG,
             "Main-board BLE command write: conn=%u rx_handle=%u bytes=%u payload=%s",
             static_cast<unsigned>(s_client.conn_handle),
             static_cast<unsigned>(s_client.rx_value_handle),
             static_cast<unsigned>(std::strlen(payload)),
             payload);

    const esp_err_t write_err = write_rx_payload_chunks("command", payload, std::strlen(payload));
    if (write_err != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to send BLE command to main board (%s)", esp_err_to_name(write_err));
        return write_err;
    }

    ESP_LOGI(TAG, "Main-board BLE command write confirmed");

    return ESP_OK;
#endif
}

bool get_last_main_board_ble_binding(std::string *out)
{
    if (out == nullptr)
    {
        return false;
    }

#if !CONFIG_BT_NIMBLE_ENABLED
    out->clear();
    return false;
#else
    *out = current_bound_peer_id();
    return !out->empty();
#endif
}

bool get_main_board_ble_signal_strength(uint8_t *out_percent)
{
    if (out_percent == nullptr)
    {
        return false;
    }

#if !CONFIG_BT_NIMBLE_ENABLED
    *out_percent = 0;
    return false;
#else
    if (!s_client.ready || s_client.conn_handle == BLE_HS_CONN_HANDLE_NONE || !s_client.last_rssi_valid)
    {
        *out_percent = 0;
        return false;
    }

    *out_percent = rssi_to_percent(s_client.last_rssi);
    return true;
#endif
}
} // namespace prototracer
