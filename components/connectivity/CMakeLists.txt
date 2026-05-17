idf_component_register(
    SRCS "connectivity_services.cpp" "main_board_ble_client.cpp"
    INCLUDE_DIRS "include"
    REQUIRES common board_support bt esp_wifi esp_event esp_netif nvs_flash esp_http_client esp_http_server app_update cjson mbedtls
)