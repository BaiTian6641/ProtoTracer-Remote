idf_component_register(
    SRCS "config_storage.cpp" "controller_app.cpp" "hardware_test_app.cpp"
    INCLUDE_DIRS "include"
    REQUIRES common board_support connectivity drivers nvs_flash spiffs
)