idf_component_register(
    SRCS "driver_services.cpp" "i2c_bus.cpp" "tca9534.cpp" "ssd1351_display.cpp" "lsm6dso.cpp" "vcnl4035.cpp" "max17055.cpp"
    INCLUDE_DIRS "include"
    REQUIRES common board_support LovyanGFX esp_adc esp_driver_gpio esp_driver_i2c esp_driver_rmt esp_driver_spi heap led_strip ulp
)

set(ulp_app_name ulp_drivers_lp_wake)
set(ulp_riscv_sources "ulp/lp_wake_main.c")
set(ulp_exp_dep_srcs "driver_services.cpp")

ulp_embed_binary(${ulp_app_name} "${ulp_riscv_sources}" "${ulp_exp_dep_srcs}")