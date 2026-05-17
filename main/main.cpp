#include "controller_app.hpp"

#include "esp_err.h"
#include "esp_log.h"

extern "C" void app_main(void)
{
    static const char *TAG = "main";

    static prototracer::ControllerApp app;
    const esp_err_t err = app.start();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "ControllerApp failed to start: %s", esp_err_to_name(err));
    }
}