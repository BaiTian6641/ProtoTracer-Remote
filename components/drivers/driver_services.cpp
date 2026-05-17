#include "driver_services.hpp"

#include <array>
#include <cmath>

#include "i2c_bus.hpp"
#include "prototracer_board.hpp"
#include "ssd1351_display.hpp"
#include "tca9534.hpp"

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "esp_log.h"
#include "hal/gpio_hal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "led_strip.h"
#include "sdkconfig.h"

namespace
{
constexpr const char *TAG = "drivers";

constexpr uint32_t kSharedI2cFrequencyHz = 400000;
constexpr UBaseType_t kInputIrqQueueLength = 8;
constexpr UBaseType_t kInputEventQueueLength = 16;
constexpr UBaseType_t kSensorWorkQueueLength = 8;
constexpr UBaseType_t kSensorEventQueueLength = 8;
constexpr uint8_t kExpanderAllInputsMask = 0xFF;
constexpr uint32_t kInputPollIntervalMs = 10;
constexpr uint32_t kExpanderIrqSettleDelayMs = 2;
constexpr uint32_t kExpanderStableReadAttempts = 4;
constexpr int kJoystickDeadZonePercent = 32;
constexpr int kJoystickOversampleCount = 4;
constexpr uint32_t kLedStripResolutionHz = 10 * 1000 * 1000;
constexpr bool kUseExpanderIrq = true;
constexpr float kThreeCellNiMhEmptyVoltage = 2.70f;
constexpr float kThreeCellNiMhFullVoltage = 4.35f;
constexpr float kChargeCurrentDetectThresholdMa = 25.0f;

struct JoystickAdcState
{
    adc_oneshot_unit_handle_t handle = nullptr;
    adc_unit_t unit = ADC_UNIT_1;
    adc_channel_t x_channel = ADC_CHANNEL_0;
    adc_channel_t y_channel = ADC_CHANNEL_1;
    int center_x = 2048;
    int center_y = 2048;
    bool x_ok = false;
    bool x_digital = false;
    int x_digital_idle_level = 0;
    bool y_ok = false;
    bool ready = false;
};

struct InputListenerSlot
{
    prototracer::InputEventCallback callback = nullptr;
    void *context = nullptr;
};

QueueHandle_t g_input_irq_queue = nullptr;
QueueHandle_t g_input_event_queue = nullptr;
TaskHandle_t g_input_router_task = nullptr;
gpio_isr_handle_t g_input_gpio_isr_handle = nullptr;
bool g_input_gpio_isr_registered = false;
gpio_hal_context_t g_input_gpio_hal = {
    .dev = GPIO_HAL_GET_HW(0),
};
std::array<InputListenerSlot, 4> g_input_listeners = {};
prototracer::InputSnapshot g_latest_snapshot = {};

QueueHandle_t g_sensor_work_queue = nullptr;
QueueHandle_t g_sensor_event_queue = nullptr;
TaskHandle_t g_sensor_task = nullptr;
bool g_sensor_listener_attached = false;
prototracer::MotionSample g_latest_motion = {};
prototracer::GestureSample g_latest_gesture = {};
prototracer::FuelGaugeSample g_latest_battery = {};
bool g_charger_enabled = false;
bool g_last_charger_power_status = false;
bool g_last_charger_vbat_monitor = false;
prototracer::BatteryChemistry g_last_charger_chemistry = prototracer::BatteryChemistry::Unknown;
bool g_sensor_rail_output_ready = false;
JoystickAdcState g_joystick_adc = {};
led_strip_handle_t g_status_led_strip = nullptr;

esp_err_t ensure_sensor_rail_enabled();

prototracer::I2cBus &shared_i2c_bus()
{
    static prototracer::I2cBus bus;
    return bus;
}

prototracer::Tca9534 &shared_expander()
{
    static prototracer::Tca9534 expander;
    return expander;
}

prototracer::Ssd1351Display &shared_display()
{
    static prototracer::Ssd1351Display display;
    return display;
}

prototracer::Lsm6dso &shared_imu()
{
    static prototracer::Lsm6dso imu;
    return imu;
}

prototracer::Vcnl4035 &shared_gesture_sensor()
{
    static prototracer::Vcnl4035 sensor;
    return sensor;
}

prototracer::Max17055 &shared_fuel_gauge()
{
    static prototracer::Max17055 gauge;
    return gauge;
}

TickType_t expander_settle_delay_ticks()
{
    const TickType_t ticks = pdMS_TO_TICKS(kExpanderIrqSettleDelayMs);
    return ticks > 0 ? ticks : 1;
}

esp_err_t read_expander_inputs(uint8_t *state)
{
    if (state == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(ensure_sensor_rail_enabled(), TAG, "Failed to keep sensor rail asserted for expander read");
    return shared_expander().read_inputs(state);
}

esp_err_t read_expander_inputs_stable(uint8_t *state)
{
    if (state == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t previous = 0xFF;
    ESP_RETURN_ON_ERROR(read_expander_inputs(&previous), TAG, "Initial expander read failed");

    for (uint32_t attempt = 1; attempt < kExpanderStableReadAttempts; ++attempt)
    {
        vTaskDelay(expander_settle_delay_ticks());

        uint8_t current = previous;
        ESP_RETURN_ON_ERROR(read_expander_inputs(&current), TAG, "Stable expander read failed");
        if (current == previous)
        {
            *state = current;
            return ESP_OK;
        }

        previous = current;
    }

    *state = previous;
    return ESP_OK;
}

void drain_input_irq_queue()
{
    if (g_input_irq_queue == nullptr)
    {
        return;
    }

    uint32_t pending = 0;
    while (xQueueReceive(g_input_irq_queue, &pending, 0) == pdTRUE)
    {
    }
}

esp_err_t read_joystick_axis_average(const adc_oneshot_unit_handle_t handle, const adc_channel_t channel, int *out_raw)
{
    if (handle == nullptr || out_raw == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }

    int sum = 0;
    for (int sample_index = 0; sample_index < kJoystickOversampleCount; ++sample_index)
    {
        int raw = 0;
        ESP_RETURN_ON_ERROR(adc_oneshot_read(handle, channel, &raw), TAG, "Joystick ADC read failed");
        sum += raw;
    }

    *out_raw = sum / kJoystickOversampleCount;
    return ESP_OK;
}

uint32_t tick_ms_now()
{
    return static_cast<uint32_t>(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

uint64_t gpio_pin_mask(const gpio_num_t pin)
{
    if (pin == GPIO_NUM_NC)
    {
        return 0;
    }

    return 1ULL << static_cast<uint32_t>(pin);
}

esp_err_t ensure_sensor_rail_enabled()
{
    if (!g_sensor_rail_output_ready)
    {
        gpio_config_t sensor_rail = {};
        sensor_rail.pin_bit_mask = gpio_pin_mask(prototracer::board::kPinSensorRailEnable);
        sensor_rail.mode = GPIO_MODE_OUTPUT;
        sensor_rail.pull_up_en = GPIO_PULLUP_DISABLE;
        sensor_rail.pull_down_en = GPIO_PULLDOWN_DISABLE;
        sensor_rail.intr_type = GPIO_INTR_DISABLE;
        ESP_RETURN_ON_ERROR(gpio_config(&sensor_rail), TAG, "Sensor rail GPIO config failed");
        g_sensor_rail_output_ready = true;
    }

    ESP_RETURN_ON_ERROR(gpio_set_level(prototracer::board::kPinSensorRailEnable, 1), TAG, "Failed to enable sensor rail");
    return ESP_OK;
}

int normalize_axis_percent(const int raw, const int center)
{
    const int delta = raw - center;
    if (delta >= 0)
    {
        const int span = std::max(1, 4095 - center);
        return std::min(100, (delta * 100) / span);
    }

    const int span = std::max(1, center);
    return std::max(-100, (delta * 100) / span);
}

void update_latest_battery_from_sample(const prototracer::Max17055::Sample &sample)
{
    g_latest_battery.valid = true;
    g_latest_battery.voltage_v = sample.voltage_v;
    g_latest_battery.current_ma = sample.current_ma;
    g_latest_battery.average_current_ma = sample.average_current_ma;
    g_latest_battery.state_of_charge_percent = sample.state_of_charge_percent;
    g_latest_battery.reported_capacity_mah = sample.reported_capacity_mah;
    g_latest_battery.temperature_c = sample.temperature_c;
    g_latest_battery.status = sample.status;
    g_latest_battery.power_on_reset = sample.power_on_reset;
    g_latest_battery.soc_change_alert = sample.soc_change_alert;
}

void status_led_rgb_for_mode(const prototracer::StatusLedMode mode, uint8_t *red, uint8_t *green, uint8_t *blue)
{
    uint8_t out_red = 0;
    uint8_t out_green = 0;
    uint8_t out_blue = 0;

    switch (mode)
    {
    case prototracer::StatusLedMode::Booting:
        out_red = 24;
        out_green = 56;
        out_blue = 96;
        break;
    case prototracer::StatusLedMode::Provisioning:
        out_red = 112;
        out_green = 56;
        out_blue = 0;
        break;
    case prototracer::StatusLedMode::Linked:
        out_red = 0;
        out_green = 112;
        out_blue = 18;
        break;
    case prototracer::StatusLedMode::Updating:
        out_red = 0;
        out_green = 92;
        out_blue = 112;
        break;
    case prototracer::StatusLedMode::Error:
        out_red = 112;
        out_green = 0;
        out_blue = 0;
        break;
    case prototracer::StatusLedMode::Off:
    default:
        break;
    }

    if (red != nullptr)
    {
        *red = out_red;
    }
    if (green != nullptr)
    {
        *green = out_green;
    }
    if (blue != nullptr)
    {
        *blue = out_blue;
    }
}


bool is_button_bit(const prototracer::board::ExpanderInputBit bit)
{
    switch (bit)
    {
    case prototracer::board::ExpanderInputBit::JoystickButton:
    case prototracer::board::ExpanderInputBit::Button0:
    case prototracer::board::ExpanderInputBit::Button1:
    case prototracer::board::ExpanderInputBit::Button2:
        return true;
    default:
        return false;
    }
}

uint16_t display_accent_for_mode(const prototracer::StatusLedMode mode)
{
    switch (mode)
    {
    case prototracer::StatusLedMode::Provisioning:
        return 0x3FE0;
    case prototracer::StatusLedMode::Linked:
        return 0x07E0;
    case prototracer::StatusLedMode::Updating:
        return 0xFD20;
    case prototracer::StatusLedMode::Error:
        return 0xF800;
    case prototracer::StatusLedMode::Booting:
    default:
        return 0x5D9B;
    }
}

prototracer::InputEvent make_input_event(const prototracer::board::ExpanderInputBit bit, const bool asserted, const uint8_t raw_bits)
{
    prototracer::InputEvent event = {};
    event.source = bit;
    event.raw_bits = raw_bits;
    event.tick_ms = tick_ms_now();
    if (is_button_bit(bit))
    {
        event.type = asserted ? prototracer::InputEvent::Type::ButtonPressed : prototracer::InputEvent::Type::ButtonReleased;
    }
    else
    {
        event.type = asserted ? prototracer::InputEvent::Type::SensorInterruptAsserted : prototracer::InputEvent::Type::SensorInterruptReleased;
    }
    return event;
}

void queue_input_event(const prototracer::InputEvent &event)
{
    if (g_input_event_queue != nullptr)
    {
        xQueueSend(g_input_event_queue, &event, 0);
    }

    for (const InputListenerSlot &slot : g_input_listeners)
    {
        if (slot.callback != nullptr)
        {
            slot.callback(event, slot.context);
        }
    }
}

void queue_sensor_event(const prototracer::SensorEvent &event)
{
    if (g_sensor_event_queue != nullptr)
    {
        xQueueSend(g_sensor_event_queue, &event, 0);
    }
}

void log_expander_changes(const uint8_t previous, const uint8_t current)
{
    const uint8_t changed = previous ^ current;
    if (changed == 0)
    {
        return;
    }

    for (int index = 0; index < 8; ++index)
    {
        if ((changed & (1U << index)) == 0)
        {
            continue;
        }

        const auto bit = static_cast<prototracer::board::ExpanderInputBit>(index);
        const bool asserted = (current & (1U << index)) == 0;
        ESP_LOGI(TAG, "Expander input %s -> %s", prototracer::board::expander_input_name(bit), asserted ? "asserted" : "released");
        queue_input_event(make_input_event(bit, asserted, current));
    }
}

void queue_sensor_fault(const prototracer::board::ExpanderInputBit source, const esp_err_t status)
{
    prototracer::SensorEvent event = {};
    event.type = prototracer::SensorEvent::Type::Fault;
    event.tick_ms = tick_ms_now();
    event.status = status;
    event.source = source;
    queue_sensor_event(event);
}

void process_imu_interrupt(const prototracer::board::ExpanderInputBit source)
{
    if (!shared_imu().ready())
    {
        return;
    }

    prototracer::Lsm6dso::Sample sample = {};
    const esp_err_t err = shared_imu().read_sample(&sample);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "IMU sample read failed: %s", esp_err_to_name(err));
        queue_sensor_fault(source, err);
        return;
    }

    g_latest_motion.valid = true;
    g_latest_motion.x_raw = sample.x_raw;
    g_latest_motion.y_raw = sample.y_raw;
    g_latest_motion.z_raw = sample.z_raw;
    g_latest_motion.x_mg = sample.x_mg;
    g_latest_motion.y_mg = sample.y_mg;
    g_latest_motion.z_mg = sample.z_mg;
    g_latest_motion.gyro_valid = sample.gyro_valid;
    g_latest_motion.gx_raw = sample.gx_raw;
    g_latest_motion.gy_raw = sample.gy_raw;
    g_latest_motion.gz_raw = sample.gz_raw;
    g_latest_motion.gx_dps = sample.gx_dps;
    g_latest_motion.gy_dps = sample.gy_dps;
    g_latest_motion.gz_dps = sample.gz_dps;
    g_latest_motion.wake_source = sample.wake_source;
    g_latest_motion.wake_event = sample.wake_event;
    g_latest_motion.sleep_change = sample.sleep_change;
    g_latest_motion.x_wake = sample.x_wake;
    g_latest_motion.y_wake = sample.y_wake;
    g_latest_motion.z_wake = sample.z_wake;

    prototracer::SensorEvent event = {};
    event.type = prototracer::SensorEvent::Type::Motion;
    event.tick_ms = tick_ms_now();
    event.status = ESP_OK;
    event.source = source;
    event.motion = g_latest_motion;
    queue_sensor_event(event);
}

void process_gesture_interrupt(const prototracer::board::ExpanderInputBit source)
{
    if (!shared_gesture_sensor().ready())
    {
        return;
    }

    prototracer::Vcnl4035::Sample sample = {};
    const esp_err_t err = shared_gesture_sensor().read_sample(&sample);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "Gesture sample read failed: %s", esp_err_to_name(err));
        queue_sensor_fault(source, err);
        return;
    }

    g_latest_gesture.valid = true;
    g_latest_gesture.proximity_1 = sample.proximity_1;
    g_latest_gesture.proximity_2 = sample.proximity_2;
    g_latest_gesture.proximity_3 = sample.proximity_3;
    g_latest_gesture.ambient = sample.ambient;
    g_latest_gesture.white = sample.white;
    g_latest_gesture.interrupt_flags = sample.interrupt_flags;
    g_latest_gesture.gesture_ready = sample.gesture_ready;
    g_latest_gesture.proximity_close = sample.proximity_close;
    g_latest_gesture.proximity_away = sample.proximity_away;
    g_latest_gesture.ambient_high = sample.ambient_high;
    g_latest_gesture.ambient_low = sample.ambient_low;

    prototracer::SensorEvent event = {};
    event.type = prototracer::SensorEvent::Type::Gesture;
    event.tick_ms = tick_ms_now();
    event.status = ESP_OK;
    event.source = source;
    event.gesture = g_latest_gesture;
    queue_sensor_event(event);
}

void process_fuel_gauge_interrupt(const prototracer::board::ExpanderInputBit source)
{
    if (!shared_fuel_gauge().ready())
    {
        return;
    }

    prototracer::Max17055::Sample sample = {};
    const esp_err_t err = shared_fuel_gauge().read_sample(&sample);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "Fuel gauge sample read failed: %s", esp_err_to_name(err));
        queue_sensor_fault(source, err);
        return;
    }

    update_latest_battery_from_sample(sample);

    if (sample.power_on_reset || sample.soc_change_alert)
    {
        shared_fuel_gauge().clear_status_alerts();
    }

    prototracer::SensorEvent event = {};
    event.type = prototracer::SensorEvent::Type::FuelGauge;
    event.tick_ms = tick_ms_now();
    event.status = ESP_OK;
    event.source = source;
    event.battery = g_latest_battery;
    queue_sensor_event(event);
}

void sensor_task(void *arg)
{
    while (true)
    {
        prototracer::board::ExpanderInputBit source = prototracer::board::ExpanderInputBit::GestureInterrupt;
        if (xQueueReceive(g_sensor_work_queue, &source, portMAX_DELAY) != pdTRUE)
        {
            continue;
        }

        switch (source)
        {
        case prototracer::board::ExpanderInputBit::GestureInterrupt:
            process_gesture_interrupt(source);
            break;
        case prototracer::board::ExpanderInputBit::ImuInterrupt1:
        case prototracer::board::ExpanderInputBit::ImuInterrupt2:
            process_imu_interrupt(source);
            break;
        case prototracer::board::ExpanderInputBit::FuelGaugeAlarm:
            process_fuel_gauge_interrupt(source);
            break;
        default:
            break;
        }
    }
}

void sensor_input_listener(const prototracer::InputEvent &event, void *context)
{
    (void)context;
    if (event.type != prototracer::InputEvent::Type::SensorInterruptAsserted)
    {
        return;
    }

    if (g_sensor_work_queue != nullptr)
    {
        const auto source = event.source;
        xQueueSend(g_sensor_work_queue, &source, 0);
    }
}

void IRAM_ATTR expander_irq_isr(void *arg)
{
    (void)arg;
    uint32_t status = 0;
    gpio_hal_get_intr_status(&g_input_gpio_hal, 0, &status);

    const uint32_t expander_irq_mask = 1UL << static_cast<uint32_t>(prototracer::board::kPinExpanderInt);
    if ((status & expander_irq_mask) == 0)
    {
        return;
    }

    gpio_hal_clear_intr_status_bit(&g_input_gpio_hal, prototracer::board::kPinExpanderInt);

    const uint32_t event = 1;
    BaseType_t higher_priority_task_woken = pdFALSE;
    if (g_input_irq_queue != nullptr)
    {
        xQueueSendFromISR(g_input_irq_queue, &event, &higher_priority_task_woken);
    }
    if (higher_priority_task_woken == pdTRUE)
    {
        portYIELD_FROM_ISR();
    }
}

void input_router_task(void *arg)
{
    (void)arg;
    uint8_t current_state = g_latest_snapshot.raw_bits;
    if (read_expander_inputs_stable(&current_state) == ESP_OK)
    {
        g_latest_snapshot.raw_bits = current_state;
        ESP_LOGI(TAG, "Initial TCA9534 snapshot = 0x%02x", current_state);
    }

    while (true)
    {
        uint32_t event = 0;
        if (xQueueReceive(g_input_irq_queue, &event, pdMS_TO_TICKS(kInputPollIntervalMs)) != pdTRUE)
        {
            // Periodic fallback poll: re-read expander in case an edge was missed
            uint8_t polled_state = g_latest_snapshot.raw_bits;
            if (read_expander_inputs_stable(&polled_state) == ESP_OK)
            {
                const uint8_t previous = g_latest_snapshot.raw_bits;
                g_latest_snapshot.raw_bits = polled_state;
                log_expander_changes(previous, polled_state);
            }
            continue;
        }

        vTaskDelay(expander_settle_delay_ticks());
        drain_input_irq_queue();

        uint8_t updated_state = g_latest_snapshot.raw_bits;
        if (read_expander_inputs_stable(&updated_state) != ESP_OK)
        {
            ESP_LOGW(TAG, "Failed to read TCA9534 after IRQ");
            continue;
        }

        const uint8_t previous = g_latest_snapshot.raw_bits;
        g_latest_snapshot.raw_bits = updated_state;
        log_expander_changes(previous, updated_state);
    }
}
} // namespace

namespace prototracer
{
bool InputSnapshot::is_asserted(const board::ExpanderInputBit bit) const
{
    return (raw_bits & (1U << static_cast<uint8_t>(bit))) == 0;
}

bool InputEvent::asserted() const
{
    return type == Type::ButtonPressed || type == Type::SensorInterruptAsserted;
}

esp_err_t SensorHub::init()
{
    gpio_config_t sensor_outputs = {};
    sensor_outputs.pin_bit_mask = (1ULL << board::kPinSensorRailEnable) | (1ULL << board::kPinImuCs);
    sensor_outputs.mode = GPIO_MODE_OUTPUT;
    sensor_outputs.pull_up_en = GPIO_PULLUP_DISABLE;
    sensor_outputs.pull_down_en = GPIO_PULLDOWN_DISABLE;
    sensor_outputs.intr_type = GPIO_INTR_DISABLE;
    ESP_RETURN_ON_ERROR(gpio_config(&sensor_outputs), TAG, "Sensor control GPIO config failed");
    g_sensor_rail_output_ready = true;

    ESP_RETURN_ON_ERROR(ensure_sensor_rail_enabled(), TAG, "Failed to assert sensor rail");
    ESP_RETURN_ON_ERROR(gpio_set_level(board::kPinImuCs, 1), TAG, "Failed to drive IMU CS high for I2C mode");
    vTaskDelay(pdMS_TO_TICKS(10));

    I2cBus::Config bus_config = {};
    bus_config.port = I2C_NUM_0;
    bus_config.sda = board::kPinI2cSda;
    bus_config.scl = board::kPinI2cScl;
    bus_config.frequency_hz = kSharedI2cFrequencyHz;
    bus_config.enable_internal_pullups = true;
    ESP_RETURN_ON_ERROR(shared_i2c_bus().init(bus_config), TAG, "Shared I2C init failed");

    if (g_sensor_work_queue == nullptr)
    {
        g_sensor_work_queue = xQueueCreate(kSensorWorkQueueLength, sizeof(board::ExpanderInputBit));
        if (g_sensor_work_queue == nullptr)
        {
            return ESP_ERR_NO_MEM;
        }
    }

    if (g_sensor_event_queue == nullptr)
    {
        g_sensor_event_queue = xQueueCreate(kSensorEventQueueLength, sizeof(SensorEvent));
        if (g_sensor_event_queue == nullptr)
        {
            return ESP_ERR_NO_MEM;
        }
    }

    if (g_sensor_task == nullptr)
    {
        const BaseType_t task_ok = xTaskCreate(sensor_task, "sensor_router", 6144, nullptr, 10, &g_sensor_task);
        if (task_ok != pdPASS)
        {
            return ESP_ERR_NO_MEM;
        }
    }

    Lsm6dso::Config imu_config = {};
    imu_config.address = board::kLsm6dsoAddress;
    imu_config.route_sleep_change_to_int2 = true;
    const esp_err_t imu_err = shared_imu().init(&shared_i2c_bus(), imu_config);
    if (imu_err != ESP_OK)
    {
        ESP_LOGW(TAG, "IMU init failed: %s", esp_err_to_name(imu_err));
    }

    Vcnl4035::Config gesture_config = {};
    gesture_config.address = board::kVcnl4035Address;
    const esp_err_t gesture_err = shared_gesture_sensor().init(&shared_i2c_bus(), gesture_config);
    if (gesture_err != ESP_OK)
    {
        ESP_LOGW(TAG, "Gesture sensor init failed: %s", esp_err_to_name(gesture_err));
    }

    Max17055::Config gauge_config = {};
    gauge_config.address = board::kMax17055Address;
    const esp_err_t gauge_err = shared_fuel_gauge().init(&shared_i2c_bus(), gauge_config);
    if (gauge_err != ESP_OK)
    {
        ESP_LOGW(TAG, "Fuel gauge init failed: %s", esp_err_to_name(gauge_err));
    }
    else
    {
        ESP_LOGI(TAG, "Fuel gauge initial sample deferred until runtime to avoid blocking boot");
    }

    ESP_LOGI(TAG, "Sensor hub wired over shared I2C on GPIO%d/GPIO%d", board::kPinI2cSda, board::kPinI2cScl);
    ESP_LOGI(TAG, "Gesture sensor=VCNL4035, IMU=LSM6DSOWTR, fuel gauge=MAX17055, expander=TCA9534");
    ESP_LOGI(TAG, "IMU CS is on GPIO%d and the sensor-domain interrupt fan-in arrives on GPIO%d", board::kPinImuCs, board::kPinExpanderInt);
    return ESP_OK;
}

esp_err_t SensorHub::attach_input_router(InputRouter *router)
{
    if (router == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (g_sensor_listener_attached)
    {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(router->register_listener(sensor_input_listener, nullptr), TAG, "Failed to attach sensor listener to input router");
    g_sensor_listener_attached = true;
    return ESP_OK;
}

esp_err_t SensorHub::wait_for_event(SensorEvent *out, const TickType_t timeout) const
{
    if (out == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (g_sensor_event_queue == nullptr)
    {
        return ESP_ERR_INVALID_STATE;
    }

    return xQueueReceive(g_sensor_event_queue, out, timeout) == pdTRUE ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t SensorHub::get_latest_motion(MotionSample *out) const
{
    if (out == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }

    *out = g_latest_motion;
    return g_latest_motion.valid ? ESP_OK : ESP_ERR_NOT_FOUND;
}

esp_err_t SensorHub::get_latest_gesture(GestureSample *out) const
{
    if (out == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }

    *out = g_latest_gesture;
    return g_latest_gesture.valid ? ESP_OK : ESP_ERR_NOT_FOUND;
}

esp_err_t SensorHub::get_latest_battery(FuelGaugeSample *out) const
{
    if (out == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }

    *out = g_latest_battery;
    return g_latest_battery.valid ? ESP_OK : ESP_ERR_NOT_FOUND;
}

esp_err_t SensorHub::refresh_battery(FuelGaugeSample *out) const
{
    if (!shared_fuel_gauge().ready())
    {
        return ESP_ERR_INVALID_STATE;
    }

    Max17055::Sample sample = {};
    const esp_err_t err = shared_fuel_gauge().read_sample(&sample);
    if (err != ESP_OK)
    {
        return err;
    }

    update_latest_battery_from_sample(sample);
    if (sample.power_on_reset || sample.soc_change_alert)
    {
        shared_fuel_gauge().clear_status_alerts();
    }

    if (out != nullptr)
    {
        *out = g_latest_battery;
    }
    return ESP_OK;
}

esp_err_t SensorHub::sample_motion_now(MotionSample *out) const
{
    if (out == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (!shared_imu().ready())
    {
        return ESP_ERR_INVALID_STATE;
    }

    Lsm6dso::Sample sample = {};
    const esp_err_t err = shared_imu().read_sample(&sample);
    if (err != ESP_OK)
    {
        return err;
    }

    g_latest_motion.valid = true;
    g_latest_motion.x_raw = sample.x_raw;
    g_latest_motion.y_raw = sample.y_raw;
    g_latest_motion.z_raw = sample.z_raw;
    g_latest_motion.x_mg = sample.x_mg;
    g_latest_motion.y_mg = sample.y_mg;
    g_latest_motion.z_mg = sample.z_mg;
    g_latest_motion.gyro_valid = sample.gyro_valid;
    g_latest_motion.gx_raw = sample.gx_raw;
    g_latest_motion.gy_raw = sample.gy_raw;
    g_latest_motion.gz_raw = sample.gz_raw;
    g_latest_motion.gx_dps = sample.gx_dps;
    g_latest_motion.gy_dps = sample.gy_dps;
    g_latest_motion.gz_dps = sample.gz_dps;
    g_latest_motion.wake_source = sample.wake_source;
    g_latest_motion.wake_event = sample.wake_event;
    g_latest_motion.sleep_change = sample.sleep_change;
    g_latest_motion.x_wake = sample.x_wake;
    g_latest_motion.y_wake = sample.y_wake;
    g_latest_motion.z_wake = sample.z_wake;
    *out = g_latest_motion;
    return ESP_OK;
}

esp_err_t SensorHub::sample_gesture_now(GestureSample *out) const
{
    if (out == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (!shared_gesture_sensor().ready())
    {
        return ESP_ERR_INVALID_STATE;
    }

    Vcnl4035::Sample sample = {};
    const esp_err_t err = shared_gesture_sensor().read_sample(&sample);
    if (err != ESP_OK)
    {
        return err;
    }

    g_latest_gesture.valid = true;
    g_latest_gesture.proximity_1 = sample.proximity_1;
    g_latest_gesture.proximity_2 = sample.proximity_2;
    g_latest_gesture.proximity_3 = sample.proximity_3;
    g_latest_gesture.ambient = sample.ambient;
    g_latest_gesture.white = sample.white;
    g_latest_gesture.interrupt_flags = sample.interrupt_flags;
    g_latest_gesture.gesture_ready = sample.gesture_ready;
    g_latest_gesture.proximity_close = sample.proximity_close;
    g_latest_gesture.proximity_away = sample.proximity_away;
    g_latest_gesture.ambient_high = sample.ambient_high;
    g_latest_gesture.ambient_low = sample.ambient_low;
    *out = g_latest_gesture;
    return ESP_OK;
}

esp_err_t SensorHub::configure_gesture_channels(const bool enable_proximity, const bool enable_white_channel) const
{
    if (!shared_gesture_sensor().ready())
    {
        return ESP_ERR_INVALID_STATE;
    }

    return shared_gesture_sensor().set_channels_enabled(enable_proximity, enable_white_channel);
}

esp_err_t InputRouter::init()
{
    ESP_RETURN_ON_ERROR(ensure_sensor_rail_enabled(), TAG, "Failed to assert sensor rail for inputs");
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_RETURN_ON_ERROR(shared_expander().init(&shared_i2c_bus(), board::kTca9534Address), TAG, "Expander init failed");
    ESP_RETURN_ON_ERROR(shared_expander().set_polarity(0x00), TAG, "Expander polarity config failed");
    ESP_RETURN_ON_ERROR(shared_expander().set_configuration(kExpanderAllInputsMask), TAG, "Expander input config failed");

    uint8_t initial_state = 0xFF;
    ESP_RETURN_ON_ERROR(read_expander_inputs_stable(&initial_state), TAG, "Initial expander read failed");
    g_latest_snapshot.raw_bits = initial_state;

    if (!g_joystick_adc.ready)
    {
        adc_unit_t logical_x_unit = ADC_UNIT_1;
        adc_channel_t logical_x_channel = ADC_CHANNEL_0;

        // Board bring-up found the joystick axes are physically swapped and the silkscreened
        // X axis is routed to a non-ADC GPIO. Treat the ADC-capable Y pin as logical X and
        // discard logical Y until the hardware is rerouted.
        const bool x_ok = adc_oneshot_io_to_channel(board::kPinJoystickY, &logical_x_unit, &logical_x_channel) == ESP_OK;
        const bool x_digital = false;
        const bool y_ok = false;

        if (!x_ok)
        {
            ESP_LOGW(TAG, "Logical joystick X uses GPIO%d (board Y), but that pin is not ADC-capable; joystick disabled", board::kPinJoystickY);
        }
        ESP_LOGW(TAG, "Discarding logical joystick Y because the board X axis is wired to non-ADC GPIO%d", board::kPinJoystickX);

        if (!x_ok)
        {
            ESP_LOGW(TAG, "No joystick input path available; joystick disabled");
        }
        else
        {
            const bool any_analog = x_ok;
            if (any_analog)
            {
                const adc_unit_t adc_unit = logical_x_unit;

                adc_oneshot_unit_init_cfg_t adc_init = {};
                adc_init.unit_id = adc_unit;
                adc_init.ulp_mode = ADC_ULP_MODE_DISABLE;
                ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&adc_init, &g_joystick_adc.handle), TAG, "Joystick ADC unit init failed");

                adc_oneshot_chan_cfg_t adc_channel_config = {};
                adc_channel_config.atten = ADC_ATTEN_DB_12;
                adc_channel_config.bitwidth = ADC_BITWIDTH_DEFAULT;

                ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(g_joystick_adc.handle, logical_x_channel, &adc_channel_config), TAG, "Joystick X ADC config failed");

                g_joystick_adc.unit = adc_unit;
                g_joystick_adc.x_channel = logical_x_channel;

                constexpr int kCalibrationSamples = 8;
                int sum_x = 0;
                for (int index = 0; index < kCalibrationSamples; ++index)
                {
                    int raw = 0;
                    (void)adc_oneshot_read(g_joystick_adc.handle, g_joystick_adc.x_channel, &raw);
                    sum_x += raw;
                    vTaskDelay(pdMS_TO_TICKS(2));
                }

                g_joystick_adc.center_x = sum_x / kCalibrationSamples;
                g_joystick_adc.center_y = 2048;
            }
            else
            {
                g_joystick_adc.center_x = 2048;
                g_joystick_adc.center_y = 2048;
            }

            g_joystick_adc.x_ok = x_ok;
            g_joystick_adc.x_digital = x_digital;
            g_joystick_adc.y_ok = y_ok;
            g_joystick_adc.ready = true;
        }
    }

    if (g_input_irq_queue == nullptr)
    {
        g_input_irq_queue = xQueueCreate(kInputIrqQueueLength, sizeof(uint32_t));
        if (g_input_irq_queue == nullptr)
        {
            return ESP_ERR_NO_MEM;
        }
    }

    if (g_input_event_queue == nullptr)
    {
        g_input_event_queue = xQueueCreate(kInputEventQueueLength, sizeof(InputEvent));
        if (g_input_event_queue == nullptr)
        {
            return ESP_ERR_NO_MEM;
        }
    }

    if (kUseExpanderIrq && board::kPinExpanderInt != GPIO_NUM_NC)
    {
        gpio_config_t irq_config = {};
        irq_config.pin_bit_mask = gpio_pin_mask(board::kPinExpanderInt);
        irq_config.mode = GPIO_MODE_INPUT;
        irq_config.pull_up_en = GPIO_PULLUP_ENABLE;
        irq_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
        irq_config.intr_type = GPIO_INTR_DISABLE;
        ESP_RETURN_ON_ERROR(gpio_config(&irq_config), TAG, "Expander IRQ GPIO config failed");

        if (!g_input_gpio_isr_registered)
        {
            ESP_RETURN_ON_ERROR(
                gpio_isr_register(expander_irq_isr, nullptr, ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL1, &g_input_gpio_isr_handle),
                TAG,
                "Failed to register expander GPIO ISR");
            g_input_gpio_isr_registered = true;
        }

        gpio_hal_clear_intr_status_bit(&g_input_gpio_hal, board::kPinExpanderInt);
        ESP_RETURN_ON_ERROR(gpio_set_intr_type(board::kPinExpanderInt, GPIO_INTR_NEGEDGE), TAG, "Expander IRQ type config failed");
        ESP_RETURN_ON_ERROR(gpio_intr_enable(board::kPinExpanderInt), TAG, "Failed to enable expander IRQ");
    }

    ESP_LOGI(TAG, "Input router initialized: logical X<=GPIO%d (board Y), logical Y=disabled, expander IRQ=%d", board::kPinJoystickY, board::kPinExpanderInt);
    ESP_LOGI(TAG, "Buttons and secondary sensor interrupts are behind the TCA9534 expander at 0x%02x", board::kTca9534Address);
    if (!kUseExpanderIrq || board::kPinExpanderInt == GPIO_NUM_NC)
    {
        ESP_LOGI(TAG, "Expander IRQ disabled for bring-up; TCA9534 inputs are polled every %lu ms", static_cast<unsigned long>(kInputPollIntervalMs));
    }
    else
    {
        ESP_LOGI(TAG, "Expander IRQ enabled on GPIO%d using dedicated GPIO ISR; %lu ms poll remains as fallback", board::kPinExpanderInt, static_cast<unsigned long>(kInputPollIntervalMs));
    }

    // Post a synthetic event so input_router_task reads the initial expander state immediately
    if (g_input_irq_queue != nullptr)
    {
        const uint32_t synthetic = 1;
        xQueueSend(g_input_irq_queue, &synthetic, 0);
    }

    if (g_input_router_task == nullptr)
    {
        const BaseType_t task_ok = xTaskCreate(input_router_task, "input_router", 6144, nullptr, 10, &g_input_router_task);
        if (task_ok != pdPASS)
        {
            return ESP_ERR_NO_MEM;
        }
    }

    return ESP_OK;
}

esp_err_t InputRouter::get_snapshot(InputSnapshot *out) const
{
    if (out == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }

    *out = g_latest_snapshot;
    return ESP_OK;
}

esp_err_t InputRouter::read_joystick(JoystickSample *out) const
{
    if (out == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(ensure_sensor_rail_enabled(), TAG, "Failed to keep sensor rail asserted for joystick read");

    if (!g_joystick_adc.ready)
    {
        // Joystick unavailable — return neutral sample
        *out = {};
        out->valid = true;
        out->x_available = false;
        out->y_available = false;
        out->raw_x = g_joystick_adc.center_x;
        out->raw_y = g_joystick_adc.center_y;
        out->normalized_x = 0;
        out->normalized_y = 0;
        out->pressed = (g_latest_snapshot.raw_bits & (1U << static_cast<uint8_t>(board::ExpanderInputBit::JoystickButton))) == 0;
        return ESP_OK;
    }

    int raw_x = g_joystick_adc.x_digital ? (gpio_get_level(board::kPinJoystickX) != 0 ? 1 : 0) : g_joystick_adc.center_x;
    int raw_y = g_joystick_adc.center_y;
    if (g_joystick_adc.x_ok && g_joystick_adc.handle != nullptr)
    {
        ESP_RETURN_ON_ERROR(read_joystick_axis_average(g_joystick_adc.handle, g_joystick_adc.x_channel, &raw_x), TAG, "Joystick X ADC read failed");
    }
    if (g_joystick_adc.y_ok && g_joystick_adc.handle != nullptr)
    {
        ESP_RETURN_ON_ERROR(read_joystick_axis_average(g_joystick_adc.handle, g_joystick_adc.y_channel, &raw_y), TAG, "Joystick Y ADC read failed");
    }

    *out = {};
    out->valid = true;
    out->x_available = g_joystick_adc.x_ok || g_joystick_adc.x_digital;
    out->x_digital = g_joystick_adc.x_digital;
    out->y_available = g_joystick_adc.y_ok;
    out->raw_x = raw_x;
    out->raw_y = raw_y;
    if (g_joystick_adc.x_digital)
    {
        const int digital_level = raw_x != 0 ? 1 : 0;
        out->normalized_x = digital_level == g_joystick_adc.x_digital_idle_level ? 0 : (g_joystick_adc.x_digital_idle_level != 0 ? -100 : 100);
    }
    else
    {
        out->normalized_x = normalize_axis_percent(raw_x, g_joystick_adc.center_x);
    }
    out->normalized_y = -normalize_axis_percent(raw_y, g_joystick_adc.center_y);
    out->pressed = (g_latest_snapshot.raw_bits & (1U << static_cast<uint8_t>(board::ExpanderInputBit::JoystickButton))) == 0;
    out->left = out->x_available && out->normalized_x <= -kJoystickDeadZonePercent;
    out->right = out->x_available && out->normalized_x >= kJoystickDeadZonePercent;
    out->up = out->y_available && out->normalized_y >= kJoystickDeadZonePercent;
    out->down = out->y_available && out->normalized_y <= -kJoystickDeadZonePercent;
    return ESP_OK;
}

esp_err_t InputRouter::wait_for_event(InputEvent *out, const TickType_t timeout) const
{
    if (out == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (g_input_event_queue == nullptr)
    {
        return ESP_ERR_INVALID_STATE;
    }

    return xQueueReceive(g_input_event_queue, out, timeout) == pdTRUE ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t InputRouter::register_listener(InputEventCallback callback, void *context)
{
    if (callback == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }

    for (InputListenerSlot &slot : g_input_listeners)
    {
        if (slot.callback == callback && slot.context == context)
        {
            return ESP_OK;
        }
    }

    for (InputListenerSlot &slot : g_input_listeners)
    {
        if (slot.callback == nullptr)
        {
            slot.callback = callback;
            slot.context = context;
            return ESP_OK;
        }
    }

    return ESP_ERR_NO_MEM;
}

esp_err_t PowerManager::init()
{
    gpio_config_t power_inputs = {};
    power_inputs.pin_bit_mask = (1ULL << board::kPinBatteryMonitor) | (1ULL << board::kPinPowerStatus);
    power_inputs.mode = GPIO_MODE_INPUT;
    power_inputs.pull_up_en = GPIO_PULLUP_DISABLE;
    power_inputs.pull_down_en = GPIO_PULLDOWN_DISABLE;
    power_inputs.intr_type = GPIO_INTR_DISABLE;
    ESP_RETURN_ON_ERROR(gpio_config(&power_inputs), TAG, "Power input GPIO config failed");

    gpio_config_t charger_output = {};
    charger_output.pin_bit_mask = 1ULL << board::kPinChargerEnable;
    charger_output.mode = GPIO_MODE_OUTPUT;
    charger_output.pull_up_en = GPIO_PULLUP_DISABLE;
    charger_output.pull_down_en = GPIO_PULLDOWN_DISABLE;
    charger_output.intr_type = GPIO_INTR_DISABLE;
    ESP_RETURN_ON_ERROR(gpio_config(&charger_output), TAG, "Charger enable GPIO config failed");
    ESP_RETURN_ON_ERROR(gpio_set_level(board::kPinChargerEnable, 0), TAG, "Failed to disable charger during init");
    g_charger_enabled = false;

    ESP_LOGI(TAG, "Power manager initialized: VBAT_MON=%d, PWR_STAT=%d, CHR_EN=%d default=off", board::kPinBatteryMonitor, board::kPinPowerStatus, board::kPinChargerEnable);
    return ESP_OK;
}

BatteryChemistry PowerManager::classify_battery_chemistry() const
{
    if (!g_latest_battery.valid)
    {
        return BatteryChemistry::Unknown;
    }

    const float voltage = g_latest_battery.voltage_v;
    const float current = g_latest_battery.current_ma;
    const float average_current = g_latest_battery.average_current_ma;

    if (current > kChargeCurrentDetectThresholdMa || average_current > kChargeCurrentDetectThresholdMa)
    {
        return BatteryChemistry::NiMH;
    }
    if (voltage >= kThreeCellNiMhFullVoltage)
    {
        return BatteryChemistry::Alkaline;
    }
    if (voltage >= kThreeCellNiMhEmptyVoltage && voltage < kThreeCellNiMhFullVoltage)
    {
        return BatteryChemistry::NiMH;
    }

    return BatteryChemistry::Unknown;
}

esp_err_t PowerManager::update_charger_control(const BatteryChemistry chemistry)
{
    const bool power_status = external_power_present();
    const bool vbat_monitor = gpio_get_level(board::kPinBatteryMonitor) != 0;
    const bool should_enable = chemistry == BatteryChemistry::NiMH || (power_status && chemistry == BatteryChemistry::Unknown);

    if (g_charger_enabled != should_enable)
    {
        ESP_RETURN_ON_ERROR(gpio_set_level(board::kPinChargerEnable, should_enable ? 1 : 0), TAG, "Failed to update charger enable");
        g_charger_enabled = should_enable;
        ESP_LOGI(
            TAG,
            "Charger input path %s (pwr_stat=%u vbat_mon=%u chemistry=%s)",
            should_enable ? "enabled" : "disabled",
            power_status ? 1U : 0U,
            vbat_monitor ? 1U : 0U,
            battery_chemistry_name(chemistry));
    }
    else if (g_last_charger_power_status != power_status ||
             g_last_charger_vbat_monitor != vbat_monitor ||
             g_last_charger_chemistry != chemistry)
    {
        ESP_LOGI(
            TAG,
            "Charger input path remains %s (pwr_stat=%u vbat_mon=%u chemistry=%s)",
            g_charger_enabled ? "enabled" : "disabled",
            power_status ? 1U : 0U,
            vbat_monitor ? 1U : 0U,
            battery_chemistry_name(chemistry));
    }

    g_last_charger_power_status = power_status;
    g_last_charger_vbat_monitor = vbat_monitor;
    g_last_charger_chemistry = chemistry;
    return ESP_OK;
}

bool PowerManager::external_power_present() const
{
    return gpio_get_level(board::kPinPowerStatus) != 0;
}

bool PowerManager::charger_enabled() const
{
    return g_charger_enabled;
}

void PowerManager::log_hardware_limits() const
{
    ESP_LOGW(TAG, "Battery chemistry is estimated for a 3-cell NiMH pack from MAX17055 voltage/current; VBAT_MON and PWR_STAT are digital diagnostic signals, not charger enable vetoes");
}

esp_err_t StatusLed::init()
{
    gpio_config_t emitter_enable = {};
    emitter_enable.pin_bit_mask = 1ULL << board::kPinRearEmitterEnable;
    emitter_enable.mode = GPIO_MODE_OUTPUT;
    emitter_enable.pull_up_en = GPIO_PULLUP_DISABLE;
    emitter_enable.pull_down_en = GPIO_PULLDOWN_DISABLE;
    emitter_enable.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&emitter_enable));
    ESP_ERROR_CHECK(gpio_set_level(board::kPinRearEmitterEnable, 0));

    if (g_status_led_strip == nullptr)
    {
        led_strip_config_t strip_config = {};
        strip_config.strip_gpio_num = board::kPinAddressableLedData;
        strip_config.max_leds = 4;

        led_strip_rmt_config_t rmt_config = {};
        rmt_config.resolution_hz = kLedStripResolutionHz;
        rmt_config.flags.with_dma = false;

        ESP_RETURN_ON_ERROR(led_strip_new_rmt_device(&strip_config, &rmt_config, &g_status_led_strip), TAG, "WS2812 RMT init failed");
    }

    ESP_RETURN_ON_ERROR(led_strip_clear(g_status_led_strip), TAG, "WS2812 clear failed");
    ESP_LOGI(TAG, "LED framework initialized: rear emitter disable GPIO=%d, WS2812 data GPIO=%d", board::kPinRearEmitterEnable, board::kPinAddressableLedData);
    return ESP_OK;
}

esp_err_t StatusLed::set_mode(const StatusLedMode mode)
{
    ESP_LOGI(TAG, "Status LED mode -> %s", status_led_mode_name(mode));
    ESP_ERROR_CHECK(gpio_set_level(board::kPinRearEmitterEnable, 0));

    if (g_status_led_strip == nullptr)
    {
        return ESP_OK;
    }

    uint8_t red = 0;
    uint8_t green = 0;
    uint8_t blue = 0;
    status_led_rgb_for_mode(mode, &red, &green, &blue);

    for (uint8_t led = 0; led < 4; ++led)
    {
        const uint8_t pixel_red = led == 0 ? red : 0;
        const uint8_t pixel_green = led == 0 ? green : 0;
        const uint8_t pixel_blue = led == 0 ? blue : 0;
        ESP_RETURN_ON_ERROR(led_strip_set_pixel(g_status_led_strip, led, pixel_red, pixel_green, pixel_blue), TAG, "WS2812 set pixel failed");
    }

    return led_strip_refresh(g_status_led_strip);
}

esp_err_t StatusLed::set_pixel(const uint8_t index, const uint8_t red, const uint8_t green, const uint8_t blue)
{
    if (g_status_led_strip == nullptr)
    {
        return ESP_ERR_INVALID_STATE;
    }
    if (index >= StatusLed::pixel_count)
    {
        return ESP_ERR_INVALID_ARG;
    }
    return led_strip_set_pixel(g_status_led_strip, index, red, green, blue);
}

esp_err_t StatusLed::refresh()
{
    if (g_status_led_strip == nullptr)
    {
        return ESP_ERR_INVALID_STATE;
    }
    return led_strip_refresh(g_status_led_strip);
}

esp_err_t StatusLed::clear()
{
    if (g_status_led_strip == nullptr)
    {
        return ESP_ERR_INVALID_STATE;
    }
    ESP_RETURN_ON_ERROR(led_strip_clear(g_status_led_strip), TAG, "WS2812 clear failed");
    return led_strip_refresh(g_status_led_strip);
}

esp_err_t DisplayService::init()
{
    return shared_display().init();
}

void DisplayService::set_language(const UiLanguage language)
{
    shared_display().set_language(language);
}

esp_err_t DisplayService::set_brightness(const uint8_t brightness)
{
    return shared_display().set_brightness(brightness);
}

esp_err_t DisplayService::set_sleeping(const bool sleeping)
{
    return shared_display().set_sleeping(sleeping);
}

void DisplayService::set_signal_strength(const uint8_t percent, const bool visible)
{
    shared_display().set_signal_strength(percent, visible);
}

esp_err_t DisplayService::show_status(const char *title, const char *detail, const StatusLedMode mode)
{
    return shared_display().show_status_screen(title, detail, display_accent_for_mode(mode));
}

esp_err_t DisplayService::show_provisioning(const char *portal_name, const char *hint, const uint8_t percent)
{
    return shared_display().show_provisioning_screen(portal_name, hint, percent);
}

esp_err_t DisplayService::show_bind_progress(const char *phase, const uint8_t percent)
{
    return shared_display().show_bind_progress_screen(phase, percent);
}

esp_err_t DisplayService::show_error(const char *title, const char *detail, const char *hint)
{
    return shared_display().show_error_screen(title, detail, hint);
}

esp_err_t DisplayService::show_update_progress(const char *phase, const uint8_t percent)
{
    return shared_display().show_update_progress_screen(phase, percent);
}

esp_err_t DisplayService::show_test_menu(const char *title,
                                         const char *aux,
                                         const char (*lines)[kTestMenuLineLength],
                                         const uint8_t line_count,
                                         const uint8_t selected_index)
{
    static_assert(kTestMenuLineLength == Ssd1351Display::kTestMenuLineLength, "Test menu line length mismatch");
    static_assert(kTestMenuMaxLines == Ssd1351Display::kTestMenuMaxLines, "Test menu line count mismatch");
    return shared_display().show_test_menu_screen(title, aux, lines, line_count, selected_index, 0x07FF);
}

esp_err_t LowPowerController::init(const bool enabled)
{
    ESP_RETURN_ON_ERROR(ensure_sensor_rail_enabled(), TAG, "Failed to hold sensor rail high during active runtime");

    if (!enabled)
    {
        ESP_LOGI(TAG, "Low-power core is disabled by configuration; SENSOR_EN is held high on GPIO%d", board::kPinSensorRailEnable);
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Low-power wake setup deferred; active runtime keeps GPIO%d owned by the TCA9534 input router", board::kPinExpanderInt);
    return ESP_OK;
}
} // namespace prototracer