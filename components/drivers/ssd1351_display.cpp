#include "ssd1351_display.hpp"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <new>
#include <string>

#define LGFX_USE_V1

#include <LovyanGFX.hpp>

#include "lgfx/v1/lgfx_fonts.hpp"
#include "prototracer_board.hpp"

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace
{
constexpr const char *TAG = "ssd1351";
constexpr UBaseType_t kRenderQueueLength = 1;
constexpr uint32_t kRenderTaskStack = 8192;

constexpr uint32_t kColorBackground = 0x090B0F;
constexpr uint32_t kColorShell = 0x10151B;
constexpr uint32_t kColorCard = 0x1B2028;
constexpr uint32_t kColorCardAlt = 0x242A33;
constexpr uint32_t kColorTextPrimary = 0xF8FAFC;
constexpr uint32_t kColorTextBody = 0xCBD5E1;
constexpr uint32_t kColorTextMuted = 0x94A3B8;
constexpr uint32_t kColorTextWarning = 0xFDBA74;
constexpr uint32_t kColorTrack = 0x343B46;
constexpr uint32_t kColorOutline = 0x3C4654;
constexpr uint32_t kColorBatteryGood = 0x4ADE80;
constexpr uint32_t kColorBatteryLow = 0xFB923C;
constexpr uint8_t kSsd1351MasterCurrentCommand = 0xC7;
constexpr char kColorMarker = '\x1E';
constexpr char kSwitchMarker = '\x1F';

uint8_t clamp_progress(const uint8_t percent)
{
    return percent > 100 ? 100 : percent;
}

uint8_t oled_master_current(const uint8_t brightness)
{
    return static_cast<uint8_t>(std::clamp<int>(((static_cast<int>(brightness) * 15) + 127) / 255, 1, 15));
}

template <size_t N>
void copy_text(char (&destination)[N], const char *source)
{
    if (source == nullptr)
    {
        destination[0] = '\0';
        return;
    }

    std::snprintf(destination, N, "%s", source);
}

const char *localized(const prototracer::UiLanguage language, const char *english, const char *chinese)
{
    return language == prototracer::UiLanguage::Chinese ? chinese : english;
}

uint32_t scale_rgb888(const uint32_t color, const uint8_t scale)
{
    const uint32_t red = ((color >> 16) & 0xFFU) * scale / 255U;
    const uint32_t green = ((color >> 8) & 0xFFU) * scale / 255U;
    const uint32_t blue = (color & 0xFFU) * scale / 255U;
    return (red << 16) | (green << 8) | blue;
}

size_t utf8_span(const char *text)
{
    const unsigned char lead = static_cast<unsigned char>(*text);
    if ((lead & 0x80U) == 0)
    {
        return 1;
    }
    if ((lead & 0xE0U) == 0xC0U)
    {
        return 2;
    }
    if ((lead & 0xF0U) == 0xE0U)
    {
        return 3;
    }
    if ((lead & 0xF8U) == 0xF0U)
    {
        return 4;
    }
    return 1;
}

void trim_ascii(std::string &value)
{
    const size_t start = value.find_first_not_of(" \t\r\n");
    if (start == std::string::npos)
    {
        value.clear();
        return;
    }

    const size_t end = value.find_last_not_of(" \t\r\n");
    value = value.substr(start, end - start + 1);
}

int line_height_for(const lgfx::IFont *font)
{
    if (font == &fonts::DejaVu18)
    {
        return 22;
    }
    if (font == &fonts::DejaVu12)
    {
        return 16;
    }
    if (font == &fonts::efontCN_16_b)
    {
        return 22;
    }
    if (font == &fonts::efontCN_14)
    {
        return 18;
    }
    return 16;
}

bool parse_percent(const char *text, int *out_percent)
{
    if (text == nullptr || out_percent == nullptr)
    {
        return false;
    }

    const char *percent = std::strchr(text, '%');
    if (percent == nullptr)
    {
        return false;
    }

    const char *number_end = percent;
    while (number_end > text && std::isspace(static_cast<unsigned char>(*(number_end - 1))))
    {
        --number_end;
    }

    const char *number_start = number_end;
    while (number_start > text && std::isdigit(static_cast<unsigned char>(*(number_start - 1))))
    {
        --number_start;
    }

    if (number_start == number_end)
    {
        return false;
    }

    int value = 0;
    for (const char *cursor = number_start; cursor < number_end; ++cursor)
    {
        value = value * 10 + (*cursor - '0');
    }
    *out_percent = std::clamp(value, 0, 100);
    return true;
}

bool parse_slider_text(const char *text, int *out_filled, int *out_total)
{
    if (text == nullptr || out_filled == nullptr || out_total == nullptr || text[0] != '[')
    {
        return false;
    }

    int filled = 0;
    int total = 0;
    for (const char *cursor = text + 1; *cursor != '\0' && *cursor != ']'; ++cursor)
    {
        if (*cursor == '#' || *cursor == '-')
        {
            filled += *cursor == '#' ? 1 : 0;
            ++total;
        }
    }

    if (total == 0)
    {
        return false;
    }

    *out_filled = std::clamp(filled, 0, total);
    *out_total = total;
    return true;
}

bool parse_switch_text(const char *text, char *label, const size_t label_size, bool *out_enabled)
{
    if (text == nullptr || label == nullptr || label_size == 0 || out_enabled == nullptr)
    {
        return false;
    }

    const char *marker = std::strchr(text, kSwitchMarker);
    if (marker == nullptr || marker[1] == '\0')
    {
        return false;
    }

    const size_t label_length = std::min(static_cast<size_t>(marker - text), label_size - 1U);
    std::memcpy(label, text, label_length);
    label[label_length] = '\0';
    *out_enabled = marker[1] == '1';
    return true;
}

int hex_digit_value(const char value)
{
    if (value >= '0' && value <= '9')
    {
        return value - '0';
    }
    if (value >= 'a' && value <= 'f')
    {
        return 10 + value - 'a';
    }
    if (value >= 'A' && value <= 'F')
    {
        return 10 + value - 'A';
    }
    return -1;
}

bool parse_color_swatch_text(const char *text, char *label, const size_t label_size, uint32_t *out_color)
{
    if (text == nullptr || label == nullptr || label_size == 0 || out_color == nullptr)
    {
        return false;
    }

    const char *marker = std::strchr(text, kColorMarker);
    if (marker == nullptr)
    {
        return false;
    }

    const char *hex = marker + 1;
    if (*hex == '#')
    {
        ++hex;
    }

    uint32_t color = 0;
    for (int index = 0; index < 6; ++index)
    {
        const int digit = hex_digit_value(hex[index]);
        if (digit < 0)
        {
            return false;
        }
        color = (color << 4) | static_cast<uint32_t>(digit);
    }

    const size_t label_length = std::min(static_cast<size_t>(marker - text), label_size - 1U);
    std::memcpy(label, text, label_length);
    label[label_length] = '\0';
    *out_color = color;
    return true;
}

bool text_has_on_token(const char *text)
{
    return text != nullptr &&
           (std::strstr(text, "Gate ON") != nullptr ||
            std::strstr(text, "Chg ON") != nullptr ||
            std::strstr(text, "USB") != nullptr);
}

bool text_has_battery_context(const char *text)
{
    if (text == nullptr)
    {
        return false;
    }

    if (std::strstr(text, "Battery") != nullptr ||
        std::strstr(text, "Batt") != nullptr ||
        std::strstr(text, "Gate") != nullptr ||
        std::strstr(text, "Chg") != nullptr)
    {
        return true;
    }

    const char *percent = std::strchr(text, '%');
    if (percent == nullptr)
    {
        return false;
    }

    for (const char *cursor = text; cursor < percent; ++cursor)
    {
        if (*cursor == 'V' || *cursor == 'v')
        {
            return true;
        }
    }
    return false;
}

bool parse_battery_percent(const char *text, int *out_percent)
{
    return text_has_battery_context(text) && parse_percent(text, out_percent);
}

bool battery_status_from_scene(const prototracer::Ssd1351Display::Scene &scene, int *out_percent, bool *out_charging)
{
    if (out_percent == nullptr || out_charging == nullptr)
    {
        return false;
    }

    bool found = false;
    int percent = 0;
    bool charging = text_has_on_token(scene.aux);
    if (parse_battery_percent(scene.aux, &percent))
    {
        found = true;
    }

    for (uint8_t index = 0; index < scene.menu_count; ++index)
    {
        charging = charging || text_has_on_token(scene.menu_lines[index]);
        int candidate = 0;
        if (!found && parse_battery_percent(scene.menu_lines[index], &candidate))
        {
            percent = candidate;
            found = true;
        }
    }

    if (!found && charging)
    {
        percent = 50;
    }
    *out_percent = percent;
    *out_charging = charging;
    return found || charging;
}

class ProtoTracerDisplay final : public lgfx::LGFX_Device
{
public:
    ProtoTracerDisplay()
    {
        auto bus_cfg = bus_.config();
        bus_cfg.spi_host = SPI2_HOST;
        bus_cfg.spi_mode = 0;
        bus_cfg.freq_write = 20000000;
        bus_cfg.freq_read = 0;
        bus_cfg.spi_3wire = false;
        bus_cfg.use_lock = true;
        bus_cfg.dma_channel = SPI_DMA_CH_AUTO;
        bus_cfg.pin_sclk = prototracer::board::kPinDisplaySclk;
        bus_cfg.pin_mosi = prototracer::board::kPinDisplayMosi;
        bus_cfg.pin_miso = -1;
        bus_cfg.pin_dc = prototracer::board::kPinDisplayDc;
        bus_.config(bus_cfg);
        panel_.setBus(&bus_);

        auto panel_cfg = panel_.config();
        panel_cfg.pin_cs = prototracer::board::kPinDisplayCs;
        panel_cfg.pin_rst = prototracer::board::kPinDisplayReset;
        panel_cfg.pin_busy = -1;
        panel_cfg.panel_width = prototracer::Ssd1351Display::kWidth;
        panel_cfg.panel_height = prototracer::Ssd1351Display::kHeight;
        panel_cfg.memory_width = prototracer::Ssd1351Display::kWidth;
        panel_cfg.memory_height = prototracer::Ssd1351Display::kHeight;
        panel_cfg.offset_x = 0;
        panel_cfg.offset_y = 0;
        panel_cfg.offset_rotation = 4;
        panel_cfg.readable = false;
        panel_cfg.invert = false;
        panel_cfg.rgb_order = false;
        panel_cfg.dlen_16bit = false;
        panel_cfg.bus_shared = false;
        panel_.config(panel_cfg);

        setPanel(&panel_);
    }

private:
    lgfx::Bus_SPI bus_;
    lgfx::Panel_SSD1351 panel_;
};
} // namespace

namespace prototracer
{
esp_err_t Ssd1351Display::init()
{
    if (initialized_)
    {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(init_panel_power_(), TAG, "Display power init failed");

    gfx_ = new (std::nothrow) ProtoTracerDisplay();
    if (gfx_ == nullptr)
    {
        return ESP_ERR_NO_MEM;
    }

    if (!gfx_->init())
    {
        return ESP_FAIL;
    }
    gfx_->setRotation(0);
    gfx_->setBrightness(brightness_);
    gfx_->startWrite();
    gfx_->writeCommand(kSsd1351MasterCurrentCommand);
    gfx_->writeData(oled_master_current(brightness_));
    gfx_->endWrite();

    canvas_ = new (std::nothrow) lgfx::LGFX_Sprite(gfx_);
    if (canvas_ == nullptr)
    {
        return ESP_ERR_NO_MEM;
    }
    canvas_->setColorDepth(16);
    canvas_->setPsram(false);
    if (canvas_->createSprite(kWidth, kHeight) == nullptr)
    {
        return ESP_ERR_NO_MEM;
    }
    canvas_->setTextWrap(false, false);

    render_queue_ = xQueueCreate(kRenderQueueLength, sizeof(Scene));
    if (render_queue_ == nullptr)
    {
        return ESP_ERR_NO_MEM;
    }

    const BaseType_t task_ok = xTaskCreate(render_task_entry_, "display_ui", kRenderTaskStack, this, 9, &render_task_);
    if (task_ok != pdPASS)
    {
        return ESP_ERR_NO_MEM;
    }

    initialized_ = true;
    return show_status_screen(localized(language_, "Boot", "启动"), localized(language_, "Display ready", "显示已就绪"), 0x5D9B);
}

void Ssd1351Display::set_language(const UiLanguage language)
{
    language_ = language;
}

esp_err_t Ssd1351Display::set_brightness(const uint8_t brightness)
{
    brightness_ = std::clamp<uint8_t>(brightness, 16, 255);
    if (!initialized_ || gfx_ == nullptr)
    {
        return ESP_ERR_INVALID_STATE;
    }
    if (!sleeping_)
    {
        gfx_->setBrightness(brightness_);
        gfx_->startWrite();
        gfx_->writeCommand(kSsd1351MasterCurrentCommand);
        gfx_->writeData(oled_master_current(brightness_));
        gfx_->endWrite();
    }
    return ESP_OK;
}

esp_err_t Ssd1351Display::set_sleeping(const bool sleeping)
{
    if (!initialized_ || gfx_ == nullptr)
    {
        return ESP_ERR_INVALID_STATE;
    }
    if (sleeping_ == sleeping)
    {
        return ESP_OK;
    }

    sleeping_ = sleeping;
    if (sleeping_)
    {
        gfx_->sleep();
    }
    else
    {
        ESP_RETURN_ON_ERROR(gpio_set_level(board::kPinDisplayEnable, 1), TAG, "Failed to enable display rail");
        gfx_->wakeup();
        gfx_->setBrightness(brightness_);
        gfx_->startWrite();
        gfx_->writeCommand(kSsd1351MasterCurrentCommand);
        gfx_->writeData(oled_master_current(brightness_));
        gfx_->endWrite();
    }
    return ESP_OK;
}

void Ssd1351Display::set_signal_strength(const uint8_t percent, const bool visible)
{
    signal_percent_ = std::clamp<uint8_t>(percent, 0, 100);
    signal_visible_ = visible;
}

esp_err_t Ssd1351Display::show_status_screen(const char *title, const char *detail, const uint16_t accent_rgb565)
{
    Scene scene = {};
    scene.kind = SceneKind::Status;
    scene.accent_rgb565 = accent_rgb565;
    copy_text(scene.title, title != nullptr ? title : localized(language_, "State", "状态"));
    copy_text(scene.detail, detail != nullptr ? detail : "");
    copy_text(scene.aux, localized(language_, "ProtoTracer Remote", "ProtoTracer 遥控器"));
    return enqueue_scene_(scene);
}

esp_err_t Ssd1351Display::show_provisioning_screen(const char *portal_name, const char *hint, const uint8_t progress_percent)
{
    Scene scene = {};
    scene.kind = SceneKind::Provisioning;
    scene.accent_rgb565 = 0x3FE0;
    scene.progress = clamp_progress(progress_percent);
    copy_text(scene.title, localized(language_, "Provision", "配网"));
    copy_text(scene.detail, portal_name != nullptr ? portal_name : localized(language_, "Setup portal", "配置门户"));
    copy_text(scene.aux, hint != nullptr ? hint : localized(language_, "Join AP and open portal", "连接热点并打开门户"));
    return enqueue_scene_(scene);
}

esp_err_t Ssd1351Display::show_bind_progress_screen(const char *phase, const uint8_t progress_percent)
{
    Scene scene = {};
    scene.kind = SceneKind::BindProgress;
    scene.accent_rgb565 = 0x5D9B;
    scene.progress = clamp_progress(progress_percent);
    copy_text(scene.title, localized(language_, "Bind", "绑定"));
    copy_text(scene.detail, phase != nullptr ? phase : localized(language_, "Pairing", "配对中"));
    copy_text(scene.aux, localized(language_, "BLE link with main board", "与主板建立蓝牙链路"));
    return enqueue_scene_(scene);
}

esp_err_t Ssd1351Display::show_error_screen(const char *title, const char *detail, const char *hint)
{
    Scene scene = {};
    scene.kind = SceneKind::ErrorDetail;
    scene.accent_rgb565 = 0xF800;
    copy_text(scene.title, title != nullptr ? title : localized(language_, "Error", "错误"));
    copy_text(scene.detail, detail != nullptr ? detail : localized(language_, "Unknown fault", "未知故障"));
    copy_text(scene.aux, hint != nullptr ? hint : localized(language_, "Check log for details", "请查看日志详情"));
    return enqueue_scene_(scene);
}

esp_err_t Ssd1351Display::show_update_progress_screen(const char *phase, const uint8_t progress_percent)
{
    Scene scene = {};
    scene.kind = SceneKind::UpdateProgress;
    scene.accent_rgb565 = 0xFD20;
    scene.progress = clamp_progress(progress_percent);
    copy_text(scene.title, localized(language_, "Update", "更新"));
    copy_text(scene.detail, phase != nullptr ? phase : localized(language_, "Sync", "同步中"));
    copy_text(scene.aux, localized(language_, "Repository or local image", "仓库或本地镜像"));
    return enqueue_scene_(scene);
}

esp_err_t Ssd1351Display::show_test_menu_screen(const char *title,
                                                const char *aux,
                                                const char (*lines)[kTestMenuLineLength],
                                                const uint8_t line_count,
                                                const uint8_t selected_index,
                                                const uint16_t accent_rgb565)
{
    Scene scene = {};
    scene.kind = SceneKind::TestMenu;
    scene.accent_rgb565 = accent_rgb565 != 0 ? accent_rgb565 : 0x07FF;
    copy_text(scene.title, title != nullptr ? title : localized(language_, "Test", "测试"));
    copy_text(scene.detail, "");
    copy_text(scene.aux, aux != nullptr ? aux : "");
    const uint8_t clamped = line_count > kTestMenuMaxLines ? kTestMenuMaxLines : line_count;
    scene.menu_count = clamped;
    scene.menu_index = selected_index;
    if (lines != nullptr)
    {
        for (uint8_t i = 0; i < clamped; ++i)
        {
            copy_text(scene.menu_lines[i], lines[i]);
        }
    }
    return enqueue_scene_(scene);
}

esp_err_t Ssd1351Display::init_panel_power_()
{
    gpio_config_t io_config = {};
    io_config.pin_bit_mask = (1ULL << board::kPinDisplayEnable);
    io_config.mode = GPIO_MODE_OUTPUT;
    io_config.pull_up_en = GPIO_PULLUP_DISABLE;
    io_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_config.intr_type = GPIO_INTR_DISABLE;
    ESP_RETURN_ON_ERROR(gpio_config(&io_config), TAG, "Display GPIO configuration failed");
    ESP_RETURN_ON_ERROR(gpio_set_level(board::kPinDisplayEnable, 1), TAG, "Failed to enable display rail");
    vTaskDelay(pdMS_TO_TICKS(20));
    return ESP_OK;
}

esp_err_t Ssd1351Display::enqueue_scene_(const Scene &scene)
{
    if (!initialized_ || render_queue_ == nullptr)
    {
        return ESP_ERR_INVALID_STATE;
    }

    Scene queued = scene;
    queued.signal_percent = signal_percent_;
    queued.signal_visible = signal_visible_;
    return xQueueOverwrite(render_queue_, &queued) == pdPASS ? ESP_OK : ESP_FAIL;
}

void Ssd1351Display::render_task_entry_(void *arg)
{
    static_cast<Ssd1351Display *>(arg)->render_task_loop_();
}

void Ssd1351Display::render_task_loop_()
{
    Scene scene = {};
    while (true)
    {
        if (xQueueReceive(render_queue_, &scene, portMAX_DELAY) != pdTRUE)
        {
            continue;
        }

        render_scene_(scene);
    }
}

void Ssd1351Display::render_scene_(const Scene &scene)
{
    if (gfx_ == nullptr || canvas_ == nullptr)
    {
        return;
    }
    if (sleeping_)
    {
        return;
    }

    const uint32_t accent_rgb888 = expand_rgb565_(scene.accent_rgb565);

    draw_background_(accent_rgb888);
    draw_shell_(scene, accent_rgb888, scene_tag_for_(scene.kind));

    switch (scene.kind)
    {
    case SceneKind::Status:
        draw_status_scene_(scene, accent_rgb888);
        break;
    case SceneKind::Provisioning:
        draw_provisioning_scene_(scene, accent_rgb888);
        break;
    case SceneKind::BindProgress:
        draw_bind_scene_(scene, accent_rgb888);
        break;
    case SceneKind::ErrorDetail:
        draw_error_scene_(scene, accent_rgb888);
        break;
    case SceneKind::UpdateProgress:
        draw_update_scene_(scene, accent_rgb888);
        break;
    case SceneKind::TestMenu:
        draw_test_menu_scene_(scene, accent_rgb888);
        break;
    }

    gfx_->startWrite();
    canvas_->pushSprite(0, 0);
    gfx_->endWrite();
}

void Ssd1351Display::draw_background_(const uint32_t accent_rgb888)
{
    canvas_->fillScreen(kColorBackground);
    canvas_->fillRect(0, 0, kWidth, 3, scale_rgb888(accent_rgb888, 88));
}

void Ssd1351Display::draw_shell_(const Scene &scene, const uint32_t accent_rgb888, const char *section_tag)
{
    (void)section_tag;
    const uint32_t accent_soft = scale_rgb888(accent_rgb888, 88);
    const uint32_t accent_wash = scale_rgb888(accent_rgb888, 44);

    canvas_->fillRoundRect(2, 4, 124, 122, 6, kColorShell);
    canvas_->drawRoundRect(2, 4, 124, 122, 6, kColorOutline);
    canvas_->fillRoundRect(4, 6, 120, 22, 4, kColorCardAlt);
    canvas_->fillRect(4, 26, 120, 2, accent_soft);
    draw_text_line_(10, 9, 73, scene.title, compact_font_for_(scene.title), kColorTextPrimary, lgfx::textdatum_t::top_left);
    draw_signal_icon_(84, 10, scene.signal_percent, scene.signal_visible, accent_rgb888);

    int battery_percent = 0;
    bool battery_charging = false;
    if (scene.kind == SceneKind::TestMenu && battery_status_from_scene(scene, &battery_percent, &battery_charging))
    {
        draw_battery_icon_(99, 10, 20, 11, battery_percent, battery_charging, accent_rgb888);
    }
    else
    {
        canvas_->fillRoundRect(101, 12, 18, 7, 3, accent_wash);
    }
}

void Ssd1351Display::draw_status_scene_(const Scene &scene, const uint32_t accent_rgb888)
{
    const uint32_t accent_soft = scale_rgb888(accent_rgb888, 80);

    canvas_->fillRoundRect(6, 36, 116, 52, 6, kColorCard);
    canvas_->fillRoundRect(6, 36, 5, 52, 3, accent_rgb888);
    draw_text_block_(16, 44, 98, 2, scene.detail, body_font_for_(scene.detail), kColorTextBody);
    draw_text_block_(8, 98, 112, 1, scene.aux, compact_font_for_(scene.aux), accent_soft);
}

void Ssd1351Display::draw_provisioning_scene_(const Scene &scene, const uint32_t accent_rgb888)
{
    canvas_->fillRoundRect(6, 36, 116, 56, 6, kColorCard);
    draw_text_line_(14, 44, 100, scene.detail, body_font_for_(scene.detail), accent_rgb888, lgfx::textdatum_t::top_left);
    draw_text_block_(14, 62, 100, 2, scene.aux, compact_font_for_(scene.aux), kColorTextBody);
    draw_progress_bar_(8, 106, 112, 10, scene.progress, accent_rgb888);
}

void Ssd1351Display::draw_bind_scene_(const Scene &scene, const uint32_t accent_rgb888)
{
    canvas_->fillRoundRect(6, 36, 116, 42, 6, kColorCardAlt);
    draw_text_block_(14, 44, 100, 2, scene.detail, body_font_for_(scene.detail), kColorTextPrimary);
    draw_progress_bar_(8, 86, 112, 10, scene.progress, accent_rgb888);
    draw_text_block_(8, 103, 112, 1, scene.aux, compact_font_for_(scene.aux), kColorTextMuted);
}

void Ssd1351Display::draw_error_scene_(const Scene &scene, const uint32_t accent_rgb888)
{
    constexpr uint32_t kAlertPanel = 0x2A121B;

    canvas_->fillRoundRect(6, 36, 116, 48, 6, kAlertPanel);
    canvas_->fillRoundRect(6, 36, 5, 48, 3, accent_rgb888);
    draw_text_block_(16, 44, 98, 2, scene.detail, body_font_for_(scene.detail), kColorTextWarning);
    draw_text_block_(8, 96, 112, 2, scene.aux, compact_font_for_(scene.aux), kColorTextBody);
}

void Ssd1351Display::draw_update_scene_(const Scene &scene, const uint32_t accent_rgb888)
{
    canvas_->fillRoundRect(6, 36, 116, 42, 6, kColorCardAlt);
    draw_text_line_(14, 44, 100, scene.detail, body_font_for_(scene.detail), accent_rgb888, lgfx::textdatum_t::top_left);
    draw_progress_bar_(8, 88, 112, 10, scene.progress, accent_rgb888);
    draw_text_block_(8, 106, 112, 1, scene.aux, compact_font_for_(scene.aux), kColorTextMuted);
}

void Ssd1351Display::draw_test_menu_scene_(const Scene &scene, const uint32_t accent_rgb888)
{
    const uint32_t accent_wash = scale_rgb888(accent_rgb888, 44);
    constexpr int kListX = 4;
    constexpr int kListY = 31;
    constexpr int kListW = 120;
    const int rows = scene.menu_count > 0 ? scene.menu_count : 0;
    const int row_h = rows >= 6 ? 16 : (rows >= 5 ? 17 : (rows >= 4 ? 20 : 24));
    const int list_h = rows > 0 ? std::min(96, rows * row_h) : row_h;

    canvas_->fillRoundRect(kListX, kListY, kListW, list_h, 5, kColorCard);
    canvas_->drawRoundRect(kListX, kListY, kListW, list_h, 5, kColorOutline);

    for (int i = 0; i < rows; ++i)
    {
        const int row_y = kListY + i * row_h;
        const bool is_selected = (scene.menu_index != kTestMenuNoCursor) && (static_cast<uint8_t>(i) == scene.menu_index);
        if (is_selected)
        {
            canvas_->fillRoundRect(kListX + 2, row_y + 1, kListW - 4, row_h - 2, 4, accent_wash);
            canvas_->fillRoundRect(kListX + 5, row_y + 4, 3, std::max(4, row_h - 8), 1, accent_rgb888);
        }
        else if ((i & 1) != 0)
        {
            canvas_->fillRect(kListX + 2, row_y + 1, kListW - 4, row_h - 1, kColorCardAlt);
        }

        const uint32_t color = is_selected ? kColorTextPrimary : kColorTextBody;
        int slider_filled = 0;
        int slider_total = 0;
        if (parse_slider_text(scene.menu_lines[i], &slider_filled, &slider_total))
        {
            draw_material_slider_(kListX + 11, row_y + (row_h / 2) - 4, kListW - 22, 8, slider_filled, slider_total, accent_rgb888);
            continue;
        }

        char switch_label[Ssd1351Display::kTestMenuLineLength] = {};
        bool switch_enabled = false;
        if (parse_switch_text(scene.menu_lines[i], switch_label, sizeof(switch_label), &switch_enabled))
        {
            constexpr int kSwitchW = 31;
            constexpr int kSwitchH = 13;
            const int switch_x = kListX + kListW - kSwitchW - 8;
            const int switch_y = row_y + (row_h / 2) - (kSwitchH / 2);
            draw_material_switch_(switch_x, switch_y, kSwitchW, kSwitchH, switch_enabled, accent_rgb888);

            int text_width = switch_x - (kListX + 11) - 5;
            const lgfx::IFont *font = body_font_for_(switch_label);
            if (canvas_->textWidth(switch_label, font) > text_width || rows >= 6)
            {
                font = compact_font_for_(switch_label);
            }
            const int text_y = row_y + std::max(1, (row_h - line_height_for(font)) / 2);
            draw_text_line_(kListX + 11, text_y, text_width, switch_label, font, color, lgfx::textdatum_t::top_left);
            continue;
        }

        char swatch_label[Ssd1351Display::kTestMenuLineLength] = {};
        uint32_t swatch_color = 0;
        if (parse_color_swatch_text(scene.menu_lines[i], swatch_label, sizeof(swatch_label), &swatch_color))
        {
            constexpr int kSwatchW = 34;
            constexpr int kSwatchH = 12;
            const int swatch_x = kListX + kListW - kSwatchW - 9;
            const int swatch_y = row_y + (row_h / 2) - (kSwatchH / 2);
            draw_color_swatch_(swatch_x, swatch_y, kSwatchW, kSwatchH, swatch_color, accent_rgb888);

            int text_width = swatch_x - (kListX + 11) - 6;
            const lgfx::IFont *font = body_font_for_(swatch_label);
            if (canvas_->textWidth(swatch_label, font) > text_width || rows >= 6)
            {
                font = compact_font_for_(swatch_label);
            }
            const int text_y = row_y + std::max(1, (row_h - line_height_for(font)) / 2);
            draw_text_line_(kListX + 11, text_y, text_width, swatch_label, font, color, lgfx::textdatum_t::top_left);
            continue;
        }

        int row_percent = 0;
        const bool has_battery_percent = parse_battery_percent(scene.menu_lines[i], &row_percent);
        const bool row_charging = text_has_on_token(scene.menu_lines[i]) || text_has_on_token(scene.aux);
        int text_width = kListW - 18;
        if (has_battery_percent)
        {
            draw_battery_icon_(kListX + kListW - 34, row_y + (row_h / 2) - 5, 24, 11, row_percent, row_charging, accent_rgb888);
            text_width -= 34;
        }

        const lgfx::IFont *font = body_font_for_(scene.menu_lines[i]);
        if (canvas_->textWidth(scene.menu_lines[i], font) > text_width || rows >= 6)
        {
            font = compact_font_for_(scene.menu_lines[i]);
        }
        const int text_y = row_y + std::max(1, (row_h - line_height_for(font)) / 2);
        draw_text_line_(kListX + 11, text_y, text_width, scene.menu_lines[i], font, color, lgfx::textdatum_t::top_left);
    }

    const int aux_y = kListY + list_h + 2;
    const int aux_lines = aux_y <= 111 ? 1 : 0;
    if (scene.aux[0] != '\0' && aux_lines > 0)
    {
        draw_text_block_(6, aux_y, 116, aux_lines, scene.aux, compact_font_for_(scene.aux), kColorTextMuted);
    }
}

void Ssd1351Display::draw_battery_icon_(const int x,
                                        const int y,
                                        const int width,
                                        const int height,
                                        const int percent,
                                        const bool charging,
                                        const uint32_t accent_rgb888)
{
    const int clamped = std::clamp(percent, 0, 100);
    const int body_width = std::max(8, width - 3);
    const int fill_width = ((body_width - 4) * clamped) / 100;
    const uint32_t outline_color = charging ? accent_rgb888 : kColorTextMuted;
    const uint32_t cap_color = charging ? accent_rgb888 : kColorTextMuted;
    const uint32_t level_color = clamped <= 20 && !charging ? kColorBatteryLow : (charging ? accent_rgb888 : kColorBatteryGood);

    canvas_->fillRoundRect(x, y, body_width, height, 2, kColorBackground);
    canvas_->drawRoundRect(x, y, body_width, height, 2, outline_color);
    canvas_->fillRoundRect(x + body_width, y + (height / 3), 3, std::max(3, height / 3), 1, cap_color);
    if (fill_width > 0)
    {
        canvas_->fillRoundRect(x + 2, y + 2, fill_width, std::max(1, height - 4), 1, level_color);
    }
    if (charging)
    {
        const int bolt_x = x + (body_width / 2) - 2;
        canvas_->drawLine(bolt_x + 2, y + 1, bolt_x, y + (height / 2), kColorTextPrimary);
        canvas_->drawLine(bolt_x + 1, y + 1, bolt_x - 1, y + (height / 2), kColorTextPrimary);
        canvas_->drawLine(bolt_x, y + (height / 2), bolt_x + 5, y + (height / 2), kColorTextPrimary);
        canvas_->drawLine(bolt_x + 4, y + (height / 2), bolt_x + 1, y + height - 1, kColorTextPrimary);
        canvas_->drawLine(bolt_x + 5, y + (height / 2), bolt_x + 2, y + height - 1, kColorTextPrimary);
    }
}

void Ssd1351Display::draw_color_swatch_(const int x,
                                        const int y,
                                        const int width,
                                        const int height,
                                        const uint32_t color_rgb888,
                                        const uint32_t accent_rgb888)
{
    const uint32_t outline = scale_rgb888(accent_rgb888, 130);
    canvas_->fillRoundRect(x, y, width, height, 3, kColorBackground);
    canvas_->drawRoundRect(x, y, width, height, 3, outline);
    canvas_->fillRoundRect(x + 2, y + 2, width - 4, height - 4, 2, color_rgb888 & 0xFFFFFFU);
    canvas_->drawLine(x + 3, y + 3, x + width - 4, y + 3, scale_rgb888(kColorTextPrimary, 80));
}

void Ssd1351Display::draw_signal_icon_(const int x,
                                       const int y,
                                       const int percent,
                                       const bool visible,
                                       const uint32_t accent_rgb888)
{
    constexpr int kBarCount = 4;
    constexpr int kBarWidth = 2;
    constexpr int kBarGap = 2;
    const int clamped = std::clamp(percent, 0, 100);
    for (int index = 0; index < kBarCount; ++index)
    {
        const int height = 3 + (index * 2);
        const int threshold = 12 + (index * 25);
        const uint32_t color = (visible && clamped >= threshold) ? accent_rgb888 : kColorTrack;
        canvas_->fillRoundRect(x + (index * (kBarWidth + kBarGap)), y + 10 - height, kBarWidth, height, 1, color);
    }
}

void Ssd1351Display::draw_material_switch_(const int x,
                                           const int y,
                                           const int width,
                                           const int height,
                                           const bool enabled,
                                           const uint32_t accent_rgb888)
{
    const int radius = height / 2;
    const uint32_t track = enabled ? scale_rgb888(accent_rgb888, 96) : kColorTrack;
    const uint32_t outline = enabled ? accent_rgb888 : kColorOutline;
    const int knob_radius = std::max(3, (height - 4) / 2);
    const int knob_x = enabled ? (x + width - radius) : (x + radius);
    const int knob_y = y + radius;

    canvas_->fillRoundRect(x, y, width, height, radius, track);
    canvas_->drawRoundRect(x, y, width, height, radius, outline);
    canvas_->fillCircle(knob_x, knob_y, knob_radius, enabled ? kColorTextPrimary : kColorTextMuted);
}

void Ssd1351Display::draw_material_slider_(const int x,
                                           const int y,
                                           const int width,
                                           const int height,
                                           const int filled_segments,
                                           const int total_segments,
                                           const uint32_t accent_rgb888)
{
    const int total = std::max(1, total_segments);
    const int filled_width = ((width - 4) * std::clamp(filled_segments, 0, total)) / total;

    canvas_->fillRoundRect(x, y, width, height, height / 2, kColorTrack);
    if (filled_width > 0)
    {
        canvas_->fillRoundRect(x + 2, y + 2, filled_width, std::max(1, height - 4), std::max(1, (height - 4) / 2), accent_rgb888);
    }
    const int knob_x = std::clamp(x + 2 + filled_width, x + 4, x + width - 4);
    canvas_->fillCircle(knob_x, y + (height / 2), 4, kColorTextPrimary);
}

void Ssd1351Display::draw_progress_bar_(const int x, const int y, const int width, const int height, const uint8_t percent, const uint32_t accent_rgb888)
{
    const int safe_percent = clamp_progress(percent);
    const int fill_width = std::max(1, ((width - 4) * safe_percent) / 100);
    const uint32_t accent_soft = scale_rgb888(accent_rgb888, 72);

    canvas_->fillRoundRect(x, y, width, height, height / 2, kColorTrack);
    canvas_->fillRoundRect(x + 2, y + 2, fill_width, std::max(1, height - 4), std::max(1, (height - 4) / 2), accent_rgb888);
    canvas_->drawRoundRect(x, y, width, height, height / 2, accent_soft);

    char percent_text[8] = {};
    std::snprintf(percent_text, sizeof(percent_text), "%u%%", static_cast<unsigned>(safe_percent));
    draw_text_line_(x, y - 16, width, percent_text, body_font_for_(percent_text), kColorTextMuted, lgfx::textdatum_t::top_right);
}

void Ssd1351Display::draw_text_block_(const int x,
                                      int y,
                                      const int width,
                                      const int max_lines,
                                      const char *text,
                                      const lgfx::IFont *font,
                                      const uint32_t color)
{
    if (canvas_ == nullptr || text == nullptr || text[0] == '\0' || max_lines <= 0)
    {
        return;
    }

    const std::string source(text);
    size_t cursor = 0;
    int rendered_lines = 0;
    const int line_height = line_height_for(font);

    while (cursor < source.size() && rendered_lines < max_lines)
    {
        while (cursor < source.size() && (source[cursor] == ' ' || source[cursor] == '\n' || source[cursor] == '\r' || source[cursor] == '\t'))
        {
            ++cursor;
        }
        if (cursor >= source.size())
        {
            break;
        }

        size_t probe = cursor;
        size_t line_end = cursor;
        size_t last_space = std::string::npos;

        while (probe < source.size())
        {
            if (source[probe] == '\n' || source[probe] == '\r')
            {
                line_end = probe;
                break;
            }

            const size_t span = utf8_span(source.c_str() + probe);
            const size_t next = std::min(source.size(), probe + span);
            const std::string candidate = source.substr(cursor, next - cursor);
            if (canvas_->textWidth(candidate.c_str(), font) > width)
            {
                if (last_space != std::string::npos && last_space > cursor)
                {
                    line_end = last_space;
                }
                else if (probe > cursor)
                {
                    line_end = probe;
                }
                else
                {
                    line_end = next;
                }
                break;
            }

            if (source[probe] == ' ')
            {
                last_space = probe;
            }

            line_end = next;
            probe = next;
        }

        if (line_end <= cursor)
        {
            break;
        }

        std::string line = source.substr(cursor, line_end - cursor);
        trim_ascii(line);
        if (!line.empty())
        {
            draw_text_line_(x, y, width, line.c_str(), font, color, lgfx::textdatum_t::top_left);
            y += line_height;
            ++rendered_lines;
        }

        cursor = line_end;
        while (cursor < source.size() && (source[cursor] == ' ' || source[cursor] == '\n' || source[cursor] == '\r' || source[cursor] == '\t'))
        {
            ++cursor;
        }
    }
}

void Ssd1351Display::draw_text_line_(const int x,
                                     const int y,
                                     const int width,
                                     const char *text,
                                     const lgfx::IFont *font,
                                     const uint32_t color,
                                     const int datum)
{
    if (canvas_ == nullptr || text == nullptr || text[0] == '\0')
    {
        return;
    }

    int anchor_x = x;
    switch (static_cast<lgfx::textdatum_t>(datum))
    {
    case lgfx::textdatum_t::top_center:
        anchor_x = x + (width / 2);
        break;
    case lgfx::textdatum_t::top_right:
        anchor_x = x + width;
        break;
    default:
        break;
    }

    canvas_->setTextColor(color);
    canvas_->setTextDatum(static_cast<lgfx::textdatum_t>(datum));
    canvas_->drawString(text, anchor_x, y, font);
    canvas_->setTextDatum(lgfx::textdatum_t::top_left);
}

bool Ssd1351Display::contains_cjk_(const char *text) const
{
    if (text == nullptr)
    {
        return false;
    }

    for (size_t index = 0; text[index] != '\0'; ++index)
    {
        if ((static_cast<unsigned char>(text[index]) & 0x80U) != 0)
        {
            return true;
        }
    }
    return false;
}

const lgfx::IFont *Ssd1351Display::heading_font_for_(const char *text) const
{
    return contains_cjk_(text) ? static_cast<const lgfx::IFont *>(&fonts::efontCN_16_b) : static_cast<const lgfx::IFont *>(&fonts::DejaVu18);
}

const lgfx::IFont *Ssd1351Display::body_font_for_(const char *text) const
{
    return contains_cjk_(text) ? static_cast<const lgfx::IFont *>(&fonts::efontCN_14) : static_cast<const lgfx::IFont *>(&fonts::DejaVu12);
}

const lgfx::IFont *Ssd1351Display::compact_font_for_(const char *text) const
{
    return body_font_for_(text);
}

const char *Ssd1351Display::scene_tag_for_(const SceneKind kind) const
{
    switch (kind)
    {
    case SceneKind::Provisioning:
        return localized(language_, "WIFI", "无线");
    case SceneKind::BindProgress:
        return localized(language_, "BLE", "蓝牙");
    case SceneKind::ErrorDetail:
        return localized(language_, "FAULT", "异常");
    case SceneKind::UpdateProgress:
        return localized(language_, "SYNC", "同步");
    case SceneKind::TestMenu:
        return localized(language_, "TEST", "测试");
    case SceneKind::Status:
    default:
        return localized(language_, "REMOTE", "遥控器");
    }
}

uint32_t Ssd1351Display::expand_rgb565_(const uint16_t rgb565) const
{
    const uint32_t red = ((rgb565 >> 11) & 0x1FU) * 255U / 31U;
    const uint32_t green = ((rgb565 >> 5) & 0x3FU) * 255U / 63U;
    const uint32_t blue = (rgb565 & 0x1FU) * 255U / 31U;
    return (red << 16) | (green << 8) | blue;
}
} // namespace prototracer
