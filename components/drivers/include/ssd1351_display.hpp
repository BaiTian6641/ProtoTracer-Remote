#pragma once

#include <cstdint>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "prototracer_types.hpp"

namespace lgfx
{
inline namespace v1
{
class IFont;
class LGFX_Device;
class LGFX_Sprite;
} // namespace v1
} // namespace lgfx

namespace prototracer
{
class Ssd1351Display
{
public:
    static constexpr int kWidth = 128;
    static constexpr int kHeight = 128;

    enum class SceneKind : uint8_t
    {
        Status = 0,
        Provisioning = 1,
        BindProgress = 2,
        ErrorDetail = 3,
        UpdateProgress = 4,
        TestMenu = 5,
    };

    static constexpr uint8_t kTestMenuMaxLines = 6;
    static constexpr uint8_t kTestMenuLineLength = 22;
    static constexpr uint8_t kTestMenuNoCursor = 0xFF;

    struct Scene
    {
        SceneKind kind = SceneKind::Status;
        uint16_t accent_rgb565 = 0;
        uint8_t progress = 0;
        char title[24] = {};
        char detail[72] = {};
        char aux[72] = {};
        uint8_t menu_count = 0;
        uint8_t menu_index = kTestMenuNoCursor;
        uint8_t signal_percent = 0;
        bool signal_visible = false;
        char menu_lines[kTestMenuMaxLines][kTestMenuLineLength] = {};
    };

    esp_err_t init();
    void set_language(UiLanguage language);
    esp_err_t set_brightness(uint8_t brightness);
    esp_err_t set_sleeping(bool sleeping);
    void set_signal_strength(uint8_t percent, bool visible);
    esp_err_t show_status_screen(const char *title, const char *detail, uint16_t accent_rgb565);
    esp_err_t show_provisioning_screen(const char *portal_name, const char *hint, uint8_t progress_percent);
    esp_err_t show_bind_progress_screen(const char *phase, uint8_t progress_percent);
    esp_err_t show_error_screen(const char *title, const char *detail, const char *hint);
    esp_err_t show_update_progress_screen(const char *phase, uint8_t progress_percent);
    esp_err_t show_test_menu_screen(const char *title,
                                    const char *aux,
                                    const char (*lines)[kTestMenuLineLength],
                                    uint8_t line_count,
                                    uint8_t selected_index,
                                    uint16_t accent_rgb565);

private:
    esp_err_t init_panel_power_();
    esp_err_t enqueue_scene_(const Scene &scene);
    static void render_task_entry_(void *arg);
    void render_task_loop_();
    void render_scene_(const Scene &scene);
    void draw_background_(uint32_t accent_rgb888);
    void draw_shell_(const Scene &scene, uint32_t accent_rgb888, const char *section_tag);
    void draw_status_scene_(const Scene &scene, uint32_t accent_rgb888);
    void draw_provisioning_scene_(const Scene &scene, uint32_t accent_rgb888);
    void draw_bind_scene_(const Scene &scene, uint32_t accent_rgb888);
    void draw_error_scene_(const Scene &scene, uint32_t accent_rgb888);
    void draw_update_scene_(const Scene &scene, uint32_t accent_rgb888);
    void draw_test_menu_scene_(const Scene &scene, uint32_t accent_rgb888);
    void draw_battery_icon_(int x, int y, int width, int height, int percent, bool charging, uint32_t accent_rgb888);
    void draw_signal_icon_(int x, int y, int percent, bool visible, uint32_t accent_rgb888);
    void draw_material_switch_(int x, int y, int width, int height, bool enabled, uint32_t accent_rgb888);
    void draw_material_slider_(int x, int y, int width, int height, int filled_segments, int total_segments, uint32_t accent_rgb888);
    void draw_progress_bar_(int x, int y, int width, int height, uint8_t percent, uint32_t accent_rgb888);
    void draw_text_block_(int x, int y, int width, int max_lines, const char *text, const lgfx::IFont *font, uint32_t color);
    void draw_text_line_(int x, int y, int width, const char *text, const lgfx::IFont *font, uint32_t color, int datum);
    bool contains_cjk_(const char *text) const;
    const lgfx::IFont *heading_font_for_(const char *text) const;
    const lgfx::IFont *body_font_for_(const char *text) const;
    const lgfx::IFont *compact_font_for_(const char *text) const;
    const char *scene_tag_for_(SceneKind kind) const;
    uint32_t expand_rgb565_(uint16_t rgb565) const;

    QueueHandle_t render_queue_ = nullptr;
    TaskHandle_t render_task_ = nullptr;
    lgfx::LGFX_Device *gfx_ = nullptr;
    lgfx::LGFX_Sprite *canvas_ = nullptr;
    UiLanguage language_ = UiLanguage::English;
    uint8_t brightness_ = 192;
    uint8_t signal_percent_ = 0;
    bool signal_visible_ = false;
    bool sleeping_ = false;
    bool initialized_ = false;
};
} // namespace prototracer