#pragma once

#include <atomic>
#include <cstdint>

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"

#include "task.hpp"   // espp::Task
#include "logger.hpp" // espp::Logger

#include "nvs_config.hpp" // FanMode

// Shared state snapshot written by FanControl and read by Display.
// All fields are individually atomic so no mutex is needed for single-field reads/writes.
struct DisplayState {
    std::atomic<float>    dust_mgm3{0.0f};
    std::atomic<FanMode>  mode{FanMode::Auto};
    std::atomic<bool>     fan_on{false};
    std::atomic<uint32_t> heartbeat{0}; // incremented each display refresh
};

class Display {
public:
    struct Config {
        int           gpio_sda; // GPIO 8
        int           gpio_scl; // GPIO 9
        DisplayState *state;
    };

    explicit Display(Config cfg);
    void start();

private:
    bool task_fn(std::mutex &m, std::condition_variable &cv);

    // Low-level rendering helpers (write to internal framebuffer then flush)
    void render();
    void draw_text(int x, int row, const char *text);
    void flush();

    Config        cfg_;
    espp::Task    task_;
    espp::Logger  logger_{{.tag = "Display", .level = espp::Logger::Verbosity::INFO}};

    esp_lcd_panel_handle_t    panel_{nullptr};
    esp_lcd_panel_io_handle_t io_{nullptr};

    // SSD1306 128×64 monochrome framebuffer (1 bit per pixel, packed as bytes)
    static constexpr int kWidth  = 128;
    static constexpr int kHeight = 64;
    static constexpr int kPages  = kHeight / 8; // 8 pages of 8 rows each
    uint8_t framebuf_[kPages][kWidth]{};
};
