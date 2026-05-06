#include "display.hpp"

#include <cstdio>
#include <cstring>

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "driver/i2c_master.h"

// SSD1306 I2C address (SA0 pin low = 0x3C, high = 0x3D)
static constexpr uint8_t  kSSD1306Addr = 0x3C;
static constexpr uint32_t kI2CFreqHz   = 400'000;

// Minimal 5×7 ASCII font (printable range 0x20–0x7E), 5 bytes per glyph.
// Each byte is a column of 8 pixels (LSB = top row).
// Only a small subset is needed; others default to 0 (blank).
static const uint8_t kFont5x7[][5] = {
    // 0x20 ' '
    {0x00, 0x00, 0x00, 0x00, 0x00},
    // 0x21 '!'
    {0x00, 0x00, 0x5F, 0x00, 0x00},
    // ... (full font omitted for brevity — replace with a complete 5×7 font table)
    // Digits 0x30–0x39
    // 0x30 '0'
    {0x3E, 0x51, 0x49, 0x45, 0x3E},
    // 0x31 '1'
    {0x00, 0x42, 0x7F, 0x40, 0x00},
    // 0x32 '2'
    {0x42, 0x61, 0x51, 0x49, 0x46},
    // 0x33 '3'
    {0x21, 0x41, 0x45, 0x4B, 0x31},
    // 0x34 '4'
    {0x18, 0x14, 0x12, 0x7F, 0x10},
    // 0x35 '5'
    {0x27, 0x45, 0x45, 0x45, 0x39},
    // 0x36 '6'
    {0x3C, 0x4A, 0x49, 0x49, 0x30},
    // 0x37 '7'
    {0x01, 0x71, 0x09, 0x05, 0x03},
    // 0x38 '8'
    {0x36, 0x49, 0x49, 0x49, 0x36},
    // 0x39 '9'
    {0x06, 0x49, 0x49, 0x29, 0x1E},
};

// Resolve a character to its font glyph (returns blank for unmapped chars)
static const uint8_t *glyph_for(char c) {
    if (c == ' ') return kFont5x7[0];
    if (c >= '0' && c <= '9') return kFont5x7[2 + (c - '0')];
    return kFont5x7[0]; // blank fallback
}

// ─── Constructor ─────────────────────────────────────────────────────────────

Display::Display(Config cfg)
    : cfg_{cfg}
    , task_{{
          .callback = [this](std::mutex &m, std::condition_variable &cv) {
              return task_fn(m, cv);
          },
          .task_config = {
              .name             = "display",
              .stack_size_bytes = 6144,
              .priority         = 2,
              .core_id          = 0,
          },
      }}
{
    // Set up I2C master bus
    i2c_master_bus_config_t bus_cfg{
        .i2c_port      = I2C_NUM_0,
        .sda_io_num    = static_cast<gpio_num_t>(cfg_.gpio_sda),
        .scl_io_num    = static_cast<gpio_num_t>(cfg_.gpio_scl),
        .clk_source    = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = { .enable_internal_pullup = true },
    };
    i2c_master_bus_handle_t bus{};
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

    // Attach SSD1306 as an LCD panel via esp_lcd I2C IO
    esp_lcd_panel_io_i2c_config_t io_cfg{
        .dev_addr            = kSSD1306Addr,
        .scl_speed_hz        = kI2CFreqHz,
        .control_phase_bytes = 1,
        .dc_bit_offset       = 6,
        .lcd_cmd_bits        = 8,
        .lcd_param_bits      = 8,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(
        reinterpret_cast<esp_lcd_i2c_bus_handle_t>(bus), &io_cfg, &io_));

    // Create SSD1306 panel driver
    esp_lcd_panel_dev_config_t panel_cfg{
        .reset_gpio_num = GPIO_NUM_NC,
        .bits_per_pixel = 1,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_, &panel_cfg, &panel_));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_, true));

    memset(framebuf_, 0, sizeof(framebuf_));
    logger_.info("SSD1306 initialised");
}

void Display::start() {
    task_.start();
}

// ─── Task body ───────────────────────────────────────────────────────────────

bool Display::task_fn(std::mutex &, std::condition_variable &) {
    render();
    cfg_.state->heartbeat.fetch_add(1);
    vTaskDelay(pdMS_TO_TICKS(1000));
    return false;
}

// ─── Rendering ───────────────────────────────────────────────────────────────

void Display::render() {
    memset(framebuf_, 0, sizeof(framebuf_));

    const float    dust    = cfg_.state->dust_mgm3.load();
    const FanMode  mode    = cfg_.state->mode.load();
    const bool     fan_on  = cfg_.state->fan_on.load();
    const uint32_t hb      = cfg_.state->heartbeat.load();

    char line[22]{};

    // Row 0: dust level
    snprintf(line, sizeof(line), "Dust: %.3f mg/m3", dust);
    draw_text(0, 0, line);

    // Row 1: mode
    const char *mode_str =
        mode == FanMode::Off        ? "Mode: OFF"  :
        mode == FanMode::ManualHigh ? "Mode: MANUAL" : "Mode: AUTO";
    draw_text(0, 1, mode_str);

    // Row 2: fan state
    draw_text(0, 2, fan_on ? "Fan:  ON" : "Fan:  OFF");

    // Row 3: heartbeat (alternating indicator)
    snprintf(line, sizeof(line), "HB: %s  [%lu]", (hb & 1) ? "*" : "o", (unsigned long)hb);
    draw_text(0, 3, line);

    flush();
}

void Display::draw_text(int x, int row, const char *text) {
    // row is a character row (8 px tall). row 0 → page 0, row 1 → page 1, etc.
    if (row >= kPages) return;

    int cx = x;
    for (const char *p = text; *p && cx + 5 <= kWidth; ++p) {
        const uint8_t *g = glyph_for(*p);
        for (int col = 0; col < 5; ++col) {
            framebuf_[row][cx + col] = g[col];
        }
        cx += 6; // 5 px glyph + 1 px spacing
    }
}

void Display::flush() {
    // esp_lcd_panel_draw_bitmap expects (x_start, y_start, x_end, y_end, data)
    // For SSD1306, data layout is page-by-page (128 bytes per page × 8 pages)
    esp_lcd_panel_draw_bitmap(panel_, 0, 0, kWidth, kHeight, framebuf_);
}
