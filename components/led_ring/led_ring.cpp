#include "led_ring.hpp"

#include <algorithm>
#include <cmath>

// ─── Helpers ─────────────────────────────────────────────────────────────────

// Map a dust concentration (µg/m³) to a bar count [1, 7].
static int dust_to_bar_leds(float dust) {
    if (dust <  17.5f) return 1;
    if (dust <  35.0f) return 2;
    if (dust <  55.0f) return 3;
    if (dust <  75.0f) return 4;
    if (dust < 112.5f) return 5;
    if (dust < 150.0f) return 6;
    return 7;
}

// RGB values for each bar count (index 0 unused; 1–6 used; 7 = flashing red)
struct Rgb { uint8_t r, g, b; };
static constexpr Rgb kBarColours[8] = {
    {  0,   0,   0},  // 0 — unused
    {  0, 200,   0},  // 1 — Green
    {  0, 200,   0},  // 2 — Green
    {200, 200,   0},  // 3 — Yellow
    {200, 200,   0},  // 4 — Yellow
    {255, 100,   0},  // 5 — Orange
    {255, 100,   0},  // 6 — Orange
    {255,   0,   0},  // 7 — Red (flashing handled in update_zone_b)
};

// ─── Constructor ─────────────────────────────────────────────────────────────

LedRing::LedRing(Config cfg)
    : cfg_{cfg}
    , task_{{
          .callback = [this](std::mutex &m, std::condition_variable &cv) {
              return task_fn(m, cv);
          },
          .task_config = {
              .name             = "led_ring",
              .stack_size_bytes = 4096,
              .priority         = 3,
              .core_id          = 0,
          },
      }}
{
    led_strip_config_t strip_cfg{
        .strip_gpio_num   = cfg_.gpio_data,
        .max_leds         = cfg_.num_leds,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,  // WS2812B uses GRB order
        .led_model        = LED_MODEL_WS2812,
        .flags            = { .invert_out = false },
    };
    led_strip_rmt_config_t rmt_cfg{
        .clk_src       = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10'000'000,  // 10 MHz — standard for WS2812B timing
        .flags         = { .with_dma = false },
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &strip_));
    ESP_ERROR_CHECK(led_strip_clear(strip_));
    logger_.info("WS2812B ring initialised ({} LEDs on GPIO {})", cfg_.num_leds, cfg_.gpio_data);
}

void LedRing::start() {
    task_.start();
}

// ─── Task body ───────────────────────────────────────────────────────────────

bool LedRing::task_fn(std::mutex &, std::condition_variable &) {
    const FanMode mode  = cfg_.state->mode.load();
    const float   dust  = cfg_.state->dust_ugm3.load();

    update_zone_a(mode);
    update_zone_b(dust);

    led_strip_refresh(strip_);
    vTaskDelay(pdMS_TO_TICKS(kTickMs));
    return false;
}

// ─── Zone A: LED 0 — mode indicator ──────────────────────────────────────────

void LedRing::update_zone_a(FanMode mode) {
    switch (mode) {
        case FanMode::Off:
            // Solid Red
            led_strip_set_pixel(strip_, 0, 255, 0, 0);
            break;

        case FanMode::Auto: {
            // Breathing Yellow — smooth sine wave, minimum brightness 5 %
            const float angle      = 2.0f * static_cast<float>(M_PI)
                                   * static_cast<float>(breath_tick_ % kBreathPeriod)
                                   / static_cast<float>(kBreathPeriod);
            const float brightness = 0.05f + 0.95f * (sinf(angle) + 1.0f) / 2.0f;
            ++breath_tick_;

            const auto r = static_cast<uint8_t>(255 * brightness);
            const auto g = static_cast<uint8_t>(180 * brightness);
            led_strip_set_pixel(strip_, 0, r, g, 0);
            break;
        }

        case FanMode::ManualHigh:
            // Solid Green
            led_strip_set_pixel(strip_, 0, 0, 255, 0);
            break;
    }
}

// ─── Zone B: LEDs 1–7 — air quality bar ──────────────────────────────────────

void LedRing::update_zone_b(float dust) {
    const int  lit     = dust_to_bar_leds(dust);
    const bool hazard  = dust >= kOrangeMax;
    const bool flash_on = hazard && (flash_tick_ % kFlashPeriod) < (kFlashPeriod / 2);

    if (hazard) ++flash_tick_;

    const Rgb &col = kBarColours[lit];

    for (int i = 1; i <= 7; ++i) {
        if (i <= lit) {
            if (hazard && !flash_on) {
                led_strip_set_pixel(strip_, i, 0, 0, 0);  // flash off
            } else {
                led_strip_set_pixel(strip_, i, col.r, col.g, col.b);
            }
        } else {
            led_strip_set_pixel(strip_, i, 0, 0, 0);  // unlit
        }
    }
}
