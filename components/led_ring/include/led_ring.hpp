#pragma once

#include <atomic>
#include <cstdint>

#include "led_strip.h"

#include "task.hpp"    // espp::Task
#include "logger.hpp"  // espp::Logger

#include "nvs_config.hpp"  // FanMode

// Shared state written by FanControl (and DustSensor via dust_q) and read by LedRing.
// All fields are individually atomic — no mutex needed for single-field access.
struct SystemState {
    std::atomic<float>    dust_ugm3{0.0f};
    std::atomic<FanMode>  mode{FanMode::Auto};
    std::atomic<bool>     fan_on{false};
};

class LedRing {
public:
    struct Config {
        int          gpio_data;  // GPIO 18
        uint32_t     num_leds;   // 8
        SystemState *state;
    };

    explicit LedRing(Config cfg);
    void start();

private:
    bool task_fn(std::mutex &m, std::condition_variable &cv);

    // Zone A: LED 0 — operational mode indicator
    void update_zone_a(FanMode mode);

    // Zone B: LEDs 1–7 — PM2.5 air quality bar
    void update_zone_b(float dust_ugm3);

    Config             cfg_;
    espp::Task         task_;
    espp::Logger       logger_{{.tag = "LedRing", .level = espp::Logger::Verbosity::INFO}};

    led_strip_handle_t strip_{nullptr};

    // Breathing animation state (Zone A in AUTO mode)
    uint32_t breath_tick_{0};
    static constexpr uint32_t kBreathPeriod = 60; // ticks; 60 × 50 ms = 3 s period

    // Flashing state (Zone B at hazardous level)
    uint32_t flash_tick_{0};
    static constexpr uint32_t kFlashPeriod = 10;  // ticks; 5 on / 5 off = 500 ms cycle

    // Task tick period in milliseconds
    static constexpr uint32_t kTickMs = 50;

    // Zone B thresholds (µg/m³) — fixed per spec, not user-configurable
    static constexpr float kGreenMax  =  35.0f;
    static constexpr float kYellowMax =  75.0f;
    static constexpr float kOrangeMax = 150.0f;
};
