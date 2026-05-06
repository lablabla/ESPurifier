#pragma once

#include <chrono>
#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#include "task.hpp"    // espp::Task
#include "logger.hpp"  // espp::Logger

#include "nvs_config.hpp"  // FanMode
#include "led_ring.hpp"    // SystemState
#include "rf_remote.hpp"   // RfCommand

class FanControl {
public:
    struct Config {
        int            gpio_ssr;
        QueueHandle_t  dust_q;
        QueueHandle_t  mode_cmd_q;
        SystemState   *system_state;
        float          threshold_high;  // µg/m³
        float          threshold_low;   // µg/m³
        FanMode        initial_mode;
    };

    explicit FanControl(Config cfg);
    void start();

private:
    bool task_fn(std::mutex &m, std::condition_variable &cv);
    void set_fan(bool on);
    void apply_auto_logic(float dust);
    void handle_mode_command(RfCommand cmd);

    Config       cfg_;
    espp::Task   task_;
    espp::Logger logger_{{.tag = "FanControl", .level = espp::Logger::Verbosity::INFO}};

    bool    fan_on_{false};
    FanMode mode_;
    FanMode previous_mode_{FanMode::Auto};  // restored when Button B lifts from OFF

    using Clock     = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    TimePoint fan_on_since_{};
    TimePoint manual_high_until_{};

    static constexpr auto kMinRunTime    = std::chrono::minutes{5};
    static constexpr auto kManualHighDur = std::chrono::minutes{30};
};
