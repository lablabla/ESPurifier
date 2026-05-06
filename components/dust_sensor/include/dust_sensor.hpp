#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <numeric>   // std::reduce

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"

#include "oneshot_adc.hpp"  // espp::OneshotAdc, espp::AdcConfig
#include "task.hpp"         // espp::Task
#include "logger.hpp"       // espp::Logger

class DustSensor {
public:
    struct Config {
        int           gpio_led;
        adc_channel_t adc_channel;
        QueueHandle_t dust_q;
        uint32_t      sample_interval_ms = 10;
    };

    explicit DustSensor(Config cfg);
    void start();

private:
    bool  task_fn(std::mutex &m, std::condition_variable &cv);
    float sample_once();

    // Sharp GP2Y1010AU0F datasheet conversion:
    //   Voc = 0.9 V (no dust), sensitivity = 0.5 mV per µg/m³
    //   density [µg/m³] = (voltage - 0.9) / 0.0005
    static constexpr float voltage_to_ugm3(float v_volts) {
        constexpr float kVoc      = 0.9f;
        constexpr float kSens_vpu = 0.0005f;  // V per µg/m³
        const float density = (v_volts - kVoc) / kSens_vpu;
        return density > 0.0f ? density : 0.0f;
    }

    Config           cfg_;
    espp::AdcConfig  adc_cfg_;
    espp::OneshotAdc adc_;
    espp::Task       task_;
    espp::Logger     logger_{{.tag = "DustSensor", .level = espp::Logger::Verbosity::INFO}};

    // 20-sample circular buffer for moving average
    static constexpr size_t kFilterLen = 20;
    std::array<float, kFilterLen> filter_buf_{};
    size_t                        filter_idx_{0};
};
