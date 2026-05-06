#include "dust_sensor.hpp"

#include "esp_rom_sys.h"   // esp_rom_delay_us

DustSensor::DustSensor(Config cfg)
    : cfg_{cfg}
    , adc_cfg_{
          .unit        = ADC_UNIT_1,
          .channel     = cfg.adc_channel,
          .attenuation = ADC_ATTEN_DB_12,
      }
    , adc_{{
          .unit     = ADC_UNIT_1,
          .channels = {adc_cfg_},
      }}
    , task_{{
          .callback = [this](std::mutex &m, std::condition_variable &cv) {
              return task_fn(m, cv);
          },
          .task_config = {
              .name             = "dust_sensor",
              .stack_size_bytes = 4096,
              .priority         = 5,
              .core_id          = 1,
          },
      }}
{
    // Configure LED control GPIO
    gpio_config_t io_cfg{
        .pin_bit_mask = 1ULL << cfg_.gpio_led,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_cfg);
    gpio_set_level(static_cast<gpio_num_t>(cfg_.gpio_led), 0);
}

void DustSensor::start() {
    task_.start();
}

bool DustSensor::task_fn(std::mutex &, std::condition_variable &) {
    const float reading = sample_once();

    // Non-blocking post; drop reading if fan_control task is slow
    xQueueSend(cfg_.dust_q, &reading, 0);

    vTaskDelay(pdMS_TO_TICKS(cfg_.sample_interval_ms));
    return false; // keep running
}

float DustSensor::sample_once() {
    // Sharp GP2Y1010AU0F pulse sequence (datasheet timing):
    //   1. LED ON
    //   2. Wait 280 µs
    //   3. Sample ADC
    //   4. Wait 40 µs
    //   5. LED OFF
    gpio_set_level(static_cast<gpio_num_t>(cfg_.gpio_led), 1);
    esp_rom_delay_us(280);

    const auto maybe_mv = adc_.read_mv(adc_cfg_);

    esp_rom_delay_us(40);
    gpio_set_level(static_cast<gpio_num_t>(cfg_.gpio_led), 0);

    if (!maybe_mv) {
        // ADC read failed — reuse last valid value in the buffer
        return filter_buf_[filter_idx_ % kFilterLen];
    }

    const float v_volts = static_cast<float>(*maybe_mv) / 1000.0f;
    filter_buf_[filter_idx_++ % kFilterLen] = voltage_to_ugm3(v_volts);

    const float sum = std::reduce(filter_buf_.begin(), filter_buf_.end());
    return sum / static_cast<float>(kFilterLen);
}
