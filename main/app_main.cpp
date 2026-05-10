#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"
#include "nvs_flash.h"

#include "nvs_config.hpp"
#include "dust_sensor.hpp"
#include "rf_remote.hpp"
#include "fan_control.hpp"
#include "led_ring.hpp"
#include "pressure_sensor.hpp"

extern "C" void app_main() {
    // NVS must be initialised before any espp::Nvs usage
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    NvsConfig nvs_cfg;
    const AppConfig cfg = nvs_cfg.load();

    // Shared state — written by FanControl/DustSensor, read by LedRing
    SystemState system_state;

    // Inter-task queues
    QueueHandle_t dust_q     = xQueueCreate(5, sizeof(float));
    QueueHandle_t mode_cmd_q = xQueueCreate(5, sizeof(RfCommand));

    // Construct all components (tasks are NOT started yet)
    DustSensor dust{{
        .gpio_led           = 4,
        .adc_channel        = ADC_CHANNEL_4,
        .dust_q             = dust_q,
        .sample_interval_ms = 10,
    }};

    RfRemote rf{{
        .gpio_data   = 7,
        .mode_cmd_q  = mode_cmd_q,
        .paired_addr = 0,   // accept any EV1527 address
    }};

    FanControl fan{{
        .gpio_ssr       = 6,
        .dust_q         = dust_q,
        .mode_cmd_q     = mode_cmd_q,
        .system_state   = &system_state,
        .threshold_high = cfg.threshold_high,
        .threshold_low  = cfg.threshold_low,
        .initial_mode   = cfg.mode,
    }};

    LedRing leds{{
        .gpio_data = 18,
        .num_leds  = 8,
        .state     = &system_state,
    }};

    // GPIO 8 (SDA) and 9 (SCL) freed up after OLED was replaced by LED ring
    PressureSensor pressure{{
        .gpio_sda         = 8,
        .gpio_scl         = 9,
        .i2c_addr         = 0x77,
        .system_state     = &system_state,
        .clog_threshold_pa = cfg.clog_threshold_pa,
    }};

    // Start all tasks
    dust.start();
    rf.start();
    fan.start();
    leds.start();
    pressure.start();

    // Register app_main with the task watchdog
    ESP_ERROR_CHECK(esp_task_wdt_add(nullptr));

    while (true) {
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
