#pragma once

#include <cstdint>

#include "driver/i2c_master.h"

#include "task.hpp"    // espp::Task
#include "logger.hpp"  // espp::Logger

#include "led_ring.hpp"    // SystemState
#include "nvs_config.hpp"  // (for FanMode via led_ring transitive include)

class PressureSensor {
public:
    struct Config {
        int          gpio_sda;
        int          gpio_scl;
        uint8_t      i2c_addr         = 0x77;  // BMP280/BMP180 default; 0x76 if SDO low
        SystemState *system_state;
        float        clog_threshold_pa;         // from NVS; Pa drop = clogged
    };

    explicit PressureSensor(Config cfg);
    void start();

private:
    // ── Sensor type ───────────────────────────────────────────────────────────
    enum class SensorType { None, BMP280, BMP180 };

    // ── Calibration state machine ─────────────────────────────────────────────
    // Calibration waits for the fan to first turn on, then settles 5 s before
    // averaging kCalibReadings samples as the clean-filter baseline.
    enum class CalState { WaitForFan, Settling, Calibrating, Monitoring };

    bool task_fn(std::mutex &, std::condition_variable &);

    // Detect which sensor is present at the I2C address (or its complement)
    // and initialise it. Returns SensorType::None on failure.
    SensorType detect_and_init();

    // Dispatch to the appropriate compensation path
    float read_pressure_pa();

    // ── BMP280 ────────────────────────────────────────────────────────────────
    struct Bmp280Cal {
        uint16_t T1; int16_t T2, T3;
        uint16_t P1; int16_t P2, P3, P4, P5, P6, P7, P8, P9;
    };
    bool  init_bmp280();
    float read_bmp280_pa();

    // ── BMP180 ────────────────────────────────────────────────────────────────
    struct Bmp180Cal {
        int16_t  AC1, AC2, AC3;
        uint16_t AC4, AC5, AC6;
        int16_t  B1, B2, MB, MC, MD;
    };
    bool  init_bmp180();
    float read_bmp180_pa();

    // ── I2C helpers ───────────────────────────────────────────────────────────
    void    write_reg(uint8_t reg, uint8_t val);
    void    read_regs(uint8_t reg, uint8_t *buf, size_t len);
    uint8_t read_u8(uint8_t reg);
    // BMP280 calibration registers are little-endian; BMP180 are big-endian
    uint16_t read_u16_le(uint8_t reg);
    int16_t  read_s16_le(uint8_t reg);
    uint16_t read_u16_be(uint8_t reg);
    int16_t  read_s16_be(uint8_t reg);

    // ── State ─────────────────────────────────────────────────────────────────
    Config                  cfg_;
    espp::Task              task_;
    espp::Logger            logger_{{.tag = "PressureSensor",
                                     .level = espp::Logger::Verbosity::INFO}};

    i2c_master_bus_handle_t bus_{nullptr};
    i2c_master_dev_handle_t dev_{nullptr};

    SensorType type_{SensorType::None};
    Bmp280Cal  bmp280_cal_{};
    Bmp180Cal  bmp180_cal_{};

    CalState cal_state_{CalState::WaitForFan};
    float    baseline_pa_{0.0f};
    uint32_t settle_start_ms_{0};
    float    calib_sum_{0.0f};
    int      calib_count_{0};

    static constexpr uint32_t kSettleMs        = 5'000;
    static constexpr int      kCalibReadings   = 10;
    static constexpr uint32_t kSampleMs        = 1'000;
    // Hysteresis: clear clogged flag when delta drops below threshold × 70 %
    static constexpr float    kClearFactor     = 0.70f;
};
