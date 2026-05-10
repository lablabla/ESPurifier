#include "pressure_sensor.hpp"

#include <cmath>     // fabsf
#include <cstring>   // memset

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ── BMP280 register map ───────────────────────────────────────────────────────
static constexpr uint8_t kBmp280RegId     = 0xD0;
static constexpr uint8_t kBmp280IdVal     = 0x58;
static constexpr uint8_t kBme280IdVal     = 0x60;
static constexpr uint8_t kBmp280RegReset  = 0xE0;
static constexpr uint8_t kBmp280RegCtrl   = 0xF4;
static constexpr uint8_t kBmp280RegCfg    = 0xF5;
static constexpr uint8_t kBmp280RegPress  = 0xF7;  // burst-reads 6 bytes (P+T)
static constexpr uint8_t kBmp280CalBase   = 0x88;  // T1..P9, 24 bytes, little-endian

// ── BMP180 register map ───────────────────────────────────────────────────────
static constexpr uint8_t kBmp180RegId     = 0xD0;
static constexpr uint8_t kBmp180IdVal     = 0x55;
static constexpr uint8_t kBmp180RegCtrl   = 0xF4;
static constexpr uint8_t kBmp180RegOut    = 0xF6;
static constexpr uint8_t kBmp180CalBase   = 0xAA;  // AC1..MD, 22 bytes, big-endian
static constexpr uint8_t kBmp180CmdTemp   = 0x2E;
static constexpr uint8_t kBmp180CmdPres   = 0x34;  // OSS = 0
static constexpr uint32_t kBmp180TempMs   = 5;
static constexpr uint32_t kBmp180PresMs   = 5;     // OSS=0 max 4.5 ms → round up

// ─────────────────────────────────────────────────────────────────────────────

PressureSensor::PressureSensor(Config cfg)
    : cfg_{cfg}
    , task_{{
          .callback = [this](std::mutex &m, std::condition_variable &cv) {
              return task_fn(m, cv);
          },
          .task_config = {
              .name             = "pressure_sensor",
              .stack_size_bytes = 4096,
              .priority         = 3,
              .core_id          = 0,
          },
      }}
{
    i2c_master_bus_config_t bus_cfg{
        .i2c_port          = I2C_NUM_0,
        .sda_io_num        = static_cast<gpio_num_t>(cfg_.gpio_sda),
        .scl_io_num        = static_cast<gpio_num_t>(cfg_.gpio_scl),
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags             = { .enable_internal_pullup = true },
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_));

    i2c_device_config_t dev_cfg{
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = cfg_.i2c_addr,
        .scl_speed_hz    = 400'000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_, &dev_cfg, &dev_));

    type_ = detect_and_init();
    if (type_ == SensorType::None) {
        logger_.warn("No BMP280/BMP180 found at I2C addr 0x{:02X}", cfg_.i2c_addr);
    } else {
        logger_.info("{} ready at 0x{:02X}",
                     type_ == SensorType::BMP280 ? "BMP280" : "BMP180",
                     cfg_.i2c_addr);
    }
}

void PressureSensor::start() {
    task_.start();
}

// ─── Task loop ────────────────────────────────────────────────────────────────

bool PressureSensor::task_fn(std::mutex &, std::condition_variable &) {
    if (type_ == SensorType::None) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        return false;
    }

    const uint32_t now_ms =
        static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);

    switch (cal_state_) {
        case CalState::WaitForFan:
            if (cfg_.system_state->fan_on.load()) {
                cal_state_       = CalState::Settling;
                settle_start_ms_ = now_ms;
                logger_.info("Fan ON — settling {}s before pressure baseline",
                             kSettleMs / 1000u);
            }
            vTaskDelay(pdMS_TO_TICKS(kSampleMs));
            return false;

        case CalState::Settling:
            if (now_ms - settle_start_ms_ >= kSettleMs) {
                cal_state_   = CalState::Calibrating;
                calib_sum_   = 0.0f;
                calib_count_ = 0;
                logger_.info("Calibrating pressure baseline ({} samples)", kCalibReadings);
            }
            vTaskDelay(pdMS_TO_TICKS(kSampleMs));
            return false;

        case CalState::Calibrating: {
            const float pa = read_pressure_pa();
            if (pa > 0.0f) {
                calib_sum_ += pa;
                ++calib_count_;
            }
            if (calib_count_ >= kCalibReadings) {
                baseline_pa_ = calib_sum_ / static_cast<float>(calib_count_);
                cal_state_   = CalState::Monitoring;
                logger_.info("Pressure baseline: {:.1f} Pa", baseline_pa_);
            }
            vTaskDelay(pdMS_TO_TICKS(kSampleMs));
            return false;
        }

        case CalState::Monitoring: {
            if (!cfg_.system_state->fan_on.load()) {
                // Fan turned off — reset so we re-calibrate on next fan start
                cal_state_ = CalState::WaitForFan;
                cfg_.system_state->clog_detected.store(false);
                logger_.info("Fan OFF — pressure calibration reset");
                vTaskDelay(pdMS_TO_TICKS(kSampleMs));
                return false;
            }

            const float pa    = read_pressure_pa();
            const float delta = fabsf(pa - baseline_pa_);

            const bool clogged = cfg_.system_state->clog_detected.load();
            if (!clogged && delta > cfg_.clog_threshold_pa) {
                cfg_.system_state->clog_detected.store(true);
                logger_.warn("Filter CLOGGED — Δ={:.1f} Pa > threshold {:.1f} Pa",
                             delta, cfg_.clog_threshold_pa);
            } else if (clogged && delta < cfg_.clog_threshold_pa * kClearFactor) {
                cfg_.system_state->clog_detected.store(false);
                logger_.info("Filter clear — Δ={:.1f} Pa", delta);
            }

            vTaskDelay(pdMS_TO_TICKS(kSampleMs));
            return false;
        }
    }
    return false;
}

// ─── Sensor detection ─────────────────────────────────────────────────────────

PressureSensor::SensorType PressureSensor::detect_and_init() {
    const uint8_t id = read_u8(kBmp280RegId);
    if (id == kBmp280IdVal || id == kBme280IdVal) {
        logger_.info("Detected BMP280/BME280 (chip_id=0x{:02X})", id);
        return init_bmp280() ? SensorType::BMP280 : SensorType::None;
    }
    if (id == kBmp180IdVal) {
        logger_.info("Detected BMP180 (chip_id=0x{:02X})", id);
        return init_bmp180() ? SensorType::BMP180 : SensorType::None;
    }
    logger_.warn("Unknown chip ID 0x{:02X} — sensor absent or wrong address", id);
    return SensorType::None;
}

float PressureSensor::read_pressure_pa() {
    switch (type_) {
        case SensorType::BMP280: return read_bmp280_pa();
        case SensorType::BMP180: return read_bmp180_pa();
        default:                  return 0.0f;
    }
}

// ─── BMP280 ───────────────────────────────────────────────────────────────────

bool PressureSensor::init_bmp280() {
    write_reg(kBmp280RegReset, 0xB6);  // soft reset
    vTaskDelay(pdMS_TO_TICKS(5));

    // Read 24-byte calibration block (T1..P9, all little-endian)
    uint8_t buf[24]{};
    read_regs(kBmp280CalBase, buf, sizeof(buf));

    auto u16 = [&](int i) -> uint16_t {
        return static_cast<uint16_t>(buf[i] | (static_cast<uint16_t>(buf[i + 1]) << 8));
    };
    auto s16 = [&](int i) -> int16_t { return static_cast<int16_t>(u16(i)); };

    bmp280_cal_ = {
        .T1 = u16(0),  .T2 = s16(2),  .T3 = s16(4),
        .P1 = u16(6),  .P2 = s16(8),  .P3 = s16(10),
        .P4 = s16(12), .P5 = s16(14), .P6 = s16(16),
        .P7 = s16(18), .P8 = s16(20), .P9 = s16(22),
    };

    // Normal mode: temp OSS×1, press OSS×4, IIR filter off, standby 500 ms
    // ctrl_meas: osrs_t=001, osrs_p=011, mode=11  → 0b001_011_11 = 0x2F
    write_reg(kBmp280RegCtrl, 0x2F);
    write_reg(kBmp280RegCfg,  0x80);  // t_sb=500ms, filter=off, spi3w=off

    return true;
}

float PressureSensor::read_bmp280_pa() {
    uint8_t raw[6]{};
    read_regs(kBmp280RegPress, raw, sizeof(raw));

    const int32_t adc_P = (static_cast<int32_t>(raw[0]) << 12)
                        | (static_cast<int32_t>(raw[1]) <<  4)
                        | (static_cast<int32_t>(raw[2]) >>  4);
    const int32_t adc_T = (static_cast<int32_t>(raw[3]) << 12)
                        | (static_cast<int32_t>(raw[4]) <<  4)
                        | (static_cast<int32_t>(raw[5]) >>  4);

    // Bosch temperature compensation → t_fine
    const int32_t var1t =
        ((((adc_T >> 3) - (static_cast<int32_t>(bmp280_cal_.T1) << 1)))
        * static_cast<int32_t>(bmp280_cal_.T2)) >> 11;
    const int32_t var2t =
        (((((adc_T >> 4) - static_cast<int32_t>(bmp280_cal_.T1))
        *  ((adc_T >> 4) - static_cast<int32_t>(bmp280_cal_.T1))) >> 12)
        * static_cast<int32_t>(bmp280_cal_.T3)) >> 14;
    const int32_t t_fine = var1t + var2t;

    // Bosch pressure compensation (64-bit fixed-point, datasheet 4.2.3)
    int64_t v1 = static_cast<int64_t>(t_fine) - 128000LL;
    int64_t v2 = v1 * v1 * static_cast<int64_t>(bmp280_cal_.P6);
    v2 += (v1 * static_cast<int64_t>(bmp280_cal_.P5)) << 17;
    v2 += static_cast<int64_t>(bmp280_cal_.P4) << 35;
    v1  = ((v1 * v1 * static_cast<int64_t>(bmp280_cal_.P3)) >> 8)
        + ((v1 * static_cast<int64_t>(bmp280_cal_.P2)) << 12);
    v1  = ((static_cast<int64_t>(1) << 47) + v1)
        * static_cast<int64_t>(bmp280_cal_.P1) >> 33;
    if (v1 == 0LL) return 0.0f;  // avoid division by zero

    int64_t p = 1048576LL - static_cast<int64_t>(adc_P);
    p = (((p << 31) - v2) * 3125LL) / v1;
    v1 = (static_cast<int64_t>(bmp280_cal_.P9) * (p >> 13) * (p >> 13)) >> 25;
    v2 = (static_cast<int64_t>(bmp280_cal_.P8) * p) >> 19;
    p  = ((p + v1 + v2) >> 8) + (static_cast<int64_t>(bmp280_cal_.P7) << 4);

    // p is in Q24.8 Pa — divide by 256 to get Pa
    return static_cast<float>(static_cast<uint32_t>(p)) / 256.0f;
}

// ─── BMP180 ───────────────────────────────────────────────────────────────────

bool PressureSensor::init_bmp180() {
    // Read 22-byte calibration block (AC1..MD, big-endian)
    uint8_t buf[22]{};
    read_regs(kBmp180CalBase, buf, sizeof(buf));

    auto s16 = [&](int i) -> int16_t {
        return static_cast<int16_t>((static_cast<uint16_t>(buf[i]) << 8) | buf[i + 1]);
    };
    auto u16 = [&](int i) -> uint16_t {
        return static_cast<uint16_t>((static_cast<uint16_t>(buf[i]) << 8) | buf[i + 1]);
    };

    bmp180_cal_ = {
        .AC1 = s16(0),  .AC2 = s16(2),  .AC3 = s16(4),
        .AC4 = u16(6),  .AC5 = u16(8),  .AC6 = u16(10),
        .B1  = s16(12), .B2  = s16(14),
        .MB  = s16(16), .MC  = s16(18), .MD  = s16(20),
    };

    return true;
}

float PressureSensor::read_bmp180_pa() {
    // Step 1: read uncompensated temperature
    write_reg(kBmp180RegCtrl, kBmp180CmdTemp);
    vTaskDelay(pdMS_TO_TICKS(kBmp180TempMs));
    const int32_t UT = static_cast<int32_t>(read_u16_be(kBmp180RegOut));

    // Step 2: read uncompensated pressure (OSS=0)
    write_reg(kBmp180RegCtrl, kBmp180CmdPres);
    vTaskDelay(pdMS_TO_TICKS(kBmp180PresMs));
    uint8_t raw[3]{};
    read_regs(kBmp180RegOut, raw, 3);
    const int32_t UP = ((static_cast<int32_t>(raw[0]) << 16)
                      | (static_cast<int32_t>(raw[1]) <<  8)
                      |  static_cast<int32_t>(raw[2])) >> 8;  // OSS=0 → shift by 8-OSS=8

    // Step 3: Bosch BMP180 compensation (datasheet 4.2.3)
    int32_t X1 = ((UT - static_cast<int32_t>(bmp180_cal_.AC6))
                * static_cast<int32_t>(bmp180_cal_.AC5)) >> 15;
    int32_t X2 = (static_cast<int32_t>(bmp180_cal_.MC) << 11)
               / (X1 + static_cast<int32_t>(bmp180_cal_.MD));
    const int32_t B5 = X1 + X2;

    const int32_t B6  = B5 - 4000;
    X1 = (static_cast<int32_t>(bmp180_cal_.B2) * ((B6 * B6) >> 12)) >> 11;
    X2 = (static_cast<int32_t>(bmp180_cal_.AC2) * B6) >> 11;
    int32_t X3  = X1 + X2;
    const int32_t  B3 = ((static_cast<int32_t>(bmp180_cal_.AC1) * 4 + X3) + 2) >> 2;
    X1 = (static_cast<int32_t>(bmp180_cal_.AC3) * B6) >> 13;
    X2 = (static_cast<int32_t>(bmp180_cal_.B1) * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    const uint32_t B4 = (static_cast<uint32_t>(bmp180_cal_.AC4)
                       * static_cast<uint32_t>(X3 + 32768)) >> 15;
    const uint32_t B7 = static_cast<uint32_t>(UP - B3) * 50000UL;

    int32_t p;
    if (B7 < 0x80000000UL) {
        p = static_cast<int32_t>((B7 << 1) / B4);
    } else {
        p = static_cast<int32_t>((B7 / B4) << 1);
    }
    X1 = (p >> 8) * (p >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * p) >> 16;
    p  = p + ((X1 + X2 + 3791) >> 4);

    return static_cast<float>(p);
}

// ─── I2C helpers ─────────────────────────────────────────────────────────────

void PressureSensor::write_reg(uint8_t reg, uint8_t val) {
    const uint8_t buf[2] = {reg, val};
    i2c_master_transmit(dev_, buf, 2, /*timeout_ms=*/100);
}

void PressureSensor::read_regs(uint8_t reg, uint8_t *buf, size_t len) {
    const esp_err_t err =
        i2c_master_transmit_receive(dev_, &reg, 1, buf, len, /*timeout_ms=*/100);
    if (err != ESP_OK) {
        memset(buf, 0, len);
    }
}

uint8_t PressureSensor::read_u8(uint8_t reg) {
    uint8_t val{0};
    read_regs(reg, &val, 1);
    return val;
}

uint16_t PressureSensor::read_u16_le(uint8_t reg) {
    uint8_t b[2]{};
    read_regs(reg, b, 2);
    return static_cast<uint16_t>(b[0] | (static_cast<uint16_t>(b[1]) << 8));
}

int16_t PressureSensor::read_s16_le(uint8_t reg) {
    return static_cast<int16_t>(read_u16_le(reg));
}

uint16_t PressureSensor::read_u16_be(uint8_t reg) {
    uint8_t b[2]{};
    read_regs(reg, b, 2);
    return static_cast<uint16_t>((static_cast<uint16_t>(b[0]) << 8) | b[1]);
}

int16_t PressureSensor::read_s16_be(uint8_t reg) {
    return static_cast<int16_t>(read_u16_be(reg));
}
