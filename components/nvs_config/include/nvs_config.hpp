#pragma once

#include <cstdint>

#include "nvs.hpp"    // espp::Nvs
#include "logger.hpp" // espp::Logger

// Forward-declared here so nvs_config is not coupled to fan_control headers.
// fan_control.hpp includes nvs_config.hpp and re-declares FanMode consistently.
enum class FanMode : uint8_t { Off = 0, Auto = 1, ManualHigh = 2 };

struct AppConfig {
    FanMode mode           = FanMode::Auto;
    float   threshold_high = 0.05f;  // mg/m³ — fan turns ON above this
    float   threshold_low  = 0.02f;  // mg/m³ — fan turns OFF below this
};

class NvsConfig {
public:
    NvsConfig();

    AppConfig load();
    void save_mode(FanMode m);
    void save_thresholds(float hi, float lo);

private:
    static constexpr const char *kNs    = "espurifier";
    static constexpr const char *kMode  = "mode";
    static constexpr const char *kThrHi = "thr_hi";
    static constexpr const char *kThrLo = "thr_lo";

    espp::Nvs    nvs_;
    espp::Logger logger_{{.tag = "NvsConfig", .level = espp::Logger::Verbosity::INFO}};
};
