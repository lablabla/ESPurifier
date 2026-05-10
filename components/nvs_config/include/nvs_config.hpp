#pragma once

#include <cstdint>

#include "nvs.hpp"    // espp::Nvs
#include "logger.hpp" // espp::Logger

// Forward-declared here so nvs_config is not coupled to fan_control headers.
// fan_control.hpp includes nvs_config.hpp and re-declares FanMode consistently.
enum class FanMode : uint8_t { Off = 0, Auto = 1, ManualHigh = 2 };

struct AppConfig {
    FanMode mode              = FanMode::Auto;
    float   threshold_high    = 50.0f;   // µg/m³ — fan turns ON above this
    float   threshold_low     = 20.0f;   // µg/m³ — fan turns OFF below this
    float   clog_threshold_pa = 100.0f;  // Pa deviation from baseline → filter clogged
};

class NvsConfig {
public:
    NvsConfig();

    AppConfig load();
    void save_mode(FanMode m);
    void save_thresholds(float hi, float lo);
    void save_clog_threshold(float pa);

private:
    static constexpr const char *kNs     = "espurifier";
    static constexpr const char *kMode   = "mode";
    static constexpr const char *kThrHi  = "thr_hi";
    static constexpr const char *kThrLo  = "thr_lo";
    static constexpr const char *kClogPa = "clog_pa";

    espp::Nvs    nvs_;
    espp::Logger logger_{{.tag = "NvsConfig", .level = espp::Logger::Verbosity::INFO}};
};
