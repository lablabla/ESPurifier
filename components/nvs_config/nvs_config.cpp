#include "nvs_config.hpp"

#include <bit>       // std::bit_cast
#include <system_error>

NvsConfig::NvsConfig() = default;

AppConfig NvsConfig::load() {
    AppConfig cfg;
    std::error_code ec;

    uint8_t  mode_raw  = static_cast<uint8_t>(FanMode::Auto);
    uint32_t hi_bits   = std::bit_cast<uint32_t>(AppConfig{}.threshold_high);
    uint32_t lo_bits   = std::bit_cast<uint32_t>(AppConfig{}.threshold_low);
    uint32_t clog_bits = std::bit_cast<uint32_t>(AppConfig{}.clog_threshold_pa);

    nvs_.get_or_set_var(kNs, kMode,   mode_raw,  mode_raw,  ec);
    nvs_.get_or_set_var(kNs, kThrHi,  hi_bits,   hi_bits,   ec);
    nvs_.get_or_set_var(kNs, kThrLo,  lo_bits,   lo_bits,   ec);
    nvs_.get_or_set_var(kNs, kClogPa, clog_bits, clog_bits, ec);

    if (ec) {
        logger_.warn("NVS load error: {}, using defaults", ec.message().c_str());
    }

    cfg.mode              = static_cast<FanMode>(mode_raw);
    cfg.threshold_high    = std::bit_cast<float>(hi_bits);
    cfg.threshold_low     = std::bit_cast<float>(lo_bits);
    cfg.clog_threshold_pa = std::bit_cast<float>(clog_bits);

    logger_.info("Loaded: mode={} thr_hi={:.1f} thr_lo={:.1f} clog_pa={:.1f}",
                 static_cast<int>(cfg.mode), cfg.threshold_high,
                 cfg.threshold_low, cfg.clog_threshold_pa);
    return cfg;
}

void NvsConfig::save_mode(FanMode m) {
    std::error_code ec;
    nvs_.set_var(kNs, kMode, static_cast<uint8_t>(m), ec);
    if (ec) logger_.warn("NVS save_mode error: {}", ec.message().c_str());
}

void NvsConfig::save_thresholds(float hi, float lo) {
    std::error_code ec;
    nvs_.set_var(kNs, kThrHi, std::bit_cast<uint32_t>(hi), ec);
    nvs_.set_var(kNs, kThrLo, std::bit_cast<uint32_t>(lo), ec);
    if (ec) logger_.warn("NVS save_thresholds error: {}", ec.message().c_str());
}

void NvsConfig::save_clog_threshold(float pa) {
    std::error_code ec;
    nvs_.set_var(kNs, kClogPa, std::bit_cast<uint32_t>(pa), ec);
    if (ec) logger_.warn("NVS save_clog_threshold error: {}", ec.message().c_str());
}
