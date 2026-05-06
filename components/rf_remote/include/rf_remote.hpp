#pragma once

#include <cstdint>
#include <optional>
#include <span>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/rmt_rx.h"

#include "task.hpp"   // espp::Task
#include "logger.hpp" // espp::Logger

enum class RfCommand : uint8_t {
    ToggleAutoManual = 0,
    ToggleOff        = 1,
};

class RfRemote {
public:
    struct Config {
        int           gpio_data;   // GPIO 7
        QueueHandle_t mode_cmd_q;  // sends RfCommand to FanControl
        uint32_t      paired_addr; // 20-bit EV1527 address; 0 = accept any
    };

    explicit RfRemote(Config cfg);
    ~RfRemote();
    void start();

private:
    bool task_fn(std::mutex &m, std::condition_variable &cv);
    std::optional<RfCommand> decode_ev1527(std::span<const rmt_symbol_word_t> syms);

    // RMT receive-done callback — called from ISR context
    static bool IRAM_ATTR rmt_rx_done_cb(rmt_channel_handle_t channel,
                                          const rmt_rx_done_event_data_t *edata,
                                          void *user_ctx);

    Config        cfg_;
    espp::Task    task_;
    espp::Logger  logger_{{.tag = "RfRemote", .level = espp::Logger::Verbosity::INFO}};

    rmt_channel_handle_t rmt_chan_{nullptr};

    // Internal queue fed by RMT ISR, drained by task_fn
    QueueHandle_t rmt_q_{nullptr};

    // Symbol receive buffer — sized for one full EV1527 frame with margin
    static constexpr size_t kSymBufLen = 64;
    rmt_symbol_word_t sym_buf_[kSymBufLen]{};
};
