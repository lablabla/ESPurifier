#include "rf_remote.hpp"

#include <cstring>

// EV1527 protocol constants (base unit T ≈ 300 µs at 1 µs RMT resolution)
static constexpr uint32_t kT_us     = 300; // base unit
static constexpr uint32_t kSlack_us = 150; // ±50 % tolerance
static constexpr uint32_t kSyncLow  = 31 * kT_us; // ~9300 µs

// An EV1527 frame is 1 sync + 24 data symbols = 25 total RMT symbols
static constexpr size_t kEv1527Symbols = 25;

// ─── ISR callback ────────────────────────────────────────────────────────────

bool IRAM_ATTR RfRemote::rmt_rx_done_cb(rmt_channel_handle_t /*channel*/,
                                          const rmt_rx_done_event_data_t *edata,
                                          void *user_ctx)
{
    auto *self = static_cast<RfRemote *>(user_ctx);
    BaseType_t high_task_woken = pdFALSE;
    xQueueSendFromISR(self->rmt_q_, edata, &high_task_woken);
    return high_task_woken == pdTRUE;
}

// ─── Constructor ─────────────────────────────────────────────────────────────

RfRemote::RfRemote(Config cfg)
    : cfg_{cfg}
    , task_{{
          .callback = [this](std::mutex &m, std::condition_variable &cv) {
              return task_fn(m, cv);
          },
          .task_config = {
              .name             = "rf_remote",
              .stack_size_bytes = 4096,
              .priority         = 6,
              .core_id          = 0,
          },
      }}
{
    rmt_q_ = xQueueCreate(4, sizeof(rmt_rx_done_event_data_t));

    // Configure RMT RX channel at 1 µs resolution
    rmt_rx_channel_config_t rx_cfg{
        .gpio_num          = cfg_.gpio_data,
        .clk_src           = RMT_CLK_SRC_DEFAULT,
        .resolution_hz     = 1'000'000, // 1 µs per tick
        .mem_block_symbols = kSymBufLen,
    };
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_cfg, &rmt_chan_));

    rmt_rx_event_callbacks_t cbs{.on_recv_done = rmt_rx_done_cb};
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rmt_chan_, &cbs, this));
    ESP_ERROR_CHECK(rmt_enable(rmt_chan_));

    // Start the first receive — RMT will keep receiving after each done event
    rmt_receive_config_t recv_cfg{
        .signal_range_min_ns =    1'000, // ignore glitches < 1 µs
        .signal_range_max_ns = 15'000'000, // max symbol width ~15 ms (EV1527 sync)
    };
    ESP_ERROR_CHECK(rmt_receive(rmt_chan_, sym_buf_, sizeof(sym_buf_), &recv_cfg));
}

RfRemote::~RfRemote() {
    rmt_disable(rmt_chan_);
    rmt_del_channel(rmt_chan_);
    vQueueDelete(rmt_q_);
}

void RfRemote::start() {
    task_.start();
}

// ─── Task body ───────────────────────────────────────────────────────────────

bool RfRemote::task_fn(std::mutex &, std::condition_variable &) {
    rmt_rx_done_event_data_t edata{};
    if (xQueueReceive(rmt_q_, &edata, pdMS_TO_TICKS(100)) != pdTRUE) {
        return false; // timeout — nothing received, keep looping
    }

    const auto cmd = decode_ev1527(
        std::span<const rmt_symbol_word_t>{edata.received_symbols, edata.num_symbols});

    if (cmd) {
        const RfCommand c = *cmd;
        xQueueSend(cfg_.mode_cmd_q, &c, 0);
        logger_.info("RF command: {}", static_cast<int>(c));
    }

    // Re-arm receive for next frame
    rmt_receive_config_t recv_cfg{
        .signal_range_min_ns =    1'000,
        .signal_range_max_ns = 15'000'000,
    };
    rmt_receive(rmt_chan_, sym_buf_, sizeof(sym_buf_), &recv_cfg);

    return false;
}

// ─── EV1527 decoder ──────────────────────────────────────────────────────────

std::optional<RfCommand> RfRemote::decode_ev1527(
    std::span<const rmt_symbol_word_t> syms)
{
    if (syms.size() < kEv1527Symbols) {
        logger_.debug("Too few symbols: {}", syms.size());
        return std::nullopt;
    }

    // Validate sync symbol: short high (~1T) followed by long low (~31T)
    const auto &sync = syms[0];
    if (sync.duration1 < (kSyncLow - kSlack_us)) {
        logger_.debug("Sync mismatch: low_dur={}", sync.duration1);
        return std::nullopt;
    }

    // Decode 24 data bits: Bit 1 = long high (3T), Bit 0 = short high (1T)
    uint32_t addr{0}, data{0};
    for (size_t i = 1; i <= 24; ++i) {
        // duration0 = high width of the data symbol
        const bool bit = syms[i].duration0 > (2 * kT_us - kSlack_us);
        if (i <= 20) {
            addr = (addr << 1) | static_cast<uint32_t>(bit);
        } else {
            data = (data << 1) | static_cast<uint32_t>(bit);
        }
    }

    logger_.debug("EV1527 addr=0x{:05X} data=0x{:X}", addr, data);

    if (cfg_.paired_addr != 0 && addr != cfg_.paired_addr) {
        logger_.debug("Address mismatch (expected 0x{:05X})", cfg_.paired_addr);
        return std::nullopt;
    }

    switch (data) {
        case 0b0001: return RfCommand::ToggleAutoManual;
        case 0b0010: return RfCommand::ToggleOff;
        default:
            logger_.warn("Unknown EV1527 data nibble: 0b{:04b}", data);
            return std::nullopt;
    }
}
