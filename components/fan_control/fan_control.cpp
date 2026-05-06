#include "fan_control.hpp"

using namespace std::chrono_literals;

FanControl::FanControl(Config cfg)
    : cfg_{cfg}
    , mode_{cfg.initial_mode}
    , task_{{
          .callback = [this](std::mutex &m, std::condition_variable &cv) {
              return task_fn(m, cv);
          },
          .task_config = {
              .name             = "fan_control",
              .stack_size_bytes = 4096,
              .priority         = 4,
              .core_id          = 0,
          },
      }}
{
    gpio_config_t io_cfg{
        .pin_bit_mask = 1ULL << cfg_.gpio_ssr,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_cfg);
    set_fan(false);
}

void FanControl::start() {
    task_.start();
}

bool FanControl::task_fn(std::mutex &, std::condition_variable &) {
    // Drain any pending mode commands first (non-blocking)
    RfCommand cmd{};
    while (xQueueReceive(cfg_.mode_cmd_q, &cmd, 0) == pdTRUE) {
        handle_mode_command(cmd);
    }

    // Check for MANUAL_HIGH timeout
    if (mode_ == FanMode::ManualHigh && Clock::now() > manual_high_until_) {
        logger_.info("ManualHigh expired, reverting to Auto");
        mode_ = FanMode::Auto;
        cfg_.display_state->mode.store(mode_);
    }

    // Consume latest dust reading (block up to 200 ms so we don't spin-loop)
    float dust{0.0f};
    if (xQueueReceive(cfg_.dust_q, &dust, pdMS_TO_TICKS(200)) == pdTRUE) {
        cfg_.display_state->dust_mgm3.store(dust);
        if (mode_ == FanMode::Auto) {
            apply_auto_logic(dust);
        }
    }

    // In ManualHigh, keep fan running regardless of dust level
    if (mode_ == FanMode::ManualHigh && !fan_on_) {
        set_fan(true);
    }

    return false; // keep running
}

void FanControl::apply_auto_logic(float dust) {
    if (!fan_on_ && dust > cfg_.threshold_high) {
        set_fan(true);
        fan_on_since_ = Clock::now();
        logger_.info("Fan ON (dust={:.3f} > {:.3f})", dust, cfg_.threshold_high);
    } else if (fan_on_ && dust < cfg_.threshold_low) {
        const auto elapsed = Clock::now() - fan_on_since_;
        if (elapsed >= kMinRunTime) {
            set_fan(false);
            logger_.info("Fan OFF (dust={:.3f} < {:.3f})", dust, cfg_.threshold_low);
        }
        // else: minimum run time not yet elapsed, keep fan running
    }
}

void FanControl::handle_mode_command(RfCommand cmd) {
    switch (cmd) {
        case RfCommand::ToggleAutoManual:
            if (mode_ == FanMode::Auto) {
                mode_ = FanMode::ManualHigh;
                manual_high_until_ = Clock::now() + kManualHighDur;
                set_fan(true);
                logger_.info("Mode → ManualHigh (30 min)");
            } else if (mode_ == FanMode::ManualHigh) {
                mode_ = FanMode::Auto;
                logger_.info("Mode → Auto");
            }
            break;

        case RfCommand::ToggleOff:
            if (mode_ == FanMode::Off) {
                mode_ = FanMode::Auto;
                logger_.info("Mode → Auto (was Off)");
            } else {
                mode_ = FanMode::Off;
                set_fan(false);
                logger_.info("Mode → Off");
            }
            break;
    }

    cfg_.display_state->mode.store(mode_);
}

void FanControl::set_fan(bool on) {
    fan_on_ = on;
    gpio_set_level(static_cast<gpio_num_t>(cfg_.gpio_ssr), on ? 1 : 0);
    cfg_.display_state->fan_on.store(on);
}
