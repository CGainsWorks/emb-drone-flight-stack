#include "fc/sensors/battery.hpp"

#include <cmath>

namespace fc {

void BatteryMonitor::set_config(const BatteryConfig& cfg) {
    cfg_ = cfg;
    voltage_filter_.set_cutoff(cfg_.voltage_filter_hz);
    current_filter_.set_cutoff(cfg_.current_filter_hz);
}

void BatteryMonitor::reset() {
    voltage_filter_.reset();
    current_filter_.reset();
    voltage_v_ = 0.0f;
    current_a_ = 0.0f;
    used_mah_ = 0.0f;
    cells_ = 0;
    have_sample_ = false;
}

float BatteryMonitor::counts_to_pack_volts(uint16_t counts) const {
    if (cfg_.adc_max_counts == 0) {
        return 0.0f;
    }
    const float pin_v = (static_cast<float>(counts) / static_cast<float>(cfg_.adc_max_counts)) *
                        cfg_.adc_reference_v;
    return pin_v * cfg_.voltage_divider_ratio;
}

void BatteryMonitor::update(uint16_t voltage_counts, uint16_t current_counts, float dt) {
    const float raw_v = counts_to_pack_volts(voltage_counts);

    if (dt > 0.0f) {
        voltage_v_ = voltage_filter_.update(raw_v, dt);
    } else {
        voltage_v_ = raw_v;
    }

    const float pin_v =
        (cfg_.adc_max_counts == 0)
            ? 0.0f
            : (static_cast<float>(current_counts) / static_cast<float>(cfg_.adc_max_counts)) *
                  cfg_.adc_reference_v;
    float raw_a = (pin_v - cfg_.current_offset_v) * cfg_.current_amps_per_volt;
    if (raw_a < 0.0f) {
        // A small negative reading is sensor offset, not regenerative current.
        raw_a = 0.0f;
    }

    if (dt > 0.0f) {
        current_a_ = current_filter_.update(raw_a, dt);
        // Integrate consumed capacity: amps * seconds -> mAh.
        used_mah_ += current_a_ * dt * (1000.0f / 3600.0f);
    } else {
        current_a_ = raw_a;
    }

    have_sample_ = true;
}

void BatteryMonitor::detect_cell_count() {
    if (!have_sample_ || voltage_v_ <= 0.0f) {
        return;
    }
    // Smallest cell count for which the per-cell voltage is plausible.
    for (uint8_t n = 1; n <= 12; ++n) {
        if (voltage_v_ / static_cast<float>(n) <= cfg_.cell_detect_max_v) {
            cells_ = n;
            return;
        }
    }
    cells_ = 12;
}

float BatteryMonitor::cell_voltage() const {
    if (cells_ == 0) {
        return 0.0f;
    }
    return voltage_v_ / static_cast<float>(cells_);
}

uint8_t BatteryMonitor::remaining_percent() const {
    if (cells_ == 0) {
        return 0;
    }
    const float span = cfg_.cell_full_v - cfg_.cell_empty_v;
    if (span <= 0.0f) {
        return 0;
    }
    const float fraction = (cell_voltage() - cfg_.cell_empty_v) / span;
    if (fraction <= 0.0f) {
        return 0;
    }
    if (fraction >= 1.0f) {
        return 100;
    }
    return static_cast<uint8_t>(fraction * 100.0f + 0.5f);
}

}  // namespace fc
