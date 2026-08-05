#pragma once

#include <cstdint>

#include "fc/math/filter.hpp"

namespace fc {

struct BatteryConfig {
    // Resistor divider on /ADC_VBAT: volts at the pack per volt at the pin.
    // Must be measured on the assembled board, not assumed from the schematic
    // values, because divider tolerance dominates the error here.
    float voltage_divider_ratio{11.0f};

    // ADC reference and resolution.
    float adc_reference_v{3.3f};
    uint16_t adc_max_counts{4095};

    // Current sense on /ADC_CURR: amps per volt at the pin, and the zero-current
    // offset voltage.
    float current_amps_per_volt{50.0f};
    float current_offset_v{0.0f};

    // Nominal per-cell thresholds for a LiPo.
    float cell_full_v{4.20f};
    float cell_empty_v{3.30f};
    // Above this the pack is assumed to have one more cell.
    float cell_detect_max_v{4.35f};

    float voltage_filter_hz{2.0f};
    float current_filter_hz{10.0f};
};

// Pack voltage, current and consumed capacity.
//
// Cell count is detected once, at power-on, from the first stable reading. It
// is deliberately never re-detected: sag under load would otherwise make a 4S
// pack look like a 3S mid-flight and move every voltage threshold under the
// failsafe's feet.
class BatteryMonitor {
public:
    BatteryMonitor() = default;
    explicit BatteryMonitor(const BatteryConfig& cfg) { set_config(cfg); }

    void set_config(const BatteryConfig& cfg);
    const BatteryConfig& config() const { return cfg_; }

    // Feed raw ADC counts. dt in seconds.
    void update(uint16_t voltage_counts, uint16_t current_counts, float dt);

    // Establish the cell count from the current reading. Call once at startup
    // with the pack connected and the motors stopped.
    void detect_cell_count();

    void reset();

    float voltage() const { return voltage_v_; }
    float current() const { return current_a_; }
    float used_mah() const { return used_mah_; }
    uint8_t cell_count() const { return cells_; }
    bool cells_detected() const { return cells_ > 0; }

    // Volts per cell, or 0 if the cell count is not known yet.
    float cell_voltage() const;

    // 0..100. Linear between the empty and full per-cell thresholds; a crude
    // estimate, but honest about being one.
    uint8_t remaining_percent() const;

    // Convert raw counts to volts at the pack, without filtering.
    float counts_to_pack_volts(uint16_t counts) const;

private:
    BatteryConfig cfg_{};
    LowPassFilter voltage_filter_{};
    LowPassFilter current_filter_{};

    float voltage_v_{0.0f};
    float current_a_{0.0f};
    float used_mah_{0.0f};
    uint8_t cells_{0};
    bool have_sample_{false};
};

}  // namespace fc
