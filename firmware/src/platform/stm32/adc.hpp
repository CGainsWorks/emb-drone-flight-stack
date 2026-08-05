#pragma once

#include <cstdint>

namespace fc {
namespace stm32 {

// Round-robin ADC sampling of the three analogue sense inputs.
//
//   /ADC_VBAT  PC1  ADC123_IN11
//   /ADC_RSSI  PC2  ADC123_IN12
//   /ADC_CURR  PC3  ADC123_IN13
//
// Non-blocking by design: poll() starts one conversion and collects the
// previous one, so the control loop never waits on the ADC. Each channel is
// refreshed every third cycle, which at 1 kHz is ~333 Hz per channel - far
// faster than the battery voltage can meaningfully change.
class Stm32Adc {
public:
    void begin();

    // Call once per control cycle.
    void poll();

    uint16_t vbat_counts() const { return counts_[0]; }
    uint16_t rssi_counts() const { return counts_[1]; }
    uint16_t current_counts() const { return counts_[2]; }

    bool has_sampled_all() const { return sampled_all_; }

private:
    static constexpr uint8_t kChannelCount = 3;
    // ADC channel numbers, in the order sampled.
    static constexpr uint8_t kChannels[kChannelCount] = {11, 12, 13};

    void start_conversion(uint8_t channel);

    uint16_t counts_[kChannelCount]{};
    uint8_t index_{0};
    bool conversion_active_{false};
    bool sampled_all_{false};
    uint8_t completed_{0};
};

}  // namespace stm32
}  // namespace fc
