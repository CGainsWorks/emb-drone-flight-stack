#pragma once

#include <cstddef>
#include <cstdint>

#include "fc/hal/uart.hpp"
#include "fc/state/arming.hpp"

namespace fc {

// What the transmitter reports. Filled from the flight controller each cycle.
struct TelemetrySnapshot {
    float roll_rad{0.0f};
    float pitch_rad{0.0f};
    float yaw_rad{0.0f};
    float battery_voltage{0.0f};
    float battery_current{0.0f};
    uint32_t used_mah{0};
    uint8_t battery_remaining_pct{0};
    ArmState arm_state{ArmState::Disarmed};
};

struct TelemetryConfig {
    uint32_t attitude_period_us{100000};  // 10 Hz
    uint32_t battery_period_us{500000};   // 2 Hz
};

// Periodic CRSF telemetry output.
//
// Two properties matter here and are both tested:
//
//  * It never blocks. The control loop calls update() every cycle, and a full
//    UART transmit buffer must never stall it. Partial writes are resumed on
//    later calls.
//  * It never emits a torn frame. A frame is queued whole and drained in order,
//    so a receiver never sees interleaved or truncated telemetry even when the
//    port is congested.
class TelemetryTransmitter {
public:
    explicit TelemetryTransmitter(Uart& uart) : uart_(uart) {}

    void set_config(const TelemetryConfig& cfg) { cfg_ = cfg; }
    const TelemetryConfig& config() const { return cfg_; }

    // Call once per control cycle.
    void update(uint32_t now_us, const TelemetrySnapshot& snapshot);

    void reset();

    uint32_t frames_sent() const { return frames_sent_; }
    uint32_t frames_dropped() const { return frames_dropped_; }
    bool busy() const { return pending_len_ > pending_pos_; }

private:
    static constexpr std::size_t kBufferSize = 64;

    Uart& uart_;
    TelemetryConfig cfg_{};

    uint8_t pending_[kBufferSize]{};
    std::size_t pending_len_{0};
    std::size_t pending_pos_{0};

    bool have_sent_attitude_{false};
    bool have_sent_battery_{false};
    uint32_t last_attitude_us_{0};
    uint32_t last_battery_us_{0};

    uint32_t frames_sent_{0};
    uint32_t frames_dropped_{0};

    void flush();
    bool queue(const uint8_t* data, std::size_t len);
};

}  // namespace fc
