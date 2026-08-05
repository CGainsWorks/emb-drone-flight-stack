#pragma once

#include <cstdint>

namespace fc {

enum class ArmState : uint8_t {
    Disarmed = 0,
    Armed = 1,
    Failsafe = 2,
};

// Why arming is currently refused. Surfaced over telemetry so the cause is
// visible on the bench instead of being guessed at.
enum class ArmBlocker : uint8_t {
    None = 0,
    SwitchNotCycled,  // switch was already on at boot; must be cycled off first
    ThrottleNotLow,
    NoRxLink,
    ImuUnhealthy,
    NotCalibrated,
    TiltTooHigh,
    BatteryLow,
};

enum class FailsafeCause : uint8_t {
    None = 0,
    RxTimeout,
    BatteryCritical,
    ImuFailure,
};

struct ArmingConfig {
    float max_arm_throttle{0.05f};      // stick must be down to arm
    float max_arm_tilt_rad{0.436f};     // ~25 deg
    float min_arm_voltage{14.0f};       // 4S pack, refuse to arm below this
    float critical_voltage{13.2f};      // sustained below this -> failsafe
    uint32_t rx_timeout_us{500000};     // 500 ms without a frame -> failsafe
    uint32_t battery_debounce_us{2000000};  // voltage must stay low 2 s
};

struct ArmingInputs {
    bool arm_switch{false};
    float throttle{0.0f};
    uint32_t last_rx_us{0};   // timestamp of the most recent valid RC frame
    bool rx_ever_received{false};
    bool imu_healthy{false};
    bool calibrated{false};
    float battery_voltage{0.0f};
    float tilt_rad{0.0f};     // angle between body z and vertical
};

// Arming and failsafe supervisor.
//
// Safety properties this enforces, each covered by a test:
//  * Motors are only ever enabled in the Armed state.
//  * The arm switch must be seen OFF once before arming is allowed, so a board
//    powered up with the switch already on cannot spin motors immediately.
//  * Failsafe never transitions straight back to Armed. Recovery requires an
//    explicit disarm, so a momentarily restored link cannot cause an
//    unannounced restart of the motors.
class ArmingStateMachine {
public:
    ArmingStateMachine() = default;
    explicit ArmingStateMachine(const ArmingConfig& cfg) : cfg_(cfg) {}

    void set_config(const ArmingConfig& cfg) { cfg_ = cfg; }
    const ArmingConfig& config() const { return cfg_; }

    void update(const ArmingInputs& in, uint32_t now_us);

    ArmState state() const { return state_; }
    bool motors_enabled() const { return state_ == ArmState::Armed; }
    ArmBlocker blocker() const { return blocker_; }
    FailsafeCause failsafe_cause() const { return failsafe_cause_; }

    void reset();

private:
    ArmingConfig cfg_{};
    ArmState state_{ArmState::Disarmed};
    ArmBlocker blocker_{ArmBlocker::SwitchNotCycled};
    FailsafeCause failsafe_cause_{FailsafeCause::None};

    bool switch_cycled_{false};
    bool battery_low_active_{false};
    uint32_t battery_low_since_us_{0};

    bool rx_link_ok(const ArmingInputs& in, uint32_t now_us) const;
    ArmBlocker evaluate_arm_blockers(const ArmingInputs& in, uint32_t now_us) const;
};

const char* to_string(ArmState s);
const char* to_string(ArmBlocker b);
const char* to_string(FailsafeCause c);

}  // namespace fc
