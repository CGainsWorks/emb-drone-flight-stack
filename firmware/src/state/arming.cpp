#include "fc/state/arming.hpp"

#include <cmath>

#include "fc/hal/clock.hpp"

namespace fc {

void ArmingStateMachine::reset() {
    state_ = ArmState::Disarmed;
    blocker_ = ArmBlocker::SwitchNotCycled;
    failsafe_cause_ = FailsafeCause::None;
    switch_cycled_ = false;
    battery_low_active_ = false;
    battery_low_since_us_ = 0;
}

bool ArmingStateMachine::rx_link_ok(const ArmingInputs& in, uint32_t now_us) const {
    if (!in.rx_ever_received) {
        return false;
    }
    return elapsed_us(now_us, in.last_rx_us) < cfg_.rx_timeout_us;
}

ArmBlocker ArmingStateMachine::evaluate_arm_blockers(const ArmingInputs& in,
                                                     uint32_t now_us) const {
    // Ordered roughly by how fundamental the problem is, so the reported cause
    // is the most useful one when several apply at once.
    if (!switch_cycled_) {
        return ArmBlocker::SwitchNotCycled;
    }
    if (!in.imu_healthy) {
        return ArmBlocker::ImuUnhealthy;
    }
    if (!in.calibrated) {
        return ArmBlocker::NotCalibrated;
    }
    if (!rx_link_ok(in, now_us)) {
        return ArmBlocker::NoRxLink;
    }
    if (in.throttle > cfg_.max_arm_throttle) {
        return ArmBlocker::ThrottleNotLow;
    }
    if (std::fabs(in.tilt_rad) > cfg_.max_arm_tilt_rad) {
        return ArmBlocker::TiltTooHigh;
    }
    if (in.battery_voltage < cfg_.min_arm_voltage) {
        return ArmBlocker::BatteryLow;
    }
    return ArmBlocker::None;
}

void ArmingStateMachine::update(const ArmingInputs& in, uint32_t now_us) {
    // The switch must be observed off at least once. A board that boots with
    // the switch already on must not arm until the pilot cycles it.
    if (!in.arm_switch) {
        switch_cycled_ = true;
    }

    // Track how long the pack has been under the critical threshold. A single
    // sag under a throttle punch must not trigger failsafe.
    const bool below_critical = in.battery_voltage < cfg_.critical_voltage;
    if (below_critical) {
        if (!battery_low_active_) {
            battery_low_active_ = true;
            battery_low_since_us_ = now_us;
        }
    } else {
        battery_low_active_ = false;
    }
    const bool battery_critical =
        battery_low_active_ &&
        elapsed_us(now_us, battery_low_since_us_) >= cfg_.battery_debounce_us;

    switch (state_) {
        case ArmState::Disarmed: {
            failsafe_cause_ = FailsafeCause::None;
            const ArmBlocker blocker = evaluate_arm_blockers(in, now_us);
            blocker_ = blocker;
            if (in.arm_switch && blocker == ArmBlocker::None) {
                state_ = ArmState::Armed;
                blocker_ = ArmBlocker::None;
            }
            break;
        }

        case ArmState::Armed: {
            blocker_ = ArmBlocker::None;

            if (!in.arm_switch) {
                // Normal disarm. Require another off->on cycle before re-arming.
                state_ = ArmState::Disarmed;
                switch_cycled_ = true;
                break;
            }
            if (!rx_link_ok(in, now_us)) {
                state_ = ArmState::Failsafe;
                failsafe_cause_ = FailsafeCause::RxTimeout;
                break;
            }
            if (!in.imu_healthy) {
                state_ = ArmState::Failsafe;
                failsafe_cause_ = FailsafeCause::ImuFailure;
                break;
            }
            if (battery_critical) {
                state_ = ArmState::Failsafe;
                failsafe_cause_ = FailsafeCause::BatteryCritical;
                break;
            }
            break;
        }

        case ArmState::Failsafe: {
            // Deliberately one-way: only an explicit disarm leaves failsafe.
            // Restoring the link must not silently spin the motors back up.
            if (!in.arm_switch) {
                state_ = ArmState::Disarmed;
                failsafe_cause_ = FailsafeCause::None;
                switch_cycled_ = true;
            }
            break;
        }
    }
}

const char* to_string(ArmState s) {
    switch (s) {
        case ArmState::Disarmed:
            return "DISARMED";
        case ArmState::Armed:
            return "ARMED";
        case ArmState::Failsafe:
            return "FAILSAFE";
    }
    return "UNKNOWN";
}

const char* to_string(ArmBlocker b) {
    switch (b) {
        case ArmBlocker::None:
            return "NONE";
        case ArmBlocker::SwitchNotCycled:
            return "SWITCH_NOT_CYCLED";
        case ArmBlocker::ThrottleNotLow:
            return "THROTTLE_NOT_LOW";
        case ArmBlocker::NoRxLink:
            return "NO_RX_LINK";
        case ArmBlocker::ImuUnhealthy:
            return "IMU_UNHEALTHY";
        case ArmBlocker::NotCalibrated:
            return "NOT_CALIBRATED";
        case ArmBlocker::TiltTooHigh:
            return "TILT_TOO_HIGH";
        case ArmBlocker::BatteryLow:
            return "BATTERY_LOW";
    }
    return "UNKNOWN";
}

const char* to_string(FailsafeCause c) {
    switch (c) {
        case FailsafeCause::None:
            return "NONE";
        case FailsafeCause::RxTimeout:
            return "RX_TIMEOUT";
        case FailsafeCause::BatteryCritical:
            return "BATTERY_CRITICAL";
        case FailsafeCause::ImuFailure:
            return "IMU_FAILURE";
    }
    return "UNKNOWN";
}

}  // namespace fc
