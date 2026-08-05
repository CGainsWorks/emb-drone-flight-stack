#pragma once

#include "fc/math/filter.hpp"

namespace fc {

struct PidGains {
    float kp{0.0f};
    float ki{0.0f};
    float kd{0.0f};
};

struct PidLimits {
    // Maximum absolute value of the integral term's contribution.
    float integral_limit{100.0f};
    // Maximum absolute controller output.
    float output_limit{1000.0f};
    // Cutoff for the derivative low-pass. <= 0 disables it.
    float d_cutoff_hz{80.0f};
};

// Single-axis PID.
//
// Design choices that matter on an airframe:
//  * Derivative is taken on the MEASUREMENT, not the error, so a step change in
//    setpoint (a stick flick) does not produce a derivative spike into the
//    motors.
//  * The derivative is low-pass filtered; raw gyro difference is far too noisy
//    to differentiate directly.
//  * Integral uses clamping plus conditional integration: when the output is
//    already saturated, error that would push further into saturation is not
//    accumulated. This prevents the integrator winding up during a sustained
//    saturation (e.g. a blocked motor on the bench).
class Pid {
public:
    Pid() = default;
    Pid(const PidGains& gains, const PidLimits& limits);

    void set_gains(const PidGains& gains);
    void set_limits(const PidLimits& limits);
    const PidGains& gains() const { return gains_; }

    // dt in seconds. Non-positive dt returns the previous output unchanged and
    // does not disturb internal state.
    float update(float setpoint, float measurement, float dt);

    void reset();

    float integrator() const { return integrator_; }
    float last_output() const { return last_output_; }

private:
    PidGains gains_{};
    PidLimits limits_{};
    LowPassFilter d_filter_{};

    float integrator_{0.0f};
    float prev_measurement_{0.0f};
    float last_output_{0.0f};
    bool have_prev_{false};
};

}  // namespace fc
