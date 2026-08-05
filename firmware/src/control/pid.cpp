#include "fc/control/pid.hpp"

namespace fc {
namespace {

float clamp(float v, float lo, float hi) {
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

}  // namespace

Pid::Pid(const PidGains& gains, const PidLimits& limits) : gains_(gains), limits_(limits) {
    d_filter_.set_cutoff(limits_.d_cutoff_hz);
}

void Pid::set_gains(const PidGains& gains) { gains_ = gains; }

void Pid::set_limits(const PidLimits& limits) {
    limits_ = limits;
    d_filter_.set_cutoff(limits_.d_cutoff_hz);
}

void Pid::reset() {
    integrator_ = 0.0f;
    prev_measurement_ = 0.0f;
    last_output_ = 0.0f;
    have_prev_ = false;
    d_filter_.reset();
}

float Pid::update(float setpoint, float measurement, float dt) {
    if (!(dt > 0.0f)) {
        return last_output_;
    }

    const float error = setpoint - measurement;

    const float p_term = gains_.kp * error;

    // Derivative on measurement: d(error)/dt with a constant setpoint is
    // -d(measurement)/dt, hence the negation.
    float d_term = 0.0f;
    if (have_prev_) {
        const float raw_derivative = -(measurement - prev_measurement_) / dt;
        d_term = gains_.kd * d_filter_.update(raw_derivative, dt);
    } else {
        // First sample: no valid history, so seed the filter without producing
        // a derivative kick.
        d_filter_.reset(0.0f);
    }
    prev_measurement_ = measurement;
    have_prev_ = true;

    // Tentative integration, then decide whether to keep it.
    const float candidate_integrator =
        clamp(integrator_ + gains_.ki * error * dt, -limits_.integral_limit,
              limits_.integral_limit);

    const float unsaturated = p_term + candidate_integrator + d_term;
    const float saturated = clamp(unsaturated, -limits_.output_limit, limits_.output_limit);

    // Conditional integration: only accept the new integrator if we are not
    // saturated, or if the error acts to bring us out of saturation.
    const bool is_saturated = (unsaturated != saturated);
    const bool error_drives_deeper = (unsaturated > 0.0f && error > 0.0f) ||
                                     (unsaturated < 0.0f && error < 0.0f);
    if (!is_saturated || !error_drives_deeper) {
        integrator_ = candidate_integrator;
    }

    // Recompute with the integrator we actually committed to.
    const float output =
        clamp(p_term + integrator_ + d_term, -limits_.output_limit, limits_.output_limit);

    last_output_ = output;
    return output;
}

}  // namespace fc
