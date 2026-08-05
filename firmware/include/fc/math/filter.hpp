#pragma once

#include <cmath>

namespace fc {

// One-pole IIR low-pass. Used for gyro noise and PID derivative smoothing.
// cutoff_hz <= 0 disables filtering (pass-through), which keeps tests that do
// not care about filtering simple.
class LowPassFilter {
public:
    LowPassFilter() = default;
    explicit LowPassFilter(float cutoff_hz) : cutoff_hz_(cutoff_hz) {}

    void set_cutoff(float cutoff_hz) { cutoff_hz_ = cutoff_hz; }
    float cutoff() const { return cutoff_hz_; }

    void reset() {
        initialised_ = false;
        state_ = 0.0f;
    }
    void reset(float value) {
        initialised_ = true;
        state_ = value;
    }

    float update(float input, float dt) {
        if (cutoff_hz_ <= 0.0f || dt <= 0.0f) {
            state_ = input;
            initialised_ = true;
            return state_;
        }
        if (!initialised_) {
            state_ = input;
            initialised_ = true;
            return state_;
        }
        // alpha = dt / (RC + dt), RC = 1 / (2*pi*fc)
        const float rc = 1.0f / (2.0f * 3.14159265358979323846f * cutoff_hz_);
        const float alpha = dt / (rc + dt);
        state_ += alpha * (input - state_);
        return state_;
    }

    float value() const { return state_; }

private:
    float cutoff_hz_{0.0f};
    float state_{0.0f};
    bool initialised_{false};
};

}  // namespace fc
