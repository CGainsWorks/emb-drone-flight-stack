#include "fc/control/mixer.hpp"

namespace fc {
namespace {

struct MixRow {
    float roll;
    float pitch;
    float yaw;
};

// Order matches the documented index -> position mapping in mixer.hpp.
constexpr MixRow kQuadX[kMotorCount] = {
    {-1.0f, -1.0f, +1.0f},  // 0 front-right, CCW
    {-1.0f, +1.0f, -1.0f},  // 1 rear-right,  CW
    {+1.0f, +1.0f, +1.0f},  // 2 rear-left,   CCW
    {+1.0f, -1.0f, -1.0f},  // 3 front-left,  CW
};

float clamp01(float v) {
    if (v < 0.0f) {
        return 0.0f;
    }
    if (v > 1.0f) {
        return 1.0f;
    }
    return v;
}

}  // namespace

MixerOutput mix_quad_x(const MixerCommand& cmd) {
    MixerOutput out;

    float mix[kMotorCount];
    float min_mix = 0.0f;
    float max_mix = 0.0f;

    for (std::size_t i = 0; i < kMotorCount; ++i) {
        mix[i] = kQuadX[i].roll * cmd.roll + kQuadX[i].pitch * cmd.pitch +
                 kQuadX[i].yaw * cmd.yaw;
        if (i == 0 || mix[i] < min_mix) {
            min_mix = mix[i];
        }
        if (i == 0 || mix[i] > max_mix) {
            max_mix = mix[i];
        }
    }

    // Choose a collective offset that keeps the differential intact if it fits.
    const float range = max_mix - min_mix;
    float base = cmd.throttle;

    if (range > 1.0f) {
        // The commanded differential alone exceeds the available output range.
        // Scale it down uniformly; attitude authority is shared out evenly
        // rather than one motor silently clipping.
        const float scale = 1.0f / range;
        for (std::size_t i = 0; i < kMotorCount; ++i) {
            mix[i] *= scale;
        }
        min_mix *= scale;
        max_mix *= scale;
    }

    // Shift the whole set into range if the throttle puts it over a rail.
    if (base + max_mix > 1.0f) {
        base = 1.0f - max_mix;
    }
    if (base + min_mix < 0.0f) {
        base = -min_mix;
    }

    for (std::size_t i = 0; i < kMotorCount; ++i) {
        out.motor[i] = clamp01(base + mix[i]);
    }

    return out;
}

}  // namespace fc
