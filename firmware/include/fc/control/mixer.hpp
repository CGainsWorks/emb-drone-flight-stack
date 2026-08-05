#pragma once

#include <cstddef>

#include "fc/hal/motor_output.hpp"

namespace fc {

// Quad-X motor mixer.
//
// Motor index -> physical position and prop direction. The board's outputs are
// nets /S1../S4 (see config/board_v105.hpp); which physical arm each is wired
// to is a build-time decision, recorded here as the canonical order:
//
//     index 0 : front-right, CCW
//     index 1 : rear-right,  CW
//     index 2 : rear-left,   CCW
//     index 3 : front-left,  CW
//
// Sign conventions match the attitude estimator (NED, body x forward, y right,
// z down):
//     roll  > 0 : right side down
//     pitch > 0 : nose up
//     yaw   > 0 : nose right (clockwise seen from above)
//
// A motor spinning CW imparts a CCW reaction torque on the airframe, so a
// positive (clockwise) yaw command increases the CCW-spinning motors.

struct MixerCommand {
    float throttle{0.0f};  // 0..1 collective
    float roll{0.0f};      // normalised control authority, typically -1..1
    float pitch{0.0f};
    float yaw{0.0f};
};

struct MixerOutput {
    float motor[kMotorCount]{};
};

// Mixes and then desaturates. If any motor would exceed the 0..1 range, the
// whole set is shifted so that attitude authority is preserved at the expense
// of collective throttle. Losing a little climb rate is always preferable to
// losing the ability to hold the aircraft level.
MixerOutput mix_quad_x(const MixerCommand& cmd);

}  // namespace fc
