#include "fc/control/mixer.hpp"
#include "test_framework.hpp"

using namespace fc;

namespace {
// Index -> position, per mixer.hpp
constexpr std::size_t FR = 0;
constexpr std::size_t RR = 1;
constexpr std::size_t RL = 2;
constexpr std::size_t FL = 3;
}  // namespace

TEST(mixer_pure_throttle_is_equal_on_all_motors) {
    const MixerOutput o = mix_quad_x({0.5f, 0.0f, 0.0f, 0.0f});
    for (std::size_t i = 0; i < kMotorCount; ++i) {
        CHECK_NEAR(o.motor[i], 0.5f, 1e-6);
    }
}

TEST(mixer_zero_command_is_all_stopped) {
    const MixerOutput o = mix_quad_x({0.0f, 0.0f, 0.0f, 0.0f});
    for (std::size_t i = 0; i < kMotorCount; ++i) {
        CHECK_NEAR(o.motor[i], 0.0f, 1e-6);
    }
}

TEST(mixer_roll_right_lifts_the_left_side) {
    // Positive roll = right side down, so left motors must spin up and right
    // motors must spin down.
    const MixerOutput o = mix_quad_x({0.5f, 0.2f, 0.0f, 0.0f});
    CHECK(o.motor[FL] > 0.5f);
    CHECK(o.motor[RL] > 0.5f);
    CHECK(o.motor[FR] < 0.5f);
    CHECK(o.motor[RR] < 0.5f);
    // Left pair and right pair should match each other.
    CHECK_NEAR(o.motor[FL], o.motor[RL], 1e-6);
    CHECK_NEAR(o.motor[FR], o.motor[RR], 1e-6);
}

TEST(mixer_pitch_up_lifts_the_rear) {
    // Positive pitch = nose up, so the rear motors do the work.
    const MixerOutput o = mix_quad_x({0.5f, 0.0f, 0.2f, 0.0f});
    CHECK(o.motor[RL] > 0.5f);
    CHECK(o.motor[RR] > 0.5f);
    CHECK(o.motor[FL] < 0.5f);
    CHECK(o.motor[FR] < 0.5f);
}

TEST(mixer_yaw_right_speeds_up_the_ccw_props) {
    // Positive yaw = nose right. CCW props (FR, RL) impart a clockwise reaction
    // torque, so they increase.
    const MixerOutput o = mix_quad_x({0.5f, 0.0f, 0.0f, 0.2f});
    CHECK(o.motor[FR] > 0.5f);
    CHECK(o.motor[RL] > 0.5f);
    CHECK(o.motor[RR] < 0.5f);
    CHECK(o.motor[FL] < 0.5f);
    // Diagonal pairs, so the increase and decrease must balance.
    CHECK_NEAR((o.motor[FR] + o.motor[RL]) - (o.motor[RR] + o.motor[FL]), 0.8f, 1e-5);
}

TEST(mixer_axes_are_independent) {
    // Rolling must not induce pitch, and vice versa.
    const MixerOutput roll_only = mix_quad_x({0.5f, 0.3f, 0.0f, 0.0f});
    const float front = roll_only.motor[FR] + roll_only.motor[FL];
    const float rear = roll_only.motor[RR] + roll_only.motor[RL];
    CHECK_NEAR(front, rear, 1e-5);

    const MixerOutput pitch_only = mix_quad_x({0.5f, 0.0f, 0.3f, 0.0f});
    const float left = pitch_only.motor[FL] + pitch_only.motor[RL];
    const float right = pitch_only.motor[FR] + pitch_only.motor[RR];
    CHECK_NEAR(left, right, 1e-5);
}

TEST(mixer_output_never_leaves_zero_to_one) {
    // Sweep a wide range of absurd commands; nothing may escape the range,
    // because an out-of-range value would wrap when converted to DShot.
    for (int t = -2; t <= 12; ++t) {
        for (int r = -20; r <= 20; r += 5) {
            for (int p = -20; p <= 20; p += 5) {
                for (int y = -20; y <= 20; y += 5) {
                    MixerCommand cmd;
                    cmd.throttle = static_cast<float>(t) / 10.0f;
                    cmd.roll = static_cast<float>(r) / 10.0f;
                    cmd.pitch = static_cast<float>(p) / 10.0f;
                    cmd.yaw = static_cast<float>(y) / 10.0f;
                    const MixerOutput o = mix_quad_x(cmd);
                    for (std::size_t i = 0; i < kMotorCount; ++i) {
                        CHECK(o.motor[i] >= 0.0f);
                        CHECK(o.motor[i] <= 1.0f);
                        CHECK(std::isfinite(o.motor[i]));
                    }
                }
            }
        }
    }
}

TEST(mixer_preserves_attitude_authority_at_full_throttle) {
    // At 100% throttle there is no headroom left to increase a motor. The mixer
    // must lower the collective rather than silently clip and lose all roll
    // authority.
    const MixerOutput o = mix_quad_x({1.0f, 0.3f, 0.0f, 0.0f});
    const float left = o.motor[FL];
    const float right = o.motor[FR];
    CHECK(left > right);
    CHECK_NEAR(left - right, 0.6f, 1e-5);
}

TEST(mixer_preserves_attitude_authority_at_zero_throttle) {
    // Same argument at the bottom of the range.
    const MixerOutput o = mix_quad_x({0.0f, 0.3f, 0.0f, 0.0f});
    CHECK(o.motor[FL] > o.motor[FR]);
    CHECK_NEAR(o.motor[FL] - o.motor[FR], 0.6f, 1e-5);
}

TEST(mixer_scales_down_an_impossible_differential) {
    // Demanding more differential than the output range can express: it must be
    // scaled proportionally, keeping the relative shape, not clipped lopsidedly.
    const MixerOutput o = mix_quad_x({0.5f, 5.0f, 0.0f, 0.0f});
    CHECK_NEAR(o.motor[FL], 1.0f, 1e-5);
    CHECK_NEAR(o.motor[RL], 1.0f, 1e-5);
    CHECK_NEAR(o.motor[FR], 0.0f, 1e-5);
    CHECK_NEAR(o.motor[RR], 0.0f, 1e-5);
}

TEST(mixer_combined_command_is_sane) {
    const MixerOutput o = mix_quad_x({0.4f, 0.1f, 0.1f, 0.05f});
    for (std::size_t i = 0; i < kMotorCount; ++i) {
        CHECK(o.motor[i] >= 0.0f);
        CHECK(o.motor[i] <= 1.0f);
    }
    // Rear-left gets +roll +pitch +yaw and must be the highest.
    for (std::size_t i = 0; i < kMotorCount; ++i) {
        if (i != RL) {
            CHECK(o.motor[RL] >= o.motor[i]);
        }
    }
}

TEST_MAIN()
