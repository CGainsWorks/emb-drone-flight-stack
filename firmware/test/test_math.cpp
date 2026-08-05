#include "fc/math/filter.hpp"
#include "fc/math/quaternion.hpp"
#include "fc/math/vec3.hpp"
#include "test_framework.hpp"

using namespace fc;

namespace {
constexpr float kPi = 3.14159265358979323846f;
constexpr float deg(float d) { return d * kPi / 180.0f; }
}  // namespace

TEST(vec3_basic_algebra) {
    const Vec3 a{1.0f, 2.0f, 3.0f};
    const Vec3 b{4.0f, 5.0f, 6.0f};

    CHECK_NEAR(a.dot(b), 32.0f, 1e-6);

    const Vec3 c = a.cross(b);
    CHECK_NEAR(c.x, -3.0f, 1e-6);
    CHECK_NEAR(c.y, 6.0f, 1e-6);
    CHECK_NEAR(c.z, -3.0f, 1e-6);

    // cross product is perpendicular to both inputs
    CHECK_NEAR(c.dot(a), 0.0f, 1e-5);
    CHECK_NEAR(c.dot(b), 0.0f, 1e-5);
}

TEST(vec3_normalize_handles_zero_vector) {
    const Vec3 zero{0.0f, 0.0f, 0.0f};
    const Vec3 n = zero.normalized();
    // Must not produce NaN/inf: a zero accel reading in free fall must not
    // poison the estimator.
    CHECK(n.is_finite());
    CHECK_NEAR(n.norm(), 0.0f, 1e-9);

    const Vec3 v{3.0f, 0.0f, 4.0f};
    CHECK_NEAR(v.normalized().norm(), 1.0f, 1e-6);
}

TEST(quaternion_identity_rotation_is_noop) {
    const Quaternion q = Quaternion::identity();
    const Vec3 v{1.0f, -2.0f, 3.0f};
    const Vec3 r = q.rotate(v);
    CHECK_NEAR(r.x, v.x, 1e-6);
    CHECK_NEAR(r.y, v.y, 1e-6);
    CHECK_NEAR(r.z, v.z, 1e-6);
}

TEST(quaternion_90deg_yaw_rotates_x_to_y) {
    // Yaw is about body/earth z (down). Rotating the North vector by +90 deg
    // yaw should point it East.
    const Quaternion q = Quaternion::from_axis_angle({0.0f, 0.0f, 1.0f}, deg(90.0f));
    const Vec3 r = q.rotate({1.0f, 0.0f, 0.0f});
    CHECK_NEAR(r.x, 0.0f, 1e-5);
    CHECK_NEAR(r.y, 1.0f, 1e-5);
    CHECK_NEAR(r.z, 0.0f, 1e-5);
}

TEST(quaternion_rotation_preserves_length) {
    const Quaternion q = Quaternion::from_euler(deg(31.0f), deg(-17.0f), deg(88.0f));
    const Vec3 v{0.3f, -1.7f, 2.2f};
    CHECK_NEAR(q.rotate(v).norm(), v.norm(), 1e-4);
}

TEST(quaternion_inverse_rotate_round_trips) {
    const Quaternion q = Quaternion::from_euler(deg(20.0f), deg(35.0f), deg(-50.0f));
    const Vec3 v{1.0f, 2.0f, -0.5f};
    const Vec3 back = q.inverse_rotate(q.rotate(v));
    CHECK_NEAR(back.x, v.x, 1e-4);
    CHECK_NEAR(back.y, v.y, 1e-4);
    CHECK_NEAR(back.z, v.z, 1e-4);
}

TEST(euler_round_trip_matches_input) {
    const float roll = deg(25.0f);
    const float pitch = deg(-12.0f);
    const float yaw = deg(140.0f);

    const EulerAngles e = Quaternion::from_euler(roll, pitch, yaw).to_euler();
    CHECK_NEAR(e.roll, roll, 1e-4);
    CHECK_NEAR(e.pitch, pitch, 1e-4);
    CHECK_NEAR(e.yaw, yaw, 1e-4);
}

TEST(euler_pitch_is_clamped_at_gimbal_lock) {
    // Straight up: pitch = +90 deg. asin must not go out of domain and produce
    // NaN, which would propagate into the whole control loop.
    const Quaternion q = Quaternion::from_euler(0.0f, deg(90.0f), 0.0f);
    const EulerAngles e = q.to_euler();
    CHECK(std::isfinite(e.roll));
    CHECK(std::isfinite(e.pitch));
    CHECK(std::isfinite(e.yaw));
    CHECK_NEAR(e.pitch, deg(90.0f), 1e-3);
}

TEST(quaternion_normalize_recovers_from_zero) {
    Quaternion q{0.0f, 0.0f, 0.0f, 0.0f};
    q.normalize();
    CHECK(q.is_finite());
    CHECK_NEAR(q.norm(), 1.0f, 1e-6);
}

TEST(level_attitude_expects_negative_z_gravity_in_body) {
    // Documents the sign convention the estimator relies on: with body z down,
    // a level board reads (0, 0, -1) g of specific force.
    const Quaternion level = Quaternion::identity();
    const Vec3 expected = level.inverse_rotate({0.0f, 0.0f, -1.0f});
    CHECK_NEAR(expected.x, 0.0f, 1e-6);
    CHECK_NEAR(expected.y, 0.0f, 1e-6);
    CHECK_NEAR(expected.z, -1.0f, 1e-6);
}

TEST(lowpass_converges_to_dc_input) {
    LowPassFilter f{10.0f};
    float out = 0.0f;
    for (int i = 0; i < 2000; ++i) {
        out = f.update(5.0f, 0.001f);
    }
    CHECK_NEAR(out, 5.0f, 1e-3);
}

TEST(lowpass_first_sample_does_not_ramp_from_zero) {
    // Seeding matters: if the filter started at 0 it would inject a large
    // transient on the first control cycle.
    LowPassFilter f{10.0f};
    CHECK_NEAR(f.update(42.0f, 0.001f), 42.0f, 1e-6);
}

TEST(lowpass_attenuates_high_frequency) {
    // A signal alternating every sample should be heavily attenuated.
    LowPassFilter f{5.0f};
    f.update(0.0f, 0.001f);
    float last = 0.0f;
    for (int i = 0; i < 500; ++i) {
        last = f.update((i % 2 == 0) ? 1.0f : -1.0f, 0.001f);
    }
    CHECK(std::fabs(last) < 0.2f);
}

TEST(lowpass_zero_cutoff_is_passthrough) {
    LowPassFilter f{0.0f};
    CHECK_NEAR(f.update(3.0f, 0.001f), 3.0f, 1e-6);
    CHECK_NEAR(f.update(-9.0f, 0.001f), -9.0f, 1e-6);
}

TEST_MAIN()
