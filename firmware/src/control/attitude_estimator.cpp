#include "fc/control/attitude_estimator.hpp"

#include <cmath>

namespace fc {
namespace {

float clampf(float v, float lo, float hi) {
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

// Direction of measured specific force in the body frame when level.
// See the frame conventions in math/quaternion.hpp.
constexpr Vec3 kGravityRefEarth{0.0f, 0.0f, -1.0f};

}  // namespace

void AttitudeEstimator::reset() {
    q_ = Quaternion::identity();
    bias_ = Vec3{};
    accel_used_ = false;
}

void AttitudeEstimator::align_to_accel(const Vec3& accel) {
    const Vec3 a = accel.normalized();
    if (a.norm() < 0.5f) {
        return;  // unusable reading, leave attitude untouched
    }

    // Find the rotation that takes the expected level reading (0,0,-1) onto the
    // measured direction. That rotation is the initial attitude (yaw arbitrary).
    const Vec3 ref = kGravityRefEarth;
    const Vec3 axis = ref.cross(a);
    const float dot = clampf(ref.dot(a), -1.0f, 1.0f);
    const float axis_len = axis.norm();

    if (axis_len < 1e-6f) {
        // Parallel or anti-parallel to the reference.
        q_ = (dot > 0.0f) ? Quaternion::identity() : Quaternion{0.0f, 1.0f, 0.0f, 0.0f};
        return;
    }

    const float angle = std::acos(dot);
    // The measured vector is the reference seen in the body frame, so the
    // body->earth rotation is the inverse of ref->measured.
    q_ = Quaternion::from_axis_angle(axis, angle).conjugate();
    q_.normalize();
}

void AttitudeEstimator::update(const Vec3& gyro, const Vec3& accel, float dt) {
    accel_used_ = false;

    if (!(dt > 0.0f) || !gyro.is_finite() || !accel.is_finite()) {
        return;
    }

    Vec3 corrected_gyro = gyro;

    const float accel_mag = accel.norm();
    const bool accel_trustworthy = accel_mag > 1e-6f &&
                                   accel_mag >= cfg_.accel_trust_min_g &&
                                   accel_mag <= cfg_.accel_trust_max_g;

    if (accel_trustworthy) {
        const Vec3 measured = accel.normalized();

        // Where the estimator believes "down-reaction" points, in body frame.
        const Vec3 expected = q_.inverse_rotate(kGravityRefEarth);

        // Rotation error between measured and expected, in the body frame.
        const Vec3 error = measured.cross(expected);

        if (cfg_.ki > 0.0f) {
            bias_ += error * (cfg_.ki * dt);
            bias_.x = clampf(bias_.x, -cfg_.max_gyro_bias, cfg_.max_gyro_bias);
            bias_.y = clampf(bias_.y, -cfg_.max_gyro_bias, cfg_.max_gyro_bias);
            bias_.z = clampf(bias_.z, -cfg_.max_gyro_bias, cfg_.max_gyro_bias);
        }

        corrected_gyro += error * cfg_.kp;
        accel_used_ = true;
    }

    corrected_gyro += bias_;

    // Integrate: qdot = 0.5 * q (x) omega_body
    const Quaternion omega{0.0f, corrected_gyro.x, corrected_gyro.y, corrected_gyro.z};
    const Quaternion qdot = q_ * omega;

    q_.w += 0.5f * qdot.w * dt;
    q_.x += 0.5f * qdot.x * dt;
    q_.y += 0.5f * qdot.y * dt;
    q_.z += 0.5f * qdot.z * dt;

    q_.normalize();

    if (!q_.is_finite()) {
        q_ = Quaternion::identity();
    }
}

}  // namespace fc
