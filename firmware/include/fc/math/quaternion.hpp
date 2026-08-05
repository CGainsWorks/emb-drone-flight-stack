#pragma once

#include <cmath>

#include "fc/math/vec3.hpp"

namespace fc {

// ---------------------------------------------------------------------------
// Frame conventions used throughout this firmware. Fixed here once so every
// module agrees; the attitude tests below assert these signs explicitly.
//
//   Earth frame : NED  (x North, y East, z Down)
//   Body frame  : x forward, y right, z down
//   Quaternion q: rotates a vector FROM body TO earth.
//   Euler order : ZYX -> yaw about z, then pitch about y, then roll about x.
//
// A level, stationary board has an accelerometer reading of (0, 0, -g),
// because the sensor measures specific force (the reaction to gravity) and
// body z points down.
// ---------------------------------------------------------------------------

struct EulerAngles {
    float roll{0.0f};   // rad, positive = right wing down
    float pitch{0.0f};  // rad, positive = nose up
    float yaw{0.0f};    // rad, positive = nose right (clockwise from above)
};

struct Quaternion {
    float w{1.0f};
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};

    constexpr Quaternion() = default;
    constexpr Quaternion(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}

    static constexpr Quaternion identity() { return {1.0f, 0.0f, 0.0f, 0.0f}; }

    static Quaternion from_axis_angle(const Vec3& axis, float angle_rad) {
        const Vec3 a = axis.normalized();
        const float half = angle_rad * 0.5f;
        const float s = std::sin(half);
        return {std::cos(half), a.x * s, a.y * s, a.z * s};
    }

    // Hamilton product. (*this) then rhs, i.e. result rotates by rhs applied in
    // the frame produced by *this.
    Quaternion operator*(const Quaternion& r) const {
        return {w * r.w - x * r.x - y * r.y - z * r.z,
                w * r.x + x * r.w + y * r.z - z * r.y,
                w * r.y - x * r.z + y * r.w + z * r.x,
                w * r.z + x * r.y - y * r.x + z * r.w};
    }

    Quaternion conjugate() const { return {w, -x, -y, -z}; }

    float norm() const { return std::sqrt(w * w + x * x + y * y + z * z); }

    void normalize() {
        const float n = norm();
        if (n < 1e-9f) {
            *this = identity();
            return;
        }
        const float inv = 1.0f / n;
        w *= inv;
        x *= inv;
        y *= inv;
        z *= inv;
    }

    Quaternion normalized() const {
        Quaternion q = *this;
        q.normalize();
        return q;
    }

    // Rotate a vector from the body frame into the earth frame.
    Vec3 rotate(const Vec3& v) const {
        // t = 2 * (q_vec x v); v' = v + w*t + q_vec x t
        const Vec3 qv{x, y, z};
        const Vec3 t = qv.cross(v) * 2.0f;
        return v + t * w + qv.cross(t);
    }

    // Rotate a vector from the earth frame into the body frame.
    Vec3 inverse_rotate(const Vec3& v) const { return conjugate().rotate(v); }

    EulerAngles to_euler() const {
        EulerAngles e;

        // roll (x-axis rotation)
        const float sinr_cosp = 2.0f * (w * x + y * z);
        const float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
        e.roll = std::atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation), clamped to keep asin in domain at gimbal lock
        float sinp = 2.0f * (w * y - z * x);
        if (sinp > 1.0f) {
            sinp = 1.0f;
        }
        if (sinp < -1.0f) {
            sinp = -1.0f;
        }
        e.pitch = std::asin(sinp);

        // yaw (z-axis rotation)
        const float siny_cosp = 2.0f * (w * z + x * y);
        const float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
        e.yaw = std::atan2(siny_cosp, cosy_cosp);

        return e;
    }

    static Quaternion from_euler(float roll, float pitch, float yaw) {
        const float cr = std::cos(roll * 0.5f);
        const float sr = std::sin(roll * 0.5f);
        const float cp = std::cos(pitch * 0.5f);
        const float sp = std::sin(pitch * 0.5f);
        const float cy = std::cos(yaw * 0.5f);
        const float sy = std::sin(yaw * 0.5f);

        return {cr * cp * cy + sr * sp * sy,
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy};
    }

    bool is_finite() const {
        return std::isfinite(w) && std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
    }
};

}  // namespace fc
