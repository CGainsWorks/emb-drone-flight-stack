#pragma once

#include <cmath>

namespace fc {

// Plain 3-vector. Header-only and free of any hardware dependency so it can be
// compiled and tested natively on a host machine.
struct Vec3 {
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};

    constexpr Vec3() = default;
    constexpr Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    constexpr Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    constexpr Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    constexpr Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    constexpr Vec3 operator-() const { return {-x, -y, -z}; }

    Vec3& operator+=(const Vec3& o) {
        x += o.x;
        y += o.y;
        z += o.z;
        return *this;
    }
    Vec3& operator-=(const Vec3& o) {
        x -= o.x;
        y -= o.y;
        z -= o.z;
        return *this;
    }

    constexpr float dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }

    constexpr Vec3 cross(const Vec3& o) const {
        return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
    }

    float norm() const { return std::sqrt(dot(*this)); }

    // Returns the unit vector, or the zero vector if the magnitude is too small
    // to normalise safely. Callers that care must check before trusting it.
    Vec3 normalized() const {
        const float n = norm();
        if (n < 1e-9f) {
            return {0.0f, 0.0f, 0.0f};
        }
        const float inv = 1.0f / n;
        return {x * inv, y * inv, z * inv};
    }

    bool is_finite() const {
        return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
    }
};

}  // namespace fc
