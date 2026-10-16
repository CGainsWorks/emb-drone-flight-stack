#pragma once

// Rigid-body quadrotor with first-order motor and propeller model.
//
// SIMULATION ONLY. Every parameter below is an assumption chosen to be
// representative of a 5-inch, 4S quad-X; none of them was measured on the
// FCSTMV1.05 airframe, which has never been assembled.
//
// Frames follow the firmware (include/fc/control/mixer.hpp):
//   earth: NED (x north, y east, z down)
//   body:  x forward, y right, z down
// Motor index -> position and spin, as documented in mixer.hpp:
//   0 front-right CCW, 1 rear-right CW, 2 rear-left CCW, 3 front-left CW

#include <array>
#include <cstddef>

namespace fc {
namespace sil {

struct V3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};

    V3 operator+(const V3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    V3 operator-(const V3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    V3 operator*(double s) const { return {x * s, y * s, z * s}; }
    double dot(const V3& o) const { return x * o.x + y * o.y + z * o.z; }
    V3 cross(const V3& o) const {
        return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
    }
};

// Unit quaternion, body -> earth, Hamilton convention (same as the firmware).
struct Q4 {
    double w{1.0};
    double x{0.0};
    double y{0.0};
    double z{0.0};

    Q4 operator*(const Q4& o) const;
    Q4 conjugate() const { return {w, -x, -y, -z}; }
    void normalise();
    V3 rotate(const V3& v) const;          // body -> earth
    V3 inverse_rotate(const V3& v) const;  // earth -> body
    static Q4 from_euler(double roll, double pitch, double yaw);
    void to_euler(double& roll, double& pitch, double& yaw) const;
};

struct QuadParams {
    double mass_kg{0.65};
    double arm_m{0.11};             // centre to motor axis
    double ixx{2.5e-3};             // kg m^2
    double iyy{2.5e-3};
    double izz{4.5e-3};
    double max_thrust_n{9.0};       // per motor at full command
    double torque_per_thrust_m{0.015};  // rotor drag torque / thrust
    double motor_tau_s{0.030};      // first-order spin-up time constant
    double linear_drag{0.25};       // N per m/s, translational
    double gravity{9.80665};

    // Constant body torque standing in for a centre-of-mass offset and
    // mismatched motors (N m). Without it every axis is perfectly trimmed and
    // an integrator term would look useless to the tuner.
    double disturbance_x{0.02};
    double disturbance_y{-0.015};
    double disturbance_z{0.004};

    // Throttle command that holds a level hover with all four motors equal.
    double hover_command() const;
};

struct QuadState {
    V3 position;      // m, NED
    V3 velocity;      // m/s, NED
    Q4 attitude;      // body -> earth
    V3 rate;          // rad/s, body
    std::array<double, 4> motor_speed{};  // normalised 0..1
};

class QuadPlant {
public:
    explicit QuadPlant(const QuadParams& p = QuadParams{}) : p_(p) {}

    const QuadParams& params() const { return p_; }
    QuadState& state() { return s_; }
    const QuadState& state() const { return s_; }

    // Advance by dt seconds with motor commands in 0..1.
    void step(const std::array<double, 4>& command, double dt);

    // What an ideal accelerometer on the body would read, in g: specific
    // force (acceleration minus gravity) rotated into the body frame.
    V3 specific_force_g() const { return last_specific_force_ * (1.0 / p_.gravity); }

    // Per-motor thrust and the body torque they produce, for the sign tests.
    V3 body_torque(const std::array<double, 4>& speed) const;
    double total_thrust(const std::array<double, 4>& speed) const;

private:
    QuadParams p_;
    QuadState s_;
    V3 last_specific_force_{0.0, 0.0, -9.80665};
};

}  // namespace sil
}  // namespace fc
