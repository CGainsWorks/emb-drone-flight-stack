#include "quad_plant.hpp"

#include <algorithm>
#include <cmath>

namespace fc {
namespace sil {

Q4 Q4::operator*(const Q4& o) const {
    return {w * o.w - x * o.x - y * o.y - z * o.z,
            w * o.x + x * o.w + y * o.z - z * o.y,
            w * o.y - x * o.z + y * o.w + z * o.x,
            w * o.z + x * o.y - y * o.x + z * o.w};
}

void Q4::normalise() {
    const double n = std::sqrt(w * w + x * x + y * y + z * z);
    w /= n;
    x /= n;
    y /= n;
    z /= n;
}

V3 Q4::rotate(const V3& v) const {
    const Q4 p{0.0, v.x, v.y, v.z};
    const Q4 r = (*this) * p * conjugate();
    return {r.x, r.y, r.z};
}

V3 Q4::inverse_rotate(const V3& v) const { return conjugate().rotate(v); }

Q4 Q4::from_euler(double roll, double pitch, double yaw) {
    const double cr = std::cos(roll * 0.5);
    const double sr = std::sin(roll * 0.5);
    const double cp = std::cos(pitch * 0.5);
    const double sp = std::sin(pitch * 0.5);
    const double cy = std::cos(yaw * 0.5);
    const double sy = std::sin(yaw * 0.5);
    return {cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy};
}

void Q4::to_euler(double& roll, double& pitch, double& yaw) const {
    roll = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    pitch = std::asin(std::clamp(2.0 * (w * y - z * x), -1.0, 1.0));
    yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

double QuadParams::hover_command() const {
    // thrust = max_thrust * speed^2 and speed settles on the command.
    return std::sqrt(mass_kg * gravity / (4.0 * max_thrust_n));
}

namespace {

// Motor layout from mixer.hpp. Spin +1 = counter-clockwise seen from above,
// whose reaction torque on the airframe is clockwise, i.e. +z (yaw right).
struct MotorGeometry {
    double x;
    double y;
    double spin;
};

std::array<MotorGeometry, 4> geometry(double arm) {
    const double a = arm / std::sqrt(2.0);
    return {{
        {+a, +a, +1.0},  // 0 front-right, CCW
        {-a, +a, -1.0},  // 1 rear-right, CW
        {-a, -a, +1.0},  // 2 rear-left, CCW
        {+a, -a, -1.0},  // 3 front-left, CW
    }};
}

}  // namespace

double QuadPlant::total_thrust(const std::array<double, 4>& speed) const {
    double t = 0.0;
    for (double s : speed) {
        t += p_.max_thrust_n * s * s;
    }
    return t;
}

V3 QuadPlant::body_torque(const std::array<double, 4>& speed) const {
    const auto g = geometry(p_.arm_m);
    V3 torque{};
    for (std::size_t i = 0; i < 4; ++i) {
        const double thrust = p_.max_thrust_n * speed[i] * speed[i];
        // Thrust acts along body -z at the motor position.
        const V3 r{g[i].x, g[i].y, 0.0};
        torque = torque + r.cross(V3{0.0, 0.0, -thrust});
        torque.z += g[i].spin * p_.torque_per_thrust_m * thrust;
    }
    return torque;
}

void QuadPlant::step(const std::array<double, 4>& command, double dt) {
    // Motors: first-order lag on the normalised speed. Exact discretisation,
    // so the result does not depend on dt being small against tau.
    const double alpha = 1.0 - std::exp(-dt / p_.motor_tau_s);
    for (std::size_t i = 0; i < 4; ++i) {
        const double c = std::clamp(command[i], 0.0, 1.0);
        s_.motor_speed[i] += (c - s_.motor_speed[i]) * alpha;
    }

    const double thrust = total_thrust(s_.motor_speed);
    const V3 torque =
        body_torque(s_.motor_speed) + V3{p_.disturbance_x, p_.disturbance_y, p_.disturbance_z};

    // Translational dynamics in the earth frame.
    const V3 thrust_body{0.0, 0.0, -thrust};
    const V3 drag_earth = s_.velocity * (-p_.linear_drag);
    const V3 non_gravity_earth = s_.attitude.rotate(thrust_body) + drag_earth;
    const V3 accel_earth = non_gravity_earth * (1.0 / p_.mass_kg) + V3{0.0, 0.0, p_.gravity};

    last_specific_force_ = s_.attitude.inverse_rotate(non_gravity_earth * (1.0 / p_.mass_kg));

    // Rotational dynamics, Euler's equations with a diagonal inertia.
    const V3& w = s_.rate;
    const V3 iw{p_.ixx * w.x, p_.iyy * w.y, p_.izz * w.z};
    const V3 gyro = w.cross(iw);
    const V3 w_dot{(torque.x - gyro.x) / p_.ixx, (torque.y - gyro.y) / p_.iyy,
                   (torque.z - gyro.z) / p_.izz};

    // Semi-implicit Euler: velocities first, then positions with the new ones.
    s_.velocity = s_.velocity + accel_earth * dt;
    s_.position = s_.position + s_.velocity * dt;
    s_.rate = s_.rate + w_dot * dt;

    const Q4 omega{0.0, s_.rate.x, s_.rate.y, s_.rate.z};
    const Q4 q_dot = s_.attitude * omega;
    s_.attitude.w += 0.5 * q_dot.w * dt;
    s_.attitude.x += 0.5 * q_dot.x * dt;
    s_.attitude.y += 0.5 * q_dot.y * dt;
    s_.attitude.z += 0.5 * q_dot.z * dt;
    s_.attitude.normalise();
}

}  // namespace sil
}  // namespace fc
