// Software-in-the-loop checks: the real FlightController flying a simulated
// quadrotor (firmware/sim/sil). Everything asserted here is about simulated
// behaviour against an assumed plant, not about any physical aircraft.
//
// Two groups:
//  * the plant itself obeys the sign conventions the firmware assumes, so a
//    pass below cannot come from two matching mistakes;
//  * the CI stability gate: with the firmware's default gains every standard
//    step settles inside its overshoot and settling limits, on the nominal
//    plant and on a slower, heavier variant.
#include <cmath>

#include "fc/control/mixer.hpp"
#include "quad_plant.hpp"
#include "sil_harness.hpp"
#include "test_framework.hpp"

using namespace fc;
using namespace fc::sil;

namespace {

constexpr std::size_t FR = 0;
constexpr std::size_t RR = 1;
constexpr std::size_t RL = 2;
constexpr std::size_t FL = 3;

std::array<double, 4> hover_with(std::size_t motor, double delta) {
    const QuadParams p;
    std::array<double, 4> s{};
    s.fill(p.hover_command());
    s[motor] += delta;
    return s;
}

std::array<double, 4> from_mixer(const MixerCommand& cmd) {
    const MixerOutput o = mix_quad_x(cmd);
    return {o.motor[0], o.motor[1], o.motor[2], o.motor[3]};
}

QuadParams perturbed() {
    QuadParams p;
    p.motor_tau_s = 0.045;
    p.ixx *= 1.3;
    p.iyy *= 1.3;
    p.izz *= 1.3;
    return p;
}

void check_scenarios_pass(const QuadParams& plant) {
    const FlightControllerConfig defaults;
    for (const auto& s : standard_scenarios()) {
        const RunResult r = run_scenario(s, defaults, plant);
        const StepLimits l = limits_for(s);
        CHECK(r.armed);
        CHECK(r.metrics.bounded);
        CHECK(r.metrics.settled);
        CHECK(r.metrics.overshoot_pct <= l.max_overshoot_pct);
        CHECK(r.metrics.settling_time_s <= l.max_settling_time_s);
        CHECK(r.metrics.steady_state_error_pct <= l.max_steady_state_error_pct);
        if (!within_limits(r.metrics, l)) {
            std::printf("    %s: overshoot %.2f %% settling %.3f s ss error %.2f %%\n",
                        s.name.c_str(), r.metrics.overshoot_pct, r.metrics.settling_time_s,
                        r.metrics.steady_state_error_pct);
        }
    }
}

}  // namespace

TEST(plant_hover_command_balances_gravity) {
    QuadParams p;
    p.disturbance_x = p.disturbance_y = p.disturbance_z = 0.0;
    QuadPlant plant(p);
    plant.state().motor_speed.fill(p.hover_command());
    std::array<double, 4> cmd{};
    cmd.fill(p.hover_command());
    for (int i = 0; i < 4000; ++i) {
        plant.step(cmd, 0.00025);
    }
    CHECK_NEAR(plant.state().velocity.z, 0.0, 1e-9);
    CHECK_NEAR(plant.specific_force_g().z, -1.0, 1e-9);
}

TEST(plant_left_motors_roll_right_side_down) {
    const QuadPlant plant;
    CHECK(plant.body_torque(hover_with(FL, 0.05)).x > 0.0);
    CHECK(plant.body_torque(hover_with(RL, 0.05)).x > 0.0);
    CHECK(plant.body_torque(hover_with(FR, 0.05)).x < 0.0);
    CHECK(plant.body_torque(hover_with(RR, 0.05)).x < 0.0);
}

TEST(plant_front_motors_pitch_nose_up) {
    const QuadPlant plant;
    CHECK(plant.body_torque(hover_with(FL, 0.05)).y > 0.0);
    CHECK(plant.body_torque(hover_with(FR, 0.05)).y > 0.0);
    CHECK(plant.body_torque(hover_with(RL, 0.05)).y < 0.0);
    CHECK(plant.body_torque(hover_with(RR, 0.05)).y < 0.0);
}

TEST(plant_ccw_props_yaw_nose_right) {
    const QuadPlant plant;
    CHECK(plant.body_torque(hover_with(FR, 0.05)).z > 0.0);
    CHECK(plant.body_torque(hover_with(RL, 0.05)).z > 0.0);
    CHECK(plant.body_torque(hover_with(RR, 0.05)).z < 0.0);
    CHECK(plant.body_torque(hover_with(FL, 0.05)).z < 0.0);
}

TEST(mixer_commands_produce_moments_of_the_same_sign) {
    // The mixer's documented conventions, checked against independent physics
    // rather than against the mixer's own table.
    const QuadPlant plant;
    const V3 roll = plant.body_torque(from_mixer({0.42f, 0.1f, 0.0f, 0.0f}));
    const V3 pitch = plant.body_torque(from_mixer({0.42f, 0.0f, 0.1f, 0.0f}));
    const V3 yaw = plant.body_torque(from_mixer({0.42f, 0.0f, 0.0f, 0.1f}));
    CHECK(roll.x > 0.0);
    CHECK(std::fabs(roll.y) < 1e-9);
    CHECK(pitch.y > 0.0);
    CHECK(std::fabs(pitch.x) < 1e-9);
    CHECK(yaw.z > 0.0);
    CHECK(std::fabs(yaw.x) < 1e-9);
    CHECK(std::fabs(yaw.y) < 1e-9);
}

TEST(plant_specific_force_tilts_with_the_airframe) {
    // Held at 10 deg right side down in a hover, the accelerometer sees the
    // thrust direction: straight down the body z axis.
    QuadParams p;
    p.disturbance_x = p.disturbance_y = p.disturbance_z = 0.0;
    QuadPlant plant(p);
    plant.state().attitude = Q4::from_euler(10.0 * 3.14159265358979 / 180.0, 0.0, 0.0);
    std::array<double, 4> cmd{};
    cmd.fill(p.hover_command());
    plant.state().motor_speed = cmd;
    plant.step(cmd, 0.00025);
    const V3 f = plant.specific_force_g();
    CHECK_NEAR(f.x, 0.0, 1e-6);
    CHECK_NEAR(f.y, 0.0, 1e-6);
    CHECK(f.z < -0.99);
}

TEST(metrics_recognise_a_clean_first_order_step) {
    std::vector<Sample> v;
    for (int k = 0; k < 2000; ++k) {
        Sample s{};
        s.t = k * 0.001;
        s.setpoint = s.t >= 0.5 ? 10.0 : 0.0;
        s.response = s.t >= 0.5 ? 10.0 * (1.0 - std::exp(-(s.t - 0.5) / 0.05)) : 0.0;
        s.estimate = s.response;
        v.push_back(s);
    }
    const StepMetrics m = compute_metrics(v, 0.5, false);
    CHECK(m.bounded);
    CHECK(m.settled);
    CHECK_NEAR(m.overshoot_pct, 0.0, 1e-9);
    // 5 % band of a first-order lag is reached at 3 tau.
    CHECK_NEAR(m.settling_time_s, 0.15, 0.002);
    CHECK_NEAR(m.rise_time_s, 0.05 * std::log(9.0), 0.002);
}

TEST(metrics_flag_a_divergent_response) {
    std::vector<Sample> v;
    for (int k = 0; k < 2000; ++k) {
        Sample s{};
        s.t = k * 0.001;
        s.setpoint = s.t >= 0.5 ? 10.0 : 0.0;
        s.response = s.t >= 0.5 ? 10.0 * std::exp((s.t - 0.5) * 3.0) : 0.0;
        s.roll_deg = s.response;
        s.estimate = s.response;
        v.push_back(s);
    }
    const StepMetrics m = compute_metrics(v, 0.5, false);
    CHECK(!m.bounded);
    CHECK(!m.settled);
}

TEST(sil_default_gains_are_stable_on_the_nominal_plant) {
    check_scenarios_pass(QuadParams{});
}

TEST(sil_default_gains_are_stable_on_a_slower_heavier_plant) {
    check_scenarios_pass(perturbed());
}

TEST(sil_angle_mode_estimate_drifts_under_sustained_tilt) {
    // Recorded rather than hidden: in a held angle-mode tilt the accelerometer
    // sees only thrust, so the Mahony correction pulls the estimate towards
    // level and the true angle keeps growing past the commanded one. This
    // test pins that the effect exists and stays bounded over the record, so
    // a change to the estimator that alters it is noticed.
    const FlightControllerConfig defaults;
    const RunResult r = run_scenario(standard_scenarios()[3], defaults);
    CHECK(r.scenario.name == "roll_angle_step");
    CHECK(r.metrics.final_true_value > r.metrics.amplitude * 1.2);
    CHECK(r.metrics.final_true_value < 60.0);
}

TEST_MAIN()
