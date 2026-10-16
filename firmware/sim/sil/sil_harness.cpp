#include "sil_harness.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <sstream>

#include "mocks/mock_spi.hpp"

namespace fc {
namespace sil {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kControlDt = 0.001;   // firmware loop: 1 kHz
constexpr int kPhysicsSubsteps = 4;    // plant integrated at 4 kHz
constexpr float kBatteryVolts = 16.0f; // nominal 4S, no sag modelled
// Assumed output latency between the firmware writing a motor command and the
// ESC acting on it (PWM frame plus ESC processing), in control cycles.
constexpr std::size_t kOutputDelayCycles = 2;

// The firmware configures the IMU for +/-16 g and +/-2000 dps.
constexpr double kAccelLsbPerG = 32768.0 / 16.0;
constexpr double kGyroLsbPerDps = 32768.0 / 2000.0;

// Deterministic Gaussian source. std::normal_distribution is implementation
// defined, so it would make results differ between standard libraries.
class Noise {
public:
    explicit Noise(uint32_t seed) : state_(seed == 0 ? 1u : seed) {}

    double gaussian() {
        if (have_spare_) {
            have_spare_ = false;
            return spare_;
        }
        double u1 = 0.0;
        do {
            u1 = uniform();
        } while (u1 <= 1e-12);
        const double u2 = uniform();
        const double mag = std::sqrt(-2.0 * std::log(u1));
        spare_ = mag * std::sin(2.0 * kPi * u2);
        have_spare_ = true;
        return mag * std::cos(2.0 * kPi * u2);
    }

private:
    double uniform() {
        state_ ^= state_ << 13;
        state_ ^= state_ >> 17;
        state_ ^= state_ << 5;
        return static_cast<double>(state_) / 4294967296.0;
    }

    uint32_t state_;
    double spare_{0.0};
    bool have_spare_{false};
};

int16_t to_raw(double value, double lsb_per_unit) {
    const double raw = std::round(value * lsb_per_unit);
    return static_cast<int16_t>(std::clamp(raw, -32768.0, 32767.0));
}

uint16_t stick_to_channel(float stick) {
    const double mid = crsf::kChannelMid;
    const double half_span = (crsf::kChannelMax - crsf::kChannelMin) * 0.5;
    return static_cast<uint16_t>(std::lround(mid + std::clamp(stick, -1.0f, 1.0f) * half_span));
}

uint16_t throttle_to_channel(double throttle) {
    const double span = crsf::kChannelMax - crsf::kChannelMin;
    return static_cast<uint16_t>(std::lround(crsf::kChannelMin + std::clamp(throttle, 0.0, 1.0) * span));
}

std::size_t stick_index(Axis a) {
    switch (a) {
        case Axis::Roll:
            return rc::kRoll;
        case Axis::Pitch:
            return rc::kPitch;
        case Axis::Yaw:
            return rc::kYaw;
    }
    return rc::kRoll;
}

struct Rig {
    testing_mocks::MockIcm42688 dev;
    testing_mocks::FakeClock clock;
    testing_mocks::MockMotorOutput motors;
    Icm42688 imu{dev, clock};
    FlightController fc{imu, motors, clock};
    crsf::RcChannels sticks{};

    Rig() {
        for (auto& c : sticks.channel) {
            c = crsf::kChannelMid;
        }
        sticks.channel[rc::kThrottle] = crsf::kChannelMin;
        sticks.channel[rc::kArmSwitch] = crsf::kChannelMin;
    }

    void load_imu(const V3& accel_g, const V3& rate_rad_s) {
        dev.set_accel_raw(to_raw(accel_g.x, kAccelLsbPerG), to_raw(accel_g.y, kAccelLsbPerG),
                          to_raw(accel_g.z, kAccelLsbPerG));
        dev.set_gyro_raw(to_raw(rate_rad_s.x * kRadToDeg, kGyroLsbPerDps),
                         to_raw(rate_rad_s.y * kRadToDeg, kGyroLsbPerDps),
                         to_raw(rate_rad_s.z * kRadToDeg, kGyroLsbPerDps));
    }

    void tick() {
        clock.advance_us(1000);
        fc.on_rc_frame(sticks, clock.micros());
        fc.update(clock.micros());
    }
};

}  // namespace

std::vector<Scenario> standard_scenarios() {
    std::vector<Scenario> v;
    // Rate mode: inner loop only. 0.15 of full stick is 0.9 rad/s, which
    // keeps the attitude well inside 80 deg over the record.
    v.push_back({"roll_rate_step", FlightMode::Rate, Axis::Roll, 0.15f, 0.5, 1.2});
    v.push_back({"pitch_rate_step", FlightMode::Rate, Axis::Pitch, 0.15f, 0.5, 1.2});
    v.push_back({"yaw_rate_step", FlightMode::Rate, Axis::Yaw, 0.25f, 0.5, 2.0});
    // Angle mode: outer loop plus estimator. Half stick is about 17.5 deg.
    v.push_back({"roll_angle_step", FlightMode::Angle, Axis::Roll, 0.5f, 0.5, 2.5});
    v.push_back({"pitch_angle_step", FlightMode::Angle, Axis::Pitch, 0.5f, 0.5, 2.5});
    return v;
}

// Requirements, not results: chosen before tuning as what a small quad's
// attitude loops should achieve and left alone afterwards.
StepLimits limits_for(const Scenario& s) {
    if (judged_on_estimate(s)) {
        return {20.0, 1.0, 5.0};  // angle: settle inside 1 s
    }
    if (s.axis == Axis::Yaw) {
        return {25.0, 0.5, 5.0};
    }
    return {25.0, 0.25, 5.0};     // roll and pitch rate
}

bool within_limits(const StepMetrics& m, const StepLimits& l) {
    return m.bounded && m.settled && m.overshoot_pct <= l.max_overshoot_pct &&
           m.settling_time_s <= l.max_settling_time_s &&
           m.steady_state_error_pct <= l.max_steady_state_error_pct;
}

SensorNoise default_noise() { return {0.002, 0.002, 1u}; }

StepMetrics compute_metrics(const std::vector<Sample>& samples, double step_time_s,
                            bool use_estimate) {
    StepMetrics m;
    if (samples.empty()) {
        return m;
    }

    m.bounded = true;
    for (const auto& s : samples) {
        const bool finite = std::isfinite(s.response) && std::isfinite(s.roll_deg) &&
                            std::isfinite(s.pitch_deg);
        if (!finite || std::fabs(s.roll_deg) > 80.0 || std::fabs(s.pitch_deg) > 80.0 ||
            std::fabs(s.p_dps) > 2000.0 || std::fabs(s.q_dps) > 2000.0 ||
            std::fabs(s.r_dps) > 2000.0) {
            m.bounded = false;
        }
    }

    std::size_t k0 = 0;
    while (k0 < samples.size() && samples[k0].t < step_time_s) {
        ++k0;
    }
    if (k0 == 0 || k0 >= samples.size()) {
        return m;
    }

    const auto value = [use_estimate](const Sample& smp) {
        return use_estimate ? smp.estimate : smp.response;
    };
    for (std::size_t k = k0; k < samples.size(); ++k) {
        m.max_estimator_error =
            std::max(m.max_estimator_error, std::fabs(samples[k].response - samples[k].estimate));
    }
    m.final_true_value = samples.back().response;

    const double y0 = value(samples[k0 - 1]);
    const double target = samples.back().setpoint;
    m.amplitude = target - y0;
    if (std::fabs(m.amplitude) < 1e-9) {
        return m;
    }

    double t10 = -1.0;
    double t90 = -1.0;
    double peak = 0.0;
    double last_outside = samples[k0].t;
    bool ever_outside = false;
    for (std::size_t k = k0; k < samples.size(); ++k) {
        const double e = (value(samples[k]) - y0) / m.amplitude;
        if (t10 < 0.0 && e >= 0.1) {
            t10 = samples[k].t;
        }
        if (t90 < 0.0 && e >= 0.9) {
            t90 = samples[k].t;
        }
        peak = std::max(peak, e);
        if (std::fabs(e - 1.0) > 0.05) {
            last_outside = samples[k].t;
            ever_outside = true;
        }
    }

    m.rise_time_s = (t10 >= 0.0 && t90 >= 0.0) ? (t90 - t10)
                                               : std::numeric_limits<double>::infinity();
    m.overshoot_pct = std::max(0.0, peak - 1.0) * 100.0;

    // Settled means inside the band from some point on, with at least the final
    // 0.3 s of the record inside it.
    const double end = samples.back().t;
    m.settled = !ever_outside || (end - last_outside) >= 0.3;
    m.settling_time_s = m.settled ? (last_outside - step_time_s)
                                  : std::numeric_limits<double>::infinity();

    double sum = 0.0;
    int n = 0;
    for (std::size_t k = k0; k < samples.size(); ++k) {
        if (samples[k].t >= end - 0.3) {
            sum += (value(samples[k]) - y0) / m.amplitude;
            ++n;
        }
    }
    m.steady_state_error_pct = n > 0 ? std::fabs(sum / n - 1.0) * 100.0 : 100.0;
    return m;
}

RunResult run_scenario(const Scenario& s, const FlightControllerConfig& base_cfg,
                       const QuadParams& plant_params, const SensorNoise& noise_cfg) {
    RunResult out;
    out.scenario = s;

    FlightControllerConfig cfg = base_cfg;
    cfg.mode = s.mode;

    Rig rig;
    QuadPlant plant(plant_params);
    Noise noise(noise_cfg.seed);

    // Boot on the ground: level and still, so init and gyro calibration see
    // exactly what a powered board on a bench would.
    rig.load_imu(V3{0.0, 0.0, -1.0}, V3{});
    rig.fc.set_config(cfg);
    rig.fc.set_battery_voltage(kBatteryVolts);
    if (!rig.fc.init() || !rig.fc.calibrate_gyro()) {
        return out;
    }

    // Arm with the switch cycled and throttle down. The plant is held still.
    for (int i = 0; i < 20; ++i) {
        rig.tick();
    }
    rig.sticks.channel[rc::kArmSwitch] = crsf::kChannelMax;
    for (int i = 0; i < 20; ++i) {
        rig.tick();
    }
    out.armed = rig.fc.arming().state() == ArmState::Armed;
    if (!out.armed) {
        return out;
    }

    // Hover throttle as an open-loop collective: the firmware has no altitude
    // hold, so the pilot (here, the script) supplies it.
    rig.sticks.channel[rc::kThrottle] = throttle_to_channel(plant_params.hover_command());

    const std::size_t axis_channel = stick_index(s.axis);
    double setpoint_after = s.stick * cfg.max_angle_rad * kRadToDeg;
    if (s.axis == Axis::Yaw) {
        setpoint_after = s.stick * cfg.max_yaw_rate_rad_s * kRadToDeg;
    } else if (s.mode == FlightMode::Rate) {
        setpoint_after = s.stick * cfg.max_rate_rad_s * kRadToDeg;
    }

    const int cycles = static_cast<int>(std::lround(s.duration_s / kControlDt));
    out.samples.reserve(static_cast<std::size_t>(cycles));

    // Commands in flight to the ESCs, oldest first.
    std::vector<std::array<double, 4>> pipeline(kOutputDelayCycles, std::array<double, 4>{});

    for (int k = 0; k < cycles; ++k) {
        const double t = k * kControlDt;
        const bool stepped = t >= s.step_time_s;
        rig.sticks.channel[axis_channel] = stick_to_channel(stepped ? s.stick : 0.0f);

        const QuadState& st = plant.state();
        const V3 f = plant.specific_force_g();
        const V3 accel{f.x + noise_cfg.accel_sigma_g * noise.gaussian(),
                       f.y + noise_cfg.accel_sigma_g * noise.gaussian(),
                       f.z + noise_cfg.accel_sigma_g * noise.gaussian()};
        const V3 rate{st.rate.x + noise_cfg.gyro_sigma_rad_s * noise.gaussian(),
                      st.rate.y + noise_cfg.gyro_sigma_rad_s * noise.gaussian(),
                      st.rate.z + noise_cfg.gyro_sigma_rad_s * noise.gaussian()};
        rig.load_imu(accel, rate);

        rig.tick();

        std::array<double, 4> written{};
        if (rig.motors.armed_output) {
            for (std::size_t i = 0; i < 4; ++i) {
                written[i] = static_cast<double>(rig.motors.motor[i]);
            }
        }
        pipeline.push_back(written);
        const std::array<double, 4> cmd = pipeline.front();
        pipeline.erase(pipeline.begin());
        for (int j = 0; j < kPhysicsSubsteps; ++j) {
            plant.step(cmd, kControlDt / kPhysicsSubsteps);
        }

        Sample smp{};
        smp.t = t + kControlDt;
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
        plant.state().attitude.to_euler(roll, pitch, yaw);
        const EulerAngles est = rig.fc.estimator().euler();
        smp.roll_deg = roll * kRadToDeg;
        smp.pitch_deg = pitch * kRadToDeg;
        smp.yaw_deg = yaw * kRadToDeg;
        smp.est_roll_deg = est.roll * kRadToDeg;
        smp.est_pitch_deg = est.pitch * kRadToDeg;
        smp.est_yaw_deg = est.yaw * kRadToDeg;
        smp.p_dps = plant.state().rate.x * kRadToDeg;
        smp.q_dps = plant.state().rate.y * kRadToDeg;
        smp.r_dps = plant.state().rate.z * kRadToDeg;
        for (std::size_t i = 0; i < 4; ++i) {
            smp.motor[i] = written[i];
        }
        smp.altitude_m = -plant.state().position.z;
        smp.setpoint = stepped ? setpoint_after : 0.0;
        const Vec3 gyro = rig.fc.last_sample().gyro;
        const bool rate_step = s.mode == FlightMode::Rate || s.axis == Axis::Yaw;
        switch (s.axis) {
            case Axis::Roll:
                smp.response = rate_step ? smp.p_dps : smp.roll_deg;
                smp.estimate = rate_step ? static_cast<double>(gyro.x) * kRadToDeg
                                         : smp.est_roll_deg;
                break;
            case Axis::Pitch:
                smp.response = rate_step ? smp.q_dps : smp.pitch_deg;
                smp.estimate = rate_step ? static_cast<double>(gyro.y) * kRadToDeg
                                         : smp.est_pitch_deg;
                break;
            case Axis::Yaw:
                smp.response = smp.r_dps;
                smp.estimate = static_cast<double>(gyro.z) * kRadToDeg;
                break;
        }
        out.samples.push_back(smp);
    }

    out.metrics = compute_metrics(out.samples, s.step_time_s, judged_on_estimate(s));

    // Motor command activity in steady hover before the step: mean absolute
    // change per cycle. Sensor noise amplified by the gains shows up here, so
    // the tuner can be stopped from buying speed with a noisy, hot output.
    double activity = 0.0;
    int n = 0;
    for (std::size_t k = 1; k < out.samples.size(); ++k) {
        if (out.samples[k].t < 0.25 || out.samples[k].t >= s.step_time_s) {
            continue;
        }
        for (std::size_t i = 0; i < 4; ++i) {
            activity += std::fabs(out.samples[k].motor[i] - out.samples[k - 1].motor[i]);
        }
        ++n;
    }
    out.metrics.hover_motor_activity = n > 0 ? activity / (4.0 * n) : 0.0;
    return out;
}

bool write_csv(const RunResult& r, const std::string& path) {
    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (f == nullptr) {
        return false;
    }
    const bool rate = r.scenario.mode == FlightMode::Rate || r.scenario.axis == Axis::Yaw;
    std::fprintf(f, "# SIMULATED: software-in-the-loop run of the flight firmware against an\n");
    std::fprintf(f, "# assumed quadrotor model (firmware/sim/sil). Not measured on hardware.\n");
    std::fprintf(f, "# scenario=%s units=%s judged_on=%s\n", r.scenario.name.c_str(),
                 rate ? "deg/s" : "deg", judged_on_estimate(r.scenario) ? "estimate" : "response");
    std::fprintf(f, "# rows every 2 ms; control loop 1 kHz, plant integrated at 4 kHz\n");
    std::fprintf(f,
                 "t_s,setpoint,response,estimate,roll_deg,pitch_deg,yaw_deg,est_roll_deg,"
                 "est_pitch_deg,est_yaw_deg,p_dps,q_dps,r_dps,m0,m1,m2,m3,altitude_m\n");
    for (std::size_t k = 1; k < r.samples.size(); k += 2) {
        const Sample& s = r.samples[k];
        std::fprintf(f,
                     "%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.4f,%.4f,"
                     "%.4f,%.4f,%.4f\n",
                     s.t, s.setpoint, s.response, s.estimate, s.roll_deg, s.pitch_deg,
                     s.yaw_deg, s.est_roll_deg, s.est_pitch_deg, s.est_yaw_deg, s.p_dps,
                     s.q_dps, s.r_dps, s.motor[0], s.motor[1], s.motor[2], s.motor[3],
                     s.altitude_m);
    }
    return std::fclose(f) == 0;
}

namespace {

std::string num(double v) {
    if (!std::isfinite(v)) {
        return "null";
    }
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.6g", v);
    return buf;
}

std::string gains(const PidGains& g) {
    return "[" + num(static_cast<double>(g.kp)) + ", " + num(static_cast<double>(g.ki)) + ", " +
           num(static_cast<double>(g.kd)) + "]";
}

}  // namespace

std::string metrics_json(const std::vector<RunResult>& runs, const FlightControllerConfig& cfg,
                         const QuadParams& p) {
    std::ostringstream o;
    o << "{\n";
    o << "  \"simulated\": true,\n";
    o << "  \"note\": \"Software-in-the-loop against an assumed plant. Not hardware data.\",\n";
    o << "  \"gains\": {\"rate_roll\": " << gains(cfg.rate_roll)
      << ", \"rate_pitch\": " << gains(cfg.rate_pitch) << ", \"rate_yaw\": " << gains(cfg.rate_yaw)
      << ", \"angle_p\": " << num(static_cast<double>(cfg.angle_p)) << "},\n";
    o << "  \"plant\": {\"mass_kg\": " << num(p.mass_kg) << ", \"arm_m\": " << num(p.arm_m)
      << ", \"ixx\": " << num(p.ixx) << ", \"iyy\": " << num(p.iyy) << ", \"izz\": " << num(p.izz)
      << ", \"max_thrust_n\": " << num(p.max_thrust_n)
      << ", \"motor_tau_s\": " << num(p.motor_tau_s) << ", \"hover_command\": "
      << num(p.hover_command()) << "},\n";
    o << "  \"scenarios\": [\n";
    for (std::size_t i = 0; i < runs.size(); ++i) {
        const auto& r = runs[i];
        const auto& m = r.metrics;
        const StepLimits l = limits_for(r.scenario);
        o << "    {\"name\": \"" << r.scenario.name << "\", \"armed\": "
          << (r.armed ? "true" : "false") << ", \"amplitude\": " << num(m.amplitude)
          << ", \"rise_time_s\": " << num(m.rise_time_s)
          << ", \"overshoot_pct\": " << num(m.overshoot_pct)
          << ", \"settling_time_s\": " << num(m.settling_time_s)
          << ", \"steady_state_error_pct\": " << num(m.steady_state_error_pct)
          << ", \"settled\": " << (m.settled ? "true" : "false")
          << ", \"bounded\": " << (m.bounded ? "true" : "false")
          << ", \"judged_on\": \"" << (judged_on_estimate(r.scenario) ? "estimate" : "response")
          << "\", \"max_estimator_error\": " << num(m.max_estimator_error)
          << ", \"final_true_value\": " << num(m.final_true_value)
          << ", \"hover_motor_activity\": " << num(m.hover_motor_activity)
          << ", \"limits\": {\"max_overshoot_pct\": " << num(l.max_overshoot_pct)
          << ", \"max_settling_time_s\": " << num(l.max_settling_time_s)
          << ", \"max_steady_state_error_pct\": " << num(l.max_steady_state_error_pct) << "}"
          << ", \"pass\": " << (within_limits(m, l) ? "true" : "false") << "}"
          << (i + 1 < runs.size() ? "," : "") << "\n";
    }
    o << "  ]\n}\n";
    return o.str();
}

}  // namespace sil
}  // namespace fc
