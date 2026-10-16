#pragma once

// Software-in-the-loop harness: the unmodified FlightController, IMU driver,
// PID, mixer and arming logic, closed around the QuadPlant through the same HAL
// mocks the unit tests use (register-level ICM-42688 on SpiBus, FakeClock,
// MockMotorOutput). Nothing here runs on, or says anything measured about,
// real hardware: every output is simulated.

#include <cstdint>
#include <string>
#include <vector>

#include "fc/control/flight_controller.hpp"
#include "quad_plant.hpp"

namespace fc {
namespace sil {

enum class Axis : uint8_t { Roll, Pitch, Yaw };

struct Scenario {
    std::string name;
    FlightMode mode{FlightMode::Angle};
    Axis axis{Axis::Roll};
    float stick{0.5f};          // -1..1 deflection applied at step_time
    double step_time_s{0.5};    // after release into hover
    double duration_s{2.5};     // total recorded time from release
};

// Rate steps are judged on the true body rate. Angle steps are judged on the
// firmware's own attitude estimate, because that is the quantity the angle
// loop controls; the true angle is recorded alongside and the gap between the
// two is reported as estimator error, not hidden.
inline bool judged_on_estimate(const Scenario& s) {
    return s.mode == FlightMode::Angle && s.axis != Axis::Yaw;
}

// The standard set used by CI, the committed results and tools/pid_tuner.py.
std::vector<Scenario> standard_scenarios();

struct Sample {
    double t;
    double setpoint;          // deg for angle steps, deg/s for rate steps
    double response;          // true plant value, same units
    double estimate;          // what the firmware measures or believes, same units
    double roll_deg, pitch_deg, yaw_deg;          // true attitude
    double est_roll_deg, est_pitch_deg, est_yaw_deg;
    double p_dps, q_dps, r_dps;                   // true body rates
    double motor[4];
    double altitude_m;
};

struct StepMetrics {
    double amplitude{0.0};          // commanded change, response units
    double rise_time_s{0.0};        // 10 % -> 90 %
    double overshoot_pct{0.0};
    double settling_time_s{0.0};    // to within +/-5 % of the step, inf if never
    double steady_state_error_pct{0.0};
    bool settled{false};
    bool bounded{false};            // finite, attitude within 80 deg, rates sane
    double max_estimator_error{0.0};  // |true - estimate| after the step
    double final_true_value{0.0};     // true response at the end of the record
    double hover_motor_activity{0.0};  // mean |change| of motor command per cycle
};

// Acceptance bounds for the CI stability gate. Shared with the tuner through
// the metrics JSON so the two can never disagree.
struct StepLimits {
    double max_overshoot_pct;
    double max_settling_time_s;
    double max_steady_state_error_pct;
};

StepLimits limits_for(const Scenario& s);
bool within_limits(const StepMetrics& m, const StepLimits& l);

struct SensorNoise {
    double gyro_sigma_rad_s{0.0};   // white, per sample
    double accel_sigma_g{0.0};
    uint32_t seed{1};
};

// Assumed noise for the standard runs. Order of magnitude only, not taken
// from a characterised part.
SensorNoise default_noise();

struct RunResult {
    Scenario scenario;
    std::vector<Sample> samples;
    StepMetrics metrics;
    bool armed{false};
};

RunResult run_scenario(const Scenario& s, const FlightControllerConfig& cfg,
                       const QuadParams& plant = QuadParams{},
                       const SensorNoise& noise = default_noise());

StepMetrics compute_metrics(const std::vector<Sample>& samples, double step_time_s,
                            bool use_estimate);

// Output helpers. Paths are whatever the caller passes; the CLI passes
// repository-relative ones.
bool write_csv(const RunResult& r, const std::string& path);
std::string metrics_json(const std::vector<RunResult>& runs, const FlightControllerConfig& cfg,
                         const QuadParams& plant);

}  // namespace sil
}  // namespace fc
