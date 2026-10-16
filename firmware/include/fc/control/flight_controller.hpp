#pragma once

#include "fc/control/attitude_estimator.hpp"
#include "fc/control/mixer.hpp"
#include "fc/control/pid.hpp"
#include "fc/drivers/crsf.hpp"
#include "fc/drivers/icm42688.hpp"
#include "fc/hal/clock.hpp"
#include "fc/hal/motor_output.hpp"
#include "fc/state/arming.hpp"

namespace fc {

// RC channel assignment (AETR, the CRSF/ExpressLRS default).
namespace rc {
inline constexpr std::size_t kRoll = 0;
inline constexpr std::size_t kPitch = 1;
inline constexpr std::size_t kThrottle = 2;
inline constexpr std::size_t kYaw = 3;
inline constexpr std::size_t kArmSwitch = 4;
}  // namespace rc

enum class FlightMode : uint8_t {
    // Sticks command angular rate. The estimator is not in the loop.
    Rate = 0,
    // Sticks command a lean angle; an outer loop turns angle error into a rate
    // setpoint for the inner loop. Self-levelling.
    Angle = 1,
};

struct FlightControllerConfig {
    FlightMode mode{FlightMode::Angle};

    // Gains below were tuned in software-in-the-loop simulation only
    // (tools/pid_tuner.py against the assumed airframe in firmware/sim/sil,
    // nominal and perturbed), then the yaw P term was eased by hand for
    // overshoot margin. They have never driven a motor; treat them as a
    // starting point for bench tuning, not as flight-proven values.

    // Inner rate-loop gains (per axis).
    PidGains rate_roll{100.0f, 90.0f, 1.35f};
    PidGains rate_pitch{100.0f, 90.0f, 1.35f};
    PidGains rate_yaw{130.0f, 33.0f, 0.0f};
    PidLimits rate_limits{150.0f, 500.0f, 80.0f};

    // Outer angle-loop gain: rad of error -> rad/s of demanded rate.
    float angle_p{19.5f};

    // Stick scaling.
    float max_rate_rad_s{6.0f};      // full stick in Rate mode
    float max_angle_rad{0.61f};      // ~35 deg, full stick in Angle mode
    float max_yaw_rate_rad_s{4.0f};

    // Normalisation applied to PID output before mixing.
    float output_scale{0.002f};

    ArmingConfig arming{};
    AttitudeEstimatorConfig estimator{};
};

// Ties the pieces together into one control iteration.
//
// This class holds no hardware knowledge of its own: it talks to the IMU
// driver, a MotorOutput and a Clock, all of which are interfaces. That means
// the entire loop, including arming and failsafe behaviour, is exercised in
// host tests with mock hardware.
class FlightController {
public:
    FlightController(Icm42688& imu, MotorOutput& motors, Clock& clock)
        : imu_(imu), motors_(motors), clock_(clock) {}

    void set_config(const FlightControllerConfig& cfg);
    const FlightControllerConfig& config() const { return cfg_; }

    // Configures the IMU. Returns false if the IMU did not respond, in which
    // case the controller will refuse to arm.
    bool init(const Icm42688Config& imu_cfg = Icm42688Config{});

    // Average `samples` gyro readings to find the stationary bias. The
    // airframe must be still. Returns false if the readings are too noisy to
    // trust, which usually means it was moved during calibration.
    bool calibrate_gyro(std::size_t samples = 500);

    // Feed a decoded RC frame.
    void on_rc_frame(const crsf::RcChannels& channels, uint32_t now_us);

    // Run one control iteration.
    void update(uint32_t now_us);

    // --- observation, for telemetry and tests ---
    const AttitudeEstimator& estimator() const { return estimator_; }
    const ArmingStateMachine& arming() const { return arming_; }
    const MixerOutput& motor_output() const { return last_output_; }
    const ImuSample& last_sample() const { return sample_; }
    float tilt_rad() const { return tilt_rad_; }
    bool imu_healthy() const { return imu_healthy_; }
    bool calibrated() const { return calibrated_; }
    uint32_t loop_count() const { return loop_count_; }

    void set_battery_voltage(float v) { battery_voltage_ = v; }

private:
    Icm42688& imu_;
    MotorOutput& motors_;
    Clock& clock_;

    FlightControllerConfig cfg_{};
    AttitudeEstimator estimator_{};
    ArmingStateMachine arming_{};

    Pid pid_roll_{};
    Pid pid_pitch_{};
    Pid pid_yaw_{};

    ImuSample sample_{};
    MixerOutput last_output_{};

    crsf::RcChannels channels_{};
    bool rx_ever_received_{false};
    uint32_t last_rx_us_{0};

    float battery_voltage_{0.0f};
    float tilt_rad_{0.0f};
    bool imu_healthy_{false};
    bool calibrated_{false};
    bool have_last_update_{false};
    uint32_t last_update_us_{0};
    uint32_t loop_count_{0};

    void apply_gains();
    void stop_motors();
};

}  // namespace fc
