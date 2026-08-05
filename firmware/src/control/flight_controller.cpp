#include "fc/control/flight_controller.hpp"

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

}  // namespace

void FlightController::apply_gains() {
    pid_roll_.set_gains(cfg_.rate_roll);
    pid_roll_.set_limits(cfg_.rate_limits);
    pid_pitch_.set_gains(cfg_.rate_pitch);
    pid_pitch_.set_limits(cfg_.rate_limits);
    pid_yaw_.set_gains(cfg_.rate_yaw);
    pid_yaw_.set_limits(cfg_.rate_limits);
    estimator_.set_config(cfg_.estimator);
    arming_.set_config(cfg_.arming);
}

void FlightController::set_config(const FlightControllerConfig& cfg) {
    cfg_ = cfg;
    apply_gains();
}

bool FlightController::init(const Icm42688Config& imu_cfg) {
    apply_gains();
    imu_healthy_ = imu_.init(imu_cfg);
    return imu_healthy_;
}

bool FlightController::calibrate_gyro(std::size_t samples) {
    if (!imu_healthy_ || samples == 0) {
        return false;
    }

    imu_.set_gyro_bias(Vec3{});

    Vec3 sum{};
    Vec3 min_seen{};
    Vec3 max_seen{};

    for (std::size_t i = 0; i < samples; ++i) {
        ImuSample s;
        if (!imu_.read(s)) {
            return false;
        }
        sum += s.gyro;
        if (i == 0) {
            min_seen = s.gyro;
            max_seen = s.gyro;
        } else {
            min_seen.x = std::fmin(min_seen.x, s.gyro.x);
            min_seen.y = std::fmin(min_seen.y, s.gyro.y);
            min_seen.z = std::fmin(min_seen.z, s.gyro.z);
            max_seen.x = std::fmax(max_seen.x, s.gyro.x);
            max_seen.y = std::fmax(max_seen.y, s.gyro.y);
            max_seen.z = std::fmax(max_seen.z, s.gyro.z);
        }
        clock_.delay_us(1000);
    }

    // If the spread is large the airframe was moving; a bias captured then
    // would be baked into every subsequent reading.
    const Vec3 spread = max_seen - min_seen;
    constexpr float kMaxSpreadRadS = 0.15f;
    if (spread.x > kMaxSpreadRadS || spread.y > kMaxSpreadRadS ||
        spread.z > kMaxSpreadRadS) {
        return false;
    }

    const float inv = 1.0f / static_cast<float>(samples);
    imu_.set_gyro_bias(sum * inv);

    // Start the estimator from the measured orientation rather than identity.
    ImuSample s;
    if (imu_.read(s)) {
        estimator_.align_to_accel(s.accel);
    }

    calibrated_ = true;
    return true;
}

void FlightController::on_rc_frame(const crsf::RcChannels& channels, uint32_t now_us) {
    channels_ = channels;
    last_rx_us_ = now_us;
    rx_ever_received_ = true;
}

void FlightController::stop_motors() {
    for (std::size_t i = 0; i < kMotorCount; ++i) {
        last_output_.motor[i] = 0.0f;
    }
    motors_.disarm();
    pid_roll_.reset();
    pid_pitch_.reset();
    pid_yaw_.reset();
}

void FlightController::update(uint32_t now_us) {
    ++loop_count_;

    // --- timebase -------------------------------------------------------
    float dt = 0.0f;
    if (have_last_update_) {
        dt = static_cast<float>(elapsed_us(now_us, last_update_us_)) * 1e-6f;
    }
    last_update_us_ = now_us;
    have_last_update_ = true;

    // --- sensors --------------------------------------------------------
    ImuSample sample;
    const bool read_ok = imu_.read(sample);
    if (read_ok) {
        sample_ = sample;
        estimator_.update(sample_.gyro, sample_.accel, dt);
    } else {
        imu_healthy_ = false;
    }

    // Tilt from vertical: angle between the body down axis and earth down.
    const Vec3 body_down_in_earth = estimator_.attitude().rotate(Vec3{0.0f, 0.0f, 1.0f});
    tilt_rad_ = std::acos(clampf(body_down_in_earth.z, -1.0f, 1.0f));

    // --- supervisor -----------------------------------------------------
    ArmingInputs arm_in;
    arm_in.arm_switch =
        rx_ever_received_ && crsf::is_switch_high(channels_.channel[rc::kArmSwitch]);
    arm_in.throttle =
        rx_ever_received_ ? crsf::to_unipolar(channels_.channel[rc::kThrottle]) : 0.0f;
    arm_in.last_rx_us = last_rx_us_;
    arm_in.rx_ever_received = rx_ever_received_;
    arm_in.imu_healthy = imu_healthy_;
    arm_in.calibrated = calibrated_;
    arm_in.battery_voltage = battery_voltage_;
    arm_in.tilt_rad = tilt_rad_;

    arming_.update(arm_in, now_us);

    if (!arming_.motors_enabled()) {
        stop_motors();
        return;
    }

    // --- setpoints ------------------------------------------------------
    const float roll_stick = crsf::to_bipolar(channels_.channel[rc::kRoll]);
    const float pitch_stick = crsf::to_bipolar(channels_.channel[rc::kPitch]);
    const float yaw_stick = crsf::to_bipolar(channels_.channel[rc::kYaw]);
    const float throttle = crsf::to_unipolar(channels_.channel[rc::kThrottle]);

    float roll_rate_sp = 0.0f;
    float pitch_rate_sp = 0.0f;

    if (cfg_.mode == FlightMode::Angle) {
        // Outer loop: stick sets a target lean angle, error becomes a rate.
        const EulerAngles att = estimator_.euler();
        const float roll_target = roll_stick * cfg_.max_angle_rad;
        const float pitch_target = pitch_stick * cfg_.max_angle_rad;
        roll_rate_sp = (roll_target - att.roll) * cfg_.angle_p;
        pitch_rate_sp = (pitch_target - att.pitch) * cfg_.angle_p;
        roll_rate_sp = clampf(roll_rate_sp, -cfg_.max_rate_rad_s, cfg_.max_rate_rad_s);
        pitch_rate_sp = clampf(pitch_rate_sp, -cfg_.max_rate_rad_s, cfg_.max_rate_rad_s);
    } else {
        roll_rate_sp = roll_stick * cfg_.max_rate_rad_s;
        pitch_rate_sp = pitch_stick * cfg_.max_rate_rad_s;
    }
    const float yaw_rate_sp = yaw_stick * cfg_.max_yaw_rate_rad_s;

    // --- inner rate loop ------------------------------------------------
    const float roll_out = pid_roll_.update(roll_rate_sp, sample_.gyro.x, dt);
    const float pitch_out = pid_pitch_.update(pitch_rate_sp, sample_.gyro.y, dt);
    const float yaw_out = pid_yaw_.update(yaw_rate_sp, sample_.gyro.z, dt);

    // --- mix and output -------------------------------------------------
    MixerCommand cmd;
    cmd.throttle = throttle;
    cmd.roll = roll_out * cfg_.output_scale;
    cmd.pitch = pitch_out * cfg_.output_scale;
    cmd.yaw = yaw_out * cfg_.output_scale;

    last_output_ = mix_quad_x(cmd);
    motors_.write(last_output_.motor);
}

}  // namespace fc
