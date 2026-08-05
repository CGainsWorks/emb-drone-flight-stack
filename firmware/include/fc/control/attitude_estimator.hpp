#pragma once

#include "fc/math/quaternion.hpp"
#include "fc/math/vec3.hpp"

namespace fc {

struct AttitudeEstimatorConfig {
    // Proportional gain: how hard the accelerometer pulls the estimate back
    // towards gravity. Higher = faster convergence, more vibration sensitivity.
    float kp{1.0f};
    // Integral gain: estimates and cancels constant gyro bias. Set to 0 to
    // disable bias tracking.
    float ki{0.05f};
    // The accelerometer is only trusted as a gravity reference when its
    // magnitude is close to 1 g. Under acceleration or heavy vibration the
    // reading is not gravity and must be ignored, or the estimate is dragged
    // off true.
    float accel_trust_min_g{0.75f};
    float accel_trust_max_g{1.25f};
    // Safety rail on the estimated bias (rad/s).
    float max_gyro_bias{0.35f};
};

// Mahony-style complementary filter.
//
// Gyro integration provides the high-frequency attitude; the accelerometer
// provides a low-frequency absolute reference for roll and pitch. Yaw is NOT
// observable from gravity alone, so yaw here is dead-reckoned from the gyro
// and will drift. (The board's MMC5983MA magnetometer sits on a separate I2C
// bus in the GPS power island and is not wired to the F405, so there is no
// heading reference available to this MCU.)
class AttitudeEstimator {
public:
    AttitudeEstimator() = default;
    explicit AttitudeEstimator(const AttitudeEstimatorConfig& cfg) : cfg_(cfg) {}

    void set_config(const AttitudeEstimatorConfig& cfg) { cfg_ = cfg; }
    const AttitudeEstimatorConfig& config() const { return cfg_; }

    // gyro in rad/s (body frame), accel in g (body frame), dt in seconds.
    void update(const Vec3& gyro, const Vec3& accel, float dt);

    // Initialise attitude directly from an accelerometer reading, so the
    // estimator starts level-correct instead of converging from identity.
    void align_to_accel(const Vec3& accel);

    void reset();

    const Quaternion& attitude() const { return q_; }
    EulerAngles euler() const { return q_.to_euler(); }
    const Vec3& gyro_bias() const { return bias_; }

    // True if the most recent update actually used the accelerometer.
    bool accel_used() const { return accel_used_; }

private:
    AttitudeEstimatorConfig cfg_{};
    Quaternion q_{Quaternion::identity()};
    Vec3 bias_{};
    bool accel_used_{false};
};

}  // namespace fc
