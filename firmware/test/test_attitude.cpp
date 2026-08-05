#include "fc/control/attitude_estimator.hpp"
#include "test_framework.hpp"

using namespace fc;

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float deg(float d) { return d * kPi / 180.0f; }

// What a real accelerometer reads (in g) when the airframe is at attitude
// q_true and not accelerating: the specific force is the reaction to gravity,
// expressed in the body frame.
Vec3 simulated_accel(const Quaternion& q_true) {
    return q_true.inverse_rotate(Vec3{0.0f, 0.0f, -1.0f});
}

AttitudeEstimatorConfig fast_config() {
    AttitudeEstimatorConfig cfg;
    cfg.kp = 5.0f;
    cfg.ki = 0.0f;  // isolate the proportional correction
    return cfg;
}

AttitudeEstimatorConfig integrate_only_config() {
    AttitudeEstimatorConfig cfg;
    cfg.kp = 0.0f;
    cfg.ki = 0.0f;
    return cfg;
}

}  // namespace

TEST(estimator_starts_level) {
    AttitudeEstimator est;
    const EulerAngles e = est.euler();
    CHECK_NEAR(e.roll, 0.0f, 1e-6);
    CHECK_NEAR(e.pitch, 0.0f, 1e-6);
    CHECK_NEAR(e.yaw, 0.0f, 1e-6);
}

TEST(estimator_integrates_roll_rate_from_gyro) {
    // Pure integration, accelerometer correction disabled.
    AttitudeEstimator est{integrate_only_config()};
    const float dt = 0.001f;
    const float rate = 1.0f;  // rad/s about body x
    for (int i = 0; i < 500; ++i) {
        est.update(Vec3{rate, 0.0f, 0.0f}, Vec3{0.0f, 0.0f, 0.0f}, dt);
    }
    CHECK_NEAR(est.euler().roll, 0.5f, 1e-3);
}

TEST(estimator_integrates_yaw_rate_from_gyro) {
    AttitudeEstimator est{integrate_only_config()};
    const float dt = 0.001f;
    for (int i = 0; i < 500; ++i) {
        est.update(Vec3{0.0f, 0.0f, 2.0f}, Vec3{0.0f, 0.0f, 0.0f}, dt);
    }
    CHECK_NEAR(est.euler().yaw, 1.0f, 1e-3);
}

TEST(estimator_converges_to_roll_from_accelerometer) {
    // The estimator starts level but the board is actually rolled 20 deg.
    // With a stationary accelerometer it must converge to the truth.
    const Quaternion truth = Quaternion::from_euler(deg(20.0f), 0.0f, 0.0f);
    const Vec3 accel = simulated_accel(truth);

    AttitudeEstimator est{fast_config()};
    for (int i = 0; i < 20000; ++i) {
        est.update(Vec3{0.0f, 0.0f, 0.0f}, accel, 0.001f);
    }
    CHECK_NEAR(est.euler().roll, deg(20.0f), deg(0.5f));
    CHECK_NEAR(est.euler().pitch, 0.0f, deg(0.5f));
}

TEST(estimator_converges_to_pitch_from_accelerometer) {
    const Quaternion truth = Quaternion::from_euler(0.0f, deg(-15.0f), 0.0f);
    const Vec3 accel = simulated_accel(truth);

    AttitudeEstimator est{fast_config()};
    for (int i = 0; i < 20000; ++i) {
        est.update(Vec3{0.0f, 0.0f, 0.0f}, accel, 0.001f);
    }
    CHECK_NEAR(est.euler().pitch, deg(-15.0f), deg(0.5f));
    CHECK_NEAR(est.euler().roll, 0.0f, deg(0.5f));
}

TEST(estimator_converges_on_combined_roll_and_pitch) {
    const Quaternion truth = Quaternion::from_euler(deg(25.0f), deg(18.0f), 0.0f);
    const Vec3 accel = simulated_accel(truth);

    AttitudeEstimator est{fast_config()};
    for (int i = 0; i < 30000; ++i) {
        est.update(Vec3{0.0f, 0.0f, 0.0f}, accel, 0.001f);
    }
    CHECK_NEAR(est.euler().roll, deg(25.0f), deg(1.0f));
    CHECK_NEAR(est.euler().pitch, deg(18.0f), deg(1.0f));
}

TEST(estimator_holds_level_when_already_correct) {
    // Starting at truth, it must not wander.
    AttitudeEstimator est{fast_config()};
    const Vec3 accel{0.0f, 0.0f, -1.0f};
    for (int i = 0; i < 5000; ++i) {
        est.update(Vec3{0.0f, 0.0f, 0.0f}, accel, 0.001f);
    }
    CHECK_NEAR(est.euler().roll, 0.0f, deg(0.05f));
    CHECK_NEAR(est.euler().pitch, 0.0f, deg(0.05f));
}

TEST(estimator_rejects_accelerometer_under_high_g) {
    // 2 g reading is not gravity: it must be ignored, leaving the estimate on
    // gyro integration alone.
    AttitudeEstimator est{fast_config()};
    const Vec3 high_g{0.0f, -2.0f, 0.0f};
    for (int i = 0; i < 1000; ++i) {
        est.update(Vec3{0.0f, 0.0f, 0.0f}, high_g, 0.001f);
    }
    CHECK(!est.accel_used());
    CHECK_NEAR(est.euler().roll, 0.0f, deg(0.01f));
}

TEST(estimator_rejects_accelerometer_in_free_fall) {
    AttitudeEstimator est{fast_config()};
    for (int i = 0; i < 1000; ++i) {
        est.update(Vec3{0.0f, 0.0f, 0.0f}, Vec3{0.0f, 0.0f, 0.0f}, 0.001f);
    }
    CHECK(!est.accel_used());
    CHECK(est.attitude().is_finite());
    CHECK_NEAR(est.euler().roll, 0.0f, 1e-4);
}

TEST(estimator_uses_accelerometer_at_one_g) {
    AttitudeEstimator est{fast_config()};
    est.update(Vec3{0.0f, 0.0f, 0.0f}, Vec3{0.0f, 0.0f, -1.0f}, 0.001f);
    CHECK(est.accel_used());
}

TEST(estimator_estimates_and_cancels_constant_gyro_bias) {
    // A real gyro has a bias. With ki enabled the filter should learn it, so
    // the attitude stays level instead of slowly rolling away.
    AttitudeEstimatorConfig cfg;
    cfg.kp = 2.0f;
    cfg.ki = 1.0f;
    AttitudeEstimator est{cfg};

    const Vec3 bias{0.02f, -0.015f, 0.0f};  // rad/s
    const Vec3 accel{0.0f, 0.0f, -1.0f};    // board is genuinely level

    for (int i = 0; i < 120000; ++i) {
        est.update(bias, accel, 0.001f);  // gyro reports only its bias
    }

    // Attitude must remain level despite the biased gyro.
    CHECK_NEAR(est.euler().roll, 0.0f, deg(1.0f));
    CHECK_NEAR(est.euler().pitch, 0.0f, deg(1.0f));

    // And the learned bias should oppose the injected one.
    CHECK_NEAR(est.gyro_bias().x, -bias.x, 0.01f);
    CHECK_NEAR(est.gyro_bias().y, -bias.y, 0.01f);
}

TEST(estimator_bias_is_clamped) {
    AttitudeEstimatorConfig cfg;
    cfg.kp = 1.0f;
    cfg.ki = 50.0f;
    cfg.max_gyro_bias = 0.1f;
    AttitudeEstimator est{cfg};

    const Quaternion truth = Quaternion::from_euler(deg(60.0f), 0.0f, 0.0f);
    const Vec3 accel = simulated_accel(truth);
    for (int i = 0; i < 20000; ++i) {
        est.update(Vec3{0.0f, 0.0f, 0.0f}, accel, 0.001f);
    }
    CHECK(std::fabs(est.gyro_bias().x) <= 0.1f + 1e-6f);
    CHECK(std::fabs(est.gyro_bias().y) <= 0.1f + 1e-6f);
    CHECK(std::fabs(est.gyro_bias().z) <= 0.1f + 1e-6f);
}

TEST(estimator_align_to_accel_snaps_immediately) {
    // Bring-up convenience: no waiting for convergence at power-on.
    const Quaternion truth = Quaternion::from_euler(deg(30.0f), deg(-20.0f), 0.0f);
    AttitudeEstimator est;
    est.align_to_accel(simulated_accel(truth));
    CHECK_NEAR(est.euler().roll, deg(30.0f), deg(0.5f));
    CHECK_NEAR(est.euler().pitch, deg(-20.0f), deg(0.5f));
}

TEST(estimator_align_ignores_garbage_reading) {
    AttitudeEstimator est;
    est.align_to_accel(Vec3{0.0f, 0.0f, 0.0f});
    CHECK(est.attitude().is_finite());
    CHECK_NEAR(est.euler().roll, 0.0f, 1e-6);
}

TEST(estimator_survives_nan_input) {
    // A failed SPI read must not be able to destroy the attitude estimate.
    AttitudeEstimator est{fast_config()};
    for (int i = 0; i < 100; ++i) {
        est.update(Vec3{0.0f, 0.0f, 0.0f}, Vec3{0.0f, 0.0f, -1.0f}, 0.001f);
    }
    const float nan_v = std::nan("");
    est.update(Vec3{nan_v, nan_v, nan_v}, Vec3{0.0f, 0.0f, -1.0f}, 0.001f);
    est.update(Vec3{0.0f, 0.0f, 0.0f}, Vec3{nan_v, 0.0f, -1.0f}, 0.001f);
    CHECK(est.attitude().is_finite());
    CHECK(std::isfinite(est.euler().roll));
}

TEST(estimator_ignores_non_positive_dt) {
    AttitudeEstimator est{fast_config()};
    est.update(Vec3{1.0f, 0.0f, 0.0f}, Vec3{0.0f, 0.0f, -1.0f}, 0.0f);
    CHECK_NEAR(est.euler().roll, 0.0f, 1e-9);
    est.update(Vec3{1.0f, 0.0f, 0.0f}, Vec3{0.0f, 0.0f, -1.0f}, -0.01f);
    CHECK_NEAR(est.euler().roll, 0.0f, 1e-9);
}

TEST(estimator_quaternion_stays_normalised) {
    AttitudeEstimator est{fast_config()};
    for (int i = 0; i < 10000; ++i) {
        est.update(Vec3{0.7f, -0.4f, 1.1f}, Vec3{0.1f, 0.2f, -0.97f}, 0.001f);
    }
    CHECK_NEAR(est.attitude().norm(), 1.0f, 1e-4);
}

TEST(estimator_tracks_a_slow_roll_sweep) {
    // Rotate the truth slowly and feed consistent gyro + accel. The estimate
    // should follow rather than lag badly or diverge.
    AttitudeEstimatorConfig cfg;
    cfg.kp = 2.0f;
    cfg.ki = 0.0f;
    AttitudeEstimator est{cfg};

    const float dt = 0.001f;
    const float rate = 0.2f;  // rad/s
    float truth_roll = 0.0f;
    for (int i = 0; i < 3000; ++i) {
        truth_roll += rate * dt;
        const Quaternion truth = Quaternion::from_euler(truth_roll, 0.0f, 0.0f);
        est.update(Vec3{rate, 0.0f, 0.0f}, simulated_accel(truth), dt);
    }
    CHECK_NEAR(est.euler().roll, truth_roll, deg(1.0f));
}

TEST_MAIN()
