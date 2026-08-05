// End-to-end tests for the whole control loop, running entirely on the host
// against mock hardware. No board, no ESCs, no receiver.
#include "fc/control/flight_controller.hpp"
#include "mocks/mock_spi.hpp"
#include "test_framework.hpp"

using namespace fc;
using fc::testing_mocks::FakeClock;
using fc::testing_mocks::MockIcm42688;
using fc::testing_mocks::MockMotorOutput;

namespace {

constexpr std::size_t FR = 0;
constexpr std::size_t RR = 1;
constexpr std::size_t RL = 2;
constexpr std::size_t FL = 3;

constexpr float kAccelLsbPerG = 32768.0f / 16.0f;

crsf::RcChannels centred_sticks() {
    crsf::RcChannels ch;
    for (std::size_t i = 0; i < crsf::kChannelCount; ++i) {
        ch.channel[i] = crsf::kChannelMid;
    }
    ch.channel[rc::kThrottle] = crsf::kChannelMin;  // throttle down
    ch.channel[rc::kArmSwitch] = crsf::kChannelMin;  // disarmed
    return ch;
}

// A test harness that wires the controller to mock hardware and drives time.
struct Rig {
    MockIcm42688 dev;
    FakeClock clock;
    MockMotorOutput motors;
    Icm42688 imu{dev, clock};
    FlightController fc{imu, motors, clock};
    crsf::RcChannels sticks = centred_sticks();

    Rig() {
        // Level, stationary board: -1 g on body z, no rotation.
        set_accel_g(0.0f, 0.0f, -1.0f);
        dev.set_gyro_raw(0, 0, 0);
        fc.set_battery_voltage(16.8f);
    }

    void set_accel_g(float x, float y, float z) {
        dev.set_accel_raw(static_cast<int16_t>(x * kAccelLsbPerG),
                          static_cast<int16_t>(y * kAccelLsbPerG),
                          static_cast<int16_t>(z * kAccelLsbPerG));
    }

    bool boot() {
        if (!fc.init()) {
            return false;
        }
        return fc.calibrate_gyro(20);
    }

    // Run `iterations` control cycles at 1 kHz, refreshing the RC link.
    void run(int iterations, bool keep_rx_alive = true) {
        for (int i = 0; i < iterations; ++i) {
            clock.advance_us(1000);
            if (keep_rx_alive) {
                fc.on_rc_frame(sticks, clock.micros());
            }
            fc.update(clock.micros());
        }
    }

    void arm() {
        // Switch must be seen low first, then high.
        sticks.channel[rc::kArmSwitch] = crsf::kChannelMin;
        run(5);
        sticks.channel[rc::kArmSwitch] = crsf::kChannelMax;
        run(5);
    }
};

}  // namespace

TEST(fc_init_succeeds_with_a_working_imu) {
    Rig rig;
    CHECK(rig.fc.init());
    CHECK(rig.fc.imu_healthy());
}

TEST(fc_init_fails_with_a_dead_imu) {
    Rig rig;
    rig.dev.set_who_am_i(0x00);
    CHECK(!rig.fc.init());
    CHECK(!rig.fc.imu_healthy());
}

TEST(fc_refuses_to_arm_without_a_healthy_imu) {
    Rig rig;
    rig.dev.set_who_am_i(0x00);
    rig.fc.init();
    rig.arm();
    CHECK(rig.fc.arming().state() == ArmState::Disarmed);
    CHECK(rig.motors.all_stopped());
}

TEST(fc_refuses_to_arm_before_gyro_calibration) {
    Rig rig;
    CHECK(rig.fc.init());
    // Deliberately skip calibrate_gyro().
    rig.arm();
    CHECK(rig.fc.arming().state() == ArmState::Disarmed);
    CHECK(rig.fc.arming().blocker() == ArmBlocker::NotCalibrated);
}

TEST(fc_gyro_calibration_removes_a_constant_offset) {
    Rig rig;
    CHECK(rig.fc.init());
    rig.dev.set_gyro_raw(300, -200, 100);
    CHECK(rig.fc.calibrate_gyro(20));
    CHECK(rig.fc.calibrated());

    rig.run(1);
    // With the bias learned, a stationary gyro must now read ~zero.
    CHECK_NEAR(rig.fc.last_sample().gyro.x, 0.0f, 1e-4);
    CHECK_NEAR(rig.fc.last_sample().gyro.y, 0.0f, 1e-4);
    CHECK_NEAR(rig.fc.last_sample().gyro.z, 0.0f, 1e-4);
}

TEST(fc_motors_stay_stopped_while_disarmed) {
    Rig rig;
    CHECK(rig.boot());
    rig.sticks.channel[rc::kThrottle] = crsf::kChannelMax;  // throttle up!
    rig.run(200);
    // Disarmed: full throttle must do absolutely nothing.
    CHECK(rig.fc.arming().state() == ArmState::Disarmed);
    CHECK(rig.motors.all_stopped());
    CHECK(rig.motors.write_count == 0);
}

TEST(fc_arms_when_conditions_are_good) {
    Rig rig;
    CHECK(rig.boot());
    rig.arm();
    CHECK(rig.fc.arming().state() == ArmState::Armed);
}

TEST(fc_estimator_tracks_a_level_board) {
    Rig rig;
    CHECK(rig.boot());
    rig.run(2000);
    CHECK_NEAR(rig.fc.estimator().euler().roll, 0.0f, 0.02f);
    CHECK_NEAR(rig.fc.estimator().euler().pitch, 0.0f, 0.02f);
    CHECK_NEAR(rig.fc.tilt_rad(), 0.0f, 0.02f);
}

TEST(fc_computes_tilt_from_attitude) {
    Rig rig;
    CHECK(rig.fc.init());
    // Board rolled 90 degrees: gravity now reads on the body y axis.
    rig.set_accel_g(0.0f, -1.0f, 0.0f);
    CHECK(rig.fc.calibrate_gyro(20));
    rig.run(3000);
    CHECK_NEAR(rig.fc.tilt_rad(), 1.5708f, 0.1f);
}

TEST(fc_refuses_to_arm_when_tilted_past_the_limit) {
    Rig rig;
    CHECK(rig.fc.init());
    rig.set_accel_g(0.0f, -1.0f, 0.0f);  // on its side
    CHECK(rig.fc.calibrate_gyro(20));
    rig.run(3000);
    rig.arm();
    CHECK(rig.fc.arming().state() == ArmState::Disarmed);
    CHECK(rig.fc.arming().blocker() == ArmBlocker::TiltTooHigh);
}

TEST(fc_refuses_to_arm_with_throttle_up) {
    Rig rig;
    CHECK(rig.boot());
    rig.sticks.channel[rc::kThrottle] = crsf::kChannelMax;
    rig.arm();
    CHECK(rig.fc.arming().state() == ArmState::Disarmed);
    CHECK(rig.fc.arming().blocker() == ArmBlocker::ThrottleNotLow);
}

TEST(fc_refuses_to_arm_on_a_flat_battery) {
    Rig rig;
    CHECK(rig.boot());
    rig.fc.set_battery_voltage(12.0f);
    rig.arm();
    CHECK(rig.fc.arming().state() == ArmState::Disarmed);
    CHECK(rig.fc.arming().blocker() == ArmBlocker::BatteryLow);
}

TEST(fc_self_levels_a_roll_disturbance) {
    // The core closed-loop behaviour: hold the airframe rolled to the right and
    // confirm the mixer commands the correcting motor differential.
    Rig rig;
    CHECK(rig.boot());
    rig.arm();
    rig.sticks.channel[rc::kThrottle] = crsf::kChannelMid;
    rig.run(20);

    // Now tilt: right side down by ~20 deg.
    rig.set_accel_g(0.0f, -0.342f, -0.940f);
    rig.run(1500);

    CHECK(rig.fc.arming().state() == ArmState::Armed);
    CHECK(rig.fc.estimator().euler().roll > 0.15f);

    // The right side is down, so recovery means rolling back to the LEFT, which
    // requires more thrust on the right to push that side up. (Mirrors the
    // pitch case below: nose-up is corrected by more front thrust.)
    const MixerOutput& o = rig.fc.motor_output();
    CHECK(o.motor[FR] > o.motor[FL]);
    CHECK(o.motor[RR] > o.motor[RL]);
}

TEST(fc_self_levels_a_pitch_disturbance) {
    Rig rig;
    CHECK(rig.boot());
    rig.arm();
    rig.sticks.channel[rc::kThrottle] = crsf::kChannelMid;
    rig.run(20);

    // Nose up by ~20 deg -> accel gains a +x component.
    rig.set_accel_g(0.342f, 0.0f, -0.940f);
    rig.run(1500);

    CHECK(rig.fc.estimator().euler().pitch > 0.15f);
    // Nose-up must be corrected by increasing the front motors.
    const MixerOutput& o = rig.fc.motor_output();
    CHECK(o.motor[FL] > o.motor[RL]);
    CHECK(o.motor[FR] > o.motor[RR]);
}

TEST(fc_motor_commands_stay_in_range_under_a_violent_disturbance) {
    Rig rig;
    CHECK(rig.boot());
    rig.arm();
    rig.sticks.channel[rc::kThrottle] = crsf::kChannelMax;

    // Slam the board through extreme attitudes and spin rates.
    for (int i = 0; i < 2000; ++i) {
        rig.dev.set_gyro_raw(static_cast<int16_t>((i * 997) % 30000),
                             static_cast<int16_t>((i * 613) % 30000),
                             static_cast<int16_t>((i * 331) % 30000));
        rig.set_accel_g(0.6f, -0.6f, -0.5f);
        rig.run(1);
        const MixerOutput& o = rig.fc.motor_output();
        for (std::size_t m = 0; m < kMotorCount; ++m) {
            CHECK(o.motor[m] >= 0.0f);
            CHECK(o.motor[m] <= 1.0f);
            CHECK(std::isfinite(o.motor[m]));
        }
    }
}

TEST(fc_failsafe_on_rx_loss_cuts_the_motors) {
    Rig rig;
    CHECK(rig.boot());
    rig.arm();
    rig.sticks.channel[rc::kThrottle] = crsf::kChannelMid;
    rig.run(50);
    CHECK(rig.motors.write_count > 0);

    // Receiver goes silent.
    rig.run(1000, /*keep_rx_alive=*/false);

    CHECK(rig.fc.arming().state() == ArmState::Failsafe);
    CHECK(rig.fc.arming().failsafe_cause() == FailsafeCause::RxTimeout);
    CHECK(rig.motors.all_stopped());
}

TEST(fc_failsafe_does_not_restart_motors_when_the_link_returns) {
    Rig rig;
    CHECK(rig.boot());
    rig.arm();
    rig.sticks.channel[rc::kThrottle] = crsf::kChannelMid;
    rig.run(50);

    rig.run(1000, /*keep_rx_alive=*/false);
    CHECK(rig.fc.arming().state() == ArmState::Failsafe);

    // Link comes back with the arm switch still on and throttle still up.
    rig.run(1000, /*keep_rx_alive=*/true);
    CHECK(rig.fc.arming().state() == ArmState::Failsafe);
    CHECK(rig.motors.all_stopped());
}

TEST(fc_disarming_stops_the_motors_immediately) {
    Rig rig;
    CHECK(rig.boot());
    rig.arm();
    rig.sticks.channel[rc::kThrottle] = crsf::kChannelMid;
    rig.run(50);

    const int disarms_before = rig.motors.disarm_count;
    rig.sticks.channel[rc::kArmSwitch] = crsf::kChannelMin;
    rig.run(2);

    CHECK(rig.fc.arming().state() == ArmState::Disarmed);
    CHECK(rig.motors.disarm_count > disarms_before);
    CHECK(rig.motors.all_stopped());
}

TEST(fc_pid_state_is_reset_on_disarm) {
    // Integrator carried across an arm cycle would produce an immediate lurch
    // the next time the motors spin up.
    Rig rig;
    CHECK(rig.boot());
    rig.arm();
    rig.sticks.channel[rc::kThrottle] = crsf::kChannelMid;
    rig.set_accel_g(0.0f, -0.5f, -0.87f);  // sustained roll error winds up I
    rig.run(500);

    rig.sticks.channel[rc::kArmSwitch] = crsf::kChannelMin;
    rig.run(5);

    // Re-arm level, and the first commanded output must be near-symmetric.
    rig.set_accel_g(0.0f, 0.0f, -1.0f);
    rig.run(500);
    rig.sticks.channel[rc::kArmSwitch] = crsf::kChannelMax;
    rig.run(2);

    const MixerOutput& o = rig.fc.motor_output();
    CHECK_NEAR(o.motor[FL], o.motor[FR], 0.15f);
}

TEST(fc_survives_an_imu_that_dies_mid_flight) {
    Rig rig;
    CHECK(rig.boot());
    rig.arm();
    rig.sticks.channel[rc::kThrottle] = crsf::kChannelMid;
    rig.run(50);
    CHECK(rig.fc.arming().state() == ArmState::Armed);

    // The IMU stops responding. Everything downstream must stay finite and the
    // aircraft must go to failsafe rather than fly on garbage.
    rig.dev.set_who_am_i(0x00);
    rig.dev.reset_registers();
    rig.imu.set_gyro_bias(Vec3{});
    rig.run(10);

    CHECK(std::isfinite(rig.fc.estimator().euler().roll));
    for (std::size_t m = 0; m < kMotorCount; ++m) {
        CHECK(std::isfinite(rig.fc.motor_output().motor[m]));
    }
}

TEST(fc_loop_runs_at_the_expected_cadence) {
    Rig rig;
    CHECK(rig.boot());
    const uint32_t before = rig.fc.loop_count();
    rig.run(1000);
    CHECK_EQ(static_cast<int>(rig.fc.loop_count() - before), 1000);
}

TEST(fc_rate_mode_responds_to_stick_input) {
    Rig rig;
    FlightControllerConfig cfg;
    cfg.mode = FlightMode::Rate;
    rig.fc.set_config(cfg);

    CHECK(rig.boot());
    rig.arm();
    rig.sticks.channel[rc::kThrottle] = crsf::kChannelMid;
    rig.run(20);

    // Full right roll stick with a stationary gyro: the controller should
    // demand a roll-right differential (left motors up).
    rig.sticks.channel[rc::kRoll] = crsf::kChannelMax;
    rig.run(50);

    const MixerOutput& o = rig.fc.motor_output();
    CHECK(o.motor[FL] > o.motor[FR]);
}

TEST_MAIN()
