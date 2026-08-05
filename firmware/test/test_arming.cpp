#include "fc/state/arming.hpp"
#include "test_framework.hpp"

using namespace fc;

namespace {

// A set of inputs that satisfies every arming precondition.
ArmingInputs good_inputs(uint32_t now_us) {
    ArmingInputs in;
    in.arm_switch = false;
    in.throttle = 0.0f;
    in.last_rx_us = now_us;
    in.rx_ever_received = true;
    in.imu_healthy = true;
    in.calibrated = true;
    in.battery_voltage = 16.8f;
    in.tilt_rad = 0.0f;
    return in;
}

// Drive the machine to Armed, returning the time after arming.
uint32_t arm_it(ArmingStateMachine& sm, uint32_t t) {
    ArmingInputs in = good_inputs(t);
    in.arm_switch = false;
    sm.update(in, t);  // observe switch off -> cycle satisfied

    t += 1000;
    in = good_inputs(t);
    in.arm_switch = true;
    sm.update(in, t);
    return t;
}

}  // namespace

TEST(arming_starts_disarmed_with_motors_off) {
    ArmingStateMachine sm;
    CHECK(sm.state() == ArmState::Disarmed);
    CHECK(!sm.motors_enabled());
}

TEST(arming_succeeds_when_all_conditions_are_met) {
    ArmingStateMachine sm;
    arm_it(sm, 1000000u);
    CHECK(sm.state() == ArmState::Armed);
    CHECK(sm.motors_enabled());
    CHECK(sm.blocker() == ArmBlocker::None);
}

TEST(arming_refused_if_switch_was_already_on_at_boot) {
    // Critical safety property: powering up with the arm switch on must not
    // spin the motors. The switch has to be cycled off first.
    ArmingStateMachine sm;
    const uint32_t t = 1000000u;
    ArmingInputs in = good_inputs(t);
    in.arm_switch = true;

    for (int i = 0; i < 100; ++i) {
        sm.update(in, t + static_cast<uint32_t>(i) * 1000u);
    }
    CHECK(sm.state() == ArmState::Disarmed);
    CHECK(!sm.motors_enabled());
    CHECK(sm.blocker() == ArmBlocker::SwitchNotCycled);

    // Cycle the switch off, then on: now it may arm.
    in.arm_switch = false;
    sm.update(in, t + 200000u);
    in.arm_switch = true;
    sm.update(in, t + 201000u);
    CHECK(sm.state() == ArmState::Armed);
}

TEST(arming_refused_with_throttle_up) {
    ArmingStateMachine sm;
    const uint32_t t = 1000000u;
    ArmingInputs in = good_inputs(t);
    sm.update(in, t);  // cycle switch

    in.arm_switch = true;
    in.throttle = 0.5f;
    sm.update(in, t + 1000u);
    CHECK(sm.state() == ArmState::Disarmed);
    CHECK(sm.blocker() == ArmBlocker::ThrottleNotLow);
}

TEST(arming_refused_without_rx_link) {
    ArmingStateMachine sm;
    const uint32_t t = 1000000u;
    ArmingInputs in = good_inputs(t);
    in.rx_ever_received = false;
    sm.update(in, t);

    in.arm_switch = true;
    sm.update(in, t + 1000u);
    CHECK(sm.state() == ArmState::Disarmed);
    CHECK(sm.blocker() == ArmBlocker::NoRxLink);
}

TEST(arming_refused_with_stale_rx_link) {
    ArmingStateMachine sm;
    const uint32_t t = 1000000u;
    ArmingInputs in = good_inputs(t);
    sm.update(in, t);

    in.arm_switch = true;
    in.last_rx_us = t;
    // 600 ms later with no new frame: beyond the 500 ms timeout.
    sm.update(in, t + 600000u);
    CHECK(sm.state() == ArmState::Disarmed);
    CHECK(sm.blocker() == ArmBlocker::NoRxLink);
}

TEST(arming_refused_with_unhealthy_imu) {
    ArmingStateMachine sm;
    const uint32_t t = 1000000u;
    ArmingInputs in = good_inputs(t);
    in.imu_healthy = false;
    sm.update(in, t);
    in.arm_switch = true;
    sm.update(in, t + 1000u);
    CHECK(sm.state() == ArmState::Disarmed);
    CHECK(sm.blocker() == ArmBlocker::ImuUnhealthy);
}

TEST(arming_refused_when_not_calibrated) {
    ArmingStateMachine sm;
    const uint32_t t = 1000000u;
    ArmingInputs in = good_inputs(t);
    in.calibrated = false;
    sm.update(in, t);
    in.arm_switch = true;
    sm.update(in, t + 1000u);
    CHECK(sm.state() == ArmState::Disarmed);
    CHECK(sm.blocker() == ArmBlocker::NotCalibrated);
}

TEST(arming_refused_when_tilted_too_far) {
    ArmingStateMachine sm;
    const uint32_t t = 1000000u;
    ArmingInputs in = good_inputs(t);
    in.tilt_rad = 1.2f;  // ~69 deg, well past the limit
    sm.update(in, t);
    in.arm_switch = true;
    sm.update(in, t + 1000u);
    CHECK(sm.state() == ArmState::Disarmed);
    CHECK(sm.blocker() == ArmBlocker::TiltTooHigh);

    // Negative tilt must be treated the same.
    in.tilt_rad = -1.2f;
    sm.update(in, t + 2000u);
    CHECK(sm.state() == ArmState::Disarmed);
    CHECK(sm.blocker() == ArmBlocker::TiltTooHigh);
}

TEST(arming_refused_on_a_flat_battery) {
    ArmingStateMachine sm;
    const uint32_t t = 1000000u;
    ArmingInputs in = good_inputs(t);
    in.battery_voltage = 13.0f;
    sm.update(in, t);
    in.arm_switch = true;
    sm.update(in, t + 1000u);
    CHECK(sm.state() == ArmState::Disarmed);
    CHECK(sm.blocker() == ArmBlocker::BatteryLow);
}

TEST(disarm_on_switch_off) {
    ArmingStateMachine sm;
    uint32_t t = arm_it(sm, 1000000u);
    CHECK(sm.state() == ArmState::Armed);

    t += 1000;
    ArmingInputs in = good_inputs(t);
    in.arm_switch = false;
    sm.update(in, t);
    CHECK(sm.state() == ArmState::Disarmed);
    CHECK(!sm.motors_enabled());
}

TEST(failsafe_on_rx_timeout) {
    ArmingStateMachine sm;
    uint32_t t = arm_it(sm, 1000000u);

    ArmingInputs in = good_inputs(t);
    in.arm_switch = true;
    in.last_rx_us = t;  // link dies here

    // Still fine just under the timeout.
    sm.update(in, t + 400000u);
    CHECK(sm.state() == ArmState::Armed);

    // Past the timeout: failsafe, motors cut.
    sm.update(in, t + 600000u);
    CHECK(sm.state() == ArmState::Failsafe);
    CHECK(!sm.motors_enabled());
    CHECK(sm.failsafe_cause() == FailsafeCause::RxTimeout);
}

TEST(failsafe_on_imu_failure) {
    ArmingStateMachine sm;
    const uint32_t t = arm_it(sm, 1000000u);

    ArmingInputs in = good_inputs(t + 1000u);
    in.arm_switch = true;
    in.imu_healthy = false;
    sm.update(in, t + 1000u);

    CHECK(sm.state() == ArmState::Failsafe);
    CHECK(sm.failsafe_cause() == FailsafeCause::ImuFailure);
    CHECK(!sm.motors_enabled());
}

TEST(brief_voltage_sag_does_not_trigger_failsafe) {
    // A throttle punch sags the pack momentarily. Cutting the motors for that
    // would itself cause a crash.
    ArmingStateMachine sm;
    const uint32_t t = arm_it(sm, 1000000u);

    ArmingInputs in = good_inputs(t);
    in.arm_switch = true;

    for (uint32_t dt = 0; dt < 1000000u; dt += 10000u) {
        in.last_rx_us = t + dt;
        in.battery_voltage = 12.5f;  // below critical, but only briefly
        sm.update(in, t + dt);
    }
    // 1 s of sag is under the 2 s debounce.
    CHECK(sm.state() == ArmState::Armed);

    // Recovery clears the condition.
    in.battery_voltage = 16.0f;
    in.last_rx_us = t + 1010000u;
    sm.update(in, t + 1010000u);
    CHECK(sm.state() == ArmState::Armed);
}

TEST(sustained_low_voltage_triggers_failsafe) {
    ArmingStateMachine sm;
    const uint32_t t = arm_it(sm, 1000000u);

    ArmingInputs in = good_inputs(t);
    in.arm_switch = true;

    for (uint32_t dt = 0; dt <= 3000000u; dt += 10000u) {
        in.last_rx_us = t + dt;
        in.battery_voltage = 12.5f;
        sm.update(in, t + dt);
    }
    CHECK(sm.state() == ArmState::Failsafe);
    CHECK(sm.failsafe_cause() == FailsafeCause::BatteryCritical);
}

TEST(failsafe_does_not_auto_rearm_when_link_returns) {
    // The most important property here. A link that flickers back must not
    // restart the motors without the pilot explicitly disarming first.
    ArmingStateMachine sm;
    const uint32_t t = arm_it(sm, 1000000u);

    ArmingInputs in = good_inputs(t);
    in.arm_switch = true;
    in.last_rx_us = t;
    sm.update(in, t + 600000u);
    CHECK(sm.state() == ArmState::Failsafe);

    // Link comes back, switch still on.
    for (uint32_t dt = 700000u; dt < 3000000u; dt += 10000u) {
        in.last_rx_us = t + dt;
        sm.update(in, t + dt);
        CHECK(sm.state() == ArmState::Failsafe);
        CHECK(!sm.motors_enabled());
    }
}

TEST(failsafe_clears_only_via_disarm_then_requires_rearm) {
    ArmingStateMachine sm;
    const uint32_t t = arm_it(sm, 1000000u);

    ArmingInputs in = good_inputs(t);
    in.arm_switch = true;
    in.last_rx_us = t;
    sm.update(in, t + 600000u);
    CHECK(sm.state() == ArmState::Failsafe);

    // Disarm.
    in.arm_switch = false;
    in.last_rx_us = t + 700000u;
    sm.update(in, t + 700000u);
    CHECK(sm.state() == ArmState::Disarmed);
    CHECK(sm.failsafe_cause() == FailsafeCause::None);

    // And it can arm again afterwards.
    in.arm_switch = true;
    in.last_rx_us = t + 710000u;
    sm.update(in, t + 710000u);
    CHECK(sm.state() == ArmState::Armed);
}

TEST(motors_are_only_ever_enabled_in_the_armed_state) {
    // Exhaustive sweep over the input space: assert the invariant that
    // motors_enabled() implies state == Armed, and that arming never happens
    // while any precondition is violated.
    for (int mask = 0; mask < 128; ++mask) {
        ArmingStateMachine sm;
        const uint32_t t = 1000000u;

        ArmingInputs in = good_inputs(t);
        in.throttle = (mask & 1) ? 0.8f : 0.0f;
        in.rx_ever_received = (mask & 2) == 0;
        in.imu_healthy = (mask & 4) == 0;
        in.calibrated = (mask & 8) == 0;
        in.tilt_rad = (mask & 16) ? 1.5f : 0.0f;
        in.battery_voltage = (mask & 32) ? 11.0f : 16.8f;
        const bool switch_on_at_boot = (mask & 64) != 0;

        in.arm_switch = switch_on_at_boot;
        sm.update(in, t);
        in.arm_switch = true;
        sm.update(in, t + 1000u);

        CHECK(sm.motors_enabled() == (sm.state() == ArmState::Armed));

        const bool everything_ok = (mask & 63) == 0;
        if (!everything_ok) {
            CHECK(!sm.motors_enabled());
            CHECK(sm.blocker() != ArmBlocker::None);
        }
    }
}

TEST(rx_timeout_is_correct_across_a_timer_wrap) {
    // micros() is a 32-bit counter and wraps roughly every 71 minutes. A naive
    // signed comparison would report a stale link forever after the wrap, or
    // worse, mask a genuine loss of signal.
    ArmingStateMachine sm;

    const uint32_t before_wrap = 0xFFFFFF00u;
    ArmingInputs in = good_inputs(before_wrap);
    in.arm_switch = false;
    sm.update(in, before_wrap);
    in.arm_switch = true;
    in.last_rx_us = before_wrap;
    sm.update(in, before_wrap);
    CHECK(sm.state() == ArmState::Armed);

    // Time wraps past zero; only 0x200 us have actually elapsed.
    const uint32_t after_wrap = 0x00000100u;
    in.last_rx_us = before_wrap;
    sm.update(in, after_wrap);
    CHECK(sm.state() == ArmState::Armed);

    // Now genuinely exceed the timeout across the wrap.
    sm.update(in, before_wrap + 700000u);
    CHECK(sm.state() == ArmState::Failsafe);
    CHECK(sm.failsafe_cause() == FailsafeCause::RxTimeout);
}

TEST(reset_returns_to_a_safe_initial_state) {
    ArmingStateMachine sm;
    arm_it(sm, 1000000u);
    CHECK(sm.state() == ArmState::Armed);

    sm.reset();
    CHECK(sm.state() == ArmState::Disarmed);
    CHECK(!sm.motors_enabled());
    CHECK(sm.blocker() == ArmBlocker::SwitchNotCycled);
}

TEST(state_names_are_reported_for_telemetry) {
    CHECK(std::string(to_string(ArmState::Armed)) == "ARMED");
    CHECK(std::string(to_string(ArmBlocker::ThrottleNotLow)) == "THROTTLE_NOT_LOW");
    CHECK(std::string(to_string(FailsafeCause::RxTimeout)) == "RX_TIMEOUT");
}

TEST_MAIN()
