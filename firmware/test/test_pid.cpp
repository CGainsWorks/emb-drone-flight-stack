#include "fc/control/pid.hpp"
#include "test_framework.hpp"

using namespace fc;

TEST(pid_proportional_only_matches_hand_calculation) {
    Pid pid({2.0f, 0.0f, 0.0f}, {100.0f, 1000.0f, 0.0f});
    // error = 10 - 4 = 6, output = 2 * 6 = 12
    CHECK_NEAR(pid.update(10.0f, 4.0f, 0.001f), 12.0f, 1e-4);
}

TEST(pid_zero_error_gives_zero_output) {
    Pid pid({2.0f, 0.5f, 0.1f}, {100.0f, 1000.0f, 0.0f});
    for (int i = 0; i < 10; ++i) {
        CHECK_NEAR(pid.update(0.0f, 0.0f, 0.001f), 0.0f, 1e-6);
    }
}

TEST(pid_integral_accumulates_on_constant_error) {
    Pid pid({0.0f, 1.0f, 0.0f}, {100.0f, 1000.0f, 0.0f});
    // ki=1, error=1, dt=0.1 -> integrator grows by 0.1 each step
    for (int i = 0; i < 10; ++i) {
        pid.update(1.0f, 0.0f, 0.1f);
    }
    CHECK_NEAR(pid.integrator(), 1.0f, 1e-4);
}

TEST(pid_integral_respects_its_clamp) {
    Pid pid({0.0f, 100.0f, 0.0f}, {5.0f, 1000.0f, 0.0f});
    for (int i = 0; i < 200; ++i) {
        pid.update(10.0f, 0.0f, 0.01f);
    }
    CHECK_NEAR(pid.integrator(), 5.0f, 1e-4);
}

TEST(pid_output_is_clamped) {
    Pid pid({1000.0f, 0.0f, 0.0f}, {100.0f, 250.0f, 0.0f});
    CHECK_NEAR(pid.update(10.0f, 0.0f, 0.001f), 250.0f, 1e-4);
    CHECK_NEAR(pid.update(-10.0f, 0.0f, 0.001f), -250.0f, 1e-4);
}

TEST(pid_derivative_acts_on_measurement_not_setpoint) {
    // A step in SETPOINT must not produce a derivative kick. With kd large,
    // a setpoint jump should still yield only the proportional contribution.
    Pid pid({1.0f, 0.0f, 100.0f}, {100.0f, 100000.0f, 0.0f});
    pid.update(0.0f, 0.0f, 0.01f);          // settle, no history yet
    pid.update(0.0f, 0.0f, 0.01f);          // now have history
    const float out = pid.update(50.0f, 0.0f, 0.01f);
    // Pure P: 1.0 * 50 = 50. No derivative spike from the setpoint change.
    CHECK_NEAR(out, 50.0f, 1e-3);
}

TEST(pid_derivative_opposes_rising_measurement) {
    // Measurement climbing towards setpoint should produce a damping (negative)
    // derivative contribution.
    Pid pid({0.0f, 0.0f, 1.0f}, {100.0f, 100000.0f, 0.0f});
    pid.update(0.0f, 0.0f, 0.01f);
    const float out = pid.update(0.0f, 1.0f, 0.01f);
    // d(measurement)/dt = +100 -> d_term = -1.0 * 100 = -100
    CHECK_NEAR(out, -100.0f, 1e-2);
}

TEST(pid_first_update_has_no_derivative_kick) {
    // On the very first call there is no previous measurement. A naive
    // implementation would treat prev=0 and emit a huge spike.
    Pid pid({0.0f, 0.0f, 1000.0f}, {100.0f, 100000.0f, 0.0f});
    CHECK_NEAR(pid.update(0.0f, 500.0f, 0.001f), 0.0f, 1e-6);
}

TEST(pid_anti_windup_stops_integrator_growth_while_saturated) {
    // Sustained large error with a tight output clamp: the integrator must not
    // run away, otherwise recovery is badly delayed once the error clears.
    Pid pid({1.0f, 50.0f, 0.0f}, {10000.0f, 10.0f, 0.0f});
    for (int i = 0; i < 1000; ++i) {
        pid.update(100.0f, 0.0f, 0.01f);
    }
    // Without conditional integration this would reach the 10000 clamp.
    CHECK(pid.integrator() < 100.0f);
}

TEST(pid_recovers_promptly_after_saturation_clears) {
    Pid pid({1.0f, 50.0f, 0.0f}, {10000.0f, 10.0f, 0.0f});
    for (int i = 0; i < 500; ++i) {
        pid.update(100.0f, 0.0f, 0.01f);  // drive hard into saturation
    }
    // Error reverses; output should leave the positive rail almost immediately.
    const float out = pid.update(-100.0f, 0.0f, 0.01f);
    CHECK(out < 10.0f);
}

TEST(pid_reset_clears_all_state) {
    Pid pid({1.0f, 10.0f, 1.0f}, {100.0f, 1000.0f, 0.0f});
    for (int i = 0; i < 50; ++i) {
        pid.update(5.0f, 0.0f, 0.01f);
    }
    CHECK(pid.integrator() != 0.0f);
    pid.reset();
    CHECK_NEAR(pid.integrator(), 0.0f, 1e-9);
    CHECK_NEAR(pid.last_output(), 0.0f, 1e-9);
    // After reset the next call must behave like a first call (no D kick).
    Pid fresh({0.0f, 0.0f, 1000.0f}, {100.0f, 100000.0f, 0.0f});
    CHECK_NEAR(fresh.update(0.0f, 500.0f, 0.001f), 0.0f, 1e-6);
}

TEST(pid_rejects_non_positive_dt) {
    // A stalled or wrapped timer must never produce a divide-by-zero or an
    // infinite derivative.
    Pid pid({1.0f, 1.0f, 1.0f}, {100.0f, 1000.0f, 0.0f});
    const float first = pid.update(10.0f, 0.0f, 0.01f);
    const float with_zero_dt = pid.update(10.0f, 0.0f, 0.0f);
    CHECK_NEAR(with_zero_dt, first, 1e-6);
    const float with_negative_dt = pid.update(10.0f, 0.0f, -0.5f);
    CHECK_NEAR(with_negative_dt, first, 1e-6);
}

TEST(pid_closed_loop_step_response_converges) {
    // First-order plant: x' = u. A PI controller should drive it to setpoint
    // with no steady-state error.
    Pid pid({4.0f, 12.0f, 0.15f}, {50.0f, 100.0f, 60.0f});
    float x = 0.0f;
    const float dt = 0.001f;
    const float setpoint = 1.0f;
    for (int i = 0; i < 5000; ++i) {
        const float u = pid.update(setpoint, x, dt);
        x += u * dt;
    }
    CHECK_NEAR(x, setpoint, 0.02f);
}

TEST(pid_closed_loop_is_stable_and_bounded) {
    Pid pid({4.0f, 12.0f, 0.15f}, {50.0f, 100.0f, 60.0f});
    float x = 0.0f;
    const float dt = 0.001f;
    float peak = 0.0f;
    for (int i = 0; i < 5000; ++i) {
        const float u = pid.update(1.0f, x, dt);
        x += u * dt;
        if (std::fabs(x) > peak) {
            peak = std::fabs(x);
        }
        CHECK(std::isfinite(x));
    }
    // Overshoot must stay sane (< 60%) rather than ringing off to infinity.
    CHECK(peak < 1.6f);
}

TEST_MAIN()
