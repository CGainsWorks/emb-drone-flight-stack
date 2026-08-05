#pragma once

#include <cstdint>

#include "fc/hal/motor_output.hpp"

namespace fc {
namespace stm32 {

// Conventional servo-style PWM ESC output, 1000-2000 us at 400 Hz.
//
// Motor index -> board net -> pin -> timer channel:
//   0  /S1  PA9  TIM1_CH2 (AF1)
//   1  /S2  PA8  TIM1_CH1 (AF1)
//   2  /S3  PC9  TIM8_CH4 (AF3)
//   3  /S4  PC8  TIM8_CH3 (AF3)
//
// PWM is deliberately the first output mode brought up: it needs no DMA, so
// there is far less to go wrong on a board that has never been powered. The
// DShot encoder (drivers/dshot.cpp) is written and unit-tested, and moving to
// it is a matter of wiring the timer DMA requests once PWM is proven.
class PwmMotorOutput : public MotorOutput {
public:
    void begin();

    void write(const float throttles[kMotorCount]) override;
    void disarm() override;

private:
    static constexpr uint32_t kTimerClockHz = 168000000u;
    static constexpr uint32_t kTickHz = 1000000u;      // 1 us per tick
    static constexpr uint32_t kPeriodTicks = 2500u;    // 400 Hz
    static constexpr uint32_t kMinPulseUs = 1000u;
    static constexpr uint32_t kMaxPulseUs = 2000u;

    static uint32_t pulse_for(float throttle);
    void set_channel(std::size_t index, uint32_t pulse_us);
};

}  // namespace stm32
}  // namespace fc
