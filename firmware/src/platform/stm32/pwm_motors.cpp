#include "pwm_motors.hpp"

#include "gpio.hpp"
#include "registers.hpp"

namespace fc {
namespace stm32 {
namespace {

// PWM mode 1 with preload enabled, for the two channels in each CCMR register.
constexpr uint32_t kOcPwmMode1Low = (0x6u << 4) | (1u << 3);    // OC1M | OC1PE
constexpr uint32_t kOcPwmMode1High = (0x6u << 12) | (1u << 11);  // OC2M | OC2PE

// TIM1 and TIM8 sit on APB2. With APB2 at 84 MHz the timer clock doubles to
// 168 MHz, so a prescaler of 168 gives a 1 us tick.
constexpr uint32_t kTimerClockHz = 168000000u;
constexpr uint32_t kPeriodTicks = 2500u;  // 400 Hz frame

void configure_timer(TimRegs* t) {
    t->CR1 = 0;
    t->PSC = (kTimerClockHz / 1000000u) - 1u;
    t->ARR = kPeriodTicks - 1u;

    // Both channels of each register in PWM mode 1 with preload.
    t->CCMR1 = kOcPwmMode1Low | kOcPwmMode1High;
    t->CCMR2 = kOcPwmMode1Low | kOcPwmMode1High;

    // Park at minimum throttle before the outputs are enabled, so enabling the
    // timer can never emit a high pulse.
    t->CCR1 = 1000u;
    t->CCR2 = 1000u;
    t->CCR3 = 1000u;
    t->CCR4 = 1000u;

    t->EGR = tim_egr::kUg;                 // load the preloaded values now
    set_bits(t->BDTR, tim_bdtr::kMoe);     // advanced timers need main output enable
    t->CR1 = tim_cr1::kArpe | tim_cr1::kCen;
}

}  // namespace

void PwmMotorOutput::begin() {
    set_bits(rcc()->APB2ENR, rcc_apb2enr::kTim1 | rcc_apb2enr::kTim8);

    auto* porta = gpio(kGpioABase);
    auto* portc = gpio(kGpioCBase);

    // Drive the pins low before switching them to the timer, so no ESC sees a
    // floating or high line while the timer is being set up.
    configure_output(porta, 8);
    configure_output(porta, 9);
    configure_output(portc, 8);
    configure_output(portc, 9);
    porta->BSRR = (1u << (8 + 16)) | (1u << (9 + 16));
    portc->BSRR = (1u << (8 + 16)) | (1u << (9 + 16));

    configure_timer(tim(kTim1Base));
    configure_timer(tim(kTim8Base));

    // Enable the compare outputs.
    set_bits(tim(kTim1Base)->CCER, (1u << 0) | (1u << 4));   // CC1E, CC2E
    set_bits(tim(kTim8Base)->CCER, (1u << 8) | (1u << 12));  // CC3E, CC4E

    // Hand the pins to the timers.
    configure_alternate(porta, 8, 1);   // TIM1_CH1
    configure_alternate(porta, 9, 1);   // TIM1_CH2
    configure_alternate(portc, 8, 3);   // TIM8_CH3
    configure_alternate(portc, 9, 3);   // TIM8_CH4
}

uint32_t PwmMotorOutput::pulse_for(float throttle) {
    if (!(throttle > 0.0f)) {
        return kMinPulseUs;
    }
    if (throttle >= 1.0f) {
        return kMaxPulseUs;
    }
    const float span = static_cast<float>(kMaxPulseUs - kMinPulseUs);
    return kMinPulseUs + static_cast<uint32_t>(throttle * span + 0.5f);
}

void PwmMotorOutput::set_channel(std::size_t index, uint32_t pulse_us) {
    switch (index) {
        case 0:
            tim(kTim1Base)->CCR2 = pulse_us;  // /S1 PA9
            break;
        case 1:
            tim(kTim1Base)->CCR1 = pulse_us;  // /S2 PA8
            break;
        case 2:
            tim(kTim8Base)->CCR4 = pulse_us;  // /S3 PC9
            break;
        case 3:
            tim(kTim8Base)->CCR3 = pulse_us;  // /S4 PC8
            break;
        default:
            break;
    }
}

void PwmMotorOutput::write(const float throttles[kMotorCount]) {
    for (std::size_t i = 0; i < kMotorCount; ++i) {
        set_channel(i, pulse_for(throttles[i]));
    }
}

void PwmMotorOutput::disarm() {
    for (std::size_t i = 0; i < kMotorCount; ++i) {
        set_channel(i, kMinPulseUs);
    }
}

}  // namespace stm32
}  // namespace fc
