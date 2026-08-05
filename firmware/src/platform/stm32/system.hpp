#pragma once

#include <cstdint>

#include "fc/hal/clock.hpp"

namespace fc {
namespace stm32 {

// Brings the MCU from the 16 MHz HSI reset default up to 168 MHz using the
// 8.000 MHz HSE crystal fitted on FCSTMV1.05 (FY0800027).
//
//   VCO in  = 8 MHz / PLLM(8)   = 1 MHz
//   VCO out = 1 MHz * PLLN(336) = 336 MHz
//   SYSCLK  = 336 MHz / PLLP(2) = 168 MHz
//   USB     = 336 MHz / PLLQ(7) = 48 MHz
//
// AHB = 168 MHz, APB1 = 42 MHz, APB2 = 84 MHz.
// Returns false if the HSE or the PLL fails to lock, in which case the caller
// must not proceed: every timing constant downstream assumes 168 MHz.
bool configure_clocks();

// Starts TIM2 as a free-running 32-bit microsecond counter.
//
// TIM2 is one of the F4's 32-bit timers, so it maps exactly onto the 32-bit
// micros() contract used by the portable code (which already handles wrap with
// unsigned subtraction). A 16-bit timer would wrap every 65 ms and break the
// failsafe timeouts.
void start_microsecond_timer();

class Stm32Clock : public Clock {
public:
    uint32_t micros() const override;
    void delay_us(uint32_t us) override;
};

}  // namespace stm32
}  // namespace fc
