#pragma once

#include "fc/drivers/bitbang_spi.hpp"

namespace fc {
namespace stm32 {

// The board's IMU has to be driven in software (see docs/PINOUT.md: PA6 and PA7
// carry the IMU's data lines the wrong way round for the F405's fixed SPI1
// alternate functions, and the F405 has no swap bit).
//
// This subclass only adds the inter-edge delay. At 168 MHz a GPIO BSRR store is
// a handful of cycles, so without any padding the clock would run well above
// the ICM-42688-P's 24 MHz limit for register access. The NOP count below is a
// starting point and MUST be checked with a scope during bring-up: the real
// half-period depends on flash wait states and bus contention, neither of which
// can be predicted reliably from source.
class BitBangSpiStm32 : public BitBangSpi {
public:
    using BitBangSpi::BitBangSpi;

protected:
    void delay_half_bit() override {
        for (int i = 0; i < kNopsPerHalfBit; ++i) {
            __asm volatile("nop");
        }
    }

private:
    static constexpr int kNopsPerHalfBit = 8;
};

}  // namespace stm32
}  // namespace fc
