#pragma once

#include <cstdint>

#include "fc/hal/gpio.hpp"
#include "registers.hpp"

namespace fc {
namespace stm32 {

enum class Pull : uint32_t { None = 0, Up = 1, Down = 2 };
enum class Speed : uint32_t { Low = 0, Medium = 1, High = 2, VeryHigh = 3 };

void enable_gpio_clocks();

void configure_output(GpioRegs* port, uint8_t pin, Speed speed = Speed::High);
void configure_input(GpioRegs* port, uint8_t pin, Pull pull = Pull::None);
void configure_alternate(GpioRegs* port, uint8_t pin, uint8_t af,
                         Speed speed = Speed::High);

// A single pin driven through BSRR.
//
// BSRR is used rather than a read-modify-write of ODR because it is a single
// atomic store: an interrupt landing mid-update cannot corrupt a neighbouring
// pin. That matters here because the bit-banged SPI clock and the motor outputs
// share GPIO banks.
class Stm32GpioPin : public GpioPin {
public:
    Stm32GpioPin(GpioRegs* port, uint8_t pin) : port_(port), pin_(pin) {}

    void write(bool high) override {
        port_->BSRR = high ? (1u << pin_) : (1u << (pin_ + 16u));
    }

    bool read() const override { return ((port_->IDR >> pin_) & 1u) != 0u; }

private:
    GpioRegs* port_;
    uint8_t pin_;
};

}  // namespace stm32
}  // namespace fc
