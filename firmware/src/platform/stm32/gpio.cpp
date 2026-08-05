#include "gpio.hpp"

namespace fc {
namespace stm32 {
namespace {

void set_field(volatile uint32_t& reg, uint8_t pin, uint32_t bits_per_pin,
               uint32_t value) {
    const uint32_t shift = static_cast<uint32_t>(pin) * bits_per_pin;
    const uint32_t mask = ((1u << bits_per_pin) - 1u) << shift;
    reg = (reg & ~mask) | ((value << shift) & mask);
}

}  // namespace

void enable_gpio_clocks() {
    set_bits(rcc()->AHB1ENR, rcc_ahb1enr::kGpioA | rcc_ahb1enr::kGpioB |
                                 rcc_ahb1enr::kGpioC | rcc_ahb1enr::kGpioD |
                                 rcc_ahb1enr::kGpioE);
}

void configure_output(GpioRegs* port, uint8_t pin, Speed speed) {
    set_field(port->MODER, pin, 2, 0x1u);   // general purpose output
    clear_bits(port->OTYPER, 1u << pin);    // push-pull
    set_field(port->OSPEEDR, pin, 2, static_cast<uint32_t>(speed));
    set_field(port->PUPDR, pin, 2, static_cast<uint32_t>(Pull::None));
}

void configure_input(GpioRegs* port, uint8_t pin, Pull pull) {
    set_field(port->MODER, pin, 2, 0x0u);  // input
    set_field(port->PUPDR, pin, 2, static_cast<uint32_t>(pull));
}

void configure_alternate(GpioRegs* port, uint8_t pin, uint8_t af, Speed speed) {
    set_field(port->AFR[pin >> 3u], static_cast<uint8_t>(pin & 0x7u), 4, af);
    set_field(port->MODER, pin, 2, 0x2u);  // alternate function
    clear_bits(port->OTYPER, 1u << pin);
    set_field(port->OSPEEDR, pin, 2, static_cast<uint32_t>(speed));
    set_field(port->PUPDR, pin, 2, static_cast<uint32_t>(Pull::None));
}

}  // namespace stm32
}  // namespace fc
