#pragma once

#include <cstdint>

#include "fc/hal/uart.hpp"
#include "registers.hpp"

namespace fc {
namespace stm32 {

// Interrupt-driven receive, polled non-blocking transmit.
//
// Receive must be interrupt-driven: CRSF runs at 420 kbaud, which is a byte
// roughly every 24 us. Polling once per 1 ms control cycle would lose most of
// a frame. Transmit is polled because telemetry is best-effort and must never
// stall the loop - write() accepts what fits and reports the rest as unwritten.
class Stm32Uart : public Uart {
public:
    Stm32Uart(uint32_t usart_base, uint32_t pclk_hz)
        : base_(usart_base), pclk_hz_(pclk_hz) {}

    void begin(uint32_t baud, bool enable_rx_interrupt);

    std::size_t write(const uint8_t* data, std::size_t len) override;
    std::size_t read(uint8_t* buf, std::size_t len) override;
    std::size_t available() const override;

    // Called from the USART interrupt handler.
    void handle_irq();

    // True once since the last call: the line went idle, which marks a frame
    // boundary for the CRSF parser.
    bool take_idle_flag();

    uint32_t overrun_errors() const { return overruns_; }

private:
    static constexpr std::size_t kRxBufferSize = 256;  // power of two

    uint32_t base_;
    uint32_t pclk_hz_;

    volatile uint8_t rx_buf_[kRxBufferSize]{};
    volatile std::size_t rx_head_{0};
    volatile std::size_t rx_tail_{0};
    volatile bool idle_flag_{false};
    volatile uint32_t overruns_{0};
};

// Registered so the USART1 interrupt vector can reach the driver instance.
void set_usart1_instance(Stm32Uart* uart);

}  // namespace stm32
}  // namespace fc
