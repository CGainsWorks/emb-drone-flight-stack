#include "uart_stm32.hpp"

namespace fc {
namespace stm32 {
namespace {

Stm32Uart* g_usart1 = nullptr;

}  // namespace

void set_usart1_instance(Stm32Uart* uart) { g_usart1 = uart; }

void Stm32Uart::begin(uint32_t baud, bool enable_rx_interrupt) {
    auto* u = usart(base_);

    u->CR1 = 0;

    // Oversampling by 16: BRR holds USARTDIV * 16 directly, so this is simply
    // the peripheral clock divided by the baud rate, rounded to nearest.
    u->BRR = (pclk_hz_ + baud / 2u) / baud;

    u->CR2 = 0;  // 1 stop bit
    u->CR3 = 0;  // no flow control

    uint32_t cr1 = usart_cr1::kUe | usart_cr1::kTe | usart_cr1::kRe;
    if (enable_rx_interrupt) {
        cr1 |= usart_cr1::kRxneie | usart_cr1::kIdleie;
    }
    u->CR1 = cr1;
}

void Stm32Uart::handle_irq() {
    auto* u = usart(base_);
    const uint32_t sr = u->SR;

    if ((sr & usart_sr::kOre) != 0u) {
        // Overrun: reading DR clears the flag. The byte is already lost, so
        // just count it - the CRSF CRC will reject the damaged frame.
        (void)u->DR;
        overruns_ = overruns_ + 1u;
    }

    if ((sr & usart_sr::kRxne) != 0u) {
        const uint8_t byte = static_cast<uint8_t>(u->DR & 0xFFu);
        const std::size_t next = (rx_head_ + 1u) & (kRxBufferSize - 1u);
        if (next != rx_tail_) {
            rx_buf_[rx_head_] = byte;
            rx_head_ = next;
        } else {
            overruns_ = overruns_ + 1u;  // software buffer full
        }
    }

    if ((sr & usart_sr::kIdle) != 0u) {
        // Clearing IDLE requires a read of SR followed by a read of DR.
        (void)u->DR;
        idle_flag_ = true;
    }
}

bool Stm32Uart::take_idle_flag() {
    if (!idle_flag_) {
        return false;
    }
    idle_flag_ = false;
    return true;
}

std::size_t Stm32Uart::available() const {
    return (rx_head_ - rx_tail_) & (kRxBufferSize - 1u);
}

std::size_t Stm32Uart::read(uint8_t* buf, std::size_t len) {
    std::size_t n = 0;
    while (n < len && rx_tail_ != rx_head_) {
        buf[n++] = rx_buf_[rx_tail_];
        rx_tail_ = (rx_tail_ + 1u) & (kRxBufferSize - 1u);
    }
    return n;
}

std::size_t Stm32Uart::write(const uint8_t* data, std::size_t len) {
    auto* u = usart(base_);
    std::size_t written = 0;
    // Push only what the transmit register can take right now. Never spin
    // waiting for the shift register: at 115 kbaud a full frame would cost
    // most of a control cycle.
    while (written < len && (u->SR & usart_sr::kTxe) != 0u) {
        u->DR = data[written];
        ++written;
    }
    return written;
}

// Declared inside the namespace so it can see the file-local instance pointer;
// extern "C" still gives it the plain symbol name the vector table expects.
extern "C" void USART1_IRQHandler() {
    if (g_usart1 != nullptr) {
        g_usart1->handle_irq();
    }
}

}  // namespace stm32
}  // namespace fc
