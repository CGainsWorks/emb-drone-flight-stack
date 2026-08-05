#include "adc.hpp"

#include "gpio.hpp"
#include "registers.hpp"

namespace fc {
namespace stm32 {

constexpr uint8_t Stm32Adc::kChannels[Stm32Adc::kChannelCount];

void Stm32Adc::begin() {
    set_bits(rcc()->APB2ENR, rcc_apb2enr::kAdc1);

    // PC1, PC2 and PC3 to analogue mode (MODER = 11).
    auto* portc = gpio(kGpioCBase);
    for (uint8_t pin = 1; pin <= 3; ++pin) {
        const uint32_t shift = static_cast<uint32_t>(pin) * 2u;
        modify_bits(portc->MODER, 0x3u << shift, 0x3u << shift);
        modify_bits(portc->PUPDR, 0x3u << shift, 0u);  // no pull on an analogue in
    }

    // ADCPRE = PCLK2 / 4. APB2 is 84 MHz, so the ADC clock is 21 MHz, inside
    // the 36 MHz maximum.
    modify_bits(adc_common()->CCR, 0x3u << 16, 0x1u << 16);

    auto* a = adc1();
    a->CR1 = 0;  // 12-bit resolution, no scan
    a->CR2 = 0;

    // 144-cycle sample time for channels 11..13. The battery divider is high
    // impedance, so the sampling capacitor needs time to settle; too short a
    // window reads low.
    uint32_t smpr1 = a->SMPR1;
    for (uint8_t i = 0; i < kChannelCount; ++i) {
        const uint32_t ch = kChannels[i];
        const uint32_t shift = (ch - 10u) * 3u;
        smpr1 = (smpr1 & ~(0x7u << shift)) | (0x6u << shift);  // 110 = 144 cycles
    }
    a->SMPR1 = smpr1;

    a->SQR1 = 0;  // L = 0 -> one conversion in the regular sequence

    set_bits(a->CR2, adc_cr2::kAdon);
}

void Stm32Adc::start_conversion(uint8_t channel) {
    auto* a = adc1();
    a->SQR3 = channel;
    set_bits(a->CR2, adc_cr2::kSwstart);
    conversion_active_ = true;
}

void Stm32Adc::poll() {
    auto* a = adc1();

    if (!conversion_active_) {
        start_conversion(kChannels[index_]);
        return;
    }

    if ((a->SR & adc_sr::kEoc) == 0u) {
        return;  // still converting; never block the loop waiting
    }

    counts_[index_] = static_cast<uint16_t>(a->DR & 0xFFFFu);  // reading DR clears EOC
    conversion_active_ = false;

    if (completed_ < kChannelCount) {
        ++completed_;
        if (completed_ >= kChannelCount) {
            sampled_all_ = true;
        }
    }

    index_ = static_cast<uint8_t>((index_ + 1u) % kChannelCount);
    start_conversion(kChannels[index_]);
}

}  // namespace stm32
}  // namespace fc
