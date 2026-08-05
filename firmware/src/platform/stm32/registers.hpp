#pragma once

#include <cstdint>

// Minimal STM32F405 register definitions.
//
// Written by hand rather than pulled from CMSIS so the firmware has no vendor
// dependency and the build is self-contained. Only the peripherals this
// firmware actually touches are declared. Offsets follow RM0090.

namespace fc {
namespace stm32 {

// --- base addresses ----------------------------------------------------
inline constexpr uint32_t kPeriphBase = 0x40000000u;
inline constexpr uint32_t kApb1Base = kPeriphBase;
inline constexpr uint32_t kApb2Base = kPeriphBase + 0x00010000u;
inline constexpr uint32_t kAhb1Base = kPeriphBase + 0x00020000u;

inline constexpr uint32_t kGpioABase = kAhb1Base + 0x0000u;
inline constexpr uint32_t kGpioBBase = kAhb1Base + 0x0400u;
inline constexpr uint32_t kGpioCBase = kAhb1Base + 0x0800u;
inline constexpr uint32_t kGpioDBase = kAhb1Base + 0x0C00u;
inline constexpr uint32_t kGpioEBase = kAhb1Base + 0x1000u;
inline constexpr uint32_t kRccBase = kAhb1Base + 0x3800u;
inline constexpr uint32_t kFlashRegBase = kAhb1Base + 0x3C00u;

inline constexpr uint32_t kPwrBase = kApb1Base + 0x7000u;
inline constexpr uint32_t kTim2Base = kApb1Base + 0x0000u;
inline constexpr uint32_t kUart5Base = kApb1Base + 0x5000u;

inline constexpr uint32_t kTim1Base = kApb2Base + 0x0000u;
inline constexpr uint32_t kTim8Base = kApb2Base + 0x0400u;
inline constexpr uint32_t kUsart1Base = kApb2Base + 0x1000u;
inline constexpr uint32_t kAdc1Base = kApb2Base + 0x2000u;
inline constexpr uint32_t kAdcCommonBase = kApb2Base + 0x2300u;

// --- peripheral layouts ------------------------------------------------

struct GpioRegs {
    volatile uint32_t MODER;
    volatile uint32_t OTYPER;
    volatile uint32_t OSPEEDR;
    volatile uint32_t PUPDR;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t LCKR;
    volatile uint32_t AFR[2];
};

struct RccRegs {
    volatile uint32_t CR;          // 0x00
    volatile uint32_t PLLCFGR;     // 0x04
    volatile uint32_t CFGR;        // 0x08
    volatile uint32_t CIR;         // 0x0C
    volatile uint32_t AHB1RSTR;    // 0x10
    volatile uint32_t AHB2RSTR;    // 0x14
    volatile uint32_t AHB3RSTR;    // 0x18
    uint32_t RESERVED0;            // 0x1C
    volatile uint32_t APB1RSTR;    // 0x20
    volatile uint32_t APB2RSTR;    // 0x24
    uint32_t RESERVED1[2];         // 0x28
    volatile uint32_t AHB1ENR;     // 0x30
    volatile uint32_t AHB2ENR;     // 0x34
    volatile uint32_t AHB3ENR;     // 0x38
    uint32_t RESERVED2;            // 0x3C
    volatile uint32_t APB1ENR;     // 0x40
    volatile uint32_t APB2ENR;     // 0x44
};

struct FlashRegs {
    volatile uint32_t ACR;
};

struct PwrRegs {
    volatile uint32_t CR;
    volatile uint32_t CSR;
};

struct TimRegs {
    volatile uint32_t CR1;    // 0x00
    volatile uint32_t CR2;    // 0x04
    volatile uint32_t SMCR;   // 0x08
    volatile uint32_t DIER;   // 0x0C
    volatile uint32_t SR;     // 0x10
    volatile uint32_t EGR;    // 0x14
    volatile uint32_t CCMR1;  // 0x18
    volatile uint32_t CCMR2;  // 0x1C
    volatile uint32_t CCER;   // 0x20
    volatile uint32_t CNT;    // 0x24
    volatile uint32_t PSC;    // 0x28
    volatile uint32_t ARR;    // 0x2C
    volatile uint32_t RCR;    // 0x30
    volatile uint32_t CCR1;   // 0x34
    volatile uint32_t CCR2;   // 0x38
    volatile uint32_t CCR3;   // 0x3C
    volatile uint32_t CCR4;   // 0x40
    volatile uint32_t BDTR;   // 0x44
    volatile uint32_t DCR;    // 0x48
    volatile uint32_t DMAR;   // 0x4C
};

struct AdcRegs {
    volatile uint32_t SR;     // 0x00
    volatile uint32_t CR1;    // 0x04
    volatile uint32_t CR2;    // 0x08
    volatile uint32_t SMPR1;  // 0x0C
    volatile uint32_t SMPR2;  // 0x10
    volatile uint32_t JOFR[4];// 0x14
    volatile uint32_t HTR;    // 0x24
    volatile uint32_t LTR;    // 0x28
    volatile uint32_t SQR1;   // 0x2C
    volatile uint32_t SQR2;   // 0x30
    volatile uint32_t SQR3;   // 0x34
    volatile uint32_t JSQR;   // 0x38
    volatile uint32_t JDR[4]; // 0x3C
    volatile uint32_t DR;     // 0x4C
};

struct AdcCommonRegs {
    volatile uint32_t CSR;  // 0x00
    volatile uint32_t CCR;  // 0x04
    volatile uint32_t CDR;  // 0x08
};

struct UsartRegs {
    volatile uint32_t SR;    // 0x00
    volatile uint32_t DR;    // 0x04
    volatile uint32_t BRR;   // 0x08
    volatile uint32_t CR1;   // 0x0C
    volatile uint32_t CR2;   // 0x10
    volatile uint32_t CR3;   // 0x14
    volatile uint32_t GTPR;  // 0x18
};

// --- volatile access helpers -------------------------------------------
//
// C++20 deprecates compound assignment (|=, &=, ++) on volatile operands
// because the read and the write are two separate accesses and the order is
// not obvious from the syntax. On memory-mapped registers that distinction
// genuinely matters, so the read-modify-write is spelled out here once.

inline void set_bits(volatile uint32_t& reg, uint32_t bits) {
    const uint32_t value = reg;
    reg = value | bits;
}

inline void clear_bits(volatile uint32_t& reg, uint32_t bits) {
    const uint32_t value = reg;
    reg = value & ~bits;
}

inline void modify_bits(volatile uint32_t& reg, uint32_t clear_mask,
                        uint32_t set_mask) {
    const uint32_t value = reg;
    reg = (value & ~clear_mask) | set_mask;
}

// --- accessors ---------------------------------------------------------
inline GpioRegs* gpio(uint32_t base) { return reinterpret_cast<GpioRegs*>(base); }
inline RccRegs* rcc() { return reinterpret_cast<RccRegs*>(kRccBase); }
inline FlashRegs* flash_regs() { return reinterpret_cast<FlashRegs*>(kFlashRegBase); }
inline PwrRegs* pwr() { return reinterpret_cast<PwrRegs*>(kPwrBase); }
inline TimRegs* tim(uint32_t base) { return reinterpret_cast<TimRegs*>(base); }
inline UsartRegs* usart(uint32_t base) { return reinterpret_cast<UsartRegs*>(base); }
inline AdcRegs* adc1() { return reinterpret_cast<AdcRegs*>(kAdc1Base); }
inline AdcCommonRegs* adc_common() {
    return reinterpret_cast<AdcCommonRegs*>(kAdcCommonBase);
}

// --- bit definitions used here ----------------------------------------
namespace rcc_cr {
inline constexpr uint32_t kHseOn = 1u << 16;
inline constexpr uint32_t kHseRdy = 1u << 17;
inline constexpr uint32_t kPllOn = 1u << 24;
inline constexpr uint32_t kPllRdy = 1u << 25;
}  // namespace rcc_cr

namespace rcc_cfgr {
inline constexpr uint32_t kSwPll = 0x2u;
inline constexpr uint32_t kSwsMask = 0xCu;
inline constexpr uint32_t kSwsPll = 0x8u;
inline constexpr uint32_t kHpreDiv1 = 0x0u << 4;
inline constexpr uint32_t kPpre1Div4 = 0x5u << 10;
inline constexpr uint32_t kPpre2Div2 = 0x4u << 13;
}  // namespace rcc_cfgr

namespace rcc_ahb1enr {
inline constexpr uint32_t kGpioA = 1u << 0;
inline constexpr uint32_t kGpioB = 1u << 1;
inline constexpr uint32_t kGpioC = 1u << 2;
inline constexpr uint32_t kGpioD = 1u << 3;
inline constexpr uint32_t kGpioE = 1u << 4;
}  // namespace rcc_ahb1enr

namespace rcc_apb1enr {
inline constexpr uint32_t kTim2 = 1u << 0;
inline constexpr uint32_t kUart5 = 1u << 20;
inline constexpr uint32_t kPwr = 1u << 28;
}  // namespace rcc_apb1enr

namespace rcc_apb2enr {
inline constexpr uint32_t kTim1 = 1u << 0;
inline constexpr uint32_t kTim8 = 1u << 1;
inline constexpr uint32_t kUsart1 = 1u << 4;
inline constexpr uint32_t kAdc1 = 1u << 8;
}  // namespace rcc_apb2enr

namespace adc_sr {
inline constexpr uint32_t kEoc = 1u << 1;
}

namespace adc_cr2 {
inline constexpr uint32_t kAdon = 1u << 0;
inline constexpr uint32_t kSwstart = 1u << 30;
}  // namespace adc_cr2

namespace usart_sr {
inline constexpr uint32_t kRxne = 1u << 5;
inline constexpr uint32_t kTc = 1u << 6;
inline constexpr uint32_t kTxe = 1u << 7;
inline constexpr uint32_t kIdle = 1u << 4;
inline constexpr uint32_t kOre = 1u << 3;
}  // namespace usart_sr

namespace usart_cr1 {
inline constexpr uint32_t kUe = 1u << 13;
inline constexpr uint32_t kTe = 1u << 3;
inline constexpr uint32_t kRe = 1u << 2;
inline constexpr uint32_t kRxneie = 1u << 5;
inline constexpr uint32_t kIdleie = 1u << 4;
}  // namespace usart_cr1

namespace tim_cr1 {
inline constexpr uint32_t kCen = 1u << 0;
inline constexpr uint32_t kArpe = 1u << 7;
}  // namespace tim_cr1

namespace tim_egr {
inline constexpr uint32_t kUg = 1u << 0;
}

namespace tim_bdtr {
inline constexpr uint32_t kMoe = 1u << 15;
}

// --- Cortex-M core -----------------------------------------------------
inline volatile uint32_t* nvic_iser() {
    return reinterpret_cast<volatile uint32_t*>(0xE000E100u);
}

inline void nvic_enable_irq(uint32_t irq) {
    nvic_iser()[irq >> 5u] = 1u << (irq & 0x1Fu);
}

// IRQ numbers used here (RM0090 vector table positions).
inline constexpr uint32_t kIrqUsart1 = 37u;

}  // namespace stm32
}  // namespace fc
