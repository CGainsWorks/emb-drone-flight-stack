#include "system.hpp"

#include "registers.hpp"

namespace fc {
namespace stm32 {
namespace {

// Bounded spin so a missing crystal reports a failure instead of hanging the
// board forever with no indication why.
constexpr uint32_t kStartupTimeout = 0x00200000u;

bool wait_for_flag(volatile uint32_t& reg, uint32_t mask, bool want_set) {
    for (uint32_t i = 0; i < kStartupTimeout; ++i) {
        const bool is_set = (reg & mask) != 0u;
        if (is_set == want_set) {
            return true;
        }
    }
    return false;
}

}  // namespace

bool configure_clocks() {
    auto* r = rcc();

    // 1. Start the external crystal.
    set_bits(r->CR, rcc_cr::kHseOn);
    if (!wait_for_flag(r->CR, rcc_cr::kHseRdy, true)) {
        return false;
    }

    // 2. Voltage scale 1, required for 168 MHz operation.
    set_bits(r->APB1ENR, rcc_apb1enr::kPwr);
    set_bits(pwr()->CR, 1u << 14);  // VOS = scale 1

    // 3. Flash: 5 wait states at 168 MHz / 3.3 V, with caches and prefetch on.
    flash_regs()->ACR = (1u << 8) | (1u << 9) | (1u << 10) | 5u;

    // 4. Bus prescalers, set before the PLL is selected so no bus is ever
    //    briefly overclocked during the switch.
    uint32_t cfgr = r->CFGR;
    cfgr &= ~((0xFu << 4) | (0x7u << 10) | (0x7u << 13));
    cfgr |= rcc_cfgr::kHpreDiv1 | rcc_cfgr::kPpre1Div4 | rcc_cfgr::kPpre2Div2;
    r->CFGR = cfgr;

    // 5. PLL: M=8, N=336, P=2, Q=7, source HSE.
    constexpr uint32_t kPllM = 8u;
    constexpr uint32_t kPllN = 336u;
    constexpr uint32_t kPllP = 0u;  // 00 = divide by 2
    constexpr uint32_t kPllQ = 7u;
    r->PLLCFGR = kPllM | (kPllN << 6) | (kPllP << 16) | (1u << 22) | (kPllQ << 24);

    // 6. Start the PLL and wait for lock.
    set_bits(r->CR, rcc_cr::kPllOn);
    if (!wait_for_flag(r->CR, rcc_cr::kPllRdy, true)) {
        return false;
    }

    // 7. Switch the system clock over.
    cfgr = r->CFGR;
    cfgr &= ~0x3u;
    cfgr |= rcc_cfgr::kSwPll;
    r->CFGR = cfgr;

    for (uint32_t i = 0; i < kStartupTimeout; ++i) {
        if ((r->CFGR & rcc_cfgr::kSwsMask) == rcc_cfgr::kSwsPll) {
            return true;
        }
    }
    return false;
}

void start_microsecond_timer() {
    set_bits(rcc()->APB1ENR, rcc_apb1enr::kTim2);

    auto* t = tim(kTim2Base);
    t->CR1 = 0;
    // APB1 is 42 MHz, so the timer clock is doubled to 84 MHz.
    t->PSC = 84u - 1u;       // -> 1 MHz, one tick per microsecond
    t->ARR = 0xFFFFFFFFu;    // full 32-bit range
    t->EGR = tim_egr::kUg;   // load the prescaler immediately
    t->CNT = 0;
    t->CR1 = tim_cr1::kCen;
}

uint32_t Stm32Clock::micros() const { return tim(kTim2Base)->CNT; }

void Stm32Clock::delay_us(uint32_t us) {
    const uint32_t start = micros();
    // Unsigned subtraction, so this is correct across the 32-bit wrap.
    while (static_cast<uint32_t>(micros() - start) < us) {
    }
}

}  // namespace stm32
}  // namespace fc
