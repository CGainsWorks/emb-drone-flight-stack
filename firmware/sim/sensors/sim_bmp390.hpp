#pragma once

// Simulated BMP390 on an I2C bus, for host tests only.
//
// It answers at its 7-bit address with the part's register map, holds a set
// of trimming coefficients in "NVM", and turns a true pressure and temperature
// into the raw 24-bit ADC values that the datasheet compensation maps back to
// them. The coefficients are SYNTHETIC: chosen to be well conditioned over
// 30 to 125 kPa and -40 to 85 degC, not copied from any real part.

#include <cstdint>
#include <cstring>

#include "fc/drivers/bmp390.hpp"
#include "fc/hal/i2c_bus.hpp"

namespace fc {
namespace sim {

class SimBmp390 : public I2cBus {
public:
    SimBmp390() {
        set_nvm(kSyntheticNvm);
        reset();
    }

    // --- truth -------------------------------------------------------------
    double pressure_pa{101325.0};
    double temperature_c{25.0};

    // --- faults ------------------------------------------------------------
    // False models CSB left floating (R128 not fitted): nothing acknowledges.
    bool responds{true};
    uint8_t address{bmp390::kAddress};
    uint8_t chip_id{bmp390::kChipId};

    int soft_resets{0};

    // --- I2cBus ------------------------------------------------------------
    bool write(uint8_t addr, const uint8_t* data, std::size_t len) override {
        if (!acks(addr) || len == 0) {
            return false;
        }
        uint8_t reg = data[0];
        for (std::size_t i = 1; i < len; ++i) {
            write_register(reg, data[i]);
            reg = static_cast<uint8_t>(reg + 1u);
        }
        return true;
    }

    bool write_read(uint8_t addr, const uint8_t* tx, std::size_t tx_len, uint8_t* rx,
                    std::size_t rx_len) override {
        if (!acks(addr) || tx_len != 1) {
            return false;
        }
        refresh_data();
        uint8_t reg = tx[0];
        for (std::size_t i = 0; i < rx_len; ++i) {
            rx[i] = regs_[reg];
            reg = static_cast<uint8_t>(reg + 1u);
        }
        return true;
    }

    uint8_t reg(uint8_t r) const { return regs_[r]; }

    // Raw ADC values the device would report for the current truth, found by
    // bisection on the datasheet compensation (both are monotonic here).
    uint32_t raw_temperature() const {
        return invert([&](uint32_t raw) { return bmp390::compensate_temperature(cal_, raw); },
                      temperature_c);
    }

    uint32_t raw_pressure() const {
        const double t = bmp390::compensate_temperature(cal_, raw_temperature());
        return invert([&](uint32_t raw) { return bmp390::compensate_pressure(cal_, raw, t); },
                      pressure_pa);
    }

private:
    // NVM_PAR_T1 .. NVM_PAR_P11, little-endian, as stored from 0x31.
    static constexpr uint8_t kSyntheticNvm[bmp390::kCalibBytes] = {
        0x78, 0x69,  // T1  = 27000
        0x38, 0x4A,  // T2  = 19000
        0xF9,        // T3  = -7
        0x18, 0x79,  // P1  = 31000
        0x80, 0x3E,  // P2  = 16000
        0x05,        // P3  = 5
        0x00,        // P4  = 0
        0xD0, 0x07,  // P5  = 2000
        0xDC, 0x05,  // P6  = 1500
        0x02,        // P7  = 2
        0xFD,        // P8  = -3
        0xE8, 0x03,  // P9  = 1000
        0x05,        // P10 = 5
        0xF6,        // P11 = -10
    };

    bool acks(uint8_t addr) const { return responds && addr == address; }

    void set_nvm(const uint8_t* nvm) {
        std::memcpy(nvm_, nvm, sizeof(nvm_));
        cal_ = bmp390::parse_calibration(nvm_);
    }

    void reset() {
        std::memset(regs_, 0, sizeof(regs_));
        regs_[bmp390::kRegChipId] = chip_id;
        regs_[bmp390::kRegOdr] = 0x00;
        regs_[bmp390::kRegOsr] = 0x02;  // datasheet reset value
        std::memcpy(&regs_[bmp390::kRegCalib], nvm_, sizeof(nvm_));
    }

    void write_register(uint8_t r, uint8_t value) {
        if (r == bmp390::kRegCmd) {
            if (value == bmp390::kCmdSoftReset) {
                ++soft_resets;
                reset();
            }
            return;
        }
        if (r == bmp390::kRegChipId || r == bmp390::kRegErr || r == bmp390::kRegStatus ||
            (r >= bmp390::kRegData0 && r < bmp390::kRegData0 + 6) ||
            (r >= bmp390::kRegCalib && r < bmp390::kRegCalib + bmp390::kCalibBytes)) {
            return;  // read-only
        }
        regs_[r] = value;
    }

    void refresh_data() {
        const bool running = (regs_[bmp390::kRegPwrCtrl] & 0x30u) == 0x30u;
        uint32_t p = 0x800000u;  // reset value of the data registers
        uint32_t t = 0x800000u;
        if (running) {
            p = raw_pressure();
            t = raw_temperature();
            regs_[bmp390::kRegStatus] = 0x70;  // command ready, both data ready
        }
        for (int i = 0; i < 3; ++i) {
            regs_[bmp390::kRegData0 + i] = static_cast<uint8_t>((p >> (8 * i)) & 0xFFu);
            regs_[bmp390::kRegData0 + 3 + i] = static_cast<uint8_t>((t >> (8 * i)) & 0xFFu);
        }
    }

    template <typename F>
    static uint32_t invert(F f, double target) {
        uint32_t lo = 0;
        uint32_t hi = 0xFFFFFFu;
        while (hi - lo > 1u) {
            const uint32_t mid = lo + (hi - lo) / 2u;
            if (f(mid) < target) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        return (target - f(lo) < f(hi) - target) ? lo : hi;
    }

    uint8_t nvm_[bmp390::kCalibBytes]{};
    bmp390::Calibration cal_{};
    uint8_t regs_[256]{};
};

}  // namespace sim
}  // namespace fc
