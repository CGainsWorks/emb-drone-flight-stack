#pragma once

// Simulated MMC5983MA on an I2C bus, for host tests only.
//
// Models what the driver relies on: the register map, single-shot 18-bit
// conversions that take a few status polls to complete, and a bridge offset
// that stays put while the sensed field changes sign between the SET and
// RESET states. Field values and offsets are whatever a test sets.

#include <cmath>
#include <cstdint>
#include <cstring>

#include "fc/drivers/mmc5983.hpp"
#include "fc/hal/i2c_bus.hpp"

namespace fc {
namespace sim {

class SimMmc5983 : public I2cBus {
public:
    SimMmc5983() { soft_reset(); }

    // --- truth -------------------------------------------------------------
    double field_gauss[3]{0.0, 0.0, 0.0};   // body frame
    int32_t bridge_offset_counts[3]{0, 0, 0};

    // --- behaviour ---------------------------------------------------------
    bool responds{true};
    uint8_t address{mmc5983::kAddress};
    uint8_t product_id{mmc5983::kProductId};
    int polls_until_done{2};     // status reads before a conversion completes
    bool conversions_hang{false};

    int measurements{0};
    int sets{0};
    int resets{0};

    bool write(uint8_t addr, const uint8_t* data, std::size_t len) override {
        if (!acks(addr) || len < 2) {
            return false;
        }
        const uint8_t reg = data[0];
        const uint8_t value = data[1];
        if (reg == mmc5983::kRegControl0) {
            if ((value & mmc5983::kCtrl0Set) != 0u) {
                polarity_ = +1;
                ++sets;
            }
            if ((value & mmc5983::kCtrl0Reset) != 0u) {
                polarity_ = -1;
                ++resets;
            }
            if ((value & mmc5983::kCtrl0TakeMeasurement) != 0u) {
                start_measurement();
            }
        } else if (reg == mmc5983::kRegControl1) {
            if ((value & mmc5983::kCtrl1SoftReset) != 0u) {
                soft_reset();
            }
        }
        return true;
    }

    bool write_read(uint8_t addr, const uint8_t* tx, std::size_t tx_len, uint8_t* rx,
                    std::size_t rx_len) override {
        if (!acks(addr) || tx_len != 1) {
            return false;
        }
        uint8_t reg = tx[0];
        for (std::size_t i = 0; i < rx_len; ++i) {
            if (reg == mmc5983::kRegStatus && pending_polls_ > 0) {
                --pending_polls_;
                if (pending_polls_ == 0 && !conversions_hang) {
                    regs_[mmc5983::kRegStatus] |= mmc5983::kStatusMeasDone;
                }
            }
            rx[i] = regs_[reg];
            reg = static_cast<uint8_t>(reg + 1u);
        }
        return true;
    }

private:
    bool acks(uint8_t addr) const { return responds && addr == address; }

    void soft_reset() {
        std::memset(regs_, 0, sizeof(regs_));
        regs_[mmc5983::kRegProductId] = product_id;
        polarity_ = +1;
        pending_polls_ = 0;
    }

    void start_measurement() {
        ++measurements;
        regs_[mmc5983::kRegStatus] &= static_cast<uint8_t>(~mmc5983::kStatusMeasDone);
        pending_polls_ = polls_until_done;
        if (pending_polls_ == 0 && !conversions_hang) {
            regs_[mmc5983::kRegStatus] |= mmc5983::kStatusMeasDone;
        }
        uint8_t low_bits = 0;
        for (int axis = 0; axis < 3; ++axis) {
            const double sensed = polarity_ * field_gauss[axis] *
                                  static_cast<double>(mmc5983::kCountsPerGauss);
            long v = std::lround(sensed) + bridge_offset_counts[axis] + mmc5983::kNullCounts;
            if (v < 0) {
                v = 0;
            }
            if (v > 262143) {
                v = 262143;
            }
            const uint32_t u = static_cast<uint32_t>(v);
            regs_[2 * axis] = static_cast<uint8_t>((u >> 10) & 0xFFu);
            regs_[2 * axis + 1] = static_cast<uint8_t>((u >> 2) & 0xFFu);
            low_bits = static_cast<uint8_t>(low_bits | ((u & 0x3u) << (6 - 2 * axis)));
        }
        regs_[6] = low_bits;
    }

    uint8_t regs_[64]{};
    int polarity_{+1};
    int pending_polls_{0};
};

}  // namespace sim
}  // namespace fc
