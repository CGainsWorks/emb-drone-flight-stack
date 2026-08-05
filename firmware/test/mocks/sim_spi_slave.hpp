#pragma once

#include <cstring>

#include "fc/drivers/icm42688.hpp"
#include "fc/hal/gpio.hpp"

namespace fc {
namespace testing_mocks {

// A bit-level simulation of the ICM-42688-P as an SPI slave.
//
// Unlike the register-level MockIcm42688 (mock_spi.hpp), this one sees only pin
// transitions and reconstructs the protocol from the waveform: it presents MISO
// on falling clock edges and samples MOSI on rising edges, exactly as a mode-3
// slave does. That makes it a real check of the bit-banged master's clock
// polarity, clock phase, bit order and chip-select framing - the things that
// would otherwise need an oscilloscope and a populated board to verify.
class SimulatedIcm42688 {
public:
    SimulatedIcm42688() { reset_registers(); }

    // --- pins handed to the master ---
    GpioPin& sck() { return sck_pin_; }
    GpioPin& mosi() { return mosi_pin_; }
    GpioPin& miso() { return miso_pin_; }
    GpioPin& cs() { return cs_pin_; }

    // --- register file helpers (same surface as the register-level mock) ---
    void reset_registers() {
        std::memset(regs, 0, sizeof(regs));
        regs[icm42688::kRegWhoAmI] = who_am_i_value;
    }

    void set_who_am_i(uint8_t v) {
        who_am_i_value = v;
        regs[icm42688::kRegWhoAmI] = v;
    }

    void set_be16(uint8_t reg, int16_t value) {
        regs[reg] = static_cast<uint8_t>((static_cast<uint16_t>(value) >> 8) & 0xFFu);
        regs[reg + 1] = static_cast<uint8_t>(static_cast<uint16_t>(value) & 0xFFu);
    }

    void set_accel_raw(int16_t x, int16_t y, int16_t z) {
        set_be16(icm42688::kRegAccelDataX1 + 0, x);
        set_be16(icm42688::kRegAccelDataX1 + 2, y);
        set_be16(icm42688::kRegAccelDataX1 + 4, z);
    }

    void set_gyro_raw(int16_t x, int16_t y, int16_t z) {
        set_be16(icm42688::kRegAccelDataX1 + 6, x);
        set_be16(icm42688::kRegAccelDataX1 + 8, y);
        set_be16(icm42688::kRegAccelDataX1 + 10, z);
    }

    uint8_t reg(uint8_t a) const { return regs[a & 0x7Fu]; }

    uint8_t regs[128]{};
    uint8_t who_am_i_value{icm42688::kWhoAmIValue};

    // --- observations used by the waveform tests ---
    int rising_edges{0};
    int falling_edges{0};
    int transactions{0};
    int soft_reset_count{0};
    // Set if the clock ever moved while chip select was deasserted, or if a
    // transaction ended on a non-byte boundary. Either is a protocol error.
    bool clock_while_deselected{false};
    bool ragged_transaction{false};
    bool clock_idled_low_at_select{false};

private:
    // Pin objects that funnel every transition back into the slave model.
    class OutPin : public GpioPin {
    public:
        OutPin(SimulatedIcm42688& owner, void (SimulatedIcm42688::*handler)(bool))
            : owner_(owner), handler_(handler) {}
        void write(bool high) override {
            state_ = high;
            (owner_.*handler_)(high);
        }
        bool read() const override { return state_; }

    private:
        SimulatedIcm42688& owner_;
        void (SimulatedIcm42688::*handler_)(bool);
        bool state_{false};
    };

    class MisoPin : public GpioPin {
    public:
        explicit MisoPin(SimulatedIcm42688& owner) : owner_(owner) {}
        void write(bool) override {}  // an input to the master; ignore writes
        bool read() const override { return owner_.miso_state_; }

    private:
        SimulatedIcm42688& owner_;
    };

    void on_sck(bool high) {
        if (high == sck_state_) {
            return;  // no edge
        }
        sck_state_ = high;

        if (!selected_) {
            // A clock edge with CS high is meaningless to a real slave and
            // indicates the master is toggling the bus outside a transaction.
            clock_while_deselected = true;
            return;
        }

        if (high) {
            ++rising_edges;
            // Trailing edge: sample MOSI.
            if (mosi_state_) {
                shift_in_ = static_cast<uint8_t>(shift_in_ | (1u << (7 - bit_index_)));
            }
            ++bit_index_;
            if (bit_index_ == 8) {
                process_byte(shift_in_);
                shift_in_ = 0;
                bit_index_ = 0;
                ++byte_index_;
            }
        } else {
            ++falling_edges;
            // Leading edge: present the next MISO bit.
            miso_state_ = ((shift_out_ >> (7 - bit_index_)) & 0x01u) != 0u;
        }
    }

    void on_mosi(bool high) { mosi_state_ = high; }

    void on_cs(bool high) {
        if (high == !selected_) {
            return;  // no change
        }
        if (!high) {
            // CS falling: a transaction begins.
            selected_ = true;
            ++transactions;
            bit_index_ = 0;
            byte_index_ = 0;
            shift_in_ = 0;
            shift_out_ = 0;  // status byte during the address phase
            // Mode 3 requires the clock to be parked high before CS falls.
            if (!sck_state_) {
                clock_idled_low_at_select = true;
            }
        } else {
            // CS rising: the transaction ends. It must land on a byte boundary.
            if (bit_index_ != 0) {
                ragged_transaction = true;
            }
            selected_ = false;
        }
    }

    void process_byte(uint8_t b) {
        if (byte_index_ == 0) {
            addr_ = static_cast<uint8_t>(b & 0x7Fu);
            is_read_ = (b & 0x80u) != 0u;
            shift_out_ = is_read_ ? regs[addr_] : 0x00u;
            return;
        }

        if (!is_read_) {
            write_register(addr_, b);
        }
        addr_ = static_cast<uint8_t>((addr_ + 1u) & 0x7Fu);
        shift_out_ = is_read_ ? regs[addr_] : 0x00u;
    }

    void write_register(uint8_t addr, uint8_t value) {
        if (addr == icm42688::kRegDeviceConfig && (value & 0x01u) != 0u) {
            ++soft_reset_count;
            reset_registers();
            return;
        }
        regs[addr & 0x7Fu] = value;
    }

    OutPin sck_pin_{*this, &SimulatedIcm42688::on_sck};
    OutPin mosi_pin_{*this, &SimulatedIcm42688::on_mosi};
    OutPin cs_pin_{*this, &SimulatedIcm42688::on_cs};
    MisoPin miso_pin_{*this};

    bool sck_state_{true};  // idles high
    bool mosi_state_{false};
    bool miso_state_{false};
    bool selected_{false};

    uint8_t shift_in_{0};
    uint8_t shift_out_{0};
    uint8_t bit_index_{0};
    uint16_t byte_index_{0};
    uint8_t addr_{0};
    bool is_read_{false};
};

}  // namespace testing_mocks
}  // namespace fc
