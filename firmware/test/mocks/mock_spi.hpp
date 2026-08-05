#pragma once

#include <cstring>
#include <vector>

#include "fc/drivers/icm42688.hpp"
#include "fc/hal/clock.hpp"
#include "fc/hal/motor_output.hpp"
#include "fc/hal/spi_bus.hpp"
#include "fc/hal/uart.hpp"

namespace fc {
namespace testing_mocks {

// Deterministic fake clock. Time only advances when the test says so (or via
// delay_us), which makes timeout and failsafe logic exactly reproducible.
class FakeClock : public Clock {
public:
    uint32_t micros() const override { return now_us_; }
    void delay_us(uint32_t us) override { now_us_ += us; }
    void advance_us(uint32_t us) { now_us_ += us; }
    void advance_ms(uint32_t ms) { now_us_ += ms * 1000u; }
    void set_us(uint32_t us) { now_us_ = us; }

private:
    uint32_t now_us_{0};
};

// A fake ICM-42688-P sitting on the far end of the SPI bus.
//
// Implements the part's actual wire protocol: one address byte whose MSB
// selects read vs write, followed by auto-incrementing register access. This
// lets the real driver be exercised end to end with no hardware.
class MockIcm42688 : public SpiBus {
public:
    MockIcm42688() { reset_registers(); }

    // --- SpiBus ---------------------------------------------------------
    void select() override {
        selected_ = true;
        ++select_count;
    }

    void deselect() override {
        selected_ = false;
        ++deselect_count;
    }

    void transfer(const uint8_t* tx, uint8_t* rx, std::size_t len) override {
        ++transfer_count;
        if (!selected_) {
            // Talking to the device without asserting CS is a driver bug.
            ++transfers_without_cs;
        }
        if (len == 0) {
            return;
        }

        const uint8_t addr_byte = (tx != nullptr) ? tx[0] : 0x00;
        const bool is_read = (addr_byte & 0x80u) != 0u;
        uint8_t addr = addr_byte & 0x7Fu;

        if (rx != nullptr) {
            rx[0] = 0x00;  // status byte shifted out during the address phase
        }

        for (std::size_t i = 1; i < len; ++i) {
            if (is_read) {
                const uint8_t value = regs[addr & 0x7Fu];
                if (rx != nullptr) {
                    rx[i] = value;
                }
                read_log.push_back(addr);
            } else {
                const uint8_t value = (tx != nullptr) ? tx[i] : 0x00;
                write_register_internal(addr, value);
                if (rx != nullptr) {
                    rx[i] = 0x00;
                }
            }
            addr = static_cast<uint8_t>(addr + 1u);
        }
    }

    // --- test helpers ---------------------------------------------------
    void reset_registers() {
        std::memset(regs, 0, sizeof(regs));
        regs[icm42688::kRegWhoAmI] = who_am_i_value;
    }

    void set_who_am_i(uint8_t v) {
        who_am_i_value = v;
        regs[icm42688::kRegWhoAmI] = v;
    }

    // Load a signed 16-bit big-endian sample into consecutive registers.
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

    void set_temp_raw(int16_t t) { set_be16(icm42688::kRegTempData1, t); }

    uint8_t reg(uint8_t a) const { return regs[a & 0x7Fu]; }

    uint8_t regs[128]{};
    uint8_t who_am_i_value{icm42688::kWhoAmIValue};

    int select_count{0};
    int deselect_count{0};
    int transfer_count{0};
    int transfers_without_cs{0};
    int soft_reset_count{0};
    std::vector<uint8_t> read_log;

    // If set, writes to registers are refused, simulating a dead/miswired bus.
    bool writes_ignored{false};

private:
    void write_register_internal(uint8_t addr, uint8_t value) {
        if (writes_ignored) {
            return;
        }
        if (addr == icm42688::kRegDeviceConfig && (value & 0x01u) != 0u) {
            // Soft reset: registers return to defaults but the part identity
            // survives, exactly like the real device.
            ++soft_reset_count;
            reset_registers();
            return;
        }
        regs[addr & 0x7Fu] = value;
    }

    bool selected_{false};
};

// Records what the control loop commanded, so tests can assert on motor
// behaviour without any ESC present.
class MockMotorOutput : public MotorOutput {
public:
    void write(const float throttles[kMotorCount]) override {
        for (std::size_t i = 0; i < kMotorCount; ++i) {
            motor[i] = throttles[i];
        }
        ++write_count;
        armed_output = true;
    }

    void disarm() override {
        for (std::size_t i = 0; i < kMotorCount; ++i) {
            motor[i] = 0.0f;
        }
        ++disarm_count;
        armed_output = false;
    }

    bool all_stopped() const {
        for (std::size_t i = 0; i < kMotorCount; ++i) {
            if (motor[i] != 0.0f) {
                return false;
            }
        }
        return true;
    }

    float motor[kMotorCount]{};
    int write_count{0};
    int disarm_count{0};
    bool armed_output{false};
};

// Loopback UART for exercising telemetry output.
class MockUart : public Uart {
public:
    std::size_t write(const uint8_t* data, std::size_t len) override {
        // A real UART transmit buffer can be full. accept_limit models that so
        // partial-write handling is exercised rather than assumed.
        std::size_t n = len;
        if (accept_limit >= 0 && static_cast<std::size_t>(accept_limit) < n) {
            n = static_cast<std::size_t>(accept_limit);
        }
        for (std::size_t i = 0; i < n; ++i) {
            tx.push_back(data[i]);
        }
        return n;
    }

    // -1 = unlimited. Otherwise the most bytes a single write() will accept.
    int accept_limit{-1};

    std::size_t read(uint8_t* buf, std::size_t len) override {
        std::size_t n = 0;
        while (n < len && read_pos < rx.size()) {
            buf[n++] = rx[read_pos++];
        }
        return n;
    }

    std::size_t available() const override { return rx.size() - read_pos; }

    void feed(const uint8_t* data, std::size_t len) {
        for (std::size_t i = 0; i < len; ++i) {
            rx.push_back(data[i]);
        }
    }

    std::vector<uint8_t> tx;
    std::vector<uint8_t> rx;
    std::size_t read_pos{0};
};

}  // namespace testing_mocks
}  // namespace fc
