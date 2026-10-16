#pragma once

#include <cstddef>
#include <cstdint>

namespace fc {

// Abstract I2C master, 7-bit addressing.
//
// On FCSTMV1.05 the BMP390 barometer sits on the F405's I2C1 (PB8/PB9). The
// MMC5983MA magnetometer is on a separate bus in the GPS power island that does
// not reach the F405 on this board revision (see docs/PINOUT.md), so its driver
// is written against this interface for a later revision or an external
// module. No STM32 implementation of this interface exists yet; host tests
// substitute simulated devices from sim/sensors/.
class I2cBus {
public:
    virtual ~I2cBus() = default;

    // Write len bytes to the device. Returns false if it did not acknowledge.
    virtual bool write(uint8_t address, const uint8_t* data, std::size_t len) = 0;

    // Write tx (usually a register address), repeated start, read rx_len bytes.
    virtual bool write_read(uint8_t address, const uint8_t* tx, std::size_t tx_len, uint8_t* rx,
                            std::size_t rx_len) = 0;

    bool write_register(uint8_t address, uint8_t reg, uint8_t value) {
        const uint8_t buf[2] = {reg, value};
        return write(address, buf, 2);
    }

    bool read_registers(uint8_t address, uint8_t reg, uint8_t* out, std::size_t len) {
        return write_read(address, &reg, 1, out, len);
    }
};

}  // namespace fc
