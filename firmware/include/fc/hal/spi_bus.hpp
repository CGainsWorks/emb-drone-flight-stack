#pragma once

#include <cstddef>
#include <cstdint>

namespace fc {

// Abstract SPI master.
//
// IMPORTANT (board FCSTMV1.05): the IMU cannot be driven by the STM32F405
// hardware SPI1 peripheral as routed. On this board the F405 PA6 net carries
// the IMU's SDI (data in) and PA7 carries the IMU's SDO (data out), but on the
// F405 the alternate-function mapping is fixed: PA6 can only be SPI1_MISO and
// PA7 can only be SPI1_MOSI. The F405 has no MISO/MOSI swap bit (unlike F3/G4/
// H7). Driving hardware SPI1 would therefore push MOSI into the IMU's output
// pin and the IMU would never receive a command byte.
//
// The concrete implementation for this board is therefore a bit-banged GPIO
// master (see src/platform/), which is free to use PA6 as an output and PA7 as
// an input. This interface exists so the driver above it neither knows nor
// cares, and so host tests can substitute a mock device.
class SpiBus {
public:
    virtual ~SpiBus() = default;

    // Assert / release chip select.
    virtual void select() = 0;
    virtual void deselect() = 0;

    // Full-duplex transfer of len bytes.
    // tx may be nullptr (sends 0x00 padding); rx may be nullptr (discards).
    virtual void transfer(const uint8_t* tx, uint8_t* rx, std::size_t len) = 0;

    // Convenience helper: write one byte, return the byte shifted back.
    uint8_t transfer_byte(uint8_t value) {
        uint8_t rx = 0;
        transfer(&value, &rx, 1);
        return rx;
    }
};

}  // namespace fc
