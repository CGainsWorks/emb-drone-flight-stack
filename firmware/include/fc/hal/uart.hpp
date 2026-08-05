#pragma once

#include <cstddef>
#include <cstdint>

namespace fc {

// Abstract byte-oriented serial port.
class Uart {
public:
    virtual ~Uart() = default;

    // Non-blocking write. Returns the number of bytes accepted, which may be
    // fewer than len if the transmit buffer is full. Telemetry must never
    // block the control loop, so callers are expected to tolerate a short write.
    virtual std::size_t write(const uint8_t* data, std::size_t len) = 0;

    // Non-blocking read into buf. Returns bytes actually read (0 if idle).
    virtual std::size_t read(uint8_t* buf, std::size_t len) = 0;

    // Bytes currently waiting in the receive buffer.
    virtual std::size_t available() const = 0;
};

}  // namespace fc
