#pragma once

#include <cstdint>

namespace fc {

// Monotonic microsecond time source.
//
// Host tests inject a fake clock so that loop timing, timeouts and failsafe
// deadlines are all deterministic and instantly testable.
class Clock {
public:
    virtual ~Clock() = default;

    // Microseconds since boot. Must be monotonic. Wrap-around is handled by
    // callers using unsigned subtraction, so a 32-bit counter is acceptable.
    virtual uint32_t micros() const = 0;

    virtual void delay_us(uint32_t us) = 0;

    uint32_t millis() const { return micros() / 1000u; }
};

// Unsigned-safe elapsed time. Correct across a 32-bit wrap.
inline uint32_t elapsed_us(uint32_t now, uint32_t since) {
    return static_cast<uint32_t>(now - since);
}

}  // namespace fc
