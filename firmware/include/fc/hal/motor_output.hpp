#pragma once

#include <cstddef>
#include <cstdint>

namespace fc {

// Number of motor channels on FCSTMV1.05 (nets /S1../S4).
inline constexpr std::size_t kMotorCount = 4;

// Abstract ESC output stage. The concrete implementation is either DShot
// (timer + DMA) or classic PWM; the control loop above is written against this
// interface only, so both are interchangeable and neither is needed on host.
class MotorOutput {
public:
    virtual ~MotorOutput() = default;

    // Throttle values are normalised 0..1 per motor. Implementations map that
    // onto their own wire format (DShot 48..2047, or 1000..2000 us PWM).
    virtual void write(const float throttles[kMotorCount]) = 0;

    // Immediately command all motors to idle/stop. Must be safe to call from a
    // failsafe path at any time.
    virtual void disarm() = 0;
};

}  // namespace fc
