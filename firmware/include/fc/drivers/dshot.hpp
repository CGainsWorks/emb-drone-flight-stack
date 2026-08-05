#pragma once

#include <cstddef>
#include <cstdint>

namespace fc {

// DShot digital ESC protocol.
//
// A DShot frame is 16 bits: an 11-bit value, one telemetry-request bit, and a
// 4-bit CRC. Values 1..47 are reserved commands; 48..2047 are throttle.
//
// The encoding here is pure arithmetic with no hardware dependency, so the
// exact bits that will be clocked out to the ESCs can be verified on a host
// long before a board exists.
namespace dshot {

inline constexpr uint16_t kCmdMotorStop = 0;
inline constexpr uint16_t kMinThrottle = 48;
inline constexpr uint16_t kMaxThrottle = 2047;

// Bits per DShot frame.
inline constexpr std::size_t kFrameBits = 16;

enum class Rate : uint32_t {
    Dshot150 = 150000,
    Dshot300 = 300000,
    Dshot600 = 600000,
    Dshot1200 = 1200000,
};

// 4-bit CRC over the 12-bit payload (11-bit value + telemetry bit).
uint8_t crc4(uint16_t payload12);

// Build the full 16-bit frame. `value11` is masked to 11 bits.
uint16_t make_frame(uint16_t value11, bool request_telemetry);

// Map a normalised 0..1 throttle onto the DShot throttle range.
// Values <= 0 produce kCmdMotorStop so a zero command is a genuine stop.
uint16_t throttle_to_value(float normalised);

// Timer compare values for one DShot bit, derived from the timer clock.
struct Timing {
    uint32_t period_counts{0};  // full bit period
    uint32_t t0h_counts{0};     // high time for a 0 bit
    uint32_t t1h_counts{0};     // high time for a 1 bit
};

// DShot signals a 0 with ~37.5% duty and a 1 with ~75% duty.
Timing compute_timing(uint32_t timer_clock_hz, Rate rate);

// Expand a frame into per-bit compare values for a timer+DMA burst.
// `out` must have room for kFrameBits entries; returns the number written.
std::size_t encode_frame(uint16_t frame, const Timing& timing, uint16_t* out,
                         std::size_t out_len);

}  // namespace dshot
}  // namespace fc
