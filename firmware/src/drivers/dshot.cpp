#include "fc/drivers/dshot.hpp"

namespace fc {
namespace dshot {

uint8_t crc4(uint16_t payload12) {
    const uint16_t v = static_cast<uint16_t>(payload12 & 0x0FFFu);
    const uint16_t c = static_cast<uint16_t>(v ^ (v >> 4) ^ (v >> 8));
    return static_cast<uint8_t>(c & 0x0Fu);
}

uint16_t make_frame(uint16_t value11, bool request_telemetry) {
    const uint16_t value = static_cast<uint16_t>(value11 & 0x07FFu);
    const uint16_t payload =
        static_cast<uint16_t>((value << 1) | (request_telemetry ? 1u : 0u));
    return static_cast<uint16_t>((payload << 4) | crc4(payload));
}

uint16_t throttle_to_value(float normalised) {
    if (!(normalised > 0.0f)) {
        return kCmdMotorStop;
    }
    if (normalised >= 1.0f) {
        return kMaxThrottle;
    }
    const float span = static_cast<float>(kMaxThrottle - kMinThrottle);
    const float scaled = static_cast<float>(kMinThrottle) + normalised * span;
    return static_cast<uint16_t>(scaled + 0.5f);
}

Timing compute_timing(uint32_t timer_clock_hz, Rate rate) {
    Timing t;
    const uint32_t bitrate = static_cast<uint32_t>(rate);
    if (bitrate == 0u || timer_clock_hz == 0u) {
        return t;
    }
    t.period_counts = timer_clock_hz / bitrate;
    // 0 bit ~ 37.5% duty, 1 bit ~ 75% duty.
    t.t0h_counts = (t.period_counts * 3u) / 8u;
    t.t1h_counts = (t.period_counts * 3u) / 4u;
    return t;
}

std::size_t encode_frame(uint16_t frame, const Timing& timing, uint16_t* out,
                         std::size_t out_len) {
    if (out == nullptr || out_len < kFrameBits) {
        return 0;
    }
    // MSB first on the wire.
    for (std::size_t i = 0; i < kFrameBits; ++i) {
        const bool bit_set = (frame & (0x8000u >> i)) != 0u;
        out[i] = static_cast<uint16_t>(bit_set ? timing.t1h_counts : timing.t0h_counts);
    }
    return kFrameBits;
}

}  // namespace dshot
}  // namespace fc
