#include "fc/drivers/crsf.hpp"

#include <cmath>
#include <cstring>

namespace fc {
namespace crsf {
namespace {

// DVB-S2 CRC-8, polynomial 0xD5, computed bitwise. A table would be faster but
// this runs once per frame at a few hundred Hz, so clarity wins.
uint8_t crc8_byte(uint8_t crc, uint8_t data) {
    crc = static_cast<uint8_t>(crc ^ data);
    for (int i = 0; i < 8; ++i) {
        if ((crc & 0x80u) != 0u) {
            crc = static_cast<uint8_t>((crc << 1) ^ 0xD5u);
        } else {
            crc = static_cast<uint8_t>(crc << 1);
        }
    }
    return crc;
}

void write_be16(uint8_t* p, int16_t v) {
    const uint16_t u = static_cast<uint16_t>(v);
    p[0] = static_cast<uint8_t>((u >> 8) & 0xFFu);
    p[1] = static_cast<uint8_t>(u & 0xFFu);
}

void write_be16u(uint8_t* p, uint16_t u) {
    p[0] = static_cast<uint8_t>((u >> 8) & 0xFFu);
    p[1] = static_cast<uint8_t>(u & 0xFFu);
}

float clampf(float v, float lo, float hi) {
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

// Assemble address/length/type/payload/crc into out.
std::size_t build_frame(uint8_t type, const uint8_t* payload, std::size_t payload_len,
                        uint8_t* out, std::size_t out_len) {
    const std::size_t total = payload_len + 4;  // addr + len + type + payload + crc
    if (out == nullptr || out_len < total) {
        return 0;
    }
    out[0] = kAddressRadioTransmitter;
    out[1] = static_cast<uint8_t>(payload_len + 2);  // type + payload + crc
    out[2] = type;
    if (payload_len > 0 && payload != nullptr) {
        std::memcpy(&out[3], payload, payload_len);
    }
    out[3 + payload_len] = crc8(&out[2], payload_len + 1);
    return total;
}

}  // namespace

uint8_t crc8(const uint8_t* data, std::size_t len) {
    uint8_t crc = 0;
    for (std::size_t i = 0; i < len; ++i) {
        crc = crc8_byte(crc, data[i]);
    }
    return crc;
}

float to_bipolar(uint16_t raw) {
    const float span = static_cast<float>(kChannelMax - kChannelMin) * 0.5f;
    const float centred = static_cast<float>(raw) - static_cast<float>(kChannelMid);
    return clampf(centred / span, -1.0f, 1.0f);
}

float to_unipolar(uint16_t raw) {
    const float span = static_cast<float>(kChannelMax - kChannelMin);
    const float v = (static_cast<float>(raw) - static_cast<float>(kChannelMin)) / span;
    return clampf(v, 0.0f, 1.0f);
}

bool is_switch_high(uint16_t raw) { return raw > kChannelMid; }

void unpack_channels(const uint8_t* payload, RcChannels& out) {
    uint32_t bits = 0;
    uint8_t bit_count = 0;
    std::size_t channel = 0;

    for (std::size_t i = 0; i < kRcPayloadBytes && channel < kChannelCount; ++i) {
        bits |= static_cast<uint32_t>(payload[i]) << bit_count;
        bit_count = static_cast<uint8_t>(bit_count + 8u);
        while (bit_count >= 11u && channel < kChannelCount) {
            out.channel[channel++] = static_cast<uint16_t>(bits & 0x07FFu);
            bits >>= 11;
            bit_count = static_cast<uint8_t>(bit_count - 11u);
        }
    }
}

void pack_channels(const RcChannels& in, uint8_t* payload) {
    std::memset(payload, 0, kRcPayloadBytes);
    uint32_t bits = 0;
    uint8_t bit_count = 0;
    std::size_t out_index = 0;

    for (std::size_t ch = 0; ch < kChannelCount; ++ch) {
        bits |= static_cast<uint32_t>(in.channel[ch] & 0x07FFu) << bit_count;
        bit_count = static_cast<uint8_t>(bit_count + 11u);
        while (bit_count >= 8u && out_index < kRcPayloadBytes) {
            payload[out_index++] = static_cast<uint8_t>(bits & 0xFFu);
            bits >>= 8;
            bit_count = static_cast<uint8_t>(bit_count - 8u);
        }
    }
    if (bit_count > 0u && out_index < kRcPayloadBytes) {
        payload[out_index] = static_cast<uint8_t>(bits & 0xFFu);
    }
}

void Parser::reset() {
    state_ = State::WaitAddress;
    expected_ = 0;
    collected_ = 0;
}

void Parser::notify_idle() {
    // Mid-frame when the line went quiet: the rest of that frame is never
    // arriving, so drop it rather than letting it consume the next one.
    if (state_ != State::WaitAddress) {
        ++truncated_frames_;
    }
    reset();
}

bool Parser::handle_complete_frame() {
    // buf_ holds [type][payload...][crc], length == expected_
    const uint8_t received_crc = buf_[expected_ - 1];
    const uint8_t computed = crc8(buf_, expected_ - 1);

    if (received_crc != computed) {
        ++crc_errors_;
        return false;
    }

    ++frames_decoded_;

    const uint8_t type = buf_[0];
    if (type == kFrameTypeRcChannelsPacked && (expected_ - 2) >= kRcPayloadBytes) {
        unpack_channels(&buf_[1], channels_);
        has_channels_ = true;
        return true;
    }
    return false;
}

bool Parser::push(uint8_t byte) {
    switch (state_) {
        case State::WaitAddress:
            if (byte == kAddressFlightController || byte == kAddressRadioTransmitter) {
                state_ = State::WaitLength;
            }
            return false;

        case State::WaitLength: {
            // A length outside the legal range means we synced on a byte that
            // only looked like an address. Drop back and keep hunting.
            if (byte < 2u || byte > kMaxFrameLength) {
                ++length_errors_;
                state_ = State::WaitAddress;
                return false;
            }
            expected_ = byte;
            collected_ = 0;
            state_ = State::Collect;
            return false;
        }

        case State::Collect: {
            buf_[collected_++] = byte;
            if (collected_ < expected_) {
                return false;
            }
            const bool got_channels = handle_complete_frame();
            state_ = State::WaitAddress;
            collected_ = 0;
            return got_channels;
        }
    }
    return false;
}

std::size_t Parser::push(const uint8_t* data, std::size_t len) {
    std::size_t frames = 0;
    for (std::size_t i = 0; i < len; ++i) {
        if (push(data[i])) {
            ++frames;
        }
    }
    return frames;
}

bool Parser::take(RcChannels& out) {
    if (!has_channels_) {
        return false;
    }
    out = channels_;
    has_channels_ = false;
    return true;
}

std::size_t build_attitude_frame(float pitch_rad, float roll_rad, float yaw_rad,
                                 uint8_t* out, std::size_t out_len) {
    // CRSF attitude is transmitted in units of 1/10000 radian.
    uint8_t payload[6];
    write_be16(&payload[0], static_cast<int16_t>(std::lround(pitch_rad * 10000.0f)));
    write_be16(&payload[2], static_cast<int16_t>(std::lround(roll_rad * 10000.0f)));
    write_be16(&payload[4], static_cast<int16_t>(std::lround(yaw_rad * 10000.0f)));
    return build_frame(kFrameTypeAttitude, payload, sizeof(payload), out, out_len);
}

std::size_t build_battery_frame(float voltage_v, float current_a, uint32_t used_mah,
                                uint8_t remaining_pct, uint8_t* out, std::size_t out_len) {
    // Voltage and current are in tenths; capacity is a 24-bit mAh count.
    uint8_t payload[8];
    write_be16u(&payload[0], static_cast<uint16_t>(std::lround(voltage_v * 10.0f)));
    write_be16u(&payload[2], static_cast<uint16_t>(std::lround(current_a * 10.0f)));
    payload[4] = static_cast<uint8_t>((used_mah >> 16) & 0xFFu);
    payload[5] = static_cast<uint8_t>((used_mah >> 8) & 0xFFu);
    payload[6] = static_cast<uint8_t>(used_mah & 0xFFu);
    payload[7] = remaining_pct;
    return build_frame(kFrameTypeBatterySensor, payload, sizeof(payload), out, out_len);
}

}  // namespace crsf
}  // namespace fc
