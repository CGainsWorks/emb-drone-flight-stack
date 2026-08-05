#pragma once

#include <cstddef>
#include <cstdint>

namespace fc {

// CRSF (Crossfire / ExpressLRS) serial protocol.
//
// On FCSTMV1.05 the control receiver is on USART1 (nets /CRX_UART1_TX and
// /CRX_UART1_RX, F405 pins PB6/PB7). The same protocol carries telemetry back
// up the link, so one codec serves both directions.
//
// Frame layout:  [address] [length] [type] [payload ...] [crc8]
// `length` counts type + payload + crc. The CRC is DVB-S2 (poly 0xD5) taken
// over type + payload.
namespace crsf {

inline constexpr uint8_t kAddressFlightController = 0xC8;
inline constexpr uint8_t kAddressRadioTransmitter = 0xEA;

inline constexpr uint8_t kFrameTypeRcChannelsPacked = 0x16;
inline constexpr uint8_t kFrameTypeBatterySensor = 0x08;
inline constexpr uint8_t kFrameTypeAttitude = 0x1E;

inline constexpr std::size_t kChannelCount = 16;
inline constexpr std::size_t kRcPayloadBytes = 22;  // 16 channels x 11 bits

// Maximum bytes after the length field.
inline constexpr std::size_t kMaxFrameLength = 62;
inline constexpr std::size_t kMaxFrameSize = kMaxFrameLength + 2;

// Raw channel scale used by CRSF.
inline constexpr uint16_t kChannelMin = 172;
inline constexpr uint16_t kChannelMid = 992;
inline constexpr uint16_t kChannelMax = 1811;

uint8_t crc8(const uint8_t* data, std::size_t len);

struct RcChannels {
    uint16_t channel[kChannelCount]{};
};

// Map a raw channel to -1..+1 (for roll/pitch/yaw sticks).
float to_bipolar(uint16_t raw);
// Map a raw channel to 0..1 (for throttle).
float to_unipolar(uint16_t raw);
// True if the channel is in the upper half of its travel (2-position switch).
bool is_switch_high(uint16_t raw);

// Incremental, byte-at-a-time frame parser.
//
// Written to be resynchronising: a corrupt or partial frame must never wedge
// the parser, because on a real link that would mean permanent loss of control
// input rather than a single dropped frame.
class Parser {
public:
    // Feed one received byte. Returns true if a complete, CRC-valid RC channel
    // frame became available on this byte.
    bool push(uint8_t byte);

    // Feed a buffer. Returns the number of complete RC frames decoded.
    std::size_t push(const uint8_t* data, std::size_t len);

    const RcChannels& channels() const { return channels_; }
    bool has_channels() const { return has_channels_; }

    // Consume the pending channel set. Returns false if none is pending.
    bool take(RcChannels& out);

    // Call when the serial line has gone idle between bursts (on STM32, from
    // the USART IDLE-line interrupt).
    //
    // This matters: CRSF frames arrive as bursts separated by idle gaps. If a
    // frame is truncated by noise, a pure byte-stream parser keeps counting and
    // swallows the *next* frame as the tail of the broken one, which can
    // cascade into repeated loss of control input. Resetting on the idle gap
    // bounds the damage to the single corrupted frame.
    void notify_idle();

    void reset();

    uint32_t frames_decoded() const { return frames_decoded_; }
    uint32_t crc_errors() const { return crc_errors_; }
    uint32_t length_errors() const { return length_errors_; }
    // Frames abandoned part-way because the line went idle.
    uint32_t truncated_frames() const { return truncated_frames_; }

private:
    enum class State : uint8_t { WaitAddress, WaitLength, Collect };

    State state_{State::WaitAddress};
    uint8_t buf_[kMaxFrameSize]{};
    std::size_t expected_{0};
    std::size_t collected_{0};

    RcChannels channels_{};
    bool has_channels_{false};

    uint32_t frames_decoded_{0};
    uint32_t crc_errors_{0};
    uint32_t length_errors_{0};
    uint32_t truncated_frames_{0};

    bool handle_complete_frame();
};

// Unpack 22 bytes of packed 11-bit channels.
void unpack_channels(const uint8_t* payload, RcChannels& out);
// Inverse of unpack_channels, used by the tests and by any loopback tooling.
void pack_channels(const RcChannels& in, uint8_t* payload);

// --- telemetry builders -------------------------------------------------
// Each returns the number of bytes written, or 0 if the buffer is too small.

std::size_t build_attitude_frame(float pitch_rad, float roll_rad, float yaw_rad,
                                 uint8_t* out, std::size_t out_len);

std::size_t build_battery_frame(float voltage_v, float current_a, uint32_t used_mah,
                                uint8_t remaining_pct, uint8_t* out, std::size_t out_len);

}  // namespace crsf
}  // namespace fc
