#pragma once

#include <cstddef>
#include <cstdint>

#include "fc/hal/uart.hpp"

namespace fc {

// u-blox UBX binary protocol, for the SAM-M10Q GPS (board reference ANT2).
//
// On FCSTMV1.05 the receiver's UART lands on the J129 pads only and does not
// reach USART6 (docs/PINOUT.md), so this runs on host tests until a revision
// or a jumper wires it. Only UBX-NAV-PVT is decoded: it carries everything a
// flight controller needs (fix, position, velocity, accuracy) in one message.
namespace ubx {

inline constexpr uint8_t kSync1 = 0xB5;
inline constexpr uint8_t kSync2 = 0x62;
inline constexpr uint8_t kClassNav = 0x01;
inline constexpr uint8_t kIdNavPvt = 0x07;
inline constexpr uint16_t kNavPvtLength = 92;
inline constexpr std::size_t kMaxPayload = 256;

// 8-bit Fletcher checksum over class, id, length and payload.
void checksum(const uint8_t* data, std::size_t len, uint8_t& ck_a, uint8_t& ck_b);

// Build a complete frame. Returns its length, or 0 if out is too small.
std::size_t build_frame(uint8_t msg_class, uint8_t msg_id, const uint8_t* payload,
                        uint16_t len, uint8_t* out, std::size_t out_len);

enum class FixType : uint8_t {
    None = 0,
    DeadReckoning = 1,
    Fix2D = 2,
    Fix3D = 3,
    GnssDeadReckoning = 4,
    TimeOnly = 5,
};

struct NavPvt {
    uint32_t itow_ms{0};
    FixType fix_type{FixType::None};
    bool gnss_fix_ok{false};
    uint8_t num_sv{0};
    int32_t lon_e7{0};       // degrees * 1e7
    int32_t lat_e7{0};
    int32_t height_mm{0};    // above ellipsoid
    int32_t hmsl_mm{0};      // above mean sea level
    uint32_t h_acc_mm{0};
    uint32_t v_acc_mm{0};
    int32_t vel_n_mm_s{0};
    int32_t vel_e_mm_s{0};
    int32_t vel_d_mm_s{0};
    uint32_t s_acc_mm_s{0};
    uint16_t pdop_e2{0};     // 0.01 units
};

bool decode_nav_pvt(const uint8_t* payload, std::size_t len, NavPvt& out);

// Byte-at-a-time frame parser. Anything that is not a well-formed UBX frame
// (NMEA sentences, line noise, a truncated frame) is skipped.
class Parser {
public:
    // Returns true when a complete frame with a valid checksum has arrived.
    bool push(uint8_t byte);

    uint8_t msg_class() const { return class_; }
    uint8_t msg_id() const { return id_; }
    const uint8_t* payload() const { return payload_; }
    uint16_t length() const { return length_; }

    uint32_t frames() const { return frames_; }
    uint32_t checksum_errors() const { return checksum_errors_; }
    uint32_t oversize() const { return oversize_; }

private:
    enum class State : uint8_t { Sync1, Sync2, Class, Id, Len1, Len2, Payload, CkA, CkB };

    State state_{State::Sync1};
    uint8_t class_{0};
    uint8_t id_{0};
    uint16_t length_{0};
    uint16_t index_{0};
    uint8_t ck_a_{0};
    uint8_t ck_b_{0};
    uint8_t rx_ck_a_{0};
    uint8_t payload_[kMaxPayload]{};

    uint32_t frames_{0};
    uint32_t checksum_errors_{0};
    uint32_t oversize_{0};

    void add(uint8_t b);
};

}  // namespace ubx

// Drains a UART, keeps the most recent NAV-PVT solution.
class GpsReceiver {
public:
    explicit GpsReceiver(Uart& uart) : uart_(uart) {}

    // Non-blocking: consumes whatever bytes are waiting. Returns true if a new
    // NAV-PVT solution arrived.
    bool update();

    const ubx::NavPvt& latest() const { return latest_; }
    bool has_solution() const { return solutions_ > 0; }
    // A 3D fix the receiver itself flags as valid.
    bool has_3d_fix() const {
        return has_solution() && latest_.gnss_fix_ok && latest_.fix_type == ubx::FixType::Fix3D;
    }
    uint32_t solutions() const { return solutions_; }
    const ubx::Parser& parser() const { return parser_; }

private:
    Uart& uart_;
    ubx::Parser parser_;
    ubx::NavPvt latest_{};
    uint32_t solutions_{0};
};

}  // namespace fc
