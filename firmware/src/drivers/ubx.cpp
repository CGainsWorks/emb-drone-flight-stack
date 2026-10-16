#include "fc/drivers/ubx.hpp"

namespace fc {
namespace ubx {
namespace {

uint16_t le16(const uint8_t* p) {
    return static_cast<uint16_t>(static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8));
}

uint32_t le32(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

int32_t le32s(const uint8_t* p) { return static_cast<int32_t>(le32(p)); }

}  // namespace

void checksum(const uint8_t* data, std::size_t len, uint8_t& ck_a, uint8_t& ck_b) {
    uint8_t a = 0;
    uint8_t b = 0;
    for (std::size_t i = 0; i < len; ++i) {
        a = static_cast<uint8_t>(a + data[i]);
        b = static_cast<uint8_t>(b + a);
    }
    ck_a = a;
    ck_b = b;
}

std::size_t build_frame(uint8_t msg_class, uint8_t msg_id, const uint8_t* payload,
                        uint16_t len, uint8_t* out, std::size_t out_len) {
    const std::size_t total = static_cast<std::size_t>(len) + 8u;
    if (out_len < total) {
        return 0;
    }
    out[0] = kSync1;
    out[1] = kSync2;
    out[2] = msg_class;
    out[3] = msg_id;
    out[4] = static_cast<uint8_t>(len & 0xFFu);
    out[5] = static_cast<uint8_t>((len >> 8) & 0xFFu);
    for (std::size_t i = 0; i < len; ++i) {
        out[6 + i] = payload[i];
    }
    checksum(&out[2], static_cast<std::size_t>(len) + 4u, out[6 + len], out[7 + len]);
    return total;
}

bool decode_nav_pvt(const uint8_t* p, std::size_t len, NavPvt& out) {
    if (len != kNavPvtLength) {
        return false;
    }
    out.itow_ms = le32(&p[0]);
    out.fix_type = static_cast<FixType>(p[20] <= 5 ? p[20] : 0);
    out.gnss_fix_ok = (p[21] & 0x01u) != 0u;
    out.num_sv = p[23];
    out.lon_e7 = le32s(&p[24]);
    out.lat_e7 = le32s(&p[28]);
    out.height_mm = le32s(&p[32]);
    out.hmsl_mm = le32s(&p[36]);
    out.h_acc_mm = le32(&p[40]);
    out.v_acc_mm = le32(&p[44]);
    out.vel_n_mm_s = le32s(&p[48]);
    out.vel_e_mm_s = le32s(&p[52]);
    out.vel_d_mm_s = le32s(&p[56]);
    out.s_acc_mm_s = le32(&p[68]);
    out.pdop_e2 = le16(&p[76]);
    return true;
}

void Parser::add(uint8_t b) {
    ck_a_ = static_cast<uint8_t>(ck_a_ + b);
    ck_b_ = static_cast<uint8_t>(ck_b_ + ck_a_);
}

bool Parser::push(uint8_t byte) {
    switch (state_) {
        case State::Sync1:
            if (byte == kSync1) {
                state_ = State::Sync2;
            }
            return false;
        case State::Sync2:
            if (byte == kSync2) {
                state_ = State::Class;
                ck_a_ = 0;
                ck_b_ = 0;
            } else {
                state_ = (byte == kSync1) ? State::Sync2 : State::Sync1;
            }
            return false;
        case State::Class:
            class_ = byte;
            add(byte);
            state_ = State::Id;
            return false;
        case State::Id:
            id_ = byte;
            add(byte);
            state_ = State::Len1;
            return false;
        case State::Len1:
            length_ = byte;
            add(byte);
            state_ = State::Len2;
            return false;
        case State::Len2:
            length_ = static_cast<uint16_t>(length_ | (static_cast<uint16_t>(byte) << 8));
            add(byte);
            index_ = 0;
            if (length_ > kMaxPayload) {
                // Not something this parser keeps; drop it and resync.
                ++oversize_;
                state_ = State::Sync1;
            } else {
                state_ = (length_ == 0) ? State::CkA : State::Payload;
            }
            return false;
        case State::Payload:
            payload_[index_++] = byte;
            add(byte);
            if (index_ >= length_) {
                state_ = State::CkA;
            }
            return false;
        case State::CkA:
            rx_ck_a_ = byte;
            state_ = State::CkB;
            return false;
        case State::CkB:
            state_ = State::Sync1;
            if (rx_ck_a_ == ck_a_ && byte == ck_b_) {
                ++frames_;
                return true;
            }
            ++checksum_errors_;
            return false;
    }
    state_ = State::Sync1;
    return false;
}

}  // namespace ubx

bool GpsReceiver::update() {
    bool fresh = false;
    uint8_t buf[64];
    std::size_t n = uart_.read(buf, sizeof(buf));
    while (n > 0) {
        for (std::size_t i = 0; i < n; ++i) {
            if (parser_.push(buf[i]) && parser_.msg_class() == ubx::kClassNav &&
                parser_.msg_id() == ubx::kIdNavPvt) {
                ubx::NavPvt pvt;
                if (ubx::decode_nav_pvt(parser_.payload(), parser_.length(), pvt)) {
                    latest_ = pvt;
                    ++solutions_;
                    fresh = true;
                }
            }
        }
        n = uart_.read(buf, sizeof(buf));
    }
    return fresh;
}

}  // namespace fc
