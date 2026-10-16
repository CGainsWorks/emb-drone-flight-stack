#pragma once

// Simulated u-blox receiver output, for host tests only.
//
// Encodes a true position and velocity as UBX-NAV-PVT frames, laid out per the
// u-blox M10 interface description. The checksum is computed here
// independently of the driver, so a driver checksum bug cannot be hidden by a
// matching bug in the simulator.

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace fc {
namespace sim {

struct GpsTruth {
    double lat_deg{0.0};
    double lon_deg{0.0};
    double hmsl_m{0.0};
    double geoid_sep_m{47.0};   // ellipsoid height = hmsl + this
    double vel_n{0.0};          // m/s
    double vel_e{0.0};
    double vel_d{0.0};
    uint8_t fix_type{3};
    bool fix_ok{true};
    uint8_t num_sv{14};
    double h_acc_m{1.2};
    double v_acc_m{2.0};
    double s_acc_m_s{0.2};
    double pdop{1.3};
};

class SimUbxGps {
public:
    static void put32(std::vector<uint8_t>& p, std::size_t at, uint32_t v) {
        for (int i = 0; i < 4; ++i) {
            p[at + static_cast<std::size_t>(i)] = static_cast<uint8_t>((v >> (8 * i)) & 0xFFu);
        }
    }

    static void put32s(std::vector<uint8_t>& p, std::size_t at, double v) {
        put32(p, at, static_cast<uint32_t>(static_cast<int32_t>(std::llround(v))));
    }

    static std::vector<uint8_t> nav_pvt_payload(const GpsTruth& t, uint32_t itow_ms) {
        std::vector<uint8_t> p(92, 0);
        put32(p, 0, itow_ms);
        p[20] = t.fix_type;
        p[21] = t.fix_ok ? 0x01 : 0x00;
        p[23] = t.num_sv;
        put32s(p, 24, t.lon_deg * 1e7);
        put32s(p, 28, t.lat_deg * 1e7);
        put32s(p, 32, (t.hmsl_m + t.geoid_sep_m) * 1000.0);
        put32s(p, 36, t.hmsl_m * 1000.0);
        put32(p, 40, static_cast<uint32_t>(std::llround(t.h_acc_m * 1000.0)));
        put32(p, 44, static_cast<uint32_t>(std::llround(t.v_acc_m * 1000.0)));
        put32s(p, 48, t.vel_n * 1000.0);
        put32s(p, 52, t.vel_e * 1000.0);
        put32s(p, 56, t.vel_d * 1000.0);
        put32s(p, 60, std::sqrt(t.vel_n * t.vel_n + t.vel_e * t.vel_e) * 1000.0);
        put32(p, 68, static_cast<uint32_t>(std::llround(t.s_acc_m_s * 1000.0)));
        const auto pdop = static_cast<uint16_t>(std::lround(t.pdop * 100.0));
        p[76] = static_cast<uint8_t>(pdop & 0xFFu);
        p[77] = static_cast<uint8_t>(pdop >> 8);
        return p;
    }

    static std::vector<uint8_t> frame(uint8_t cls, uint8_t id, const std::vector<uint8_t>& payload) {
        std::vector<uint8_t> f;
        f.push_back(0xB5);
        f.push_back(0x62);
        f.push_back(cls);
        f.push_back(id);
        f.push_back(static_cast<uint8_t>(payload.size() & 0xFFu));
        f.push_back(static_cast<uint8_t>((payload.size() >> 8) & 0xFFu));
        f.insert(f.end(), payload.begin(), payload.end());
        unsigned a = 0;
        unsigned b = 0;
        for (std::size_t i = 2; i < f.size(); ++i) {
            a = (a + f[i]) & 0xFFu;
            b = (b + a) & 0xFFu;
        }
        f.push_back(static_cast<uint8_t>(a));
        f.push_back(static_cast<uint8_t>(b));
        return f;
    }

    static std::vector<uint8_t> nav_pvt_frame(const GpsTruth& t, uint32_t itow_ms) {
        return frame(0x01, 0x07, nav_pvt_payload(t, itow_ms));
    }

    // Plain text the receiver might also be emitting on the same port.
    static std::vector<uint8_t> nmea_noise() {
        const std::string s = "$GNTXT,01,01,02,simulated receiver*00\r\n";
        return std::vector<uint8_t>(s.begin(), s.end());
    }
};

}  // namespace sim
}  // namespace fc
