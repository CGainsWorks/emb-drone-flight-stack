// UBX parser and GPS receiver against simulated NAV-PVT output
// (sim/sensors/sim_ubx_gps.hpp). On FCSTMV1.05 the SAM-M10Q's UART does not
// reach the F405 (docs/PINOUT.md), so this is where the code is exercised.
#include <cmath>
#include <vector>

#include "fc/drivers/ubx.hpp"
#include "mocks/mock_spi.hpp"
#include "sim_ubx_gps.hpp"
#include "test_framework.hpp"

using namespace fc;
using fc::sim::GpsTruth;
using fc::sim::SimUbxGps;
using fc::testing_mocks::MockUart;

namespace {

void feed(MockUart& uart, const std::vector<uint8_t>& bytes) {
    uart.feed(bytes.data(), bytes.size());
}

GpsTruth somewhere() {
    // An arbitrary point and a gentle north-easterly drift.
    GpsTruth t;
    t.lat_deg = 51.4801234;
    t.lon_deg = -2.5912345;
    t.hmsl_m = 62.4;
    t.vel_n = 3.2;
    t.vel_e = 1.1;
    t.vel_d = -0.4;
    return t;
}

}  // namespace

TEST(ubx_checksum_matches_the_published_mon_ver_poll) {
    // B5 62 0A 04 00 00 0E 34: the MON-VER poll from the u-blox protocol docs.
    uint8_t out[8];
    CHECK_EQ(ubx::build_frame(0x0A, 0x04, nullptr, 0, out, sizeof(out)), 8u);
    const uint8_t expected[8] = {0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34};
    for (int i = 0; i < 8; ++i) {
        CHECK_EQ(out[i], expected[i]);
    }
}

TEST(ubx_build_frame_refuses_a_short_buffer) {
    uint8_t payload[4] = {1, 2, 3, 4};
    uint8_t out[11];
    CHECK_EQ(ubx::build_frame(0x06, 0x8A, payload, 4, out, sizeof(out)), 0u);
}

TEST(ubx_parser_agrees_with_the_independent_simulator) {
    const auto f = SimUbxGps::nav_pvt_frame(somewhere(), 1000);
    ubx::Parser p;
    int complete = 0;
    for (uint8_t b : f) {
        complete += p.push(b) ? 1 : 0;
    }
    CHECK_EQ(complete, 1);
    CHECK_EQ(p.msg_class(), ubx::kClassNav);
    CHECK_EQ(p.msg_id(), ubx::kIdNavPvt);
    CHECK_EQ(p.length(), ubx::kNavPvtLength);
    CHECK_EQ(p.checksum_errors(), 0u);
}

TEST(gps_decodes_position_velocity_and_quality) {
    MockUart uart;
    GpsReceiver gps(uart);
    const GpsTruth t = somewhere();
    feed(uart, SimUbxGps::nav_pvt_frame(t, 345600));
    CHECK(gps.update());
    CHECK(gps.has_3d_fix());
    const auto& s = gps.latest();
    CHECK_EQ(s.itow_ms, 345600u);
    CHECK_EQ(s.num_sv, 14);
    CHECK_EQ(s.lat_e7, 514801234);
    CHECK_EQ(s.lon_e7, -25912345);
    CHECK_EQ(s.hmsl_mm, 62400);
    CHECK_EQ(s.height_mm, 109400);
    CHECK_EQ(s.vel_n_mm_s, 3200);
    CHECK_EQ(s.vel_e_mm_s, 1100);
    CHECK_EQ(s.vel_d_mm_s, -400);
    CHECK_EQ(s.h_acc_mm, 1200u);
    CHECK_EQ(s.v_acc_mm, 2000u);
    CHECK_EQ(s.pdop_e2, 130);
}

TEST(gps_ignores_nmea_and_noise_between_frames) {
    MockUart uart;
    GpsReceiver gps(uart);
    feed(uart, SimUbxGps::nmea_noise());
    const std::vector<uint8_t> junk = {0x00, 0xB5, 0x00, 0xB5, 0xB5, 0x13, 0xFF};
    feed(uart, junk);
    feed(uart, SimUbxGps::nav_pvt_frame(somewhere(), 2000));
    feed(uart, SimUbxGps::nmea_noise());
    CHECK(gps.update());
    CHECK_EQ(gps.solutions(), 1u);
    CHECK_EQ(gps.latest().itow_ms, 2000u);
}

TEST(gps_rejects_a_corrupted_frame_and_recovers) {
    MockUart uart;
    GpsReceiver gps(uart);
    auto bad = SimUbxGps::nav_pvt_frame(somewhere(), 3000);
    bad[40] ^= 0x10;  // flip a payload bit, as line noise would
    feed(uart, bad);
    CHECK(!gps.update());
    CHECK(!gps.has_solution());
    CHECK_EQ(gps.parser().checksum_errors(), 1u);

    feed(uart, SimUbxGps::nav_pvt_frame(somewhere(), 3200));
    CHECK(gps.update());
    CHECK_EQ(gps.latest().itow_ms, 3200u);
}

TEST(gps_handles_frames_split_across_reads) {
    MockUart uart;
    GpsReceiver gps(uart);
    const auto f = SimUbxGps::nav_pvt_frame(somewhere(), 4000);
    // Arrive in awkward pieces, with an update between each.
    std::size_t pos = 0;
    const std::size_t chunks[] = {1, 5, 37, 2, 50, 5};
    int fresh = 0;
    for (std::size_t c : chunks) {
        uart.feed(&f[pos], c);
        pos += c;
        fresh += gps.update() ? 1 : 0;
    }
    CHECK_EQ(pos, f.size());
    CHECK_EQ(fresh, 1);
    CHECK_EQ(gps.latest().itow_ms, 4000u);
}

TEST(gps_does_not_report_a_fix_the_receiver_has_not_validated) {
    MockUart uart;
    GpsReceiver gps(uart);
    GpsTruth t = somewhere();
    t.fix_type = 3;
    t.fix_ok = false;
    feed(uart, SimUbxGps::nav_pvt_frame(t, 5000));
    CHECK(gps.update());
    CHECK(gps.has_solution());
    CHECK(!gps.has_3d_fix());

    t.fix_ok = true;
    t.fix_type = 2;
    feed(uart, SimUbxGps::nav_pvt_frame(t, 5200));
    CHECK(gps.update());
    CHECK(!gps.has_3d_fix());
}

TEST(gps_skips_oversized_and_other_messages) {
    MockUart uart;
    GpsReceiver gps(uart);
    // A message bigger than the parser keeps, then an unrelated one.
    feed(uart, SimUbxGps::frame(0x0A, 0x31, std::vector<uint8_t>(300, 0x55)));
    feed(uart, SimUbxGps::frame(0x01, 0x35, std::vector<uint8_t>(20, 0x01)));
    feed(uart, SimUbxGps::nav_pvt_frame(somewhere(), 6000));
    CHECK(gps.update());
    CHECK_EQ(gps.solutions(), 1u);
    CHECK_EQ(gps.parser().oversize(), 1u);
}

TEST(gps_tracks_a_simulated_trajectory) {
    MockUart uart;
    GpsReceiver gps(uart);
    GpsTruth t = somewhere();
    t.vel_n = 5.0;
    t.vel_e = 0.0;
    const double metres_per_deg_lat = 111320.0;
    for (int k = 0; k < 50; ++k) {  // 10 s at 5 Hz
        feed(uart, SimUbxGps::nav_pvt_frame(t, static_cast<uint32_t>(k * 200)));
        CHECK(gps.update());
        const double lat = gps.latest().lat_e7 * 1e-7;
        CHECK_NEAR((lat - t.lat_deg) * metres_per_deg_lat, 0.0, 0.02);
        t.lat_deg += t.vel_n * 0.2 / metres_per_deg_lat;
    }
    CHECK_EQ(gps.solutions(), 50u);
}

TEST_MAIN()
