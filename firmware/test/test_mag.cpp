// MMC5983MA driver against the simulated device in sim/sensors/sim_mmc5983.hpp,
// plus tilt-compensated heading. On FCSTMV1.05 this magnetometer does not reach
// the F405 (docs/PINOUT.md), so these tests are the only place the driver runs.
#include <cmath>

#include "fc/drivers/mmc5983.hpp"
#include "mocks/mock_spi.hpp"
#include "quad_plant.hpp"
#include "sim_mmc5983.hpp"
#include "test_framework.hpp"

using namespace fc;
using fc::sim::SimMmc5983;
using fc::testing_mocks::FakeClock;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;
constexpr float kOneCount = 1.0f / 16384.0f;

// Illustrative mid-latitude field in a magnetic-north NED frame: 0.19 G
// horizontal, 0.45 G pointing down. Not a survey value for any location.
const fc::sil::V3 kEarthField{0.19, 0.0, 0.45};

struct Rig {
    SimMmc5983 dev;
    FakeClock clock;
    Mmc5983 mag{dev, clock};

    void set_field(const fc::sil::V3& b) {
        dev.field_gauss[0] = b.x;
        dev.field_gauss[1] = b.y;
        dev.field_gauss[2] = b.z;
    }
};

}  // namespace

TEST(mag_decode_unpacks_18_bit_samples) {
    const uint8_t raw[7] = {0xFF, 0xFF, 0x00, 0x00, 0x80, 0x00, 0xC4};
    int32_t c[3];
    mmc5983::decode(raw, c);
    CHECK_EQ(c[0], 262143);
    CHECK_EQ(c[1], 0);
    CHECK_EQ(c[2], 131073);
}

TEST(mag_init_identifies_and_measures_the_offset) {
    Rig rig;
    rig.dev.bridge_offset_counts[0] = 850;
    rig.dev.bridge_offset_counts[1] = -1200;
    rig.dev.bridge_offset_counts[2] = 40;
    rig.set_field({0.2, -0.1, 0.4});
    CHECK(rig.mag.init());
    CHECK_EQ(rig.dev.sets, 1);
    CHECK_EQ(rig.dev.resets, 1);
    CHECK_EQ(rig.mag.offset_counts()[0], 850);
    CHECK_EQ(rig.mag.offset_counts()[1], -1200);
    CHECK_EQ(rig.mag.offset_counts()[2], 40);
}

TEST(mag_read_removes_the_bridge_offset) {
    Rig rig;
    rig.dev.bridge_offset_counts[0] = 3000;
    rig.dev.bridge_offset_counts[1] = -2500;
    rig.dev.bridge_offset_counts[2] = 1800;
    rig.set_field({0.31, -0.12, 0.47});
    CHECK(rig.mag.init());
    Vec3 g;
    CHECK(rig.mag.read(g));
    CHECK_NEAR(g.x, 0.31f, kOneCount);
    CHECK_NEAR(g.y, -0.12f, kOneCount);
    CHECK_NEAR(g.z, 0.47f, kOneCount);
}

TEST(mag_recalibration_tracks_offset_drift) {
    Rig rig;
    rig.set_field({0.2, 0.0, 0.4});
    CHECK(rig.mag.init());
    // The bridge offset moves with temperature; the old offset is now wrong.
    rig.dev.bridge_offset_counts[0] = 1640;  // about 0.1 G
    Vec3 g;
    CHECK(rig.mag.read(g));
    CHECK_NEAR(g.x, 0.2f + 1640.0f / 16384.0f, kOneCount);
    CHECK(rig.mag.calibrate_offset());
    CHECK(rig.mag.read(g));
    CHECK_NEAR(g.x, 0.2f, kOneCount);
}

TEST(mag_init_fails_without_the_part) {
    Rig rig;
    rig.dev.responds = false;
    CHECK(!rig.mag.init());
    Vec3 g;
    CHECK(!rig.mag.read(g));
}

TEST(mag_init_rejects_the_wrong_part) {
    Rig rig;
    rig.dev.product_id = 0x0C;
    CHECK(!rig.mag.init());
}

TEST(mag_a_hung_conversion_times_out_instead_of_blocking) {
    Rig rig;
    CHECK(rig.mag.init());
    rig.dev.conversions_hang = true;
    const uint32_t before = rig.clock.micros();
    Vec3 g;
    CHECK(!rig.mag.read(g));
    CHECK(rig.clock.micros() - before <= 25000u);
}

TEST(mag_heading_is_recovered_through_roll_and_pitch) {
    Rig rig;
    CHECK(rig.mag.init());
    const double tilts[] = {-30.0, -12.0, 0.0, 17.0, 30.0};
    for (double yaw = -170.0; yaw <= 180.0; yaw += 10.0) {
        for (double roll : tilts) {
            for (double pitch : tilts) {
                const auto q = fc::sil::Q4::from_euler(roll * kDeg, pitch * kDeg, yaw * kDeg);
                rig.set_field(q.inverse_rotate(kEarthField));
                Vec3 g;
                CHECK(rig.mag.read(g));
                const float h = tilt_compensated_heading(g, static_cast<float>(roll * kDeg),
                                                         static_cast<float>(pitch * kDeg));
                double err = static_cast<double>(h) - yaw * kDeg;
                err = std::atan2(std::sin(err), std::cos(err));
                CHECK_NEAR(err, 0.0, 0.3 * kDeg);
            }
        }
    }
}

TEST_MAIN()
