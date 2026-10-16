// BMP390 driver against the simulated device in sim/sensors/sim_bmp390.hpp.
// The device's trimming coefficients are synthetic; what is tested is that the
// driver speaks the register protocol and applies the datasheet compensation
// correctly, not anything about a particular physical part.
#include <cmath>

#include "fc/drivers/bmp390.hpp"
#include "mocks/mock_spi.hpp"
#include "sim_bmp390.hpp"
#include "test_framework.hpp"

using namespace fc;
using fc::sim::SimBmp390;
using fc::testing_mocks::FakeClock;

namespace {

struct Rig {
    SimBmp390 dev;
    FakeClock clock;
    Bmp390 baro{dev, clock};
};

}  // namespace

TEST(baro_init_resets_identifies_and_configures) {
    Rig rig;
    CHECK(rig.baro.init());
    CHECK(rig.baro.initialised());
    CHECK_EQ(rig.dev.soft_resets, 1);
    CHECK_EQ(rig.dev.reg(bmp390::kRegPwrCtrl), bmp390::kPwrCtrlNormal);
    CHECK_EQ(rig.dev.reg(bmp390::kRegOsr), bmp390::kOsrP8T1);
    CHECK_EQ(rig.dev.reg(bmp390::kRegOdr), bmp390::kOdr50Hz);
    CHECK_EQ(rig.dev.reg(bmp390::kRegConfig), bmp390::kConfigIir3);
    // The reset needs time before the part will talk again.
    CHECK(rig.clock.micros() >= 10000u);
}

TEST(baro_init_fails_when_csb_is_left_floating) {
    // R128 is not fitted on FCSTMV1.05: until CSB is strapped the part does
    // not acknowledge on I2C, and the driver must say so.
    Rig rig;
    rig.dev.responds = false;
    CHECK(!rig.baro.init());
    CHECK(!rig.baro.initialised());
    BaroSample s;
    CHECK(!rig.baro.read(s));
}

TEST(baro_init_rejects_the_wrong_part) {
    Rig rig;
    rig.dev.chip_id = 0x50;  // BMP388
    CHECK(!rig.baro.init());
}

TEST(baro_init_rejects_the_wrong_address) {
    Rig rig;
    rig.dev.address = 0x76;  // SDO strapped low instead
    CHECK(!rig.baro.init());
}

TEST(baro_read_before_init_fails) {
    Rig rig;
    BaroSample s;
    CHECK(!rig.baro.read(s));
}

TEST(baro_calibration_is_scaled_as_the_datasheet_specifies) {
    Rig rig;
    CHECK(rig.baro.init());
    const auto& c = rig.baro.calibration();
    CHECK_NEAR(c.t1, 27000.0 * 256.0, 1e-9);
    CHECK_NEAR(c.t2, 19000.0 / std::ldexp(1.0, 30), 1e-18);
    CHECK_NEAR(c.t3, -7.0 / std::ldexp(1.0, 48), 1e-24);
    CHECK_NEAR(c.p1, (31000.0 - 16384.0) / std::ldexp(1.0, 20), 1e-12);
    CHECK_NEAR(c.p5, 2000.0 * 8.0, 1e-9);
    CHECK_NEAR(c.p11, -10.0 / std::ldexp(1.0, 65), 1e-30);
}

TEST(baro_recovers_pressure_and_temperature_across_the_envelope) {
    Rig rig;
    CHECK(rig.baro.init());
    for (double t = -40.0; t <= 85.0; t += 12.5) {
        for (double p = 30000.0; p <= 125000.0; p += 5000.0) {
            rig.dev.temperature_c = t;
            rig.dev.pressure_pa = p;
            BaroSample s;
            CHECK(rig.baro.read(s));
            // One raw LSB is about 0.014 Pa here; float output adds ~0.008 Pa.
            CHECK_NEAR(s.pressure_pa, p, 0.05);
            CHECK_NEAR(s.temperature_c, t, 0.001);
        }
    }
}

TEST(baro_altitude_follows_the_standard_atmosphere) {
    CHECK_NEAR(pressure_to_altitude_m(101325.0f), 0.0f, 1e-3);
    // ISA: 1000 m corresponds to 89874.6 Pa.
    CHECK_NEAR(pressure_to_altitude_m(89874.6f), 1000.0f, 1.0f);
    CHECK_NEAR(pressure_to_altitude_m(0.0f), 0.0f, 1e-6);
}

TEST(baro_resolves_a_ten_metre_climb) {
    Rig rig;
    CHECK(rig.baro.init());
    // Truth from the same ISA relation, inverted.
    const auto isa_pressure = [](double h) {
        return 101325.0 * std::pow(1.0 - h / 44330.0, 1.0 / 0.190295);
    };
    rig.dev.pressure_pa = isa_pressure(120.0);
    BaroSample a;
    CHECK(rig.baro.read(a));
    rig.dev.pressure_pa = isa_pressure(130.0);
    BaroSample b;
    CHECK(rig.baro.read(b));
    const float climb = pressure_to_altitude_m(b.pressure_pa) - pressure_to_altitude_m(a.pressure_pa);
    CHECK_NEAR(climb, 10.0f, 0.1f);
}

TEST_MAIN()
