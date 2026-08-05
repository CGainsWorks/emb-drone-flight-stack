#include "fc/drivers/icm42688.hpp"
#include "mocks/mock_spi.hpp"
#include "test_framework.hpp"

using namespace fc;
using fc::testing_mocks::FakeClock;
using fc::testing_mocks::MockIcm42688;

namespace {
constexpr float kDegToRad = 0.017453292519943295f;
}

TEST(imu_init_succeeds_against_a_correct_device) {
    MockIcm42688 dev;
    FakeClock clk;
    Icm42688 imu{dev, clk};

    CHECK(imu.init(Icm42688Config{}));
    CHECK(imu.initialised());
}

TEST(imu_init_rejects_wrong_who_am_i) {
    // A dead bus, a miswired CS, or the wrong part fitted must all be caught
    // here rather than producing plausible-looking garbage in flight.
    MockIcm42688 dev;
    FakeClock clk;
    dev.set_who_am_i(0x00);

    Icm42688 imu{dev, clk};
    CHECK(!imu.init(Icm42688Config{}));
    CHECK(!imu.initialised());
}

TEST(imu_init_rejects_device_that_does_not_accept_config) {
    MockIcm42688 dev;
    FakeClock clk;
    dev.writes_ignored = true;

    Icm42688 imu{dev, clk};
    // WHO_AM_I still reads back, but configuration cannot be written, so the
    // read-back verification must fail.
    CHECK(!imu.init(Icm42688Config{}));
}

TEST(imu_init_performs_a_soft_reset) {
    MockIcm42688 dev;
    FakeClock clk;
    Icm42688 imu{dev, clk};
    CHECK(imu.init(Icm42688Config{}));
    CHECK_EQ(dev.soft_reset_count, 1);
}

TEST(imu_init_sets_low_noise_power_mode) {
    MockIcm42688 dev;
    FakeClock clk;
    Icm42688 imu{dev, clk};
    CHECK(imu.init(Icm42688Config{}));
    CHECK_EQ(static_cast<int>(dev.reg(icm42688::kRegPwrMgmt0)),
             static_cast<int>(icm42688::kPwrMgmt0LowNoise));
}

TEST(imu_init_writes_expected_range_and_odr_bits) {
    MockIcm42688 dev;
    FakeClock clk;
    Icm42688 imu{dev, clk};

    Icm42688Config cfg;
    cfg.gyro_range = GyroRange::Dps1000;   // selector 1 -> bits 7:5 = 001
    cfg.accel_range = AccelRange::G8;      // selector 1 -> bits 7:5 = 001
    cfg.odr = OutputDataRate::Hz8000;      // 0x03 in the low nibble
    CHECK(imu.init(cfg));

    CHECK_EQ(static_cast<int>(dev.reg(icm42688::kRegGyroConfig0)), 0x23);
    CHECK_EQ(static_cast<int>(dev.reg(icm42688::kRegAccelConfig0)), 0x23);
}

TEST(imu_init_waits_for_the_part_after_reset) {
    // The datasheet requires a settling delay; assert we actually consume time
    // rather than racing ahead.
    MockIcm42688 dev;
    FakeClock clk;
    Icm42688 imu{dev, clk};
    CHECK(imu.init(Icm42688Config{}));
    CHECK(clk.micros() >= 1000u);
}

TEST(imu_read_fails_before_init) {
    MockIcm42688 dev;
    FakeClock clk;
    Icm42688 imu{dev, clk};
    ImuSample s;
    CHECK(!imu.read(s));
}

TEST(imu_read_scales_accelerometer_to_g) {
    MockIcm42688 dev;
    FakeClock clk;
    Icm42688 imu{dev, clk};

    Icm42688Config cfg;
    cfg.accel_range = AccelRange::G16;
    CHECK(imu.init(cfg));

    // Full positive scale on z = +16 g at the 16 g setting.
    dev.set_accel_raw(0, 0, 32767);
    ImuSample s;
    CHECK(imu.read(s));
    CHECK_NEAR(s.accel.z, 16.0f, 0.01f);

    // A level board reads -1 g on z: raw = -1/16 * 32768 = -2048.
    dev.set_accel_raw(0, 0, -2048);
    CHECK(imu.read(s));
    CHECK_NEAR(s.accel.z, -1.0f, 0.001f);
    CHECK_NEAR(s.accel.x, 0.0f, 1e-6);
    CHECK_NEAR(s.accel.y, 0.0f, 1e-6);
}

TEST(imu_read_scales_gyroscope_to_radians_per_second) {
    MockIcm42688 dev;
    FakeClock clk;
    Icm42688 imu{dev, clk};

    Icm42688Config cfg;
    cfg.gyro_range = GyroRange::Dps2000;
    CHECK(imu.init(cfg));

    // Half positive scale = +1000 dps.
    dev.set_gyro_raw(16384, 0, 0);
    ImuSample s;
    CHECK(imu.read(s));
    CHECK_NEAR(s.gyro.x, 1000.0f * kDegToRad, 0.05f);

    // Negative full scale = -2000 dps.
    dev.set_gyro_raw(0, -32768, 0);
    CHECK(imu.read(s));
    CHECK_NEAR(s.gyro.y, -2000.0f * kDegToRad, 0.05f);
}

TEST(imu_read_respects_selected_gyro_range) {
    MockIcm42688 dev;
    FakeClock clk;
    Icm42688 imu{dev, clk};

    Icm42688Config cfg;
    cfg.gyro_range = GyroRange::Dps250;
    CHECK(imu.init(cfg));

    dev.set_gyro_raw(32767, 0, 0);
    ImuSample s;
    CHECK(imu.read(s));
    // At the 250 dps setting, full scale is 250 dps, not 2000.
    CHECK_NEAR(s.gyro.x, 250.0f * kDegToRad, 0.05f);
}

TEST(imu_read_handles_negative_two_complement_correctly) {
    MockIcm42688 dev;
    FakeClock clk;
    Icm42688 imu{dev, clk};
    CHECK(imu.init(Icm42688Config{}));

    // 0xFFFF must decode as -1, not 65535. A sign-extension bug here would
    // read as a violent rotation.
    dev.set_gyro_raw(-1, -1, -1);
    ImuSample s;
    CHECK(imu.read(s));
    CHECK(s.gyro.x < 0.0f);
    CHECK(s.gyro.y < 0.0f);
    CHECK(s.gyro.z < 0.0f);
    CHECK(std::fabs(s.gyro.x) < 0.01f);
}

TEST(imu_read_decodes_all_six_axes_independently) {
    // Guards against a register-offset or byte-order slip that would swap axes.
    MockIcm42688 dev;
    FakeClock clk;
    Icm42688 imu{dev, clk};
    CHECK(imu.init(Icm42688Config{}));

    dev.set_accel_raw(1000, 2000, 3000);
    dev.set_gyro_raw(4000, 5000, 6000);

    ImuSample s;
    CHECK(imu.read(s));

    CHECK(s.accel.x < s.accel.y);
    CHECK(s.accel.y < s.accel.z);
    CHECK(s.gyro.x < s.gyro.y);
    CHECK(s.gyro.y < s.gyro.z);

    const float accel_lsb = 16.0f / 32768.0f;
    CHECK_NEAR(s.accel.x, 1000.0f * accel_lsb, 1e-4);
    CHECK_NEAR(s.accel.y, 2000.0f * accel_lsb, 1e-4);
    CHECK_NEAR(s.accel.z, 3000.0f * accel_lsb, 1e-4);
}

TEST(imu_read_decodes_temperature) {
    MockIcm42688 dev;
    FakeClock clk;
    Icm42688 imu{dev, clk};
    CHECK(imu.init(Icm42688Config{}));

    dev.set_temp_raw(0);
    ImuSample s;
    CHECK(imu.read(s));
    CHECK_NEAR(s.temperature_c, 25.0f, 0.01f);

    dev.set_temp_raw(1325);  // ~ +10 degC above the 25 degC reference
    CHECK(imu.read(s));
    CHECK_NEAR(s.temperature_c, 35.0f, 0.2f);
}

TEST(imu_gyro_bias_is_subtracted) {
    MockIcm42688 dev;
    FakeClock clk;
    Icm42688 imu{dev, clk};
    CHECK(imu.init(Icm42688Config{}));

    dev.set_gyro_raw(1000, 1000, 1000);
    ImuSample raw;
    CHECK(imu.read(raw));

    imu.set_gyro_bias(raw.gyro);
    ImuSample corrected;
    CHECK(imu.read(corrected));

    // With the measured stationary offset removed, the output must be zero.
    CHECK_NEAR(corrected.gyro.x, 0.0f, 1e-6);
    CHECK_NEAR(corrected.gyro.y, 0.0f, 1e-6);
    CHECK_NEAR(corrected.gyro.z, 0.0f, 1e-6);
}

TEST(imu_always_brackets_transfers_with_chip_select) {
    // On a bit-banged bus, forgetting to frame a transaction with CS is an easy
    // and very confusing bug. Assert the driver never does it.
    MockIcm42688 dev;
    FakeClock clk;
    Icm42688 imu{dev, clk};
    CHECK(imu.init(Icm42688Config{}));

    ImuSample s;
    for (int i = 0; i < 10; ++i) {
        CHECK(imu.read(s));
    }

    CHECK_EQ(dev.transfers_without_cs, 0);
    CHECK_EQ(dev.select_count, dev.deselect_count);
    CHECK_EQ(dev.select_count, dev.transfer_count);
}

TEST(imu_read_uses_a_single_burst_transaction) {
    // One burst per sample keeps the 8 kHz loop affordable on a bit-banged bus.
    MockIcm42688 dev;
    FakeClock clk;
    Icm42688 imu{dev, clk};
    CHECK(imu.init(Icm42688Config{}));

    const int before = dev.transfer_count;
    ImuSample s;
    CHECK(imu.read(s));
    CHECK_EQ(dev.transfer_count - before, 1);
}

TEST(imu_read_sets_the_read_flag_on_the_address_byte) {
    MockIcm42688 dev;
    FakeClock clk;
    Icm42688 imu{dev, clk};
    CHECK(imu.init(Icm42688Config{}));

    dev.read_log.clear();
    ImuSample s;
    CHECK(imu.read(s));

    // Burst should walk TEMP_DATA1 (0x1D) upward for 14 registers.
    CHECK_EQ(static_cast<int>(dev.read_log.size()), 14);
    CHECK_EQ(static_cast<int>(dev.read_log.front()), 0x1D);
    CHECK_EQ(static_cast<int>(dev.read_log.back()), 0x2A);
}

TEST_MAIN()
