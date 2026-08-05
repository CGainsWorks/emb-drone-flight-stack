// Waveform-level tests for the bit-banged SPI master.
//
// These matter more than usual on this board: the IMU cannot be driven by the
// F405 hardware SPI1 peripheral (PA6/PA7 carry the IMU's data lines the wrong
// way round for the fixed alternate-function map), so this software master is
// the only path to the gyro. It is verified here against a slave model that
// only sees pin transitions.
#include "fc/drivers/bitbang_spi.hpp"
#include "fc/drivers/icm42688.hpp"
#include "mocks/mock_spi.hpp"
#include "mocks/sim_spi_slave.hpp"
#include "test_framework.hpp"

using namespace fc;
using fc::testing_mocks::FakeClock;
using fc::testing_mocks::SimulatedIcm42688;

namespace {

struct Rig {
    SimulatedIcm42688 slave;
    BitBangSpi master{slave.sck(), slave.mosi(), slave.miso(), slave.cs()};
};

constexpr float kDegToRad = 0.017453292519943295f;

}  // namespace

TEST(bitbang_clock_idles_high_before_any_transaction) {
    Rig rig;
    // Constructing the master must park the bus: mode 3 idles the clock high
    // and leaves chip select deasserted.
    CHECK(rig.slave.sck().read());
    CHECK(rig.slave.cs().read());
    CHECK(rig.slave.transactions == 0);
}

TEST(bitbang_emits_exactly_eight_clock_pulses_per_byte) {
    Rig rig;
    const uint8_t tx[3] = {0xAA, 0x55, 0x00};
    rig.master.select();
    rig.master.transfer(tx, nullptr, sizeof(tx));
    rig.master.deselect();

    CHECK_EQ(rig.slave.rising_edges, 24);
    CHECK_EQ(rig.slave.falling_edges, 24);
}

TEST(bitbang_never_clocks_while_deselected) {
    // Clocking outside a transaction would desynchronise a real slave.
    Rig rig;
    const uint8_t tx[2] = {0x12, 0x34};
    for (int i = 0; i < 5; ++i) {
        rig.master.select();
        rig.master.transfer(tx, nullptr, sizeof(tx));
        rig.master.deselect();
    }
    CHECK(!rig.slave.clock_while_deselected);
}

TEST(bitbang_parks_the_clock_high_before_asserting_chip_select) {
    Rig rig;
    rig.master.select();
    rig.master.deselect();
    CHECK(!rig.slave.clock_idled_low_at_select);
}

TEST(bitbang_transactions_end_on_a_byte_boundary) {
    Rig rig;
    const uint8_t tx[4] = {1, 2, 3, 4};
    rig.master.select();
    rig.master.transfer(tx, nullptr, sizeof(tx));
    rig.master.deselect();
    CHECK(!rig.slave.ragged_transaction);
}

TEST(bitbang_write_reaches_the_slave_register_file) {
    // Proves bit order and phase are right in the master -> slave direction.
    Rig rig;
    const uint8_t tx[2] = {0x20, 0xA7};  // write 0xA7 to register 0x20
    rig.master.select();
    rig.master.transfer(tx, nullptr, sizeof(tx));
    rig.master.deselect();

    CHECK_EQ(static_cast<int>(rig.slave.reg(0x20)), 0xA7);
}

TEST(bitbang_read_returns_the_slave_register_value) {
    // Proves bit order and phase in the slave -> master direction.
    Rig rig;
    rig.slave.regs[0x30] = 0x5C;

    uint8_t tx[2] = {static_cast<uint8_t>(0x30 | 0x80), 0x00};
    uint8_t rx[2] = {0, 0};
    rig.master.select();
    rig.master.transfer(tx, rx, sizeof(tx));
    rig.master.deselect();

    CHECK_EQ(static_cast<int>(rx[1]), 0x5C);
}

TEST(bitbang_round_trips_every_byte_value) {
    // Exhaustive: any single-bit slip in the shift logic shows up here.
    for (int v = 0; v < 256; ++v) {
        Rig rig;
        rig.slave.regs[0x40] = static_cast<uint8_t>(v);

        uint8_t tx[2] = {static_cast<uint8_t>(0x40 | 0x80), 0x00};
        uint8_t rx[2] = {0, 0};
        rig.master.select();
        rig.master.transfer(tx, rx, sizeof(tx));
        rig.master.deselect();

        CHECK_EQ(static_cast<int>(rx[1]), v);
    }
}

TEST(bitbang_writes_every_byte_value_correctly) {
    for (int v = 0; v < 256; ++v) {
        Rig rig;
        const uint8_t tx[2] = {0x41, static_cast<uint8_t>(v)};
        rig.master.select();
        rig.master.transfer(tx, nullptr, sizeof(tx));
        rig.master.deselect();
        CHECK_EQ(static_cast<int>(rig.slave.reg(0x41)), v);
    }
}

TEST(bitbang_burst_read_auto_increments_the_address) {
    Rig rig;
    for (int i = 0; i < 8; ++i) {
        rig.slave.regs[0x50 + i] = static_cast<uint8_t>(0xE0 + i);
    }

    uint8_t tx[9] = {static_cast<uint8_t>(0x50 | 0x80)};
    uint8_t rx[9] = {0};
    rig.master.select();
    rig.master.transfer(tx, rx, sizeof(tx));
    rig.master.deselect();

    for (int i = 0; i < 8; ++i) {
        CHECK_EQ(static_cast<int>(rx[1 + i]), 0xE0 + i);
    }
}

TEST(bitbang_handles_null_tx_and_rx_buffers) {
    Rig rig;
    rig.master.select();
    rig.master.transfer(nullptr, nullptr, 3);  // must not fault
    rig.master.deselect();
    CHECK_EQ(rig.slave.rising_edges, 24);
}

// ---------------------------------------------------------------------------
// The important one: the real IMU driver, over the real bit-banged master, over
// a slave that only sees the waveform. If this passes, the driver and the
// software SPI agree on the protocol end to end.
// ---------------------------------------------------------------------------

TEST(imu_driver_works_over_the_bitbanged_master) {
    SimulatedIcm42688 slave;
    BitBangSpi spi{slave.sck(), slave.mosi(), slave.miso(), slave.cs()};
    FakeClock clock;
    Icm42688 imu{spi, clock};

    CHECK(imu.init(Icm42688Config{}));
    CHECK(imu.initialised());
    CHECK_EQ(slave.soft_reset_count, 1);
    CHECK(!slave.clock_while_deselected);
    CHECK(!slave.ragged_transaction);
}

TEST(imu_driver_over_bitbang_rejects_a_wrong_part) {
    SimulatedIcm42688 slave;
    slave.set_who_am_i(0x12);
    BitBangSpi spi{slave.sck(), slave.mosi(), slave.miso(), slave.cs()};
    FakeClock clock;
    Icm42688 imu{spi, clock};

    CHECK(!imu.init(Icm42688Config{}));
}

TEST(imu_driver_over_bitbang_reads_scaled_samples) {
    SimulatedIcm42688 slave;
    BitBangSpi spi{slave.sck(), slave.mosi(), slave.miso(), slave.cs()};
    FakeClock clock;
    Icm42688 imu{spi, clock};
    CHECK(imu.init(Icm42688Config{}));

    // Level board, and +1000 dps about x at the 2000 dps setting.
    slave.set_accel_raw(0, 0, -2048);
    slave.set_gyro_raw(16384, 0, 0);

    ImuSample s;
    CHECK(imu.read(s));
    CHECK_NEAR(s.accel.z, -1.0f, 0.001f);
    CHECK_NEAR(s.accel.x, 0.0f, 1e-6);
    CHECK_NEAR(s.gyro.x, 1000.0f * kDegToRad, 0.05f);
    CHECK_NEAR(s.gyro.y, 0.0f, 1e-6);
}

TEST(imu_driver_over_bitbang_handles_negative_values) {
    SimulatedIcm42688 slave;
    BitBangSpi spi{slave.sck(), slave.mosi(), slave.miso(), slave.cs()};
    FakeClock clock;
    Icm42688 imu{spi, clock};
    CHECK(imu.init(Icm42688Config{}));

    slave.set_gyro_raw(-16384, -8192, 32767);
    ImuSample s;
    CHECK(imu.read(s));
    CHECK_NEAR(s.gyro.x, -1000.0f * kDegToRad, 0.05f);
    CHECK_NEAR(s.gyro.y, -500.0f * kDegToRad, 0.05f);
    CHECK(s.gyro.z > 0.0f);
}

TEST(imu_driver_over_bitbang_config_readback_matches) {
    // init() verifies its own configuration writes by reading them back. Over a
    // bit-banged bus that read-back is the main defence against a wiring or
    // timing fault producing plausible but wrong flight data.
    SimulatedIcm42688 slave;
    BitBangSpi spi{slave.sck(), slave.mosi(), slave.miso(), slave.cs()};
    FakeClock clock;
    Icm42688 imu{spi, clock};

    Icm42688Config cfg;
    cfg.gyro_range = GyroRange::Dps1000;
    cfg.accel_range = AccelRange::G8;
    cfg.odr = OutputDataRate::Hz8000;
    CHECK(imu.init(cfg));

    CHECK_EQ(static_cast<int>(slave.reg(icm42688::kRegGyroConfig0)), 0x23);
    CHECK_EQ(static_cast<int>(slave.reg(icm42688::kRegAccelConfig0)), 0x23);
    CHECK_EQ(static_cast<int>(slave.reg(icm42688::kRegPwrMgmt0)),
             static_cast<int>(icm42688::kPwrMgmt0LowNoise));
}

TEST(imu_driver_over_bitbang_sustains_repeated_reads) {
    // A long run at loop rate: the bus must stay in sync indefinitely.
    SimulatedIcm42688 slave;
    BitBangSpi spi{slave.sck(), slave.mosi(), slave.miso(), slave.cs()};
    FakeClock clock;
    Icm42688 imu{spi, clock};
    CHECK(imu.init(Icm42688Config{}));

    slave.set_accel_raw(0, 0, -2048);
    slave.set_gyro_raw(1024, -1024, 512);

    for (int i = 0; i < 2000; ++i) {
        ImuSample s;
        CHECK(imu.read(s));
        CHECK_NEAR(s.accel.z, -1.0f, 0.001f);
        CHECK_NEAR(s.gyro.z, (512.0f / 32768.0f) * 2000.0f * kDegToRad, 0.01f);
    }
    CHECK(!slave.clock_while_deselected);
    CHECK(!slave.ragged_transaction);
}

TEST_MAIN()
