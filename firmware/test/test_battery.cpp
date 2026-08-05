#include "fc/sensors/battery.hpp"
#include "test_framework.hpp"

using namespace fc;

namespace {

BatteryConfig config() {
    BatteryConfig c;
    c.voltage_divider_ratio = 11.0f;
    c.adc_reference_v = 3.3f;
    c.adc_max_counts = 4095;
    c.current_amps_per_volt = 50.0f;
    c.current_offset_v = 0.0f;
    c.voltage_filter_hz = 0.0f;  // disable filtering for deterministic checks
    c.current_filter_hz = 0.0f;
    return c;
}

// ADC counts that correspond to a given pack voltage.
uint16_t counts_for_pack_volts(const BatteryConfig& c, float pack_v) {
    const float pin_v = pack_v / c.voltage_divider_ratio;
    return static_cast<uint16_t>((pin_v / c.adc_reference_v) *
                                     static_cast<float>(c.adc_max_counts) +
                                 0.5f);
}

uint16_t counts_for_amps(const BatteryConfig& c, float amps) {
    const float pin_v = amps / c.current_amps_per_volt + c.current_offset_v;
    return static_cast<uint16_t>((pin_v / c.adc_reference_v) *
                                     static_cast<float>(c.adc_max_counts) +
                                 0.5f);
}

}  // namespace

TEST(battery_converts_counts_to_pack_voltage) {
    BatteryMonitor b{config()};
    // Full scale on the pin = 3.3 V * 11 = 36.3 V at the pack.
    CHECK_NEAR(b.counts_to_pack_volts(4095), 36.3f, 0.05f);
    CHECK_NEAR(b.counts_to_pack_volts(0), 0.0f, 1e-6);
    // Half scale.
    CHECK_NEAR(b.counts_to_pack_volts(2048), 18.15f, 0.05f);
}

TEST(battery_reads_a_4s_pack) {
    const BatteryConfig c = config();
    BatteryMonitor b{c};
    b.update(counts_for_pack_volts(c, 16.8f), 0, 0.01f);
    CHECK_NEAR(b.voltage(), 16.8f, 0.05f);
}

TEST(battery_detects_cell_count) {
    const BatteryConfig c = config();

    // 16.8 V = 4S fully charged.
    BatteryMonitor b4{c};
    b4.update(counts_for_pack_volts(c, 16.8f), 0, 0.01f);
    b4.detect_cell_count();
    CHECK_EQ(static_cast<int>(b4.cell_count()), 4);

    // 12.6 V = 3S fully charged.
    BatteryMonitor b3{c};
    b3.update(counts_for_pack_volts(c, 12.6f), 0, 0.01f);
    b3.detect_cell_count();
    CHECK_EQ(static_cast<int>(b3.cell_count()), 3);

    // 25.2 V = 6S fully charged.
    BatteryMonitor b6{c};
    b6.update(counts_for_pack_volts(c, 25.2f), 0, 0.01f);
    b6.detect_cell_count();
    CHECK_EQ(static_cast<int>(b6.cell_count()), 6);
}

TEST(battery_detects_cell_count_on_a_partly_discharged_pack) {
    const BatteryConfig c = config();
    // 14.8 V nominal 4S (3.7 V/cell).
    BatteryMonitor b{c};
    b.update(counts_for_pack_volts(c, 14.8f), 0, 0.01f);
    b.detect_cell_count();
    CHECK_EQ(static_cast<int>(b.cell_count()), 4);
}

TEST(battery_cell_count_is_not_redetected_under_sag) {
    // The important one. If the cell count were re-derived while flying, a
    // sagging 4S pack would suddenly look like 3S and every voltage threshold
    // in the failsafe would shift.
    const BatteryConfig c = config();
    BatteryMonitor b{c};
    b.update(counts_for_pack_volts(c, 16.8f), 0, 0.01f);
    b.detect_cell_count();
    CHECK_EQ(static_cast<int>(b.cell_count()), 4);

    // Heavy sag down to 13.2 V.
    for (int i = 0; i < 100; ++i) {
        b.update(counts_for_pack_volts(c, 13.2f), 0, 0.01f);
    }
    CHECK_EQ(static_cast<int>(b.cell_count()), 4);
    CHECK_NEAR(b.cell_voltage(), 3.3f, 0.05f);
}

TEST(battery_detect_does_nothing_without_a_sample) {
    BatteryMonitor b{config()};
    b.detect_cell_count();
    CHECK_EQ(static_cast<int>(b.cell_count()), 0);
    CHECK(!b.cells_detected());
}

TEST(battery_detect_does_nothing_with_no_pack_connected) {
    const BatteryConfig c = config();
    BatteryMonitor b{c};
    b.update(0, 0, 0.01f);
    b.detect_cell_count();
    CHECK_EQ(static_cast<int>(b.cell_count()), 0);
}

TEST(battery_reports_cell_voltage) {
    const BatteryConfig c = config();
    BatteryMonitor b{c};
    b.update(counts_for_pack_volts(c, 16.0f), 0, 0.01f);
    b.detect_cell_count();
    CHECK_NEAR(b.cell_voltage(), 4.0f, 0.02f);
}

TEST(battery_cell_voltage_is_zero_before_detection) {
    const BatteryConfig c = config();
    BatteryMonitor b{c};
    b.update(counts_for_pack_volts(c, 16.0f), 0, 0.01f);
    CHECK_NEAR(b.cell_voltage(), 0.0f, 1e-6);
}

TEST(battery_remaining_percent_spans_the_range) {
    const BatteryConfig c = config();

    BatteryMonitor full{c};
    full.update(counts_for_pack_volts(c, 4.20f * 4.0f), 0, 0.01f);
    full.detect_cell_count();
    CHECK_EQ(static_cast<int>(full.remaining_percent()), 100);

    BatteryMonitor empty{c};
    empty.update(counts_for_pack_volts(c, 3.30f * 4.0f), 0, 0.01f);
    empty.detect_cell_count();
    CHECK_EQ(static_cast<int>(empty.remaining_percent()), 0);

    BatteryMonitor mid{c};
    mid.update(counts_for_pack_volts(c, 3.75f * 4.0f), 0, 0.01f);
    mid.detect_cell_count();
    const int pct = static_cast<int>(mid.remaining_percent());
    CHECK(pct > 40);
    CHECK(pct < 60);
}

TEST(battery_remaining_percent_is_clamped) {
    const BatteryConfig c = config();
    BatteryMonitor over{c};
    over.update(counts_for_pack_volts(c, 4.20f * 4.0f), 0, 0.01f);
    over.detect_cell_count();
    // Over-charged reading must not report above 100.
    over.update(counts_for_pack_volts(c, 4.30f * 4.0f), 0, 0.01f);
    CHECK(over.remaining_percent() <= 100);

    BatteryMonitor under{c};
    under.update(counts_for_pack_volts(c, 3.30f * 4.0f), 0, 0.01f);
    under.detect_cell_count();
    under.update(counts_for_pack_volts(c, 2.50f * 4.0f), 0, 0.01f);
    CHECK_EQ(static_cast<int>(under.remaining_percent()), 0);
}

TEST(battery_converts_current) {
    const BatteryConfig c = config();
    BatteryMonitor b{c};
    b.update(0, counts_for_amps(c, 20.0f), 0.01f);
    CHECK_NEAR(b.current(), 20.0f, 0.2f);
}

TEST(battery_current_never_reports_negative) {
    // Sensor offset can put the reading slightly below zero at rest; that must
    // not integrate backwards into the consumed-capacity figure.
    BatteryConfig c = config();
    c.current_offset_v = 0.5f;
    BatteryMonitor b{c};
    b.update(0, 0, 0.01f);
    CHECK(b.current() >= 0.0f);
}

TEST(battery_integrates_consumed_capacity) {
    const BatteryConfig c = config();
    BatteryMonitor b{c};
    // 36 A for 100 s = 1 Ah = 1000 mAh.
    const uint16_t counts = counts_for_amps(c, 36.0f);
    for (int i = 0; i < 10000; ++i) {
        b.update(0, counts, 0.01f);
    }
    CHECK_NEAR(b.used_mah(), 1000.0f, 15.0f);
}

TEST(battery_consumed_capacity_only_grows) {
    const BatteryConfig c = config();
    BatteryMonitor b{c};
    float last = 0.0f;
    for (int i = 0; i < 500; ++i) {
        b.update(counts_for_pack_volts(c, 15.0f), counts_for_amps(c, 10.0f), 0.01f);
        CHECK(b.used_mah() >= last);
        last = b.used_mah();
    }
    CHECK(last > 0.0f);
}

TEST(battery_filter_smooths_a_noisy_reading) {
    BatteryConfig c = config();
    c.voltage_filter_hz = 2.0f;
    BatteryMonitor b{c};

    const uint16_t high = counts_for_pack_volts(c, 17.0f);
    const uint16_t low = counts_for_pack_volts(c, 15.0f);
    b.update(high, 0, 0.001f);
    for (int i = 0; i < 2000; ++i) {
        b.update((i % 2 == 0) ? high : low, 0, 0.001f);
    }
    // Should settle near the mean rather than tracking the alternation.
    CHECK_NEAR(b.voltage(), 16.0f, 0.4f);
}

TEST(battery_handles_zero_dt) {
    BatteryMonitor b{config()};
    b.update(2000, 500, 0.0f);
    CHECK(std::isfinite(b.voltage()));
    CHECK(std::isfinite(b.current()));
    // No time passed, so no capacity may be consumed.
    CHECK_NEAR(b.used_mah(), 0.0f, 1e-9);
}

TEST(battery_reset_clears_everything) {
    const BatteryConfig c = config();
    BatteryMonitor b{c};
    for (int i = 0; i < 100; ++i) {
        b.update(counts_for_pack_volts(c, 16.0f), counts_for_amps(c, 10.0f), 0.01f);
    }
    b.detect_cell_count();
    CHECK(b.cells_detected());

    b.reset();
    CHECK(!b.cells_detected());
    CHECK_NEAR(b.used_mah(), 0.0f, 1e-9);
    CHECK_NEAR(b.voltage(), 0.0f, 1e-9);
}

TEST(battery_survives_a_zero_max_counts_config) {
    BatteryConfig c = config();
    c.adc_max_counts = 0;
    BatteryMonitor b{c};
    b.update(1234, 1234, 0.01f);
    CHECK(std::isfinite(b.voltage()));
    CHECK_NEAR(b.voltage(), 0.0f, 1e-9);
}

TEST_MAIN()
