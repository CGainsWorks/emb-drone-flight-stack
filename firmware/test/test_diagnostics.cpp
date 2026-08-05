#include "fc/diagnostics/spi_benchmark.hpp"
#include "fc/diagnostics/timing_stats.hpp"
#include "mocks/mock_spi.hpp"
#include "test_framework.hpp"

using namespace fc;
using fc::testing_mocks::FakeClock;

namespace {

// An SPI bus that consumes a fixed amount of simulated time per byte, so the
// benchmark's arithmetic can be checked against a known answer.
class TimedSpiBus : public SpiBus {
public:
    TimedSpiBus(FakeClock& clock, uint32_t us_per_byte, uint32_t us_per_transaction)
        : clock_(clock), us_per_byte_(us_per_byte), us_per_txn_(us_per_transaction) {}

    void select() override { clock_.advance_us(us_per_txn_); }
    void deselect() override {}
    void transfer(const uint8_t*, uint8_t*, std::size_t len) override {
        clock_.advance_us(static_cast<uint32_t>(len) * us_per_byte_);
    }

private:
    FakeClock& clock_;
    uint32_t us_per_byte_;
    uint32_t us_per_txn_;
};

}  // namespace

// --- TimingStats --------------------------------------------------------

TEST(timing_stats_starts_empty) {
    TimingStats s;
    CHECK_EQ(static_cast<int>(s.count()), 0);
    CHECK_EQ(static_cast<int>(s.min()), 0);
    CHECK_EQ(static_cast<int>(s.max()), 0);
    CHECK_NEAR(s.mean(), 0.0f, 1e-9);
}

TEST(timing_stats_tracks_min_max_mean) {
    TimingStats s;
    s.add(100);
    s.add(300);
    s.add(200);
    CHECK_EQ(static_cast<int>(s.count()), 3);
    CHECK_EQ(static_cast<int>(s.min()), 100);
    CHECK_EQ(static_cast<int>(s.max()), 300);
    CHECK_NEAR(s.mean(), 200.0f, 1e-4);
}

TEST(timing_stats_first_sample_sets_both_bounds) {
    // A naive implementation initialising min to 0 would report min=0 forever.
    TimingStats s;
    s.add(450);
    CHECK_EQ(static_cast<int>(s.min()), 450);
    CHECK_EQ(static_cast<int>(s.max()), 450);
}

TEST(timing_stats_counts_budget_overruns) {
    TimingStats s{1000};
    s.add(900);
    s.add(1000);  // exactly on budget is not an overrun
    s.add(1001);
    s.add(1500);
    CHECK_EQ(static_cast<int>(s.overruns()), 2);
    CHECK_EQ(static_cast<int>(s.worst_overrun()), 500);
}

TEST(timing_stats_no_budget_means_no_overruns) {
    TimingStats s;
    for (int i = 0; i < 100; ++i) {
        s.add(100000);
    }
    CHECK_EQ(static_cast<int>(s.overruns()), 0);
}

TEST(timing_stats_mean_stays_exact_over_a_long_run) {
    // An hour at 1 kHz. A float running mean would drift; the 64-bit sum
    // must not.
    TimingStats s;
    for (uint32_t i = 0; i < 3600000u; ++i) {
        s.add(800);
    }
    CHECK_EQ(static_cast<int>(s.count()), 3600000);
    CHECK_NEAR(s.mean(), 800.0f, 0.01f);
    CHECK_EQ(static_cast<int>(s.total() == 2880000000ull ? 1 : 0), 1);
}

TEST(timing_stats_handles_large_samples_without_overflow) {
    TimingStats s;
    for (int i = 0; i < 1000; ++i) {
        s.add(4000000u);  // 4 s samples
    }
    // 1000 * 4e6 = 4e9, which overflows 32 bits; the 64-bit accumulator must not.
    CHECK_NEAR(s.mean(), 4000000.0f, 1.0f);
}

TEST(timing_stats_reset_clears_everything) {
    TimingStats s{1000};
    s.add(5000);
    s.reset();
    CHECK_EQ(static_cast<int>(s.count()), 0);
    CHECK_EQ(static_cast<int>(s.overruns()), 0);
    CHECK_EQ(static_cast<int>(s.worst_overrun()), 0);
    CHECK_EQ(static_cast<int>(s.min()), 0);
}

// --- SPI benchmark ------------------------------------------------------

TEST(spi_benchmark_computes_the_expected_bitrate) {
    FakeClock clock;
    // 1 us per byte, no per-transaction overhead -> 8 bits per us = 8 Mbit/s.
    TimedSpiBus bus{clock, 1, 0};

    const SpiBenchmarkResult r = benchmark_spi(bus, clock, 200, 15);
    CHECK(r.valid);
    CHECK_EQ(static_cast<int>(r.bytes_transferred), 3000);
    CHECK_EQ(static_cast<int>(r.elapsed_us), 3000);
    CHECK_EQ(static_cast<int>(r.effective_bitrate_hz), 8000000);
    CHECK_EQ(static_cast<int>(r.burst_us), 15);
}

TEST(spi_benchmark_includes_per_transaction_overhead) {
    FakeClock clock;
    // 1 us/byte plus 5 us of chip-select overhead per transaction.
    TimedSpiBus bus{clock, 1, 5};

    const SpiBenchmarkResult r = benchmark_spi(bus, clock, 200, 15);
    CHECK(r.valid);
    // 200 * (15 + 5) = 4000 us
    CHECK_EQ(static_cast<int>(r.elapsed_us), 4000);
    CHECK_EQ(static_cast<int>(r.burst_us), 20);
    // Effective rate is lower than the raw byte rate, which is the point:
    // the loop budget must account for the overhead too.
    CHECK(r.effective_bitrate_hz < 8000000u);
    CHECK_EQ(static_cast<int>(r.effective_bitrate_hz), 6000000);
}

TEST(spi_benchmark_reports_invalid_when_too_fast_to_measure) {
    // A bus that consumes no simulated time cannot be characterised by a 1 us
    // counter. It must say so rather than report an infinite bitrate.
    FakeClock clock;
    TimedSpiBus bus{clock, 0, 0};

    const SpiBenchmarkResult r = benchmark_spi(bus, clock, 10, 15);
    CHECK(!r.valid);
    CHECK_EQ(static_cast<int>(r.effective_bitrate_hz), 0);
}

TEST(spi_benchmark_rejects_nonsense_parameters) {
    FakeClock clock;
    TimedSpiBus bus{clock, 1, 0};

    CHECK(!benchmark_spi(bus, clock, 0, 15).valid);
    CHECK(!benchmark_spi(bus, clock, 100, 0).valid);
    CHECK(!benchmark_spi(bus, clock, 100, 65).valid);  // exceeds the buffer
}

TEST(spi_benchmark_frames_every_transaction_with_chip_select) {
    // The benchmark must exercise the same CS discipline as the real driver,
    // or the number it produces will not reflect real traffic.
    testing_mocks::MockIcm42688 dev;
    FakeClock clock;

    benchmark_spi(dev, clock, 50, 15);

    CHECK_EQ(dev.select_count, 50);
    CHECK_EQ(dev.deselect_count, 50);
    CHECK_EQ(dev.transfers_without_cs, 0);
}

TEST(spi_benchmark_is_correct_across_a_timer_wrap) {
    FakeClock clock;
    clock.set_us(0xFFFFF000u);  // wraps part-way through the run
    TimedSpiBus bus{clock, 1, 0};

    const SpiBenchmarkResult r = benchmark_spi(bus, clock, 200, 15);
    CHECK(r.valid);
    CHECK_EQ(static_cast<int>(r.elapsed_us), 3000);
    CHECK_EQ(static_cast<int>(r.effective_bitrate_hz), 8000000);
}

TEST_MAIN()
