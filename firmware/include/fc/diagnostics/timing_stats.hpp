#pragma once

#include <cstdint>

namespace fc {

// Running statistics for a periodic task, used to measure the control loop on
// the target without external instruments.
//
// The mean is accumulated as a 64-bit sum of microsecond samples rather than an
// incremental float average: at 1 kHz for an hour that is 3.6 million samples,
// and a float running mean would lose resolution long before then.
class TimingStats {
public:
    TimingStats() = default;
    explicit TimingStats(uint32_t budget_us) : budget_us_(budget_us) {}

    void set_budget(uint32_t budget_us) { budget_us_ = budget_us; }
    uint32_t budget() const { return budget_us_; }

    void add(uint32_t sample_us);
    void reset();

    uint32_t count() const { return count_; }
    uint32_t min() const { return count_ == 0 ? 0 : min_us_; }
    uint32_t max() const { return count_ == 0 ? 0 : max_us_; }
    uint64_t total() const { return sum_us_; }

    float mean() const;

    // Samples that exceeded the budget. On a fixed-rate control loop this is
    // the number that matters: a good mean with a handful of overruns still
    // means the loop is missing its deadline.
    uint32_t overruns() const { return overruns_; }

    // Worst overshoot beyond the budget, 0 if none.
    uint32_t worst_overrun() const { return worst_overrun_us_; }

private:
    uint32_t budget_us_{0};
    uint32_t count_{0};
    uint32_t min_us_{0};
    uint32_t max_us_{0};
    uint64_t sum_us_{0};
    uint32_t overruns_{0};
    uint32_t worst_overrun_us_{0};
};

}  // namespace fc
