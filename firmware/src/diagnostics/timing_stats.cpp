#include "fc/diagnostics/timing_stats.hpp"

namespace fc {

void TimingStats::reset() {
    count_ = 0;
    min_us_ = 0;
    max_us_ = 0;
    sum_us_ = 0;
    overruns_ = 0;
    worst_overrun_us_ = 0;
}

void TimingStats::add(uint32_t sample_us) {
    if (count_ == 0) {
        min_us_ = sample_us;
        max_us_ = sample_us;
    } else {
        if (sample_us < min_us_) {
            min_us_ = sample_us;
        }
        if (sample_us > max_us_) {
            max_us_ = sample_us;
        }
    }

    sum_us_ += sample_us;
    ++count_;

    if (budget_us_ > 0 && sample_us > budget_us_) {
        ++overruns_;
        const uint32_t over = sample_us - budget_us_;
        if (over > worst_overrun_us_) {
            worst_overrun_us_ = over;
        }
    }
}

float TimingStats::mean() const {
    if (count_ == 0) {
        return 0.0f;
    }
    return static_cast<float>(sum_us_) / static_cast<float>(count_);
}

}  // namespace fc
