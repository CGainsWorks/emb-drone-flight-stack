#pragma once

#include <cstddef>
#include <cstdint>

#include "fc/hal/clock.hpp"
#include "fc/hal/spi_bus.hpp"

namespace fc {

struct SpiBenchmarkResult {
    uint32_t bytes_transferred{0};
    uint32_t elapsed_us{0};
    // Effective clock rate including all per-byte and per-transaction overhead,
    // which is what actually matters for the loop budget - not the idealised
    // rate implied by the inter-edge delay.
    uint32_t effective_bitrate_hz{0};
    // Time for one full IMU sample burst (15 bytes), the figure that goes into
    // the control-loop budget.
    uint32_t burst_us{0};
    bool valid{false};
};

// Measures how fast the SPI bus actually runs, by timing a number of
// transactions against the system clock.
//
// This exists so the bit-banged SPI can be characterised on the bench WITHOUT
// an oscilloscope: the MCU times itself with the same microsecond counter the
// control loop uses. A scope is still the authority on signal integrity and
// edge placement, but for "is this fast enough for a 1 kHz loop, and is it
// under the IMU's 24 MHz limit" this is sufficient and repeatable.
//
// Returns valid=false if the clock has too little resolution to measure the
// requested workload, rather than reporting a fabricated rate.
SpiBenchmarkResult benchmark_spi(SpiBus& bus, Clock& clock,
                                 std::size_t transaction_count = 200,
                                 std::size_t bytes_per_transaction = 15);

}  // namespace fc
