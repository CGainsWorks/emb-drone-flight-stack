#include "fc/diagnostics/spi_benchmark.hpp"

namespace fc {

SpiBenchmarkResult benchmark_spi(SpiBus& bus, Clock& clock,
                                 std::size_t transaction_count,
                                 std::size_t bytes_per_transaction) {
    SpiBenchmarkResult result;

    if (transaction_count == 0 || bytes_per_transaction == 0 ||
        bytes_per_transaction > 64) {
        return result;
    }

    uint8_t tx[64] = {0};
    uint8_t rx[64] = {0};
    // A read of a harmless register, repeated. The address bit pattern is
    // irrelevant to timing but keeps the traffic representative.
    tx[0] = 0x80;

    const uint32_t start = clock.micros();
    for (std::size_t i = 0; i < transaction_count; ++i) {
        bus.select();
        bus.transfer(tx, rx, bytes_per_transaction);
        bus.deselect();
    }
    const uint32_t elapsed = elapsed_us(clock.micros(), start);

    result.bytes_transferred =
        static_cast<uint32_t>(transaction_count * bytes_per_transaction);
    result.elapsed_us = elapsed;

    // Below a few hundred microseconds the 1 us counter cannot give a
    // trustworthy figure. Report invalid rather than a made-up number.
    if (elapsed < 100u) {
        return result;
    }

    const uint64_t bits = static_cast<uint64_t>(result.bytes_transferred) * 8ull;
    result.effective_bitrate_hz =
        static_cast<uint32_t>((bits * 1000000ull) / static_cast<uint64_t>(elapsed));
    result.burst_us = elapsed / static_cast<uint32_t>(transaction_count);
    result.valid = true;

    return result;
}

}  // namespace fc
