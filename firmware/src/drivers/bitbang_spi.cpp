#include "fc/drivers/bitbang_spi.hpp"

namespace fc {

void BitBangSpi::select() {
    // Make sure the clock is parked in its idle state before CS falls,
    // otherwise the slave can see a spurious first edge.
    sck_.write(true);
    delay_half_bit();
    cs_.write(false);
    delay_half_bit();
}

void BitBangSpi::deselect() {
    delay_half_bit();
    cs_.write(true);
    // Leave the clock idle high, ready for the next transaction.
    sck_.write(true);
    delay_half_bit();
}

uint8_t BitBangSpi::transfer_byte_internal(uint8_t out) {
    uint8_t in = 0;

    for (int bit = 7; bit >= 0; --bit) {
        // Leading edge: clock falls, master presents the next bit.
        sck_.write(false);
        mosi_.write(((out >> bit) & 0x01u) != 0u);
        delay_half_bit();

        // Trailing edge: clock rises, both ends sample.
        sck_.write(true);
        if (miso_.read()) {
            in = static_cast<uint8_t>(in | (1u << bit));
        }
        delay_half_bit();
    }

    return in;
}

void BitBangSpi::transfer(const uint8_t* tx, uint8_t* rx, std::size_t len) {
    for (std::size_t i = 0; i < len; ++i) {
        const uint8_t out = (tx != nullptr) ? tx[i] : 0x00u;
        const uint8_t in = transfer_byte_internal(out);
        if (rx != nullptr) {
            rx[i] = in;
        }
    }
}

}  // namespace fc
