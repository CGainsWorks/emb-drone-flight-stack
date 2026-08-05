#pragma once

#include "fc/hal/gpio.hpp"
#include "fc/hal/spi_bus.hpp"

namespace fc {

// Software SPI master, SPI mode 3 (CPOL = 1, CPHA = 1).
//
// This exists because of a hard constraint of board FCSTMV1.05: the IMU's data
// lines land on STM32F405 pins whose fixed alternate functions are the wrong
// way round for the hardware SPI1 peripheral (PA6 is MISO-only, PA7 is
// MOSI-only, and the F405 has no swap bit). Bit-banging over plain GPIO has no
// such restriction, because software chooses which pin is an output.
//
// Mode 3 timing, which the ICM-42688-P supports:
//   * SCK idles HIGH
//   * the master drives MOSI on the falling (leading) edge
//   * both ends sample on the rising (trailing) edge
//   * data is MSB first
//
// Everything here is pure pin toggling, so a host test can substitute simulated
// pins and decode the resulting waveform to check the protocol exactly.
class BitBangSpi : public SpiBus {
public:
    BitBangSpi(GpioPin& sck, GpioPin& mosi, GpioPin& miso, GpioPin& cs)
        : sck_(sck), mosi_(mosi), miso_(miso), cs_(cs) {
        // Idle state: clock high (CPOL = 1), chip deselected (CS active low).
        sck_.write(true);
        cs_.write(true);
    }

    void select() override;
    void deselect() override;
    void transfer(const uint8_t* tx, uint8_t* rx, std::size_t len) override;

protected:
    // Half a bit period. The default does nothing, which is correct for host
    // tests. The STM32 implementation overrides this with a short NOP delay to
    // hold the bus inside the IMU's maximum clock rate.
    virtual void delay_half_bit() {}

private:
    GpioPin& sck_;
    GpioPin& mosi_;
    GpioPin& miso_;
    GpioPin& cs_;

    uint8_t transfer_byte_internal(uint8_t out);
};

}  // namespace fc
