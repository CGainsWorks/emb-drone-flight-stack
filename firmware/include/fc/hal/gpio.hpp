#pragma once

namespace fc {

// A single digital pin. Deliberately tiny: the whole point is that the
// bit-banged SPI master above it can be driven by simulated pins in a host
// test, so the generated waveform is verified without an oscilloscope.
class GpioPin {
public:
    virtual ~GpioPin() = default;

    virtual void write(bool high) = 0;
    virtual bool read() const = 0;

    void set() { write(true); }
    void clear() { write(false); }
};

}  // namespace fc
