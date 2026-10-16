#pragma once

#include <cstddef>
#include <cstdint>

#include "fc/hal/clock.hpp"
#include "fc/hal/i2c_bus.hpp"

namespace fc {

// Bosch BMP390 barometer, board reference IC9 on FCSTMV1.05.
//
// Bus: I2C1 at 7-bit address 0x77 (SDO strapped high by R126). Its CSB strap
// R128 is not fitted, so on an unmodified board the interface mode is
// undefined and the part will not answer until CSB is tied high at J152.1. The
// driver reports that as a failed init rather than guessing.
//
// Compensation follows the floating-point formulas in the BMP390 datasheet:
// 21 bytes of trimming coefficients are read from NVM once at init and
// applied to every raw sample.
namespace bmp390 {

inline constexpr uint8_t kAddress = 0x77;

inline constexpr uint8_t kRegChipId = 0x00;
inline constexpr uint8_t kRegErr = 0x02;
inline constexpr uint8_t kRegStatus = 0x03;
inline constexpr uint8_t kRegData0 = 0x04;  // pressure xlsb .. temperature msb
inline constexpr uint8_t kRegPwrCtrl = 0x1B;
inline constexpr uint8_t kRegOsr = 0x1C;
inline constexpr uint8_t kRegOdr = 0x1D;
inline constexpr uint8_t kRegConfig = 0x1F;
inline constexpr uint8_t kRegCalib = 0x31;
inline constexpr uint8_t kRegCmd = 0x7E;

inline constexpr uint8_t kChipId = 0x60;
inline constexpr uint8_t kCmdSoftReset = 0xB6;
inline constexpr std::size_t kCalibBytes = 21;

// PWR_CTRL: pressure and temperature enabled, normal (continuous) mode.
inline constexpr uint8_t kPwrCtrlNormal = 0x33;
// OSR: pressure x8, temperature x1. ODR: 50 Hz. CONFIG: IIR coefficient 3.
inline constexpr uint8_t kOsrP8T1 = 0x03;
inline constexpr uint8_t kOdr50Hz = 0x02;
inline constexpr uint8_t kConfigIir3 = 0x04;

// Floating-point trimming coefficients, already scaled as the datasheet
// specifies. Public so the simulated device can share the arithmetic.
struct Calibration {
    double t1, t2, t3;
    double p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11;
};

// Scale the raw NVM bytes (from register 0x31 onwards).
Calibration parse_calibration(const uint8_t raw[kCalibBytes]);

// Datasheet compensation. Temperature first: pressure needs its result.
double compensate_temperature(const Calibration& c, uint32_t raw_temperature);
double compensate_pressure(const Calibration& c, uint32_t raw_pressure, double temperature_c);

}  // namespace bmp390

struct BaroSample {
    float pressure_pa{0.0f};
    float temperature_c{0.0f};
};

// International Standard Atmosphere altitude for a pressure, relative to a
// reference sea-level pressure. Good for relative height; absolute altitude
// needs the local QNH.
float pressure_to_altitude_m(float pressure_pa, float sea_level_pa = 101325.0f);

class Bmp390 {
public:
    Bmp390(I2cBus& bus, Clock& clock, uint8_t address = bmp390::kAddress)
        : bus_(bus), clock_(clock), address_(address) {}

    // Soft reset, identify, read trimming data, configure and start continuous
    // conversion. Returns false if the part does not answer, identifies
    // wrongly, or the configuration does not read back.
    bool init();

    // Read and compensate the latest conversion. False if not initialised or
    // the bus transaction failed.
    bool read(BaroSample& out);

    bool initialised() const { return initialised_; }
    const bmp390::Calibration& calibration() const { return cal_; }

private:
    I2cBus& bus_;
    Clock& clock_;
    uint8_t address_;
    bmp390::Calibration cal_{};
    bool initialised_{false};

    bool write_checked(uint8_t reg, uint8_t value);
};

}  // namespace fc
