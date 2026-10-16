#include "fc/drivers/bmp390.hpp"

#include <cmath>

namespace fc {
namespace bmp390 {
namespace {

uint16_t u16(const uint8_t* p) {
    return static_cast<uint16_t>(static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8));
}

int16_t i16(const uint8_t* p) { return static_cast<int16_t>(u16(p)); }

int8_t i8(uint8_t v) { return static_cast<int8_t>(v); }

uint32_t u24(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16);
}

}  // namespace

Calibration parse_calibration(const uint8_t r[kCalibBytes]) {
    // Offsets are relative to NVM_PAR_T1 at 0x31.
    Calibration c{};
    c.t1 = static_cast<double>(u16(&r[0])) / std::ldexp(1.0, -8);
    c.t2 = static_cast<double>(u16(&r[2])) / std::ldexp(1.0, 30);
    c.t3 = static_cast<double>(i8(r[4])) / std::ldexp(1.0, 48);
    c.p1 = (static_cast<double>(i16(&r[5])) - std::ldexp(1.0, 14)) / std::ldexp(1.0, 20);
    c.p2 = (static_cast<double>(i16(&r[7])) - std::ldexp(1.0, 14)) / std::ldexp(1.0, 29);
    c.p3 = static_cast<double>(i8(r[9])) / std::ldexp(1.0, 32);
    c.p4 = static_cast<double>(i8(r[10])) / std::ldexp(1.0, 37);
    c.p5 = static_cast<double>(u16(&r[11])) / std::ldexp(1.0, -3);
    c.p6 = static_cast<double>(u16(&r[13])) / std::ldexp(1.0, 6);
    c.p7 = static_cast<double>(i8(r[15])) / std::ldexp(1.0, 8);
    c.p8 = static_cast<double>(i8(r[16])) / std::ldexp(1.0, 15);
    c.p9 = static_cast<double>(i16(&r[17])) / std::ldexp(1.0, 48);
    c.p10 = static_cast<double>(i8(r[19])) / std::ldexp(1.0, 48);
    c.p11 = static_cast<double>(i8(r[20])) / std::ldexp(1.0, 65);
    return c;
}

double compensate_temperature(const Calibration& c, uint32_t raw) {
    const double partial1 = static_cast<double>(raw) - c.t1;
    const double partial2 = partial1 * c.t2;
    return partial2 + partial1 * partial1 * c.t3;
}

double compensate_pressure(const Calibration& c, uint32_t raw, double t) {
    const double u = static_cast<double>(raw);
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double out1 = c.p5 + c.p6 * t + c.p7 * t2 + c.p8 * t3;
    const double out2 = u * (c.p1 + c.p2 * t + c.p3 * t2 + c.p4 * t3);
    const double pd4 = u * u * (c.p9 + c.p10 * t) + u * u * u * c.p11;
    return out1 + out2 + pd4;
}

}  // namespace bmp390

float pressure_to_altitude_m(float pressure_pa, float sea_level_pa) {
    if (!(pressure_pa > 0.0f) || !(sea_level_pa > 0.0f)) {
        return 0.0f;
    }
    return 44330.0f * (1.0f - std::pow(pressure_pa / sea_level_pa, 0.190295f));
}

bool Bmp390::write_checked(uint8_t reg, uint8_t value) {
    if (!bus_.write_register(address_, reg, value)) {
        return false;
    }
    uint8_t back = 0;
    return bus_.read_registers(address_, reg, &back, 1) && back == value;
}

bool Bmp390::init() {
    initialised_ = false;

    // A soft reset that is not acknowledged usually means CSB is still
    // floating (R128 not fitted), or the part is absent.
    if (!bus_.write_register(address_, bmp390::kRegCmd, bmp390::kCmdSoftReset)) {
        return false;
    }
    clock_.delay_us(10000);

    uint8_t id = 0;
    if (!bus_.read_registers(address_, bmp390::kRegChipId, &id, 1) || id != bmp390::kChipId) {
        return false;
    }

    uint8_t raw[bmp390::kCalibBytes] = {};
    if (!bus_.read_registers(address_, bmp390::kRegCalib, raw, sizeof(raw))) {
        return false;
    }
    cal_ = bmp390::parse_calibration(raw);

    // Oversampling, rate and filter are set while the part is asleep, then
    // the power control register starts continuous conversion.
    if (!write_checked(bmp390::kRegOsr, bmp390::kOsrP8T1) ||
        !write_checked(bmp390::kRegOdr, bmp390::kOdr50Hz) ||
        !write_checked(bmp390::kRegConfig, bmp390::kConfigIir3) ||
        !write_checked(bmp390::kRegPwrCtrl, bmp390::kPwrCtrlNormal)) {
        return false;
    }

    // ERR_REG: fatal, command and configuration errors in the low bits.
    uint8_t err = 0;
    if (!bus_.read_registers(address_, bmp390::kRegErr, &err, 1) || (err & 0x07u) != 0u) {
        return false;
    }

    initialised_ = true;
    return true;
}

bool Bmp390::read(BaroSample& out) {
    if (!initialised_) {
        return false;
    }
    uint8_t d[6] = {};
    if (!bus_.read_registers(address_, bmp390::kRegData0, d, sizeof(d))) {
        return false;
    }
    const uint32_t raw_p = bmp390::u24(&d[0]);
    const uint32_t raw_t = bmp390::u24(&d[3]);

    const double t = bmp390::compensate_temperature(cal_, raw_t);
    const double p = bmp390::compensate_pressure(cal_, raw_p, t);
    if (!std::isfinite(t) || !std::isfinite(p)) {
        return false;
    }
    out.temperature_c = static_cast<float>(t);
    out.pressure_pa = static_cast<float>(p);
    return true;
}

}  // namespace fc
