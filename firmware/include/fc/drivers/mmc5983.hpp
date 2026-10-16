#pragma once

#include <cstdint>

#include "fc/hal/clock.hpp"
#include "fc/hal/i2c_bus.hpp"
#include "fc/math/vec3.hpp"

namespace fc {

// MEMSIC MMC5983MA 3-axis magnetometer, board reference IC10 on FCSTMV1.05.
//
// On this board revision the part sits on /I2C_SCL_2 and /I2C_SDA_2 inside the
// GPS power island, which does not reach the F405, and its bus pull-ups are not
// fitted (docs/PINOUT.md). The driver is written against the I2cBus interface
// so it is ready for a revision that wires it, or for an external compass; on
// the current board yaw remains gyro-only.
//
// Readings are taken single-shot in 18-bit mode. Bridge offset (including its
// temperature drift) is removed with the part's SET/RESET straps: the sensed
// field changes sign between the two states and the offset does not.
namespace mmc5983 {

inline constexpr uint8_t kAddress = 0x30;

inline constexpr uint8_t kRegXout0 = 0x00;  // X[17:10], then X[9:2] .. XYZout2
inline constexpr uint8_t kRegStatus = 0x08;
inline constexpr uint8_t kRegControl0 = 0x09;
inline constexpr uint8_t kRegControl1 = 0x0A;
inline constexpr uint8_t kRegProductId = 0x2F;

inline constexpr uint8_t kProductId = 0x30;

inline constexpr uint8_t kStatusMeasDone = 0x01;
inline constexpr uint8_t kCtrl0TakeMeasurement = 0x01;
inline constexpr uint8_t kCtrl0Set = 0x08;
inline constexpr uint8_t kCtrl0Reset = 0x10;
inline constexpr uint8_t kCtrl1SoftReset = 0x80;
inline constexpr uint8_t kCtrl1Bandwidth100Hz = 0x00;

inline constexpr int32_t kNullCounts = 131072;  // 18-bit mid-scale
inline constexpr float kCountsPerGauss = 16384.0f;

// Unpack the 18-bit, offset-binary samples from registers 0x00..0x06.
void decode(const uint8_t raw[7], int32_t out[3]);

}  // namespace mmc5983

// Heading from a body-frame magnetic field and the current roll and pitch
// (NED body frame, radians). Returns radians from magnetic north, -pi..pi,
// positive clockwise seen from above.
float tilt_compensated_heading(const Vec3& mag_body, float roll, float pitch);

class Mmc5983 {
public:
    Mmc5983(I2cBus& bus, Clock& clock, uint8_t address = mmc5983::kAddress)
        : bus_(bus), clock_(clock), address_(address) {}

    // Software reset, identify, set bandwidth, then measure the bridge offset
    // with a SET/RESET pair. False if the part does not answer correctly.
    bool init();

    // Re-measure the bridge offset. Worth repeating as the board warms up.
    bool calibrate_offset();

    // Field in gauss, body frame, offset removed.
    bool read(Vec3& gauss);

    bool initialised() const { return initialised_; }
    const int32_t* offset_counts() const { return offset_; }

private:
    I2cBus& bus_;
    Clock& clock_;
    uint8_t address_;
    int32_t offset_[3]{};
    bool initialised_{false};

    // One single-shot conversion, counts relative to mid-scale.
    bool measure(int32_t out[3]);
};

}  // namespace fc
