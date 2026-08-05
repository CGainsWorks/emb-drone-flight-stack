#pragma once

#include <cstdint>

#include "fc/hal/clock.hpp"
#include "fc/hal/spi_bus.hpp"
#include "fc/math/vec3.hpp"

namespace fc {

// TDK InvenSense ICM-42688-P, 6-axis. Board reference U7 on FCSTMV1.05.
//
// Bus: SPI, chip select on /GYRO_CS. See hal/spi_bus.hpp for why this must be
// driven by a bit-banged GPIO master on this board revision rather than the
// F405 hardware SPI1 peripheral.
//
// Note also that the part's INT1 pin (/GYRO_EXTI1) is not routed to the F405 on
// this board, so there is no data-ready interrupt available. Sampling must be
// paced by a timer and the driver polled.
namespace icm42688 {

// Bank 0 registers used here.
inline constexpr uint8_t kRegDeviceConfig = 0x11;
inline constexpr uint8_t kRegTempData1 = 0x1D;
inline constexpr uint8_t kRegAccelDataX1 = 0x1F;
inline constexpr uint8_t kRegPwrMgmt0 = 0x4E;
inline constexpr uint8_t kRegGyroConfig0 = 0x4F;
inline constexpr uint8_t kRegAccelConfig0 = 0x50;
inline constexpr uint8_t kRegWhoAmI = 0x75;
inline constexpr uint8_t kRegBankSel = 0x76;

inline constexpr uint8_t kWhoAmIValue = 0x47;

// Read transactions set the MSB of the address byte.
inline constexpr uint8_t kReadFlag = 0x80;

// PWR_MGMT0: gyro and accel both in low-noise mode, temperature enabled.
inline constexpr uint8_t kPwrMgmt0LowNoise = 0x0F;

}  // namespace icm42688

enum class GyroRange : uint8_t {
    Dps2000 = 0,
    Dps1000 = 1,
    Dps500 = 2,
    Dps250 = 3,
};

enum class AccelRange : uint8_t {
    G16 = 0,
    G8 = 1,
    G4 = 2,
    G2 = 3,
};

// Output data rate selector, as written to the low nibble of *_CONFIG0.
enum class OutputDataRate : uint8_t {
    Hz8000 = 0x03,
    Hz4000 = 0x04,
    Hz2000 = 0x05,
    Hz1000 = 0x06,
};

struct Icm42688Config {
    GyroRange gyro_range{GyroRange::Dps2000};
    AccelRange accel_range{AccelRange::G16};
    OutputDataRate odr{OutputDataRate::Hz1000};
};

struct ImuSample {
    Vec3 gyro{};   // rad/s, body frame
    Vec3 accel{};  // g, body frame
    float temperature_c{0.0f};
};

// Full-scale in engineering units, used for the raw -> physical conversion.
float gyro_range_dps(GyroRange r);
float accel_range_g(AccelRange r);

class Icm42688 {
public:
    Icm42688(SpiBus& bus, Clock& clock) : bus_(bus), clock_(clock) {}

    // Probes WHO_AM_I, soft-resets and configures the part.
    // Returns false if the device does not respond correctly, in which case the
    // caller must not arm.
    bool init(const Icm42688Config& cfg);

    // Reads temperature, accel and gyro in one burst. Returns false if the
    // driver was never successfully initialised.
    bool read(ImuSample& out);

    uint8_t read_register(uint8_t reg);
    void write_register(uint8_t reg, uint8_t value);

    bool initialised() const { return initialised_; }
    const Icm42688Config& config() const { return cfg_; }

    // Gyro readings have a constant offset that must be measured with the
    // airframe stationary and subtracted. Applied inside read().
    void set_gyro_bias(const Vec3& bias) { gyro_bias_ = bias; }
    const Vec3& gyro_bias() const { return gyro_bias_; }

private:
    SpiBus& bus_;
    Clock& clock_;
    Icm42688Config cfg_{};
    Vec3 gyro_bias_{};
    bool initialised_{false};

    float gyro_scale_{0.0f};   // raw LSB -> rad/s
    float accel_scale_{0.0f};  // raw LSB -> g

    void update_scales();
};

}  // namespace fc
