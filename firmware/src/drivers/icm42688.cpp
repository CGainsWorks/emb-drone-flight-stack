#include "fc/drivers/icm42688.hpp"

namespace fc {
namespace {

constexpr float kDegToRad = 0.017453292519943295f;
constexpr float kFullScaleCounts = 32768.0f;

// Temperature conversion from the datasheet: degC = (raw / 132.48) + 25
constexpr float kTempSensitivity = 132.48f;
constexpr float kTempOffsetC = 25.0f;

int16_t be16(uint8_t hi, uint8_t lo) {
    return static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | static_cast<uint16_t>(lo));
}

}  // namespace

float gyro_range_dps(GyroRange r) {
    switch (r) {
        case GyroRange::Dps2000:
            return 2000.0f;
        case GyroRange::Dps1000:
            return 1000.0f;
        case GyroRange::Dps500:
            return 500.0f;
        case GyroRange::Dps250:
            return 250.0f;
    }
    return 2000.0f;
}

float accel_range_g(AccelRange r) {
    switch (r) {
        case AccelRange::G16:
            return 16.0f;
        case AccelRange::G8:
            return 8.0f;
        case AccelRange::G4:
            return 4.0f;
        case AccelRange::G2:
            return 2.0f;
    }
    return 16.0f;
}

void Icm42688::update_scales() {
    gyro_scale_ = (gyro_range_dps(cfg_.gyro_range) / kFullScaleCounts) * kDegToRad;
    accel_scale_ = accel_range_g(cfg_.accel_range) / kFullScaleCounts;
}

uint8_t Icm42688::read_register(uint8_t reg) {
    uint8_t tx[2] = {static_cast<uint8_t>(reg | icm42688::kReadFlag), 0x00};
    uint8_t rx[2] = {0, 0};
    bus_.select();
    bus_.transfer(tx, rx, 2);
    bus_.deselect();
    return rx[1];
}

void Icm42688::write_register(uint8_t reg, uint8_t value) {
    const uint8_t tx[2] = {static_cast<uint8_t>(reg & 0x7Fu), value};
    bus_.select();
    bus_.transfer(tx, nullptr, 2);
    bus_.deselect();
}

bool Icm42688::init(const Icm42688Config& cfg) {
    initialised_ = false;
    cfg_ = cfg;
    update_scales();

    // Make sure we are addressing register bank 0.
    write_register(icm42688::kRegBankSel, 0x00);

    if (read_register(icm42688::kRegWhoAmI) != icm42688::kWhoAmIValue) {
        return false;
    }

    // Soft reset, then wait for the part to come back (datasheet: 1 ms).
    write_register(icm42688::kRegDeviceConfig, 0x01);
    clock_.delay_us(2000);

    write_register(icm42688::kRegBankSel, 0x00);
    if (read_register(icm42688::kRegWhoAmI) != icm42688::kWhoAmIValue) {
        return false;
    }

    // Gyro and accel to low-noise mode. The datasheet requires no register
    // writes for 200 us after a power-mode change.
    write_register(icm42688::kRegPwrMgmt0, icm42688::kPwrMgmt0LowNoise);
    clock_.delay_us(1000);

    const uint8_t gyro_cfg =
        static_cast<uint8_t>((static_cast<uint8_t>(cfg_.gyro_range) << 5) |
                             static_cast<uint8_t>(cfg_.odr));
    const uint8_t accel_cfg =
        static_cast<uint8_t>((static_cast<uint8_t>(cfg_.accel_range) << 5) |
                             static_cast<uint8_t>(cfg_.odr));

    write_register(icm42688::kRegGyroConfig0, gyro_cfg);
    write_register(icm42688::kRegAccelConfig0, accel_cfg);

    // Verify the configuration actually landed. On a bit-banged bus a wiring or
    // timing fault shows up here rather than as silently wrong flight data.
    if (read_register(icm42688::kRegGyroConfig0) != gyro_cfg) {
        return false;
    }
    if (read_register(icm42688::kRegAccelConfig0) != accel_cfg) {
        return false;
    }

    initialised_ = true;
    return true;
}

bool Icm42688::read(ImuSample& out) {
    if (!initialised_) {
        return false;
    }

    // Burst from TEMP_DATA1 through GYRO_DATA_Z0: 1 address byte + 14 data.
    uint8_t tx[15] = {static_cast<uint8_t>(icm42688::kRegTempData1 | icm42688::kReadFlag)};
    uint8_t rx[15] = {0};
    bus_.select();
    bus_.transfer(tx, rx, sizeof(tx));
    bus_.deselect();

    const int16_t raw_temp = be16(rx[1], rx[2]);
    const int16_t ax = be16(rx[3], rx[4]);
    const int16_t ay = be16(rx[5], rx[6]);
    const int16_t az = be16(rx[7], rx[8]);
    const int16_t gx = be16(rx[9], rx[10]);
    const int16_t gy = be16(rx[11], rx[12]);
    const int16_t gz = be16(rx[13], rx[14]);

    out.temperature_c = (static_cast<float>(raw_temp) / kTempSensitivity) + kTempOffsetC;

    out.accel = Vec3{static_cast<float>(ax) * accel_scale_,
                     static_cast<float>(ay) * accel_scale_,
                     static_cast<float>(az) * accel_scale_};

    out.gyro = Vec3{static_cast<float>(gx) * gyro_scale_,
                    static_cast<float>(gy) * gyro_scale_,
                    static_cast<float>(gz) * gyro_scale_} -
               gyro_bias_;

    return true;
}

}  // namespace fc
